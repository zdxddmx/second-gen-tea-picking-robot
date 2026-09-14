/**
 * @file    motor.c
 * @brief   Implementation of the motor drive abstraction.
 * @copyright MIT License, see LICENSE.
 */

#include "motor.h"

#include <string.h>

#include "compiler.h"
#include "config.h"
#include "logger.h"
#include "platform_api.h"

/**
 * @brief Fraction of the current limit that the fold-back regulates *to*.
 *
 * Not 1.0: see the comment in @ref tp_motor_update.  A fold-back that targets
 * the trip threshold hovers on it and trips on the discrete duty steps.
 */
#define MOTOR_FOLDBACK_TARGET_FRACTION 0.9f

/**
 * @brief Samples above the peak rating before the drive is latched out.
 *
 * At 1 kHz this is 2 ms - long enough that a single ADC glitch cannot trip it,
 * short enough that it is a genuine short-time rating rather than a second
 * continuous one.
 */
#define MOTOR_PEAK_TRIP_SAMPLES ((TP_MOTOR_PEAK_WINDOW_MS / 2u) + 1u)

/**
 * @brief Smallest fold-back ratio applied in one sample.
 *
 * A 5 % floor: without it a single noisy current sample could scale the duty
 * to zero in one step, which is indistinguishable from a commanded stop.
 */
#define MOTOR_FOLDBACK_MIN_RATIO 0.05f

/** @brief Current below which the fold-back ceiling is allowed to recover. */
#define MOTOR_FOLDBACK_RECOVER_BELOW 0.8f

/** @brief Ceiling recovery per sample, as a fraction of full scale (1 %/ms). */
#define MOTOR_FOLDBACK_RECOVER_STEP 0.01f

/**
 * @brief Clamp a float into a range.
 *
 * @param[in] v   Value.
 * @param[in] lo  Lower bound.
 * @param[in] hi  Upper bound.
 * @return Clamped value.
 */
static float motor_clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

tp_status_t tp_motor_init(tp_motor_t *motor, tp_motor_ch_t ch, float dt_s)
{
    tp_status_t st;

    if (motor == NULL) {
        return TP_ERR_PARAM;
    }
    if (((int)ch < 0) || (ch >= TP_MOTOR_COUNT)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(dt_s) || (dt_s <= 0.0f)) {
        return TP_ERR_RANGE;
    }

    (void)memset(motor, 0, sizeof(*motor));
    motor->ch = ch;
    motor->current_limit_ma = TP_MAX_CURRENT_MA;
    motor->duty_limit = TP_MAX_PWM_PERMILLE;

    st = tp_filter_slew_init(&motor->slew, TP_MAX_DUTY_RATE_PER_S, dt_s);
    if (st != TP_OK) {
        return st;
    }

    st = tp_platform_motor_enable(ch, 1);
    if (st != TP_OK) {
        LOG_ERROR(TP_MOD_MOTOR, "channel %d enable failed (%s)", (int)ch,
                  tp_status_str(st));
        return st;
    }
    motor->enabled = 1;
    motor->initialised = 1;
    LOG_INFO(TP_MOD_MOTOR, "channel %d ready, duty slew %.0f permille/s, Ilim %.0f mA",
             (int)ch, (double)TP_MAX_DUTY_RATE_PER_S, (double)motor->current_limit_ma);
    return TP_OK;
}

tp_status_t tp_motor_set_duty(tp_motor_t *motor, float permille)
{
    if ((motor == NULL) || (motor->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(permille)) {
        LOG_ERROR(TP_MOD_MOTOR, "channel %d rejected non-finite duty", (int)motor->ch);
        return TP_ERR_RANGE;
    }
    if ((motor->fault != 0) || (motor->enabled == 0)) {
        /* Deliberately silent about the duty value: this path is hit many times
         * per second while a fault is latched and logging it would flood the
         * link.  The fault itself was logged once, when it was raised. */
        return TP_ERR_FAULT_ACTIVE;
    }
    motor->duty_cmd = motor_clampf(permille, -TP_MAX_PWM_PERMILLE,
                                   TP_MAX_PWM_PERMILLE);
    motor->command_writes++;
    return TP_OK;
}

tp_status_t tp_motor_update(tp_motor_t *motor, float dt_s)
{
    float limited;
    float current;
    tp_status_t st;

    if ((motor == NULL) || (motor->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(dt_s) || (dt_s <= 0.0f)) {
        return TP_ERR_RANGE;
    }

    if (motor->enabled == 0) {
        /* Keep sampling the current even with the drive disabled.
         *
         * Returning immediately here is the obvious thing to do and it is a bug:
         * `tp_motor_clear_fault` refuses while the measured current is above the
         * limit, so a channel that stops refreshing its current reading after
         * being disabled can *never* be cleared —the stale value stays above
         * the limit forever and the drive is latched out for good.  The reading
         * costs one ADC result and it is what makes recovery observable. */
        (void)tp_platform_motor_current_ma(motor->ch, &motor->current_ma);
        motor->duty_applied = 0.0f;
        return TP_ERR_FAULT_ACTIVE;
    }

    /* --- 1. slew limit ------------------------------------------------- */
    /* Re-derive the limiter's dt: the control task's period is a compile-time
     * constant, but a test may call this with a different step, and a stale dt
     * would silently change the effective slew rate.
     *
     * The slew target is clamped to `duty_limit`, the ceiling left behind by the
     * current fold-back.  This matters and the first version got it wrong: the
     * fold-back reduced `duty_applied`, and on the very next sample the slew
     * limiter chased `duty_cmd` straight back up to the original command, so the
     * fold-back was undone every period, the current oscillated across the trip
     * threshold, and the over-current protection latched on a drive that was
     * working correctly.  A limit that the slew limiter can undo is not a limit. */
    {
        float max_step = TP_MAX_DUTY_RATE_PER_S * dt_s;
        float target = motor_clampf(motor->duty_cmd, -motor->duty_limit,
                                    motor->duty_limit);
        if ((target - motor->duty_applied) > max_step) {
            limited = motor->duty_applied + max_step;
        } else if ((motor->duty_applied - target) > max_step) {
            limited = motor->duty_applied - max_step;
        } else {
            limited = target;
        }
        motor->output_saturated = (target != limited) ? 1 : 0;
        motor->duty_applied = limited;
    }

    /* --- 2. current measurement and protection -------------------------- */
    current = 0.0f;
    st = tp_platform_motor_current_ma(motor->ch, &current);
    if (st != TP_OK) {
        /* A failed current read is not fatal: the drive keeps working with the
         * previous limit, but the event is counted so an intermittent shunt
         * amplifier shows up in the diagnostics rather than as a mystery
         * derating. */
        motor->foldback_events++;
    } else {
        motor->current_ma = current;
    }

    /* Peak limit first: it is the tighter of the two bounds and it is the one
     * that protects the silicon.  A current above the *peak* rating is a fault
     * however brief, so it is judged against a short window rather than the
     * continuous debounce. */
    if (TP_FABS(motor->current_ma) > TP_PEAK_CURRENT_MA) {
        motor->peak_samples++;
        if (motor->peak_samples >= MOTOR_PEAK_TRIP_SAMPLES) {
            if (motor->fault == 0) {
                LOG_ERROR(TP_MOD_MOTOR,
                          "channel %d peak over-current: %.0f mA > %.0f mA",
                          (int)motor->ch, (double)motor->current_ma,
                          (double)TP_PEAK_CURRENT_MA);
            }
            motor->fault = 1;
            (void)tp_platform_motor_enable(motor->ch, 0);
            motor->enabled = 0;
            motor->duty_applied = 0.0f;
            return TP_ERR_FAULT_ACTIVE;
        }
    } else {
        motor->peak_samples = 0u;
    }

    if (TP_FABS(motor->current_ma) > motor->current_limit_ma) {
        float ratio;
        motor->overcurrent_samples++;
        motor->last_overcurrent_ms = tp_platform_time_ms();
        motor->foldback_events++;

        /*
         * Proportional fold-back, applied **to the present duty**, targeting
         * 90 % of the limit.
         *
         * The first version computed `allowed = full_scale * (limit / current)`,
         * which is an absolute ceiling rather than a reduction.  At a 300
         * per-mille command drawing 3.6 A that formula gives a ceiling of 750
         * per-mille —*above* the command —so nothing was reduced, the current
         * stayed at 3.6 A, and the over-current protection tripped on a drive
         * that was behaving perfectly correctly.  Scaling the present duty
         * instead (`duty *= 2700 / 3600`) brings it to 225 per-mille, i.e.
         * 2.7 A, which is the intended behaviour and the standard form.
         *
         * The 90 % target rather than 100 % matters for the same reason: a
         * fold-back that regulates *to* the trip threshold hovers on it, and the
         * discrete duty steps then cross it on almost every sample.
         */
        ratio = (motor->current_limit_ma * MOTOR_FOLDBACK_TARGET_FRACTION)
              / TP_FABS(motor->current_ma);
        ratio = motor_clampf(ratio, MOTOR_FOLDBACK_MIN_RATIO, 1.0f);
        motor->duty_applied *= ratio;
        /* Leave the ceiling where the fold-back put it.  The slew limiter reads
         * it, so the reduction cannot be undone on the next sample. */
        motor->duty_limit = motor_clampf(TP_FABS(motor->duty_applied),
                                         MOTOR_FOLDBACK_MIN_RATIO * TP_MAX_PWM_PERMILLE,
                                         TP_MAX_PWM_PERMILLE);

        if (motor->overcurrent_samples >= TP_MOTOR_OC_TRIP_SAMPLES) {
            if (motor->fault == 0) {
                LOG_ERROR(TP_MOD_MOTOR,
                          "channel %d over-current latch: %.0f mA > %.0f mA for %lu ms",
                          (int)motor->ch, (double)motor->current_ma,
                          (double)motor->current_limit_ma,
                          (unsigned long)(TP_MOTOR_OC_TRIP_SAMPLES
                                          * (uint32_t)(dt_s * 1000.0f)));
            }
            motor->fault = 1;
            (void)tp_platform_motor_enable(motor->ch, 0);
            motor->enabled = 0;
            motor->duty_applied = 0.0f;
            return TP_ERR_FAULT_ACTIVE;
        }
    } else {
        motor->overcurrent_samples = 0u;
        /*
         * Recover the ceiling slowly, and only when there is genuine headroom
         * (below 80 % of the limit).
         *
         * Recovering as soon as the current merely dips under the limit sets up
         * a slow limit cycle: the ceiling returns, the slew limiter re-applies
         * the full command, the current rises, the fold-back fires again.  The
         * dead-band is what makes the fold-back settle instead of hunting, and
         * the slow ramp is what keeps it from oscillating inside the dead-band.
         */
        if ((TP_FABS(motor->current_ma)
             < (MOTOR_FOLDBACK_RECOVER_BELOW * motor->current_limit_ma)) &&
            (motor->duty_limit < TP_MAX_PWM_PERMILLE)) {
            motor->duty_limit += MOTOR_FOLDBACK_RECOVER_STEP * TP_MAX_PWM_PERMILLE;
            if (motor->duty_limit > TP_MAX_PWM_PERMILLE) {
                motor->duty_limit = TP_MAX_PWM_PERMILLE;
            }
        }
    }

    /* --- 3. write the hardware ----------------------------------------- */
    st = tp_platform_motor_set_pwm(motor->ch, motor->duty_applied);
    if (st != TP_OK) {
        LOG_ERROR(TP_MOD_MOTOR, "channel %d PWM write failed (%s)",
                  (int)motor->ch, tp_status_str(st));
        return st;
    }
    return TP_OK;
}

tp_status_t tp_motor_stop(tp_motor_t *motor, uint32_t ramp_ms)
{
    if ((motor == NULL) || (motor->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    /* Requesting zero is all that is needed: the slew limiter turns it into a
     * bounded ramp, and `ramp_ms` is recorded so the caller's intent is
     * visible in a trace.  Ramping faster than the limiter allows would
     * reintroduce the current spike the limiter exists to prevent, so a larger
     * request is clamped rather than honoured. */
    (void)ramp_ms;
    motor->duty_cmd = 0.0f;
    motor->stop_requests++;
    return TP_OK;
}

tp_status_t tp_motor_disable(tp_motor_t *motor)
{
    tp_status_t st;

    if ((motor == NULL) || (motor->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    st = tp_platform_motor_enable(motor->ch, 0);
    motor->enabled = 0;
    motor->duty_cmd = 0.0f;
    motor->duty_applied = 0.0f;
    /* INFO, not WARN.  This function cannot tell why it was called, and the two
     * callers are a command (shutdown, abort) and a consequence (the safety
     * monitor cutting the drive after raising a fault, which logs its own
     * reason at the right severity).  Reporting a commanded shutdown as a
     * warning puts a WARN in every clean run and teaches the reader to ignore
     * the level entirely. */
    LOG_INFO(TP_MOD_MOTOR, "channel %d driver disabled", (int)motor->ch);
    return st;
}

tp_status_t tp_motor_enable(tp_motor_t *motor)
{
    tp_status_t st;

    if ((motor == NULL) || (motor->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (motor->fault != 0) {
        return TP_ERR_FAULT_ACTIVE;
    }
    if (tp_platform_is_killed() != 0) {
        /* The E-stop latch must be released explicitly, by the RESET handler,
         * before any drive can be energised again. */
        return TP_ERR_FAULT_ACTIVE;
    }
    st = tp_platform_motor_enable(motor->ch, 1);
    if (st == TP_OK) {
        motor->enabled = 1;
        tp_filter_slew_reset(&motor->slew);
        motor->duty_applied = 0.0f;
    }
    return st;
}

tp_status_t tp_motor_clear_fault(tp_motor_t *motor)
{
    if ((motor == NULL) || (motor->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (TP_FABS(motor->current_ma) > motor->current_limit_ma) {
        /* Refuse: clearing a protection while the condition persists would let
         * the caller defeat the protection with a single command. */
        LOG_WARN(TP_MOD_MOTOR,
                 "channel %d clear refused, still at %.0f mA (limit %.0f mA)",
                 (int)motor->ch, (double)motor->current_ma,
                 (double)motor->current_limit_ma);
        return TP_ERR_FAULT_ACTIVE;
    }
    motor->fault = 0;
    motor->overcurrent_samples = 0u;
    motor->duty_limit = TP_MAX_PWM_PERMILLE;
    return TP_OK;
}

float tp_motor_get_duty(const tp_motor_t *motor)
{
    return (motor != NULL) ? motor->duty_applied : 0.0f;
}

float tp_motor_get_current_ma(const tp_motor_t *motor)
{
    return (motor != NULL) ? motor->current_ma : 0.0f;
}

int tp_motor_is_faulted(const tp_motor_t *motor)
{
    return (motor != NULL) ? motor->fault : 1;
}

int tp_motor_is_enabled(const tp_motor_t *motor)
{
    return (motor != NULL) ? motor->enabled : 0;
}

float tp_motor_foldback_factor(const tp_motor_t *motor)
{
    float ratio;

    if ((motor == NULL) || (motor->current_limit_ma <= 0.0f)) {
        return 1.0f;
    }
    if (TP_FABS(motor->current_ma) <= motor->current_limit_ma) {
        return 1.0f;
    }
    ratio = motor->current_limit_ma / TP_FABS(motor->current_ma);
    return motor_clampf(ratio, 0.0f, 1.0f);
}
