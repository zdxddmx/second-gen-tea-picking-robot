/**
 * @file    force_control.c
 * @brief   Implementation of the flexible gripping controller.
 * @copyright MIT License, see LICENSE.
 */

#include "force_control.h"

#include <string.h>

#include "compiler.h"
#include "config.h"
#include "logger.h"
#include "platform_api.h"

/** @brief Bounded time to find contact before the cycle is aborted, ms. */
#define GRIP_CONTACT_TIMEOUT_MS 2500u

/** @brief Bounded time for the detachment phase, ms. */
#define GRIP_PICK_DURATION_MS 250u

/** @brief Contact debounce: consecutive samples above the threshold. */
#define GRIP_CONTACT_DEBOUNCE 3u

/** @brief Time the force must stay in band before the grip is declared settled. */
#define GRIP_SETTLE_MS TP_GRIPPER_SETTLE_MS

/** @brief Jaw velocity used while detaching the bud, mm/s (closing). */
#define GRIP_PICK_CLOSE_SPEED_MM_S 2.0f

/** @brief Jaw velocity used while releasing, mm/s (opening). */
#define GRIP_RELEASE_SPEED_MM_S 30.0f

/** @brief Sub-state names, indexed by @ref tp_grip_state_t. */
static const char *const s_grip_state_names[TP_GRIP_STATE_COUNT] = {
    "IDLE",
    "FAST_APPROACH",
    "SLOW_APPROACH",
    "CONTACT_DETECT",
    "FORCE_CONTROL",
    "PICK",
    "RELEASE",
    "FAULT"
};

const char *tp_grip_state_name(tp_grip_state_t state)
{
    if (((int)state < 0) || (state >= TP_GRIP_STATE_COUNT)) {
        return "UNKNOWN";
    }
    return s_grip_state_names[(int)state];
}

/**
 * @brief Enter a new sub-state, recording the transition time.
 *
 * Centralising the transition is what makes the dwell-time statistics in the
 * telemetry trustworthy; setting `state` directly anywhere else would leave
 * `state_entered_ms` stale and every duration measurement wrong.
 *
 * @param[in,out] fc    Controller.
 * @param[in]     state New sub-state.
 * @param[in]     now_ms Current time.
 */
static void grip_enter(tp_force_control_t *fc, tp_grip_state_t state,
                       uint32_t now_ms)
{
    if (fc->state != state) {
        LOG_DEBUG(TP_MOD_GRIPPER, "%s -> %s",
                  tp_grip_state_name(fc->state), tp_grip_state_name(state));
    }
    fc->state = state;
    fc->state_entered_ms = now_ms;
}

/**
 * @brief Accumulate regulated-phase force statistics for one period.
 *
 * Shared by the FORCE_CONTROL and PICK states so that both contribute to the
 * same "force actually held on the bud" figure, and so the two cannot drift
 * apart.
 *
 * @param[in,out] fc     Controller.
 * @param[in]     force  Latest filtered force, N.
 */
static void grip_accumulate_hold(tp_force_control_t *fc, float force)
{
    fc->hold_samples++;
    fc->hold_force_sum_n += force;
    if ((fc->hold_samples == 1u) || (force < fc->hold_force_min_n)) {
        fc->hold_force_min_n = force;
    }
    if (force > fc->hold_force_max_n) {
        fc->hold_force_max_n = force;
    }
}

/**
 * @brief Trip the protection: open the jaws and latch the fault.
 *
 * Idempotent, because several conditions can trip it within one period and the
 * log should record the event once.
 *
 * @param[in,out] fc      Controller.
 * @param[in]     grip    Gripper to open.
 * @param[in]     now_ms  Current time.
 * @param[in]     code    Fault code describing why.
 * @param[in]     reason  Human-readable reason for the log.
 */
static void grip_trip(tp_force_control_t *fc, tp_gripper_t *grip,
                      uint32_t now_ms, tp_fault_code_t code,
                      const char *reason)
{
    if (fc->fault_latched != 0) {
        return;
    }
    fc->fault_latched = 1;
    fc->fault_code = code;
    fc->fault_count++;
    (void)tp_pid_reset(&fc->force_pid);
    if (grip != NULL) {
        (void)tp_gripper_set_velocity_mm_s(grip, GRIP_RELEASE_SPEED_MM_S);
    }
    grip_enter(fc, TP_GRIP_FAULT, now_ms);
    LOG_ERROR(TP_MOD_GRIPPER, "PROTECTION TRIPPED: %s [%s] (F=%.3f N, peak %.3f N)",
              (reason != NULL) ? reason : "unspecified", tp_fault_name(code),
              (double)fc->force_feedback_n, (double)fc->peak_force_n);
}

tp_status_t tp_force_control_init(tp_force_control_t *fc, float dt_s)
{
    tp_pid_config_t cfg;
    tp_status_t st;

    if (fc == NULL) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(dt_s) || (dt_s <= 0.0f)) {
        return TP_ERR_RANGE;
    }

    (void)memset(fc, 0, sizeof(*fc));

    /* The force loop's output is a *velocity* command, so its limits come from
     * the jaw's speed range.  Note how much smaller the velocity clamp is than
     * the mechanical maximum: 25 mm/s against 120 mm/s.  A force loop that can
     * command the full jaw speed will, on the first sample after contact,
     * command a closing velocity that produces a force step big enough to
     * damage the bud before the loop can react. */
    cfg = tp_pid_config_default(TP_FORCE_PID_KP, TP_FORCE_PID_KI, TP_FORCE_PID_KD,
                               dt_s, TP_FORCE_PID_OUT_MAX, TP_FORCE_PID_IMAX,
                               TP_FORCE_PID_TAU_D_S);
    cfg.kff = 0.0f;
    st = tp_pid_init(&fc->force_pid, &cfg);
    if (st != TP_OK) {
        LOG_ERROR(TP_MOD_GRIPPER, "force loop init failed: %s", tp_status_str(st));
        return st;
    }

    fc->force_target_base_n = TP_FORCE_TARGET_N;
    fc->force_target_n = TP_FORCE_TARGET_N;
    fc->adapt_enabled = TP_FORCE_ADAPT_ENABLE;
    fc->observed_hold_ema_n = TP_FORCE_TARGET_N;
    fc->state = TP_GRIP_IDLE;
    fc->initialised = 1;

    LOG_INFO(TP_MOD_GRIPPER,
             "force control ready: F*=%.2f N, soft %.2f N, hard %.2f N, "
             "Kp=%.2f Ki=%.2f, clamp +-%.1f mm/s",
             (double)fc->force_target_n, (double)TP_FORCE_SOFT_LIMIT_N,
             (double)TP_FORCE_MAX_N, (double)TP_FORCE_PID_KP,
             (double)TP_FORCE_PID_KI, (double)TP_FORCE_PID_OUT_MAX);
    return TP_OK;
}

tp_status_t tp_force_control_begin(tp_force_control_t *fc, tp_gripper_t *grip,
                                   float approach_mm)
{
    if ((fc == NULL) || (grip == NULL) || (fc->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (fc->fault_latched != 0) {
        LOG_WARN(TP_MOD_GRIPPER, "begin refused: fault latched");
        return TP_ERR_FAULT_ACTIVE;
    }
    if (!TP_ISFINITE(approach_mm)) {
        return TP_ERR_RANGE;
    }

    /* Clamp the expected contact gap into the mechanical range.  A vision
     * system can report a depth the jaws cannot reach; trusting it would make
     * the "slow down 4 mm before contact" rule fire immediately or never. */
    if (approach_mm < TP_GRIPPER_MIN_MM) {
        approach_mm = TP_GRIPPER_MIN_MM;
    }
    if (approach_mm > TP_GRIPPER_MAX_MM) {
        approach_mm = TP_GRIPPER_MAX_MM;
    }
    fc->approach_target_mm = approach_mm;

    (void)tp_pid_reset(&fc->force_pid);
    fc->peak_force_n = 0.0f;
    fc->contact_force_n = 0.0f;
    fc->hold_samples = 0u;
    fc->hold_force_sum_n = 0.0f;
    fc->hold_force_min_n = 0.0f;
    fc->hold_force_max_n = 0.0f;
    fc->regulated = 0;
    fc->settle_samples = 0u;
    fc->settle_peak_n = 0.0f;
    fc->settle_start_ms = 0u;

    /* Default to the design setpoint; adaptation only overrides it once a hold
     * has actually been observed. */
    fc->force_target_n = fc->force_target_base_n;

    fc->cycle_start_ms = tp_platform_time_ms();
    grip_enter(fc, TP_GRIP_FAST_APPROACH, fc->cycle_start_ms);
    (void)tp_gripper_set_velocity_mm_s(grip, -TP_APPROACH_FAST_SPEED_MM_S);
    LOG_INFO(TP_MOD_GRIPPER,
             "cycle start: approach gap %.2f mm, F*=%.2f N",
             (double)approach_mm, (double)fc->force_target_n);
    return TP_OK;
}

tp_status_t tp_force_control_update(tp_force_control_t *fc, tp_gripper_t *grip,
                                    tp_force_sensor_t *fs, uint32_t now_ms,
                                    float dt_s)
{
    float force;
    float gap;
    float duty;

    if ((fc == NULL) || (grip == NULL) || (fs == NULL) ||
        (fc->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(dt_s) || (dt_s <= 0.0f)) {
        return TP_ERR_RANGE;
    }

    force = tp_force_sensor_get_force_n(fs);
    gap = tp_gripper_get_gap_mm(grip);
    fc->force_feedback_n = force;
    fc->force_error_n = fc->force_target_n - force;

    if (force > fc->peak_force_n) {
        fc->peak_force_n = force;
    }

    /* =====================================================================
     * Priority 1: absolute protection.  Evaluated BEFORE the controller, so a
     * hard over-force is acted on in the same control period it is detected in.
     * ===================================================================== */
    if (force > TP_FORCE_MAX_N) {
        fc->overforce_events++;
        grip_trip(fc, grip, now_ms, TP_FAULT_FORCE_OVERLOAD, "force above hard limit");
        return TP_ERR_FAULT_ACTIVE;
    }
    if (tp_force_sensor_is_faulty(fs) != 0) {
        grip_trip(fc, grip, now_ms, TP_FAULT_SENSOR, "force sensor faulty");
        return TP_ERR_FAULT_ACTIVE;
    }

    /* Priority 2: soft limit.  Derate the setpoint instead of tripping, so the
     * loop actively backs off rather than the whole cycle being lost. */
    if (force > TP_FORCE_SOFT_LIMIT_N) {
        if (fc->soft_limit_events == 0u) {
            LOG_WARN(TP_MOD_GRIPPER,
                     "force %.3f N above soft limit %.2f N, derating setpoint",
                     (double)force, (double)TP_FORCE_SOFT_LIMIT_N);
        }
        fc->soft_limit_events++;
        fc->force_target_n = TP_FORCE_SOFT_LIMIT_N * 0.9f;
    }

    switch (fc->state) {
    /* ------------------------------------------------------------------ */
    case TP_GRIP_FAST_APPROACH:
        if ((now_ms - fc->cycle_start_ms) > GRIP_CONTACT_TIMEOUT_MS) {
            fc->contact_timeouts++;
            grip_trip(fc, grip, now_ms, TP_FAULT_TARGET, "no contact within timeout");
            return TP_ERR_FAULT_ACTIVE;
        }
        /* Switch to the fine approach a fixed distance before the *expected*
         * contact, not at contact: the whole purpose of the slow phase is to
         * have control authority already established when contact happens. */
        if (gap <= (fc->approach_target_mm + TP_APPROACH_SLOW_AT_MM)) {
            grip_enter(fc, TP_GRIP_SLOW_APPROACH, now_ms);
            (void)tp_gripper_set_velocity_mm_s(grip, -TP_APPROACH_SLOW_SPEED_MM_S);
        } else {
            (void)tp_gripper_set_velocity_mm_s(grip, -TP_APPROACH_FAST_SPEED_MM_S);
        }
        break;

    /* ------------------------------------------------------------------ */
    case TP_GRIP_SLOW_APPROACH:
        if ((now_ms - fc->cycle_start_ms) > GRIP_CONTACT_TIMEOUT_MS) {
            fc->contact_timeouts++;
            grip_trip(fc, grip, now_ms, TP_FAULT_TARGET, "no contact within timeout");
            return TP_ERR_FAULT_ACTIVE;
        }
        if (force > TP_FORCE_CONTACT_N) {
            grip_enter(fc, TP_GRIP_CONTACT_DETECT, now_ms);
            /* Stop closing while the contact is confirmed: continuing to close
             * at 6 mm/s for the 3-sample debounce window would add roughly
             * 0.036 mm of penetration, i.e. about 11 mN, which is acceptable
             * but unnecessary. */
            (void)tp_gripper_set_velocity_mm_s(grip, 0.0f);
            fc->contact_force_n = force;
        } else {
            (void)tp_gripper_set_velocity_mm_s(grip, -TP_APPROACH_SLOW_SPEED_MM_S);
        }
        break;

    /* ------------------------------------------------------------------ */
    case TP_GRIP_CONTACT_DETECT:
        /* Require the force to remain above the threshold for
         * GRIP_CONTACT_DEBOUNCE consecutive samples.  A single sample could be
         * a spike that survived the median filter; a run of them cannot be,
         * because the median rejects isolated outliers by construction. */
        if (force <= (TP_FORCE_CONTACT_N * 0.5f)) {
            /* The force went away: it was not a contact after all. */
            LOG_DEBUG(TP_MOD_GRIPPER,
                      "contact not confirmed (F fell to %.3f N), resuming slow approach",
                      (double)force);
            grip_enter(fc, TP_GRIP_SLOW_APPROACH, now_ms);
            break;
        }
        if ((now_ms - fc->state_entered_ms) >=
            (GRIP_CONTACT_DEBOUNCE * (uint32_t)(dt_s * 1000.0f))) {
            LOG_INFO(TP_MOD_GRIPPER,
                     "Contact detected at gap %.3f mm, F=%.3f N", (double)gap,
                     (double)force);
            /* Hand over to the force loop with a clean integrator.  Carrying
             * the (empty) integrator over is harmless here, but resetting makes
             * the transition explicit and is essential if this code is ever
             * reached from FORCE_CONTROL again after a re-approach. */
            (void)tp_pid_reset(&fc->force_pid);
            grip_enter(fc, TP_GRIP_FORCE_CONTROL, now_ms);
        }
        break;

    /* ------------------------------------------------------------------ */
    case TP_GRIP_FORCE_CONTROL:
        /*
         * The force PID's output is a *closing velocity*.  Its sign convention
         * is inverted before it reaches the gripper: a positive duty closes, so
         * a positive velocity command opens.  Doing the inversion here, once,
         * keeps `tp_pid_update` a plain controller with no knowledge of the
         * mechanical sense of the axis.
         */
        duty = tp_pid_update(&fc->force_pid, fc->force_target_n, force);
        (void)tp_pid_get_terms(&fc->force_pid, &fc->force_terms);
        fc->velocity_cmd_mm_s = -duty;
        (void)tp_gripper_set_velocity_mm_s(grip, fc->velocity_cmd_mm_s);

        /* Statistics.  The settling transient is measured separately from the
         * regulated phase, because mixing them makes a correctly regulated
         * 2.2 N grip report a mean of 1.66 N. */
        if (fc->regulated == 0) {
            fc->settle_samples++;
            if (force > fc->settle_peak_n) {
                fc->settle_peak_n = force;
            }
        }

        /* Settled?  The force must stay inside the band *continuously*: the
         * timer is restarted whenever it leaves, so a force that oscillates
         * through the band never counts as settled.  That is stricter than a
         * "has been in band at some point" test, and it is what makes the
         * reported settling time honest. */
        if (TP_FABS(fc->force_error_n) <= TP_FORCE_BAND_N) {
            fc->regulated = 1;
            grip_accumulate_hold(fc, force);
            if (fc->settle_start_ms == 0u) {
                fc->settle_start_ms = now_ms;
            } else if ((now_ms - fc->settle_start_ms) >= GRIP_SETTLE_MS) {
                LOG_INFO(TP_MOD_GRIPPER,
                         "Target force reached: %.2fN (band +-%.2f N, %.2f N overshoot, "
                         "%lu periods to settle)",
                         (double)force, (double)TP_FORCE_BAND_N,
                         (double)tp_force_control_overshoot(fc),
                         (unsigned long)fc->settle_samples);
                grip_enter(fc, TP_GRIP_PICK, now_ms);
            }
        } else {
            fc->settle_start_ms = 0u;
        }

        /* A force loop that has been running for a long time without settling
         * is a loop that is not working (gains, a slipping jaw, a bud that
         * rolled).  Bounding it prevents an unbounded cycle time. */
        if ((now_ms - fc->state_entered_ms) > GRIP_CONTACT_TIMEOUT_MS) {
            grip_trip(fc, grip, now_ms, TP_FAULT_FORCE_OVERLOAD, "force loop failed to settle within timeout");
            return TP_ERR_FAULT_ACTIVE;
        }
        break;

    /* ------------------------------------------------------------------ */
    case TP_GRIP_PICK:
        if ((now_ms - fc->state_entered_ms) >= GRIP_PICK_DURATION_MS) {
            /* End of the detachment hold.  The application performs the actual
             * retraction move (the arm lifts) while the gripper keeps its
             * force; here the grip is simply maintained. */
            LOG_INFO(TP_MOD_GRIPPER, "Pick complete, releasing (mean hold %.2f N)",
                     (double)tp_force_control_mean_hold_force(fc));
            grip_enter(fc, TP_GRIP_RELEASE, now_ms);
            (void)tp_gripper_set_velocity_mm_s(grip, GRIP_RELEASE_SPEED_MM_S);
            (void)tp_pid_reset(&fc->force_pid);
            break;
        }
        /* Keep regulating while detaching: if the bud starts to slip out, the
         * force drops and the loop must close a little more to hold it.  A
         * controller that froze the position here would drop the bud. */
        duty = tp_pid_update(&fc->force_pid, fc->force_target_n, force);
        (void)tp_pid_get_terms(&fc->force_pid, &fc->force_terms);
        fc->velocity_cmd_mm_s = -duty - GRIP_PICK_CLOSE_SPEED_MM_S;
        (void)tp_gripper_set_velocity_mm_s(grip, fc->velocity_cmd_mm_s);
        /* The detachment phase is part of the regulated grip: the force here is
         * exactly what the bud experiences while it is being pulled free. */
        fc->regulated = 1;
        grip_accumulate_hold(fc, force);
        break;

    /* ------------------------------------------------------------------ */
    case TP_GRIP_RELEASE:
        if (tp_gripper_is_open(grip) != 0) {
            /* Update the adaptive threshold with what this cycle actually
             * needed.  Only a cycle that reached the hold phase contributes,
             * so a failed cycle cannot drag the estimate. */
            if (fc->hold_samples > 0u) {
                float observed = fc->hold_force_sum_n / (float)fc->hold_samples;
                fc->observed_hold_ema_n += (observed - fc->observed_hold_ema_n)
                                         * TP_FORCE_ADAPT_ALPHA;
                if (fc->adapt_enabled != 0) {
                    float suggested = fc->observed_hold_ema_n
                                    + TP_FORCE_ADAPT_MARGIN_N;
                    if (suggested < TP_FORCE_TARGET_MIN_N) {
                        suggested = TP_FORCE_TARGET_MIN_N;
                    }
                    if (suggested > TP_FORCE_TARGET_MAX_N) {
                        suggested = TP_FORCE_TARGET_MAX_N;
                    }
                    fc->force_target_base_n = suggested;
                }
            }
            fc->cycle_count++;
            grip_enter(fc, TP_GRIP_IDLE, now_ms);
            return TP_OK;
        }
        (void)tp_gripper_set_velocity_mm_s(grip, GRIP_RELEASE_SPEED_MM_S);
        break;

    /* ------------------------------------------------------------------ */
    case TP_GRIP_FAULT:
        /* Hold the jaws open; the application decides when to clear. */
        (void)tp_gripper_set_velocity_mm_s(grip, GRIP_RELEASE_SPEED_MM_S);
        return TP_ERR_FAULT_ACTIVE;

    /* ------------------------------------------------------------------ */
    case TP_GRIP_IDLE:
    default:
        return TP_ERR_STATE;
    }

    return TP_OK;
}

tp_status_t tp_force_control_abort(tp_force_control_t *fc, tp_gripper_t *grip)
{
    if ((fc == NULL) || (fc->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    (void)tp_pid_reset(&fc->force_pid);
    if (grip != NULL) {
        (void)tp_gripper_open(grip);
    }
    fc->state = TP_GRIP_IDLE;
    fc->velocity_cmd_mm_s = 0.0f;
    LOG_INFO(TP_MOD_GRIPPER, "sequence aborted, jaws opening");
    return TP_OK;
}

tp_status_t tp_force_control_set_force(tp_force_control_t *fc, float force_n)
{
    if ((fc == NULL) || (fc->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(force_n)) {
        return TP_ERR_RANGE;
    }
    if ((force_n < TP_FORCE_TARGET_MIN_N) || (force_n > TP_FORCE_TARGET_MAX_N)) {
        LOG_WARN(TP_MOD_GRIPPER,
                 "force target %.3f N outside [%.2f, %.2f] N, rejected",
                 (double)force_n, (double)TP_FORCE_TARGET_MIN_N,
                 (double)TP_FORCE_TARGET_MAX_N);
        return TP_ERR_RANGE;
    }
    fc->force_target_base_n = force_n;
    fc->force_target_n = force_n;
    return TP_OK;
}

float tp_force_control_get_force_target(const tp_force_control_t *fc)
{
    return (fc != NULL) ? fc->force_target_n : 0.0f;
}

tp_status_t tp_force_control_set_gains(tp_force_control_t *fc, float kp,
                                       float ki, float kd)
{
    if ((fc == NULL) || (fc->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    return tp_pid_set_gains(&fc->force_pid, kp, ki, kd);
}

tp_status_t tp_force_control_get_gains(const tp_force_control_t *fc, float *kp,
                                       float *ki, float *kd)
{
    if ((fc == NULL) || (kp == NULL) || (ki == NULL) || (kd == NULL)) {
        return TP_ERR_PARAM;
    }
    *kp = fc->force_pid.kp;
    *ki = fc->force_pid.ki;
    *kd = fc->force_pid.kd;
    return TP_OK;
}

void tp_force_control_set_adaptive(tp_force_control_t *fc, int enable)
{
    if (fc == NULL) {
        return;
    }
    fc->adapt_enabled = (enable != 0) ? 1 : 0;
    LOG_INFO(TP_MOD_GRIPPER, "adaptive threshold %s",
             (enable != 0) ? "enabled" : "disabled");
}

tp_grip_state_t tp_force_control_state(const tp_force_control_t *fc)
{
    return (fc != NULL) ? fc->state : TP_GRIP_FAULT;
}

int tp_force_control_is_complete(const tp_force_control_t *fc)
{
    return ((fc != NULL) && (fc->state == TP_GRIP_IDLE)) ? 1 : 0;
}

int tp_force_control_is_faulted(const tp_force_control_t *fc)
{
    return (fc != NULL) ? fc->fault_latched : 1;
}

tp_status_t tp_force_control_clear_fault(tp_force_control_t *fc)
{
    if ((fc == NULL) || (fc->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (fc->force_feedback_n > TP_FORCE_SOFT_LIMIT_N) {
        LOG_WARN(TP_MOD_GRIPPER,
                 "clear refused: force still %.3f N (soft limit %.2f N)",
                 (double)fc->force_feedback_n, (double)TP_FORCE_SOFT_LIMIT_N);
        return TP_ERR_FAULT_ACTIVE;
    }
    fc->fault_latched = 0;
    fc->fault_code = TP_FAULT_NONE;
    fc->overforce_events = 0u;
    fc->soft_limit_events = 0u;
    fc->state = TP_GRIP_IDLE;
    return TP_OK;
}

tp_fault_code_t tp_force_control_fault_code(const tp_force_control_t *fc)
{
    return (fc != NULL) ? fc->fault_code : TP_FAULT_NONE;
}

float tp_force_control_mean_hold_force(const tp_force_control_t *fc)
{
    if ((fc == NULL) || (fc->hold_samples == 0u)) {
        return 0.0f;
    }
    return fc->hold_force_sum_n / (float)fc->hold_samples;
}

float tp_force_control_peak_force(const tp_force_control_t *fc)
{
    return (fc != NULL) ? fc->peak_force_n : 0.0f;
}

float tp_force_control_overshoot(const tp_force_control_t *fc)
{
    float over;

    if (fc == NULL) {
        return 0.0f;
    }
    over = fc->settle_peak_n - fc->force_target_n;
    return (over > 0.0f) ? over : 0.0f;
}

uint32_t tp_force_control_settle_samples(const tp_force_control_t *fc)
{
    return (fc != NULL) ? fc->settle_samples : 0u;
}

float tp_force_control_force_error(const tp_force_control_t *fc)
{
    return (fc != NULL) ? fc->force_error_n : 0.0f;
}
