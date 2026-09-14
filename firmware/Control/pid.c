/**
 * @file    pid.c
 * @brief   Implementation of the shared PID controller.
 *
 * The header documents the discrete equations; this file documents the
 * *ordering* decisions, because in a real controller the order of clamping,
 * integration and anti-windup changes the transient response measurably.
 *
 * @copyright MIT License, see LICENSE.
 */

#include "pid.h"

#include <string.h>

#include "compiler.h"
#include "logger.h"

/** @brief Smallest `dt` accepted, in seconds. Guards against divide-by-zero. */
#define PID_MIN_DT_S 1.0e-6f

/** @brief Default integral-leak time constant when none is configured. */
#define PID_DEFAULT_TAU_I_S 0.0f

/**
 * @brief Clamp a float into a range.
 *
 * @param[in] v   Value.
 * @param[in] lo  Lower bound.
 * @param[in] hi  Upper bound.
 * @return Clamped value.
 */
static float clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

tp_pid_config_t tp_pid_config_default(float kp, float ki, float kd, float dt_s,
                                      float out_max, float i_max, float tau_d_s)
{
    tp_pid_config_t cfg;

    (void)memset(&cfg, 0, sizeof(cfg));
    cfg.kp = kp;
    cfg.ki = ki;
    cfg.kd = kd;
    cfg.kff = 0.0f;
    cfg.dt_s = dt_s;
    cfg.out_max = out_max;
    cfg.out_min = -out_max;
    cfg.i_max = i_max;
    cfg.i_min = -i_max;
    cfg.tau_d_s = tau_d_s;
    cfg.tau_i_s = PID_DEFAULT_TAU_I_S;
    cfg.aw_gain = 0.0f;               /* 0 => derive the tracking gain.      */
    cfg.derivative_on_measurement = 1; /* The safe default; see pid.h table.  */
    cfg.antiwindup = TP_PID_AW_BACK_CALCULATION;
    return cfg;
}

tp_status_t tp_pid_init(tp_pid_t *pid, const tp_pid_config_t *cfg)
{
    if ((pid == NULL) || (cfg == NULL)) {
        return TP_ERR_PARAM;
    }

    /* Validate before touching the instance so a rejected config leaves the
     * previous controller state intact. */
    if (!TP_ISFINITE(cfg->dt_s) || (cfg->dt_s < PID_MIN_DT_S)) {
        LOG_ERROR(TP_MOD_CONTROL, "PID init rejected: dt=%f s invalid", cfg->dt_s);
        return TP_ERR_RANGE;
    }
    if (!TP_ISFINITE(cfg->out_max) || !TP_ISFINITE(cfg->out_min) ||
        (cfg->out_max <= cfg->out_min)) {
        LOG_ERROR(TP_MOD_CONTROL, "PID init rejected: output limits [%f, %f]",
                  cfg->out_min, cfg->out_max);
        return TP_ERR_RANGE;
    }
    if (!TP_ISFINITE(cfg->i_max) || !TP_ISFINITE(cfg->i_min) ||
        (cfg->i_max < cfg->i_min)) {
        LOG_ERROR(TP_MOD_CONTROL, "PID init rejected: integral limits [%f, %f]",
                  cfg->i_min, cfg->i_max);
        return TP_ERR_RANGE;
    }
    if ((cfg->tau_d_s < 0.0f) || !TP_ISFINITE(cfg->tau_d_s)) {
        return TP_ERR_RANGE;
    }

    (void)memset(pid, 0, sizeof(*pid));
    pid->kp = cfg->kp;
    pid->ki = cfg->ki;
    pid->kd = cfg->kd;
    pid->kff = cfg->kff;
    pid->dt_s = cfg->dt_s;
    pid->out_min = cfg->out_min;
    pid->out_max = cfg->out_max;
    pid->i_min = cfg->i_min;
    pid->i_max = cfg->i_max;
    pid->tau_d_s = cfg->tau_d_s;
    pid->tau_i_s = cfg->tau_i_s;
    pid->derivative_on_measurement = cfg->derivative_on_measurement;
    pid->antiwindup = cfg->antiwindup;

    /* The classical tracking time constant for back-calculation is
     * Tt = Ti = Kp/Ki, which makes the integrator unwind on the same time
     * scale as it wound up.  Callers may override it. */
    if (cfg->aw_gain > 0.0f) {
        pid->aw_gain = cfg->aw_gain;
    } else if ((cfg->ki > 1.0e-9f) && (cfg->kp > 1.0e-9f)) {
        pid->aw_gain = cfg->ki / cfg->kp;
    } else {
        /* No integral action configured: leave the integrator permanently
         * free so the (unused) I term can never wind up. */
        pid->aw_gain = 0.0f;
    }

    /* The integral clamp must not be able to drive the output past its own
     * clamp, otherwise the I term alone could saturate the actuator forever. */
    if (pid->i_max > pid->out_max) {
        pid->i_max = pid->out_max;
    }
    if (pid->i_min < pid->out_min) {
        pid->i_min = pid->out_min;
    }

    pid->initialised = 1;
    pid->first_update = 1;
    return TP_OK;
}

tp_status_t tp_pid_set_gains(tp_pid_t *pid, float kp, float ki, float kd)
{
    if ((pid == NULL) || (pid->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(kp) || !TP_ISFINITE(ki) || !TP_ISFINITE(kd)) {
        return TP_ERR_RANGE;
    }
    /* A negative integral gain would turn the anti-windup feedback into a
     * positive-feedback loop, so it is rejected outright. */
    if (ki < 0.0f) {
        return TP_ERR_RANGE;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    if ((pid->aw_gain <= 0.0f) && (ki > 1.0e-9f) && (kp > 1.0e-9f)) {
        pid->aw_gain = ki / kp;
    }
    return TP_OK;
}

tp_status_t tp_pid_set_ff_gain(tp_pid_t *pid, float kff)
{
    if ((pid == NULL) || (pid->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(kff)) {
        return TP_ERR_RANGE;
    }
    pid->kff = kff;
    return TP_OK;
}

tp_status_t tp_pid_set_limits(tp_pid_t *pid, float out_min, float out_max,
                              float i_min, float i_max)
{
    if ((pid == NULL) || (pid->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(out_min) || !TP_ISFINITE(out_max) ||
        !TP_ISFINITE(i_min) || !TP_ISFINITE(i_max)) {
        return TP_ERR_RANGE;
    }
    if ((out_max <= out_min) || (i_max < i_min)) {
        return TP_ERR_RANGE;
    }
    pid->out_min = out_min;
    pid->out_max = out_max;
    /* Same invariant as in init(): the I term alone must not saturate. */
    pid->i_min = TP_MAX(i_min, out_min);
    pid->i_max = TP_MIN(i_max, out_max);
    pid->integral = clampf(pid->integral, pid->i_min, pid->i_max);
    return TP_OK;
}

tp_status_t tp_pid_set_dt(tp_pid_t *pid, float dt_s)
{
    if ((pid == NULL) || (pid->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(dt_s) || (dt_s < PID_MIN_DT_S)) {
        return TP_ERR_RANGE;
    }
    pid->dt_s = dt_s;
    return TP_OK;
}

void tp_pid_reset(tp_pid_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->integral = 0.0f;
    pid->err_prev = 0.0f;
    pid->meas_prev = 0.0f;
    pid->d_filt = 0.0f;
    pid->last_output = 0.0f;
    pid->first_update = 1;
    pid->diverged = 0;
    pid->update_count = 0u;
    (void)memset(&pid->terms, 0, sizeof(pid->terms));
}

int tp_pid_is_diverged(const tp_pid_t *pid)
{
    if (pid == NULL) {
        return 1;
    }
    return pid->diverged;
}

tp_status_t tp_pid_get_terms(const tp_pid_t *pid, tp_pid_terms_t *terms)
{
    if ((pid == NULL) || (terms == NULL)) {
        if (terms != NULL) {
            (void)memset(terms, 0, sizeof(*terms));
        }
        return TP_ERR_PARAM;
    }
    *terms = pid->terms;
    return TP_OK;
}

float tp_pid_update(tp_pid_t *pid, float setpoint, float measurement)
{
    return tp_pid_update_ff(pid, setpoint, measurement, 0.0f);
}

float tp_pid_update_ff(tp_pid_t *pid, float setpoint, float measurement,
                       float feedforward)
{
    float error;
    float p_term;
    float i_term;
    float d_raw;
    float d_term;
    float ff_term;
    float out_raw;
    float out;

    if ((pid == NULL) || (pid->initialised == 0)) {
        return 0.0f;
    }

    /* --- input sanitation -------------------------------------------------
     * A NaN from a stuck ADC or a division by zero upstream must not be
     * allowed to propagate into the PWM register.  Latch and hold the last
     * good output so the joint keeps doing whatever it was doing rather than
     * snapping to zero or, worse, to full scale. */
    if (!TP_ISFINITE(setpoint) || !TP_ISFINITE(measurement) ||
        !TP_ISFINITE(feedforward)) {
        pid->diverged = 1;
        LOG_ERROR(TP_MOD_CONTROL, "PID non-finite input: r=%f y=%f ff=%f",
                  (double)setpoint, (double)measurement, (double)feedforward);
        return pid->last_output;
    }

    error = setpoint - measurement;

    /* --- first sample -----------------------------------------------------
     * There is no previous measurement, so any derivative computed from one
     * would be a fabrication.  Seed the memory and emit P + FF only. */
    if (pid->first_update != 0) {
        pid->meas_prev = measurement;
        pid->err_prev = error;
        pid->first_update = 0;
    }

    /* --- proportional ---------------------------------------------------- */
    p_term = pid->kp * error;

    /* --- integral (trapezoidal) ------------------------------------------
     * Trapezoidal rather than rectangular: for a ramping error the
     * rectangular rule adds a systematic half-sample bias which appears as a
     * steady-state position offset. */
    i_term = pid->integral + (pid->ki * 0.5f * (error + pid->err_prev) * pid->dt_s);

    /* Optional slow leak: turns the integrator into a first-order lag so a
     * constant disturbance is eventually rejected by position feedback alone.
     * tau_i_s == 0 disables it and leaves a true integrator. */
    if (pid->tau_i_s > 0.0f) {
        float alpha = pid->dt_s / (pid->tau_i_s + pid->dt_s);
        i_term -= alpha * pid->integral;
    }
    i_term = clampf(i_term, pid->i_min, pid->i_max);

    /* --- derivative ------------------------------------------------------ */
    if (pid->derivative_on_measurement != 0) {
        /* Negative sign: an increasing measurement means we need less output. */
        d_raw = -(measurement - pid->meas_prev) / pid->dt_s;
    } else {
        d_raw = (error - pid->err_prev) / pid->dt_s;
    }
    if (pid->tau_d_s > 0.0f) {
        /* Backward-Euler discretisation of 1/(tau*s + 1).  Using
         * dt/(tau+dt) keeps the filter stable for any dt, including
         * dt > tau, where the "textbook" alpha = dt/tau would blow up. */
        float alpha = pid->dt_s / (pid->tau_d_s + pid->dt_s);
        pid->d_filt += (d_raw - pid->d_filt) * alpha;
        d_term = pid->kd * pid->d_filt;
    } else {
        pid->d_filt = d_raw;
        d_term = pid->kd * d_raw;
    }

    /* --- feed-forward ---------------------------------------------------- */
    ff_term = pid->kff * feedforward;

    /* --- sum and clamp --------------------------------------------------- */
    out_raw = p_term + i_term + d_term + ff_term;
    if (!TP_ISFINITE(out_raw)) {
        pid->diverged = 1;
        LOG_ERROR(TP_MOD_CONTROL, "PID output non-finite, holding last value");
        return pid->last_output;
    }
    out = clampf(out_raw, pid->out_min, pid->out_max);

    /* --- anti-windup ------------------------------------------------------ */
    if (out != out_raw) {
        if (pid->antiwindup == TP_PID_AW_BACK_CALCULATION) {
            /* Track the actuator: pull the integrator towards the value that
             * would just barely produce the clamped output.  The correction
             * is limited to one clamp-width per sample so a huge saturation
             * error cannot fling the integrator to the opposite rail. */
            float correction = pid->aw_gain * (out - out_raw) * pid->dt_s;
            i_term += correction;
            i_term = clampf(i_term, pid->i_min, pid->i_max);
        } else if (pid->antiwindup == TP_PID_AW_CLAMP_INTEGRAL) {
            /* Conditional integration: only freeze when the error pushes
             * further into the saturation we are already in, otherwise the
             * integrator could never unwind. */
            int sat_high = (out >= pid->out_max) ? 1 : 0;
            if ((sat_high != 0) && (error > 0.0f)) {
                i_term = pid->integral; /* revert this step's integration */
            } else if ((sat_high == 0) && (error < 0.0f)) {
                i_term = pid->integral;
            }
        }
    }
    pid->integral = clampf(i_term, pid->i_min, pid->i_max);

    /* --- bookkeeping ------------------------------------------------------ */
    pid->err_prev = error;
    pid->meas_prev = measurement;
    pid->last_output = out;
    pid->update_count++;

    pid->terms.p = p_term;
    pid->terms.i = pid->integral;
    pid->terms.d = d_term;
    pid->terms.ff = ff_term;
    pid->terms.out = out;
    pid->terms.out_raw = out_raw;
    pid->terms.error = error;
    pid->terms.derivative = pid->d_filt;
    pid->terms.saturated = (out > out_raw) ? -1 : ((out < out_raw) ? 1 : 0);

    return out;
}
