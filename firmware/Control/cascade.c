/**
 * @file    cascade.c
 * @brief   Implementation of the cascaded position/velocity controller.
 * @copyright MIT License, see LICENSE.
 */

#include "cascade.h"

#include <string.h>

#include "compiler.h"
#include "config.h"
#include "logger.h"

/** @brief Position error above which a tracking error is counted, mrad (5 deg). */
#define CASCADE_TRACKING_ERROR_MRAD 5000.0f

tp_status_t tp_cascade_init(tp_cascade_t *c, float dt_s)
{
    tp_pid_config_t cfg;
    tp_status_t st;

    if (c == NULL) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(dt_s) || (dt_s <= 0.0f)) {
        return TP_ERR_RANGE;
    }

    (void)memset(c, 0, sizeof(*c));

    /* --- inner loop first -------------------------------------------------
     * Its output is the PWM duty, so its limits come from the drive.  Doing it
     * first also means the outer loop's output limit can be stated as "the
     * inner loop's input range", which is the design intent. */
    cfg = tp_pid_config_default(TP_VEL_PID_KP, TP_VEL_PID_KI, TP_VEL_PID_KD,
                               dt_s, TP_VEL_PID_OUT_MAX, TP_VEL_PID_IMAX,
                               TP_VEL_PID_TAU_D_S);
    cfg.kff = TP_VEL_PID_FF_GAIN;
    st = tp_pid_init(&c->velocity_pid, &cfg);
    if (st != TP_OK) {
        LOG_ERROR(TP_MOD_CONTROL, "velocity loop init failed: %s",
                  tp_status_str(st));
        return st;
    }

    /* --- outer loop ------------------------------------------------------
     * Its output is a velocity demand, so its clamp is the joint speed limit.
     * Note the asymmetry: the integral clamp is much smaller than the output
     * clamp.  That is intentional and is the single most effective
     * anti-overshoot measure in the whole controller: it bounds how much
     * velocity the I term can demand on its own, so a long approach phase
     * cannot build up a charge that is released as an overshoot at the end. */
    cfg = tp_pid_config_default(TP_POS_PID_KP, TP_POS_PID_KI, TP_POS_PID_KD,
                               dt_s, TP_POS_PID_OUT_MAX, TP_POS_PID_IMAX,
                               TP_POS_PID_TAU_D_S);
    cfg.kff = TP_POS_PID_FF_GAIN;
    st = tp_pid_init(&c->position_pid, &cfg);
    if (st != TP_OK) {
        LOG_ERROR(TP_MOD_CONTROL, "position loop init failed: %s",
                  tp_status_str(st));
        return st;
    }

    st = tp_trajectory_init(&c->trajectory, TP_PROFILE_TRAPEZOID,
                            TP_JOINT_HOME_MRAD, dt_s);
    if (st != TP_OK) {
        return st;
    }

    c->mode = TP_CASCADE_MODE_OFF;
    c->initialised = 1;
    LOG_INFO(TP_MOD_CONTROL,
             "cascade ready: pos Kp=%.2f Ki=%.2f (clamp +-%.0f, I +-%.0f), "
             "vel Kp=%.4f Ki=%.3f (clamp +-%.0f)",
             (double)TP_POS_PID_KP, (double)TP_POS_PID_KI,
             (double)TP_POS_PID_OUT_MAX, (double)TP_POS_PID_IMAX,
             (double)TP_VEL_PID_KP, (double)TP_VEL_PID_KI,
             (double)TP_VEL_PID_OUT_MAX);
    return TP_OK;
}

tp_status_t tp_cascade_set_gains(tp_cascade_t *c, uint8_t loop, float kp,
                                 float ki, float kd)
{
    if ((c == NULL) || (c->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    switch (loop) {
    case TP_CASCADE_LOOP_POSITION:
        return tp_pid_set_gains(&c->position_pid, kp, ki, kd);
    case TP_CASCADE_LOOP_VELOCITY:
        return tp_pid_set_gains(&c->velocity_pid, kp, ki, kd);
    default:
        return TP_ERR_RANGE;
    }
}

tp_status_t tp_cascade_get_gains(const tp_cascade_t *c, uint8_t loop, float *kp,
                                 float *ki, float *kd)
{
    const tp_pid_t *pid;

    if ((c == NULL) || (kp == NULL) || (ki == NULL) || (kd == NULL)) {
        return TP_ERR_PARAM;
    }
    switch (loop) {
    case TP_CASCADE_LOOP_POSITION:
        pid = &c->position_pid;
        break;
    case TP_CASCADE_LOOP_VELOCITY:
        pid = &c->velocity_pid;
        break;
    default:
        return TP_ERR_RANGE;
    }
    *kp = pid->kp;
    *ki = pid->ki;
    *kd = pid->kd;
    return TP_OK;
}

tp_status_t tp_cascade_set_mode(tp_cascade_t *c, tp_cascade_mode_t mode)
{
    if ((c == NULL) || (c->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (c->mode != mode) {
        LOG_INFO(TP_MOD_CONTROL, "mode %d -> %d, integrators cleared",
                 (int)c->mode, (int)mode);
    }
    c->mode = mode;
    tp_cascade_reset(c);
    return TP_OK;
}

tp_status_t tp_cascade_set_reference(tp_cascade_t *c, float mrad, float v_ff)
{
    if ((c == NULL) || (c->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(mrad) || !TP_ISFINITE(v_ff)) {
        return TP_ERR_RANGE;
    }
    /* The feed-forward velocity is a *trajectory* quantity and is therefore
     * bounded by the same joint speed limit as everything else.  Validating only
     * for finiteness let a caller pass 1e9 mrad/s — finite, absurd, and enough to
     * saturate the inner loop for the whole move.  The bound is deliberately
     * twice the limit rather than equal to it, so a legitimate profile that
     * briefly plans at the limit is not rejected by rounding. */
    if (TP_FABS(v_ff) > (2.0f * TP_JOINT_MAX_SPEED_MRAD_S)) {
        LOG_WARN(TP_MOD_CONTROL, "feed-forward %.1f mrad/s outside the permitted band",
                 (double)v_ff);
        return TP_ERR_RANGE;
    }
    /* Clamp to the soft limits here rather than trusting the caller.  A target
     * outside the mechanical range is not a controller problem, it is invalid
     * input, and letting it through would drive the joint into its stop. */
    if ((mrad < TP_JOINT_MIN_MRAD) || (mrad > TP_JOINT_MAX_MRAD)) {
        LOG_WARN(TP_MOD_CONTROL, "reference %.1f mrad outside [%.0f, %.0f]",
                 (double)mrad, (double)TP_JOINT_MIN_MRAD,
                 (double)TP_JOINT_MAX_MRAD);
        return TP_ERR_RANGE;
    }
    c->position_ref = mrad;
    c->velocity_ff = v_ff;
    return TP_OK;
}

tp_status_t tp_cascade_set_velocity_reference(tp_cascade_t *c, float mrad_s)
{
    if ((c == NULL) || (c->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(mrad_s)) {
        return TP_ERR_RANGE;
    }
    if (TP_FABS(mrad_s) > TP_JOINT_MAX_SPEED_MRAD_S) {
        return TP_ERR_RANGE;
    }
    c->velocity_ref = mrad_s;
    return TP_OK;
}

tp_status_t tp_cascade_set_profile(tp_cascade_t *c, tp_profile_t profile)
{
    if ((c == NULL) || (c->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (((int)profile < 0) || (profile > TP_PROFILE_SCURVE)) {
        return TP_ERR_RANGE;
    }
    c->trajectory.profile = profile;
    return TP_OK;
}

float tp_cascade_update(tp_cascade_t *c, tp_encoder_t *enc, float dt_s)
{
    float position_fb;
    float velocity_fb;
    float v_demand;
    float duty;

    if ((c == NULL) || (c->initialised == 0) || (enc == NULL)) {
        return 0.0f;
    }
    if (!TP_ISFINITE(dt_s) || (dt_s <= 0.0f)) {
        return 0.0f;
    }

    /* --- 1. feedback ---------------------------------------------------- */
    position_fb = tp_encoder_get_position_mrad(enc);
    velocity_fb = tp_encoder_get_speed_mrad_s(enc);
    if (!TP_ISFINITE(position_fb) || !TP_ISFINITE(velocity_fb)) {
        LOG_ERROR(TP_MOD_CONTROL, "non-finite encoder feedback, output zeroed");
        return 0.0f;
    }
    c->position_fb = position_fb;
    c->velocity_fb = velocity_fb;

    if (c->mode == TP_CASCADE_MODE_OFF) {
        c->duty_cmd = 0.0f;
        return 0.0f;
    }

    /* --- 2. outer loop (position -> velocity demand) --------------------- */
    if (c->mode == TP_CASCADE_MODE_POSITION) {
        v_demand = tp_pid_update_ff(&c->position_pid, c->position_ref,
                                    position_fb, c->velocity_ff);
        (void)tp_pid_get_terms(&c->position_pid, &c->pos_terms);
        if (c->pos_terms.saturated != 0) {
            c->pos_saturated_count++;
        }
    } else {
        /* Velocity mode: the outer loop is bypassed entirely and the velocity
         * reference is taken from outside. */
        v_demand = c->velocity_ref;
        c->pos_terms.p = 0.0f;
        c->pos_terms.i = 0.0f;
        c->pos_terms.d = 0.0f;
        c->pos_terms.ff = 0.0f;
        c->pos_terms.out = v_demand;
        c->pos_terms.out_raw = v_demand;
        c->pos_terms.error = 0.0f;
        c->pos_terms.derivative = 0.0f;
        c->pos_terms.saturated = 0;
    }
    c->velocity_ref = v_demand;
    c->position_error = c->position_ref - position_fb;

    /* --- 3. inner loop (velocity -> duty) -------------------------------- */
    duty = tp_pid_update(&c->velocity_pid, v_demand, velocity_fb);
    (void)tp_pid_get_terms(&c->velocity_pid, &c->vel_terms);
    if (c->vel_terms.saturated != 0) {
        c->vel_saturated_count++;
    }

    /* --- 4. statistics ---------------------------------------------------- */
    c->duty_cmd = duty;
    c->velocity_error = v_demand - velocity_fb;
    c->update_count++;
    {
        float ae = TP_FABS(c->position_error);
        if (ae > c->max_position_error) {
            c->max_position_error = ae;
        }
        /* Running mean of the squared error, so the RMS can be reported
         * without storing the whole history. */
        c->rms_position_error += ((ae * ae) - c->rms_position_error)
                               / (float)c->update_count;
        if (ae > CASCADE_TRACKING_ERROR_MRAD) {
            c->tracking_error_count++;
        }
    }
    return duty;
}

void tp_cascade_reset(tp_cascade_t *c)
{
    if (c == NULL) {
        return;
    }
    tp_pid_reset(&c->position_pid);
    tp_pid_reset(&c->velocity_pid);
}

void tp_cascade_reset_stats(tp_cascade_t *c)
{
    if (c == NULL) {
        return;
    }
    c->update_count = 0u;
    c->vel_saturated_count = 0u;
    c->pos_saturated_count = 0u;
    c->tracking_error_count = 0u;
    c->max_position_error = 0.0f;
    c->rms_position_error = 0.0f;
}

float tp_cascade_position_ref(const tp_cascade_t *c)
{
    return (c != NULL) ? c->position_ref : 0.0f;
}

float tp_cascade_position_error(const tp_cascade_t *c)
{
    return (c != NULL) ? c->position_error : 0.0f;
}

float tp_cascade_duty(const tp_cascade_t *c)
{
    return (c != NULL) ? c->duty_cmd : 0.0f;
}

tp_trajectory_t *tp_cascade_trajectory(tp_cascade_t *c)
{
    return (c != NULL) ? &c->trajectory : NULL;
}

int tp_cascade_is_diverged(const tp_cascade_t *c)
{
    if (c == NULL) {
        return 1;
    }
    return ((tp_pid_is_diverged(&c->position_pid) != 0) ||
            (tp_pid_is_diverged(&c->velocity_pid) != 0)) ? 1 : 0;
}
