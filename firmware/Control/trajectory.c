/**
 * @file    trajectory.c
 * @brief   Implementation of the reference profile generators.
 * @copyright MIT License, see LICENSE.
 */

#include "trajectory.h"

#include <string.h>

#include "compiler.h"
#include "config.h"
#include "logger.h"

/**
 * @brief Clamp a float into a range.
 *
 * @param[in] v   Value.
 * @param[in] lo  Lower bound.
 * @param[in] hi  Upper bound.
 * @return Clamped value.
 */
static float traj_clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

/**
 * @brief Compute the phase durations of a trapezoidal move.
 *
 * Two cases:
 *
 *   - **Trapezoidal**: the move is long enough to reach `v_max`.  The
 *     acceleration phase lasts `v_max/a` and covers `v_max^2/(2a)` of distance
 *     in each of the two ramps.
 *   - **Triangular**: the move is too short to reach `v_max`.  The peak is then
 *     `sqrt(a*d)`, which is the largest velocity that still allows the move to
 *     be completed with the given acceleration over distance `d`.
 *
 * Getting the triangular case right is what makes short moves behave: an
 * implementation that always assumes the trapezoid computes a negative flat
 * phase and produces a reference that jumps backwards.
 *
 * @param[in,out] t  Generator to compute for.
 * @param[in]     d  Absolute distance to travel.
 */
static void traj_plan_trapezoid(tp_trajectory_t *t, float d)
{
    float ramp_distance;

    t->triangular = 0;
    if ((t->max_accel <= 0.0f) || (t->max_velocity <= 0.0f)) {
        /* No limits configured: degenerate to a step so the generator cannot
         * divide by zero or produce an infinite duration. */
        t->t_accel = 0.0f;
        t->t_flat = 0.0f;
        t->v_peak = 0.0f;
        t->duration_s = 0.0f;
        return;
    }
    if (d <= 0.0f) {
        t->t_accel = 0.0f;
        t->t_flat = 0.0f;
        t->v_peak = 0.0f;
        t->duration_s = 0.0f;
        return;
    }

    /* Distance consumed by one acceleration ramp when reaching v_max. */
    ramp_distance = (t->max_velocity * t->max_velocity) / (2.0f * t->max_accel);

    if ((2.0f * ramp_distance) <= d) {
        t->v_peak = t->max_velocity;
        t->t_accel = t->max_velocity / t->max_accel;
        t->t_flat = (d - (2.0f * ramp_distance)) / t->max_velocity;
        t->duration_s = (2.0f * t->t_accel) + t->t_flat;
    } else {
        t->triangular = 1;
        {
            float v2 = d * t->max_accel;
            /* Square root without libm: Newton's method converges in 4
             * iterations to better than 1e-6 relative for this range, and it
             * keeps the firmware free of a `sqrtf` dependency in the 1 kHz
             * path. */
            float x;
            int   i;
            if (v2 <= 0.0f) {
                t->v_peak = 0.0f;
            } else {
                x = (v2 > 1.0f) ? v2 * 0.5f : 1.0f;
                for (i = 0; i < 24; ++i) {
                    x = 0.5f * (x + (v2 / x));
                }
                t->v_peak = x;
            }
        }
        t->t_accel = t->v_peak / t->max_accel;
        t->t_flat = 0.0f;
        t->duration_s = 2.0f * t->t_accel;
    }
}

tp_status_t tp_trajectory_init(tp_trajectory_t *traj, tp_profile_t profile,
                               float start_value, float dt_s)
{
    if (traj == NULL) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(dt_s) || (dt_s <= 0.0f)) {
        return TP_ERR_RANGE;
    }
    if (((int)profile < 0) || (profile > TP_PROFILE_SCURVE)) {
        return TP_ERR_PARAM;
    }
    (void)memset(traj, 0, sizeof(*traj));
    traj->profile = profile;
    traj->start = TP_ISFINITE(start_value) ? start_value : 0.0f;
    traj->target = traj->start;
    traj->value = traj->start;
    traj->max_velocity = TP_JOINT_MAX_SPEED_MRAD_S;
    traj->max_accel = TP_JOINT_MAX_ACCEL_MRAD_S2;
    /* A jerk limit of 10x the acceleration is the usual starting point: it
     * spreads the acceleration ramp over 100 ms at this machine's limits, which
     * is long enough to suppress the excitation of the geared axis. */
    traj->jerk = TP_JOINT_MAX_ACCEL_MRAD_S2 * 10.0f;
    traj->initialised = 1;
    return TP_OK;
}

tp_status_t tp_trajectory_set_limits(tp_trajectory_t *traj, float max_velocity,
                                     float max_accel, float jerk)
{
    if ((traj == NULL) || (traj->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(max_velocity) || !TP_ISFINITE(max_accel) ||
        !TP_ISFINITE(jerk)) {
        return TP_ERR_RANGE;
    }
    if ((max_velocity <= 0.0f) || (max_accel <= 0.0f) || (jerk <= 0.0f)) {
        return TP_ERR_RANGE;
    }
    traj->max_velocity = max_velocity;
    traj->max_accel = max_accel;
    traj->jerk = jerk;
    return TP_OK;
}

tp_status_t tp_trajectory_start(tp_trajectory_t *traj, float target)
{
    if ((traj == NULL) || (traj->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(target)) {
        return TP_ERR_RANGE;
    }
    traj->start = traj->value;
    traj->target = target;
    traj->elapsed_s = 0.0f;
    traj->velocity = 0.0f;
    traj->accel = 0.0f;
    traj_plan_trapezoid(traj, TP_FABS(target - traj->start));
    traj->active = (traj->duration_s > 0.0f) ? 1 : 0;
    if (traj->active == 0) {
        traj->value = target;
    }
    return TP_OK;
}

/**
 * @brief Evaluate the trapezoidal profile at time @p t.
 *
 * @param[in,out] traj  Generator (phase durations already planned).
 * @param[in]     t     Time since the move started.
 * @param[in]     d     Absolute distance.
 * @param[in]     sign  +1 or -1, the direction of travel.
 */
static void traj_eval_trapezoid(tp_trajectory_t *traj, float t, float d,
                                float sign)
{
    float pos;

    if (traj->duration_s <= 0.0f) {
        traj->value = traj->target;
        traj->velocity = 0.0f;
        traj->accel = 0.0f;
        return;
    }

    if (t >= traj->duration_s) {
        traj->value = traj->target;
        traj->velocity = 0.0f;
        traj->accel = 0.0f;
        traj->active = 0;
        return;
    }

    if (t < traj->t_accel) {
        /* Accelerating. */
        traj->accel = traj->max_accel;
        traj->velocity = traj->max_accel * t;
        pos = 0.5f * traj->max_accel * t * t;
    } else if (t < (traj->t_accel + traj->t_flat)) {
        /* Constant velocity. */
        float dt = t - traj->t_accel;
        traj->accel = 0.0f;
        traj->velocity = traj->v_peak;
        pos = (0.5f * traj->v_peak * traj->t_accel) + (traj->v_peak * dt);
    } else {
        /* Decelerating. */
        float dt = t - traj->t_accel - traj->t_flat;
        float remaining = traj->t_accel - dt;
        traj->accel = -traj->max_accel;
        traj->velocity = traj->max_accel * remaining;
        pos = d - (0.5f * traj->max_accel * remaining * remaining);
    }

    traj->value = traj->start + (sign * pos);
}

/**
 * @brief Evaluate the S-curve profile by numerically integrating a
 *        jerk-limited acceleration.
 *
 * Rather than deriving a closed-form seven-segment S-curve (which is a page of
 * algebra and, more importantly, a page of algebra that has to be re-derived
 * whenever a limit changes), the acceleration is shaped online: it ramps
 * towards the value the trapezoid would use, bounded by the jerk limit, and is
 * then clamped so that the remaining distance can still be stopped.  The result
 * is a profile with continuous acceleration and a bounded jerk, which is what
 * the bud needs, at the cost of a slightly longer move.
 *
 * @param[in,out] traj  Generator.
 * @param[in]     dt_s  Step.
 * @param[in]     d     Absolute distance.
 * @param[in]     sign  Direction.
 */
static void traj_eval_scurve(tp_trajectory_t *traj, float dt_s, float d,
                             float sign)
{
    float remaining;
    float stop_distance;
    float accel_cmd;

    remaining = TP_FABS(traj->target - traj->value);
    if (remaining <= 1.0e-4f) {
        traj->value = traj->target;
        traj->velocity = 0.0f;
        traj->accel = 0.0f;
        traj->active = 0;
        return;
    }

    /* Distance needed to stop from the current velocity at the limit. */
    stop_distance = (traj->velocity * traj->velocity) / (2.0f * traj->max_accel);

    if (stop_distance >= remaining) {
        /* Braking is now mandatory: command full deceleration. */
        accel_cmd = -traj->max_accel;
    } else if (traj->velocity >= traj->max_velocity) {
        accel_cmd = 0.0f;
    } else {
        accel_cmd = traj->max_accel;
    }

    /* Ramp the acceleration towards the command at the jerk limit: this is the
     * step that turns a trapezoid into an S-curve. */
    {
        float max_step = traj->jerk * dt_s;
        if ((accel_cmd - traj->accel) > max_step) {
            traj->accel += max_step;
        } else if ((traj->accel - accel_cmd) > max_step) {
            traj->accel -= max_step;
        } else {
            traj->accel = accel_cmd;
        }
    }

    traj->velocity += traj->accel * dt_s;
    traj->velocity = traj_clampf(traj->velocity, 0.0f, traj->max_velocity);
    traj->value += sign * traj->velocity * dt_s;

    /* Overshoot guard: with a discrete step the profile can cross the target
     * between samples.  Clamping here keeps the reference monotone, which the
     * position loop depends on. */
    if (((sign > 0.0f) && (traj->value > traj->target)) ||
        ((sign < 0.0f) && (traj->value < traj->target))) {
        traj->value = traj->target;
        traj->velocity = 0.0f;
        traj->accel = 0.0f;
        traj->active = 0;
    }
    traj->duration_s = (d > 0.0f) ? (d / ((traj->max_velocity > 0.0f)
                                        ? traj->max_velocity : 1.0f)) : 0.0f;
}

float tp_trajectory_step(tp_trajectory_t *traj, float dt_s)
{
    float d;
    float sign;

    if ((traj == NULL) || (traj->initialised == 0)) {
        return 0.0f;
    }
    if (!TP_ISFINITE(dt_s) || (dt_s <= 0.0f)) {
        return traj->value;
    }
    if (traj->active == 0) {
        return traj->value;
    }

    traj->elapsed_s += dt_s;
    d = TP_FABS(traj->target - traj->start);
    sign = (traj->target >= traj->start) ? 1.0f : -1.0f;

    switch (traj->profile) {
    case TP_PROFILE_STEP:
        traj->value = traj->target;
        traj->velocity = 0.0f;
        traj->accel = 0.0f;
        traj->active = 0;
        break;
    case TP_PROFILE_TRAPEZOID:
        traj_eval_trapezoid(traj, traj->elapsed_s, d, sign);
        break;
    case TP_PROFILE_SCURVE:
        traj_eval_scurve(traj, dt_s, d, sign);
        break;
    default:
        traj->active = 0;
        break;
    }
    return traj->value;
}

float tp_trajectory_value(const tp_trajectory_t *traj)
{
    return (traj != NULL) ? traj->value : 0.0f;
}

float tp_trajectory_velocity(const tp_trajectory_t *traj)
{
    return (traj != NULL) ? traj->velocity : 0.0f;
}

float tp_trajectory_accel(const tp_trajectory_t *traj)
{
    return (traj != NULL) ? traj->accel : 0.0f;
}

int tp_trajectory_is_active(const tp_trajectory_t *traj)
{
    return (traj != NULL) ? traj->active : 0;
}

float tp_trajectory_progress(const tp_trajectory_t *traj)
{
    if (traj == NULL) {
        return 1.0f;
    }
    if (traj->duration_s <= 0.0f) {
        return 1.0f;
    }
    if (traj->elapsed_s >= traj->duration_s) {
        return 1.0f;
    }
    return traj->elapsed_s / traj->duration_s;
}

float tp_trajectory_abort(tp_trajectory_t *traj)
{
    if (traj == NULL) {
        return 0.0f;
    }
    traj->active = 0;
    traj->velocity = 0.0f;
    traj->accel = 0.0f;
    /* Leave `value` where it is and adopt it as the new origin, so a subsequent
     * move is planned from the true position rather than from the abandoned
     * target. */
    traj->start = traj->value;
    traj->target = traj->value;
    traj->elapsed_s = 0.0f;
    traj->duration_s = 0.0f;
    return traj->value;
}

float tp_trajectory_duration(const tp_trajectory_t *traj)
{
    return (traj != NULL) ? traj->duration_s : 0.0f;
}
