/**
 * @file    trajectory.h
 * @brief   Reference profiles: step, trapezoidal and S-curve.
 *
 * @details
 * # Why a trajectory generator at all
 * If the position loop is handed a step reference it will demand the maximum
 * velocity the instant the command arrives.  That is not a controller tuning
 * problem, it is a badly posed problem: the reference itself is not physically
 * achievable.  Three consequences, in increasing order of severity:
 *
 *   1. The velocity loop saturates, the position loop's integrator winds up, and
 *      the joint overshoots.  The overshoot is blamed on the gains, so the gains
 *      get detuned until the overshoot stops — and now the machine is slow.
 *   2. The current limit is hit on every move, so the effective acceleration is
 *      whatever the drive can deliver, i.e. undefined.  Every "we measured the
 *      settling time" number is then unrepeatable.
 *   3. On a machine with a flexible structure or a grasped object, the
 *      acceleration step excites a resonance.  On this robot the grasped object
 *      *is the bud*, so a step reference is a mechanism for dropping it.
 *
 * A trajectory generator replaces the step with a reference that respects the
 * machine's velocity and acceleration limits, which makes the closed-loop
 * response reproducible and the measurements in `docs/12_results_analysis.md`
 * meaningful.
 *
 * # The three profiles
 *
 * | Profile | Acceleration | Use |
 * |---|---|---|
 * | `STEP` | unbounded | Experiment 01 only.  Kept because a step response is the standard way to characterise a loop, and being explicit that it is a *test* input rather than an operating mode is part of the design. |
 * | `TRAPEZOID` | rectangular, discontinuous | The default for picking.  Within the jerk budget of a geared axis and the cheapest to compute. |
 * | `S_CURVE` | trapezoidal, continuous | For the final approach, where a jerk discontinuity would shake the bud.  Costs four extra multiplies per sample. |
 *
 * # Why not just low-pass the step
 * A low-pass makes the reference *smooth* but not *bounded in acceleration*:
 * the acceleration is still whatever the filter's second derivative produces,
 * and it changes with the filter's time constant rather than with a physical
 * limit.  A profile generator expresses the constraint in the units the machine
 * actually has (mrad/s and mrad/s2), so the numbers in `config.h` can be checked
 * against a datasheet.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_TRAJECTORY_H
#define TP_TRAJECTORY_H

#include <stdint.h>

#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Available reference profile shapes. */
typedef enum {
    TP_PROFILE_STEP = 0,     /**< Instantaneous jump; test use only.          */
    TP_PROFILE_TRAPEZOID = 1,/**< Velocity-limited, acceleration-limited.     */
    TP_PROFILE_SCURVE = 2    /**< Additionally jerk-limited.                  */
} tp_profile_t;

/**
 * @brief One profile generator.
 *
 * The generator is a pure function of time: given a start, an end, a duration
 * and the limits, it produces the reference for the current instant.  It holds
 * no integrator, so a pause or a rate change cannot corrupt it — unlike an
 * integrating "ramp" implementation.
 */
typedef struct {
    tp_profile_t profile;   /**< Shape selector.                             */
    float start;            /**< Reference value at t = 0.                   */
    float target;           /**< Reference value at t = duration.            */
    float max_velocity;     /**< Limit, absolute value (units/s).            */
    float max_accel;        /**< Limit, absolute value (units/s2).           */
    float jerk;             /**< S-curve jerk limit (units/s3).              */
    float duration_s;       /**< Total move duration.                        */
    float elapsed_s;        /**< Time since the move started.                */
    float value;            /**< Latest generated reference.                 */
    float velocity;         /**< Latest generated velocity reference.        */
    float accel;            /**< Latest generated acceleration.              */
    int   active;           /**< Non-zero while the move is in progress.     */
    int   initialised;      /**< Non-zero after a successful setup.          */

    /* --- trapezoid bookkeeping ---------------------------------------- */
    float t_accel;          /**< Acceleration phase duration.                */
    float t_flat;           /**< Constant-velocity phase duration.           */
    float v_peak;           /**< Achieved peak velocity.                     */
    int   triangular;       /**< Non-zero when the move never reaches v_max. */
} tp_trajectory_t;

/**
 * @brief Initialise a generator in the idle state at @p start_value.
 *
 * @param[in,out] traj         Instance.
 * @param[in]     profile      Shape to use.
 * @param[in]     start_value  Reference before any move begins.
 * @param[in]     dt_s         Control period; used only to compute durations.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_RANGE.
 */
tp_status_t tp_trajectory_init(tp_trajectory_t *traj, tp_profile_t profile,
                               float start_value, float dt_s);

/**
 * @brief Set the kinematic limits.
 *
 * @param[in,out] traj          Instance.
 * @param[in]     max_velocity  Absolute velocity limit, units/s.
 * @param[in]     max_accel     Absolute acceleration limit, units/s2.
 * @param[in]     jerk          Jerk limit for S-curve moves, units/s3; ignored
 *                              by the other profiles.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_RANGE.
 */
tp_status_t tp_trajectory_set_limits(tp_trajectory_t *traj, float max_velocity,
                                     float max_accel, float jerk);

/**
 * @brief Begin a move to @p target.
 *
 * The duration is derived from the limits, not requested by the caller — a
 * caller-specified duration would let an operator ask for a move the machine
 * cannot make, and the resulting saturation is exactly what the generator
 * exists to prevent.
 *
 * @param[in,out] traj    Instance.
 * @param[in]     target  Destination value.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_RANGE.
 */
tp_status_t tp_trajectory_start(tp_trajectory_t *traj, float target);

/**
 * @brief Advance the profile by @p dt_s.
 *
 * @param[in,out] traj  Instance.
 * @param[in]     dt_s  Elapsed seconds.
 * @return The new reference value.  Once the move completes, the generator
 *         holds at @ref tp_trajectory_t::target and reports itself idle.
 */
float tp_trajectory_step(tp_trajectory_t *traj, float dt_s);

/**
 * @brief Latest reference value.
 *
 * @param[in] traj  Instance.
 * @return Reference value.
 */
float tp_trajectory_value(const tp_trajectory_t *traj);

/**
 * @brief Latest reference velocity, used as the cascade's feed-forward term.
 *
 * This is the single most valuable output of the generator: feeding it forward
 * means the position loop does not have to *invent* the velocity from the
 * error, so it runs with a much smaller effective gain and far less overshoot.
 *
 * @param[in] traj  Instance.
 * @return Velocity reference.
 */
float tp_trajectory_velocity(const tp_trajectory_t *traj);

/**
 * @brief Latest reference acceleration.
 *
 * @param[in] traj  Instance.
 * @return Acceleration reference.
 */
float tp_trajectory_accel(const tp_trajectory_t *traj);

/**
 * @brief Whether a move is in progress.
 *
 * @param[in] traj  Instance.
 * @return Non-zero while moving.
 */
int tp_trajectory_is_active(const tp_trajectory_t *traj);

/**
 * @brief Fraction of the move completed, 0..1.
 *
 * @param[in] traj  Instance.
 * @return Progress fraction; 1.0 when idle.
 */
float tp_trajectory_progress(const tp_trajectory_t *traj);

/**
 * @brief Abort the move, holding the current value as the new start.
 *
 * The generator does **not** snap to the target: aborting a move leaves the
 * robot wherever it is, which is the correct behaviour for a stop command.
 *
 * @param[in,out] traj  Instance.
 * @return The value held at the moment of the abort.
 */
float tp_trajectory_abort(tp_trajectory_t *traj);

/**
 * @brief Duration the generator computed for the current move.
 *
 * Documented and exposed because the picking cycle's time budget is derived
 * from it: `docs/06_force_control.md` uses it to show that the approach phase
 * dominates the cycle time.
 *
 * @param[in] traj  Instance.
 * @return Duration in seconds.
 */
float tp_trajectory_duration(const tp_trajectory_t *traj);

#ifdef __cplusplus
}
#endif

#endif /* TP_TRAJECTORY_H */
