/**
 * @file    cascade.h
 * @brief   Cascaded position -> velocity control of the arm joint.
 *
 * @details
 * # The control structure
 * @verbatim
 *                        +-----------+   v_ref    +-----------+   duty
 *   theta_ref -->(+)---->| Position  |----------->| Velocity  |--------> Motor
 *                ^  -   |   PID     |      ^  -  |   PID     |            |
 *                |      +-----------+      |     +-----------+            |
 *                |           ^             |                              |
 *                |           | v_ff        |                              v
 *                |      +----+----+        |                          +-------+
 *                |      |Trajectory|       |                          |Encoder|
 *                |      +---------+        |                          +-------+
 *                |                         |                              |
 *                +-------------------------+------------------------------+
 *                                theta, omega
 * @endverbatim
 *
 * # Why two loops instead of one PID on position
 * This is the question an interviewer asks first, and the honest answer has
 * three parts.
 *
 * **1. A single position PID has to do two incompatible jobs.**  Its output
 * must be a *velocity demand*, so its gain sets both how fast the joint
 * responds and how much damping it has.  Those pull in opposite directions:
 * raising the gain for a faster response reduces the phase margin until the
 * joint oscillates.  Splitting the problem lets the inner loop set the damping
 * (which it owns, because it directly commands the torque-producing quantity)
 * and the outer loop set the response speed.
 *
 * **2. The inner loop linearises the plant.**  A real joint's torque constant
 * varies with current and temperature, the supply sags under load, and friction
 * is nonlinear near zero speed.  All of those live *inside* the velocity loop,
 * so closing it around the encoder makes the plant, as seen by the position
 * loop, close to a pure integrator (`theta' = v_cmd`).  The position loop can
 * then be tuned with a single first-order model instead of a lookup table.
 *
 * **3. It rejects disturbance torque where it is generated.**  Gravity and
 * cutting reaction act as a torque; the velocity loop sees them as a velocity
 * error and corrects within its own bandwidth, *without* the position
 * changing.  In a single loop the disturbance must first move the joint before
 * the controller notices, so the position error is unavoidable.
 *
 * **When NOT to cascade.**  If the inner loop cannot be made much faster than
 * the outer one —say within a factor of three —the cascade is worse than a
 * single loop, because the extra lag adds phase loss for no benefit.  Here the
 * velocity loop closes at roughly 8 Hz and the position loop at about
 * 0.55 Hz, a separation of ~15x, which is comfortable.  `docs/05_control_algorithm.md`
 * derives both numbers.
 *
 * # Bandwidth separation is a design constraint, not a tuning outcome
 * The ratio is chosen first (10-20x), the inner loop is tuned to meet it, and
 * only then is the outer loop tuned.  Tuning them independently and hoping is
 * how a cascade ends up worse than a single loop.
 *
 * # Feed-forward
 * The inner loop receives the trajectory's velocity, and the outer loop's error
 * is therefore only responsible for *correction*, not for *creating* the
 * motion.  With `Kff = 1` and a perfect inner loop the position error during a
 * constant-velocity segment is zero without any integral action at all.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_CASCADE_H
#define TP_CASCADE_H

#include <stdint.h>

#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "status.h"
#include "trajectory.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Which loop the mode selector refers to. */
typedef enum {
    TP_CASCADE_MODE_OFF = 0,     /**< Open loop: output forced to zero.      */
    TP_CASCADE_MODE_VELOCITY = 1,/**< Inner loop only; position loop bypassed.*/
    TP_CASCADE_MODE_POSITION = 2 /**< Full cascade.                          */
} tp_cascade_mode_t;

/**
 * @name Loop selectors for @ref tp_cascade_set_gains / @ref tp_cascade_get_gains
 *
 * The values deliberately match the wire encoding used by the `SET_PID` and
 * `PID_RPT` commands (see `protocol_types.h`), so the application's mapping
 * between a loop_id from the link and a loop here is the identity function and
 * cannot be got wrong.
 *
 * They are re-declared rather than reused so that the Control layer does not
 * have to include a protocol header —the dependency would point upward and
 * make `cascade.c` untestable without the protocol module.
 * @{ */
#define TP_CASCADE_LOOP_POSITION 0u /**< Outer position loop.                 */
#define TP_CASCADE_LOOP_VELOCITY 1u /**< Inner velocity loop.                 */
/** @} */

/**
 * @brief Cascade controller state.
 */
typedef struct {
    tp_pid_t position_pid;  /**< Outer loop: mrad error -> mrad/s demand.    */
    tp_pid_t velocity_pid;  /**< Inner loop: mrad/s error -> per-mille duty.  */

    tp_trajectory_t trajectory; /**< Reference generator, owned by the cascade
                                 *   so its feed-forward velocity and the
                                 *   position reference can never disagree.   */

    tp_cascade_mode_t mode; /**< Active control mode.                       */

    float position_ref;     /**< Reference angle, mrad.                     */
    float position_fb;      /**< Feedback angle, mrad.                      */
    float velocity_ref;     /**< Velocity demand, mrad/s.                   */
    float velocity_fb;      /**< Feedback velocity, mrad/s.                 */
    float velocity_ff;      /**< Feed-forward velocity, mrad/s.             */
    float duty_cmd;         /**< Final duty demand, per-mille.              */

    float position_error;   /**< Latest position error, mrad.               */
    float velocity_error;   /**< Latest velocity error, mrad/s.             */

    tp_pid_terms_t pos_terms;/**< Term breakdown of the outer loop.         */
    tp_pid_terms_t vel_terms;/**< Term breakdown of the inner loop.         */

    /* --- diagnostics, all of them used by the experiments ----------------- */
    uint32_t update_count;      /**< Control periods executed.              */
    uint32_t vel_saturated_count;/**< Periods with the inner loop clamped.  */
    uint32_t pos_saturated_count;/**< Periods with the outer loop clamped.  */
    uint32_t tracking_error_count;/**< Periods where |e_pos| exceeded 5 deg. */
    float    max_position_error;/**< Worst |position error| since reset.    */
    /**
     * @brief Running **mean square** of the position error, mrad^2.
     *
     * The running mean of the squares is what can be accumulated in O(1) with
     * respect to the sample count; the RMS is its square root and is *not* that
     * quantity.  The field used to be called `rms_position_error` and was
     * documented as "running RMS" while holding the mean square, so every reader
     * - and any report that quoted it - was out by a square root.  The name now
     * says what is stored, and @ref tp_cascade_rms_position_error is the accessor
     * that takes the root.
     */
    float    mean_sq_position_error;

    int      initialised;   /**< Non-zero after a successful init.          */
} tp_cascade_t;

/**
 * @brief Initialise the cascade with the default gains from `config.h`.
 *
 * The inner loop is initialised *first* and its output limits are taken from
 * the duty range; the outer loop's output limit is the joint's maximum speed.
 * Those two assignments encode the design intent (outer loop's output *is* the
 * inner loop's input range) in one place.
 *
 * @param[in,out] c     Instance.
 * @param[in]     dt_s  Control period in seconds.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM / @ref TP_ERR_RANGE.
 */
tp_status_t tp_cascade_init(tp_cascade_t *c, float dt_s);

/**
 * @brief Change the gains of one loop at run time.
 *
 * Used by the `SET_PID` command and by Experiment 02's parameter sweep.
 *
 * @param[in,out] c     Instance.
 * @param[in]     loop  Which loop.
 * @param[in]     kp    Proportional gain.
 * @param[in]     ki    Integral gain.
 * @param[in]     kd    Derivative gain.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM / @ref TP_ERR_RANGE.
 */
tp_status_t tp_cascade_set_gains(tp_cascade_t *c, uint8_t loop, float kp,
                                 float ki, float kd);

/**
 * @brief Read the gains of one loop.
 *
 * @param[in]  c     Instance.
 * @param[in]  loop  Which loop.
 * @param[out] kp    Receives the proportional gain.
 * @param[out] ki    Receives the integral gain.
 * @param[out] kd    Receives the derivative gain.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM.
 */
tp_status_t tp_cascade_get_gains(const tp_cascade_t *c, uint8_t loop, float *kp,
                                 float *ki, float *kd);

/**
 * @brief Select the control mode.
 *
 * Switching modes resets **both** integrators.  A stale integral from position
 * mode would otherwise appear as an unexplained bias the moment velocity mode
 * is entered; that is precisely the kind of bug that "goes away when you reset
 * the board".
 *
 * @param[in,out] c     Instance.
 * @param[in]     mode  New mode.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_cascade_set_mode(tp_cascade_t *c, tp_cascade_mode_t mode);

/**
 * @brief Set the position reference for the outer loop.
 *
 * @param[in,out] c      Instance.
 * @param[in]     mrad   Reference angle in milliradians.
 * @param[in]     v_ff   Feed-forward velocity in milliradians per second.
 * @return @ref TP_OK or @ref TP_ERR_PARAM / @ref TP_ERR_RANGE.
 */
tp_status_t tp_cascade_set_reference(tp_cascade_t *c, float mrad, float v_ff);

/**
 * @brief Set the velocity reference for the inner loop (velocity mode).
 *
 * @param[in,out] c      Instance.
 * @param[in]     mrad_s Velocity reference.
 * @return @ref TP_OK or @ref TP_ERR_PARAM / @ref TP_ERR_RANGE.
 */
tp_status_t tp_cascade_set_velocity_reference(tp_cascade_t *c, float mrad_s);

/**
 * @brief Run one control period.
 *
 * Order of operations, which matters:
 *   1. sample the encoder,
 *   2. (position mode) outer PID on `ref - feedback`, output clamped to the
 *      joint speed limit, plus the feed-forward term,
 *   3. inner PID on `v_ref - v_fb`, output in per-mille,
 *   4. return the duty for the motor module to apply.
 *
 * The feed-forward is added to the **inner loop's reference**, not to its
 * output.  Adding it to the output would bypass the velocity limiter; adding it
 * to the reference means the inner loop still enforces the speed limit and the
 * feed-forward is therefore always achievable —a subtle but important detail.
 *
 * @param[in,out] c        Instance.
 * @param[in]     enc      Joint encoder, already sampled or sampled here.
 * @param[in]     dt_s     Elapsed seconds.
 * @return Duty demand in per-mille.
 */
float tp_cascade_update(tp_cascade_t *c, tp_encoder_t *enc, float dt_s);

/**
 * @brief Clear both integrators and the derivative memory.
 *
 * @param[in,out] c  Instance.
 */
void tp_cascade_reset(tp_cascade_t *c);

/**
 * @brief Clear the diagnostic counters and accumulated error statistics.
 *
 * @param[in,out] c  Instance.
 */
void tp_cascade_reset_stats(tp_cascade_t *c);

/**
 * @brief Latest position reference.
 *
 * @param[in] c  Instance.
 * @return Reference in milliradians.
 */
float tp_cascade_position_ref(const tp_cascade_t *c);

/**
 * @brief Latest position error.
 *
 * @param[in] c  Instance.
 * @return Error in milliradians.
 */
float tp_cascade_position_error(const tp_cascade_t *c);

/**
 * @brief Root-mean-square position error since the last statistics reset.
 *
 * The quantity a settling analysis actually wants.  It is computed on demand
 * from the running mean square rather than accumulated directly, because a
 * running RMS cannot be updated in constant time.
 *
 * @param[in] c  Instance.
 * @return RMS error in milliradians; 0 when no samples have been taken.
 */
float tp_cascade_rms_position_error(const tp_cascade_t *c);

/**
 * @brief Latest duty demand.
 *
 * @param[in] c  Instance.
 * @return Duty in per-mille.
 */
float tp_cascade_duty(const tp_cascade_t *c);

/**
 * @brief Select the reference profile.
 *
 * @param[in,out] c        Instance.
 * @param[in]     profile  Profile shape.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_cascade_set_profile(tp_cascade_t *c, tp_profile_t profile);

/**
 * @brief Access the embedded trajectory generator.
 *
 * The application drives the generator (start a move, abort a move) while the
 * cascade consumes its output each period.  Exposing it avoids duplicating a
 * copy of the generator state in the application, which would be a
 * synchronisation bug waiting to happen.
 *
 * @param[in] c  Instance.
 * @return Pointer to the generator, or NULL.
 */
tp_trajectory_t *tp_cascade_trajectory(tp_cascade_t *c);

/**
 * @brief Whether the cascade has detected a non-finite value.
 *
 * @param[in] c  Instance.
 * @return Non-zero when either loop has diverged.
 */
int tp_cascade_is_diverged(const tp_cascade_t *c);

#ifdef __cplusplus
}
#endif

#endif /* TP_CASCADE_H */
