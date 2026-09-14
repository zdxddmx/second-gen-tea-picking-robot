/**
 * @file    force_control.h
 * @brief   Flexible (force-controlled) gripping: the safety-critical core.
 *
 * @details
 * # The problem this file solves
 * A tea bud is a soft, thin, living object.  A gripper that closes to a fixed
 * *position* has no idea how hard it is squeezing: the force depends on the
 * stem's stiffness, which varies with cultivar, moisture, bud age and even the
 * time of day, and it is not something the robot can measure in advance.  Close
 * to a fixed gap on a stiffer-than-expected stem and the force rises along the
 * quadratic curve of @ref tp_plant_stem_gripper 閳?past the point where the bud
 * is crushed 閳?before the controller has any information at all.
 *
 * The fix is to close on **force**, not on position.
 *
 * # The approach sequence
 * @verbatim
 *   FAST_APPROACH  ---- 40 mm/s ---> until |gap - stem| < 4 mm
 *        |
 *        v
 *   SLOW_APPROACH  ----  6 mm/s ---> until F > 0.15 N
 *        |
 *        v
 *   CONTACT_DETECT ---- verify the force step is real (not noise)
 *        |
 *        v
 *   FORCE_CONTROL  <-- force PID --> jaw velocity, until |F - F*| < 0.10 N
 *        |                                       for 80 ms
 *        v
 *   PICK           ---- detach (pull + slight twist), holding force
 *        |
 *        v
 *   RELEASE        ---- open at 30 mm/s, back off 12 mm
 * @endverbatim
 *
 * # Why each stage exists
 *
 * | Stage | Without it |
 * |---|---|
 * | FAST_APPROACH | Closing at 6 mm/s from 40 mm out takes 5.7 s per bud; at a 1 Hz cycle target that is the whole budget. |
 * | SLOW_APPROACH | Approaching at 40 mm/s and switching to force control on contact gives the force loop only ~0.2 mm of travel to react; the impact alone exceeds the damage threshold. |
 * | CONTACT_DETECT | A single noisy sample above the threshold would hand control to the force loop with nothing in the jaws. The detector requires the force to *stay* above the threshold for a debounce period. |
 * | FORCE_CONTROL | This is the whole point: it regulates the quantity that causes damage. |
 * | PICK | Closing the jaws is not the same as detaching the bud; the stem must be pulled or twisted free while the grip is maintained. |
 * | RELEASE | Dropping the bud requires opening *and* retreating; opening alone leaves it resting on the jaws and the next cycle knocks it off somewhere random. |
 *
 * # Protection, in order of priority
 * 1. `F > F_max (5.0 N)` -> **immediate** open, latch `ERR_FORCE_OVERLOAD`, no
 *    further closing until cleared.  Checked *before* the controller runs, so a
 *    fault takes effect in the same control period it is detected in.
 * 2. `F > F_soft (3.5 N)` -> warning, and the force setpoint is derated so the
 *    loop backs off instead of fighting the limit.
 * 3. Sensor fault -> force control is impossible; the controller refuses to
 *    close on force and the application falls back to position-only gripping.
 * 4. A **contact timeout**: if no contact is detected within a bounded time, the
 *    bud is not where the vision system said it was.  Stop and report, rather
 *    than closing until the jaws hit their hard stop.
 *
 * # Adaptive threshold
 * With @ref TP_FORCE_ADAPT_ENABLE the controller tracks an exponential moving
 * average of the force actually needed to hold a bud and adds a margin.  The
 * rationale is honest and worth stating plainly: **the fixed 2.2 N target is a
 * design starting point, not a measured optimum.**  Adaptation is shipped
 * disabled by default precisely because it has not been validated against real
 * buds, and enabling it blindly would be overclaiming.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_FORCE_CONTROL_H
#define TP_FORCE_CONTROL_H

#include <stdint.h>

#include "filters.h"
#include "force_sensor.h"
#include "fault_codes.h"
#include "gripper.h"
#include "pid.h"
#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Sub-states of the flexible gripping sequence. */
typedef enum {
    TP_GRIP_IDLE = 0,          /**< Not gripping; jaws free.                 */
    TP_GRIP_FAST_APPROACH = 1, /**< Closing quickly, pre-contact.            */
    TP_GRIP_SLOW_APPROACH = 2, /**< Closing slowly, watching for contact.    */
    TP_GRIP_CONTACT_DETECT = 3,/**< Confirming the contact is real.          */
    TP_GRIP_FORCE_CONTROL = 4, /**< Regulating the grip force.               */
    TP_GRIP_PICK = 5,          /**< Holding force while detaching.           */
    TP_GRIP_RELEASE = 6,       /**< Opening and retreating.                  */
    TP_GRIP_FAULT = 7,         /**< Protection tripped; jaws held open.      */
    TP_GRIP_STATE_COUNT = 8
} tp_grip_state_t;

/**
 * @brief Flexible gripping controller.
 */
typedef struct {
    tp_pid_t force_pid;        /**< Force loop: N error -> mm/s jaw velocity. */
    tp_grip_state_t state;     /**< Current sub-state.                        */

    float force_target_n;      /**< Active setpoint, N.                       */
    float force_target_base_n; /**< Operator/design setpoint before adaptation.*/
    float force_feedback_n;    /**< Latest filtered force.                    */
    float force_error_n;       /**< Setpoint minus feedback.                  */
    float velocity_cmd_mm_s;   /**< Jaw velocity command issued this period.  */
    tp_pid_terms_t force_terms;/**< Force-loop term breakdown, for plotting.  */

    float approach_target_mm;  /**< Gap at which contact is expected.         */
    float contact_force_n;     /**< Force at which contact was declared.      */
    float peak_force_n;        /**< Maximum force seen this cycle.            */
    float hold_force_avg_n;    /**< Mean force during the hold phase.         */

    uint32_t state_entered_ms; /**< When the current sub-state began.         */
    uint32_t cycle_start_ms;   /**< When the current pick cycle began.        */
    uint32_t settle_start_ms;  /**< When the force first entered the band.    */
    uint32_t cycle_count;      /**< Completed cycles.                         */
    uint32_t fault_count;      /**< Times protection tripped.                 */
    uint32_t contact_timeouts; /**< Cycles aborted because no contact.        */
    uint32_t overforce_events; /**< Times @ref TP_FORCE_MAX_N was exceeded.   */
    uint32_t soft_limit_events;/**< Times @ref TP_FORCE_SOFT_LIMIT_N was hit. */

    /* --- adaptive threshold -------------------------------------------- */
    float    observed_hold_ema_n;/**< EMA of the force needed to hold a bud.  */
    int      adapt_enabled;    /**< Non-zero when adaptation is active.       */

    /* --- hold-phase statistics ----------------------------------------- *
     * Accumulated only once the force has actually entered the settling band,
     * and unconditionally during PICK.  Including the initial ramp from the
     * contact threshold up to the setpoint would drag the mean down and make a
     * correctly regulated 2.2 N grip report as 1.66 N 鈥?which is exactly what
     * the first revision of this file did.  The number has to answer "what
     * force did we hold the bud at?", not "what was the average over the whole
     * closure?". */
    int      regulated;        /**< Non-zero once the force entered band.     */
    uint32_t hold_samples;     /**< Samples in the regulated phase.           */
    float    hold_force_sum_n; /**< Sum, for the mean.                        */
    float    hold_force_min_n; /**< Minimum during the regulated phase.       */
    float    hold_force_max_n; /**< Maximum during the regulated phase.       */

    /* --- settling transient, for the settling-time and overshoot metrics -- */
    uint32_t settle_samples;   /**< Periods from contact to first in-band.    */
    float    settle_peak_n;    /**< Peak force during that transient.         */

    int      fault_latched;    /**< Non-zero after protection tripped.        */
    tp_fault_code_t fault_code;/**< Which fault the last trip represents, so
                                *   the application can report the right code
                                *   instead of guessing.  A contact timeout and
                                *   an over-force are very different events and
                                *   conflating them sends an operator to the
                                *   wrong subsystem.                          */
    int      initialised;      /**< Non-zero after a successful init.         */
} tp_force_control_t;

/**
 * @brief Initialise the flexible gripping controller.
 *
 * @param[in,out] fc    Instance.
 * @param[in]     dt_s  Control period in seconds; the force PID runs at this
 *                      rate.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_RANGE.
 */
tp_status_t tp_force_control_init(tp_force_control_t *fc, float dt_s);

/**
 * @brief Begin a gripping sequence.
 *
 * @param[in,out] fc           Instance.
 * @param[in]     grip         Gripper to drive.
 * @param[in]     approach_mm  Gap at which contact is expected, from the vision
 *                             system.  Used to decide when to slow down.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_FAULT_ACTIVE when a
 *         fault is latched (a faulted controller refuses to start).
 */
tp_status_t tp_force_control_begin(tp_force_control_t *fc, tp_gripper_t *grip,
                                   float approach_mm);

/**
 * @brief Run one control period of the sequence.
 *
 * @param[in,out] fc    Instance.
 * @param[in]     grip  Gripper to drive.
 * @param[in]     fs    Force sensor to read.
 * @param[in]     now_ms Current timestamp.
 * @param[in]     dt_s  Elapsed seconds.
 * @return @ref TP_OK on progress, @ref TP_ERR_STATE when the sequence is idle
 *         or has faulted, @ref TP_ERR_FAULT_ACTIVE when protection tripped
 *         during this period.
 */
tp_status_t tp_force_control_update(tp_force_control_t *fc, tp_gripper_t *grip,
                                    tp_force_sensor_t *fs, uint32_t now_ms,
                                    float dt_s);

/**
 * @brief Abort the sequence and open the jaws.
 *
 * Used by the stop path.  Opening rather than simply stopping is deliberate: a
 * robot that halts mid-grip with a bud held is not in a safe state, because the
 * bud is a living object that will be damaged by an indefinite hold.
 *
 * @param[in,out] fc    Instance.
 * @param[in]     grip  Gripper to open, or NULL to only reset the state.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_force_control_abort(tp_force_control_t *fc, tp_gripper_t *grip);

/**
 * @brief Change the grip force setpoint.
 *
 * @param[in,out] fc     Instance.
 * @param[in]     force_n New setpoint in newtons.
 * @return @ref TP_OK, or @ref TP_ERR_RANGE when outside
 *         [`TP_FORCE_TARGET_MIN_N`, `TP_FORCE_TARGET_MAX_N`].
 */
tp_status_t tp_force_control_set_force(tp_force_control_t *fc, float force_n);

/**
 * @brief Read the current grip force setpoint.
 *
 * @param[in] fc  Instance.
 * @return Setpoint in newtons.
 */
float tp_force_control_get_force_target(const tp_force_control_t *fc);

/**
 * @brief Change the force-loop gains at run time.
 *
 * @param[in,out] fc  Instance.
 * @param[in]     kp  Proportional gain.
 * @param[in]     ki  Integral gain.
 * @param[in]     kd  Derivative gain.
 * @return @ref TP_OK or @ref TP_ERR_PARAM / @ref TP_ERR_RANGE.
 */
tp_status_t tp_force_control_set_gains(tp_force_control_t *fc, float kp,
                                       float ki, float kd);

/**
 * @brief Read the force-loop gains.
 *
 * @param[in]  fc  Instance.
 * @param[out] kp  Receives the proportional gain.
 * @param[out] ki  Receives the integral gain.
 * @param[out] kd  Receives the derivative gain.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_force_control_get_gains(const tp_force_control_t *fc, float *kp,
                                       float *ki, float *kd);

/**
 * @brief Enable or disable the adaptive force threshold.
 *
 * @param[in,out] fc      Instance.
 * @param[in]     enable  Non-zero to enable.
 */
void tp_force_control_set_adaptive(tp_force_control_t *fc, int enable);

/**
 * @brief Current sub-state.
 *
 * @param[in] fc  Instance.
 * @return Sub-state.
 */
tp_grip_state_t tp_force_control_state(const tp_force_control_t *fc);

/**
 * @brief Symbolic name of a sub-state, for logs and CSV output.
 *
 * @param[in] state  Sub-state.
 * @return Static string such as `"FORCE_CONTROL"`.
 */
const char *tp_grip_state_name(tp_grip_state_t state);

/**
 * @brief Whether the sequence has completed normally.
 *
 * @param[in] fc  Instance.
 * @return Non-zero when the jaws are open and the cycle is done.
 */
int tp_force_control_is_complete(const tp_force_control_t *fc);

/**
 * @brief Whether protection has tripped.
 *
 * @param[in] fc  Instance.
 * @return Non-zero when latched.
 */
int tp_force_control_is_faulted(const tp_force_control_t *fc);

/**
 * @brief Clear the fault latch.
 *
 * Refuses while the measured force is still above the soft limit, so clearing
 * cannot be used to defeat the protection while the bud is still being crushed.
 *
 * @param[in,out] fc  Instance.
 * @return @ref TP_OK, or @ref TP_ERR_FAULT_ACTIVE while the force is high.
 */
tp_status_t tp_force_control_clear_fault(tp_force_control_t *fc);

/**
 * @brief Fault code describing why the sequence last tripped.
 *
 * @param[in] fc  Instance.
 * @return The fault code, or @ref TP_FAULT_NONE when no trip has occurred.
 */
tp_fault_code_t tp_force_control_fault_code(const tp_force_control_t *fc);

/**
 * @brief Mean force during the hold phase of the last cycle.
 *
 * This is the number Experiment 03 reports: the force actually maintained on
 * the bud, versus the peak force reached by a position-only gripper.
 *
 * @param[in] fc  Instance.
 * @return Mean force in newtons, or 0 if no cycle has completed.
 */
float tp_force_control_mean_hold_force(const tp_force_control_t *fc);

/**
 * @brief Force overshoot above the setpoint during the last closure.
 *
 * The difference between this and the peak force is the contact transient: the
 * peak includes the impact, the overshoot is how far the loop let the force go
 * past its target before regulating.  Experiment 03 reports the overshoot,
 * because bounding it is precisely what a position-only gripper cannot do.
 *
 * @param[in] fc  Instance.
 * @return Overshoot in newtons; 0 when the setpoint was never exceeded.
 */
float tp_force_control_overshoot(const tp_force_control_t *fc);

/**
 * @brief Number of control periods from contact to a settled force.
 *
 * @param[in] fc  Instance.
 * @return Sample count; 0 when no cycle has settled.
 */
uint32_t tp_force_control_settle_samples(const tp_force_control_t *fc);

/**
 * @brief Peak force seen during the last cycle.
 *
 * @param[in] fc  Instance.
 * @return Peak force in newtons.
 */
float tp_force_control_peak_force(const tp_force_control_t *fc);

/**
 * @brief Force error of the latest period.
 *
 * @param[in] fc  Instance.
 * @return Error in newtons.
 */
float tp_force_control_force_error(const tp_force_control_t *fc);

#ifdef __cplusplus
}
#endif

#endif /* TP_FORCE_CONTROL_H */
