/**
 * @file    safety_monitor.h
 * @brief   Continuous supervision of every hazard the robot can detect.
 *
 * @details
 * # What it is
 * One function that runs at @ref TP_SAFETY_FREQ_HZ and asks eight independent
 * questions about the machine's state.  It owns **no actuation**: it raises
 * faults in the registry and stops the drives through the motor abstraction,
 * and the application decides what to do about the fault.  Keeping detection
 * and reaction apart means the detection logic can be unit-tested by asserting
 * on the registry, with no robot and no timing.
 *
 * # The eight checks, and why each one needs debouncing
 *
 * | Check | Threshold | Debounce | Why debouncing is required |
 * |---|---|---|---|
 * | Over-force | `TP_FORCE_MAX_N` | **none** | A crushing bud is not a transient. The check must fire in the same period it is detected. |
 * | Soft force | `TP_FORCE_SOFT_LIMIT_N` | none | A warning only; firing early is harmless. |
 * | Motor stall | `TP_STALL_SPEED_THRESHOLD_MRAD_S`, `TP_STALL_PWM_THRESHOLD` | `TP_STALL_TRIP_MS` | Every motor is briefly below the speed threshold while *starting*. Without debouncing, every move trips a stall fault. |
 * | Over-current | `TP_MAX_CURRENT_MA` | `TP_OVERCURRENT_TRIP_MS` | The current legitimately spikes for a few milliseconds at the start of an acceleration ramp. |
 * | Encoder fault | `TP_ENCODER_MAX_ACCEL_MRAD_S2` | immediate | An impossible acceleration means the *feedback* is wrong; there is nothing to wait for. |
 * | Sensor fault | `TP_SENSOR_ADC_*_COUNTS`, spike streak | `TP_SENSOR_FAULT_DEBOUNCE` | A railed reading could be a single bad conversion. |
 * | Limit switch | asserted + commanded into it | 3 samples (in the driver) | Mechanical contact bounce. |
 * | Communication timeout | `TP_COMM_TIMEOUT_MS` | none | The timeout *is* the debounce. |
 * | E-stop | NC loop low | none | Physical safety input; must never be filtered. |
 * | Watchdog | reset cause | boot only | Read once at start-up. |
 *
 * The asymmetry is the point: **checks that protect the machine from itself are
 * debounced; checks that protect the object and the operator are not.**  A
 * nuisance trip on a stall is an annoyance; a delayed trip on an over-force is
 * a destroyed bud.
 *
 * # Why stall detection needs the PWM term
 * "Speed is zero" is not a stall: a motor that is commanded to zero speed and
 * is at zero speed is simply stopped.  A stall is "torque is being demanded and
 * nothing is moving".  Requiring the commanded duty to exceed
 * `TP_STALL_PWM_THRESHOLD` before arming the check is what separates the two,
 * and it is the single most common mistake in a home-grown stall detector.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_SAFETY_MONITOR_H
#define TP_SAFETY_MONITOR_H

#include <stdint.h>

#include "encoder.h"
#include "fault_manager.h"
#include "force_sensor.h"
#include "gripper.h"
#include "limit_switch.h"
#include "motor.h"
#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Everything the monitor needs to see.
 *
 * Bundled into one struct rather than passed as nine arguments so that adding a
 * check does not change every call site, and so a test can build a whole
 * scenario by filling in a struct literal.
 */
typedef struct {
    tp_encoder_t      *joint_encoder;   /**< Arm joint feedback.             */
    tp_motor_t        *joint_motor;     /**< Arm joint drive.                */
    tp_gripper_t      *gripper;         /**< Gripper (motor + encoder inside).*/
    tp_force_sensor_t *force_sensor;    /**< Load-cell channel.              */
    tp_limit_switch_t *estop;           /**< E-stop input.                   */
    tp_limit_switch_t *limit_min;       /**< Arm negative limit.             */
    tp_limit_switch_t *limit_max;       /**< Arm positive limit.             */
    uint32_t           last_comm_ms;    /**< Timestamp of the last valid frame.*/
    int                comm_seen;       /**< Non-zero once any frame arrived.*/
} tp_safety_inputs_t;

/**
 * @brief Internal debounce and diagnostic state of the monitor.
 */
typedef struct {
    uint32_t stall_ms;          /**< Time the stall condition has held.      */
    uint32_t overcurrent_ms;    /**< Time over-current has held.             */
    uint32_t sensor_bad_count;  /**< Consecutive implausible sensor samples. */
    uint32_t encoder_bad_count; /**< Consecutive impossible-acceleration samples.*/
    float    last_speed_mrad_s; /**< Previous speed, for the acceleration check.*/
    uint32_t last_check_ms;     /**< Timestamp of the previous run.          */
    float    max_speed_seen;    /**< Peak speed since reset, for the report. */
    float    max_current_seen;  /**< Peak current since reset.               */
    float    max_force_seen;    /**< Peak force since reset.                 */

    uint32_t checks_run;        /**< Number of @ref tp_safety_monitor_update. */
    uint32_t fault_raises;      /**< Faults raised by this module.          */
    uint32_t estop_events;      /**< Times the E-stop was seen asserted.    */
    uint32_t limit_events;      /**< Times a limit switch vetoed motion.    */
    int      initialised;       /**< Non-zero after a successful init.      */
} tp_safety_state_t;

/**
 * @brief Initialise the monitor's debounce state.
 *
 * @param[in,out] st  State block.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM.
 */
tp_status_t tp_safety_monitor_init(tp_safety_state_t *st);

/**
 * @brief Run every check once.
 *
 * @param[in,out] st      Monitor state.
 * @param[in]     in      Inputs to inspect.
 * @param[in]     fm      Fault registry to raise into.
 * @param[in]     now_ms  Current timestamp.
 * @return Number of faults raised by this call.  A non-zero return means the
 *         caller should re-evaluate whether motion may continue.
 */
uint32_t tp_safety_monitor_update(tp_safety_state_t *st,
                                  const tp_safety_inputs_t *in,
                                  tp_fault_manager_t *fm, uint32_t now_ms);

/**
 * @brief Check the E-stop input alone.
 *
 * Separated because the E-stop is handled on the fastest path available — the
 * safety task calls this first, before anything else, and a positive result
 * short-circuits the rest of the update.
 *
 * @param[in,out] st      Monitor state.
 * @param[in]     in      Inputs.
 * @param[in]     fm      Fault registry.
 * @param[in]     now_ms  Current timestamp.
 * @return Non-zero when the E-stop is asserted.
 */
int tp_safety_check_estop(tp_safety_state_t *st, const tp_safety_inputs_t *in,
                          tp_fault_manager_t *fm, uint32_t now_ms);

/**
 * @brief Reset the peak-value statistics and the debounce timers.
 *
 * @param[in,out] st  Monitor state.
 */
void tp_safety_monitor_reset_stats(tp_safety_state_t *st);

/**
 * @brief Whether the arm joint is currently considered stalled.
 *
 * Exposed so the application can log the raw condition as well as the fault.
 *
 * @param[in] st  Monitor state.
 * @return Non-zero when the stall condition currently holds.
 */
int tp_safety_is_stalled(const tp_safety_state_t *st);

#ifdef __cplusplus
}
#endif

#endif /* TP_SAFETY_MONITOR_H */
