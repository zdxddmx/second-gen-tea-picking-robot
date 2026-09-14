/**
 * @file    motor.h
 * @brief   Motor drive abstraction: duty, enable, current and protections.
 *
 * @details
 * # What it is
 * The only thing in the firmware allowed to command a motor.  It owns:
 *   - the **slew rate limit** on the duty command,
 *   - the **continuous and peak current limits**,
 *   - the **driver enable** line,
 *   - a **ramp-to-zero stop** (as opposed to a hard disable), and
 *   - the per-channel fault latch.
 *
 * # Why not let the control loop write PWM directly
 * Three reasons, in order of how much trouble they cause in practice:
 *
 * 1. **The current loop cannot follow a step.**  The velocity PID's output can
 *    jump by 1000 permille between two samples.  With `L = 1.5 mH` and
 *    `R = 2 ohm` the electrical pole is at 133 Hz; a full-scale step therefore
 *    produces a current spike limited only by the winding resistance
 *    (~12 A here), which trips the driver's over-current protection and, on a
 *    cheaper board, destroys the FETs.  The slew limiter turns that into a
 *    controlled ramp whose slope is a *design parameter* in `config.h` rather
 *    than an accident of the loop timing.
 * 2. **Two modules would otherwise fight over the enable pin.**  The safety
 *    layer must be able to disable a channel; if the control loop also wrote
 *    the pin directly, a re-enable could happen one sample after a fault.
 *    Routing every write through this module makes "disabled" sticky.
 * 3. **Units.**  Everything above this module works in physical units; the
 *    per-mille duty, the sign convention and the current scaling live here and
 *    nowhere else.
 *
 * # Sign convention
 * A **positive duty** produces **positive velocity** on the joint and
 * **closing motion** on the gripper.  The gripper's sign is the opposite of the
 * joint's mechanical sense on purpose: "positive = doing the task" is easier to
 * review than remembering which channel is inverted.
 *
 * # What happens without it
 * The arm works on the bench and fails in the field, because the failure mode
 * is a current spike that only appears when the commanded step is large —i.e.
 * during the first move after a long idle, which is exactly the demo.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_MOTOR_H
#define TP_MOTOR_H

#include <stdint.h>

#include "bsp_types.h"
#include "filters.h"
#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Window over which the *peak* current limit applies, milliseconds.
 *
 * `TP_MAX_CURRENT_MA` is a continuous rating and `TP_PEAK_CURRENT_MA` a
 * short-time one, and enforcing only the former means a drive can sit at the
 * continuous limit forever while the peak figure in `config.h` describes
 * nothing.  The value was documented and never used, which is worse than not
 * having it: a reader sizing a driver against a 5 A peak would be working from a
 * number the firmware does not implement.
 */
#define TP_MOTOR_PEAK_WINDOW_MS 50u

/** @brief Number of consecutive over-current samples before the latch trips. */
#define TP_MOTOR_OC_TRIP_SAMPLES 20u

/** @brief Time without over-current before the latch self-clears. */
#define TP_MOTOR_OC_RECOVER_MS 500u

/**
 * @brief One motor channel.
 */
typedef struct {
    tp_motor_ch_t ch;             /**< Hardware channel.                     */
    float    duty_cmd;            /**< Duty requested by the controller.     */
    float    duty_applied;        /**< Duty after the slew limiter.          */
    float    current_ma;          /**< Latest measured current.              */
    float    current_limit_ma;    /**< Continuous limit for this channel.    */
    float    duty_limit;          /**< Further reduction from fold-back.     */

    tp_filter_slew_t slew;        /**< Duty rate limiter.                    */

    int      enabled;             /**< Non-zero when the driver is enabled.  */
    int      fault;               /**< Latched over-current / driver fault.  */
    int      output_saturated;    /**< Non-zero when the duty is clamped.    */

    uint32_t overcurrent_samples; /**< Consecutive samples above the continuous limit. */
    uint32_t peak_samples;        /**< Consecutive samples above the peak limit.   */
    uint32_t last_overcurrent_ms; /**< Timestamp of the last violation.      */
    uint32_t foldback_events;     /**< How often the limit was applied.      */
    uint32_t stop_requests;       /**< Number of @ref tp_motor_stop calls.   */
    uint32_t command_writes;      /**< Number of @ref tp_motor_set_duty.     */
    int      initialised;         /**< Non-zero after a successful init.     */
} tp_motor_t;

/**
 * @brief Initialise a motor channel and enable its driver.
 *
 * @param[in,out] motor  Instance.
 * @param[in]     ch     Hardware channel.
 * @param[in]     dt_s   Control period in seconds, used by the slew limiter.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM / @ref TP_ERR_RANGE.
 */
tp_status_t tp_motor_init(tp_motor_t *motor, tp_motor_ch_t ch, float dt_s);

/**
 * @brief Request a duty.
 *
 * The request is recorded but **not** written to the hardware; the value only
 * reaches the PWM register in @ref tp_motor_update, once the slew limiter and
 * the current limit have been applied.  This separation is what makes the order
 * of operations in the control task reviewable instead of implicit.
 *
 * @param[in,out] motor   Instance.
 * @param[in]     permille Duty in per-mille, positive = "do the task".
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_FAULT_ACTIVE when the
 *         channel is faulted or disabled (the request is then ignored).
 */
tp_status_t tp_motor_set_duty(tp_motor_t *motor, float permille);

/**
 * @brief Apply the slew limit, current limit and write the hardware.
 *
 * Must be called exactly once per control period.
 *
 * @param[in,out] motor   Instance.
 * @param[in]     dt_s    Elapsed time in seconds.
 * @return @ref TP_OK, or @ref TP_ERR_FAULT_ACTIVE when the channel is faulted.
 */
tp_status_t tp_motor_update(tp_motor_t *motor, float dt_s);

/**
 * @brief Ramp the duty to zero over a bounded time, then leave the driver on.
 *
 * Used by the normal stop path: it avoids the mechanical shock of cutting the
 * drive, and it keeps the joint actively braked rather than free.
 *
 * @param[in,out] motor       Instance.
 * @param[in]     ramp_ms     Time allowed for the ramp, in milliseconds.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM.
 */
tp_status_t tp_motor_stop(tp_motor_t *motor, uint32_t ramp_ms);

/**
 * @brief Cut the driver immediately.
 *
 * The joint coasts.  Reserved for faults where continuing to drive is worse
 * than losing position, e.g. `ERR_MOTOR_STALL`.
 *
 * @param[in,out] motor  Instance.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM.
 */
tp_status_t tp_motor_disable(tp_motor_t *motor);

/**
 * @brief Re-enable a channel whose fault condition has cleared.
 *
 * @param[in,out] motor  Instance.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_FAULT_ACTIVE when the
 *         channel is still latched (call @ref tp_motor_clear_fault first).
 */
tp_status_t tp_motor_enable(tp_motor_t *motor);

/**
 * @brief Clear the fault latch if the physical condition has gone away.
 *
 * Refuses while the measured current is still above the limit, so a clear
 * command cannot be used to defeat an active protection.
 *
 * @param[in,out] motor  Instance.
 * @return @ref TP_OK, or @ref TP_ERR_FAULT_ACTIVE when still over-current.
 */
tp_status_t tp_motor_clear_fault(tp_motor_t *motor);

/**
 * @brief Latest duty actually applied to the hardware.
 *
 * @param[in] motor  Instance.
 * @return Duty in per-mille.
 */
float tp_motor_get_duty(const tp_motor_t *motor);

/**
 * @brief Latest measured current.
 *
 * @param[in] motor  Instance.
 * @return Current in milliamperes.
 */
float tp_motor_get_current_ma(const tp_motor_t *motor);

/**
 * @brief Whether the channel is faulted.
 *
 * @param[in] motor  Instance.
 * @return Non-zero when latched.
 */
int tp_motor_is_faulted(const tp_motor_t *motor);

/**
 * @brief Whether the driver is currently enabled.
 *
 * @param[in] motor  Instance.
 * @return Non-zero when enabled.
 */
int tp_motor_is_enabled(const tp_motor_t *motor);

/**
 * @brief Suggest a duty reduction that would bring the current back in range.
 *
 * A simple proportional fold-back used by the safety layer's derating path.
 * Returned rather than applied, so the caller can log the decision.
 *
 * @param[in] motor  Instance.
 * @return Multiplicative factor in (0, 1].
 */
float tp_motor_foldback_factor(const tp_motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* TP_MOTOR_H */
