/**
 * @file    fault_codes.h
 * @brief   The complete, closed set of fault conditions the robot can report.
 *
 * @details
 * # Why a dedicated header
 * Fault codes are part of the **wire protocol**, the **log format**, the **FAT
 * (factory acceptance test)** and the **safety case**.  If they live inside
 * `fault_manager.h` every module that wants to name a fault ends up including
 * the whole manager.  A standalone header with no dependencies keeps the
 * dependency graph acyclic and lets the Python tooling mirror the enum
 * one-to-one.
 *
 * # Latching policy
 * Every fault carries an explicit **severity**, which determines what the
 * manager does with it:
 *
 * | Severity | Behaviour | Cleared by |
 * |---|---|---|
 * | `INFO` | logged only, no state change | automatic |
 * | `WARNING` | logged, telemetry flag raised, motion continues | `CLEAR_FAULT` or automatic when the condition goes away |
 * | `ERROR` | motion disabled, FSM forced to `ERROR` | `CLEAR_FAULT`, and only if the physical condition has cleared |
 * | `CRITICAL` | PWM hardware-killed, FSM forced to `EMERGENCY_STOP`, requires a power cycle or an explicit `RESET` | `RESET` after the condition clears |
 *
 * The distinction matters: an over-force event is recoverable in software,
 * whereas an encoder that reports physically impossible motion means the
 * feedback path is untrustworthy and continuing to drive is not defensible.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_FAULT_CODES_H
#define TP_FAULT_CODES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Fault identifiers.
 *
 * The numeric values are **stable** and appear verbatim on the wire
 * (`FAULT_RPT` / `STATUS_RPT`), in the CSV telemetry and in the log.  Never
 * renumber an existing entry: append instead.
 */
typedef enum {
    /** @brief No fault. Healthy state. */
    TP_FAULT_NONE = 0,

    /** @brief Measured grip force exceeded @ref TP_FORCE_MAX_N.
     *  Raised by: safety monitor.  Reaction: stop the gripper immediately and
     *  open the jaws by @ref TP_GRIPPER_BACKOFF_MM to release the bud. */
    TP_FAULT_FORCE_OVERLOAD = 1,

    /** @brief Motor is being driven but the encoder does not move.
     *  Raised by: safety monitor.  Reaction: disable the drive; a stalled
     *  motor with current applied is a thermal hazard, and on a real machine
     *  it usually means the arm is jammed against a branch. */
    TP_FAULT_MOTOR_STALL = 2,

    /** @brief Motor current above @ref TP_MAX_CURRENT_MA for
     *  @ref TP_OVERCURRENT_TRIP_MS.  Raised by: safety monitor.  Reaction:
     *  disable the drive, allow a cool-down before re-enabling. */
    TP_FAULT_OVERCURRENT = 3,

    /** @brief Encoder feedback is physically implausible (impossible
     *  acceleration, or the two channels disagree).  Raised by: safety
     *  monitor.  Reaction: disable the drive; closed-loop control is no
     *  longer trustworthy. */
    TP_FAULT_ENCODER = 4,

    /** @brief No valid frame from the host for @ref TP_COMM_TIMEOUT_MS.
     *  Raised by: communication manager.  Reaction: hold position, then
     *  return to a safe idle posture; never continue a pick on stale target
     *  data. */
    TP_FAULT_COMM_TIMEOUT = 5,

    /** @brief A sensor reading is out of the physically possible range, stuck,
     *  or producing sustained spikes.  Raised by: force sensor driver /
     *  safety monitor.  Reaction: disable force control and fall back to
     *  position-only gripping, or stop if position control is also unsafe. */
    TP_FAULT_SENSOR = 6,

    /** @brief A limit switch was hit while the joint was commanded further
     *  into it.  Raised by: safety monitor.  Reaction: stop motion in that
     *  direction only; the opposite direction stays available so the arm can
     *  be backed off. */
    TP_FAULT_LIMIT = 7,

    /** @brief Emergency stop asserted (software command or the physical
     *  normally-closed loop opening).  Raised by: safety monitor.
     *  Reaction: kill PWM at the hardware level, no exceptions. */
    TP_FAULT_ESTOP = 8,

    /** @brief The watchdog fired, or the previously latched reset cause was
     *  the watchdog.  Raised by: boot self-check.  Reaction: refuse to
     *  auto-resume; require an explicit operator reset. */
    TP_FAULT_WATCHDOG = 9,

    /** @brief The vision target could not be reached or is no longer usable:
     *  the gripper closed for its full budget without finding the bud, or the
     *  target went stale before the cycle started.  Raised by: the force
     *  controller's contact timeout and the application's staleness check.
     *  Reaction: abort the cycle and ask the vision system for a fresh target.
     *
     *  This is deliberately *not* reported as @ref TP_FAULT_SENSOR.  The two
     *  send an operator to completely different places: a sensor fault means
     *  "inspect the load cell wiring", whereas this means "the bud was not
     *  where the camera said it was".  Conflating them wasted real debugging
     *  time on this project.
     *
     *  Severity is `ERROR`, not the `WARNING` it was first given.  A warning
     *  does not latch and does not block motion, so the state machine stayed in
     *  `CONTACT` with the gripping controller in `FAULT` and no event capable of
     *  moving it — the robot parked forever in a state whose name said it was
     *  still working.  A cycle that cannot proceed has to be able to stop the
     *  machine; that is what `ERROR` means. */
    TP_FAULT_TARGET = 10,

    /** @brief Catch-all for an internal invariant violation (e.g. a control
     *  output that is not finite, an FSM transition with no handler). */
    TP_FAULT_INTERNAL = 11,

    /** @brief Number of defined fault codes.  Keep last. */
    TP_FAULT_CODE_COUNT = 12
} tp_fault_code_t;

/**
 * @brief Severity of a fault, driving the manager's reaction policy.
 */
typedef enum {
    TP_SEVERITY_INFO = 0,     /**< Informational; no reaction.             */
    TP_SEVERITY_WARNING = 1,  /**< Degraded but operable.                  */
    TP_SEVERITY_ERROR = 2,    /**< Motion must stop.                       */
    TP_SEVERITY_CRITICAL = 3  /**< Hardware kill, operator intervention.   */
} tp_severity_t;

/**
 * @brief Short symbolic name of a fault code.
 *
 * @param[in] code  Fault code.
 * @return Static string such as `"ERR_FORCE_OVERLOAD"`.  Never NULL; unknown
 *         codes return `"ERR_UNKNOWN"`.
 */
const char *tp_fault_name(tp_fault_code_t code);

/**
 * @brief Human-readable one-line description of a fault code.
 *
 * @param[in] code  Fault code.
 * @return Static string.  Never NULL.
 */
const char *tp_fault_desc(tp_fault_code_t code);

/**
 * @brief Severity associated with a fault code.
 *
 * @param[in] code  Fault code.
 * @return Severity.  Unknown codes are treated as @ref TP_SEVERITY_ERROR,
 *         i.e. fail safe rather than fail open.
 */
tp_severity_t tp_fault_severity(tp_fault_code_t code);

/**
 * @brief Whether a fault latches until an explicit clear.
 *
 * `CRITICAL` faults always latch; `ERROR` faults latch but may be cleared by
 * `CLEAR_FAULT` once the physical condition has gone; `WARNING` faults
 * self-clear.
 *
 * @param[in] code  Fault code.
 * @return Non-zero when the fault must be cleared explicitly.
 */
int tp_fault_is_latching(tp_fault_code_t code);

/**
 * @brief Suggested recovery action, phrased as an instruction to an operator.
 *
 * Used by the host monitor and by the LED/blink-code reporter, so the text is
 * centralised rather than duplicated in three places.
 *
 * @param[in] code  Fault code.
 * @return Static string.  Never NULL.
 */
const char *tp_fault_recovery_hint(tp_fault_code_t code);

#ifdef __cplusplus
}
#endif

#endif /* TP_FAULT_CODES_H */
