/**
 * @file    robot_state.h
 * @brief   The robot's discrete state vocabulary, shared across layers.
 *
 * @details
 * # Why this lives in the lowest layer and not in the FSM
 * The robot's state appears in three places that must agree exactly:
 *
 *   1. `robot_fsm.c`, which decides transitions,
 *   2. the wire protocol (`STATUS_RPT`, `TELEMETRY`), which reports the state
 *      to the host, and
 *   3. the Python monitor and the C simulator, which *render* it.
 *
 * If the enum lives in `robot_fsm.h` then the protocol layer has to include an
 * application header, which inverts the layering; if it is duplicated, the two
 * copies drift the first time a state is added and the monitor silently
 * mislabels everything after the insertion point.  Putting the vocabulary in a
 * leaf header with no dependencies makes the layering acyclic *and* makes drift
 * impossible, and it lets `protocol_types.h` stay a pure description of bytes
 * on a wire.
 *
 * The same reasoning applies to @ref tp_fault_code_t, which is why that enum
 * has its own leaf header in `firmware/Safety/fault_codes.h`.
 *
 * @note The numeric values are **part of the wire protocol**.  Only append.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_ROBOT_STATE_H
#define TP_ROBOT_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Discrete states of the picking robot.
 *
 * The numbering is grouped so a log reader can tell at a glance which phase of
 * the cycle a line belongs to: `0` is boot, `1-2` is idle, `3-9` is the pick
 * cycle in order, and `10+` is abnormal.
 */
typedef enum {
    /** @brief Power-on, peripherals not yet validated. */
    TP_STATE_INIT = 0,
    /** @brief Boot self-test running: watchdogs, sensors, limits, comms. */
    TP_STATE_SELF_CHECK = 1,
    /** @brief Healthy and stationary, accepting commands. */
    TP_STATE_IDLE = 2,
    /** @brief A vision target has been latched and validated. */
    TP_STATE_TARGET_RECEIVED = 3,
    /** @brief Moving towards the pre-grasp pose above the bud. */
    TP_STATE_APPROACH = 4,
    /** @brief Jaw closing slowly, watching for the contact force step. */
    TP_STATE_CONTACT = 5,
    /** @brief Force-controlled closure to the commanded grip force. */
    TP_STATE_GRIP = 6,
    /** @brief Detaching the bud with the gripper closed. */
    TP_STATE_PICK = 7,
    /** @brief Opening the jaws to drop the bud into the basket. */
    TP_STATE_RELEASE = 8,
    /** @brief Returning to the home pose, ready for the next target. */
    TP_STATE_RETURN = 9,
    /** @brief A latched fault has stopped the cycle; motion inhibited. */
    TP_STATE_ERROR = 10,
    /** @brief Emergency stop asserted; drives hardware-killed. */
    TP_STATE_EMERGENCY_STOP = 11,
    /** @brief Number of defined states.  Keep last. */
    TP_STATE_COUNT = 12
} tp_robot_state_t;

/**
 * @brief Short symbolic name of a state, for logs and CSV output.
 *
 * Uses `snake_case` without the `TP_STATE_` prefix so that a CSV column is
 * readable by a spreadsheet and a `grep` on the log finds every occurrence.
 *
 * @param[in] state  State to name.
 * @return Static string such as `"grip"`.  Never NULL; unknown states return
 *         `"unknown"`.
 */
const char *tp_state_name(tp_robot_state_t state);

/**
 * @brief One-line description of what the robot is doing in a state.
 *
 * Used by the monitor tool's status line and by the docs, so the explanation is
 * written once.
 *
 * @param[in] state  State to describe.
 * @return Static string.  Never NULL.
 */
const char *tp_state_desc(tp_robot_state_t state);

/**
 * @brief Whether a state is part of the normal, healthy pick cycle.
 *
 * The safety layer uses this to decide whether an unexpected transition is a
 * fault: leaving a *cyclic* state for `ERROR` is normal, but being in a cyclic
 * state while a critical fault is latched is not.
 *
 * @param[in] state  State to test.
 * @return Non-zero when the state is INIT, SELF_CHECK, IDLE, or one of the
 *         eight cycle states.
 */
int tp_state_is_normal(tp_robot_state_t state);

/**
 * @brief Whether the state permits actuator motion.
 *
 * @param[in] state  State to test.
 * @return Non-zero when commanding the motors is legal in this state.
 */
int tp_state_allows_motion(tp_robot_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* TP_ROBOT_STATE_H */
