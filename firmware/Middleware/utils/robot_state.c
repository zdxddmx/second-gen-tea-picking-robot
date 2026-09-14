/**
 * @file    robot_state.c
 * @brief   Tables backing @ref robot_state.h.
 * @copyright MIT License, see LICENSE.
 */

#include "robot_state.h"

#include <stddef.h>

/** @brief Static description of one robot state. */
typedef struct {
    const char *name;  /**< Short symbolic name.                             */
    const char *desc;  /**< One-line explanation.                            */
} state_info_t;

/** @brief Lookup table indexed by @ref tp_robot_state_t. */
static const state_info_t s_state_table[TP_STATE_COUNT] = {
    { "init",            "power-on, peripherals not validated" },
    { "self_check",      "boot self-test in progress" },
    { "idle",            "healthy, stationary, accepting commands" },
    { "target_received", "vision target latched and validated" },
    { "approach",        "moving to the pre-grasp pose above the bud" },
    { "contact",         "closing slowly, watching for the contact force step" },
    { "grip",            "force-controlled closure to the commanded grip force" },
    { "pick",            "detaching the bud with the gripper closed" },
    { "release",         "opening the jaws to drop the bud" },
    { "return",          "returning to the home pose" },
    { "error",           "latched fault, motion inhibited" },
    { "emergency_stop",  "emergency stop asserted, drives killed" }
};

const char *tp_state_name(tp_robot_state_t state)
{
    if (((int)state < 0) || (state >= TP_STATE_COUNT)) {
        return "unknown";
    }
    return s_state_table[(int)state].name;
}

const char *tp_state_desc(tp_robot_state_t state)
{
    if (((int)state < 0) || (state >= TP_STATE_COUNT)) {
        return "unrecognised state code";
    }
    return s_state_table[(int)state].desc;
}

int tp_state_is_normal(tp_robot_state_t state)
{
    switch (state) {
    case TP_STATE_INIT:
    case TP_STATE_SELF_CHECK:
    case TP_STATE_IDLE:
    case TP_STATE_TARGET_RECEIVED:
    case TP_STATE_APPROACH:
    case TP_STATE_CONTACT:
    case TP_STATE_GRIP:
    case TP_STATE_PICK:
    case TP_STATE_RELEASE:
    case TP_STATE_RETURN:
        return 1;
    default:
        /* ERROR and EMERGENCY_STOP are by definition not normal, and an
         * out-of-range code must not be treated as healthy. */
        return 0;
    }
}

int tp_state_allows_motion(tp_robot_state_t state)
{
    switch (state) {
    case TP_STATE_APPROACH:
    case TP_STATE_CONTACT:
    case TP_STATE_GRIP:
    case TP_STATE_PICK:
    case TP_STATE_RELEASE:
    case TP_STATE_RETURN:
        return 1;
    default:
        /* INIT and SELF_CHECK must not move: the encoders have not been homed
         * yet, so any motion command would be relative to an unknown origin.
         * ERROR and EMERGENCY_STOP must not move for obvious reasons.  IDLE and
         * TARGET_RECEIVED are stationary by definition. */
        return 0;
    }
}
