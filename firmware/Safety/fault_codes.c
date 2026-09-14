/**
 * @file    fault_codes.c
 * @brief   Tables backing @ref fault_codes.h.
 * @copyright MIT License, see LICENSE.
 */

#include "fault_codes.h"

#include <stddef.h>

/** @brief Static description of one fault code. */
typedef struct {
    const char     *name;     /**< Symbolic name, matches the enum.         */
    const char     *desc;     /**< One-line explanation.                    */
    tp_severity_t   severity; /**< Reaction policy.                         */
    const char     *recovery; /**< What an operator should do.              */
} fault_info_t;

/**
 * @brief Lookup table.
 *
 * Indexed by @ref tp_fault_code_t, so adding a code means appending exactly one
 * row here; a unit test asserts that the table covers every enumerator.
 */
static const fault_info_t s_fault_table[TP_FAULT_CODE_COUNT] = {
    { "ERR_NONE", "no fault", TP_SEVERITY_INFO,
      "none" },
    { "ERR_FORCE_OVERLOAD", "grip force exceeded the hard limit", TP_SEVERITY_ERROR,
      "open the gripper, inspect the bud and the load cell, then CLEAR_FAULT" },
    { "ERR_MOTOR_STALL", "motor commanded but joint not moving", TP_SEVERITY_ERROR,
      "remove the obstruction, check the drive wiring and the encoder coupling" },
    { "ERR_OVERCURRENT", "motor current above continuous rating", TP_SEVERITY_ERROR,
      "let the driver cool down, check for a shorted winding or a jammed axis" },
    { "ERR_ENCODER", "encoder feedback physically implausible", TP_SEVERITY_CRITICAL,
      "power-cycle, then verify encoder A/B wiring, shielding and the coupling" },
    { "ERR_COMM_TIMEOUT", "no valid frame from the host within the timeout",
      TP_SEVERITY_WARNING,
      "check the serial/RS485 cable and the host process; telemetry resumes on "
      "the next valid frame" },
    { "ERR_SENSOR", "sensor reading range, stuck-value or spike violation",
      TP_SEVERITY_ERROR,
      "check the load-cell excitation and the ADC reference, then recalibrate" },
    { "ERR_LIMIT", "limit switch hit while driving further into it", TP_SEVERITY_WARNING,
      "back the joint off with the opposite direction command, then reset the origin" },
    { "ERR_ESTOP", "emergency stop asserted", TP_SEVERITY_CRITICAL,
      "release the E-stop, verify the workspace is clear, then RESET and re-home" },
    { "ERR_WATCHDOG", "watchdog reset or watchdog timeout", TP_SEVERITY_CRITICAL,
      "inspect the log for the task that stopped feeding the watchdog, then "
      "power-cycle" },
    { "ERR_TARGET", "vision target not reached or no longer usable",
      TP_SEVERITY_ERROR,
      "check the vision system's calibration and confidence threshold; the bud "
      "was not where the camera reported it" },
    { "ERR_INTERNAL", "internal invariant violation", TP_SEVERITY_ERROR,
      "capture the log and report; this indicates a firmware defect" }
};

const char *tp_fault_name(tp_fault_code_t code)
{
    if (((int)code < 0) || (code >= TP_FAULT_CODE_COUNT)) {
        return "ERR_UNKNOWN";
    }
    return s_fault_table[(int)code].name;
}

const char *tp_fault_desc(tp_fault_code_t code)
{
    if (((int)code < 0) || (code >= TP_FAULT_CODE_COUNT)) {
        return "unknown fault code";
    }
    return s_fault_table[(int)code].desc;
}

tp_severity_t tp_fault_severity(tp_fault_code_t code)
{
    if (((int)code < 0) || (code >= TP_FAULT_CODE_COUNT)) {
        /* Fail safe: an unrecognised code must not be treated as harmless. */
        return TP_SEVERITY_ERROR;
    }
    return s_fault_table[(int)code].severity;
}

int tp_fault_is_latching(tp_fault_code_t code)
{
    tp_severity_t sev = tp_fault_severity(code);

    return (sev >= TP_SEVERITY_ERROR) ? 1 : 0;
}

const char *tp_fault_recovery_hint(tp_fault_code_t code)
{
    if (((int)code < 0) || (code >= TP_FAULT_CODE_COUNT)) {
        return "capture the log and report the unknown code";
    }
    return s_fault_table[(int)code].recovery;
}
