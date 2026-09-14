/**
 * @file    fault_manager.h
 * @brief   Central fault registry: raise, latch, clear, report.
 *
 * @details
 * # Why a central registry and not a flag per module
 * Nine different conditions can stop this robot, and they are detected in four
 * different modules.  If each module kept its own flag:
 *
 *   - the host could only learn about the first one it happened to poll;
 *   - "was this fault ever raised?" (as opposed to "is it active now?") would
 *     be unanswerable, so an intermittent fault — the only kind that is
 *     interesting — would leave no trace;
 *   - clearing faults would mean calling nine functions in the right order, and
 *     a forgotten one shows up as a robot that will not start again.
 *
 * The registry answers three distinct questions, and keeping them distinct is
 * the main design decision in this file:
 *
 * | Question | Field | Cleared by |
 * |---|---|---|
 * | Is the condition true right now? | @ref tp_fault_record_t::active | the condition going away |
 * | Has it been latched? | @ref tp_fault_record_t::latched | an explicit clear |
 * | Did it ever happen? | @ref tp_fault_record_t::count | never (until reset) |
 *
 * A fault that is `active` but not `latched` is a transient: an over-current
 * blip that has already ended.  A fault that is `latched` and no longer
 * `active` is the normal state of a condition that has been fixed but not yet
 * acknowledged — and refusing to move until it is acknowledged is deliberate.
 *
 * # Severity-driven reaction, in one place
 * `fault_codes.h` assigns each code a severity; this module applies the
 * corresponding policy.  Putting the reaction here rather than at each call
 * site means an over-force event stops the gripper the same way no matter which
 * module noticed it.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_FAULT_MANAGER_H
#define TP_FAULT_MANAGER_H

#include <stdint.h>

#include "fault_codes.h"
#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Number of past fault events retained for the post-mortem report. */
#define TP_FAULT_HISTORY_DEPTH 16u

/**
 * @brief State of one fault code.
 */
typedef struct {
    int      active;        /**< The condition is true right now.            */
    int      latched;       /**< Requires an explicit clear.                 */
    uint32_t count;         /**< Times the fault has been raised.            */
    uint32_t first_ms;      /**< Timestamp of the first raise.               */
    uint32_t last_ms;       /**< Timestamp of the most recent raise.         */
    uint8_t  detail;        /**< Caller-supplied extra information.          */
    uint32_t active_ms;     /**< Accumulated time the condition was active.  */
} tp_fault_record_t;

/**
 * @brief One entry of the fault history ring.
 */
typedef struct {
    uint32_t        t_ms;    /**< When.                                      */
    tp_fault_code_t code;    /**< What.                                      */
    uint8_t         detail;  /**< Extra information from the detector.       */
    int             raised;  /**< 1 for a raise event, 0 for a clear.        */
} tp_fault_event_t;

/**
 * @brief The fault registry.
 */
typedef struct {
    tp_fault_record_t record[TP_FAULT_CODE_COUNT]; /**< Per-code state.       */
    tp_fault_event_t  history[TP_FAULT_HISTORY_DEPTH]; /**< Event ring.        */
    uint32_t          history_head;  /**< Next write index.                 */
    uint32_t          history_count; /**< Entries stored, saturating.       */
    uint32_t          now_ms;        /**< Latest timestamp supplied.        */
    uint32_t          latch_ms;      /**< When the first fault latched.     */
    tp_fault_code_t   highest;       /**< Highest-severity latched fault.   */
    uint32_t          raise_total;   /**< Total raises across all codes.    */
    uint32_t          clear_total;   /**< Total clears across all codes.    */
    int               any_latched;   /**< Non-zero when anything is latched.*/
    int               initialised;   /**< Non-zero after a successful init. */
} tp_fault_manager_t;

/**
 * @brief Initialise (or fully reset) the registry.
 *
 * @param[in,out] fm  Instance.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM.
 */
tp_status_t tp_fault_manager_init(tp_fault_manager_t *fm);

/**
 * @brief Raise a fault.
 *
 * Idempotent with respect to the *event* history while the fault is already
 * active: the count increments, but a duplicate history entry is only written
 * when the fault re-activates after having cleared.  Without that rule a fault
 * that is re-detected every control period would fill the history ring in 16 ms
 * and destroy the information the ring exists to preserve.
 *
 * @param[in,out] fm      Instance.
 * @param[in]     code    Fault code.
 * @param[in]     detail  Detector-specific extra value.
 * @param[in]     now_ms  Current timestamp.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM / @ref TP_ERR_RANGE.
 */
tp_status_t tp_fault_raise(tp_fault_manager_t *fm, tp_fault_code_t code,
                           uint8_t detail, uint32_t now_ms);

/**
 * @brief Declare a fault's condition no longer true.
 *
 * For a non-latching fault this clears it completely.  For a latching fault it
 * only clears the `active` flag; the latch remains until
 * @ref tp_fault_clear is called.
 *
 * @param[in,out] fm      Instance.
 * @param[in]     code    Fault code.
 * @param[in]     now_ms  Current timestamp.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM.
 */
tp_status_t tp_fault_clear_condition(tp_fault_manager_t *fm, tp_fault_code_t code,
                                     uint32_t now_ms);

/**
 * @brief Acknowledge and clear a latched fault.
 *
 * Refuses while the underlying condition is still active: clearing an
 * over-current latch while the current is still high would let the operator
 * defeat the protection by pressing a button twice.
 *
 * @param[in,out] fm      Instance.
 * @param[in]     code    Fault code.
 * @param[in]     now_ms  Current timestamp.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_FAULT_ACTIVE when the
 *         condition has not gone away.
 */
tp_status_t tp_fault_clear(tp_fault_manager_t *fm, tp_fault_code_t code,
                           uint32_t now_ms);

/**
 * @brief Clear every fault whose condition has gone away.
 *
 * Used by the `CLEAR_FAULT` command.  Returns the number actually cleared so
 * the caller can report "3 of 5 cleared, 2 conditions still present" instead of
 * a bare success.
 *
 * @param[in,out] fm      Instance.
 * @param[in]     now_ms  Current timestamp.
 * @return Number of faults cleared.
 */
uint32_t tp_fault_clear_all(tp_fault_manager_t *fm, uint32_t now_ms);

/**
 * @brief Accumulate active time for every currently-active fault.
 *
 * Called once per safety period.  Without it the registry could report *that* a
 * fault occurred but not *how long* it lasted, and duration is what distinguishes
 * a nuisance trip from a real one.
 *
 * @param[in,out] fm          Instance.
 * @param[in]     elapsed_ms  Time since the previous call.
 */
void tp_fault_tick(tp_fault_manager_t *fm, uint32_t elapsed_ms);

/**
 * @brief Whether a specific fault is latched.
 *
 * @param[in] fm    Instance.
 * @param[in] code  Fault code.
 * @return Non-zero when latched.
 */
int tp_fault_is_latched(const tp_fault_manager_t *fm, tp_fault_code_t code);

/**
 * @brief Whether a specific fault's condition is active.
 *
 * @param[in] fm    Instance.
 * @param[in] code  Fault code.
 * @return Non-zero when active.
 */
int tp_fault_is_active(const tp_fault_manager_t *fm, tp_fault_code_t code);

/**
 * @brief Whether anything is latched.
 *
 * @param[in] fm  Instance.
 * @return Non-zero when at least one fault is latched.
 */
int tp_fault_any_latched(const tp_fault_manager_t *fm);

/**
 * @brief The most severe latched fault.
 *
 * @param[in] fm  Instance.
 * @return Fault code, or @ref TP_FAULT_NONE when nothing is latched.
 */
tp_fault_code_t tp_fault_highest(const tp_fault_manager_t *fm);

/**
 * @brief Severity of the most severe latched fault.
 *
 * @param[in] fm  Instance.
 * @return Severity, or @ref TP_SEVERITY_INFO when nothing is latched.
 */
tp_severity_t tp_fault_highest_severity(const tp_fault_manager_t *fm);

/**
 * @brief Whether motion must be inhibited.
 *
 * True when any latched fault has severity `ERROR` or above.  A `WARNING`-level
 * fault (a communication timeout, a limit switch) permits motion, because
 * refusing to move on a comms blip would make the machine unusable; that
 * distinction is the reason severity exists at all.
 *
 * @param[in] fm  Instance.
 * @return Non-zero when motion must stop.
 */
int tp_fault_blocks_motion(const tp_fault_manager_t *fm);

/**
 * @brief Whether a physical hardware kill is required.
 *
 * True for `CRITICAL` severity.  The application calls
 * `tp_platform_emergency_kill()` when this returns non-zero.
 *
 * @param[in] fm  Instance.
 * @return Non-zero when the drives must be hardware-killed.
 */
int tp_fault_requires_kill(const tp_fault_manager_t *fm);

/**
 * @brief Read one fault's record.
 *
 * @param[in]  fm     Instance.
 * @param[in]  code   Fault code.
 * @param[out] out    Receives a copy of the record.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM / @ref TP_ERR_RANGE.
 */
tp_status_t tp_fault_get_record(const tp_fault_manager_t *fm,
                                tp_fault_code_t code, tp_fault_record_t *out);

/**
 * @brief Dump the active faults as a human-readable string.
 *
 * Used by the debug console and by the boot banner, so an operator can see the
 * machine's fault state without a host tool.
 *
 * @param[in]  fm       Instance.
 * @param[out] buffer   Destination.
 * @param[in]  capacity Capacity of @p buffer.
 * @return Number of characters written, excluding the terminator.
 */
uint32_t tp_fault_format_active(const tp_fault_manager_t *fm, char *buffer,
                                uint32_t capacity);

/**
 * @brief Copy out the fault event history, oldest first.
 *
 * @param[in]  fm   Instance.
 * @param[out] out  Destination array.
 * @param[in]  max  Capacity of @p out.
 * @return Number of entries copied.
 */
uint32_t tp_fault_get_history(const tp_fault_manager_t *fm,
                              tp_fault_event_t *out, uint32_t max);

#ifdef __cplusplus
}
#endif

#endif /* TP_FAULT_MANAGER_H */
