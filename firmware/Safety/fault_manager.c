/**
 * @file    fault_manager.c
 * @brief   Implementation of the fault registry.
 * @copyright MIT License, see LICENSE.
 */

#include "fault_manager.h"

#include <string.h>

#include "compiler.h"
#include "config.h"
#include "logger.h"
#include "tp_format.h"

/**
 * @brief Append an entry to the fault history ring.
 *
 * @param[in,out] fm      Registry.
 * @param[in]     code    Fault code.
 * @param[in]     detail  Detector detail.
 * @param[in]     raised  Non-zero for a raise, 0 for a clear.
 * @param[in]     now_ms  Timestamp.
 */
static void fault_history_push(tp_fault_manager_t *fm, tp_fault_code_t code,
                               uint8_t detail, int raised, uint32_t now_ms)
{
    tp_fault_event_t *e = &fm->history[fm->history_head];

    e->t_ms = now_ms;
    e->code = code;
    e->detail = detail;
    e->raised = (raised != 0) ? 1 : 0;
    fm->history_head = (fm->history_head + 1u) % TP_FAULT_HISTORY_DEPTH;
    if (fm->history_count < TP_FAULT_HISTORY_DEPTH) {
        fm->history_count++;
    }
}

/**
 * @brief Recompute the aggregate fields from the per-code records.
 *
 * Kept as one function so the aggregates can never disagree with the records —
 * a class of bug where the host shows "no fault" while the robot refuses to
 * move is exactly what happens when they are updated separately.
 *
 * @param[in,out] fm  Registry.
 */
static void fault_recompute(tp_fault_manager_t *fm)
{
    int i;
    int any = 0;
    tp_fault_code_t highest = TP_FAULT_NONE;
    tp_severity_t highest_sev = TP_SEVERITY_INFO;

    for (i = 1; i < (int)TP_FAULT_CODE_COUNT; ++i) {
        if (fm->record[i].latched != 0) {
            tp_severity_t sev = tp_fault_severity((tp_fault_code_t)i);
            any = 1;
            if (sev > highest_sev) {
                highest_sev = sev;
                highest = (tp_fault_code_t)i;
            } else if ((sev == highest_sev) && (highest == TP_FAULT_NONE)) {
                highest = (tp_fault_code_t)i;
            }
        }
    }
    fm->any_latched = any;
    fm->highest = highest;
}

tp_status_t tp_fault_manager_init(tp_fault_manager_t *fm)
{
    if (fm == NULL) {
        return TP_ERR_PARAM;
    }
    (void)memset(fm, 0, sizeof(*fm));
    fm->highest = TP_FAULT_NONE;
    fm->initialised = 1;
    LOG_INFO(TP_MOD_SAFETY, "fault manager ready, %d codes registered",
             (int)TP_FAULT_CODE_COUNT);
    return TP_OK;
}

tp_status_t tp_fault_raise(tp_fault_manager_t *fm, tp_fault_code_t code,
                           uint8_t detail, uint32_t now_ms)
{
    tp_fault_record_t *r;

    if ((fm == NULL) || (fm->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (((int)code <= 0) || (code >= TP_FAULT_CODE_COUNT)) {
        return TP_ERR_RANGE;
    }
    fm->now_ms = now_ms;
    r = &fm->record[(int)code];

    if (r->count == 0u) {
        r->first_ms = now_ms;
    }
    r->count++;
    r->last_ms = now_ms;
    r->detail = detail;
    fm->raise_total++;

    if (r->active == 0) {
        /* Only a genuine activation writes history, so a fault that re-fires
         * every period cannot flush the ring. */
        fault_history_push(fm, code, detail, 1, now_ms);
        LOG_ERROR(TP_MOD_SAFETY, "%s (code %d, detail %u): %s",
                  tp_fault_name(code), (int)code, (unsigned)detail,
                  tp_fault_desc(code));
    }
    r->active = 1;
    if (tp_fault_is_latching(code) != 0) {
        if (r->latched == 0) {
            fm->latch_ms = now_ms;
        }
        r->latched = 1;
    }
    fault_recompute(fm);
    return TP_OK;
}

tp_status_t tp_fault_clear_condition(tp_fault_manager_t *fm, tp_fault_code_t code,
                                     uint32_t now_ms)
{
    tp_fault_record_t *r;

    if ((fm == NULL) || (fm->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (((int)code <= 0) || (code >= TP_FAULT_CODE_COUNT)) {
        return TP_ERR_RANGE;
    }
    r = &fm->record[(int)code];
    if (r->active == 0) {
        return TP_OK;
    }
    r->active = 0;
    if (tp_fault_is_latching(code) == 0) {
        /* Non-latching: the latch never existed, so clearing the condition
         * fully clears the fault. */
        r->latched = 0;
        fault_history_push(fm, code, r->detail, 0, now_ms);
        LOG_INFO(TP_MOD_SAFETY, "%s cleared automatically", tp_fault_name(code));
    }
    fault_recompute(fm);
    return TP_OK;
}

tp_status_t tp_fault_clear(tp_fault_manager_t *fm, tp_fault_code_t code,
                           uint32_t now_ms)
{
    tp_fault_record_t *r;

    if ((fm == NULL) || (fm->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (((int)code <= 0) || (code >= TP_FAULT_CODE_COUNT)) {
        return TP_ERR_RANGE;
    }
    r = &fm->record[(int)code];
    if (r->latched == 0) {
        return TP_OK;
    }
    if (r->active != 0) {
        /* Refuse: the physical condition is still present.  An operator must
         * not be able to defeat a protection by acknowledging it. */
        LOG_WARN(TP_MOD_SAFETY, "%s clear refused: condition still active",
                 tp_fault_name(code));
        return TP_ERR_FAULT_ACTIVE;
    }
    r->latched = 0;
    r->count = 0u;
    fm->clear_total++;
    fault_history_push(fm, code, r->detail, 0, now_ms);
    LOG_INFO(TP_MOD_SAFETY, "%s acknowledged and cleared", tp_fault_name(code));
    fault_recompute(fm);
    return TP_OK;
}

uint32_t tp_fault_clear_all(tp_fault_manager_t *fm, uint32_t now_ms)
{
    uint32_t cleared = 0u;
    int i;

    if ((fm == NULL) || (fm->initialised == 0)) {
        return 0u;
    }
    for (i = 1; i < (int)TP_FAULT_CODE_COUNT; ++i) {
        /* Only a fault that was actually latched can be "cleared".  Counting
         * every code whose count is zero afterwards would report nine cleared
         * faults on a healthy robot, because `tp_fault_clear` returns TP_OK
         * immediately for a code that was never raised.  That made the
         * "3 of 5 cleared" message the function exists to produce completely
         * meaningless. */
        if (fm->record[i].latched == 0) {
            continue;
        }
        if (tp_fault_clear(fm, (tp_fault_code_t)i, now_ms) == TP_OK) {
            cleared++;
        }
    }
    return cleared;
}

void tp_fault_tick(tp_fault_manager_t *fm, uint32_t elapsed_ms)
{
    int i;

    if ((fm == NULL) || (fm->initialised == 0)) {
        return;
    }
    for (i = 1; i < (int)TP_FAULT_CODE_COUNT; ++i) {
        if (fm->record[i].active != 0) {
            fm->record[i].active_ms += elapsed_ms;
        }
    }
}

int tp_fault_is_latched(const tp_fault_manager_t *fm, tp_fault_code_t code)
{
    if ((fm == NULL) || ((int)code <= 0) || (code >= TP_FAULT_CODE_COUNT)) {
        return 0;
    }
    return fm->record[(int)code].latched;
}

int tp_fault_is_active(const tp_fault_manager_t *fm, tp_fault_code_t code)
{
    if ((fm == NULL) || ((int)code <= 0) || (code >= TP_FAULT_CODE_COUNT)) {
        return 0;
    }
    return fm->record[(int)code].active;
}

int tp_fault_any_latched(const tp_fault_manager_t *fm)
{
    return (fm != NULL) ? fm->any_latched : 0;
}

tp_fault_code_t tp_fault_highest(const tp_fault_manager_t *fm)
{
    return (fm != NULL) ? fm->highest : TP_FAULT_NONE;
}

tp_severity_t tp_fault_highest_severity(const tp_fault_manager_t *fm)
{
    if ((fm == NULL) || (fm->highest == TP_FAULT_NONE)) {
        return TP_SEVERITY_INFO;
    }
    return tp_fault_severity(fm->highest);
}

int tp_fault_blocks_motion(const tp_fault_manager_t *fm)
{
    return (tp_fault_highest_severity(fm) >= TP_SEVERITY_ERROR) ? 1 : 0;
}

int tp_fault_requires_kill(const tp_fault_manager_t *fm)
{
    return (tp_fault_highest_severity(fm) >= TP_SEVERITY_CRITICAL) ? 1 : 0;
}

tp_status_t tp_fault_get_record(const tp_fault_manager_t *fm,
                                tp_fault_code_t code, tp_fault_record_t *out)
{
    if ((fm == NULL) || (out == NULL)) {
        return TP_ERR_PARAM;
    }
    if (((int)code < 0) || (code >= TP_FAULT_CODE_COUNT)) {
        return TP_ERR_RANGE;
    }
    *out = fm->record[(int)code];
    return TP_OK;
}

uint32_t tp_fault_format_active(const tp_fault_manager_t *fm, char *buffer,
                                uint32_t capacity)
{
    int pos = 0;
    int i;
    int first = 1;

    if ((fm == NULL) || (buffer == NULL) || (capacity == 0u)) {
        return 0u;
    }
    if (fm->any_latched == 0) {
        pos = tp_snprintf(buffer, capacity, "OK");
        return (pos > 0) ? (uint32_t)pos : 0u;
    }
    for (i = 1; i < (int)TP_FAULT_CODE_COUNT; ++i) {
        int n;
        if (fm->record[i].latched == 0) {
            continue;
        }
        n = tp_snprintf(&buffer[pos], capacity - (uint32_t)pos, "%s%s%s",
                        first ? "" : ",",
                        tp_fault_name((tp_fault_code_t)i),
                        (fm->record[i].active != 0) ? "(active)" : "(cleared)");
        if (n <= 0) {
            break;
        }
        pos += n;
        first = 0;
        if ((uint32_t)pos >= (capacity - 1u)) {
            break;
        }
    }
    buffer[capacity - 1u] = '\0';
    return (uint32_t)pos;
}

uint32_t tp_fault_get_history(const tp_fault_manager_t *fm,
                              tp_fault_event_t *out, uint32_t max)
{
    uint32_t n;
    uint32_t i;
    uint32_t start;

    if ((fm == NULL) || (out == NULL) || (max == 0u)) {
        return 0u;
    }
    n = (fm->history_count < max) ? fm->history_count : max;
    start = (fm->history_count < TP_FAULT_HISTORY_DEPTH) ? 0u : fm->history_head;
    for (i = 0u; i < n; ++i) {
        out[i] = fm->history[(start + i) % TP_FAULT_HISTORY_DEPTH];
    }
    return n;
}
