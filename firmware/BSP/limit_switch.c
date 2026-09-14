/**
 * @file    limit_switch.c
 * @brief   Implementation of the debounced discrete-input reader.
 * @copyright MIT License, see LICENSE.
 */

#include "limit_switch.h"

#include <string.h>

#include "compiler.h"
#include "config.h"
#include "logger.h"
#include "platform_api.h"

tp_status_t tp_limit_switch_init(tp_limit_switch_t *sw, tp_io_id_t id,
                                 int active_low)
{
    uint8_t level = 0u;
    tp_status_t st;

    if (sw == NULL) {
        return TP_ERR_PARAM;
    }
    if (((int)id < 0) || (id >= TP_IO_COUNT)) {
        return TP_ERR_PARAM;
    }

    (void)memset(sw, 0, sizeof(*sw));
    sw->id = id;
    sw->active_low = (active_low != 0) ? 1 : 0;

    st = tp_platform_io_read(id, &level);
    if (st != TP_OK) {
        LOG_ERROR(TP_MOD_SAFETY, "input %d unavailable (%s)", (int)id,
                  tp_status_str(st));
        return st;
    }
    sw->raw = level;
    sw->stable = (uint8_t)((active_low != 0) ? (level == 0u) : (level != 0u));
    /* Prime the accumulator at the extreme so the first genuine change still
     * needs the full debounce window: a glitch at boot must not be accepted. */
    sw->counter = (uint8_t)(sw->stable ? TP_LIMIT_DEBOUNCE_COUNT : 0u);
    sw->initialised = 1;
    return TP_OK;
}

tp_status_t tp_limit_switch_update(tp_limit_switch_t *sw)
{
    uint8_t level = 0u;
    uint8_t logical;

    if ((sw == NULL) || (sw->initialised == 0)) {
        return TP_ERR_NOT_READY;
    }
    if (tp_platform_io_read(sw->id, &level) != TP_OK) {
        /* A failed read must NOT change the debounced state.  Losing an input
         * momentarily is not evidence that the limit was released; treating it
         * as such would let the drive push into a hard stop for one period. */
        return TP_ERR_NOT_READY;
    }
    sw->update_count++;
    sw->raw = level;
    logical = (uint8_t)((sw->active_low != 0) ? (level == 0u) : (level != 0u));

    if (logical == sw->stable) {
        /* Agrees with the published state: relax the accumulator back towards
         * that state's extreme. */
        if (sw->stable != 0u) {
            if (sw->counter < TP_LIMIT_DEBOUNCE_COUNT) {
                sw->counter++;
            }
        } else {
            if (sw->counter > 0u) {
                sw->counter--;
            }
        }
        return TP_OK;
    }

    /* Disagrees: drive the accumulator towards the other extreme. */
    if (sw->stable != 0u) {
        if (sw->counter > 0u) {
            sw->counter--;
        }
    } else {
        if (sw->counter < TP_LIMIT_DEBOUNCE_COUNT) {
            sw->counter++;
        }
    }

    if (sw->stable != 0u) {
        if (sw->counter == 0u) {
            sw->stable = 0u;
            sw->transitions++;
        }
    } else {
        if (sw->counter >= TP_LIMIT_DEBOUNCE_COUNT) {
            sw->stable = 1u;
            sw->transitions++;
        }
    }
    return TP_OK;
}

int tp_limit_switch_is_active(const tp_limit_switch_t *sw)
{
    return (sw != NULL) ? (int)sw->stable : 0;
}

uint8_t tp_limit_switch_raw(const tp_limit_switch_t *sw)
{
    return (sw != NULL) ? sw->raw : 0u;
}

void tp_limit_switch_force(tp_limit_switch_t *sw, int active)
{
    if (sw == NULL) {
        return;
    }
    sw->stable = (uint8_t)((active != 0) ? 1u : 0u);
    sw->counter = (uint8_t)((active != 0) ? TP_LIMIT_DEBOUNCE_COUNT : 0u);
    /* Mirror the raw level so that a diagnostic dump after a forced state is
     * self-consistent rather than showing a raw/stable mismatch. */
    sw->raw = (uint8_t)((sw->active_low != 0) ? (active == 0 ? 1u : 0u)
                                              : (active != 0 ? 1u : 0u));
}
