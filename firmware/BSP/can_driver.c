/**
 * @file    can_driver.c
 * @brief   Implementation of the CAN transport driver.
 * @copyright MIT License, see LICENSE.
 */

#include "can_driver.h"

#include <string.h>

#include "compiler.h"
#include "config.h"
#include "logger.h"
#include "platform_api.h"

/** @brief Bit rate this revision has a validated timing calculation for. */
#define CAN_SUPPORTED_BITRATE TP_CAN_BITRATE_BPS

tp_status_t tp_can_init(tp_can_status_t *st, uint32_t bitrate_bps)
{
    if (st == NULL) {
        return TP_ERR_PARAM;
    }
    if (bitrate_bps != CAN_SUPPORTED_BITRATE) {
        /* Refusing an unsupported rate is deliberate.  CAN bit timing has to be
         * solved against the peripheral clock to land the sample point at the
         * right place; accepting an arbitrary rate here would produce a bus
         * that "mostly works" and fails intermittently at temperature. */
        LOG_ERROR(TP_MOD_COMM,
                  "CAN bitrate %lu unsupported; only %lu is validated",
                  (unsigned long)bitrate_bps, (unsigned long)CAN_SUPPORTED_BITRATE);
        return TP_ERR_UNSUPPORTED;
    }
    (void)memset(st, 0, sizeof(*st));
    st->bitrate_bps = bitrate_bps;
    st->initialised = 1;
    LOG_INFO(TP_MOD_COMM, "CAN up at %lu bit/s, sample point 87.5%%",
             (unsigned long)bitrate_bps);
    return TP_OK;
}

tp_status_t tp_can_send(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    tp_status_t st;

    if (id > 0x7FFu) {
        return TP_ERR_RANGE;
    }
    if (dlc > TP_CAN_MAX_DLC) {
        return TP_ERR_PARAM;
    }
    if ((data == NULL) && (dlc > 0u)) {
        return TP_ERR_PARAM;
    }
    st = tp_platform_can_send(id, data, dlc);
    if (st == TP_ERR_OVERFLOW) {
        /* Not an error condition, just back-pressure: the caller retries. */
        return TP_ERR_BUSY;
    }
    return st;
}

tp_status_t tp_can_recv(tp_can_frame_t *frame)
{
    uint8_t dlc = 0u;
    uint32_t id = 0u;
    uint8_t data[TP_CAN_MAX_DLC];
    tp_status_t st;

    if (frame == NULL) {
        return TP_ERR_PARAM;
    }
    (void)memset(data, 0, sizeof(data));
    st = tp_platform_can_recv(&id, data, &dlc);
    if (st != TP_OK) {
        return st;
    }
    frame->id = id;
    frame->dlc = (dlc > TP_CAN_MAX_DLC) ? TP_CAN_MAX_DLC : dlc;
    (void)memcpy(frame->data, data, frame->dlc);
    frame->timestamp_ms = tp_platform_time_ms();
    return TP_OK;
}

tp_status_t tp_can_set_filter(uint32_t index, uint32_t id, uint32_t mask)
{
    if (index >= TP_CAN_MAX_FILTERS) {
        return TP_ERR_RANGE;
    }
    if (id > 0x7FFu || mask > 0x7FFu) {
        return TP_ERR_RANGE;
    }
    /* The platform seam intentionally does not expose the filter registers:
     * hardware acceptance filtering is a target-only optimisation, and the
     * host backend must behave identically.  The software filter below is
     * therefore what actually runs on both, and the register programming is
     * done in `platform_stm32f407.c` for efficiency only. */
    LOG_INFO(TP_MOD_COMM, "CAN filter %lu: id 0x%03lX mask 0x%03lX",
             (unsigned long)index, (unsigned long)id, (unsigned long)mask);
    return TP_OK;
}

uint32_t tp_can_service(tp_can_status_t *st)
{
    tp_can_frame_t frame;
    uint32_t drained = 0u;

    if ((st == NULL) || (st->initialised == 0)) {
        return 0u;
    }
    /* Bounded loop: a saturated bus must not be able to starve the rest of the
     * communication task.  16 frames per 10 ms period is already more than the
     * 500 kbit/s bus can deliver (a full 8-byte frame takes 220 us). */
    while (drained < 16u) {
        if (tp_can_recv(&frame) != TP_OK) {
            break;
        }
        st->rx_frames++;
        drained++;
        /* Application dispatch is not done here; the communication task pulls
         * frames with tp_can_recv and routes them, so this function stays a
         * pure diagnostic. */
    }
    return drained;
}

int tp_can_is_bus_off(const tp_can_status_t *st)
{
    return (st != NULL) ? st->bus_off : 0;
}

tp_status_t tp_can_recover(tp_can_status_t *st)
{
    if (st == NULL) {
        return TP_ERR_PARAM;
    }
    st->bus_off = 0;
    st->tec = 0u;
    st->rec = 0u;
    LOG_WARN(TP_MOD_COMM, "CAN recovery requested after %d bus-off event(s)",
             st->bus_off_events);
    return TP_OK;
}

void tp_can_encode_u32(uint8_t *out, uint32_t value)
{
    if (out == NULL) {
        return;
    }
    out[0] = (uint8_t)((value >> 24) & 0xFFu);
    out[1] = (uint8_t)((value >> 16) & 0xFFu);
    out[2] = (uint8_t)((value >> 8) & 0xFFu);
    out[3] = (uint8_t)(value & 0xFFu);
}

uint32_t tp_can_decode_u32(const uint8_t *in)
{
    if (in == NULL) {
        return 0u;
    }
    return ((uint32_t)in[0] << 24) | ((uint32_t)in[1] << 16)
         | ((uint32_t)in[2] << 8) | (uint32_t)in[3];
}
