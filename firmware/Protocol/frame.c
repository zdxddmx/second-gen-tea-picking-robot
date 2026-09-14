/**
 * @file    frame.c
 * @brief   Implementation of the streaming frame codec.
 * @copyright MIT License, see LICENSE.
 */

#include "frame.h"

#include <string.h>

#include "compiler.h"
#include "crc16.h"

void tp_frame_parser_init(tp_frame_parser_t *p)
{
    if (p == NULL) {
        return;
    }
    (void)memset(p, 0, sizeof(*p));
    p->state = TP_FRAME_WAIT_HDR0;
}

void tp_frame_parser_reset_stats(tp_frame_parser_t *p)
{
    if (p == NULL) {
        return;
    }
    p->frames_ok = 0u;
    p->crc_errors = 0u;
    p->length_errors = 0u;
    p->resyncs = 0u;
    p->bytes_discarded = 0u;
    p->bytes_consumed = 0u;
}

uint32_t tp_frame_length_for(uint8_t payload_len)
{
    if (payload_len > TP_PROTO_MAX_PAYLOAD) {
        return 0u;
    }
    return (uint32_t)payload_len + TP_PROTO_FIXED_OVERHEAD;
}

/**
 * @brief Restart the parser looking for a header, counting the resync.
 *
 * @param[in,out] p  Parser.
 */
static void frame_restart(tp_frame_parser_t *p)
{
    p->state = TP_FRAME_WAIT_HDR0;
    p->cmd = 0u;
    p->len = 0u;
    p->index = 0u;
    p->crc = TP_CRC16_MODBUS_INIT;
    p->crc_rx = 0u;
    p->crc_bytes = 0u;
}

tp_status_t tp_frame_parser_feed(tp_frame_parser_t *p, uint8_t byte,
                                 uint32_t now_ms, tp_frame_t *out)
{
    if (p == NULL) {
        return TP_ERR_PARAM;
    }
    p->bytes_consumed++;

    switch (p->state) {
    case TP_FRAME_WAIT_HDR0:
        if (byte == TP_PROTO_HDR0) {
            p->state = TP_FRAME_WAIT_HDR1;
        } else {
            p->bytes_discarded++;
        }
        return TP_ERR_NOT_READY;

    case TP_FRAME_WAIT_HDR1:
        if (byte == TP_PROTO_HDR1) {
            p->state = TP_FRAME_WAIT_CMD;
            p->crc = TP_CRC16_MODBUS_INIT;
        } else if (byte == TP_PROTO_HDR0) {
            /* `AA AA 55` : the second AA is the start of a real header.  Staying
             * in WAIT_HDR1 rather than falling back to WAIT_HDR0 is what makes
             * this resynchronise correctly instead of dropping a frame. */
            p->bytes_discarded++;
        } else {
            p->bytes_discarded++;
            p->resyncs++;
            p->state = TP_FRAME_WAIT_HDR0;
        }
        return TP_ERR_NOT_READY;

    case TP_FRAME_WAIT_CMD:
        p->cmd = byte;
        p->crc = tp_crc16_modbus_byte(p->crc, byte);
        p->state = TP_FRAME_WAIT_LEN;
        return TP_ERR_NOT_READY;

    case TP_FRAME_WAIT_LEN:
        p->len = byte;
        p->crc = tp_crc16_modbus_byte(p->crc, byte);
        if (p->len > (uint8_t)TP_PROTO_MAX_PAYLOAD) {
            /* Reject immediately.  Waiting for bytes that the peer will never
             * send would stall the parser for an unbounded time, and then a
             * real frame arriving inside that window would be eaten as
             * payload. */
            p->length_errors++;
            p->resyncs++;
            frame_restart(p);
            return TP_ERR_OVERFLOW;
        }
        p->index = 0u;
        p->state = (p->len == 0u) ? TP_FRAME_WAIT_CRC_LO : TP_FRAME_WAIT_PAYLOAD;
        return TP_ERR_NOT_READY;

    case TP_FRAME_WAIT_PAYLOAD:
        p->payload[p->index++] = byte;
        p->crc = tp_crc16_modbus_byte(p->crc, byte);
        if (p->index >= p->len) {
            p->state = TP_FRAME_WAIT_CRC_LO;
        }
        return TP_ERR_NOT_READY;

    case TP_FRAME_WAIT_CRC_LO:
        p->crc_rx = byte;
        p->crc_bytes = 1u;
        p->state = TP_FRAME_WAIT_CRC_HI;
        return TP_ERR_NOT_READY;

    case TP_FRAME_WAIT_CRC_HI:
        p->crc_rx |= (uint16_t)((uint16_t)byte << 8);
        p->crc_bytes = 2u;

        if (p->crc_rx == p->crc) {
            if (out != NULL) {
                out->cmd = p->cmd;
                out->len = p->len;
                out->crc = p->crc_rx;
                out->crc_calc = p->crc;
                out->raw_len = tp_frame_length_for(p->len);
                if (p->len > 0u) {
                    (void)memcpy(out->payload, p->payload, p->len);
                }
                /* Zero the tail so a consumer that reads past `len` cannot see
                 * stale bytes from the previous frame.  That class of bug is
                 * invisible in a test and confusing in the field. */
                if (p->len < (uint8_t)TP_PROTO_MAX_PAYLOAD) {
                    (void)memset(&out->payload[p->len], 0,
                                 (size_t)TP_PROTO_MAX_PAYLOAD - p->len);
                }
            }
            p->frames_ok++;
            p->last_frame_ms = now_ms;
            frame_restart(p);
            return TP_OK;
        }

        p->crc_errors++;
        p->resyncs++;
        /* Resume hunting from the *next* byte, not from the byte after the
         * candidate frame: a false header inside a payload must not swallow a
         * genuine frame that begins a few bytes later.  Consuming only the
         * header we already committed to is the cheapest correct answer. */
        frame_restart(p);
        return TP_ERR_CRC;

    default:
        frame_restart(p);
        return TP_ERR_NOT_READY;
    }
}

uint32_t tp_frame_parser_feed_buffer(tp_frame_parser_t *p, const uint8_t *data,
                                     uint32_t len, uint32_t now_ms,
                                     int (*handler)(const tp_frame_t *frame,
                                                    void *user),
                                     void *user)
{
    uint32_t i;
    uint32_t delivered = 0u;

    if ((p == NULL) || ((data == NULL) && (len > 0u))) {
        return 0u;
    }
    for (i = 0u; i < len; ++i) {
        tp_frame_t frame;
        if (tp_frame_parser_feed(p, data[i], now_ms, &frame) == TP_OK) {
            delivered++;
            if (handler != NULL) {
                if (handler(&frame, user) != 0) {
                    break;
                }
            }
        }
    }
    return delivered;
}

uint32_t tp_frame_encode(uint8_t *dst, uint32_t dst_size, uint8_t cmd,
                         const uint8_t *payload, uint8_t len)
{
    uint32_t total;
    uint16_t crc;
    uint32_t i;

    if (dst == NULL) {
        return 0u;
    }
    if (len > (uint8_t)TP_PROTO_MAX_PAYLOAD) {
        return 0u;
    }
    if ((payload == NULL) && (len > 0u)) {
        return 0u;
    }
    total = (uint32_t)len + TP_PROTO_FIXED_OVERHEAD;
    if (dst_size < total) {
        return 0u;
    }

    dst[0] = TP_PROTO_HDR0;
    dst[1] = TP_PROTO_HDR1;
    dst[2] = cmd;
    dst[3] = len;
    for (i = 0u; i < len; ++i) {
        dst[4u + i] = payload[i];
    }

    /* CRC covers CMD | LEN | PAYLOAD, i.e. everything from index 2 up to but not
     * including the checksum itself. */
    crc = tp_crc16_modbus(&dst[2], (size_t)len + 2u);
    dst[4u + len] = (uint8_t)(crc & 0x00FFu);
    dst[5u + len] = (uint8_t)((crc >> 8) & 0x00FFu);
    return total;
}

tp_status_t tp_frame_decode(const uint8_t *buf, uint32_t len, tp_frame_t *out)
{
    uint8_t  payload_len;
    uint16_t crc_rx;
    uint16_t crc_calc;

    if ((buf == NULL) || (out == NULL) || (len < TP_PROTO_FIXED_OVERHEAD)) {
        return TP_ERR_PARAM;
    }
    if ((buf[0] != TP_PROTO_HDR0) || (buf[1] != TP_PROTO_HDR1)) {
        return TP_ERR_PROTOCOL;
    }
    payload_len = buf[3];
    if (tp_frame_length_for(payload_len) != len) {
        return TP_ERR_PROTOCOL;
    }

    crc_calc = tp_crc16_modbus(&buf[2], (size_t)payload_len + 2u);
    crc_rx = (uint16_t)buf[4u + payload_len]
           | (uint16_t)((uint16_t)buf[5u + payload_len] << 8);
    if (crc_rx != crc_calc) {
        return TP_ERR_CRC;
    }

    out->cmd = buf[2];
    out->len = payload_len;
    out->crc = crc_rx;
    out->crc_calc = crc_calc;
    out->raw_len = len;
    if (payload_len > 0u) {
        (void)memcpy(out->payload, &buf[4], payload_len);
    }
    if (payload_len < (uint8_t)TP_PROTO_MAX_PAYLOAD) {
        (void)memset(&out->payload[payload_len], 0,
                     (size_t)TP_PROTO_MAX_PAYLOAD - payload_len);
    }
    return TP_OK;
}

tp_frame_state_t tp_frame_parser_state(const tp_frame_parser_t *p)
{
    return (p != NULL) ? p->state : TP_FRAME_WAIT_HDR0;
}
