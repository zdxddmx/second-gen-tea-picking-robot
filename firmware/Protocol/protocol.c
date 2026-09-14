/**
 * @file    protocol.c
 * @brief   Implementation of command dispatch and report encoding.
 * @copyright MIT License, see LICENSE.
 */

#include "protocol.h"

#include <string.h>

#include "compiler.h"
#include "config.h"
#include "crc16.h"
#include "logger.h"

tp_status_t tp_protocol_init(tp_protocol_t *proto, tp_uart_link_t *link,
                             const tp_protocol_ops_t *ops)
{
    if (proto == NULL) {
        return TP_ERR_PARAM;
    }
    (void)memset(proto, 0, sizeof(*proto));
    tp_frame_parser_init(&proto->parser);
    proto->link = link;
    if (ops != NULL) {
        proto->ops = *ops;
    }
    proto->link_alive = 0;
    proto->initialised = 1;
    LOG_INFO(TP_MOD_PROTO, "session ready: max payload %u B, timeout %u ms",
             (unsigned)TP_PROTO_MAX_PAYLOAD, (unsigned)TP_COMM_TIMEOUT_MS);
    return TP_OK;
}

tp_status_t tp_protocol_set_ops(tp_protocol_t *proto,
                                const tp_protocol_ops_t *ops)
{
    if ((proto == NULL) || (ops == NULL)) {
        return TP_ERR_PARAM;
    }
    proto->ops = *ops;
    return TP_OK;
}

tp_status_t tp_protocol_send_frame(tp_protocol_t *proto, uint8_t cmd,
                                   const uint8_t *payload, uint8_t len)
{
    uint8_t  buffer[TP_PROTO_MAX_FRAME];
    uint32_t total;
    tp_status_t st;

    if ((proto == NULL) || (proto->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    total = tp_frame_encode(buffer, sizeof(buffer), cmd, payload, len);
    if (total == 0u) {
        LOG_WARN(TP_MOD_PROTO, "encode failed: cmd 0x%02X len %u",
                 (unsigned)cmd, (unsigned)len);
        return TP_ERR_RANGE;
    }
    if (proto->link == NULL) {
        /* No transport: this is the unit-test path.  Count the frame so the
         * dispatcher's behaviour is still observable. */
        proto->frames_out++;
        return TP_OK;
    }
    st = tp_uart_write_frame(proto->link, buffer, total);
    if (st == TP_OK) {
        proto->frames_out++;
        proto->last_tx_ms = proto->now_ms;
    }
    return st;
}

tp_status_t tp_protocol_send_ack(tp_protocol_t *proto, uint8_t acked_cmd)
{
    tp_payload_ack_t p;

    p.acked_cmd = acked_cmd;
    return tp_protocol_send_frame(proto, TP_CMD_ACK, (const uint8_t *)&p, 1u);
}

tp_status_t tp_protocol_send_nack(tp_protocol_t *proto, uint8_t ref_cmd,
                                  tp_status_t status, uint8_t detail)
{
    uint8_t buf[3];

    buf[0] = ref_cmd;
    /* Negate so that a negative internal status becomes a positive wire code;
     * zero would be indistinguishable from "no error". */
    buf[1] = (uint8_t)(-(int)status);
    buf[2] = detail;
    proto->nacks_sent++;
    return tp_protocol_send_frame(proto, TP_CMD_NACK, buf, 3u);
}

tp_status_t tp_protocol_send_heartbeat_ack(tp_protocol_t *proto, uint32_t seq)
{
    uint8_t buf[4];

    tp_le_put_u32(buf, seq);
    return tp_protocol_send_frame(proto, TP_CMD_HEARTBEAT_ACK, buf, 4u);
}

uint32_t tp_protocol_encode_status(uint8_t *dst, const tp_payload_status_t *s)
{
    if ((dst == NULL) || (s == NULL)) {
        return 0u;
    }
    dst[0] = s->state;
    dst[1] = s->fault_code;
    tp_le_put_i32(&dst[2], s->pos_mrad);
    tp_le_put_i16(&dst[6], s->vel_mrad_s);
    tp_le_put_i16(&dst[8], s->force_mn);
    tp_le_put_i16(&dst[10], s->pwm_permille);
    tp_le_put_i16(&dst[12], s->current_ma);
    tp_le_put_u16(&dst[14], s->cycle_count);
    return 16u;
}

tp_status_t tp_protocol_decode_set_target(const uint8_t *src,
                                          tp_payload_set_target_t *out)
{
    if ((src == NULL) || (out == NULL)) {
        return TP_ERR_PARAM;
    }
    out->x_mm = tp_le_get_i16(&src[0]);
    out->y_mm = tp_le_get_i16(&src[2]);
    out->z_mm = tp_le_get_i16(&src[4]);
    out->confidence = src[6];
    out->reserved = src[7];
    out->target_id = tp_le_get_u16(&src[8]);
    return TP_OK;
}

tp_status_t tp_protocol_send_status(tp_protocol_t *proto,
                                    const tp_payload_status_t *s)
{
    /* 16 bytes: u8 state + u8 fault + i32 position + four i16 fields + u16
     * cycle_count.  The first version declared 15 and sent 15, which silently
     * truncated the high byte of cycle_count -- the encoder wrote 16 bytes into
     * the buffer and the transmit call was told 15.  The status report is what a
     * monitoring tool renders as the whole robot, so a truncation there is not a
     * cosmetic problem. */
    uint8_t buffer[16];

    if (s == NULL) {
        return TP_ERR_PARAM;
    }
    if (tp_protocol_encode_status(buffer, s) != 16u) {
        return TP_ERR_PARAM;
    }
    proto->responses_sent++;
    return tp_protocol_send_frame(proto, TP_CMD_STATUS_RPT, buffer, 16u);
}

tp_status_t tp_protocol_send_sensor(tp_protocol_t *proto,
                                    const tp_payload_sensor_t *s)
{
    /* 14 bytes: i32 position + five i16/u16 fields.  As with the status report,
     * the declared buffer and the transmitted length were both one byte short of
     * what the encoder writes. */
    uint8_t buffer[14];

    if (s == NULL) {
        return TP_ERR_PARAM;
    }
    tp_le_put_i32(&buffer[0], s->pos_mrad);
    tp_le_put_i16(&buffer[4], s->vel_mrad_s);
    tp_le_put_i16(&buffer[6], s->force_mn);
    tp_le_put_u16(&buffer[8], s->adc_raw);
    tp_le_put_i16(&buffer[10], s->current_ma);
    tp_le_put_i16(&buffer[12], s->temp_c_x10);
    proto->responses_sent++;
    return tp_protocol_send_frame(proto, TP_CMD_SENSOR_RPT, buffer, 14u);
}

tp_status_t tp_protocol_send_pid(tp_protocol_t *proto,
                                 const tp_payload_pid_t *p)
{
    uint8_t buffer[13];

    if (p == NULL) {
        return TP_ERR_PARAM;
    }
    buffer[0] = p->loop_id;
    /* Floats are sent as raw IEEE-754 bytes.  The alternative - fixed point -
     * would avoid any representation question, but gains have a very wide
     * dynamic range across the three loops (12.0 vs 0.015) and a fixed point
     * encoding for all of them would either lose resolution or waste bytes. */
    (void)memcpy(&buffer[1], &p->kp, 4u);
    (void)memcpy(&buffer[5], &p->ki, 4u);
    (void)memcpy(&buffer[9], &p->kd, 4u);
    proto->responses_sent++;
    return tp_protocol_send_frame(proto, TP_CMD_PID_RPT, buffer, 13u);
}

tp_status_t tp_protocol_send_fault(tp_protocol_t *proto,
                                   const tp_payload_fault_t *f)
{
    uint8_t buffer[6];

    if (f == NULL) {
        return TP_ERR_PARAM;
    }
    buffer[0] = f->fault_code;
    tp_le_put_u32(&buffer[1], f->timestamp_ms);
    buffer[5] = f->detail;
    proto->responses_sent++;
    return tp_protocol_send_frame(proto, TP_CMD_FAULT_RPT, buffer, 6u);
}

tp_status_t tp_protocol_send_telemetry(tp_protocol_t *proto,
                                       const tp_payload_telemetry_t *t)
{
    uint8_t buffer[16];

    if (t == NULL) {
        return TP_ERR_PARAM;
    }
    tp_le_put_u32(&buffer[0], t->t_ms);
    tp_le_put_i32(&buffer[4], t->pos_mrad);
    tp_le_put_i16(&buffer[8], t->vel_mrad_s);
    tp_le_put_i16(&buffer[10], t->force_mn);
    tp_le_put_i16(&buffer[12], t->pwm_permille);
    buffer[14] = t->state;
    buffer[15] = t->fault;
    proto->responses_sent++;
    return tp_protocol_send_frame(proto, TP_CMD_TELEMETRY, buffer, 16u);
}

/**
 * @brief Dispatch a command to the application callback table.
 *
 * @param[in,out] proto  Session.
 * @param[in]     frame  Decoded frame.
 * @return Handler status, or @ref TP_ERR_UNSUPPORTED.
 */
static tp_status_t protocol_dispatch_command(tp_protocol_t *proto,
                                             const tp_frame_t *frame)
{
    tp_protocol_ops_t *ops = &proto->ops;

    switch (frame->cmd) {
    case TP_CMD_HEARTBEAT:
        if ((ops->on_heartbeat == NULL) || (frame->len < 4u)) {
            return TP_ERR_UNSUPPORTED;
        }
        {
            uint32_t seq = tp_le_get_u32(frame->payload);
            /* Sequence-gap accounting.  An unsigned subtraction handles the
             * wrap correctly, and a gap larger than a plausible window (say
             * 1000) means the host restarted rather than that 1000 frames were
             * lost - counting that as loss would report a false 100 % outage. */
            if (proto->hb_received > 0u) {
                uint32_t delta = seq - proto->hb_seq_last;
                if ((delta > 1u) && (delta < 1000u)) {
                    proto->hb_seq_gap += (delta - 1u);
                }
            }
            proto->hb_seq_last = seq;
            proto->hb_received++;
            return ops->on_heartbeat(ops->user, seq);
        }

    case TP_CMD_SET_TARGET:
        if ((ops->on_set_target == NULL) || (frame->len < 10u)) {
            return TP_ERR_UNSUPPORTED;
        }
        {
            tp_payload_set_target_t t;
            tp_protocol_decode_set_target(frame->payload, &t);
            return ops->on_set_target(ops->user, &t);
        }

    case TP_CMD_START_PICK:
        if ((ops->on_start_pick == NULL) || (frame->len < 2u)) {
            return TP_ERR_UNSUPPORTED;
        }
        return ops->on_start_pick(ops->user, tp_le_get_u16(frame->payload));

    case TP_CMD_STOP:
        if (ops->on_stop == NULL) {
            return TP_ERR_UNSUPPORTED;
        }
        return ops->on_stop(ops->user);

    case TP_CMD_RESET:
        if (ops->on_reset == NULL) {
            return TP_ERR_UNSUPPORTED;
        }
        return ops->on_reset(ops->user);

    case TP_CMD_GET_STATUS:
        if (ops->on_get_status == NULL) {
            return TP_ERR_UNSUPPORTED;
        }
        {
            tp_payload_status_t s;
            tp_status_t st = ops->on_get_status(ops->user, &s);
            if (st == TP_OK) {
                (void)tp_protocol_send_status(proto, &s);
            }
            return st;
        }

    case TP_CMD_GET_SENSOR:
        if (ops->on_get_sensor == NULL) {
            return TP_ERR_UNSUPPORTED;
        }
        {
            tp_payload_sensor_t s;
            tp_status_t st = ops->on_get_sensor(ops->user, &s);
            if (st == TP_OK) {
                (void)tp_protocol_send_sensor(proto, &s);
            }
            return st;
        }

    case TP_CMD_GET_FAULT:
        if (ops->on_get_fault == NULL) {
            return TP_ERR_UNSUPPORTED;
        }
        {
            tp_payload_fault_t f;
            tp_status_t st = ops->on_get_fault(ops->user, &f);
            if (st == TP_OK) {
                (void)tp_protocol_send_fault(proto, &f);
            }
            return st;
        }

    case TP_CMD_SET_PID:
        if ((ops->on_set_pid == NULL) || (frame->len < 13u)) {
            return TP_ERR_UNSUPPORTED;
        }
        {
            tp_payload_pid_t p;
            p.loop_id = frame->payload[0];
            (void)memcpy(&p.kp, &frame->payload[1], 4u);
            (void)memcpy(&p.ki, &frame->payload[5], 4u);
            (void)memcpy(&p.kd, &frame->payload[9], 4u);
            if (p.loop_id >= (uint8_t)TP_PID_LOOP_COUNT) {
                return TP_ERR_RANGE;
            }
            return ops->on_set_pid(ops->user, &p);
        }

    case TP_CMD_GET_PID:
        if ((ops->on_get_pid == NULL) || (frame->len < 1u)) {
            return TP_ERR_UNSUPPORTED;
        }
        {
            tp_payload_pid_t p;
            tp_status_t st;
            if (frame->payload[0] >= (uint8_t)TP_PID_LOOP_COUNT) {
                return TP_ERR_RANGE;
            }
            st = ops->on_get_pid(ops->user, frame->payload[0], &p);
            if (st == TP_OK) {
                (void)tp_protocol_send_pid(proto, &p);
            }
            return st;
        }

    case TP_CMD_SET_FORCE_TARGET:
        if ((ops->on_set_force_target == NULL) || (frame->len < 2u)) {
            return TP_ERR_UNSUPPORTED;
        }
        return ops->on_set_force_target(ops->user, tp_le_get_u16(frame->payload));

    case TP_CMD_SET_MODE:
        if ((ops->on_set_mode == NULL) || (frame->len < 1u)) {
            return TP_ERR_UNSUPPORTED;
        }
        return ops->on_set_mode(ops->user, frame->payload[0]);

    case TP_CMD_EMERGENCY_STOP:
        if (ops->on_emergency_stop == NULL) {
            return TP_ERR_UNSUPPORTED;
        }
        return ops->on_emergency_stop(ops->user);

    case TP_CMD_CLEAR_FAULT:
        if (ops->on_clear_fault == NULL) {
            return TP_ERR_UNSUPPORTED;
        }
        return ops->on_clear_fault(ops->user);

    case TP_CMD_CALIBRATE_FORCE:
        if ((ops->on_calibrate_force == NULL) || (frame->len < 1u)) {
            return TP_ERR_UNSUPPORTED;
        }
        return ops->on_calibrate_force(ops->user, frame->payload[0]);

    default:
        /* Includes every report code arriving in the host->robot direction,
         * which is a configuration error on the far end rather than a link
         * fault.  NACKing it makes the mistake visible. */
        return TP_ERR_NOT_FOUND;
    }
}

tp_status_t tp_protocol_dispatch(tp_protocol_t *proto, const tp_frame_t *frame)
{
    tp_status_t st;

    if ((proto == NULL) || (frame == NULL) || (proto->initialised == 0)) {
        return TP_ERR_PARAM;
    }

    st = protocol_dispatch_command(proto, frame);
    if (st == TP_OK) {
        proto->commands_handled++;
        proto->link_alive = 1;
        proto->last_rx_ms = proto->now_ms;
        return TP_OK;
    }

    proto->commands_rejected++;
    if (st != TP_ERR_UNSUPPORTED) {
        LOG_WARN(TP_MOD_PROTO, "cmd 0x%02X rejected: %s",
                 (unsigned)frame->cmd, tp_status_desc(st));
    } else {
        LOG_DEBUG(TP_MOD_PROTO, "cmd 0x%02X has no handler", (unsigned)frame->cmd);
    }
    if (proto->link != NULL) {
        (void)tp_protocol_send_nack(proto, frame->cmd, st, frame->len);
    }
    return st;
}

uint32_t tp_protocol_feed(tp_protocol_t *proto, const uint8_t *data,
                          uint32_t len, uint32_t now_ms)
{
    uint32_t i;
    uint32_t handled = 0u;

    if ((proto == NULL) || (proto->initialised == 0)) {
        return 0u;
    }
    proto->now_ms = now_ms;

    for (i = 0u; i < len; ++i) {
        tp_frame_t frame;
        tp_status_t st = tp_frame_parser_feed(&proto->parser, data[i], now_ms,
                                              &frame);
        if (st == TP_OK) {
            proto->frames_in++;
            /*
             * A heartbeat must be answered even if its handler returned an
             * error, otherwise the host's own link supervision would declare
             * the robot dead while the robot is happily reporting a bad
             * command.  Keeping the link alive is a transport-level duty and
             * does not depend on the application liking the payload.
             */
            if (frame.cmd == TP_CMD_HEARTBEAT) {
                proto->link_alive = 1;
                proto->last_rx_ms = now_ms;
            }
            (void)tp_protocol_dispatch(proto, &frame);
            handled++;
        }
    }
    return handled;
}

uint32_t tp_protocol_service(tp_protocol_t *proto, uint32_t now_ms)
{
    uint8_t  buffer[128];
    uint32_t total = 0u;
    uint32_t n;

    if ((proto == NULL) || (proto->initialised == 0)) {
        return 0u;
    }
    proto->now_ms = now_ms;

    if (proto->link != NULL) {
        /* Drain everything currently buffered, in bounded chunks, so a long
         * burst cannot monopolise the communication task. */
        while (total < 1024u) {
            n = tp_uart_read(proto->link, buffer, sizeof(buffer));
            if (n == 0u) {
                break;
            }
            (void)tp_protocol_feed(proto, buffer, n, now_ms);
            total += n;
        }
    }

    /* --- link supervision ---------------------------------------------- */
    if (proto->last_rx_ms != 0u) {
        uint32_t silent = now_ms - proto->last_rx_ms;
        if (silent > TP_COMM_TIMEOUT_MS) {
            if (proto->link_alive != 0) {
                proto->link_alive = 0;
                LOG_WARN(TP_MOD_PROTO, "host link lost after %lu ms of silence",
                         (unsigned long)silent);
                if (proto->ops.on_link_lost != NULL) {
                    proto->ops.on_link_lost(proto->ops.user, silent);
                }
            }
        }
    }
    return total;
}

int tp_protocol_link_alive(const tp_protocol_t *proto)
{
    return (proto != NULL) ? proto->link_alive : 0;
}

float tp_protocol_loss_estimate(const tp_protocol_t *proto)
{
    uint32_t expected;
    float loss;

    if ((proto == NULL) || (proto->hb_received < 5u)) {
        /* Too few samples: reporting a number here would be noise presented as
         * measurement. */
        return 0.0f;
    }
    expected = proto->hb_received + proto->hb_seq_gap;
    if (expected == 0u) {
        return 0.0f;
    }
    loss = (float)proto->hb_seq_gap / (float)expected;
    if (loss > 1.0f) {
        loss = 1.0f;
    }
    return loss;
}
