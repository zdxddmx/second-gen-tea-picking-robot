/**
 * @file    protocol.h
 * @brief   Command dispatch, session state and report encoding.
 *
 * @details
 * # The layering trick this file is built around
 * The protocol layer must *not* know what a robot is.  If `protocol.c`
 * included `robot_fsm.h` and called `robot_fsm_start_pick()`, then:
 *
 *   - the protocol could not be tested without a robot,
 *   - adding a command would mean editing two layers, and
 *   - the layering diagram in the README would be a lie.
 *
 * Instead the protocol owns a **callback table** (@ref tp_protocol_ops_t) that
 * the application fills in at start-up.  Protocol -> callback -> application,
 * with the dependency arrow pointing *upwards at run time* and *downwards at
 * compile time*, which is exactly what dependency inversion is for.
 *
 * The practical payoff is in the tests: `test_protocol.c` installs a table of
 * counting stubs and asserts that a `SET_TARGET` frame produces exactly one
 * `on_set_target` call with the right decoded millimetres, with no robot and no
 * hardware involved.
 *
 * # Session supervision
 * Every accepted frame updates @ref tp_protocol_t::last_rx_ms.  If the host
 * goes quiet for longer than @ref TP_COMM_TIMEOUT_MS the application is told
 * through @ref tp_protocol_ops_t::on_link_lost, and motion must not continue on
 * stale target data — a vision target is a *measurement with a timestamp*, not
 * a permanent truth.
 *
 * # Packet-loss accounting
 * The host stamps every command stream with a sequence number in the heartbeat.
 * By tracking the gap between consecutive sequence numbers the firmware can
 * report a genuine link-quality figure to the operator rather than just "some
 * frames failed their CRC".
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_PROTOCOL_H
#define TP_PROTOCOL_H

#include <stdint.h>

#include "frame.h"
#include "protocol_types.h"
#include "status.h"
#include "uart_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Application callbacks invoked by the dispatcher.
 *
 * Every member may be NULL, in which case the corresponding command is
 * answered with `NACK` and @ref TP_ERR_UNSUPPORTED.  This is deliberate: a
 * partially implemented build still behaves correctly on the wire instead of
 * silently dropping frames.
 *
 * A handler returns @ref TP_OK on success.  Any other value produces a `NACK`
 * carrying that status, so error reporting is automatic and cannot be
 * forgotten.
 */
typedef struct {
    void *user; /**< Opaque pointer passed as the first argument to each hook. */

    /** @brief `HEARTBEAT`: host->robot liveliness with a sequence number. */
    tp_status_t (*on_heartbeat)(void *user, uint32_t seq);
    /** @brief `SET_TARGET`: a new vision target. */
    tp_status_t (*on_set_target)(void *user, const tp_payload_set_target_t *t);
    /** @brief `START_PICK`: begin a cycle. */
    tp_status_t (*on_start_pick)(void *user, uint16_t target_id);
    /** @brief `STOP`: controlled stop, stay enabled. */
    tp_status_t (*on_stop)(void *user);
    /** @brief `RESET`: clear faults and re-home. */
    tp_status_t (*on_reset)(void *user);
    /** @brief `GET_STATUS`: fill a status report. */
    tp_status_t (*on_get_status)(void *user, tp_payload_status_t *out);
    /** @brief `GET_SENSOR`: fill a raw sensor report. */
    tp_status_t (*on_get_sensor)(void *user, tp_payload_sensor_t *out);
    /** @brief `GET_FAULT`: fill a fault report. */
    tp_status_t (*on_get_fault)(void *user, tp_payload_fault_t *out);
    /** @brief `SET_PID`: change gains of one loop. */
    tp_status_t (*on_set_pid)(void *user, const tp_payload_pid_t *p);
    /** @brief `GET_PID`: read gains of one loop. */
    tp_status_t (*on_get_pid)(void *user, uint8_t loop_id, tp_payload_pid_t *out);
    /** @brief `SET_FORCE_TARGET`: change the grip force setpoint. */
    tp_status_t (*on_set_force_target)(void *user, uint16_t force_mn);
    /** @brief `SET_MODE`: change the operating mode. */
    tp_status_t (*on_set_mode)(void *user, uint8_t mode);
    /** @brief `EMERGENCY_STOP`: kill the drives now. */
    tp_status_t (*on_emergency_stop)(void *user);
    /** @brief `CLEAR_FAULT`: clear a latched fault if it has gone. */
    tp_status_t (*on_clear_fault)(void *user);
    /** @brief `CALIBRATE_FORCE`: run a sensor calibration step. */
    tp_status_t (*on_calibrate_force)(void *user, uint8_t kind);
    /** @brief Called when the host link has been silent for too long. */
    void        (*on_link_lost)(void *user, uint32_t silent_ms);
} tp_protocol_ops_t;

/**
 * @brief One protocol session.
 */
typedef struct {
    tp_frame_parser_t parser;   /**< Streaming frame decoder.                 */
    tp_uart_link_t   *link;     /**< Transport, or NULL for a pure test.       */
    tp_protocol_ops_t ops;      /**< Application callbacks.                    */

    uint32_t last_rx_ms;        /**< Timestamp of the last valid frame.        */
    uint32_t last_tx_ms;        /**< Timestamp of the last transmission.       */
    uint32_t now_ms;            /**< Latest timestamp supplied by the caller.  */

    uint32_t frames_in;         /**< Valid inbound frames.                     */
    uint32_t frames_out;        /**< Frames transmitted.                       */
    uint32_t commands_handled;  /**< Commands dispatched successfully.         */
    uint32_t commands_rejected; /**< Commands answered with NACK.              */
    uint32_t nacks_sent;        /**< NACK frames transmitted.                  */
    uint32_t responses_sent;    /**< Report frames transmitted.                */

    uint32_t hb_seq_last;       /**< Last heartbeat sequence number seen.      */
    uint32_t hb_seq_gap;        /**< Sum of missing heartbeat sequence numbers.*/
    uint32_t hb_received;       /**< Heartbeats received.                      */
    uint32_t tx_seq;            /**< Sequence stamped on outbound heartbeats.  */

    int      link_alive;        /**< Non-zero while the host is inside timeout.*/
    int      initialised;       /**< Non-zero after a successful init.         */
} tp_protocol_t;

/**
 * @brief Initialise a protocol session.
 *
 * @param[in,out] proto  Instance.
 * @param[in]     link   Transport to use, or NULL when the caller will feed
 *                       bytes directly (the unit-test path).
 * @param[in]     ops    Callback table; copied by value so the caller's struct
 *                       may live on the stack.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM.
 */
tp_status_t tp_protocol_init(tp_protocol_t *proto, tp_uart_link_t *link,
                             const tp_protocol_ops_t *ops);

/**
 * @brief Replace the callback table at run time.
 *
 * @param[in,out] proto  Instance.
 * @param[in]     ops    New table.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_protocol_set_ops(tp_protocol_t *proto,
                                const tp_protocol_ops_t *ops);

/**
 * @brief Service the link: pull bytes, parse frames, dispatch commands.
 *
 * This is the only entry point the communication task needs.  It drains the
 * whole RX buffer each call so that a burst of frames is handled in one pass,
 * and enforces the link timeout.
 *
 * @param[in,out] proto    Instance.
 * @param[in]     now_ms   Current timestamp.
 * @return Number of commands dispatched in this call.
 */
uint32_t tp_protocol_service(tp_protocol_t *proto, uint32_t now_ms);

/**
 * @brief Feed raw bytes directly, bypassing the transport.
 *
 * @param[in,out] proto   Instance.
 * @param[in]     data    Bytes.
 * @param[in]     len     Number of bytes.
 * @param[in]     now_ms  Timestamp.
 * @return Number of commands dispatched.
 */
uint32_t tp_protocol_feed(tp_protocol_t *proto, const uint8_t *data,
                          uint32_t len, uint32_t now_ms);

/**
 * @brief Dispatch one already-decoded frame.
 *
 * Exposed so a test can exercise the dispatcher without the byte-level parser,
 * and so an alternative transport (CAN, or an internal call) can reuse the
 * command handling.
 *
 * @param[in,out] proto  Instance.
 * @param[in]     frame  Decoded frame.
 * @return @ref TP_OK when the command was handled, otherwise the handler's
 *         status (or @ref TP_ERR_UNSUPPORTED for an unknown command).
 */
tp_status_t tp_protocol_dispatch(tp_protocol_t *proto, const tp_frame_t *frame);

/**
 * @brief Whether the host link is currently considered alive.
 *
 * @param[in] proto  Instance.
 * @return Non-zero while inside the timeout window.
 */
int tp_protocol_link_alive(const tp_protocol_t *proto);

/**
 * @brief Estimated packet loss as a fraction in [0, 1].
 *
 * Derived from missing heartbeat sequence numbers.  Returns 0.0 when too few
 * heartbeats have been seen for the estimate to mean anything — reporting a
 * spurious 100 % loss at boot would be worse than reporting nothing.
 *
 * @param[in] proto  Instance.
 * @return Loss fraction.
 */
float tp_protocol_loss_estimate(const tp_protocol_t *proto);

/* =========================================================================
 * Report encoders
 * ========================================================================= */

/**
 * @brief Send an `ACK`.
 *
 * @param[in,out] proto      Instance.
 * @param[in]     acked_cmd  Command being acknowledged.
 * @return @ref TP_OK or a transport error.
 */
tp_status_t tp_protocol_send_ack(tp_protocol_t *proto, uint8_t acked_cmd);

/**
 * @brief Send a `NACK`.
 *
 * @param[in,out] proto     Instance.
 * @param[in]     ref_cmd   Command that failed.
 * @param[in]     status    Why it failed.
 * @param[in]     detail    Command-specific extra information.
 * @return @ref TP_OK or a transport error.
 */
tp_status_t tp_protocol_send_nack(tp_protocol_t *proto, uint8_t ref_cmd,
                                  tp_status_t status, uint8_t detail);

/**
 * @brief Send a `HEARTBEAT_ACK`.
 *
 * @param[in,out] proto  Instance.
 * @param[in]     seq    Sequence number being echoed.
 * @return @ref TP_OK or a transport error.
 */
tp_status_t tp_protocol_send_heartbeat_ack(tp_protocol_t *proto, uint32_t seq);

/**
 * @brief Send a `STATUS_RPT`.
 *
 * @param[in,out] proto  Instance.
 * @param[in]     s      Status payload.
 * @return @ref TP_OK or a transport error.
 */
tp_status_t tp_protocol_send_status(tp_protocol_t *proto,
                                    const tp_payload_status_t *s);

/**
 * @brief Send a `SENSOR_RPT`.
 *
 * @param[in,out] proto  Instance.
 * @param[in]     s      Sensor payload.
 * @return @ref TP_OK or a transport error.
 */
tp_status_t tp_protocol_send_sensor(tp_protocol_t *proto,
                                    const tp_payload_sensor_t *s);

/**
 * @brief Send a `PID_RPT`.
 *
 * @param[in,out] proto  Instance.
 * @param[in]     p      PID payload.
 * @return @ref TP_OK or a transport error.
 */
tp_status_t tp_protocol_send_pid(tp_protocol_t *proto,
                                 const tp_payload_pid_t *p);

/**
 * @brief Send a `FAULT_RPT`.
 *
 * @param[in,out] proto  Instance.
 * @param[in]     f      Fault payload.
 * @return @ref TP_OK or a transport error.
 */
tp_status_t tp_protocol_send_fault(tp_protocol_t *proto,
                                   const tp_payload_fault_t *f);

/**
 * @brief Send a `TELEMETRY` frame.
 *
 * @param[in,out] proto  Instance.
 * @param[in]     t      Telemetry payload.
 * @return @ref TP_OK or a transport error.
 */
tp_status_t tp_protocol_send_telemetry(tp_protocol_t *proto,
                                       const tp_payload_telemetry_t *t);

/**
 * @brief Encode a payload into bytes and transmit it as a frame.
 *
 * The generic path used by every `send_*` function, exposed because the
 * application sends a few messages that have no dedicated helper.
 *
 * @param[in,out] proto    Instance.
 * @param[in]     cmd      Command code.
 * @param[in]     payload  Payload bytes.
 * @param[in]     len      Payload length.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or a transport error.
 */
tp_status_t tp_protocol_send_frame(tp_protocol_t *proto, uint8_t cmd,
                                   const uint8_t *payload, uint8_t len);

/**
 * @brief Serialise a status payload into wire bytes.
 *
 * Separated from the send path so a unit test can assert on the exact byte
 * sequence without a transport.
 *
 * @param[out] dst  Destination, at least 15 bytes.
 * @param[in]  s    Payload.
 * @return Bytes written.
 */
uint32_t tp_protocol_encode_status(uint8_t *dst, const tp_payload_status_t *s);

/**
 * @brief Deserialise a `SET_TARGET` payload.
 *
 * @param[in]  src  Payload bytes, at least 10.
 * @param[out] out  Decoded structure.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM when a pointer is NULL.
 */
tp_status_t tp_protocol_decode_set_target(const uint8_t *src,
                                          tp_payload_set_target_t *out);

#ifdef __cplusplus
}
#endif

#endif /* TP_PROTOCOL_H */
