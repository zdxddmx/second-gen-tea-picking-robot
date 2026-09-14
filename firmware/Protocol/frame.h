/**
 * @file    frame.h
 * @brief   Byte-stream frame codec: the parser state machine and the encoder.
 *
 * @details
 * # Why a streaming parser and not "read a whole frame"
 * A UART delivers bytes, not frames.  The firmware can never assume that a
 * `read()` returned exactly one message: it may return half a frame, three
 * frames, or a frame plus the first two bytes of the next one.  A parser that
 * tries to read a whole frame in one go therefore works on the bench (where
 * messages arrive slowly and alone) and fails in the field (where telemetry and
 * commands interleave).
 *
 * This parser is an explicit **state machine over a byte stream**: bytes go in
 * one at a time, complete frames come out.  It is a pure function of its input
 * — it touches no hardware — which is exactly why the protocol tests can drive
 * it with arbitrary byte sequences, including deliberately hostile ones.
 *
 * # Resynchronisation
 * The header pattern `AA 55` is the only synchronisation point.  The parser:
 *
 *   - discards bytes while searching for `AA`, then for `55`;
 *   - if the second byte is not `55`, it re-examines it as a possible *new*
 *     `AA` (so `AA AA 55` resynchronises correctly rather than losing a frame);
 *   - validates `LEN` against @ref TP_PROTO_MAX_PAYLOAD before allocating
 *     anything, so a corrupt length byte cannot make the parser wait for 200
 *     bytes that will never come — it is rejected immediately;
 *   - verifies the CRC and, on failure, resumes searching from the byte after
 *     the false header rather than consuming the whole candidate frame.  This
 *     matters: a false header inside a payload would otherwise consume a real
 *     frame that started a few bytes later.
 *
 * # Overflow policy
 * Frames longer than @ref TP_PROTO_MAX_PAYLOAD are not supported and are
 * counted, not truncated.  A truncated frame is a frame with a valid CRC over
 * the wrong data — the worst possible outcome — so refusing is the only safe
 * choice.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_FRAME_H
#define TP_FRAME_H

#include <stddef.h>
#include <stdint.h>

#include "protocol_types.h"
#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Parser states, exposed so a test can assert on the resync path. */
typedef enum {
    TP_FRAME_WAIT_HDR0 = 0, /**< Looking for 0xAA.                          */
    TP_FRAME_WAIT_HDR1,     /**< Saw 0xAA, looking for 0x55.                */
    TP_FRAME_WAIT_CMD,      /**< Reading the command byte.                  */
    TP_FRAME_WAIT_LEN,      /**< Reading the length byte.                   */
    TP_FRAME_WAIT_PAYLOAD,  /**< Accumulating the payload.                  */
    TP_FRAME_WAIT_CRC_LO,   /**< Reading the CRC low byte.                  */
    TP_FRAME_WAIT_CRC_HI    /**< Reading the CRC high byte.                 */
} tp_frame_state_t;

/**
 * @brief A decoded frame.
 */
typedef struct {
    uint8_t cmd;                          /**< Command or report code.       */
    uint8_t len;                          /**< Payload length.               */
    uint8_t payload[TP_PROTO_MAX_PAYLOAD];/**< Payload bytes.                */
    uint16_t crc;                         /**< CRC as received.              */
    uint16_t crc_calc;                    /**< CRC as computed, for diagnosis.*/
    uint32_t raw_len;                     /**< Total frame length on the wire.*/
} tp_frame_t;

/**
 * @brief Streaming parser state and statistics.
 *
 * The statistics are not decoration: @ref crc_errors climbing means
 * interference or a baud-rate mismatch, @ref length_errors means a framing
 * problem or a version mismatch with the peer, and @ref resyncs means frames
 * are being truncated somewhere upstream.  Each points at a different fault, so
 * a single "bad frames" counter would be useless.
 */
typedef struct {
    tp_frame_state_t state;        /**< Current parser state.               */
    uint8_t  cmd;                  /**< Command byte being assembled.       */
    uint8_t  len;                  /**< Expected payload length.            */
    uint8_t  index;                /**< Payload bytes received so far.      */
    uint16_t crc;                  /**< CRC accumulator.                    */
    uint16_t crc_rx;               /**< CRC bytes received.                 */
    uint8_t  crc_bytes;            /**< CRC bytes collected, 0..2.          */
    uint8_t  payload[TP_PROTO_MAX_PAYLOAD]; /**< Payload under construction. */

    uint32_t frames_ok;            /**< Frames delivered.                   */
    uint32_t crc_errors;           /**< Frames rejected on CRC.             */
    uint32_t length_errors;        /**< LEN rejected as too large.          */
    uint32_t resyncs;              /**< Times the parser restarted a header.*/
    uint32_t bytes_discarded;      /**< Bytes thrown away while hunting.    */
    uint32_t bytes_consumed;       /**< Total bytes fed in.                 */
    uint32_t last_frame_ms;        /**< Timestamp of the last good frame.   */
} tp_frame_parser_t;

/**
 * @brief Reset a parser to its initial state.
 *
 * Statistics are cleared too.  Use @ref tp_frame_parser_reset_stats when only
 * the counters should go.
 *
 * @param[in,out] p  Parser.  A NULL pointer is ignored.
 */
void tp_frame_parser_init(tp_frame_parser_t *p);

/**
 * @brief Clear the counters but keep the parser mid-frame.
 *
 * @param[in,out] p  Parser.
 */
void tp_frame_parser_reset_stats(tp_frame_parser_t *p);

/**
 * @brief Feed one byte to the parser.
 *
 * @param[in,out] p      Parser.
 * @param[in]     byte   Byte received.
 * @param[in]     now_ms Current timestamp, stored when a frame completes.
 * @param[out]    out    Receives the frame when one completes; untouched
 *                       otherwise.  May be NULL if the caller only wants the
 *                       return value.
 * @return @ref TP_OK when @p out now holds a complete, CRC-valid frame;
 *         @ref TP_ERR_NOT_READY when more bytes are needed; @ref TP_ERR_CRC
 *         when a frame completed but failed its checksum;
 *         @ref TP_ERR_OVERFLOW when a length byte exceeded the limit.
 */
tp_status_t tp_frame_parser_feed(tp_frame_parser_t *p, uint8_t byte,
                                 uint32_t now_ms, tp_frame_t *out);

/**
 * @brief Feed a whole buffer.
 *
 * @param[in,out] p        Parser.
 * @param[in]     data     Bytes received.
 * @param[in]     len      Number of bytes.
 * @param[in]     now_ms   Timestamp applied to every frame completed by this
 *                         call.  A real caller would refine this per byte; the
 *                         approximation is documented because a burst of bytes
 *                         from one DMA transfer really does arrive together.
 * @param[in]     handler  Called once per complete frame.  Return non-zero to
 *                         stop processing the rest of the buffer.
 * @param[in]     user     Opaque pointer forwarded to @p handler.
 * @return Number of frames delivered.
 */
uint32_t tp_frame_parser_feed_buffer(tp_frame_parser_t *p, const uint8_t *data,
                                     uint32_t len, uint32_t now_ms,
                                     int (*handler)(const tp_frame_t *frame,
                                                    void *user),
                                     void *user);

/**
 * @brief Encode a frame into a caller-supplied buffer.
 *
 * @param[out] dst      Destination, at least `len + 6` bytes.
 * @param[in]  dst_size Capacity of @p dst.
 * @param[in]  cmd      Command byte.
 * @param[in]  payload  Payload bytes; may be NULL when @p len is 0.
 * @param[in]  len      Payload length.
 * @return Total frame length written, or 0 on a parameter error or when
 *         @p dst is too small to hold the frame.
 */
uint32_t tp_frame_encode(uint8_t *dst, uint32_t dst_size, uint8_t cmd,
                         const uint8_t *payload, uint8_t len);

/**
 * @brief Verify a candidate frame in place.
 *
 * @param[in]  buf  Complete frame bytes.
 * @param[in]  len  Candidate length; must equal `6 + LEN`.
 * @param[out] out  Receives the decoded frame on success; untouched on failure.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, @ref TP_ERR_CRC, or
 *         @ref TP_ERR_PROTOCOL when the header or length is wrong.
 */
tp_status_t tp_frame_decode(const uint8_t *buf, uint32_t len, tp_frame_t *out);

/**
 * @brief Total length a frame with @p payload_len bytes will occupy.
 *
 * @param[in] payload_len  Payload length.
 * @return Frame length in bytes, or 0 if @p payload_len exceeds the limit.
 */
uint32_t tp_frame_length_for(uint8_t payload_len);

/**
 * @brief Parser state, for diagnostics and for unit-test assertions.
 *
 * @param[in] p  Parser.
 * @return Current state.
 */
tp_frame_state_t tp_frame_parser_state(const tp_frame_parser_t *p);

#ifdef __cplusplus
}
#endif

#endif /* TP_FRAME_H */
