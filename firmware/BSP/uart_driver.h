/**
 * @file    uart_driver.h
 * @brief   Buffered UART link: byte transport with statistics.
 *
 * @details
 * # What it is
 * A thin, buffer-managed wrapper over one `tp_bus_t` UART.  It owns the TX and
 * RX ring buffers, so:
 *
 *   - the RX interrupt (or the host shim) only ever calls
 *     @ref tp_uart_rx_push, which is O(1);
 *   - the protocol task pulls bytes with @ref tp_uart_read at its own pace;
 *   - TX is all-or-nothing per frame, so a partially transmitted frame — which
 *     the far end would have to time out on — cannot occur.
 *
 * # Why the buffers live here rather than in the protocol layer
 * Framing is a *link* concern and buffering is a *link* concern; keeping them
 * together means the frame parser can be a pure function of a byte stream and
 * therefore trivially unit-testable.  The parser also has to be reused over
 * CAN and RS485 later, so it must not know about UART ring buffers at all.
 *
 * # Statistics that matter
 * @ref tp_uart_link_t::rx_overflow is the single most useful number when a
 * serial link misbehaves: if it is climbing, the consumer is too slow or the
 * buffer is too small, and every "mysterious CRC error" report traces back to
 * it eventually.  @ref tp_uart_link_t::rx_high_water is the evidence needed to
 * size the buffer from data rather than by guessing.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_UART_DRIVER_H
#define TP_UART_DRIVER_H

#include <stdint.h>

#include "bsp_types.h"
#include "ring_buffer.h"
#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Size of each UART ring buffer, bytes. Must be a power of two. */
#define TP_UART_BUFFER_SIZE 512u

/** @brief Length of the line-status flags, see @ref tp_uart_link_t::status. */
#define TP_UART_STATUS_OVERRUN  (1u << 0u)
#define TP_UART_STATUS_FRAMING  (1u << 1u)
#define TP_UART_STATUS_PARITY   (1u << 2u)
#define TP_UART_STATUS_NOISE    (1u << 3u)

/**
 * @brief One UART link.
 */
typedef struct {
    tp_bus_t         bus;      /**< Underlying platform bus.                  */
    uint32_t         baud;     /**< Configured baud rate.                     */
    uint8_t          rx_storage[TP_UART_BUFFER_SIZE]; /**< RX ring storage.    */
    uint8_t          tx_storage[TP_UART_BUFFER_SIZE]; /**< TX ring storage.    */
    tp_ring_buffer_t rx;       /**< Bytes received, awaiting the parser.      */
    tp_ring_buffer_t tx;       /**< Bytes queued for transmission.            */

    uint32_t rx_bytes;         /**< Total bytes received.                     */
    uint32_t tx_bytes;         /**< Total bytes transmitted.                  */
    uint32_t tx_frames;        /**< Frames handed to @ref tp_uart_write_frame.*/
    uint32_t tx_rejected;      /**< Frames dropped because the TX ring was full.*/
    uint32_t rx_overflow;      /**< Bytes lost because the RX ring was full.  */
    uint32_t status;           /**< OR of the TP_UART_STATUS_* flags.         */

    int      initialised;      /**< Non-zero after a successful init.         */
} tp_uart_link_t;

/**
 * @brief Initialise a UART link.
 *
 * @param[in,out] link  Instance.
 * @param[in]     bus   Platform bus to bind.
 * @param[in]     baud  Baud rate, recorded for diagnostics and for the
 *                      theoretical-latency calculation in the docs.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM.
 */
tp_status_t tp_uart_init(tp_uart_link_t *link, tp_bus_t bus, uint32_t baud);

/**
 * @brief Push received bytes into the RX ring.
 *
 * Called from the RX interrupt on the target and from the platform shim on the
 * host.  Never blocks and never allocates.
 *
 * @param[in,out] link  Instance.
 * @param[in]     data  Bytes received.
 * @param[in]     len   Number of bytes.
 * @return @ref TP_OK, or @ref TP_ERR_OVERFLOW when the ring was full and the
 *         whole block was dropped.
 */
tp_status_t tp_uart_rx_push(tp_uart_link_t *link, const uint8_t *data,
                            uint32_t len);

/**
 * @brief Pull bytes out of the RX ring.
 *
 * @param[in,out] link     Instance.
 * @param[out]    dst      Destination buffer.
 * @param[in]     max_len  Capacity of @p dst.
 * @return Number of bytes read; 0 when nothing is pending.
 */
uint32_t tp_uart_read(tp_uart_link_t *link, uint8_t *dst, uint32_t max_len);

/**
 * @brief Queue a complete frame for transmission.
 *
 * The ring is checked for the whole length before anything is written, so a
 * frame is either fully queued or not queued at all.
 *
 * @param[in,out] link  Instance.
 * @param[in]     data  Frame bytes.
 * @param[in]     len   Frame length.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_OVERFLOW when the TX
 *         ring cannot hold the frame.
 */
tp_status_t tp_uart_write_frame(tp_uart_link_t *link, const uint8_t *data,
                                uint32_t len);

/**
 * @brief Service the transmitter: move queued bytes onto the bus.
 *
 * Must be called periodically (the communication task).  Draining in this
 * function rather than in the ISR keeps the ISR short and lets the caller see a
 * partial-drain condition.
 *
 * @param[in,out] link  Instance.
 * @return Number of bytes actually moved onto the bus.
 */
uint32_t tp_uart_service_tx(tp_uart_link_t *link);

/**
 * @brief Service the receiver: move bus bytes into the RX ring.
 *
 * @param[in,out] link  Instance.
 * @return Number of bytes ingested.
 */
uint32_t tp_uart_service_rx(tp_uart_link_t *link);

/**
 * @brief Number of bytes waiting in the RX ring.
 *
 * @param[in] link  Instance.
 * @return Pending byte count.
 */
uint32_t tp_uart_available(const tp_uart_link_t *link);

/**
 * @brief Whether the TX ring is empty.
 *
 * @param[in] link  Instance.
 * @return Non-zero when fully transmitted.
 */
int tp_uart_tx_idle(const tp_uart_link_t *link);

/**
 * @brief Discard everything in both directions.
 *
 * @param[in,out] link  Instance.
 */
void tp_uart_flush(tp_uart_link_t *link);

/**
 * @brief Theoretical time to transmit @p bytes at the link's baud rate.
 *
 * Ten bits per byte (8N1 with start and stop).  Used by the documentation's
 * latency budget and by @ref tp_uart_write_frame's overflow logging to explain
 * *why* a frame did not fit.
 *
 * @param[in] link   Instance.
 * @param[in] bytes  Byte count.
 * @return Time in microseconds, or 0 when the baud rate is 0.
 */
uint32_t tp_uart_tx_time_us(const tp_uart_link_t *link, uint32_t bytes);

/**
 * @brief Record a hardware line-status error.
 *
 * @param[in,out] link   Instance.
 * @param[in]     flags  OR of the TP_UART_STATUS_* flags.
 */
void tp_uart_note_status(tp_uart_link_t *link, uint32_t flags);

#ifdef __cplusplus
}
#endif

#endif /* TP_UART_DRIVER_H */
