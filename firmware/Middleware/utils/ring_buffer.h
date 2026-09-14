/**
 * @file    ring_buffer.h
 * @brief   Lock-free-enough single-producer/single-consumer byte ring buffer.
 *
 * @details
 * # Why a byte ring and not a queue of frames
 * A UART RX ISR cannot know where a frame ends.  If the ISR tries to assemble
 * frames it has to parse, apply the CRC and possibly drop bytes — all inside an
 * interrupt, at 921600 baud that is one byte every 10.9 us.  The standard,
 * robust split is:
 *
 *   ISR  : copy raw bytes into the ring, bump the head index  (bounded, ~1 us)
 *   Task : parse frames out of the ring at its own pace
 *
 * This keeps the ISR strictly O(1) and makes the parser testable without any
 * hardware at all.
 *
 * # Concurrency contract
 * With exactly **one** writer and **one** reader, and head/tail as monotonically
 * increasing counters (not wrapped indices), the algorithm needs no lock:
 * the writer only ever publishes `head` after writing the payload, and the
 * reader only ever publishes `tail` after consuming.  `volatile` plus the
 * natural ordering of the ARM store instructions is sufficient on a single
 * core; `tp_platform_enter_critical` is used only around multi-byte index
 * snapshots on the target for defence in depth.
 *
 * # Full-buffer policy
 * The buffer **overwrites nothing**.  A full ring drops the *new* byte and
 * increments an overflow counter.  For a command link this is the correct
 * trade-off: losing the tail of a truncated frame is harmless because the
 * parser will resynchronise on the next `0xAA 0x55`, whereas silently
 * overwriting old data would corrupt a frame that was already partially
 * accepted.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_RING_BUFFER_H
#define TP_RING_BUFFER_H

#include <stdint.h>

#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Byte-oriented ring buffer with caller-owned storage.
 *
 * No dynamic allocation: the caller supplies the array, which keeps the memory
 * map fully static and analysable (important for a certification story, and
 * simply good practice on a microcontroller).
 */
typedef struct {
    uint8_t *data;    /**< Caller-owned storage.                              */
    uint32_t capacity;/**< Size of @p data in bytes; must be a power of two.  */
    uint32_t mask;    /**< `capacity - 1`, for cheap modulo.                  */
    volatile uint32_t head; /**< Total bytes ever written.                    */
    volatile uint32_t tail; /**< Total bytes ever read.                       */
    uint32_t overflow;      /**< Bytes rejected because the ring was full.    */
    uint32_t high_water;    /**< Maximum occupancy observed, for tuning.      */
} tp_ring_buffer_t;

/**
 * @brief Initialise a ring buffer over caller-owned storage.
 *
 * @param[out] rb        Buffer control block.
 * @param[in]  storage   Caller-owned byte array.
 * @param[in]  capacity  Size of @p storage.  Non-power-of-two sizes are
 *                       rounded *down* to the previous power of two so the
 *                       index wrap can use a mask instead of a modulo (a
 *                       modulo by a non-constant is a real division on
 *                       Cortex-M4, i.e. 2-12 cycles inside an ISR).
 * @return @ref TP_OK, or @ref TP_ERR_PARAM when a pointer is NULL or
 *         @p capacity is smaller than 2.
 */
tp_status_t tp_ring_buffer_init(tp_ring_buffer_t *rb, uint8_t *storage,
                                uint32_t capacity);

/**
 * @brief Bytes currently available to read.
 *
 * @param[in] rb  Buffer.
 * @return Occupancy in bytes.
 */
uint32_t tp_ring_buffer_count(const tp_ring_buffer_t *rb);

/**
 * @brief Free space available for writing.
 *
 * @param[in] rb  Buffer.
 * @return Free bytes.
 */
uint32_t tp_ring_buffer_space(const tp_ring_buffer_t *rb);

/**
 * @brief Whether the buffer holds no data.
 *
 * @param[in] rb  Buffer.
 * @return Non-zero when empty.
 */
int tp_ring_buffer_is_empty(const tp_ring_buffer_t *rb);

/**
 * @brief Whether the buffer is completely full.
 *
 * @param[in] rb  Buffer.
 * @return Non-zero when full.
 */
int tp_ring_buffer_is_full(const tp_ring_buffer_t *rb);

/**
 * @brief Push one byte.
 *
 * @param[in,out] rb  Buffer.
 * @param[in]     b   Byte to append.
 * @return @ref TP_OK, or @ref TP_ERR_OVERFLOW when the byte was dropped.
 */
tp_status_t tp_ring_buffer_push(tp_ring_buffer_t *rb, uint8_t b);

/**
 * @brief Push a block, all-or-nothing.
 *
 * @param[in,out] rb    Buffer.
 * @param[in]     src   Source bytes.
 * @param[in]     len   Number of bytes.
 * @return @ref TP_OK, or @ref TP_ERR_OVERFLOW when the block did not fit and
 *         therefore was *not* written at all.
 */
tp_status_t tp_ring_buffer_write(tp_ring_buffer_t *rb, const uint8_t *src,
                                 uint32_t len);

/**
 * @brief Pop one byte.
 *
 * @param[in,out] rb  Buffer.
 * @param[out]    out Receives the byte; untouched when the buffer is empty.
 * @return @ref TP_OK, or @ref TP_ERR_NOT_READY when empty.
 */
tp_status_t tp_ring_buffer_pop(tp_ring_buffer_t *rb, uint8_t *out);

/**
 * @brief Read up to @p len bytes.
 *
 * @param[in,out] rb   Buffer.
 * @param[out]    dst  Destination.
 * @param[in]     len  Maximum bytes to read.
 * @return Number of bytes actually read (0 when empty).
 */
uint32_t tp_ring_buffer_read(tp_ring_buffer_t *rb, uint8_t *dst, uint32_t len);

/**
 * @brief Look at a byte @p offset positions ahead of the read pointer.
 *
 * Lets the frame parser peek for the `0xAA 0x55` header before committing to
 * consuming anything.
 *
 * @param[in] rb      Buffer.
 * @param[in] offset  Zero-based offset from the read pointer.
 * @param[out] out    Receives the byte.
 * @return @ref TP_OK, or @ref TP_ERR_NOT_READY when fewer than
 *         `offset + 1` bytes are available.
 */
tp_status_t tp_ring_buffer_peek(const tp_ring_buffer_t *rb, uint32_t offset,
                                uint8_t *out);

/**
 * @brief Discard @p len bytes from the read side.
 *
 * @param[in,out] rb   Buffer.
 * @param[in]     len  Number of bytes to drop.
 * @return Number of bytes actually discarded.
 */
uint32_t tp_ring_buffer_discard(tp_ring_buffer_t *rb, uint32_t len);

/**
 * @brief Empty the buffer without touching the counters' monotonicity.
 * @param[in,out] rb  Buffer.
 */
void tp_ring_buffer_flush(tp_ring_buffer_t *rb);

/**
 * @brief Number of bytes rejected since init.
 * @param[in] rb  Buffer.
 * @return Overflow count.
 */
uint32_t tp_ring_buffer_overflow_count(const tp_ring_buffer_t *rb);

/**
 * @brief Peak occupancy since init, as a fraction of capacity.
 *
 * A high-water mark above roughly 75 % means the consumer is too slow or the
 * buffer is too small — the sort of evidence an interviewer likes to see when
 * asking "how do you know your buffer size is right?".
 *
 * @param[in] rb  Buffer.
 * @return Peak occupancy in bytes.
 */
uint32_t tp_ring_buffer_high_water(const tp_ring_buffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* TP_RING_BUFFER_H */
