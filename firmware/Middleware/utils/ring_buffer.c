/**
 * @file    ring_buffer.c
 * @brief   Implementation of the byte ring buffer.
 * @copyright MIT License, see LICENSE.
 */

#include "ring_buffer.h"

#include <string.h>

/**
 * @brief Round a capacity down to a power of two, floor at 2.
 *
 * @param[in] n  Requested capacity.
 * @return Largest power of two that is <= @p n, or 2 when @p n < 2.
 */
static uint32_t floor_pow2(uint32_t n)
{
    uint32_t p = 2u;

    if (n < 2u) {
        return 0u;
    }
    while ((p << 1) <= n) {
        p <<= 1;
    }
    return p;
}

tp_status_t tp_ring_buffer_init(tp_ring_buffer_t *rb, uint8_t *storage,
                                uint32_t capacity)
{
    uint32_t cap;

    if ((rb == NULL) || (storage == NULL)) {
        return TP_ERR_PARAM;
    }
    cap = floor_pow2(capacity);
    if (cap == 0u) {
        return TP_ERR_PARAM;
    }
    (void)memset(rb, 0, sizeof(*rb));
    rb->data = storage;
    rb->capacity = cap;
    rb->mask = cap - 1u;
    rb->head = 0u;
    rb->tail = 0u;
    return TP_OK;
}

uint32_t tp_ring_buffer_count(const tp_ring_buffer_t *rb)
{
    if (rb == NULL) {
        return 0u;
    }
    return (uint32_t)(rb->head - rb->tail);
}

uint32_t tp_ring_buffer_space(const tp_ring_buffer_t *rb)
{
    if (rb == NULL) {
        return 0u;
    }
    return rb->capacity - tp_ring_buffer_count(rb);
}

int tp_ring_buffer_is_empty(const tp_ring_buffer_t *rb)
{
    return (tp_ring_buffer_count(rb) == 0u) ? 1 : 0;
}

int tp_ring_buffer_is_full(const tp_ring_buffer_t *rb)
{
    return (tp_ring_buffer_count(rb) >= ((rb != NULL) ? rb->capacity : 0u)) ? 1 : 0;
}

tp_status_t tp_ring_buffer_push(tp_ring_buffer_t *rb, uint8_t b)
{
    uint32_t occupancy;

    if (rb == NULL) {
        return TP_ERR_PARAM;
    }
    occupancy = tp_ring_buffer_count(rb);
    if (occupancy >= rb->capacity) {
        rb->overflow++;
        return TP_ERR_OVERFLOW;
    }
    rb->data[rb->head & rb->mask] = b;
    rb->head++; /* publish only after the payload is visible */
    if ((rb->head - rb->tail) > rb->high_water) {
        rb->high_water = rb->head - rb->tail;
    }
    return TP_OK;
}

tp_status_t tp_ring_buffer_write(tp_ring_buffer_t *rb, const uint8_t *src,
                                 uint32_t len)
{
    uint32_t i;

    if ((rb == NULL) || ((src == NULL) && (len > 0u))) {
        return TP_ERR_PARAM;
    }
    if (len == 0u) {
        return TP_OK;
    }
    /* All-or-nothing: a partially written frame is worse than a dropped one,
     * because the reader would have to time out before it could resynchronise. */
    if (tp_ring_buffer_space(rb) < len) {
        rb->overflow += len;
        return TP_ERR_OVERFLOW;
    }
    for (i = 0u; i < len; ++i) {
        rb->data[rb->head & rb->mask] = src[i];
        rb->head++;
    }
    if ((rb->head - rb->tail) > rb->high_water) {
        rb->high_water = rb->head - rb->tail;
    }
    return TP_OK;
}

tp_status_t tp_ring_buffer_pop(tp_ring_buffer_t *rb, uint8_t *out)
{
    if ((rb == NULL) || (out == NULL)) {
        return TP_ERR_PARAM;
    }
    if (tp_ring_buffer_count(rb) == 0u) {
        return TP_ERR_NOT_READY;
    }
    *out = rb->data[rb->tail & rb->mask];
    rb->tail++;
    return TP_OK;
}

uint32_t tp_ring_buffer_read(tp_ring_buffer_t *rb, uint8_t *dst, uint32_t len)
{
    uint32_t available;
    uint32_t n;
    uint32_t i;

    if ((rb == NULL) || (dst == NULL) || (len == 0u)) {
        return 0u;
    }
    available = tp_ring_buffer_count(rb);
    n = (len < available) ? len : available;
    for (i = 0u; i < n; ++i) {
        dst[i] = rb->data[rb->tail & rb->mask];
        rb->tail++;
    }
    return n;
}

tp_status_t tp_ring_buffer_peek(const tp_ring_buffer_t *rb, uint32_t offset,
                                uint8_t *out)
{
    if ((rb == NULL) || (out == NULL)) {
        return TP_ERR_PARAM;
    }
    if ((offset + 1u) > tp_ring_buffer_count(rb)) {
        return TP_ERR_NOT_READY;
    }
    *out = rb->data[(rb->tail + offset) & rb->mask];
    return TP_OK;
}

uint32_t tp_ring_buffer_discard(tp_ring_buffer_t *rb, uint32_t len)
{
    uint32_t available;
    uint32_t n;

    if ((rb == NULL) || (len == 0u)) {
        return 0u;
    }
    available = tp_ring_buffer_count(rb);
    n = (len < available) ? len : available;
    rb->tail += n;
    return n;
}

void tp_ring_buffer_flush(tp_ring_buffer_t *rb)
{
    if (rb == NULL) {
        return;
    }
    rb->tail = rb->head;
}

uint32_t tp_ring_buffer_overflow_count(const tp_ring_buffer_t *rb)
{
    return (rb != NULL) ? rb->overflow : 0u;
}

uint32_t tp_ring_buffer_high_water(const tp_ring_buffer_t *rb)
{
    return (rb != NULL) ? rb->high_water : 0u;
}
