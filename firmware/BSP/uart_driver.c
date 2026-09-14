/**
 * @file    uart_driver.c
 * @brief   Implementation of the buffered UART link.
 * @copyright MIT License, see LICENSE.
 */

#include "uart_driver.h"

#include <string.h>

#include "compiler.h"
#include "config.h"
#include "logger.h"
#include "platform_api.h"

tp_status_t tp_uart_init(tp_uart_link_t *link, tp_bus_t bus, uint32_t baud)
{
    if (link == NULL) {
        return TP_ERR_PARAM;
    }
    if (((int)bus < 0) || (bus >= TP_BUS_COUNT)) {
        return TP_ERR_PARAM;
    }

    (void)memset(link, 0, sizeof(*link));
    link->bus = bus;
    link->baud = baud;
    if (tp_ring_buffer_init(&link->rx, link->rx_storage,
                            sizeof(link->rx_storage)) != TP_OK) {
        return TP_ERR_PARAM;
    }
    if (tp_ring_buffer_init(&link->tx, link->tx_storage,
                            sizeof(link->tx_storage)) != TP_OK) {
        return TP_ERR_PARAM;
    }
    link->initialised = 1;
    LOG_INFO(TP_MOD_COMM, "bus %d up at %lu baud, buffers %u B",
             (int)bus, (unsigned long)baud, (unsigned)TP_UART_BUFFER_SIZE);
    return TP_OK;
}

tp_status_t tp_uart_rx_push(tp_uart_link_t *link, const uint8_t *data,
                            uint32_t len)
{
    tp_status_t st;

    if ((link == NULL) || (link->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if ((data == NULL) && (len > 0u)) {
        return TP_ERR_PARAM;
    }
    st = tp_ring_buffer_write(&link->rx, data, len);
    if (st == TP_OK) {
        link->rx_bytes += len;
    } else {
        /* Count in bytes, not events: "we lost 37 bytes in 2 bursts" is far
         * more actionable than "2 overflows". */
        link->rx_overflow += len;
    }
    return st;
}

uint32_t tp_uart_read(tp_uart_link_t *link, uint8_t *dst, uint32_t max_len)
{
    if ((link == NULL) || (link->initialised == 0)) {
        return 0u;
    }
    return tp_ring_buffer_read(&link->rx, dst, max_len);
}

tp_status_t tp_uart_write_frame(tp_uart_link_t *link, const uint8_t *data,
                                uint32_t len)
{
    tp_status_t st;

    if ((link == NULL) || (link->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if ((data == NULL) || (len == 0u)) {
        return TP_ERR_PARAM;
    }
    st = tp_ring_buffer_write(&link->tx, data, len);
    if (st != TP_OK) {
        link->tx_rejected++;
        LOG_WARN(TP_MOD_COMM,
                 "bus %d TX ring full: dropped %lu B frame (queued %lu B, %lu us to drain)",
                 (int)link->bus, (unsigned long)len,
                 (unsigned long)tp_ring_buffer_count(&link->tx),
                 (unsigned long)tp_uart_tx_time_us(link,
                     tp_ring_buffer_count(&link->tx)));
        return st;
    }
    link->tx_frames++;
    return TP_OK;
}

uint32_t tp_uart_service_tx(tp_uart_link_t *link)
{
    uint8_t  chunk[TP_PROTO_MAX_FRAME];
    uint32_t n;
    uint32_t i;

    if ((link == NULL) || (link->initialised == 0)) {
        return 0u;
    }
    if (tp_platform_bus_tx_done(link->bus) == 0) {
        return 0u; /* previous transfer still in flight */
    }
    n = tp_ring_buffer_count(&link->tx);
    if (n == 0u) {
        return 0u;
    }
    if (n > sizeof(chunk)) {
        n = sizeof(chunk);
    }
    /* Peek-then-discard rather than read-then-restore: if the bus write fails
     * the bytes stay queued and are retried next period, which is the correct
     * behaviour for a link that can transiently refuse a transfer. */
    for (i = 0u; i < n; ++i) {
        uint8_t b = 0u;
        if (tp_ring_buffer_peek(&link->tx, i, &b) != TP_OK) {
            n = i;
            break;
        }
        chunk[i] = b;
    }
    if (n == 0u) {
        return 0u;
    }
    if (tp_platform_bus_write(link->bus, chunk, n) != TP_OK) {
        return 0u;
    }
    (void)tp_ring_buffer_discard(&link->tx, n);
    link->tx_bytes += n;
    return n;
}

uint32_t tp_uart_service_rx(tp_uart_link_t *link)
{
    uint8_t  chunk[128];
    uint32_t n;

    if ((link == NULL) || (link->initialised == 0)) {
        return 0u;
    }
    n = tp_platform_bus_read(link->bus, chunk, sizeof(chunk));
    if (n == 0u) {
        return 0u;
    }
    (void)tp_uart_rx_push(link, chunk, n);
    return n;
}

uint32_t tp_uart_available(const tp_uart_link_t *link)
{
    if ((link == NULL) || (link->initialised == 0)) {
        return 0u;
    }
    return tp_ring_buffer_count(&link->rx);
}

int tp_uart_tx_idle(const tp_uart_link_t *link)
{
    if ((link == NULL) || (link->initialised == 0)) {
        return 0;
    }
    return tp_ring_buffer_is_empty(&link->tx);
}

void tp_uart_flush(tp_uart_link_t *link)
{
    if (link == NULL) {
        return;
    }
    tp_ring_buffer_flush(&link->rx);
    tp_ring_buffer_flush(&link->tx);
}

uint32_t tp_uart_tx_time_us(const tp_uart_link_t *link, uint32_t bytes)
{
    if ((link == NULL) || (link->baud == 0u)) {
        return 0u;
    }
    /* 8N1 = 1 start + 8 data + 1 stop = 10 bit times per byte. */
    return (uint32_t)(((uint64_t)bytes * 10u * 1000000u) / link->baud);
}

void tp_uart_note_status(tp_uart_link_t *link, uint32_t flags)
{
    if (link == NULL) {
        return;
    }
    if ((link->status & flags) == 0u) {
        LOG_WARN(TP_MOD_COMM, "bus %d line status flags 0x%02lx",
                 (int)link->bus, (unsigned long)flags);
    }
    link->status |= flags;
}
