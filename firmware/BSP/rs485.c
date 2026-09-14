/**
 * @file    rs485.c
 * @brief   Implementation of the RS485 half-duplex port.
 * @copyright MIT License, see LICENSE.
 */

#include "rs485.h"

#include <string.h>

#include "compiler.h"
#include "config.h"
#include "logger.h"
#include "platform_api.h"

/** @brief Longest silence that still counts as "inside one frame", ms. */
#define RS485_FRAME_IDLE_MS 2u

tp_status_t tp_rs485_init(tp_rs485_t *port, tp_bus_t bus, uint32_t baud,
                          uint8_t address)
{
    tp_status_t st;

    if (port == NULL) {
        return TP_ERR_PARAM;
    }
    if ((address == 0u) || (address > 247u)) {
        LOG_ERROR(TP_MOD_COMM, "RS485 address %u invalid for a slave (1..247)",
                  (unsigned)address);
        return TP_ERR_RANGE;
    }
    (void)memset(port, 0, sizeof(*port));
    st = tp_uart_init(&port->uart, bus, baud);
    if (st != TP_OK) {
        return st;
    }
    port->dir = TP_RS485_DIR_RX;
    port->slave_address = address;
    port->initialised = 1;
    LOG_INFO(TP_MOD_COMM, "RS485 ready: slave %u, %lu baud, T3.5 = %lu us",
             (unsigned)address, (unsigned long)baud,
             (unsigned long)TP_RS485_T3_5_US);
    return TP_OK;
}

tp_status_t tp_rs485_begin_tx(tp_rs485_t *port)
{
    if ((port == NULL) || (port->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (port->dir != TP_RS485_DIR_RX) {
        return TP_ERR_BUSY;
    }
    /* Assert DE only after confirming the receive buffer is quiet: if bytes
     * are still arriving, another node owns the pair and driving it now would
     * be a clash. */
    if (tp_uart_available(&port->uart) > 0u) {
        port->collisions++;
        return TP_ERR_BUSY;
    }
    port->dir = TP_RS485_DIR_TX;
    port->turnaround_count++;
    return TP_OK;
}

tp_status_t tp_rs485_end_tx(tp_rs485_t *port)
{
    if ((port == NULL) || (port->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if (port->dir == TP_RS485_DIR_TX) {
        port->dir = TP_RS485_DIR_TX_DRAIN;
        port->drain_start_ms = tp_platform_time_ms();
    }
    return TP_OK;
}

tp_status_t tp_rs485_send_frame(tp_rs485_t *port, const uint8_t *frame,
                                uint32_t len)
{
    uint32_t drain_us;
    uint32_t wait_us;
    tp_status_t st;

    if ((port == NULL) || (port->initialised == 0)) {
        return TP_ERR_PARAM;
    }
    if ((frame == NULL) || (len < 4u)) {
        /* A Modbus RTU frame is at least address + function + CRC(2). */
        return TP_ERR_PARAM;
    }
    if (len > TP_RS485_MAX_FRAME) {
        return TP_ERR_RANGE;
    }

    st = tp_rs485_begin_tx(port);
    if (st != TP_OK) {
        return st;
    }
    st = tp_uart_write_frame(&port->uart, frame, len);
    if (st != TP_OK) {
        port->dir = TP_RS485_DIR_RX;
        return st;
    }

    /* Drive the bytes out synchronously: at 115200 baud a 256-byte frame takes
     * 22 ms, which is longer than the communication period, so the drain has to
     * be waited for rather than deferred — leaving DE asserted across periods
     * would block reception for an unbounded time.  The bound below keeps a
     * stuck transmitter from hanging the task forever. */
    drain_us = tp_uart_tx_time_us(&port->uart, len) + TP_RS485_DE_GUARD_US;
    wait_us = 0u;
    while (!tp_uart_tx_idle(&port->uart) && (wait_us < (drain_us * 2u + 1000u))) {
        (void)tp_uart_service_tx(&port->uart);
        tp_platform_delay_us(10u);
        wait_us += 10u;
    }
    /* Wait the guard time so the final stop bit has physically cleared before
     * the transceiver stops driving.  Releasing DE earlier truncates the last
     * byte, which the far end reports as a CRC error on a frame it received
     * almost completely. */
    tp_platform_delay_us(TP_RS485_DE_GUARD_US);

    port->dir = TP_RS485_DIR_RX;
    port->frames_tx++;
    return TP_OK;
}

uint32_t tp_rs485_service(tp_rs485_t *port)
{
    uint32_t n;

    if ((port == NULL) || (port->initialised == 0)) {
        return 0u;
    }
    if (port->dir == TP_RS485_DIR_TX_DRAIN) {
        if ((tp_platform_time_ms() - port->drain_start_ms) >= 1u) {
            port->dir = TP_RS485_DIR_RX;
        }
    }
    n = tp_uart_service_rx(&port->uart);
    if (n > 0u) {
        if (port->dir != TP_RS485_DIR_RX) {
            /* Bytes arriving while we drive means a real bus clash. */
            port->collisions++;
        }
        /* Record when the line last went quiet, so a truncated frame can be
         * detected from the *silence* rather than from a guess. */
        port->last_rx_byte_ms = tp_platform_time_ms();
        port->last_rx_byte_count = port->uart.rx_bytes;
    }
    (void)tp_uart_service_tx(&port->uart);
    return n;
}

/**
 * @brief Expected total length of a Modbus RTU response/request, 0 if unknown.
 *
 * Deriving the length from the function code is what makes frame delimiting
 * reliable without relying on timing alone: the driver knows exactly how many
 * bytes to expect and can therefore distinguish "still arriving" from
 * "truncated".
 *
 * @param[in] frame      Frame bytes received so far.
 * @param[in] available  Bytes available.
 * @return Expected total length including the two CRC bytes, or 0 when more
 *         header bytes are needed before the length can be determined.
 */
static uint32_t rs485_expected_length(const uint8_t *frame, uint32_t available)
{
    uint8_t func;

    if (available < 2u) {
        return 0u;
    }
    func = frame[1];

    switch (func) {
    case TP_MODBUS_FC_READ_HOLDING:
    case TP_MODBUS_FC_READ_INPUT:
        if (available < 3u) {
            return 0u;
        }
        /* A read *request* is 8 bytes.  A read *response* has a byte count in
         * position 2, so its total is 3 + count + 2. */
        if (frame[2] > 0x7Fu) {
            return 8u; /* byte count above 127 cannot be a response: a request */
        }
        if (available >= 8u) {
            return 8u; /* unambiguous: 8 bytes is a full request */
        }
        return 0u;
    case TP_MODBUS_FC_WRITE_SINGLE:
        return 8u;
    case TP_MODBUS_FC_WRITE_MULTIPLE:
        if (available < 7u) {
            return 0u;
        }
        return 9u + (uint32_t)frame[6];
    default:
        /* Exception responses are always 5 bytes; unsupported function codes
         * are answered with one, so treat anything else as 5 once we have the
         * exception byte. */
        return 5u;
    }
}

uint32_t tp_rs485_poll_frame(tp_rs485_t *port, uint8_t *frame, uint32_t max_len)
{
    uint8_t  header[7];
    uint32_t available;
    uint32_t expected;
    uint32_t i;

    if ((port == NULL) || (frame == NULL) || (max_len < 5u)) {
        return 0u;
    }
    available = tp_uart_available(&port->uart);
    if (available < 4u) {
        return 0u;
    }

    /* Snapshot the header without consuming it, so the length can be derived
     * before committing to a read. */
    for (i = 0u; (i < available) && (i < sizeof(header)); ++i) {
        if (tp_ring_buffer_peek(&port->uart.rx, i, &header[i]) != TP_OK) {
            break;
        }
    }
    expected = rs485_expected_length(header, (available < sizeof(header))
                                             ? available : sizeof(header));
    if (expected == 0u) {
        return 0u; /* need more header bytes */
    }
    if (expected > max_len) {
        /* Cannot happen with the bounds above, but refusing is cheaper than
         * trusting the arithmetic. */
        port->crc_errors++;
        (void)tp_uart_read(&port->uart, frame, 1u);
        return 0u;
    }
    if (available < expected) {
        /* A short frame is only a *truncation* once the line has actually gone
         * quiet: the inter-frame silence of 3.5 character times has to have
         * elapsed with no new byte, and no byte may have arrived since the last
         * service call.  Timing alone is not enough — at 115200 baud the whole
         * 8-byte request takes 694 us, so a naive "has it been 2 ms?" test would
         * drop every healthy frame on a slow poll loop. */
        uint32_t quiet_ms = tp_platform_time_ms() - port->last_rx_byte_ms;
        if ((quiet_ms >= RS485_FRAME_IDLE_MS) &&
            (port->last_rx_byte_count == port->uart.rx_bytes)) {
            port->timeouts++;
            LOG_WARN(TP_MOD_COMM,
                     "short Modbus frame dropped: %lu of %lu bytes, line quiet %lu ms",
                     (unsigned long)available, (unsigned long)expected,
                     (unsigned long)quiet_ms);
            (void)tp_uart_read(&port->uart, frame, available);
        }
        return 0u;
    }

    if (tp_uart_read(&port->uart, frame, expected) != expected) {
        return 0u;
    }
    port->frames_rx++;
    return expected;
}

tp_rs485_dir_t tp_rs485_direction(const tp_rs485_t *port)
{
    return (port != NULL) ? port->dir : TP_RS485_DIR_RX;
}
