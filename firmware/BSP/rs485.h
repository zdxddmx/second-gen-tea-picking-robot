/**
 * @file    rs485.h
 * @brief   RS485 half-duplex link with direction control and Modbus RTU framing.
 *
 * @details
 * # The problem RS485 introduces
 * A half-duplex RS485 transceiver has one pair of wires for both directions and
 * a **driver-enable (DE)** pin.  Asserting DE while another node is talking
 * shorts two drivers together, and the resulting current is limited only by the
 * transceiver's short-circuit protection.  Every implementation therefore has to
 * answer three questions, and getting any of them wrong produces a fault that
 * only appears with long cables and multiple nodes — which is to say, never on
 * the bench:
 *
 * 1. **When to assert DE?**  Before the first start bit, not after.
 * 2. **When to release it?**  *After* the last stop bit has physically left the
 *    shift register.  Releasing on the "transmit register empty" interrupt is
 *    the classic bug: the register is empty but the shift register is not, so
 *    the final byte is truncated.  This driver therefore waits for the
 *    **transmission-complete** condition plus a guard time.
 * 3. **How long must the line be silent before a new frame?**  Modbus RTU
 *    defines the frame boundary as 3.5 character times of silence.  A receiver
 *    that starts parsing inside that window sees two frames glued together.
 *    At 115200 baud 3.5 characters is 304 us; this driver uses the standard
 *    value and documents it rather than using a round 1 ms "because it seemed
 *    safe" — an over-long gap wastes bus bandwidth on a busy line.
 *
 * # Why Modbus RTU *and* a custom protocol
 * They serve different peers.  The custom frame protocol talks to this
 * project's own host software and carries robot-specific data (target
 * coordinates, fault codes, PID gains).  Modbus RTU exists so the same machine
 * can be integrated into a factory that already has a PLC: the register map in
 * `docs/07_communication_protocol.md` exposes position, force, state and fault
 * as ordinary holding registers that any Modbus master can read.
 *
 * Implementing the standard where a standard exists, and a custom protocol only
 * where it does not, is the correct split.  Reinventing Modbus would be worse
 * engineering than either alternative.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_RS485_H
#define TP_RS485_H

#include <stdint.h>

#include "bsp_types.h"
#include "ring_buffer.h"
#include "status.h"
#include "uart_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Bytes of inter-frame silence required by Modbus RTU at 115200 baud. */
#define TP_RS485_T3_5_US 304u

/** @brief Guard time added after the last stop bit before releasing DE, us. */
#define TP_RS485_DE_GUARD_US 20u

/** @brief Maximum Modbus RTU frame length (256 for the standard, 8 for ASCII). */
#define TP_RS485_MAX_FRAME 256u

/** @brief Modbus function codes implemented by the slave. */
#define TP_MODBUS_FC_READ_HOLDING   (0x03u)
#define TP_MODBUS_FC_READ_INPUT     (0x04u)
#define TP_MODBUS_FC_WRITE_SINGLE   (0x06u)
#define TP_MODBUS_FC_WRITE_MULTIPLE (0x10u)

/** @brief Modbus exception codes. */
#define TP_MODBUS_EXC_ILLEGAL_FUNCTION (0x01u)
#define TP_MODBUS_EXC_ILLEGAL_ADDRESS  (0x02u)
#define TP_MODBUS_EXC_ILLEGAL_VALUE    (0x03u)
#define TP_MODBUS_EXC_SLAVE_FAILURE    (0x04u)

/**
 * @brief Direction-control state machine of the transceiver.
 */
typedef enum {
    TP_RS485_DIR_RX = 0, /**< DE de-asserted, listening.                      */
    TP_RS485_DIR_TX = 1, /**< DE asserted, driving the pair.                  */
    TP_RS485_DIR_TX_DRAIN = 2 /**< Waiting for the last stop bit plus guard.  */
} tp_rs485_dir_t;

/**
 * @brief One RS485 port.
 */
typedef struct {
    tp_uart_link_t uart;        /**< Byte transport and statistics.           */
    tp_rs485_dir_t dir;         /**< Current transceiver direction.           */
    uint32_t       drain_start_ms;/**< When the drain phase began.            */
    uint32_t       last_rx_byte_ms;/**< Timestamp of the most recent byte.    */
    uint32_t       last_rx_byte_count;/**< RX byte count at that timestamp.    */
    uint8_t        slave_address;/**< Modbus slave address of this node.      */
    uint32_t       turnaround_count;/**< Direction switches performed.        */
    uint32_t       collisions;  /**< Frames received while driving (a clash). */
    uint32_t       frames_rx;   /**< Complete Modbus frames received.        */
    uint32_t       frames_tx;   /**< Complete Modbus frames transmitted.     */
    uint32_t       crc_errors;  /**< Frames rejected on CRC.                 */
    uint32_t       timeouts;    /**< Silent periods exceeding the response budget.*/
    int            initialised; /**< Non-zero after a successful init.       */
} tp_rs485_t;

/**
 * @brief Initialise the port.
 *
 * @param[in,out] port     Instance.
 * @param[in]     bus      Platform bus bound to the RS485 transceiver.
 * @param[in]     baud     Baud rate; sets the 3.5-character silence threshold.
 * @param[in]     address  Modbus slave address for this node.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_RANGE for address 0
 *         (broadcast) or above 247, both of which are invalid for a slave.
 */
tp_status_t tp_rs485_init(tp_rs485_t *port, tp_bus_t bus, uint32_t baud,
                          uint8_t address);

/**
 * @brief Switch the transceiver into transmit mode.
 *
 * @param[in,out] port  Instance.
 * @return @ref TP_OK, or @ref TP_ERR_BUSY when a reception is in progress, in
 *         which case the caller must retry — asserting DE mid-reception is how
 *         a bus clash happens.
 */
tp_status_t tp_rs485_begin_tx(tp_rs485_t *port);

/**
 * @brief Finish transmitting and return to receive mode.
 *
 * Waits for the transmitter to drain plus @ref TP_RS485_DE_GUARD_US before
 * releasing DE.
 *
 * @param[in,out] port  Instance.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_rs485_end_tx(tp_rs485_t *port);

/**
 * @brief Queue a complete Modbus frame and drive it out.
 *
 * Handles the whole sequence — assert DE, write, wait for drain, release DE — so
 * a caller cannot get the order wrong.
 *
 * @param[in,out] port   Instance.
 * @param[in]     frame  Frame bytes, CRC already appended.
 * @param[in]     len    Frame length, at most @ref TP_RS485_MAX_FRAME.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, @ref TP_ERR_RANGE, or
 *         @ref TP_ERR_OVERFLOW.
 */
tp_status_t tp_rs485_send_frame(tp_rs485_t *port, const uint8_t *frame,
                                uint32_t len);

/**
 * @brief Service the port.
 *
 * Runs the direction state machine, enforces the inter-frame silence window and
 * counts complete frames.  Must be called every communication period.
 *
 * @param[in,out] port  Instance.
 * @return Number of complete frames observed this call.
 */
uint32_t tp_rs485_service(tp_rs485_t *port);

/**
 * @brief Extract one complete Modbus RTU frame from the receive buffer.
 *
 * A frame is deemed complete when the expected length (derived from the
 * function code) has arrived **and** @ref TP_RS485_T3_5_US of silence has
 * passed since the last byte.  Both conditions are needed: length alone cannot
 * detect a truncated frame, and silence alone cannot detect a stalled master.
 *
 * @param[in,out] port   Instance.
 * @param[out]    frame  Receives the frame.
 * @param[in]     max_len Capacity of @p frame.
 * @return Frame length on success, or 0 when no complete frame is available.
 */
uint32_t tp_rs485_poll_frame(tp_rs485_t *port, uint8_t *frame, uint32_t max_len);

/**
 * @brief Current transceiver direction, for diagnostics.
 *
 * @param[in] port  Instance.
 * @return Direction state.
 */
tp_rs485_dir_t tp_rs485_direction(const tp_rs485_t *port);

#ifdef __cplusplus
}
#endif

#endif /* TP_RS485_H */
