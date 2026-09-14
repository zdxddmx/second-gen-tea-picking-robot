/**
 * @file    can_driver.h
 * @brief   CAN 2.0A transport with mailbox management and bus diagnostics.
 *
 * @details
 * # Why CAN at all, when the host link is already a UART
 * The three links in this robot have genuinely different requirements, and
 * picking one bus for all of them would be the wrong engineering answer:
 *
 * | Link | Peer | Needs | Chosen bus |
 * |---|---|---|---|
 * | Vision / host | a PC | throughput (921600 baud), point to point | UART with the framed protocol |
 * | Auxiliary drives, auxiliary sensor nodes | several MCUs | multi-drop, short deterministic frames, error containment | **CAN** |
 * | Field / service devices | PLCs, VFDs, legacy controllers | interoperation with existing industrial equipment | **RS485 + Modbus RTU** |
 *
 * CAN's value here is not speed (500 kbit/s is slower than the UART) but
 * **arbitration and error containment**: several nodes share two wires, the
 * identifier doubles as the priority, and a node that goes bad is isolated by
 * the controller's error counters rather than corrupting everyone's traffic.
 * That is exactly what an auxiliary sensor node needs.
 *
 * # What this driver does and does not do
 * It is a **transport**, not an application layer.  It handles:
 *   - classic 11-bit data frames (extended identifiers are rejected, see
 *     @ref tp_can_send),
 *   - the three-mailbox transmit path with an explicit "all busy" return so the
 *     caller can retry instead of blocking,
 *   - acceptance filtering so the CPU is not woken for traffic it does not
 *     want — on a shared bus most frames are for someone else,
 *   - error-counter and bus-off tracking.
 *
 * It deliberately does **not** implement CANopen, J1939 or a segmented
 * transfer protocol.  Those are application-layer concerns and pretending
 * otherwise in a 20 kB firmware would be dishonest; `docs/13_future_work.md`
 * records the extension points.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_CAN_DRIVER_H
#define TP_CAN_DRIVER_H

#include <stdint.h>

#include "bsp_types.h"
#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Maximum data bytes in a classic CAN frame. */
#define TP_CAN_MAX_DLC 8u

/** @brief Maximum number of acceptance filters configured at once. */
#define TP_CAN_MAX_FILTERS 4u

/**
 * @brief One classic CAN data frame.
 */
typedef struct {
    uint32_t id;                     /**< 11-bit identifier.                 */
    uint8_t  dlc;                    /**< Data length, 0..8.                 */
    uint8_t  data[TP_CAN_MAX_DLC];   /**< Payload.                           */
    uint32_t timestamp_ms;           /**< Reception timestamp.               */
} tp_can_frame_t;

/**
 * @brief Driver state and diagnostics.
 */
typedef struct {
    uint32_t bitrate_bps;   /**< Configured bit rate.                       */
    uint32_t tx_frames;     /**< Frames successfully queued.                */
    uint32_t tx_retries;    /**< Times all mailboxes were busy.             */
    uint32_t tx_errors;     /**< Transmission errors reported by the controller.*/
    uint32_t rx_frames;     /**< Frames accepted by the filters.            */
    uint32_t rx_dropped;    /**< Frames lost because the FIFO overflowed.   */
    uint32_t rx_filtered;   /**< Frames discarded by the acceptance filters.*/
    uint16_t tec;           /**< Transmit error counter, from the controller.*/
    uint16_t rec;           /**< Receive error counter.                     */
    int      bus_off;       /**< Non-zero after the controller entered bus-off.*/
    int      bus_off_events;/**< Times bus-off has been entered.            */
    int      initialised;   /**< Non-zero after a successful init.          */
} tp_can_status_t;

/**
 * @brief Initialise the CAN controller.
 *
 * @param[in,out] st           Status block to populate.
 * @param[in]     bitrate_bps  Bit rate; @ref TP_CAN_BITRATE_BPS is the design
 *                             value and is the only value with an accepted
 *                             timing calculation in this revision.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_UNSUPPORTED.
 */
tp_status_t tp_can_init(tp_can_status_t *st, uint32_t bitrate_bps);

/**
 * @brief Send a classic data frame.
 *
 * @param[in] id   Identifier, 0..0x7FF.
 * @param[in] data Payload, may be NULL when @p dlc is 0.
 * @param[in] dlc  Length, 0..8.
 * @return @ref TP_OK; @ref TP_ERR_RANGE for a 29-bit identifier (this build
 *         only configures the 11-bit filter bank, so accepting one would be a
 *         silent data-corruption risk); @ref TP_ERR_OVERFLOW when no mailbox is
 *         free, in which case the caller should retry next period rather than
 *         block.
 */
tp_status_t tp_can_send(uint32_t id, const uint8_t *data, uint8_t dlc);

/**
 * @brief Receive one frame if the FIFO has one.
 *
 * @param[out] frame  Receives the frame.
 * @return @ref TP_OK, or @ref TP_ERR_NOT_READY when the FIFO is empty.
 */
tp_status_t tp_can_recv(tp_can_frame_t *frame);

/**
 * @brief Install an acceptance filter.
 *
 * @param[in] index  Filter slot, 0..TP_CAN_MAX_FILTERS-1.
 * @param[in] id     Identifier to accept.
 * @param[in] mask   Bits to compare; a mask of 0x7FF matches exactly, 0 matches
 *                   everything.  A zero mask on slot 0 is how the "listen to
 *                   the whole bus" diagnostic mode is configured.
 * @return @ref TP_OK or @ref TP_ERR_PARAM / @ref TP_ERR_RANGE.
 */
tp_status_t tp_can_set_filter(uint32_t index, uint32_t id, uint32_t mask);

/**
 * @brief Service the controller: drain the FIFO and update the diagnostics.
 *
 * The frames read here are counted and the error counters are refreshed from
 * the controller.  Any application-level dispatch is done by the communication
 * task, which calls this first.
 *
 * @param[in,out] st  Status block.
 * @return Number of frames drained.
 */
uint32_t tp_can_service(tp_can_status_t *st);

/**
 * @brief Whether the controller is in bus-off and needs recovery.
 *
 * A bus-off means the node transmitted 32 consecutive error frames, which in
 * practice means the wiring is wrong, the bit rate is wrong, or no other node
 * is present.  Recovery requires 128 occurrences of 11 recessive bits, which
 * the controller performs automatically; this flag simply reports the state so
 * the safety layer can raise a warning.
 *
 * @param[in] st  Status block.
 * @return Non-zero when in bus-off.
 */
int tp_can_is_bus_off(const tp_can_status_t *st);

/**
 * @brief Request a controller reset to recover from bus-off.
 *
 * @param[in,out] st  Status block.
 * @return @ref TP_OK or @ref TP_ERR_PARAM.
 */
tp_status_t tp_can_recover(tp_can_status_t *st);

/**
 * @brief Build a four-byte big-endian payload from a 32-bit value.
 *
 * CAN nodes disagree about byte order far more often than anyone expects;
 * centralising the conversion here (big-endian, i.e. network order) means the
 * robot speaks one convention and the documentation can state it once.
 *
 * @param[out] out    Destination, at least 4 bytes.
 * @param[in]  value  Value to encode.
 */
void tp_can_encode_u32(uint8_t *out, uint32_t value);

/**
 * @brief Decode a big-endian 32-bit value.
 *
 * @param[in] in  Source, at least 4 bytes.
 * @return Decoded value.
 */
uint32_t tp_can_decode_u32(const uint8_t *in);

#ifdef __cplusplus
}
#endif

#endif /* TP_CAN_DRIVER_H */
