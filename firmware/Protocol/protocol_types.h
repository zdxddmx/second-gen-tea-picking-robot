/**
 * @file    protocol_types.h
 * @brief   Wire-protocol vocabulary: command codes, payload layouts, limits.
 *
 * @details
 * # Frame layout
 * @verbatim
 *   +--------+--------+--------+--------+------------------+--------+--------+
 *   | 0xAA   | 0x55   | CMD    | LEN    | PAYLOAD[0..LEN-1]| CRC_LO | CRC_HI |
 *   +--------+--------+--------+--------+------------------+--------+--------+
 *     byte 0   byte 1   byte 2   byte 3      byte 4..         last 2 bytes
 * @endverbatim
 *
 *   - **Header `AA 55`**: a two-byte pattern with a low probability of
 *     appearing in arbitrary payload data, so the parser can resynchronise
 *     after a truncated frame or a burst of line noise.  One byte would be
 *     found inside payloads constantly; three would waste bandwidth.
 *   - **CMD**: command or report identifier, see @ref tp_cmd_t.
 *   - **LEN**: payload length, 0..@ref TP_PROTO_MAX_PAYLOAD.  A length byte
 *     rather than a fixed-size frame keeps the heartbeat at 6 bytes on the wire
 *     instead of padding every message to the largest one.
 *   - **CRC16**: CRC-16/MODBUS over `CMD | LEN | PAYLOAD` only.  The header is
 *     excluded because it is already a fixed pattern: including it would add no
 *     detection power while giving a corrupted header a second chance to
 *     validate.
 *
 * # Byte order
 * **Little-endian** for every multi-byte field, matching the STM32's native
 * order so the firmware can `memcpy` payloads into and out of structs without a
 * byte-swapping pass.  This is a deliberate deviation from network order, and
 * it is documented here because it is the single most common integration
 * mistake: a partner who assumes big-endian will read a 2.0 N grip force as
 * 8192 N.
 *
 * # Two protocols, one file
 * The framed protocol below is for this project's PC tools.  Modbus RTU (see
 * `rs485.h`) is the interoperability protocol for third-party PLCs.  Both
 * share @ref crc16.h.  Keeping the two vocabularies in separate files, but both
 * documented against the same CRC, makes it obvious which one a given byte
 * stream belongs to.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_PROTOCOL_TYPES_H
#define TP_PROTOCOL_TYPES_H

#include <stdint.h>

#include "bsp_types.h"
#include "compiler.h"
#include "config.h"
#include "fault_codes.h"
#include "robot_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Frame constants
 * ========================================================================= */

/** @brief First header byte. */
#define TP_PROTO_HDR0 TP_PROTO_HEADER0
/** @brief Second header byte. */
#define TP_PROTO_HDR1 TP_PROTO_HEADER1
/** @brief Fixed frame overhead: 2 header + 1 cmd + 1 len + 2 crc. */
#define TP_PROTO_FIXED_OVERHEAD 6u

/* =========================================================================
 * Commands
 * ========================================================================= */

/**
 * @brief Command and report identifiers.
 *
 * The ranges are deliberate and worth memorising when reading a bus capture:
 *
 * | Range | Owner | Meaning |
 * |---|---|---|
 * | `0x01-0x0F` | host -> robot | session and liveliness |
 * | `0x10-0x1F` | host -> robot | motion commands |
 * | `0x20-0x2F` | host -> robot | queries |
 * | `0x30-0x3F` | host -> robot | parameter writes |
 * | `0x40-0x4F` | host -> robot | safety |
 * | `0x50-0x7F` | host -> robot | reserved for future commands |
 * | `0x80-0xDF` | robot -> host | responses and reports |
 * | `0xE0-0xFF` | robot -> host | errors and acknowledgements |
 */
typedef enum {
    TP_CMD_HEARTBEAT        = 0x01, /**< host->robot, `u32 seq`.              */

    TP_CMD_SET_TARGET       = 0x10, /**< host->robot, target coordinates.     */
    TP_CMD_START_PICK       = 0x11, /**< host->robot, `u16 target_id`.        */
    TP_CMD_STOP             = 0x12, /**< host->robot, empty.                  */
    TP_CMD_RESET            = 0x13, /**< host->robot, empty.                  */

    TP_CMD_GET_STATUS       = 0x20, /**< host->robot, empty.                  */
    TP_CMD_GET_SENSOR       = 0x21, /**< host->robot, empty.                  */
    TP_CMD_GET_FAULT        = 0x22, /**< host->robot, empty.                  */

    TP_CMD_SET_PID          = 0x30, /**< host->robot, `u8 loop, f32 kp/ki/kd`.*/
    TP_CMD_GET_PID          = 0x31, /**< host->robot, `u8 loop`.              */
    TP_CMD_SET_FORCE_TARGET = 0x32, /**< host->robot, `u16 millinewtons`.     */
    TP_CMD_SET_MODE         = 0x33, /**< host->robot, `u8 mode`.              */

    TP_CMD_EMERGENCY_STOP   = 0x40, /**< host->robot, empty.                  */
    TP_CMD_CLEAR_FAULT      = 0x41, /**< host->robot, empty.                  */
    TP_CMD_CALIBRATE_FORCE  = 0x42, /**< host->robot, `u8 kind`.              */

    TP_CMD_ACK              = 0x80, /**< robot->host, `u8 acked_cmd`.         */
    TP_CMD_HEARTBEAT_ACK    = 0x81, /**< robot->host, `u32 seq`.              */

    TP_CMD_STATUS_RPT       = 0xA0, /**< robot->host, full status.            */
    TP_CMD_SENSOR_RPT       = 0xA1, /**< robot->host, raw sensor values.      */
    TP_CMD_PID_RPT          = 0xA2, /**< robot->host, gains of one loop.      */
    TP_CMD_FAULT_RPT        = 0xB0, /**< robot->host, fault announcement.     */
    TP_CMD_TELEMETRY        = 0xC0, /**< robot->host, periodic sample.        */

    TP_CMD_NACK             = 0xE0  /**< robot->host, `u8 ref_cmd, u8 err`.   */
} tp_cmd_t;

/** @brief Whether a command code is a robot-to-host report. */
#define TP_CMD_IS_REPORT(c) (((uint8_t)(c) >= 0x80u) ? 1 : 0)

/* =========================================================================
 * Payload layouts
 * ========================================================================= */

/**
 * @brief `SET_TARGET` payload: 10 bytes.
 *
 * Coordinates are relative to the arm's base frame, in millimetres, in the
 * camera's convention: `+x` along the tea row, `+y` across it, `+z` upward.
 * `confidence` is the vision system's own 0..100 score; `target_id` lets the
 * robot report which bud a cycle acted on, so a log can be joined to the vision
 * system's log without timestamps having to line up.
 */
typedef struct {
    int16_t  x_mm;        /**< Target X, millimetres.                        */
    int16_t  y_mm;        /**< Target Y, millimetres.                        */
    int16_t  z_mm;        /**< Target Z, millimetres.                        */
    uint8_t  confidence;  /**< 0..100.                                       */
    uint8_t  reserved;    /**< Padding to keep the struct 10 bytes; sent as 0.*/
    uint16_t target_id;   /**< Vision system's identifier for this bud.      */
} tp_payload_set_target_t;

/** @brief `START_PICK` payload. */
typedef struct {
    uint16_t target_id;   /**< 0 means "use the most recent target".         */
} tp_payload_start_pick_t;

/** @brief `SET_PID` / `PID_RPT` payload. */
typedef struct {
    uint8_t loop_id;      /**< @ref tp_pid_loop_t.                           */
    float   kp;           /**< IEEE-754 single, little-endian.               */
    float   ki;           /**< IEEE-754 single, little-endian.               */
    float   kd;           /**< IEEE-754 single, little-endian.               */
} tp_payload_pid_t;

/** @brief Identifiers for the three tunable loops. */
typedef enum {
    TP_PID_LOOP_POSITION = 0, /**< Outer position loop.                      */
    TP_PID_LOOP_VELOCITY = 1, /**< Inner velocity loop.                      */
    TP_PID_LOOP_FORCE    = 2, /**< Gripper force loop.                       */
    TP_PID_LOOP_COUNT    = 3
} tp_pid_loop_t;

/** @brief `SET_FORCE_TARGET` payload. */
typedef struct {
    uint16_t force_mn;    /**< Target grip force in millinewtons.            */
} tp_payload_force_target_t;

/** @brief `SET_MODE` payload. */
typedef struct {
    uint8_t mode;         /**< @ref tp_robot_mode_t.                         */
} tp_payload_mode_t;

/** @brief Robot operating modes selectable from the host. */
typedef enum {
    TP_MODE_IDLE     = 0, /**< Motors enabled, no motion commanded.          */
    TP_MODE_AUTO     = 1, /**< Accept targets and run pick cycles.           */
    TP_MODE_MANUAL   = 2, /**< Direct velocity commands, for commissioning.  */
    TP_MODE_CALIBRATE= 3  /**< Sensor calibration, motion inhibited.         */
} tp_robot_mode_t;

/** @brief `ACK` payload. */
typedef struct {
    uint8_t acked_cmd;    /**< Command being acknowledged.                   */
} tp_payload_ack_t;

/** @brief `NACK` payload. */
typedef struct {
    uint8_t ref_cmd;      /**< Command that was rejected.                    */
    uint8_t err_code;     /**< A @ref tp_status_t value, negated.            */
    uint8_t detail;       /**< Command-specific extra information.           */
} tp_payload_nack_t;

/**
 * @brief `STATUS_RPT` payload: 16 bytes.
 *
 * Sent in response to `GET_STATUS` and periodically at the telemetry rate.  It
 * is the single message a monitoring tool needs to render a full dashboard.
 */
typedef struct {
    uint8_t  state;        /**< @ref tp_robot_state_t.                       */
    uint8_t  fault_code;   /**< @ref tp_fault_code_t.                        */
    int32_t  pos_mrad;     /**< Joint angle, milliradians.                   */
    int16_t  vel_mrad_s;   /**< Joint velocity, milliradians per second.     */
    int16_t  force_mn;     /**< Grip force, millinewtons.                    */
    int16_t  pwm_permille; /**< Applied joint duty, per-mille.               */
    int16_t  current_ma;   /**< Joint motor current, milliamperes.           */
    uint16_t cycle_count;  /**< Completed pick cycles since boot.            */
} tp_payload_status_t;

/** @brief `SENSOR_RPT` payload: 13 bytes. */
typedef struct {
    int32_t  pos_mrad;     /**< Joint angle, milliradians.                   */
    int16_t  vel_mrad_s;   /**< Joint velocity.                              */
    int16_t  force_mn;     /**< Grip force, millinewtons.                    */
    uint16_t adc_raw;      /**< Raw force ADC counts, for calibration work.  */
    int16_t  current_ma;   /**< Joint motor current.                         */
    int16_t  temp_c_x10;   /**< Board temperature, tenths of a degree C.     */
} tp_payload_sensor_t;

/** @brief `FAULT_RPT` payload: 6 bytes. */
typedef struct {
    uint8_t  fault_code;   /**< @ref tp_fault_code_t.                        */
    uint32_t timestamp_ms; /**< When the fault was raised.                   */
    uint8_t  detail;       /**< Fault-specific extra information.            */
} tp_payload_fault_t;

/**
 * @brief `TELEMETRY` payload: 16 bytes.
 *
 * Deliberately smaller than `STATUS_RPT`: at 20 Hz a monitoring tool does not
 * need `cycle_count` and the fault detail, and the reduced size keeps the link
 * budget comfortable even when the debug log shares the UART.
 */
typedef struct {
    uint32_t t_ms;         /**< Robot timestamp.                             */
    int32_t  pos_mrad;     /**< Joint angle.                                 */
    int16_t  vel_mrad_s;   /**< Joint velocity.                              */
    int16_t  force_mn;     /**< Grip force.                                  */
    int16_t  pwm_permille; /**< Applied duty.                                */
    uint8_t  state;        /**< @ref tp_robot_state_t.                       */
    uint8_t  fault;        /**< @ref tp_fault_code_t.                        */
} tp_payload_telemetry_t;

/* =========================================================================
 * Little-endian field codecs
 * ========================================================================= */

/**
 * @brief Encode a signed 16-bit value little-endian.
 *
 * @param[out] out  Destination, at least 2 bytes.
 * @param[in]  v    Value.
 */
void tp_le_put_i16(uint8_t *out, int16_t v);

/**
 * @brief Encode an unsigned 16-bit value little-endian.
 *
 * @param[out] out  Destination, at least 2 bytes.
 * @param[in]  v    Value.
 */
void tp_le_put_u16(uint8_t *out, uint16_t v);

/**
 * @brief Encode a signed 32-bit value little-endian.
 *
 * @param[out] out  Destination, at least 4 bytes.
 * @param[in]  v    Value.
 */
void tp_le_put_i32(uint8_t *out, int32_t v);

/**
 * @brief Encode an unsigned 32-bit value little-endian.
 *
 * @param[out] out  Destination, at least 4 bytes.
 * @param[in]  v    Value.
 */
void tp_le_put_u32(uint8_t *out, uint32_t v);

/**
 * @brief Decode a signed 16-bit little-endian value.
 *
 * @param[in] in  Source, at least 2 bytes.
 * @return Decoded value.
 */
int16_t tp_le_get_i16(const uint8_t *in);

/**
 * @brief Decode an unsigned 16-bit little-endian value.
 *
 * @param[in] in  Source, at least 2 bytes.
 * @return Decoded value.
 */
uint16_t tp_le_get_u16(const uint8_t *in);

/**
 * @brief Decode a signed 32-bit little-endian value.
 *
 * @param[in] in  Source, at least 4 bytes.
 * @return Decoded value.
 */
int32_t tp_le_get_i32(const uint8_t *in);

/**
 * @brief Decode an unsigned 32-bit little-endian value.
 *
 * @param[in] in  Source, at least 4 bytes.
 * @return Decoded value.
 */
uint32_t tp_le_get_u32(const uint8_t *in);

/**
 * @brief Convert a float to a per-mille integer without a library call.
 *
 * Used for the wire encoding of forces and duties, where a fixed-point
 * representation is both smaller and more robust than a raw float: an integer
 * cannot be a NaN, and the rounding is explicit and testable.
 *
 * @param[in] value    Value to convert.
 * @param[in] scale    Units per integer step (e.g. 1000 for newtons->mN).
 * @param[in] min_out  Lower saturation bound.
 * @param[in] max_out  Upper saturation bound.
 * @return Saturated, rounded integer.
 */
int32_t tp_fixed_from_float(float value, float scale, int32_t min_out,
                            int32_t max_out);

/**
 * @brief Convert a fixed-point integer back to a float.
 *
 * @param[in] value  Integer value.
 * @param[in] scale  Units per integer step.
 * @return Floating-point value.
 */
float tp_fixed_to_float(int32_t value, float scale);

/** @brief Scale factor for newtons to millinewtons. */
#define TP_WIRE_N_TO_MN 1000.0f
/** @brief Scale factor for milliradians per second (identical to 1:1 int16). */
#define TP_WIRE_MRAD_S 1.0f
/** @brief Scale factor for milliamperes (identical to 1:1 int16). */
#define TP_WIRE_MA 1.0f
/** @brief Scale factor for per-mille duty (identical to 1:1 int16). */
#define TP_WIRE_PERMILLE 1.0f
/** @brief Scale factor for millimetres to micrometres. */
#define TP_WIRE_MM_TO_UM 1000.0f

#ifdef __cplusplus
}
#endif

#endif /* TP_PROTOCOL_TYPES_H */
