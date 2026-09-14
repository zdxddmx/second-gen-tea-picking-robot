/**
 * @file    bsp_types.h
 * @brief   Hardware-level types shared by the BSP, the drivers and the tests.
 *
 * @details
 * This header is the *vocabulary* of the layer directly above the silicon: how
 * many motors, which ADC channel measures what, which GPIO is a limit switch.
 * It contains no behaviour and no hardware dependency, which is what allows the
 * same driver sources to be compiled against either the STM32F407 backend or
 * the host physics backend.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_BSP_TYPES_H
#define TP_BSP_TYPES_H

#include <stdint.h>

#include "compiler.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Channels and identifiers
 * ========================================================================= */

/**
 * @brief Motor drive channels.
 *
 * Two channels cover the current machine: the Cartesian arm's dominant joint
 * (a rotary axis driven through a planetary gearbox) and the gripper jaw
 * (a small linear actuator).  The arm's remaining joints are passive/fixed, so
 * no third channel is reserved until the hardware exists.
 */
typedef enum {
    TP_MOTOR_JOINT = 0,   /**< Arm joint, TIM1_CH1 PWM + DIR/EN GPIO.        */
    TP_MOTOR_GRIPPER = 1, /**< Gripper jaw, TIM1_CH2 PWM + DIR/EN GPIO.      */
    TP_MOTOR_COUNT = 2
} tp_motor_ch_t;

/** @brief Quadrature encoder channels. */
typedef enum {
    TP_ENCODER_JOINT = 0,   /**< TIM2, quadrature X4 decoding.               */
    TP_ENCODER_GRIPPER = 1, /**< TIM4, quadrature X4 decoding.               */
    TP_ENCODER_COUNT = 2
} tp_encoder_ch_t;

/** @brief ADC channels, in DMA rank order. */
typedef enum {
    TP_ADC_FORCE = 0,          /**< PA0, ADC1_IN0: load-cell amplifier.      */
    TP_ADC_CURRENT_JOINT = 1,  /**< PA1, ADC1_IN1: joint driver shunt amp.   */
    TP_ADC_CURRENT_GRIPPER = 2,/**< PA2, ADC1_IN2: gripper shunt amp.        */
    TP_ADC_VBUS = 3,           /**< PA3, ADC1_IN3: 24 V rail divider.        */
    TP_ADC_COUNT = 4
} tp_adc_ch_t;

/** @brief Discrete inputs. */
typedef enum {
    TP_IO_ESTOP = 0,            /**< NC loop, low = asserted.                */
    TP_IO_LIMIT_JOINT_MIN = 1,  /**< Arm negative soft/hard limit.           */
    TP_IO_LIMIT_JOINT_MAX = 2,  /**< Arm positive hard limit.                */
    TP_IO_LIMIT_GRIPPER_OPEN = 3,/**< Jaw fully open.                        */
    TP_IO_LIMIT_GRIPPER_CLOSE = 4,/**< Jaw fully closed (mechanical stop).   */
    TP_IO_DRIVER_FAULT = 5,     /**< nFAULT from the gate driver.            */
    TP_IO_COUNT = 6
} tp_io_id_t;

/** @brief Communication links. */
typedef enum {
    TP_BUS_DEBUG_UART = 0, /**< USART1, 115200, human-readable logs.         */
    TP_BUS_VISION_UART = 1,/**< USART2, 921600, frame protocol to the host.  */
    TP_BUS_RS485 = 2,      /**< USART3 + DE pin, Modbus RTU.                 */
    TP_BUS_CAN = 3,        /**< CAN1, 500 kbit/s, motor drives / aux nodes.  */
    TP_BUS_COUNT = 4
} tp_bus_t;

/* =========================================================================
 * Mechanical constants
 * ========================================================================= */

/**
 * @brief Encoder counts per revolution of the joint, after X4 decoding.
 *
 * A 1024-line incremental encoder in quadrature yields 4096 counts/rev.
 * Documented as a constant because every position conversion in the driver
 * depends on it and a mismatch here is the classic cause of a "the arm moves
 * exactly 4x too far" bug.
 */
#define TP_ENCODER_COUNTS_PER_REV (4096)

/** @brief Encoder counts per millimetre of jaw travel (gripper lead screw). */
#define TP_GRIPPER_COUNTS_PER_MM (512.0f)

/** @brief Gearbox reduction between the motor and the joint output. */
#define TP_JOINT_GEAR_RATIO (50.0f)

/** @brief Milliradians per encoder count at the joint output. */
#define TP_MRAD_PER_COUNT \
    ((2.0f * 3.14159265358979f * 1000.0f) / (float)TP_ENCODER_COUNTS_PER_REV)

/** @brief ADC reference voltage, duplicate of the `config.h` value for the BSP. */
#define TP_BSP_ADC_VREF_V (3.300f)

/* =========================================================================
 * Raw sample records
 * ========================================================================= */

/**
 * @brief One raw (unscaled) acquisition set produced by the sensor task.
 *
 * Kept as a plain struct so it can be copied into a queue by value, which
 * avoids any dynamic allocation on the hot path.
 */
typedef struct {
    uint32_t tick_ms;                       /**< Acquisition timestamp.      */
    int32_t  encoder_total[TP_ENCODER_COUNT];/**< Absolute encoder counts.   */
    int32_t  encoder_delta[TP_ENCODER_COUNT];/**< Counts since last sample.  */
    uint16_t adc_counts[TP_ADC_COUNT];      /**< Raw 12-bit ADC values.      */
    uint8_t  io_state;                      /**< Bitmask of @ref tp_io_id_t. */
} tp_raw_sample_t;

/**
 * @brief Engineering-unit sample derived from a @ref tp_raw_sample_t.
 *
 * This is the interface between the BSP and the control layer: everything
 * above this point works in physical units, never in counts or volts.
 */
typedef struct {
    uint32_t tick_ms;      /**< Timestamp of the originating raw sample.     */
    float    position_mrad;/**< Joint angle.                                 */
    float    velocity_mrad_s; /**< Joint angular velocity.                   */
    float    current_ma;   /**< Joint motor current.                         */
    float    gripper_mm;   /**< Jaw gap.                                     */
    float    force_n;      /**< Grip force.                                  */
    float    vbus_v;       /**< Supply rail.                                 */
    uint8_t  io_state;     /**< Bitmask of @ref tp_io_id_t.                  */
    uint8_t  valid;        /**< Bitmask of valid fields, see below.          */
} tp_measurement_t;

/** @brief Validity bits for @ref tp_measurement_t::valid. */
#define TP_MEAS_VALID_POSITION (1u << 0)
#define TP_MEAS_VALID_VELOCITY (1u << 1)
#define TP_MEAS_VALID_FORCE    (1u << 2)
#define TP_MEAS_VALID_CURRENT  (1u << 3)
#define TP_MEAS_VALID_VBUS     (1u << 4)
#define TP_MEAS_VALID_ALL      (0x1Fu)

#ifdef __cplusplus
}
#endif

#endif /* TP_BSP_TYPES_H */
