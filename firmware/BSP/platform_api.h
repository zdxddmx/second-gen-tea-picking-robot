/**
 * @file    platform_api.h
 * @brief   The single hardware seam of the firmware.
 *
 * @details
 * # What this is
 * Every register access, every GPIO read and every byte put on a bus in this
 * project goes through the functions declared here.  Exactly two
 * implementations exist:
 *
 * | Backend | File | Built when |
 * |---|---|---|
 * | STM32F407VET6 silicon | `firmware/BSP/stm32f407/platform_stm32f407.c` | `-DTP_PLATFORM_STM32F407` |
 * | Deterministic physics  | `firmware/BSP/host/platform_host.c` | `-DTP_PLATFORM_HOST` |
 *
 * # Why one seam instead of a vtable per driver
 * A function-pointer vtable per device is the more "modern" answer and it is
 * the right one when a product ships two interchangeable sensors.  This robot
 * does not: the board is fixed and only the *target* changes.  A single
 * compile-time seam buys three concrete things that a vtable does not:
 *
 *   1. **Static linking, zero indirection.**  The 1 kHz control loop never
 *      makes an indirect call, so there is no branch-predictor/BTB pressure
 *      and no risk of a NULL vtable entry.  On a Cortex-M4 without a branch
 *      predictor this is measurable.
 *   2. **The compiler sees everything.**  With a single implementation per
 *      build, `-O2` can inline `tp_platform_motor_set_pwm` down to a register
 *      store, and `-Wl,--gc-sections` removes whatever the target does not use.
 *   3. **The port is auditable in one file.**  "Show me everything that touches
 *      the hardware" has a one-file answer, which is exactly what a safety
 *      review asks for.
 *
 * The seam is still narrow enough that the *device drivers* above it
 * (`motor.c`, `encoder.c`, `force_sensor.c`, ...) contain all of the logic —
 * unit conversion, limits, calibration, fault detection — and therefore all of
 * the unit-testable behaviour.
 *
 * # Contract for implementers
 *   - No function here may block for longer than a few microseconds.
 *   - `tp_platform_*_read` functions must never write to their output on
 *     failure, so a caller can keep the previous good value.
 *   - Every function must be safe to call before `tp_platform_init()`, except
 *     where noted; the boot self-test deliberately probes the hardware before
 *     the drivers are up.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_PLATFORM_API_H
#define TP_PLATFORM_API_H

#include <stdint.h>

#include "bsp_types.h"
#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Lifecycle and time
 * ========================================================================= */

/**
 * @brief Bring up clocks, GPIO, timers, ADC, DMA, encoders and the buses.
 *
 * Idempotent: calling it twice is harmless.  Must be called before any other
 * function in this file except the time helpers.
 *
 * @return @ref TP_OK when every peripheral initialised; the first failing
 *         peripheral's error otherwise.
 */
tp_status_t tp_platform_init(void);

/**
 * @brief Free-running millisecond counter since boot.
 *
 * Backed by the FreeRTOS tick on the target and by a monotonic clock on the
 * host.  Wraps after 2^32 ms (~49.7 days); all comparisons in the firmware use
 * unsigned subtraction so the wrap is handled correctly.
 *
 * @return Milliseconds since boot.
 */
uint32_t tp_platform_time_ms(void);

/**
 * @brief Busy-wait for at least @p ms milliseconds.
 *
 * @param[in] ms  Delay in milliseconds.  A value of 0 returns immediately.
 */
void tp_platform_delay_ms(uint32_t ms);

/**
 * @brief Microsecond-resolution busy-wait, for peripheral timing only.
 *
 * @param[in] us  Delay in microseconds.
 *
 * @warning Spins with interrupts enabled but does no work; never call it from
 *          the control task.
 */
void tp_platform_delay_us(uint32_t us);

/* =========================================================================
 * Discrete IO
 * ========================================================================= */

/**
 * @brief Read a discrete input.
 *
 * @param[in]  id     Input to read.
 * @param[out] level  Receives 1 for logic high, 0 for logic low.
 * @return @ref TP_OK, @ref TP_ERR_PARAM for an unknown @p id, or
 *         @ref TP_ERR_NOT_READY when the backend has no such input.
 *
 * @note The E-stop input is wired as a **normally-closed** loop: a broken wire
 *       reads as "asserted", which is the fail-safe polarity.  Callers get the
 *       raw level and @ref safety_monitor.c applies the polarity, so a
 *       mis-wired prototype is visible in the logs rather than silently
 *       inverted.
 */
tp_status_t tp_platform_io_read(tp_io_id_t id, uint8_t *level);

/**
 * @brief Read every discrete input at once.
 *
 * @return Bitmask with bit *n* set when @ref tp_io_id_t value *n* is high.
 */
uint8_t tp_platform_io_read_all(void);

/* =========================================================================
 * Motor drive
 * ========================================================================= */

/**
 * @brief Set the PWM duty of a motor channel.
 *
 * @param[in] ch        Motor channel.
 * @param[in] permille  Duty in per-mille, clamped by the backend to
 *                      `[-TP_MAX_PWM_PERMILLE, +TP_MAX_PWM_PERMILLE]`.
 *                      The sign selects the direction.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM for an unknown channel.
 */
tp_status_t tp_platform_motor_set_pwm(tp_motor_ch_t ch, float permille);

/**
 * @brief Enable or disable a motor channel's driver.
 *
 * Disabling must be the fastest possible reaction to a fault: on the target it
 * clears the enable GPIO, which the gate driver turns into a coast/brake
 * state without CPU involvement.
 *
 * @param[in] ch      Motor channel.
 * @param[in] enable  Non-zero to energise, 0 to disable.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM for an unknown channel.
 */
tp_status_t tp_platform_motor_enable(tp_motor_ch_t ch, int enable);

/**
 * @brief Query whether a motor channel is currently enabled.
 *
 * @param[in] ch  Motor channel.
 * @return 1 when energised, 0 when disabled or on an invalid channel.
 */
int tp_platform_motor_is_enabled(tp_motor_ch_t ch);

/**
 * @brief Read the motor current.
 *
 * @param[in]  ch  Motor channel.
 * @param[out] ma  Receives the signed current in milliamperes.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM / @ref TP_ERR_NOT_READY.
 */
tp_status_t tp_platform_motor_current_ma(tp_motor_ch_t ch, float *ma);

/* =========================================================================
 * Encoders
 * ========================================================================= */

/**
 * @brief Read an encoder's absolute count.
 *
 * The counter is 32-bit and free-running; the driver converts counts to
 * milliradians and handles the wrap.  The host backend quantises its internal
 * continuous angle to exactly the same count grid as the real hardware, so
 * quantisation effects appear in simulation too.
 *
 * @param[in]  ch     Encoder channel.
 * @param[out] count  Receives the signed count.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM / @ref TP_ERR_IO when the backend
 *         has flagged the encoder as faulty.
 */
tp_status_t tp_platform_encoder_read(tp_encoder_ch_t ch, int32_t *count);

/**
 * @brief Clear an encoder's accumulated count (used after homing).
 *
 * @param[in] ch  Encoder channel.
 * @return @ref TP_OK, or @ref TP_ERR_PARAM.
 */
tp_status_t tp_platform_encoder_reset(tp_encoder_ch_t ch);

/* =========================================================================
 * ADC
 * ========================================================================= */

/**
 * @brief Read one ADC channel.
 *
 * @param[in]  ch      ADC channel.
 * @param[out] counts  Receives the raw 12-bit conversion result (0..4095).
 * @return @ref TP_OK, or @ref TP_ERR_PARAM / @ref TP_ERR_NOT_READY.
 */
tp_status_t tp_platform_adc_read(tp_adc_ch_t ch, uint16_t *counts);

/**
 * @brief Read the most recent DMA block for a channel without blocking.
 *
 * On the target the ADC runs in circular DMA mode into a ping-pong buffer and
 * this returns the last completed half.  On the host it samples the physics
 * model.  Either way the call is non-blocking, which is what lets the sensor
 * task stay at a fixed 500 Hz period.
 *
 * @param[in]  ch      ADC channel.
 * @param[out] buffer  Receives the samples.
 * @param[in]  count   Number of samples requested.
 * @return Number of samples actually written (0 when no block is ready).
 */
uint32_t tp_platform_adc_read_block(tp_adc_ch_t ch, uint16_t *buffer,
                                    uint32_t count);

/* =========================================================================
 * Buses
 * ========================================================================= */

/**
 * @brief Write bytes to a bus.
 *
 * On the target the write is queued to a DMA ring and returns immediately; the
 * "fully transmitted" condition is only guaranteed by
 * @ref tp_platform_bus_tx_done.  This is deliberate: blocking the communication
 * task for the duration of a 60-byte frame at 921600 baud (0.65 ms) would be
 * tolerable, but blocking it for a Modbus retry would not.
 *
 * @param[in] bus   Bus to write to.
 * @param[in] data  Bytes to send.
 * @param[in] len   Number of bytes.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_OVERFLOW when the TX
 *         ring is full.
 */
tp_status_t tp_platform_bus_write(tp_bus_t bus, const uint8_t *data,
                                  uint32_t len);

/**
 * @brief Read available bytes from a bus.
 *
 * @param[in]  bus      Bus to read from.
 * @param[out] data     Destination buffer.
 * @param[in]  max_len  Capacity of @p data.
 * @return Number of bytes read; 0 when nothing is pending.
 */
uint32_t tp_platform_bus_read(tp_bus_t bus, uint8_t *data, uint32_t max_len);

/**
 * @brief Whether the bus transmitter has drained.
 *
 * @param[in] bus  Bus to query.
 * @return 1 when idle, 0 when a transfer is still in flight.
 */
int tp_platform_bus_tx_done(tp_bus_t bus);

/**
 * @brief Send a classic CAN data frame.
 *
 * Kept separate from @ref tp_platform_bus_write because a CAN frame is not a
 * byte stream: the identifier and DLC are part of the message.
 *
 * @param[in] id   11-bit standard identifier (29-bit extended is rejected).
 * @param[in] data Payload, may be NULL when @p dlc is 0.
 * @param[in] dlc  Data length code, 0..8.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_OVERFLOW when all TX
 *         mailboxes are busy (the caller must retry).
 */
tp_status_t tp_platform_can_send(uint32_t id, const uint8_t *data, uint8_t dlc);

/**
 * @brief Receive one classic CAN data frame if one is pending.
 *
 * @param[out] id   Receives the identifier.
 * @param[out] data Receives up to 8 payload bytes.
 * @param[out] dlc  Receives the data length code.
 * @return @ref TP_OK, or @ref TP_ERR_NOT_READY when the RX FIFO is empty.
 */
tp_status_t tp_platform_can_recv(uint32_t *id, uint8_t *data, uint8_t *dlc);

/* =========================================================================
 * Watchdog and reset diagnostics
 * ========================================================================= */

/**
 * @brief Start the independent watchdog with a timeout.
 *
 * @param[in] timeout_ms  Timeout in milliseconds; the backend rounds to the
 *                        nearest achievable value and logs the real one.
 * @return @ref TP_OK, or @ref TP_ERR_UNSUPPORTED on a backend without one.
 */
tp_status_t tp_platform_watchdog_init(uint32_t timeout_ms);

/**
 * @brief Feed the watchdog.
 *
 * Called from exactly one place — the safety task — after every supervised
 * task has reported in.  Feeding it from several tasks would defeat the point:
 * a hung control task would still be masked by a healthy logger.
 */
void tp_platform_watchdog_feed(void);

/**
 * @brief Whether the previous reset was caused by the watchdog.
 *
 * Read once at boot by the self-check; if it returns non-zero the firmware
 * refuses to auto-resume motion and raises @ref TP_FAULT_WATCHDOG.
 *
 * @return Non-zero when the reset cause was the watchdog.
 */
int tp_platform_watchdog_caused_reset(void);

/* =========================================================================
 * Critical sections
 * ========================================================================= */

/**
 * @brief Enter a critical section (interrupts masked up to a priority).
 *
 * @return An opaque key that must be passed back to
 *         @ref tp_platform_exit_critical.  On the host backend it is the saved
 *         interrupt-enable state, which makes the same code correct in a
 *         threaded process.
 */
uint32_t tp_platform_enter_critical(void);

/**
 * @brief Leave a critical section opened by @ref tp_platform_enter_critical.
 *
 * @param[in] key  Value returned by the matching enter call.
 */
void tp_platform_exit_critical(uint32_t key);

/* =========================================================================
 * Safety output
 * ========================================================================= */

/**
 * @brief Kill every actuator immediately, bypassing the normal control path.
 *
 * On the target this clears both enable GPIOs and zeroes the PWM compare
 * registers, then latches a software kill so a stray write cannot restart the
 * drives.  Called by the E-stop handler and by the critical-severity fault
 * path.  Must be idempotent.
 */
void tp_platform_emergency_kill(void);

/**
 * @brief Release the software kill latch so the drives can be re-enabled.
 *
 * Only the `RESET` command handler calls this, and only after
 * @ref tp_platform_io_read reports the physical E-stop has been released.
 */
void tp_platform_emergency_release(void);

/**
 * @brief Whether the software kill latch is currently set.
 * @return 1 when latched.
 */
int tp_platform_is_killed(void);

#ifdef __cplusplus
}
#endif

#endif /* TP_PLATFORM_API_H */
