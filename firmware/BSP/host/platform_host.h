/**
 * @file    platform_host.h
 * @brief   Host-only extensions to the platform seam (simulation control).
 *
 * @details
 * `platform_api.h` is the *product* interface and is identical on both targets.
 * This header adds the things that only make sense when the hardware is a
 * physics model: reading the truth behind the sensors, injecting faults,
 * driving the simulated clock, and shimming the communication buses so a PC
 * test can talk to the firmware.
 *
 * Nothing in `firmware/Core`, `firmware/Control`, `firmware/Application`,
 * `firmware/Protocol` or `firmware/Safety` may include this header.  That rule
 * is what guarantees the STM32 build is the *same program* and not a
 * lookalike.  A grep for `platform_host.h` outside `BSP/host` and the test
 * tree is a valid CI check (see `scripts/check_layering.py`).
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_PLATFORM_HOST_H
#define TP_PLATFORM_HOST_H

#include <stdint.h>

#include "plant_model.h"
#include "platform_api.h"
#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Size of each simulated bus buffer, bytes. */
#define TP_HOST_BUS_BUFFER_SIZE 1024u

/**
 * @brief Access the live plant state.
 *
 * The simulator, the demo script and the unit tests all need the ground truth
 * to compare against what the firmware *believes*; that comparison is the whole
 * point of running the real control code against a model.
 *
 * @return Pointer to the single plant instance.  Never NULL; valid after
 *         @ref tp_platform_init.
 */
tp_plant_t *tp_host_platform_plant(void);

/**
 * @brief Queue bytes as if they had arrived on a bus.
 *
 * This is how a PC test plays the role of the vision system: it builds a real
 * protocol frame, injects it here, and the firmware's own parser and dispatcher
 * handle it — no test doubles inside the firmware.
 *
 * @param[in] bus   Bus to inject into.
 * @param[in] data  Bytes.
 * @param[in] len   Number of bytes.
 * @return @ref TP_OK, @ref TP_ERR_PARAM, or @ref TP_ERR_OVERFLOW when the RX
 *         buffer is full (the injection is then dropped whole).
 */
tp_status_t tp_host_platform_inject_rx(tp_bus_t bus, const uint8_t *data,
                                       uint32_t len);

/**
 * @brief Drain bytes the firmware has transmitted on a bus.
 *
 * @param[in]  bus      Bus to drain.
 * @param[out] out      Destination buffer.
 * @param[in]  max_len  Capacity of @p out.
 * @return Number of bytes copied; 0 when nothing was transmitted.
 */
uint32_t tp_host_platform_take_tx(tp_bus_t bus, uint8_t *out, uint32_t max_len);

/**
 * @brief Peek at transmitted bytes without consuming them.
 *
 * @param[in]  bus      Bus.
 * @param[out] out      Destination buffer.
 * @param[in]  max_len  Capacity of @p out.
 * @return Number of bytes copied.
 */
uint32_t tp_host_platform_peek_tx(tp_bus_t bus, uint8_t *out, uint32_t max_len);

/**
 * @brief Advance the simulated clock and integrate the plant.
 *
 * Registered as the OSAL tick hook by `main_host.c`, so it is called
 * automatically once the scheduler starts.  Tests that drive the firmware by
 * calling `*_step()` directly must call this themselves between steps, which is
 * exactly what makes those tests deterministic.
 *
 * @param[in] elapsed_us  Microseconds of simulated time to advance.
 */
void tp_host_platform_advance(uint32_t elapsed_us);

/**
 * @brief Override the simulated supply voltage.
 *
 * Used by the sensor-fault scenario to demonstrate that the firmware notices a
 * sagging rail before the motors misbehave.
 *
 * @param[in] volts  Rail voltage in volts, clamped to a plausible range.
 */
void tp_host_platform_set_vbus(float volts);

/**
 * @brief Return the plant to its power-on state and clear the bus buffers.
 */
void tp_host_platform_reset(void);

/**
 * @brief Number of bytes dropped because a bus RX buffer was full.
 *
 * @param[in] bus  Bus to query.
 * @return Dropped byte count since the last reset.
 */
uint32_t tp_host_platform_rx_dropped(tp_bus_t bus);

/**
 * @brief Print one line describing a bus buffer, for debugging a test.
 *
 * @param[in] bus    Bus to dump.
 * @param[in] label  Prefix for the line.
 */
void tp_host_platform_dump_tx(tp_bus_t bus, const char *label);

#ifdef __cplusplus
}
#endif

#endif /* TP_PLATFORM_HOST_H */
