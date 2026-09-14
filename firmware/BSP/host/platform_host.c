/**
 * @file    platform_host.c
 * @brief   Simulation backend for the platform seam.
 * @copyright MIT License, see LICENSE.
 */

#include "platform_host.h"

#include <stdio.h>
#include <string.h>

#include "compiler.h"
#include "config.h"
#include "logger.h"
#include "ring_buffer.h"
#include "tp_format.h"

/** @brief Simulated time resolution: the control period. */
#define HOST_TICK_US 1000u

/** @brief Lifecycle state of the backend. */
typedef struct {
    int      initialised;   /**< Set by @ref tp_platform_init.              */
    uint32_t time_ms;       /**< Simulated millisecond clock.               */
    float    pwm[TP_MOTOR_COUNT];      /**< Commanded duty per channel.      */
    int      enabled[TP_MOTOR_COUNT];  /**< Driver enable state per channel. */
    int32_t  encoder_zero[TP_ENCODER_COUNT]; /**< Counts at the last reset.  */
    int      killed;        /**< Software E-stop latch.                     */
    uint32_t wdg_timeout_ms;/**< Configured watchdog period.                */
    uint32_t wdg_last_feed_ms; /**< Simulated time of the last feed.        */
    int      wdg_enabled;   /**< Non-zero once the watchdog is armed.       */
    uint32_t wdg_timeouts;  /**< Times the watchdog would have fired.       */
    int      wdg_caused_reset; /**< Reported once by the boot self-check.   */
    uint32_t critical_depth;/**< Nesting depth of critical sections.        */
    uint32_t isr_state;     /**< Saved "interrupt enable" for the host.     */
} host_state_t;

/** @brief Per-bus simulated buffers. */
typedef struct {
    uint8_t          rx_storage[TP_HOST_BUS_BUFFER_SIZE];
    uint8_t          tx_storage[TP_HOST_BUS_BUFFER_SIZE];
    tp_ring_buffer_t rx;
    tp_ring_buffer_t tx;
    uint32_t         dropped;
} host_bus_t;

static host_state_t s_host;
static host_bus_t   s_bus[TP_BUS_COUNT];
static tp_plant_t   s_plant;

/* =========================================================================
 * Lifecycle
 * ========================================================================= */

tp_status_t tp_platform_init(void)
{
    int i;

    if (s_host.initialised != 0) {
        return TP_OK; /* idempotent */
    }
    (void)memset(&s_host, 0, sizeof(s_host));
    for (i = 0; i < (int)TP_BUS_COUNT; ++i) {
        (void)tp_ring_buffer_init(&s_bus[i].rx, s_bus[i].rx_storage,
                                  sizeof(s_bus[i].rx_storage));
        (void)tp_ring_buffer_init(&s_bus[i].tx, s_bus[i].tx_storage,
                                  sizeof(s_bus[i].tx_storage));
        s_bus[i].dropped = 0u;
    }
    (void)tp_plant_init(&s_plant);

    for (i = 0; i < (int)TP_MOTOR_COUNT; ++i) {
        s_host.enabled[i] = 1;
    }
    s_host.initialised = 1;
    s_host.time_ms = 0u;
    LOG_INFO(TP_MOD_MAIN, "host platform ready (simulation mode, 1 ms tick)");
    return TP_OK;
}

void tp_host_platform_reset(void)
{
    int i;

    (void)tp_plant_reset(&s_plant);
    for (i = 0; i < (int)TP_BUS_COUNT; ++i) {
        tp_ring_buffer_flush(&s_bus[i].rx);
        tp_ring_buffer_flush(&s_bus[i].tx);
        s_bus[i].dropped = 0u;
    }
    for (i = 0; i < (int)TP_MOTOR_COUNT; ++i) {
        s_host.pwm[i] = 0.0f;
        s_host.enabled[i] = 1;
        s_host.encoder_zero[i] = 0;
    }
    s_host.killed = 0;
    s_host.time_ms = 0u;
    s_host.wdg_enabled = 0;
    s_host.wdg_timeouts = 0u;
}

tp_plant_t *tp_host_platform_plant(void)
{
    return &s_plant;
}

/* =========================================================================
 * Time
 * ========================================================================= */

uint32_t tp_platform_time_ms(void)
{
    return s_host.time_ms;
}

void tp_platform_delay_ms(uint32_t ms)
{
    /* In simulation, "waiting" is free: advance the clock and the plant so
     * that the caller observes the same effect as real elapsed time, without
     * making a unit test sleep. */
    tp_host_platform_advance(ms * 1000u);
}

void tp_platform_delay_us(uint32_t us)
{
    if (us == 0u) {
        return;
    }
    /* Sub-millisecond waits are real hardware timing (ADC settle, RS485 DE
     * turnaround).  The plant has no sub-tick behaviour worth modelling, so
     * only the fractional-millisecond carry is accumulated. */
    static uint32_t carry_us;
    carry_us += us;
    if (carry_us >= HOST_TICK_US) {
        uint32_t whole_ms = carry_us / HOST_TICK_US;
        carry_us -= whole_ms * HOST_TICK_US;
        tp_host_platform_advance(whole_ms * HOST_TICK_US);
    }
}

void tp_host_platform_advance(uint32_t elapsed_us)
{
    uint32_t remaining_us = elapsed_us;
    float    dt_s = (float)HOST_TICK_US / 1000000.0f;

    while (remaining_us >= HOST_TICK_US) {
        /* A hardware kill forces both duties to zero before the plant sees
         * them, mirroring what clearing the gate-driver enable pin does. */
        float pwm_joint = (s_host.killed != 0) ? 0.0f : s_host.pwm[TP_MOTOR_JOINT];
        float pwm_grip = (s_host.killed != 0) ? 0.0f : s_host.pwm[TP_MOTOR_GRIPPER];

        tp_plant_step(&s_plant, pwm_joint, pwm_grip, dt_s);
        s_host.time_ms += 1u;
        remaining_us -= HOST_TICK_US;

        /* Watchdog supervision, so the fault path is exercised in simulation
         * exactly as it would be on the bench. */
        if (s_host.wdg_enabled != 0) {
            if ((s_host.time_ms - s_host.wdg_last_feed_ms) > s_host.wdg_timeout_ms) {
                s_host.wdg_timeouts++;
                s_host.wdg_caused_reset = 1;
                s_host.wdg_enabled = 0;
                LOG_ERROR(TP_MOD_SAFETY,
                          "watchdog expired after %lu ms without a feed",
                          (unsigned long)s_host.wdg_timeout_ms);
            }
        }
    }
}

void tp_host_platform_set_vbus(float volts)
{
    if (!TP_ISFINITE(volts)) {
        return;
    }
    s_plant.vbus_v = TP_CLAMP(volts, 5.0f, 30.0f);
}

/* =========================================================================
 * Discrete IO
 * ========================================================================= */

tp_status_t tp_platform_io_read(tp_io_id_t id, uint8_t *level)
{
    uint8_t mask;

    if (level == NULL) {
        return TP_ERR_PARAM;
    }
    if ((int)id < 0 || id >= TP_IO_COUNT) {
        return TP_ERR_PARAM;
    }
    mask = tp_plant_read_io(&s_plant);
    *level = (uint8_t)((mask >> (int)id) & 0x1u);
    return TP_OK;
}

uint8_t tp_platform_io_read_all(void)
{
    return tp_plant_read_io(&s_plant);
}

/* =========================================================================
 * Motor drive
 * ========================================================================= */

tp_status_t tp_platform_motor_set_pwm(tp_motor_ch_t ch, float permille)
{
    if (((int)ch < 0) || (ch >= TP_MOTOR_COUNT)) {
        return TP_ERR_PARAM;
    }
    if (!TP_ISFINITE(permille)) {
        return TP_ERR_RANGE;
    }
    if (s_host.killed != 0) {
        /* A killed drive ignores writes.  This is what makes the E-stop
         * latch meaningful rather than advisory. */
        s_host.pwm[ch] = 0.0f;
        return TP_ERR_FAULT_ACTIVE;
    }
    s_host.pwm[ch] = TP_CLAMP(permille, -TP_MAX_PWM_PERMILLE, TP_MAX_PWM_PERMILLE);
    return TP_OK;
}

tp_status_t tp_platform_motor_enable(tp_motor_ch_t ch, int enable)
{
    if (((int)ch < 0) || (ch >= TP_MOTOR_COUNT)) {
        return TP_ERR_PARAM;
    }
    s_host.enabled[ch] = (enable != 0) ? 1 : 0;
    if (s_host.enabled[ch] == 0) {
        s_host.pwm[ch] = 0.0f;
    }
    return TP_OK;
}

int tp_platform_motor_is_enabled(tp_motor_ch_t ch)
{
    if (((int)ch < 0) || (ch >= TP_MOTOR_COUNT)) {
        return 0;
    }
    return s_host.enabled[ch];
}

tp_status_t tp_platform_motor_current_ma(tp_motor_ch_t ch, float *ma)
{
    float counts;

    if (ma == NULL) {
        return TP_ERR_PARAM;
    }
    if (((int)ch < 0) || (ch >= TP_MOTOR_COUNT)) {
        return TP_ERR_PARAM;
    }
    counts = (float)tp_plant_read_current_counts(&s_plant, (int)ch);
    /* Invert the shunt-amplifier mapping documented in plant_model.c. */
    *ma = ((counts - 2048.0f) / 1000.0f) * 5.0f * 1000.0f;
    return TP_OK;
}

/* =========================================================================
 * Encoders
 * ========================================================================= */

tp_status_t tp_platform_encoder_read(tp_encoder_ch_t ch, int32_t *count)
{
    if (count == NULL) {
        return TP_ERR_PARAM;
    }
    if (((int)ch < 0) || (ch >= TP_ENCODER_COUNT)) {
        return TP_ERR_PARAM;
    }
    if (s_plant.encoder_fault != 0) {
        return TP_ERR_IO;
    }
    if (ch == TP_ENCODER_JOINT) {
        *count = tp_plant_read_encoder_count(&s_plant) - s_host.encoder_zero[ch];
    } else {
        /* The jaw encoder is a linear scale: counts are derived from the gap
         * at exactly the resolution the real quadrature scale provides. */
        *count = (int32_t)((TP_GRIPPER_MAX_MM - s_plant.gripper.gap_mm)
                           * TP_GRIPPER_COUNTS_PER_MM)
               - s_host.encoder_zero[ch];
    }
    return TP_OK;
}

tp_status_t tp_platform_encoder_reset(tp_encoder_ch_t ch)
{
    int32_t current;

    if (((int)ch < 0) || (ch >= TP_ENCODER_COUNT)) {
        return TP_ERR_PARAM;
    }
    if (ch == TP_ENCODER_JOINT) {
        current = tp_plant_read_encoder_count(&s_plant);
    } else {
        current = (int32_t)((TP_GRIPPER_MAX_MM - s_plant.gripper.gap_mm)
                            * TP_GRIPPER_COUNTS_PER_MM);
    }
    s_host.encoder_zero[ch] = current;
    return TP_OK;
}

/* =========================================================================
 * ADC
 * ========================================================================= */

tp_status_t tp_platform_adc_read(tp_adc_ch_t ch, uint16_t *counts)
{
    if (counts == NULL) {
        return TP_ERR_PARAM;
    }
    switch (ch) {
    case TP_ADC_FORCE:
        *counts = tp_plant_read_force_counts(&s_plant);
        return TP_OK;
    case TP_ADC_CURRENT_JOINT:
        *counts = tp_plant_read_current_counts(&s_plant, (int)TP_MOTOR_JOINT);
        return TP_OK;
    case TP_ADC_CURRENT_GRIPPER:
        *counts = tp_plant_read_current_counts(&s_plant, (int)TP_MOTOR_GRIPPER);
        return TP_OK;
    case TP_ADC_VBUS:
        *counts = tp_plant_read_vbus_counts(&s_plant);
        return TP_OK;
    default:
        return TP_ERR_PARAM;
    }
}

uint32_t tp_platform_adc_read_block(tp_adc_ch_t ch, uint16_t *buffer,
                                    uint32_t count)
{
    uint32_t i;

    if ((buffer == NULL) || (count == 0u)) {
        return 0u;
    }
    /* Model the circular-DMA half-buffer: each call returns fresh conversions
     * that are correlated sample-to-sample, because the plant does not move
     * meaningfully inside a 2 ms block at these speeds.  That correlation is
     * realistic and it is what makes the moving average behave as it would on
     * hardware rather than averaging pure white noise. */
    for (i = 0u; i < count; ++i) {
        uint16_t value = 0u;
        if (tp_platform_adc_read(ch, &value) != TP_OK) {
            return i;
        }
        buffer[i] = value;
    }
    return count;
}

/* =========================================================================
 * Buses
 * ========================================================================= */

tp_status_t tp_platform_bus_write(tp_bus_t bus, const uint8_t *data,
                                  uint32_t len)
{
    if ((data == NULL) && (len > 0u)) {
        return TP_ERR_PARAM;
    }
    if (((int)bus < 0) || (bus >= TP_BUS_COUNT)) {
        return TP_ERR_PARAM;
    }
    return tp_ring_buffer_write(&s_bus[bus].tx, data, len);
}

uint32_t tp_platform_bus_read(tp_bus_t bus, uint8_t *data, uint32_t max_len)
{
    if (((int)bus < 0) || (bus >= TP_BUS_COUNT)) {
        return 0u;
    }
    return tp_ring_buffer_read(&s_bus[bus].rx, data, max_len);
}

int tp_platform_bus_tx_done(tp_bus_t bus)
{
    if (((int)bus < 0) || (bus >= TP_BUS_COUNT)) {
        return 0;
    }
    /* Simulation transmits instantly: the bytes are already in the capture
     * ring, so the DMA transfer is by definition complete. */
    return 1;
}

tp_status_t tp_platform_can_send(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    uint8_t header[5];

    if (dlc > 8u) {
        return TP_ERR_PARAM;
    }
    if ((data == NULL) && (dlc > 0u)) {
        return TP_ERR_PARAM;
    }
    if (id > 0x7FFu) {
        /* Only 11-bit identifiers are supported by the design; rejecting here
         * rather than silently masking keeps a wiring bug visible. */
        return TP_ERR_RANGE;
    }
    /* The capture ring stores a de-framed record so a test can assert on the
     * identifier as well as the payload: [id_lo, id_hi, dlc, data...]. */
    header[0] = (uint8_t)(id & 0xFFu);
    header[1] = (uint8_t)((id >> 8) & 0xFFu);
    header[2] = dlc;
    if (tp_ring_buffer_write(&s_bus[TP_BUS_CAN].tx, header, 3u) != TP_OK) {
        return TP_ERR_OVERFLOW;
    }
    if (dlc > 0u) {
        return tp_ring_buffer_write(&s_bus[TP_BUS_CAN].tx, data, dlc);
    }
    return TP_OK;
}

tp_status_t tp_platform_can_recv(uint32_t *id, uint8_t *data, uint8_t *dlc)
{
    uint8_t header[3];
    tp_ring_buffer_t *rx = &s_bus[TP_BUS_CAN].rx;

    if ((id == NULL) || (data == NULL) || (dlc == NULL)) {
        return TP_ERR_PARAM;
    }
    if (tp_ring_buffer_count(rx) < 3u) {
        return TP_ERR_NOT_READY;
    }
    if (tp_ring_buffer_read(rx, header, 3u) != 3u) {
        return TP_ERR_NOT_READY;
    }
    *dlc = header[2];
    *id = (uint32_t)header[0] | ((uint32_t)header[1] << 8);
    if (*dlc > 8u) {
        *dlc = 8u;
    }
    if (*dlc > 0u) {
        (void)tp_ring_buffer_read(rx, data, *dlc);
    }
    return TP_OK;
}

tp_status_t tp_host_platform_inject_rx(tp_bus_t bus, const uint8_t *data,
                                       uint32_t len)
{
    tp_status_t st;

    if ((data == NULL) && (len > 0u)) {
        return TP_ERR_PARAM;
    }
    if (((int)bus < 0) || (bus >= TP_BUS_COUNT)) {
        return TP_ERR_PARAM;
    }
    st = tp_ring_buffer_write(&s_bus[bus].rx, data, len);
    if (st != TP_OK) {
        s_bus[bus].dropped += len;
    }
    return st;
}

uint32_t tp_host_platform_take_tx(tp_bus_t bus, uint8_t *out, uint32_t max_len)
{
    if (((int)bus < 0) || (bus >= TP_BUS_COUNT)) {
        return 0u;
    }
    return tp_ring_buffer_read(&s_bus[bus].tx, out, max_len);
}

uint32_t tp_host_platform_peek_tx(tp_bus_t bus, uint8_t *out, uint32_t max_len)
{
    uint32_t i;
    uint32_t n;

    if (((int)bus < 0) || (bus >= TP_BUS_COUNT) || (out == NULL)) {
        return 0u;
    }
    n = tp_ring_buffer_count(&s_bus[bus].tx);
    if (n > max_len) {
        n = max_len;
    }
    for (i = 0u; i < n; ++i) {
        uint8_t b = 0u;
        if (tp_ring_buffer_peek(&s_bus[bus].tx, i, &b) != TP_OK) {
            break;
        }
        out[i] = b;
    }
    return i;
}

uint32_t tp_host_platform_rx_dropped(tp_bus_t bus)
{
    if (((int)bus < 0) || (bus >= TP_BUS_COUNT)) {
        return 0u;
    }
    return s_bus[bus].dropped;
}

void tp_host_platform_dump_tx(tp_bus_t bus, const char *label)
{
    static const char hex[] = "0123456789ABCDEF";
    uint8_t  chunk[64];
    uint32_t n;
    uint32_t i;
    char     line[200];
    int      pos;

    if (((int)bus < 0) || (bus >= TP_BUS_COUNT)) {
        return;
    }
    n = tp_host_platform_peek_tx(bus, chunk, sizeof(chunk));
    if (n == 0u) {
        return;
    }
    pos = tp_snprintf(line, sizeof(line), "%s (%lu bytes): ",
                      (label != NULL) ? label : "tx", (unsigned long)n);
    for (i = 0u; (i < n) && (pos < (int)sizeof(line) - 4); ++i) {
        line[pos++] = hex[(chunk[i] >> 4) & 0x0Fu];
        line[pos++] = hex[chunk[i] & 0x0Fu];
        line[pos++] = ' ';
    }
    line[pos] = '\0';
    LOG_DEBUG(TP_MOD_DEBUG, "%s", line);
}

/* =========================================================================
 * Watchdog
 * ========================================================================= */

tp_status_t tp_platform_watchdog_init(uint32_t timeout_ms)
{
    if (timeout_ms == 0u) {
        return TP_ERR_PARAM;
    }
    s_host.wdg_timeout_ms = timeout_ms;
    s_host.wdg_last_feed_ms = s_host.time_ms;
    s_host.wdg_enabled = 1;
    LOG_INFO(TP_MOD_SAFETY, "watchdog armed, timeout %lu ms",
             (unsigned long)timeout_ms);
    return TP_OK;
}

void tp_platform_watchdog_feed(void)
{
    s_host.wdg_last_feed_ms = s_host.time_ms;
}

int tp_platform_watchdog_caused_reset(void)
{
    int caused = s_host.wdg_caused_reset;

    s_host.wdg_caused_reset = 0; /* report once, as the hardware flag does */
    return caused;
}

/* =========================================================================
 * Critical sections
 * ========================================================================= */

uint32_t tp_platform_enter_critical(void)
{
    uint32_t previous = s_host.isr_state;

    /* The host build is single-threaded by default (the baremetal OSAL
     * backend), so the only thing a critical section has to do is be
     * re-entrant and observable: a test that leaves a section open is a bug,
     * and the depth counter makes it detectable instead of silent. */
    s_host.critical_depth++;
    s_host.isr_state = 1u;
    return previous;
}

void tp_platform_exit_critical(uint32_t key)
{
    if (s_host.critical_depth > 0u) {
        s_host.critical_depth--;
    }
    s_host.isr_state = key & 1u;
}

/* =========================================================================
 * Safety output
 * ========================================================================= */

void tp_platform_emergency_kill(void)
{
    int i;

    if (s_host.killed == 0) {
        LOG_ERROR(TP_MOD_SAFETY, "EMERGENCY KILL latched, all drives disabled");
    }
    s_host.killed = 1;
    for (i = 0; i < (int)TP_MOTOR_COUNT; ++i) {
        s_host.pwm[i] = 0.0f;
        s_host.enabled[i] = 0;
    }
}

void tp_platform_emergency_release(void)
{
    int i;

    if (s_host.killed != 0) {
        LOG_WARN(TP_MOD_SAFETY, "emergency kill released by RESET");
    }
    s_host.killed = 0;
    for (i = 0; i < (int)TP_MOTOR_COUNT; ++i) {
        s_host.enabled[i] = 1;
    }
}

int tp_platform_is_killed(void)
{
    return s_host.killed;
}
