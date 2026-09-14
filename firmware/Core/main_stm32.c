/**
 * @file    main_stm32.c
 * @brief   Target entry point for the STM32F407VET6.
 *
 * @details
 * The whole of `main()` is bring-up and hand-off.  There is deliberately **no
 * control logic here**: motion, PID tuning, the gripper state machine and the
 * fault policy all live in `firmware/Application/app_tasks.c`, which this file
 * reaches through the single call `tp_app_start()`.  Keeping `main()` free of
 * product logic is what makes the host build the same program rather than a
 * lookalike.
 *
 * The order is fixed by two dependencies:
 *
 *   1. `HAL_Init()` first.  It sets the NVIC priority grouping, enables the ART
 *      instruction/data caches and starts the 1 ms tick that `HAL_GetTick()`
 *      uses for its own timeouts.  The *clock* tree is already configured by
 *      `SystemInit()`, which `Reset_Handler` called before this function.
 *   2. The log sink before `tp_platform_init()`, so the platform's own
 *      "everything came up" summary is actually printed.
 *
 * @copyright MIT License, see LICENSE.
 */

#include "stm32f4xx_hal.h"

#include "app_entry.h"
#include "config.h"
#include "logger.h"
#include "platform_api.h"
#include "platform_stm32f407.h"

/* =========================================================================
 * Debug UART log sink
 * ========================================================================= */

/**
 * @brief Logger sink that writes to the debug UART through the platform seam.
 *
 * Non-blocking by construction: `tp_platform_bus_write()` queues into a DMA ring
 * and returns.  The logger's contract requires exactly that — a sink that can
 * block would let a burst of warnings stall whichever task emitted them, and the
 * trace would then hide the fault it was describing.
 *
 * Lines emitted before `tp_platform_init()` reaches the UART are refused by the
 * seam (`TP_ERR_NOT_READY`) and dropped, which is why the sink is installed
 * before that call rather than after it: the platform's own bring-up summary is
 * the first thing that must appear.
 *
 * @param[in] line  Formatted line, already terminated with CRLF by the logger.
 * @param[in] len   Length excluding the NUL.
 * @param[in] user  Unused.
 */
static void stm32_log_sink(const char *line, size_t len, void *user)
{
    (void)user;

    if ((line == NULL) || (len == 0u)) {
        return;
    }
    (void)tp_platform_bus_write(TP_BUS_DEBUG_UART, (const uint8_t *)line,
                                (uint32_t)len);
}

/* =========================================================================
 * main
 * ========================================================================= */

/**
 * @brief Program entry point.
 *
 * @return Never returns, in practice.  The trailing loop exists only so the
 *         compiler does not have to reason about a function that does not
 *         return.
 */
int main(void)
{
    tp_status_t st;

#if defined(TP_OSAL_FREERTOS)
    /* Tell the HAL that an RTOS owns the tick, so HAL_Delay() and the HAL's
     * internal timeouts cooperate with the scheduler instead of fighting it. */
    HAL_InitTick(TICK_INT_PRIORITY);
#endif

    if (HAL_Init() != HAL_OK) {
        /* HAL_Init() only fails if the tick cannot be configured, which on a
         * Cortex-M4 with a working SysTick means the clock tree was already
         * broken by SystemInit().  Say so on the blocking path, because the DMA
         * ring the log sink uses does not exist yet. */
        tp_stm32_debug_puts_blocking(
            "\r\nFATAL: HAL_Init() failed, refusing to start\r\n");
        for (;;) {
        }
    }

    /* Logger: route to the debug UART, timestamp from the platform's millisecond
     * counter, and honour the compile-time level filter. */
    tp_log_set_sink(stm32_log_sink, NULL);
    tp_log_set_time_source(tp_platform_time_ms);
#if (TP_LOG_LEVEL <= TP_LOG_LEVEL_ERROR)
    tp_log_set_level((tp_log_level_t)TP_LOG_LEVEL);
#else
    tp_log_set_level(TP_LOG_ERROR);
#endif

    LOG_INFO(TP_MOD_MAIN, "tea-picking robot fw %d.%d.%d booting on STM32F407VET6",
             TP_FW_VERSION_MAJOR, TP_FW_VERSION_MINOR, TP_FW_VERSION_PATCH);

    /* The clock tree is configured by SystemInit() before main() runs.  Verify
     * it rather than assume it: every baud rate, PWM frequency and PID gain in
     * the firmware is scaled for 168 MHz, and a board whose 8 MHz crystal failed
     * to start would otherwise run the 1 kHz control loop 10.5x too slowly with
     * no other symptom. */
    if (tp_stm32_clock_is_168mhz() == 0) {
        LOG_ERROR(TP_MOD_MAIN,
                  "SYSCLK is %lu Hz, expected %lu Hz: HSE crystal did not start",
                  (unsigned long)SystemCoreClock,
                  (unsigned long)tp_stm32_clock_expected_hz());
        tp_stm32_debug_puts_blocking(
            "\r\nFATAL: HSE/PLL did not reach 168 MHz\r\n");
        for (;;) {
            /* Deliberately does not proceed: see the comment above. */
        }
    }

    /* Report a watchdog reset before anything else can clear the flag: the boot
     * self-check uses it to refuse an automatic motion resume. */
    if (tp_platform_watchdog_caused_reset() != 0) {
        LOG_ERROR(TP_MOD_SAFETY,
                  "previous reset was caused by the watchdog (RCC_CSR=0x%08lX)",
                  (unsigned long)tp_stm32_reset_flags());
    }

    st = tp_platform_init();
    if (st != TP_OK) {
        LOG_ERROR(TP_MOD_MAIN, "platform_init failed: %s (%d)",
                  tp_status_str(st), (int)st);
        /* The debug UART may not be up (the failure can be earlier than that),
         * so use the polling writer as well; if USART1 was configured it will
         * appear, and if it was not, nothing can be said anyway. */
        tp_stm32_debug_puts_blocking("\r\nFATAL: platform_init failed\r\n");
        for (;;) {
            /* Stay put.  Resetting would loop through the same failure; parking
             * leaves the part alive for a debugger to inspect. */
        }
    }

    LOG_INFO(TP_MOD_MAIN, "platform ready, handing over to the application");

    /* The application.  Never returns. */
    tp_app_start();

    /* --- Unreachable in a correct application ---------------------------
     * If the application entry point returns, the drives are killed and the
     * part is reset, because every guarantee the firmware makes (watchdog
     * supervision, the E-stop latch, the safe PWM state) lives inside a loop
     * that has just stopped running. */
    LOG_ERROR(TP_MOD_MAIN, "tp_app_start() returned; killing drives and resetting");
    tp_platform_emergency_kill();
    {
        volatile uint32_t spin;
        for (spin = 0u; spin < 400000u; ++spin) {
            __asm volatile("nop");
        }
    }
    NVIC_SystemReset();

    for (;;) {
    }
}
