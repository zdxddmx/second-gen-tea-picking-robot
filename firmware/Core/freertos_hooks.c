/**
 * @file    freertos_hooks.c
 * @brief   FreeRTOS application hooks for the STM32F407VET6 target.
 *
 * @details
 * The whole file is compiled only when the build selects the FreeRTOS OSAL
 * backend (`make stm32 RTOS=freertos`, which defines `TP_OSAL_FREERTOS`).
 * In the default bare-metal build it is an empty translation unit, which is what
 * lets the same source tree build and link in both configurations.
 *
 * # What each hook does, and what it deliberately does not do
 *
 *   - `vApplicationStackOverflowHook` — the most dangerous of the four to get
 *     wrong.  It runs from inside the context switch, with the scheduler already
 *     in an undefined state.  It reports through the *blocking* UART writer (the
 *     DMA ring and the logger's rate limiter are both unsafe here) and resets.
 *   - `vApplicationMallocFailedHook` — same treatment: a heap that cannot
 *     satisfy an allocation means construction of some task or queue failed
 *     silently, and a robot that boots with a missing task is worse than one
 *     that does not boot.
 *   - `vApplicationIdleHook` — puts the core into `__WFI()`.  It does **not**
 *     feed the watchdog: `tp_platform_watchdog_feed()` is called from exactly one
 *     place, the safety task, after every supervised task has reported in.
 *     Feeding it from the idle hook would defeat the entire mechanism, because
 *     the idle task runs precisely when everything else is blocked — including
 *     when a control task has hung.
 *   - `vApplicationTickHook` — advances the HAL's millisecond counter, which
 *     stops being driven by `SysTick_Handler` once the RTOS owns the tick.
 *     The OSAL's own tick hook is deliberately *not* called from here: on the
 *     FreeRTOS backend `osal_set_tick_hook()` documents itself as a no-op
 *     ("real time advances by itself; there is nothing to hook"), because only
 *     the bare-metal backend has a simulated clock to advance.  The millisecond
 *     clock the firmware uses is the hardware TIM5 counter either way (see the
 *     platform BSP header), so nothing here is load-bearing for timing accuracy.
 *
 * @copyright MIT License, see LICENSE.
 */

#include "compiler.h"
#include "config.h"

#if defined(TP_OSAL_FREERTOS)

#include "FreeRTOS.h"
#include "task.h"

#include "logger.h"
#include "platform_api.h"
#include "platform_stm32f407.h"
#include "status.h"

/**
 * @brief Weak OSAL tick hook, retained for the bare-metal backend's contract.
 *
 * The FreeRTOS backend of `osal_set_tick_hook()` is a documented no-op, so this
 * hook has no caller in that configuration.  It is kept weak and empty so that
 * enabling the FreeRTOS build cannot break a translation unit that expects the
 * symbol to exist.
 */
TP_WEAK void tp_osal_tick_hook(void)
{
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;

    /* Blocking writer on purpose; see the file header.  The name is truncated to
     * 8 characters because the RTOS configuration limits it, so it is printed
     * verbatim rather than through a format that might assume more. */
    tp_stm32_debug_puts_blocking("\r\nFATAL: stack overflow in task ");
    if (pcTaskName != NULL) {
        tp_stm32_debug_puts_blocking(pcTaskName);
    } else {
        tp_stm32_debug_puts_blocking("(unnamed)");
    }
    tp_stm32_debug_puts_blocking("\r\n");

    /* Stop driving before the reset: a stack overflow can have corrupted any
     * global, including the motor command. */
    tp_platform_emergency_kill();

    __disable_irq();
    NVIC_SystemReset();
    for (;;) {
    }
}

void vApplicationMallocFailedHook(void)
{
    tp_stm32_debug_puts_blocking(
        "\r\nFATAL: FreeRTOS heap exhausted (a task or queue was not created)\r\n");
    tp_platform_emergency_kill();
    __disable_irq();
    NVIC_SystemReset();
    for (;;) {
    }
}

void vApplicationIdleHook(void)
{
    /* Sleep until the next interrupt.  With a 1 kHz tick this is at most 1 ms of
     * idle per pass, and it keeps the core out of the bus during the long gaps
     * between a 500 Hz sensor task and a 20 Hz telemetry task. */
    __asm volatile("wfi");
}

void vApplicationTickHook(void)
{
    /* Keep the HAL's millisecond counter advancing.  Once the RTOS owns SysTick,
     * `SysTick_Handler` no longer calls HAL_IncTick(), and every HAL internal
     * timeout - ADC stabilisation, UART transmission complete, flash erase - uses
     * HAL_GetTick().  A frozen uwTick turns those timeouts into either an
     * immediate spurious failure or no timeout at all, depending on which way the
     * comparison happens to fall. */
    HAL_IncTick();

    /* See the file header: the OSAL's tick hook is a documented no-op on this
     * backend, so there is nothing further to do. */
}

#endif /* TP_OSAL_FREERTOS */
