/**
 * @file    FreeRTOSConfig.h
 * @brief   FreeRTOS configuration for the STM32F407VET6 target.
 *
 * @details
 * Compiled only by `make stm32 RTOS=freertos`, which currently refuses to run on
 * this machine because the GCC Cortex-M4F port is not vendored (see
 * `FREERTOS_PORT_DIR` in the root Makefile and
 * `firmware/Drivers/third_party/README.md`).  It is written now because the
 * values below are a *design decision*, not a build detail: they encode how much
 * real-time margin the robot has, and getting them wrong is how a 1 kHz control
 * loop starts missing deadlines.
 *
 * # The numbers, and where they come from
 *
 * | Setting | Value | Why |
 * |---|---|---|
 * | `configCPU_CLOCK_HZ` | `SystemCoreClock` | Tracks the 168 MHz clock instead of duplicating 168000000 |
 * | `configTICK_RATE_HZ` | 1000 | Matches `TP_CONTROL_FREQ_HZ`; every other rate in `config.h` divides it |
 * | `configMAX_PRIORITIES` | 8 | The OSAL exposes `OSAL_PRIO_IDLE..OSAL_PRIO_CRITICAL` = 0..7, and `osal_freertos.c` static-asserts this |
 * | `configTOTAL_HEAP_SIZE` | 32 KiB | Roughly 128 bytes of TCB plus the task stacks below, with headroom for queues |
 * | `configMAX_TASK_NAME_LEN` | 8 | The logger's module tags are "8 characters or fewer" by contract, so a truncated name is still a full name |
 * | `configCHECK_FOR_STACK_OVERFLOW` | 2 | Method 2 also checks the last 20 bytes, which catches a near-miss rather than only a catastrophic one |
 * | `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` | 5 | See below |
 *
 * # Priority grouping, and why it is the single most dangerous setting here
 *
 * Cortex-M interrupts have *numerically lower* values at *higher* priority, and
 * FreeRTOS uses the opposite convention for tasks.  An ISR that calls any
 * `...FromISR()` API at a priority numerically below
 * `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` corrupts the kernel's critical
 * sections **silently** - no fault, no assert (unless `configASSERT` is enabled
 * and the port's `vPortValidateInterruptPriority` runs), just a robot that
 * occasionally drops a control period.
 *
 * The interrupt priorities this port actually assigns (see
 * `tp_platform_init()` in `firmware/BSP/stm32f407/platform_stm32f407.c`) are:
 *
 *      2  ADC1 DMA (DMA2 Stream0)      - publishes the samples the 1 kHz loop reads
 *      3  USART1/2/3 and their DMA     - byte transport
 *      4  CAN1 RX0 / status change     - frame transport
 *      5  TIM7                         - RS485 driver-enable release
 *
 * **None of those handlers calls a FreeRTOS API**, which is what makes the low
 * numeric values legal: `configMAX_SYSCALL_INTERRUPT_PRIORITY` restricts only
 * interrupt service routines that call into the kernel.  The kernel's own
 * PendSV and SysTick are left at the lowest priority (`configKERNEL_INTERRUPT_PRIORITY`,
 * 15) by `xPortStartScheduler()`.
 *
 * The HAL tick is at `TICK_INT_PRIORITY` = 15 from `stm32f4xx_hal_conf.h`, which
 * is at or below `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`, so that one *is*
 * allowed to call kernel APIs - and does, through `vApplicationTickHook`.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

#include "config.h"

/* `SystemCoreClock` is maintained by `system_stm32f4xx.c` and by every
 * `HAL_RCC_ClockConfig()` call.  Reading it here rather than hard-coding
 * 168000000 means a board whose crystal failed to start cannot configure a tick
 * that is 10.5x too fast for the clock it is actually running on. */
extern uint32_t SystemCoreClock;

/* ==========================================================================
 * Scheduler
 * ========================================================================== */

#define configUSE_PREEMPTION                     1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1 /* Cortex-M4 has CLZ; use it. */
#define configUSE_TICKLESS_IDLE                  0 /* Timers must not stop.      */
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )
#define configTICK_RATE_HZ                       ( (TickType_t) 1000 )
#define configMAX_PRIORITIES                     ( 8 )
#define configMINIMAL_STACK_SIZE                 ( (unsigned short) 128 )
#define configTOTAL_HEAP_SIZE                    ( (size_t) ( 32 * 1024 ) )
#define configMAX_TASK_NAME_LEN                  ( 8 )
#define configUSE_16_BIT_TICKS                   0
#define configIDLE_SHOULD_YIELD                  1
#define configUSE_MUTEXES                        1
#define configQUEUE_REGISTRY_SIZE                8
#define configCHECK_FOR_STACK_OVERFLOW           2
#define configUSE_RECURSIVE_MUTEXES              1
#define configUSE_MALLOC_FAILED_HOOK             1
#define configUSE_APPLICATION_TASK_TAG           0
#define configUSE_COUNTING_SEMAPHORES            1
#define configGENERATE_RUN_TIME_STATS            0

/* Application-defined hooks.  All four are implemented in
 * `firmware/Core/freertos_hooks.c`; see that file for what each one does and, in
 * particular, for why `vApplicationIdleHook` does **not** feed the watchdog. */
#define configUSE_IDLE_HOOK                      1
#define configUSE_TICK_HOOK                      1

/* ==========================================================================
 * Memory allocation
 *
 * `heap_4.c` (first-fit with coalescing) is selected by the Makefile.  Static
 * allocation is enabled as well so that a task or queue with a hard real-time
 * requirement can be created without touching the heap at all - on a robot,
 * "the control task failed to start because the heap was fragmented" is not an
 * acceptable failure mode, and the option to avoid it costs nothing.
 * ========================================================================== */

#define configSUPPORT_STATIC_ALLOCATION          1
#define configSUPPORT_DYNAMIC_ALLOCATION         1

/* ==========================================================================
 * Synchronisation primitives
 * ========================================================================== */

#define configUSE_TASK_NOTIFICATIONS             1
#define configUSE_TRACE_FACILITY                 1
#define configUSE_STATS_FORMATTING_FUNCTIONS     1

/* ==========================================================================
 * Software timers
 * ========================================================================== */

#define configUSE_TIMERS                         1
#define configTIMER_TASK_PRIORITY                ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                 10
#define configTIMER_TASK_STACK_DEPTH             ( configMINIMAL_STACK_SIZE * 2 )

/* ==========================================================================
 * Optional API inclusion
 * ========================================================================== */

#define INCLUDE_vTaskPrioritySet                 1
#define INCLUDE_uxTaskPriorityGet                1
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_vTaskDelayUntil                  1
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_xTaskGetCurrentTaskHandle        1
#define INCLUDE_uxTaskGetStackHighWaterMark      1
#define INCLUDE_xTaskGetIdleTaskHandle           1
#define INCLUDE_eTaskGetState                    1
#define INCLUDE_xTimerPendFunctionCall           1
#define INCLUDE_xTaskAbortDelay                  1
#define INCLUDE_xTaskGetHandle                   1

/* ==========================================================================
 * Cortex-M interrupt priorities - see the file header before changing these
 * ========================================================================== */

#define configPRIO_BITS                          4 /* STM32F4 implements 4 bits. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/** @brief Kernel interrupt priority, in the port's shifted form. */
#define configKERNEL_INTERRUPT_PRIORITY \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/** @brief Highest priority an ISR may have and still call a `...FromISR()` API. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* ==========================================================================
 * Assertions
 *
 * `configASSERT` is active in both debug and release builds.  A firmware whose
 * safety argument rests on "the fault monitor always runs" cannot ship a build
 * in which the kernel's own preconditions are unchecked: a `configASSERT` that
 * fires is a design bug, and finding it by having the robot stop dead is far
 * better than finding it by having the robot move.
 *
 * The trap parks with interrupts disabled so a debugger can inspect the state.
 * It deliberately does not log: it can fire inside the scheduler with the heap
 * and the logger's rate limiter both in an unknown state.
 * ========================================================================== */

#define configASSERT( x )                                                     \
    if ( ( x ) == 0 ) {                                                       \
        taskDISABLE_INTERRUPTS();                                             \
        for ( ;; ) {                                                          \
        }                                                                     \
    }

/* ==========================================================================
 * Cortex-M port interrupt handler mapping
 *
 * DELIBERATELY NOT DEFINED.
 *
 * FreeRTOS's port normally installs its handlers by remapping names in this
 * header:
 *
 *     #define vPortSVCHandler     SVC_Handler
 *     #define xPortPendSVHandler  PendSV_Handler
 *     #define xPortSysTickHandler SysTick_Handler
 *
 * This project does not do that.  `firmware/Core/stm32f4xx_it.c` defines
 * `SVC_Handler`, `PendSV_Handler` and `SysTick_Handler` itself and forwards to
 * `vPortSVCHandler()`, `xPortPendSVHandler()` and `xPortSysTickHandler()`.  That
 * keeps every vector the product uses in one auditable file, and it is what lets
 * `SysTick_Handler` distinguish "the scheduler has not started yet, keep feeding
 * the HAL tick" from "the RTOS owns this now" - a distinction that is impossible
 * if the port has taken the vector over by macro.  Defining the remaps here
 * would produce duplicate definitions of all three handlers and a link error.
 * ========================================================================== */

#endif /* FREERTOS_CONFIG_H */
