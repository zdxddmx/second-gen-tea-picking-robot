/**
 * @file    stm32f4xx_it.c
 * @brief   Exception and interrupt handlers for the STM32F407VET6 target.
 *
 * @details
 * This file is deliberately thin.  Every core exception is real (it either
 * reports and resets, or forwards to the RTOS port), and every peripheral
 * interrupt is a one-line forward to the platform BSP, which is the only place
 * that knows about ring buffers and DMA flags:
 *
 * | Vector                    | Forwards to                          |
 * |---------------------------|--------------------------------------|
 * | `USART1/2/3_IRQHandler`   | `tp_stm32_uart_irq_handler(idx)`     |
 * | `DMA1_Stream1/3/5/6`       | `tp_stm32_dma_irq_handler(1, n)`     |
 * | `DMA2_Stream0/5/7`         | `tp_stm32_dma_irq_handler(2, n)`     |
 * | `CAN1_RX0_IRQHandler`      | `tp_stm32_can_irq_handler(RX0)`      |
 * | `CAN1_SCE_IRQHandler`      | `tp_stm32_can_irq_handler(SCE)`      |
 * | `TIM7_IRQHandler`          | `tp_stm32_tim7_irq_handler()`        |
 *
 * Everything except the platform forwards is wrapped in
 * `#if defined(TP_PLATFORM_STM32F407)`, so the whole file compiles to an empty
 * translation unit for the host build.  That matters because the host Makefile
 * globs every C file under `firmware/Core`: without the guard the host build
 * would try to include `stm32f4xx.h` and fail.
 *
 * The FreeRTOS-dependent handlers (`SVC_Handler`, `PendSV_Handler`,
 * `SysTick_Handler`) are conditional on `TP_OSAL_FREERTOS`, which the build
 * defines only for `make stm32 RTOS=freertos`.  In the default bare-metal build
 * they fall through to the HAL tick, so the same source links in both
 * configurations.
 *
 * @copyright MIT License, see LICENSE.
 */

#include "stm32f4xx_it.h"

#if defined(TP_PLATFORM_STM32F407)

#include "stm32f4xx_hal.h"

#include "config.h"
#include "logger.h"
#include "platform_stm32f407.h"

#if defined(TP_OSAL_FREERTOS)
#include "FreeRTOS.h"
#include "task.h"

/* The port's own entry points.  They are provided by
 * `portable/GCC/ARM_CM4F/port.c`, which is not present on this machine; see
 * `FREERTOS_PORT_DIR` in the root Makefile. */
extern void xPortSysTickHandler(void);
extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
#endif /* TP_OSAL_FREERTOS */

/* =========================================================================
 * Cortex-M4 core exceptions
 * ========================================================================= */

/**
 * @brief Common tail for every fault: say what happened, then reset.
 *
 * Marked `noreturn`; it resets the part.  The delay before the reset exists
 * because `tp_stm32_debug_write_blocking()` only guarantees the byte reached the
 * transmit *shift register*, not that a 115200 baud line has finished shifting
 * the last one out — a reset 20 microseconds later would truncate the report.
 *
 * A robot must not continue after a fault.  The alternative to a reset is
 * "keep running with a corrupt stack and a latched fault flag", which is how a
 * fault handler turns a single bad pointer dereference into a runaway axis.
 */
void tp_stm32_fault_entry(uint32_t *frame)
{
    uint32_t exception = __get_IPSR() & 0x1FFu;

    tp_stm32_fault_report(exception, frame);

    /* ~10 ms of NOPs at 168 MHz.  Deliberately not a timer-based delay: whatever
     * broke the fault may also have broken the timer. */
    {
        volatile uint32_t spin;
        for (spin = 0u; spin < 400000u; ++spin) {
            __asm volatile("nop");
        }
    }

    __disable_irq();
    NVIC_SystemReset();

    for (;;) {
        /* NVIC_SystemReset() does not return; this loop only satisfies the
         * compiler's control-flow analysis. */
    }
}

/*
 * The stacked frame is at MSP or PSP depending on EXC_RETURN bit 2.  This must
 * be read *before* the compiler emits a prologue, hence the naked functions: in
 * a normal C function the compiler may have already pushed registers and used
 * LR for its own purposes, and the frame pointer it computed would be wrong.
 */
#define TP_FAULT_HANDLER_BODY(handler)                                        \
    __attribute__((naked, noreturn)) void handler(void)                       \
    {                                                                         \
        __asm volatile(                                                       \
            "tst   lr, #4          \n" /* EXC_RETURN bit 2: 0 = MSP, 1 = PSP */\
            "ite   eq              \n"                                        \
            "mrseq r0, msp         \n"                                        \
            "mrsne r0, psp         \n"                                        \
            "b     tp_stm32_fault_entry \n");                                 \
    }

TP_FAULT_HANDLER_BODY(HardFault_Handler)
TP_FAULT_HANDLER_BODY(MemManage_Handler)
TP_FAULT_HANDLER_BODY(BusFault_Handler)
TP_FAULT_HANDLER_BODY(UsageFault_Handler)

#undef TP_FAULT_HANDLER_BODY

/**
 * @brief Non-maskable interrupt.
 *
 * The NMI cannot be caused by any peripheral on this board (the clock security
 * system is the only source and HSE failure is detected by polling instead), so
 * reaching here means something is very wrong.  Report and reset rather than
 * spin, so the reset cause is recorded in `RCC->CSR` and the boot log can say so.
 */
void NMI_Handler(void)
{
    tp_stm32_debug_puts_blocking("\r\n*** FAULT: NMI ***\r\n");
    __disable_irq();
    NVIC_SystemReset();
    for (;;) {
    }
}

void SVC_Handler(void)
{
#if defined(TP_OSAL_FREERTOS)
    /* The RTOS uses SVCall to start the first task. */
    vPortSVCHandler();
#else
    /* Bare metal: nothing issues an SVC, so reaching here is a bug.  Report it
     * instead of silently returning into an undefined state. */
    tp_stm32_debug_puts_blocking("\r\n*** unexpected SVC ***\r\n");
#endif
}

void DebugMon_Handler(void)
{
    /* No debug monitor is configured; a debugger that wants one installs its own
     * handler.  Present so the vector is not left pointing at Default_Handler. */
}

void PendSV_Handler(void)
{
#if defined(TP_OSAL_FREERTOS)
    xPortPendSVHandler();
#endif
}

void SysTick_Handler(void)
{
#if defined(TP_OSAL_FREERTOS)
    /* Before the scheduler starts, SysTick belongs to the HAL so that
     * HAL_GetTick() works during bring-up; afterwards the RTOS owns it and
     * HAL_IncTick() is driven from vApplicationTickHook(). */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    } else {
        HAL_IncTick();
    }
#else
    /* Bare metal: HAL_Init() programmed SysTick for a 1 ms tick at
     * TICK_INT_PRIORITY, and this is what advances it. */
    HAL_IncTick();
#endif
}

/* =========================================================================
 * USART interrupts
 *
 * Each handler snapshots SR *once* in the platform (reading SR has side effects
 * on the error flags), which is why the work is not spread across three cases
 * here.
 * ========================================================================= */

void USART1_IRQHandler(void)
{
    tp_stm32_uart_irq_handler(TP_STM32_UART_DEBUG);
}

void USART2_IRQHandler(void)
{
    tp_stm32_uart_irq_handler(TP_STM32_UART_VISION);
}

void USART3_IRQHandler(void)
{
    tp_stm32_uart_irq_handler(TP_STM32_UART_RS485);
}

/* =========================================================================
 * DMA interrupts
 *
 * Mapping (RM0090 DMA request tables), which the platform BSP also documents:
 *   DMA1 Stream1 = USART3_RX   DMA1 Stream3 = USART3_TX
 *   DMA1 Stream5 = USART2_RX   DMA1 Stream6 = USART2_TX
 *   DMA2 Stream0 = ADC1        DMA2 Stream5 = USART1_RX
 *   DMA2 Stream7 = USART1_TX
 * ========================================================================= */

void DMA1_Stream1_IRQHandler(void)
{
    tp_stm32_dma_irq_handler(1, 1);
}

void DMA1_Stream3_IRQHandler(void)
{
    tp_stm32_dma_irq_handler(1, 3);
}

void DMA1_Stream5_IRQHandler(void)
{
    tp_stm32_dma_irq_handler(1, 5);
}

void DMA1_Stream6_IRQHandler(void)
{
    tp_stm32_dma_irq_handler(1, 6);
}

void DMA2_Stream0_IRQHandler(void)
{
    tp_stm32_dma_irq_handler(2, 0);
}

void DMA2_Stream5_IRQHandler(void)
{
    tp_stm32_dma_irq_handler(2, 5);
}

void DMA2_Stream7_IRQHandler(void)
{
    tp_stm32_dma_irq_handler(2, 7);
}

/* =========================================================================
 * CAN and TIM7
 * ========================================================================= */

void CAN1_RX0_IRQHandler(void)
{
    tp_stm32_can_irq_handler(TP_STM32_CAN_EVENT_RX0);
}

void CAN1_SCE_IRQHandler(void)
{
    tp_stm32_can_irq_handler(TP_STM32_CAN_EVENT_SCE);
}

void TIM7_IRQHandler(void)
{
    tp_stm32_tim7_irq_handler();
}

#endif /* TP_PLATFORM_STM32F407 */
