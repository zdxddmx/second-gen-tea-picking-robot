/**
 * @file    stm32f4xx_it.h
 * @brief   Prototypes for the exception and interrupt handlers.
 *
 * @details
 * These are the names the vector table in
 * `firmware/Drivers/startup/startup_stm32f407xx.s` points at.  They must match
 * the CMSIS `*_IRQHandler` names exactly: a typo does not cause a warning, it
 * causes the vector to fall through to `Default_Handler` and the peripheral to
 * appear dead with no other symptom.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_STM32F4XX_IT_H
#define TP_STM32F4XX_IT_H

#include <stdint.h>

#include "compiler.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Cortex-M4 core exceptions
 * ------------------------------------------------------------------------- */

/** @brief Non-maskable interrupt. */
void NMI_Handler(void);

/**
 * @brief Hard fault.
 *
 * Dumps the stacked exception frame and the fault status registers through the
 * debug UART, then resets the part.  A robot that keeps executing after a hard
 * fault is a robot whose state machine is reading garbage.
 */
void HardFault_Handler(void);

/** @brief MPU memory-management fault. */
void MemManage_Handler(void);

/** @brief Bus fault (precise or imprecise). */
void BusFault_Handler(void);

/** @brief Usage fault (undefined instruction, unaligned access, divide by zero). */
void UsageFault_Handler(void);

/** @brief Supervisor call.  Routed to the RTOS port when one is selected. */
void SVC_Handler(void);

/** @brief Debug monitor. */
void DebugMon_Handler(void);

/** @brief Pendable service request.  Routed to the RTOS port when one is selected. */
void PendSV_Handler(void);

/** @brief System tick.  Feeds the HAL tick, or the RTOS tick when one is selected. */
void SysTick_Handler(void);

/* -------------------------------------------------------------------------
 * Peripheral interrupts this product uses
 * ------------------------------------------------------------------------- */

void USART1_IRQHandler(void); /**< Debug console, PA9/PA10.       */
void USART2_IRQHandler(void); /**< Vision / host link, PD5/PD6.   */
void USART3_IRQHandler(void); /**< RS485, PB10/PB11.              */

void DMA1_Stream1_IRQHandler(void); /**< USART3 RX.             */
void DMA1_Stream3_IRQHandler(void); /**< USART3 TX.             */
void DMA1_Stream5_IRQHandler(void); /**< USART2 RX.             */
void DMA1_Stream6_IRQHandler(void); /**< USART2 TX.             */
void DMA2_Stream0_IRQHandler(void); /**< ADC1 scan.             */
void DMA2_Stream5_IRQHandler(void); /**< USART1 RX.             */
void DMA2_Stream7_IRQHandler(void); /**< USART1 TX.             */

void CAN1_RX0_IRQHandler(void); /**< CAN1 receive FIFO 0.           */
void CAN1_SCE_IRQHandler(void); /**< CAN1 status change / errors.   */

void TIM7_IRQHandler(void); /**< RS485 DE release one-shot.         */

/**
 * @brief Common fault tail: report the frame, then reset.
 *
 * Called from the naked fault handlers with R0 pointing at the stacked exception
 * frame.  Declared here rather than kept static because the hand-written
 * assembly that fills R0 refers to it by name.
 *
 * @param[in] frame  Eight stacked words: R0, R1, R2, R3, R12, LR, PC, xPSR.
 *
 * @note Never returns: it resets the microcontroller.
 */
void tp_stm32_fault_entry(uint32_t *frame) TP_NORETURN;

#ifdef __cplusplus
}
#endif

#endif /* TP_STM32F4XX_IT_H */
