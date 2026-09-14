/**
 * @file    platform_stm32f407.h
 * @brief   Target-only extensions to the platform seam (STM32F407VET6 silicon).
 *
 * @details
 * `platform_api.h` is the *product* interface and is byte-identical on both
 * targets.  This header adds the things that only make sense when the hardware is
 * real: the interrupt entry points that `firmware/Core/stm32f4xx_it.c` forwards
 * to, the register-level diagnostics that a bench session needs, and the
 * arithmetic results (IWDG prescaler, CAN bit timing, achieved baud rate) that
 * the boot log reports and that a reviewer should be able to check against the
 * reference manual without re-deriving them.
 *
 * Nothing in `firmware/Core` (other than `stm32f4xx_it.c` and `main_stm32.c`),
 * `firmware/Control`, `firmware/Application`, `firmware/Protocol` or
 * `firmware/Safety` may include this header.  That rule is what guarantees the
 * host build is the *same program* and not a lookalike: a grep for
 * `platform_stm32f407.h` outside `BSP/stm32f407` is a valid CI check, mirroring
 * the identical rule for `platform_host.h`.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_PLATFORM_STM32F407_H
#define TP_PLATFORM_STM32F407_H

#include <stdint.h>

#include "platform_api.h"
#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * Interrupt entry points
 *
 * These are the only functions in this header that move bytes.  They exist so
 * that `stm32f4xx_it.c` contains nothing but a one-line forward per vector:
 * the vector table is a hardware contract that is easy to get wrong, and the
 * logic belongs next to the ring buffers it touches.
 * ========================================================================= */

/** @brief Index of each USART in the platform's internal tables. */
#define TP_STM32_UART_DEBUG  0 /**< USART1, PA9/PA10, 115200, logs.        */
#define TP_STM32_UART_VISION 1 /**< USART2, PD5/PD6, 921600, host frames.  */
#define TP_STM32_UART_RS485  2 /**< USART3, PB10/PB11 + PD12 DE, Modbus.   */
#define TP_STM32_UART_COUNT  3

/**
 * @brief Service one USART's status register.
 *
 * Handles the IDLE-line event (end of a DMA receive burst), the RXNE path (a
 * safety net for when the RX DMA is not running yet) and the error flags
 * (overrun, framing, noise, parity).  Safe to call from an ISR only.
 *
 * @param[in] idx  One of @ref TP_STM32_UART_DEBUG, @ref TP_STM32_UART_VISION,
 *                 @ref TP_STM32_UART_RS485.  Out-of-range values are ignored.
 */
void tp_stm32_uart_irq_handler(int idx);

/**
 * @brief Harvest the bytes the RX DMA has copied since the last call.
 *
 * On the STM32F4 a UART has no "receive complete" interrupt for a variable
 * length frame; the idiomatic solution is a circular DMA into a scratch buffer
 * plus the IDLE-line interrupt as the end-of-burst marker.  This function
 * converts "DMA write pointer moved from A to B" into "push B-A bytes into the
 * software RX ring", including the wrap case.
 *
 * @param[in] bus  Bus whose RX DMA should be harvested.
 * @return Number of bytes pushed into the software RX ring.
 *
 * @note Safe to call from an ISR.  It is the *only* writer of the bus RX ring.
 */
uint32_t tp_stm32_uart_isr_rx(tp_bus_t bus);

/**
 * @brief Service one DMA stream's transfer-complete/half/error flags.
 *
 * @param[in] dma    1 or 2.
 * @param[in] stream 0..7.
 *
 * @note Safe to call from an ISR.  The flags are cleared here, so this must be
 *       the only handler for that stream: calling `HAL_DMA_IRQHandler()` as well
 *       would race with the flag clearing.
 */
void tp_stm32_dma_irq_handler(int dma, int stream);

/**
 * @brief Service CAN1.
 *
 * @param[in] event 0 for RX FIFO 0 pending, 1 for status-change/error.
 *
 * @note Safe to call from an ISR.
 */
void tp_stm32_can_irq_handler(int event);

/** @brief CAN1 RX FIFO 0 event selector for @ref tp_stm32_can_irq_handler. */
#define TP_STM32_CAN_EVENT_RX0 0
/** @brief CAN1 status-change/error selector for @ref tp_stm32_can_irq_handler. */
#define TP_STM32_CAN_EVENT_SCE 1

/**
 * @brief Service the RS485 driver-enable release timer (TIM7).
 * @note Safe to call from an ISR.
 */
void tp_stm32_tim7_irq_handler(void);

/* =========================================================================
 * Clock verification
 *
 * `SystemInit()` in `firmware/Drivers/startup/system_stm32f4xx.c` owns the clock
 * tree; these two accessors let `main_stm32.c` verify the result instead of
 * assuming it.  Declared here rather than in a private header because every
 * target-only helper this port exposes is declared in exactly one place.
 * ========================================================================= */

/**
 * @brief Whether `SystemInit()` reached the intended 168 MHz SYSCLK.
 *
 * @return 1 when SYSCLK is 168 MHz, 0 when the HSE crystal failed to start and
 *         the core is still on the 16 MHz HSI.
 */
int tp_stm32_clock_is_168mhz(void);

/** @brief The intended SYSCLK, Hz (168000000). */
uint32_t tp_stm32_clock_expected_hz(void);

/* =========================================================================
 * Register-level diagnostics
 *
 * Read-only accessors so the boot self-test and the debug console can print what
 * the hardware actually contains rather than what the code intended.
 * ========================================================================= */

/** @brief Nominal IWDG timeout in ms for the last @ref tp_platform_watchdog_init. */
uint32_t tp_stm32_iwdg_timeout_ms(void);

/** @brief IWDG prescaler divider actually programmed (4, 8, ... 256). */
uint32_t tp_stm32_iwdg_prescaler_div(void);

/** @brief IWDG reload value actually programmed (0..4095). */
uint32_t tp_stm32_iwdg_reload(void);

/** @brief 1 when the IWDG has been started, 0 before @ref tp_platform_watchdog_init. */
int tp_stm32_iwdg_armed(void);

/** @brief Raw `RCC->CSR` snapshot taken at boot, before any flag was cleared. */
uint32_t tp_stm32_reset_flags(void);

/** @brief CAN1 bit-timing register value actually programmed. */
uint32_t tp_stm32_can_btr(void);

/** @brief Achieved bit rate implied by @ref tp_stm32_can_btr, in bit/s. */
uint32_t tp_stm32_can_bitrate_bps(void);

/** @brief CAN1 sample point in tenths of a percent (875 == 87.5 %). */
uint32_t tp_stm32_can_sample_point_tenths(void);

/** @brief 1 when CAN1 is in bus-off. */
int tp_stm32_can_bus_off(void);

/** @brief CAN1 transmit error counter (0..255). */
uint32_t tp_stm32_can_tec(void);

/** @brief CAN1 receive error counter (0..255). */
uint32_t tp_stm32_can_rec(void);

/** @brief Frames dropped because the software CAN RX FIFO was full. */
uint32_t tp_stm32_can_rx_dropped(void);

/** @brief USART parity/framing/noise/overrun events seen on a bus. */
uint32_t tp_stm32_uart_errors(tp_bus_t bus);

/** @brief Bytes dropped because a bus software RX ring was full. */
uint32_t tp_stm32_bus_rx_dropped(tp_bus_t bus);

/**
 * @brief Baud-rate error of a bus, in parts per million.
 *
 * `|achieved - requested| / requested * 1e6`.  Positive means the programmed
 * divisor makes the link run fast.  A UART tolerates roughly 20000 ppm of total
 * error across both ends, so anything under 5000 ppm is not worth a second look
 * and anything over 10000 ppm should be recorded as a known limitation.
 *
 * @param[in] bus  Bus to query.
 * @return Signed error in ppm, 0 for an unknown bus.
 */
int32_t tp_stm32_uart_baud_error_ppm(tp_bus_t bus);

/** @brief Baud rate actually programmed into a bus's BRR, in bit/s. */
uint32_t tp_stm32_uart_actual_baud(tp_bus_t bus);

/** @brief Nesting depth of @ref tp_platform_enter_critical, for diagnostics. */
uint32_t tp_stm32_critical_depth(void);

/** @brief 1 when the DWT cycle counter is running and microsecond delays are exact. */
int tp_stm32_dwt_available(void);

/* =========================================================================
 * Fault-path output
 * ========================================================================= */

/**
 * @brief Write to the debug UART by polling, with interrupts ignored.
 *
 * `tp_platform_bus_write` queues into a DMA ring and returns immediately, which
 * is correct for logs and useless inside a fault handler: the reset would
 * truncate the very message that explains why the reset happened.  This function
 * spins on TXE instead, so it works before the platform is initialised and after
 * an interrupt has corrupted arbitrary state.
 *
 * @param[in] data  Bytes to send.
 * @param[in] len   Number of bytes.
 *
 * @warning Blocking, and it assumes only HSE/PLL and USART1's BRR are still
 *          sane.  Use it from fault handlers only.
 */
void tp_stm32_debug_write_blocking(const char *data, uint32_t len);

/**
 * @brief Write one NUL-terminated string through
 *        @ref tp_stm32_debug_write_blocking.
 * @param[in] s  String, ignored when NULL.
 */
void tp_stm32_debug_puts_blocking(const char *s);

/**
 * @brief Print one 32-bit value as eight hex digits, blocking.
 * @param[in] label  Prefix, may be NULL.
 * @param[in] value  Value to print.
 */
void tp_stm32_debug_put_hex_blocking(const char *label, uint32_t value);

/**
 * @brief Report a fault exception's registers through the debug UART.
 *
 * Prints the exception frame and the fault status registers, then returns so the
 * caller can decide between spinning and resetting.
 *
 * @param[in] exception  `IPSR` value; 3 = HardFault, 4 = MemManage,
 *                       5 = BusFault, 6 = UsageFault.
 * @param[in] frame      Eight words of stacked exception frame
 *                       (R0, R1, R2, R3, R12, LR, PC, xPSR).
 */
void tp_stm32_fault_report(uint32_t exception, const uint32_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* TP_PLATFORM_STM32F407_H */
