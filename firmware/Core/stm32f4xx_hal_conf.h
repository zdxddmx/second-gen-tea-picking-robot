/**
  ******************************************************************************
  * @file    stm32f4xx_hal_conf.h
  * @brief   HAL configuration for the tea-picking robot's STM32F407VET6 target.
  *
  * @details
  * This is the application's own replacement for the ST template
  * (`STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_conf_template.h`).  It matches the
  * template's structure — `stm32f4xx_hal.h` includes *this* file and expects
  * *this* file to pull in the per-module headers — but differs in three
  * deliberate ways:
  *
  *   1. **Only vendored modules are enabled.**  The vendored HAL tree has no
  *      `stm32f4xx_hal_can.c` and no `stm32f4xx_hal_iwdg.c` on this machine (see
  *      `firmware/Drivers/third_party/README.md`), so those two module macros are
  *      deliberately absent.  Enabling a module whose header is missing produces
  *      a cascade of "unresolved HAL_XXX_Init" errors that hides the real cause;
  *      leaving it off means CAN1 and IWDG are driven register-level in
  *      `firmware/BSP/stm32f407/platform_stm32f407.c`, which is where the
  *      bit-timing and prescaler arithmetic is documented.
  *   2. **`HSE_VALUE` is 8000000**, not the template's 25 MHz.  The robot's board
  *      fits an 8 MHz crystal.  `HAL_RCC_OscConfig()` validates the PLL
  *      parameters against `HSE_VALUE`, so a wrong value makes the 168 MHz
  *      configuration fail that check.
  *   3. **`TICK_INT_PRIORITY` is 15**, the lowest NVIC priority on a Cortex-M4
  *      with four implemented priority bits.  The HAL tick must never preempt the
  *      control or communication interrupts; in the FreeRTOS build it is also
  *      below `configMAX_SYSCALL_INTERRUPT_PRIORITY`.
  *
  * @copyright MIT License, see LICENSE.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_HAL_CONF_H
#define __STM32F4xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Module Selection
 * ========================================================================== */

#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* Deliberately NOT enabled, with the reason for each:
 *   HAL_CAN_MODULE_ENABLED    - stm32f4xx_hal_can.c is not available; CAN1 is
 *                               driven register-level in the platform BSP.
 *   HAL_IWDG_MODULE_ENABLED   - stm32f4xx_hal_iwdg.c is not available; the IWDG
 *                               is driven register-level in the platform BSP.
 *   HAL_I2C_MODULE_ENABLED    - no I2C device on this board.
 *   HAL_SPI_MODULE_ENABLED    - no SPI device on this board.
 *   HAL_RTC_MODULE_ENABLED    - no RTC backup battery fitted.
 *   HAL_SD_MODULE_ENABLED     - no SD card on the control board.
 *   HAL_NAND_MODULE_ENABLED   - no external memory bus.
 *   HAL_SRAM_MODULE_ENABLED   - no external memory bus.
 *   HAL_ETH_MODULE_ENABLED    - networking lives on the vision host, not here.
 *   HAL_WWDG_MODULE_ENABLED   - the IWDG supervises this design; a windowed
 *                               watchdog whose window must be tuned per build is
 *                               a second failure mode for no extra coverage.
 */

/* ==========================================================================
 * Oscillator Values adaptation
 * ========================================================================== */

/** @brief Crystal fitted on the board, Hz. Drives the 168 MHz PLL solution. */
#if !defined(HSE_VALUE)
#define HSE_VALUE 8000000U
#endif

/** @brief HSE start-up timeout, ms. */
#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT 100U
#endif

/** @brief Internal 16 MHz RC oscillator, Hz. */
#if !defined(HSI_VALUE)
#define HSI_VALUE 16000000U
#endif

/** @brief Internal 32 kHz RC oscillator, Hz. Also clocks the IWDG. */
#if !defined(LSI_VALUE)
#define LSI_VALUE 32000U
#endif

/** @brief 32.768 kHz crystal, Hz. Fitted or not, the value is harmless. */
#if !defined(LSE_VALUE)
#define LSE_VALUE 32768U
#endif

/** @brief LSE start-up timeout, ms. Longer than HSE: a watch crystal is slow. */
#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT 5000U
#endif

/** @brief I2S external clock, Hz. Unused, kept so the macro is never undefined. */
#if !defined(EXTERNAL_CLOCK_VALUE)
#define EXTERNAL_CLOCK_VALUE 12288000U
#endif

/* ==========================================================================
 * System Configuration
 * ========================================================================== */

/** @brief Nominal supply voltage in mV, used by the ADC/PWR helpers. */
#define VDD_VALUE 3300U

/** @brief Lowest NVIC priority for the HAL SysTick interrupt. */
#define TICK_INT_PRIORITY 0x0FU

/** @brief 1 when an RTOS owns the HAL tick. Set by the build, not here. */
#if !defined(USE_RTOS)
#define USE_RTOS 0U
#endif

/** @brief 1 to let `HAL_RCC_ClockConfig` enable the flash prefetch buffer. */
#define PREFETCH_ENABLE 1U

/** @brief 1 to enable the ART instruction cache (mandatory at 168 MHz). */
#define INSTRUCTION_CACHE_ENABLE 1U

/** @brief 1 to enable the ART data cache. */
#define DATA_CACHE_ENABLE 1U

/* ==========================================================================
 * Peripheral callback registration
 *
 * All left at 0.  Registering per-instance callbacks buys nothing here: there is
 * exactly one instance of every peripheral, and the interrupt handlers in
 * `firmware/Core/stm32f4xx_it.c` dispatch through the platform BSP rather than
 * through HAL callbacks, which keeps the ISR path auditable in one place.
 * ========================================================================== */

#define USE_HAL_ADC_REGISTER_CALLBACKS 0U
#define USE_HAL_CAN_REGISTER_CALLBACKS 0U
#define USE_HAL_CEC_REGISTER_CALLBACKS 0U
#define USE_HAL_CRYP_REGISTER_CALLBACKS 0U
#define USE_HAL_DAC_REGISTER_CALLBACKS 0U
#define USE_HAL_DCMI_REGISTER_CALLBACKS 0U
#define USE_HAL_DFSDM_REGISTER_CALLBACKS 0U
#define USE_HAL_DMA2D_REGISTER_CALLBACKS 0U
#define USE_HAL_DSI_REGISTER_CALLBACKS 0U
#define USE_HAL_ETH_REGISTER_CALLBACKS 0U
#define USE_HAL_HASH_REGISTER_CALLBACKS 0U
#define USE_HAL_HCD_REGISTER_CALLBACKS 0U
#define USE_HAL_I2C_REGISTER_CALLBACKS 0U
#define USE_HAL_FMPI2C_REGISTER_CALLBACKS 0U
#define USE_HAL_I2S_REGISTER_CALLBACKS 0U
#define USE_HAL_IRDA_REGISTER_CALLBACKS 0U
#define USE_HAL_LPTIM_REGISTER_CALLBACKS 0U
#define USE_HAL_LTDC_REGISTER_CALLBACKS 0U
#define USE_HAL_MMC_REGISTER_CALLBACKS 0U
#define USE_HAL_NAND_REGISTER_CALLBACKS 0U
#define USE_HAL_NOR_REGISTER_CALLBACKS 0U
#define USE_HAL_PCCARD_REGISTER_CALLBACKS 0U
#define USE_HAL_PCD_REGISTER_CALLBACKS 0U
#define USE_HAL_QSPI_REGISTER_CALLBACKS 0U
#define USE_HAL_RNG_REGISTER_CALLBACKS 0U
#define USE_HAL_RTC_REGISTER_CALLBACKS 0U
#define USE_HAL_SAI_REGISTER_CALLBACKS 0U
#define USE_HAL_SD_REGISTER_CALLBACKS 0U
#define USE_HAL_SMARTCARD_REGISTER_CALLBACKS 0U
#define USE_HAL_SDRAM_REGISTER_CALLBACKS 0U
#define USE_HAL_SRAM_REGISTER_CALLBACKS 0U
#define USE_HAL_SPDIFRX_REGISTER_CALLBACKS 0U
#define USE_HAL_SMBUS_REGISTER_CALLBACKS 0U
#define USE_HAL_SPI_REGISTER_CALLBACKS 0U
#define USE_HAL_TIM_REGISTER_CALLBACKS 0U
#define USE_HAL_UART_REGISTER_CALLBACKS 0U
#define USE_HAL_USART_REGISTER_CALLBACKS 0U
#define USE_HAL_WWDG_REGISTER_CALLBACKS 0U

/* ==========================================================================
 * Module headers
 *
 * `stm32f4xx_hal.h` includes this file and relies on it to pull in the enabled
 * modules, exactly as the ST template does.
 * ========================================================================== */

#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32f4xx_hal_rcc.h"
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32f4xx_hal_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_EXTI_MODULE_ENABLED
#include "stm32f4xx_hal_exti.h"
#endif /* HAL_EXTI_MODULE_ENABLED */

#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32f4xx_hal_dma.h"
#endif /* HAL_DMA_MODULE_ENABLED */

#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32f4xx_hal_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
#include "stm32f4xx_hal_adc.h"
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32f4xx_hal_flash.h"
#endif /* HAL_FLASH_MODULE_ENABLED */

#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32f4xx_hal_pwr.h"
#endif /* HAL_PWR_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32f4xx_hal_tim.h"
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
#include "stm32f4xx_hal_uart.h"
#endif /* HAL_UART_MODULE_ENABLED */

/* ==========================================================================
 * Assert selection
 * ==========================================================================
 *
 * `assert_param()` is called by *every* HAL source file, but it is **not defined
 * anywhere in the vendored HAL headers of this tree**: the ST templates normally
 * carry it in `stm32f4xx_hal_def.h`, and this drop does not (verified with a
 * recursive grep over `STM32F4xx_HAL_Driver/Inc` and its `Legacy` subdirectory).
 *
 * Without a definition the compiler treats `assert_param(x)` as an implicit call
 * to an external function returning `int`.  That does two bad things at once: it
 * floods the build with `-Wimplicit-function-declaration` warnings from twenty
 * vendor files, and it links a call to a symbol that does not exist unless
 * something happens to define it.  It also means the HAL's parameter checks are
 * not "disabled", they are *undefined*, which is not the same thing.
 *
 * Because `stm32f4xx_hal_conf.h` is the one HAL file this project owns - it is
 * the header the HAL itself expects to be configured through - the definition
 * belongs here rather than in a vendored file that must stay byte-identical to
 * upstream.
 *
 * The default mirrors ST's: checking is compiled out.  The HAL's assertions are
 * argument-shape checks that the type system already covers at every call site in
 * this project, and an assertion that fires inside `HAL_ADC_Init()` on a robot
 * with live actuators is a worse outcome than the call simply being wrong.
 * Defining `USE_FULL_ASSERT` turns them into a real trap that reports through the
 * debug UART and resets - useful on the bench, which is why the path is kept.
 */
#ifdef USE_FULL_ASSERT
#define assert_param(expr) \
    ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

/* ==========================================================================
 * Exported macro
 * ========================================================================== */

/* SPI CRC is disabled project-wide; the macro is referenced by the SPI HAL,
 * which is not built here, but leaving it undefined would be an undefined
 * behaviour trap for anyone who enables HAL_SPI_MODULE_ENABLED later. */
#define USE_SPI_CRC 0U

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_CONF_H */
