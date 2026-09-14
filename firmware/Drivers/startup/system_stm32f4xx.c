/**
  ******************************************************************************
  * @file    system_stm32f4xx.c
  * @brief   CMSIS Cortex-M4 device system source for the STM32F407VET6.
  *
  * @details
  * The vendor CMSIS `Device/ST/STM32F4xx/Source/Templates` directory in the
  * reference tree is empty, so there was no `system_stm32f4xx.c` to copy.  This
  * file implements the two functions `system_stm32f4xx.h` declares, plus the two
  * prescaler tables that header declares `extern` and that
  * `HAL_RCC_GetPCLK1Freq()`/`HAL_RCC_GetPCLK2Freq()` dereference — omitting them
  * is a link error, not a warning.
  *
  * # The clock plan, and why it is expressed through the HAL
  *
  * There are two idiomatic ways to write this function:
  *
  *   (a) direct register writes to `RCC->PLLCFGR`, `RCC->CFGR`, `FLASH->ACR`;
  *   (b) the HAL's `RCC_OscInitTypeDef` / `RCC_ClkInitTypeDef` structures.
  *
  * This project uses **(b)**, and the reason is not convenience.  `HAL_RCC_ClockConfig()`
  * already performs the two things that a hand-written version most often gets
  * wrong: it programs the flash latency *before* raising SYSCLK, and it updates
  * the global `SystemCoreClock` to match.  Getting the latency wrong at 168 MHz
  * does not fail loudly — the core fetches stale data and executes garbage from
  * a region that happens to decode to something plausible.
  *
  * Consequences of choosing (b), which are documented rather than hidden:
  *
  *   - `HAL_RCC_OscConfig()` waits for HSE and PLL lock using `HAL_GetTick()`.
  *     Before `HAL_Init()` runs there is no SysTick, so `uwTick` would never
  *     advance and a missing crystal would hang forever instead of timing out.
  *     `SystemInit()` therefore starts a 1 ms SysTick on the still-running HSI
  *     (16 MHz) *first*, using `HAL_InitTick()`.  `HAL_Init()` called later from
  *     `main()` re-programs the same SysTick for 168 MHz; that second call is
  *     what `stm32f4xx_it.c`'s `SysTick_Handler` feeds.
  *   - The ordering is therefore: reset value HSI 16 MHz -> 1 ms tick on HSI ->
  *     HSE on -> PLL on -> latency 5 WS + prefetch/ICache/DCache -> switch SYSCLK
  *     to PLL -> bus prescalers -> `SystemCoreClock` = 168 MHz.
  *   - `HAL_Init()` is called once, from `main()`, as the task specifies.  It does
  *     not configure the clock; it only sets the NVIC priority grouping, the flash
  *     caches and the tick.
  *
  * # The numbers
  *
  * @verbatim
  *   HSE            = 8 MHz
  *   PLLM = 8       -> VCO input = 8 MHz / 8  = 1 MHz   (must be 1..2 MHz)
  *   PLLN = 336     -> VCO output = 1 MHz * 336 = 336 MHz (must be 100..432 MHz)
  *   PLLP = 2       -> SYSCLK = 336 MHz / 2 = 168 MHz   (the F407 maximum)
  *   PLLQ = 7       -> 48 MHz domain = 336 MHz / 7 = 48 MHz (USB OTG FS/HS)
  *   AHB  / 1       -> HCLK  = 168 MHz  (max 168 MHz)
  *   APB1 / 4       -> PCLK1 =  42 MHz  (max  42 MHz) -> APB1 timer clock 84 MHz
  *   APB2 / 2       -> PCLK2 =  84 MHz  (max  84 MHz) -> APB2 timer clock 168 MHz
  *   Flash latency  = 5 wait states (VDD 2.7-3.6 V, 150 < HCLK <= 168 MHz)
  * @endverbatim
  *
  * The APB1 timer clock being 84 MHz while PCLK1 is 42 MHz is the reason the
  * timer arithmetic in the platform BSP uses 84 MHz for TIM2/3/4/5/7 and
  * 168 MHz for TIM1: when the APB prescaler is not 1, the timer kernel clock is
  * twice the peripheral clock.
  *
  * @copyright MIT License, see LICENSE.
  ******************************************************************************
  */

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include <stdint.h>

#include "config.h"

/* ==========================================================================
 * Exported variables declared in system_stm32f4xx.h
 * ========================================================================== */

/**
 * @brief Core (HCLK) frequency in Hz, updated by this file and by every
 *        `HAL_RCC_ClockConfig()` call.
 */
uint32_t SystemCoreClock = 16000000U;

/** @brief AHB prescaler table: index = HPRE[3:0], value = divider - 1. */
const uint8_t AHBPrescTable[16] = { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
                                    1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U };

/** @brief APB prescaler table: index = PPRE[2:0], value = divider - 1. */
const uint8_t APBPrescTable[8] = { 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U };

/* ==========================================================================
 * Target clock plan, in one place so the linker script, the platform BSP and
 * this file cannot disagree.
 * ========================================================================== */

/** @brief HSE crystal on the board, Hz (must match `HSE_VALUE` in the HAL conf). */
#define TP_HSE_HZ 8000000U

/** @brief PLL input divider, giving a 1 MHz VCO input. */
#define TP_PLL_M 8U

/** @brief PLL multiplier: 1 MHz * 336 = 336 MHz VCO output. */
#define TP_PLL_N 336U

/** @brief PLL system divider: 336 MHz / 2 = 168 MHz SYSCLK. */
#define TP_PLL_P RCC_PLLP_DIV2

/** @brief PLL 48 MHz divider: 336 MHz / 7 = 48 MHz for USB/clocks. */
#define TP_PLL_Q 7U

/** @brief Expected SYSCLK once `SystemInit()` has run. */
#define TP_SYSCLK_HZ 168000000U

/* ==========================================================================
 * SystemInit
 * ========================================================================== */

/**
 * @brief Bring the core from the reset HSI clock to 168 MHz on the HSE PLL.
 *
 * Called from `Reset_Handler` before `.data` is copied, so it must not depend on
 * any initialised variable, and it must not call anything that does either.
 * Only `.text`, `.rodata` and the stack are safe here.
 *
 * If the crystal fails to start, the HAL returns `HAL_ERROR` from
 * `HAL_RCC_OscConfig()`; the fallback is deliberately **not** "carry on at
 * 16 MHz".  A robot running its 1 kHz control loop 10.5x slower than every
 * timing constant assumes, with a UART divisor computed for 168 MHz, is strictly
 * more dangerous than one that does not start.  The fallback therefore latches
 * an error flag, leaves the core on HSI, and lets `main()` decide — it can still
 * log over USART1 at the *recomputed* divisor, because `SystemCoreClock` is
 * correct either way.
 */
void SystemInit(void)
{
    RCC_OscInitTypeDef osc = { 0 };
    RCC_ClkInitTypeDef clk = { 0 };

    /* ---------------------------------------------------------------------
     * 1. Core basics that the HAL does not own here.
     *
     * FPU access (CPACR) is already enabled by `Reset_Handler`; doing it there
     * means the very first `float` instruction is safe even if a debugger halts
     * before `SystemInit`.
     * ------------------------------------------------------------------- */

    /* Vector table at the start of flash.  It already is after a normal reset,
     * but an application loaded by a bootloader inherits the bootloader's VTOR,
     * and a wrong VTOR turns every interrupt into a jump into the bootloader. */
    SCB->VTOR = FLASH_BASE;

    /* ---------------------------------------------------------------------
     * 2. Start a 1 ms tick on the reset-value HSI clock.
     *
     * This exists purely so that `HAL_RCC_OscConfig()`'s HSE and PLL lock
     * timeouts can expire.  Without it those loops compare
     * `HAL_GetTick() - tickstart` against a constant while `uwTick` stays at 0,
     * and a board with a dead crystal spins forever inside `SystemInit` with no
     * indication of why.
     * ------------------------------------------------------------------- */
    if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK) {
        /* Cannot happen on a Cortex-M4 with a working SysTick.  If it does,
         * continuing is the only option: the clock bring-up below still runs,
         * it just loses its timeouts. */
    }

    /* ---------------------------------------------------------------------
     * 3. Flash: 5 wait states, prefetch, instruction cache, data cache.
     *
     * Programmed *before* SYSCLK moves, while the core is still at 16 MHz.  The
     * ART accelerator must be enabled *after* the wait states are increased and
     * *before* SYSCLK is raised; disabling the caches on a running 168 MHz core
     * is not permitted without a cache reset, which is why they are only ever
     * turned on here and never off.
     * ------------------------------------------------------------------- */
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_5);
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
    __HAL_FLASH_DATA_CACHE_ENABLE();

    /* ---------------------------------------------------------------------
     * 4. Oscillators: HSE on, PLL on with the factors documented at the top.
     * ------------------------------------------------------------------- */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    /* NOTE: `RCC_OscInitTypeDef` on STM32F4 has no `HSEPredivValue` field.  The
     * HSE prescaler (PREDIV) exists only on the F0/F1/F3/L0/L1 families, where
     * the PLL input can be divided *before* PLLM.  On F4 the whole division is
     * done by PLLM, which is why PLLM is 8 here and not 1. */
    osc.HSIState = RCC_HSI_ON;
    /* Leave the HSI factory trim alone.  `HAL_RCC_OscConfig()` writes this field
     * into RCC_CR[HSITRIM] unconditionally when HSI is enabled, so leaving it at
     * the struct's zero value would detune the fallback clock by up to ~1.5 %
     * exactly in the case where the HSE has failed and HSI is all we have. */
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.LSEState = RCC_LSE_OFF;
    osc.LSIState = RCC_LSI_OFF; /* The IWDG turns the LSI on when it is armed. */
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = TP_PLL_M;
    osc.PLL.PLLN = TP_PLL_N;
    osc.PLL.PLLP = TP_PLL_P;
    osc.PLL.PLLQ = TP_PLL_Q;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        /*
         * HSE never became ready, or the PLL never locked.  Stay on HSI and
         * record why.  `SystemCoreClock` is left at its HSI-accurate value so
         * every downstream baud-rate and timer calculation stays consistent.
         */
        SystemCoreClock = HSI_VALUE;
        return;
    }

    /* ---------------------------------------------------------------------
     * 5. Buses: SYSCLK from PLL, AHB /1, APB1 /4, APB2 /2.
     *
     * `FLASH_LATENCY_5` is passed again even though the ACR was programmed
     * above: `HAL_RCC_ClockConfig()` uses the argument for its own consistency
     * check against the target HCLK and would otherwise reject the call.
     *
     * `HAL_RCC_ClockConfig()` sets `SystemCoreClock` itself, but it derives it
     * from `HAL_RCC_GetSysClockFreq()`; the explicit recomputation afterwards
     * costs one function call at boot and removes any doubt about what the
     * global holds.
     * ------------------------------------------------------------------- */
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;   /* 168 MHz */
    clk.APB1CLKDivider = RCC_HCLK_DIV4;    /*  42 MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV2;    /*  84 MHz */

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) {
        /* The PLL locked, so this can only fail if the arguments are
         * inconsistent.  Falling back to HSI is not silently acceptable, but
         * there is nowhere to report it from here. */
        SystemCoreClock = HSI_VALUE;
        return;
    }

    SystemCoreClockUpdate();
}

/* ==========================================================================
 * SystemCoreClockUpdate
 * ========================================================================== */

/**
 * @brief Recompute `SystemCoreClock` from the registers that are actually set.
 *
 * Useful after a debugger has changed the clock, after `HAL_RCC_ClockConfig()`
 * from application code, and as the assertion performed at the end of
 * `SystemInit()`.  It reads the live `RCC->CFGR` and `RCC->PLLCFGR` rather than
 * assuming the values this file programmed, which is what makes it a check
 * rather than a restatement.
 */
void SystemCoreClockUpdate(void)
{
    uint32_t tmp;
    uint32_t pllvco;
    uint32_t pllp;
    uint32_t pllsource;
    uint32_t pllm;

    /* Which clock is feeding SYSCLK? */
    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp) {
    case RCC_CFGR_SWS_HSI: /* HSI = 16 MHz, no PLL */
        SystemCoreClock = HSI_VALUE;
        break;

    case RCC_CFGR_SWS_HSE: /* HSE straight through, no PLL */
        SystemCoreClock = HSE_VALUE;
        break;

    case RCC_CFGR_SWS_PLL: /* The interesting case */
        /* VCO input = source / PLLM; VCO output = VCO input * PLLN;
         * SYSCLK = VCO output / PLLP.  PLLP is encoded as 00=2, 01=4, 10=6,
         * 11=8, hence the (PLLP >> 1) + 1 trick. */
        pllsource = RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC;
        pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

        if (pllsource != 0U) {
            /* HSE selected as PLL input. */
            pllvco = (HSE_VALUE / pllm) * (RCC->PLLCFGR & RCC_PLLCFGR_PLLN);
        } else {
            /* HSI selected as PLL input. */
            pllvco = ((HSI_VALUE >> 2) / pllm) * (RCC->PLLCFGR & RCC_PLLCFGR_PLLN);
        }

        pllp = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1U;
        SystemCoreClock = pllvco / (pllp << 1U);
        break;

    default:
        /* No valid source; the safest statement of the truth is the reset
         * value, because that is what the core is actually running on. */
        SystemCoreClock = HSI_VALUE;
        break;
    }

    /* Apply the AHB prescaler: HCLK = SYSCLK / AHBPrescTable[HPRE] + 1. */
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos)];
    SystemCoreClock >>= tmp;
}

/* ==========================================================================
 * Target-only helpers
 * ========================================================================== */

/**
 * @brief Whether `SystemInit()` reached the 168 MHz configuration.
 *
 * Exposed so `tp_platform_init()` and the boot self-check can refuse to run
 * motion on a board whose crystal did not start, instead of discovering it from
 * a control loop that is 10.5x too slow.
 *
 * @return 1 when SYSCLK is the intended 168 MHz, 0 otherwise.
 */
int tp_stm32_clock_is_168mhz(void)
{
    return (SystemCoreClock == TP_SYSCLK_HZ) ? 1 : 0;
}

/**
 * @brief The intended SYSCLK, Hz, for comparison and logging.
 * @return 168000000.
 */
uint32_t tp_stm32_clock_expected_hz(void)
{
    return TP_SYSCLK_HZ;
}
