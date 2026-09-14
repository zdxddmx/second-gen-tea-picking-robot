/**
  ******************************************************************************
  * @file      startup_stm32f407xx.s
  * @brief     STM32F407VET6 startup for GNU as (arm-none-eabi-gcc).
  *
  * @details
  * Written for this project.  The vendor `MDK-ARM/startup_stm32f407xx.s` that
  * ships with the reference tree is **ARM assembler** syntax (`AREA`, `DCD`,
  * `IMPORT`, `PROC`/`ENDP`) and `arm-none-eabi-gcc` cannot assemble it, and the
  * vendor `CMSIS/Device/ST/STM32F4xx/Source/Templates` directory is empty, so
  * there was no GCC startup to copy either.
  *
  * What this file does, in order:
  *   1. Places the 98-word vector table at the very start of flash
  *      (`g_pfnVectors`, section `.isr_vector`), which the linker script pins to
  *      `0x08000000` so the core finds it after reset through the flash alias.
  *   2. Defines every external interrupt of the STM32F407xx, 0..81, as a *weak*
  *      alias of `Default_Handler`.  A peripheral that has no real handler in
  *      `firmware/Core/stm32f4xx_it.c` therefore traps in a defined place
  *      instead of jumping to an uninitialised vector.
  *   3. `Reset_Handler`: copies `.data` and `.ccmram` from flash, zeroes `.bss`,
  *      calls `SystemInit()` (clocks, 168 MHz), runs the C++ static constructors
  *      through `__libc_init_array()` and jumps to `main()`.
  *
  * **Why the C runtime initialisation runs *before* `SystemInit()`, and not
  * after it as in the ST template.**  ST's own `system_stm32f4xx.c` configures
  * the clock with direct register writes and therefore works with `.data` still
  * uninitialised.  This project configures the clock through the HAL's
  * `RCC_OscInitTypeDef`/`RCC_ClkInitTypeDef`, and `HAL_RCC_OscConfig()` measures
  * its HSE/PLL lock timeouts with `HAL_GetTick()`, which reads the `uwTick`
  * counter and the `uwTickFreq` scaling factor that `HAL_InitTick()` writes.
  * Both live in `.data`/`.bss`.  Called before the copy, `HAL_InitTick()` would
  * divide by a `uwTickFreq` of zero, and a missing crystal would spin forever
  * inside `SystemInit` instead of timing out.  Copying `.data` first costs
  * nothing (the stack is already valid, and `SystemInit` touches no C global
  * that needs re-initialisation afterwards) and removes the whole class of bug.
  *
  * The vector table is deliberately *not* sorted by usefulness but by hardware
  * IRQ number, exactly as the reference manual Table "Vector table" specifies;
  * a mismatch of one slot silently dispatches the wrong interrupt and is one of
  * the hardest bring-up bugs to find.
  *
  * @copyright MIT License, see LICENSE.
  ******************************************************************************
  */

  .syntax unified
  .cpu cortex-m4
  .fpu fpv4-sp-d16
  .thumb

/* ==========================================================================
 * Stack top
 *
 * SRAM1+SRAM2 on the STM32F407VET6 is 112 KiB + 16 KiB = 128 KiB starting at
 * 0x20000000, so the stack starts at 0x20020000 and grows downwards.  CCM
 * (64 KiB at 0x10000000) is *not* used for the stack: the core cannot access
 * CCM through the D-bus, and a DMA pointer into CCM silently returns zero.
 * ========================================================================== */
  .equ  _estack, 0x20020000

/* ==========================================================================
 * Reset / default handlers
 * ========================================================================== */

  .section .text.Reset_Handler
  .weak     Reset_Handler
  .type     Reset_Handler, %function
Reset_Handler:
  /* Set the stack pointer.  The core has already loaded MSP from
   * g_pfnVectors[0], but doing it explicitly means a debugger that halts at
   * Reset_Handler without running the vector fetch still gets a valid stack. */
  ldr   sp, =_estack

  /* FPU enable: CP10/CP11 full access in CPACR.  `-mfloat-abi=hard` code will
   * fault on the first VFP instruction if this is not set, and the fault is a
   * NOCP UsageFault that is easy to misread as a bad instruction. */
  ldr   r0, =0xE000ED88          /* SCB->CPACR */
  ldr   r1, [r0]
  orr   r1, r1, #(0xF << 20)     /* CP10 = full, CP11 = full */
  str   r1, [r0]
  dsb
  isb

  /* ---- C runtime initialisation, before anything written in C runs ---- */

  /* Copy .data from its flash load address into SRAM. */
  ldr   r0, =_sdata
  ldr   r1, =_edata
  ldr   r2, =_sidata
  movs  r3, #0
  b     LoopCopyData

CopyData:
  ldr   r4, [r2, r3]
  str   r4, [r0, r3]
  adds  r3, r3, #4
LoopCopyData:
  adds  r4, r0, r3
  cmp   r4, r1
  bcc   CopyData

  /* Copy .ccmram from flash into CCM.  CCM cannot hold initialised variables
   * that are addressed by DMA, but it *can* hold hot control-loop scalars, and
   * those do need their initial values. */
  ldr   r0, =_sccmram
  ldr   r1, =_eccmram
  ldr   r2, =_siccmram
  movs  r3, #0
  b     LoopCopyCcm

CopyCcm:
  ldr   r4, [r2, r3]
  str   r4, [r0, r3]
  adds  r3, r3, #4
LoopCopyCcm:
  adds  r4, r0, r3
  cmp   r4, r1
  bcc   CopyCcm

  /* Zero .bss. */
  ldr   r2, =_sbss
  ldr   r4, =_ebss
  movs  r3, #0
  b     LoopFillZerobss

FillZerobss:
  str   r3, [r2]
  adds  r2, r2, #4
LoopFillZerobss:
  cmp   r2, r4
  bcc   FillZerobss

  /* ---- Clock bring-up ----------------------------------------------------
   * HSE 8 MHz -> PLL (M=8, N=336, P=2) -> 168 MHz SYSCLK, AHB 168 MHz,
   * APB1 42 MHz, APB2 84 MHz, 5 flash wait states with prefetch and the ART
   * caches enabled.  Placed after the copies above because the HAL it uses
   * keeps its timeout bookkeeping in `.data`/`.bss`; see the file header. */
  bl    SystemInit

  /* Run C/C++ static constructors.  `tp_format.c` and the logger have no
   * constructors today, but a future `static const` C++ object or an
   * `__attribute__((constructor))` would be silently skipped without this. */
  bl    __libc_init_array

  bl    main

  /* main() must not return.  If it does, the safest defined action on a robot
   * with a live motor is a full chip reset so the watchdog and the safe-state
   * GPIO configuration are re-established from scratch. */
  dsb
  ldr   r0, =0x05FA0004          /* AIRCR: VECTKEY | SYSRESETREQ */
  ldr   r1, =0xE000ED0C
  str   r0, [r1]
  dsb
Infinite_Loop:
  b     Infinite_Loop

  .size Reset_Handler, .-Reset_Handler

/**
 * @brief Trap for any interrupt that has no real handler.
 *
 * An unexpected interrupt that fires here means either a peripheral was enabled
 * without its handler being wired, or the vector table is misaligned with the
 * reference manual.  Parking in a tight loop preserves the stacked PC/PSP for a
 * debugger instead of returning into a random address.
 */
  .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_LoopDefault:
  b     Infinite_LoopDefault
  .size Default_Handler, .-Default_Handler

/* ==========================================================================
 * Vector table
 *
 * Word 0 is the initial MSP.  Words 1..98 are exception and IRQ vectors.
 * ========================================================================== */
  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word MemManage_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler

  /* --- External interrupts, IRQ 0 .. 81 -------------------------------- */
  .word WWDG_IRQHandler                /*  0  Window watchdog             */
  .word PVD_IRQHandler                 /*  1  PVD through EXTI            */
  .word TAMP_STAMP_IRQHandler          /*  2  Tamper / timestamp          */
  .word RTC_WKUP_IRQHandler            /*  3  RTC wakeup                  */
  .word FLASH_IRQHandler               /*  4  Flash                       */
  .word RCC_IRQHandler                 /*  5  RCC                         */
  .word EXTI0_IRQHandler               /*  6  EXTI line 0                 */
  .word EXTI1_IRQHandler               /*  7  EXTI line 1                 */
  .word EXTI2_IRQHandler               /*  8  EXTI line 2                 */
  .word EXTI3_IRQHandler               /*  9  EXTI line 3                 */
  .word EXTI4_IRQHandler               /* 10  EXTI line 4                 */
  .word DMA1_Stream0_IRQHandler        /* 11  DMA1 stream 0               */
  .word DMA1_Stream1_IRQHandler        /* 12  DMA1 stream 1               */
  .word DMA1_Stream2_IRQHandler        /* 13  DMA1 stream 2               */
  .word DMA1_Stream3_IRQHandler        /* 14  DMA1 stream 3               */
  .word DMA1_Stream4_IRQHandler        /* 15  DMA1 stream 4               */
  .word DMA1_Stream5_IRQHandler        /* 16  DMA1 stream 5               */
  .word DMA1_Stream6_IRQHandler        /* 17  DMA1 stream 6               */
  .word ADC_IRQHandler                 /* 18  ADC1/2/3                    */
  .word CAN1_TX_IRQHandler             /* 19  CAN1 TX                     */
  .word CAN1_RX0_IRQHandler            /* 20  CAN1 RX0                    */
  .word CAN1_RX1_IRQHandler            /* 21  CAN1 RX1                    */
  .word CAN1_SCE_IRQHandler            /* 22  CAN1 status change / error  */
  .word EXTI9_5_IRQHandler             /* 23  EXTI lines 5..9             */
  .word TIM1_BRK_TIM9_IRQHandler       /* 24  TIM1 break, TIM9            */
  .word TIM1_UP_TIM10_IRQHandler       /* 25  TIM1 update, TIM10          */
  .word TIM1_TRG_COM_TIM11_IRQHandler  /* 26  TIM1 trigger/comm, TIM11    */
  .word TIM1_CC_IRQHandler             /* 27  TIM1 capture/compare        */
  .word TIM2_IRQHandler                /* 28  TIM2  (joint encoder)       */
  .word TIM3_IRQHandler                /* 29  TIM3                        */
  .word TIM4_IRQHandler                /* 30  TIM4  (gripper encoder)     */
  .word I2C1_EV_IRQHandler             /* 31  I2C1 event                  */
  .word I2C1_ER_IRQHandler             /* 32  I2C1 error                  */
  .word I2C2_EV_IRQHandler             /* 33  I2C2 event                  */
  .word I2C2_ER_IRQHandler             /* 34  I2C2 error                  */
  .word SPI1_IRQHandler                /* 35  SPI1                        */
  .word SPI2_IRQHandler                /* 36  SPI2                        */
  .word USART1_IRQHandler              /* 37  USART1 (debug log)          */
  .word USART2_IRQHandler              /* 38  USART2 (vision link)        */
  .word USART3_IRQHandler              /* 39  USART3 (RS485)              */
  .word EXTI15_10_IRQHandler           /* 40  EXTI lines 10..15           */
  .word RTC_Alarm_IRQHandler           /* 41  RTC alarm A/B               */
  .word OTG_FS_WKUP_IRQHandler         /* 42  OTG FS wakeup               */
  .word TIM8_BRK_TIM12_IRQHandler      /* 43  TIM8 break, TIM12           */
  .word TIM8_UP_TIM13_IRQHandler       /* 44  TIM8 update, TIM13          */
  .word TIM8_TRG_COM_TIM14_IRQHandler  /* 45  TIM8 trigger/comm, TIM14    */
  .word TIM8_CC_IRQHandler             /* 46  TIM8 capture/compare        */
  .word DMA1_Stream7_IRQHandler        /* 47  DMA1 stream 7               */
  .word FMC_IRQHandler                 /* 48  FMC                         */
  .word SDIO_IRQHandler                /* 49  SDIO                        */
  .word TIM5_IRQHandler                /* 50  TIM5                        */
  .word SPI3_IRQHandler                /* 51  SPI3                        */
  .word UART4_IRQHandler               /* 52  UART4                       */
  .word UART5_IRQHandler               /* 53  UART5                       */
  .word TIM6_DAC_IRQHandler            /* 54  TIM6, DAC underrun          */
  .word TIM7_IRQHandler                /* 55  TIM7                        */
  .word DMA2_Stream0_IRQHandler        /* 56  DMA2 stream 0 (ADC1)        */
  .word DMA2_Stream1_IRQHandler        /* 57  DMA2 stream 1               */
  .word DMA2_Stream2_IRQHandler        /* 58  DMA2 stream 2               */
  .word DMA2_Stream3_IRQHandler        /* 59  DMA2 stream 3               */
  .word DMA2_Stream4_IRQHandler        /* 60  DMA2 stream 4               */
  .word ETH_IRQHandler                 /* 61  Ethernet                    */
  .word ETH_WKUP_IRQHandler            /* 62  Ethernet wakeup             */
  .word CAN2_TX_IRQHandler             /* 63  CAN2 TX                     */
  .word CAN2_RX0_IRQHandler            /* 64  CAN2 RX0                    */
  .word CAN2_RX1_IRQHandler            /* 65  CAN2 RX1                    */
  .word CAN2_SCE_IRQHandler            /* 66  CAN2 status change          */
  .word OTG_FS_IRQHandler              /* 67  OTG FS                      */
  .word DMA2_Stream5_IRQHandler        /* 68  DMA2 stream 5               */
  .word DMA2_Stream6_IRQHandler        /* 69  DMA2 stream 6               */
  .word DMA2_Stream7_IRQHandler        /* 70  DMA2 stream 7               */
  .word USART6_IRQHandler              /* 71  USART6                      */
  .word I2C3_EV_IRQHandler             /* 72  I2C3 event                  */
  .word I2C3_ER_IRQHandler             /* 73  I2C3 error                  */
  .word OTG_HS_EP1_OUT_IRQHandler      /* 74  OTG HS endpoint 1 out       */
  .word OTG_HS_EP1_IN_IRQHandler       /* 75  OTG HS endpoint 1 in        */
  .word OTG_HS_WKUP_IRQHandler         /* 76  OTG HS wakeup               */
  .word OTG_HS_IRQHandler              /* 77  OTG HS                      */
  .word DCMI_IRQHandler                /* 78  DCMI                        */
  .word Default_Handler                /* 79  reserved in the RM; trapped */
  .word HASH_RNG_IRQHandler            /* 80  HASH / RNG                  */
  .word FPU_IRQHandler                 /* 81  FPU                         */

/* ==========================================================================
 * Weak definitions for every core exception and every IRQ.
 *
 * `stm32f4xx_it.c` provides strong definitions for the ones this product uses;
 * the linker then discards these aliases.  Using Default_Handler rather than 0
 * means an unexpected interrupt parks in a loop that a debugger can inspect
 * instead of calling through a NULL vector.
 * ========================================================================== */

  .weak      NMI_Handler
  .thumb_set NMI_Handler,Default_Handler

  .weak      HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler

  .weak      MemManage_Handler
  .thumb_set MemManage_Handler,Default_Handler

  .weak      BusFault_Handler
  .thumb_set BusFault_Handler,Default_Handler

  .weak      UsageFault_Handler
  .thumb_set UsageFault_Handler,Default_Handler

  .weak      SVC_Handler
  .thumb_set SVC_Handler,Default_Handler

  .weak      DebugMon_Handler
  .thumb_set DebugMon_Handler,Default_Handler

  .weak      PendSV_Handler
  .thumb_set PendSV_Handler,Default_Handler

  .weak      SysTick_Handler
  .thumb_set SysTick_Handler,Default_Handler

  .weak      WWDG_IRQHandler
  .thumb_set WWDG_IRQHandler,Default_Handler

  .weak      PVD_IRQHandler
  .thumb_set PVD_IRQHandler,Default_Handler

  .weak      TAMP_STAMP_IRQHandler
  .thumb_set TAMP_STAMP_IRQHandler,Default_Handler

  .weak      RTC_WKUP_IRQHandler
  .thumb_set RTC_WKUP_IRQHandler,Default_Handler

  .weak      FLASH_IRQHandler
  .thumb_set FLASH_IRQHandler,Default_Handler

  .weak      RCC_IRQHandler
  .thumb_set RCC_IRQHandler,Default_Handler

  .weak      EXTI0_IRQHandler
  .thumb_set EXTI0_IRQHandler,Default_Handler

  .weak      EXTI1_IRQHandler
  .thumb_set EXTI1_IRQHandler,Default_Handler

  .weak      EXTI2_IRQHandler
  .thumb_set EXTI2_IRQHandler,Default_Handler

  .weak      EXTI3_IRQHandler
  .thumb_set EXTI3_IRQHandler,Default_Handler

  .weak      EXTI4_IRQHandler
  .thumb_set EXTI4_IRQHandler,Default_Handler

  .weak      DMA1_Stream0_IRQHandler
  .thumb_set DMA1_Stream0_IRQHandler,Default_Handler

  .weak      DMA1_Stream1_IRQHandler
  .thumb_set DMA1_Stream1_IRQHandler,Default_Handler

  .weak      DMA1_Stream2_IRQHandler
  .thumb_set DMA1_Stream2_IRQHandler,Default_Handler

  .weak      DMA1_Stream3_IRQHandler
  .thumb_set DMA1_Stream3_IRQHandler,Default_Handler

  .weak      DMA1_Stream4_IRQHandler
  .thumb_set DMA1_Stream4_IRQHandler,Default_Handler

  .weak      DMA1_Stream5_IRQHandler
  .thumb_set DMA1_Stream5_IRQHandler,Default_Handler

  .weak      DMA1_Stream6_IRQHandler
  .thumb_set DMA1_Stream6_IRQHandler,Default_Handler

  .weak      ADC_IRQHandler
  .thumb_set ADC_IRQHandler,Default_Handler

  .weak      CAN1_TX_IRQHandler
  .thumb_set CAN1_TX_IRQHandler,Default_Handler

  .weak      CAN1_RX0_IRQHandler
  .thumb_set CAN1_RX0_IRQHandler,Default_Handler

  .weak      CAN1_RX1_IRQHandler
  .thumb_set CAN1_RX1_IRQHandler,Default_Handler

  .weak      CAN1_SCE_IRQHandler
  .thumb_set CAN1_SCE_IRQHandler,Default_Handler

  .weak      EXTI9_5_IRQHandler
  .thumb_set EXTI9_5_IRQHandler,Default_Handler

  .weak      TIM1_BRK_TIM9_IRQHandler
  .thumb_set TIM1_BRK_TIM9_IRQHandler,Default_Handler

  .weak      TIM1_UP_TIM10_IRQHandler
  .thumb_set TIM1_UP_TIM10_IRQHandler,Default_Handler

  .weak      TIM1_TRG_COM_TIM11_IRQHandler
  .thumb_set TIM1_TRG_COM_TIM11_IRQHandler,Default_Handler

  .weak      TIM1_CC_IRQHandler
  .thumb_set TIM1_CC_IRQHandler,Default_Handler

  .weak      TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler,Default_Handler

  .weak      TIM3_IRQHandler
  .thumb_set TIM3_IRQHandler,Default_Handler

  .weak      TIM4_IRQHandler
  .thumb_set TIM4_IRQHandler,Default_Handler

  .weak      I2C1_EV_IRQHandler
  .thumb_set I2C1_EV_IRQHandler,Default_Handler

  .weak      I2C1_ER_IRQHandler
  .thumb_set I2C1_ER_IRQHandler,Default_Handler

  .weak      I2C2_EV_IRQHandler
  .thumb_set I2C2_EV_IRQHandler,Default_Handler

  .weak      I2C2_ER_IRQHandler
  .thumb_set I2C2_ER_IRQHandler,Default_Handler

  .weak      SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler,Default_Handler

  .weak      SPI2_IRQHandler
  .thumb_set SPI2_IRQHandler,Default_Handler

  .weak      USART1_IRQHandler
  .thumb_set USART1_IRQHandler,Default_Handler

  .weak      USART2_IRQHandler
  .thumb_set USART2_IRQHandler,Default_Handler

  .weak      USART3_IRQHandler
  .thumb_set USART3_IRQHandler,Default_Handler

  .weak      EXTI15_10_IRQHandler
  .thumb_set EXTI15_10_IRQHandler,Default_Handler

  .weak      RTC_Alarm_IRQHandler
  .thumb_set RTC_Alarm_IRQHandler,Default_Handler

  .weak      OTG_FS_WKUP_IRQHandler
  .thumb_set OTG_FS_WKUP_IRQHandler,Default_Handler

  .weak      TIM8_BRK_TIM12_IRQHandler
  .thumb_set TIM8_BRK_TIM12_IRQHandler,Default_Handler

  .weak      TIM8_UP_TIM13_IRQHandler
  .thumb_set TIM8_UP_TIM13_IRQHandler,Default_Handler

  .weak      TIM8_TRG_COM_TIM14_IRQHandler
  .thumb_set TIM8_TRG_COM_TIM14_IRQHandler,Default_Handler

  .weak      TIM8_CC_IRQHandler
  .thumb_set TIM8_CC_IRQHandler,Default_Handler

  .weak      DMA1_Stream7_IRQHandler
  .thumb_set DMA1_Stream7_IRQHandler,Default_Handler

  .weak      FMC_IRQHandler
  .thumb_set FMC_IRQHandler,Default_Handler

  .weak      SDIO_IRQHandler
  .thumb_set SDIO_IRQHandler,Default_Handler

  .weak      TIM5_IRQHandler
  .thumb_set TIM5_IRQHandler,Default_Handler

  .weak      SPI3_IRQHandler
  .thumb_set SPI3_IRQHandler,Default_Handler

  .weak      UART4_IRQHandler
  .thumb_set UART4_IRQHandler,Default_Handler

  .weak      UART5_IRQHandler
  .thumb_set UART5_IRQHandler,Default_Handler

  .weak      TIM6_DAC_IRQHandler
  .thumb_set TIM6_DAC_IRQHandler,Default_Handler

  .weak      TIM7_IRQHandler
  .thumb_set TIM7_IRQHandler,Default_Handler

  .weak      DMA2_Stream0_IRQHandler
  .thumb_set DMA2_Stream0_IRQHandler,Default_Handler

  .weak      DMA2_Stream1_IRQHandler
  .thumb_set DMA2_Stream1_IRQHandler,Default_Handler

  .weak      DMA2_Stream2_IRQHandler
  .thumb_set DMA2_Stream2_IRQHandler,Default_Handler

  .weak      DMA2_Stream3_IRQHandler
  .thumb_set DMA2_Stream3_IRQHandler,Default_Handler

  .weak      DMA2_Stream4_IRQHandler
  .thumb_set DMA2_Stream4_IRQHandler,Default_Handler

  .weak      ETH_IRQHandler
  .thumb_set ETH_IRQHandler,Default_Handler

  .weak      ETH_WKUP_IRQHandler
  .thumb_set ETH_WKUP_IRQHandler,Default_Handler

  .weak      CAN2_TX_IRQHandler
  .thumb_set CAN2_TX_IRQHandler,Default_Handler

  .weak      CAN2_RX0_IRQHandler
  .thumb_set CAN2_RX0_IRQHandler,Default_Handler

  .weak      CAN2_RX1_IRQHandler
  .thumb_set CAN2_RX1_IRQHandler,Default_Handler

  .weak      CAN2_SCE_IRQHandler
  .thumb_set CAN2_SCE_IRQHandler,Default_Handler

  .weak      OTG_FS_IRQHandler
  .thumb_set OTG_FS_IRQHandler,Default_Handler

  .weak      DMA2_Stream5_IRQHandler
  .thumb_set DMA2_Stream5_IRQHandler,Default_Handler

  .weak      DMA2_Stream6_IRQHandler
  .thumb_set DMA2_Stream6_IRQHandler,Default_Handler

  .weak      DMA2_Stream7_IRQHandler
  .thumb_set DMA2_Stream7_IRQHandler,Default_Handler

  .weak      USART6_IRQHandler
  .thumb_set USART6_IRQHandler,Default_Handler

  .weak      I2C3_EV_IRQHandler
  .thumb_set I2C3_EV_IRQHandler,Default_Handler

  .weak      I2C3_ER_IRQHandler
  .thumb_set I2C3_ER_IRQHandler,Default_Handler

  .weak      OTG_HS_EP1_OUT_IRQHandler
  .thumb_set OTG_HS_EP1_OUT_IRQHandler,Default_Handler

  .weak      OTG_HS_EP1_IN_IRQHandler
  .thumb_set OTG_HS_EP1_IN_IRQHandler,Default_Handler

  .weak      OTG_HS_WKUP_IRQHandler
  .thumb_set OTG_HS_WKUP_IRQHandler,Default_Handler

  .weak      OTG_HS_IRQHandler
  .thumb_set OTG_HS_IRQHandler,Default_Handler

  .weak      DCMI_IRQHandler
  .thumb_set DCMI_IRQHandler,Default_Handler

  .weak      HASH_RNG_IRQHandler
  .thumb_set HASH_RNG_IRQHandler,Default_Handler

  .weak      FPU_IRQHandler
  .thumb_set FPU_IRQHandler,Default_Handler

/* ==========================================================================
 * `_init` / `_fini`
 *
 * newlib's `__libc_init_array()` ends with a call to `_init()`, and
 * `__libc_fini_array()` ends with `_fini()`.  On a hosted target those come from
 * `crti.o`/`crtn.o`, which `-nostartfiles` (correctly) does not link: including
 * crt0.o as well as this file would mean two competing startup paths.
 *
 * So they are provided here as empty functions.  That is not a workaround, it is
 * the accurate description of what they do on a bare-metal C project: `_init` is
 * the hook for a hosted C library's static initialisation, and this program's
 * static constructors are already handled by the `.init_array` walk that calls
 * it.  They are weak, so a project that later needs a real `_init` overrides them
 * without touching this file.
 * ========================================================================== */

  .section .text._init,"ax",%progbits
  .weak   _init
  .type   _init, %function
_init:
  bx    lr
  .size _init, .-_init

  .section .text._fini,"ax",%progbits
  .weak   _fini
  .type   _fini, %function
_fini:
  bx    lr
  .size _fini, .-_fini

/* Silence the "GNU-stack section implies executable stack" linker warning that
 * newer binutils emit for hand-written assembly. */
  .section .note.GNU-stack,"",%progbits
