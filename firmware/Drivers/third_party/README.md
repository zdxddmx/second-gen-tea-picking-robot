# Vendored third-party sources

Everything under this directory is **verbatim vendor source**. Not one byte of
these files was modified: they were copied from the paths recorded below and the
copy was verified with SHA-256 (`verified=122 mismatches=0`). Every original
copyright, `@attention` block and license header is intact.

If you need a local change, put it in `firmware/BSP/`, `firmware/Core/` or
`firmware/Middleware/` — never here. That rule is what makes a later
`diff -r` against upstream meaningful.

| Component | Version | Copied from | License |
|---|---|---|---|
| STM32F4xx HAL Driver | V1.8.4 | `D:\zhuomian\F407_Balance_Car_Project\Drivers\STM32F4xx_HAL_Driver` | BSD-3-Clause, "Copyright (c) 2017 STMicroelectronics" |
| CMSIS Core (Cortex-M) | 5.6.0 (`cmsis_version.h`), `core_cm4.h` V5.1.2 dated 04. June 2021 | `D:\zhuomian\F407_Balance_Car_Project\Drivers\CMSIS\Include` | Apache-2.0, "Copyright (c) 2009-2020 Arm Limited" |
| CMSIS Device STM32F4xx | STM32Cube FW_F4 device headers (2017 ST headers; `stm32f407xx.h`, `stm32f4xx.h`, `system_stm32f4xx.h`) | `D:\zhuomian\F407_Balance_Car_Project\Drivers\CMSIS\Device\ST\STM32F4xx\Include` | Apache-2.0 (per component `LICENSE.txt`), "Copyright (c) 2017 STMicroelectronics" |
| FreeRTOS Kernel | V10.0.1 | `D:\zhuomian\FreeRTOS_Projects\01_freertos_template\01_freertos_template\Middlewares\Third_Party\FreeRTOS\Source` | MIT ("Permission is hereby granted, free of charge…") |

## What was copied, and what was deliberately left behind

```
STM32F4xx_HAL_Driver/
  Src/   19 x *.c   (hal, hal_adc, hal_adc_ex, hal_cortex, hal_dma, hal_dma_ex,
                     hal_exti, hal_flash, hal_flash_ex, hal_flash_ramfunc,
                     hal_gpio, hal_pwr, hal_pwr_ex, hal_rcc, hal_rcc_ex, hal_tim,
                     hal_tim_ex, hal_uart, ll_adc)
  Inc/   32 x *.h   (the 19 matching headers plus hal_def, gpio_ex, ll_bus,
                     ll_cortex, ll_dma, ll_exti, ll_gpio, ll_pwr, ll_rcc,
                     ll_system, ll_tim, ll_usart, ll_utils)
  LICENSE.txt
CMSIS/
  Include/                          30 x *.h  (core_cm0..core_cm85, cmsis_gcc.h,
                                                cmsis_compiler.h, cmsis_version.h,
                                                mpu_armv7.h, mpu_armv8.h, ...)
  Device/ST/STM32F4xx/Include/       3 x *.h  (stm32f407xx.h, stm32f4xx.h,
                                                system_stm32f4xx.h)
FreeRTOS/Source/
  *.c                               7 x *.c   (croutine, event_groups, list, queue,
                                                stream_buffer, tasks, timers)
  include/                          17 x *.h
  portable/MemMang/                  5 x *.c   (heap_1 .. heap_5)
  portable/RVDS/ARM_CM3/             2 files   (port.c, portmacro.h)  <-- see below
  CMSIS_RTOS_V2/                     4 files   (cmsis_os.h, cmsis_os1.c,
                                                cmsis_os2.c, cmsis_os2.h)
```

### `hal_can` and `hal_iwdg` are absent

The source tree this was copied from contains **no `stm32f4xx_hal_can.c` and no
`stm32f4xx_hal_iwdg.c`**. Nothing was invented to fill the gap: `CAN1` and
`IWDG` in `firmware/BSP/stm32f407/platform_stm32f407.c` are driven by direct
register access against the CMSIS device header. See the comment blocks in that
file for the bit-timing and prescaler arithmetic.

### `portable/RVDS/ARM_CM3` is *not* a usable port for this project

It is vendored for completeness only, and the build **never compiles it**:

1. `RVDS/ARM_CM3` is written for the Arm (Keil) compiler — it uses
   `__asm` blocks and `#pragma` forms that `arm-none-eabi-gcc` rejects.
2. Cortex-M3 has no FPU, so that port saves no floating-point context. This
   target is built `-mfpu=fpv4-sp-d16 -mfloat-abi=hard` and the application uses
   `float` everywhere; a CM3 port would silently corrupt `s0`–`s31` across a
   context switch.

The correct port is `FreeRTOS-Kernel/portable/GCC/ARM_CM4F`, which does not exist
on the machine that produced this tree and cannot be downloaded here (no network
access). `make stm32 RTOS=freertos` therefore fails with an actionable message
instead of silently building something wrong. See `FREERTOS_PORT_DIR` in the root
`Makefile`.

### `Source/Templates` is empty upstream too

There is no `system_stm32f4xx.c` and no startup file in the vendor CMSIS
`Device/ST/STM32F4xx/Source/Templates` directory. Both were written for this
project and live in `firmware/Drivers/startup/`:

- `startup_stm32f407xx.s` — GNU `as` syntax (the vendor `MDK-ARM/*.s` file is
  ARM-assembler syntax and cannot be assembled by `arm-none-eabi-gcc`).
- `system_stm32f4xx.c` — 168 MHz HSE+PLL configuration.

`system_stm32f4xx.h` (the vendor declaration) *is* vendored above, so our
implementation matches the vendor prototype exactly.
