/*
 ******************************************************************************
 * @file    STM32F407VET6_FLASH.ld
 * @brief   GNU ld script for the STM32F407VET6 (512 KiB flash, 128 KiB SRAM,
 *          64 KiB CCM), bare-metal layout with the application at 0x08000000.
 *
 * @details
 * Memory map taken from the STM32F407VE datasheet and the reference manual:
 *
 *   | Region            | Address      | Size   | Notes                          |
 *   |-------------------|--------------|--------|--------------------------------|
 *   | Flash (bank 1)    | 0x08000000   | 512 KiB| up to 1 MiB on the VG part     |
 *   | SRAM1             | 0x20000000   | 112 KiB|                                |
 *   | SRAM2             | 0x2001C000   |  16 KiB| contiguous with SRAM1          |
 *   | CCM data RAM      | 0x10000000   |  64 KiB| CPU D-bus only, **not** DMA    |
 *
 * SRAM1 and SRAM2 are contiguous on this part, so they are described as one
 * 128 KiB region and `.bss`/`.data` are free to span the boundary.  CCM is a
 * separate region because it is *not* reachable by any DMA controller and is
 * deliberately never handed to the ADC or a UART (see the pin map in
 * `firmware/BSP/stm32f407/platform_stm32f407.c`).
 *
 * Region order matters and is load-bearing:
 *   `.isr_vector` must be the first thing at 0x08000000, because the core reads
 *   the initial MSP from 0x08000000 and the reset vector from 0x08000004.
 *   `.data` and `.ccmram` are RAM sections whose *load* image lives in flash, so
 *   they carry `AT> FLASH` and the addresses `_sidata` / `_siccmram` that
 *   `Reset_Handler` copies from.
 *
 * @copyright MIT License, see LICENSE.
 ******************************************************************************
 */

ENTRY(Reset_Handler)

/* ==========================================================================
 * Stack top
 *
 * 0x20000000 + 128 KiB = 0x20020000.  The stack grows down from here, chased by
 * the `._user_heap_stack` reservation below so the heap can never grow into it.
 * ========================================================================== */
_estack = 0x20020000;

/* Heap and stack reservations, in bytes.  `_Min_Heap_Size` is what `_sbrk` may
 * hand out (see `firmware/BSP/stm32f407/syscalls_stm32f407.c`); `_Min_Stack_Size`
 * is the guard band the linker keeps free below `_estack`, and `_sbrk` refuses
 * to cross `_estack - _Min_Stack_Size`. */
_Min_Heap_Size  = 0x2000;  /* 8 KiB  */
_Min_Stack_Size = 0x1000;  /* 4 KiB  */

/* Highest address the heap may reach.  Declared here so the linker script is
 * the single source of truth for the SRAM budget. */
_Heap_Limit = _estack - _Min_Stack_Size;

MEMORY
{
  CCMRAM (xrw) : ORIGIN = 0x10000000, LENGTH = 64K
  RAM    (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
}

SECTIONS
{
  /* ---------------------------------------------------------------------
   * Vector table: must be the first byte of flash.
   * --------------------------------------------------------------------- */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  /* ---------------------------------------------------------------------
   * Code.
   * --------------------------------------------------------------------- */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)          /* ARM -> Thumb veneers, harmless on a pure-M profile */
    *(.glue_7t)
    *(.eh_frame)

    KEEP(*(.init))
    KEEP(*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  /* ---------------------------------------------------------------------
   * Read-only data, including every string literal the logger uses.
   * --------------------------------------------------------------------- */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  /* ---------------------------------------------------------------------
   * Exception unwinding tables.  `-fno-exceptions` means they are normally
   * empty, but `-funwind-tables` (implied by some library objects) can emit
   * them, and an unresolved `.ARM.exidx` reference is a hard link error.
   * --------------------------------------------------------------------- */
  .ARM.extab :
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM :
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  /* ---------------------------------------------------------------------
   * C runtime init/fini arrays.  `__libc_init_array()`, called from
   * `Reset_Handler`, walks these; without the PROVIDE_HIDDEN symbols newlib
   * would fall back to its built-in (and here empty) definitions.
   * --------------------------------------------------------------------- */
  .preinit_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* ---------------------------------------------------------------------
   * Initialised data: lives in SRAM, its initial image is in flash.
   * --------------------------------------------------------------------- */
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    __data_start__ = _sdata;

    *(.data)
    *(.data*)
    *(.RamFunc)
    *(.RamFunc*)

    . = ALIGN(4);
    _edata = .;
    __data_end__ = _edata;
  } >RAM AT> FLASH

  /* ---------------------------------------------------------------------
   * Zero-initialised data.
   * --------------------------------------------------------------------- */
  . = ALIGN(4);
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;

    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  /* ---------------------------------------------------------------------
   * Heap and stack reservation.
   *
   * This section holds no contents (`NOLOAD` is implied by the absence of
   * input sections and the explicit advance below): it exists purely to make
   * the linker account for the heap and the stack so that a growing `.bss`
   * fails the link instead of silently eating the stack.
   * --------------------------------------------------------------------- */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(_heap_start = .);
    PROVIDE(end = .);          /* newlib `_sbrk` looks for `end`            */
    PROVIDE(_end = .);         /* some toolchains and SDKs look for `_end`  */
    PROVIDE(__end__ = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* ---------------------------------------------------------------------
   * Core-coupled memory.
   *
   * CCM is ideal for hot control-loop state and the 16-byte ADC DMA buffer is
   * deliberately *not* placed here (DMA cannot reach CCM).  Variables are put
   * here explicitly with `__attribute__((section(".ccmram")))`; nothing lands
   * here by accident.
   * --------------------------------------------------------------------- */
  _siccmram = LOADADDR(.ccmram);

  .ccmram :
  {
    . = ALIGN(4);
    _sccmram = .;
    __ccmram_start__ = _sccmram;

    *(.ccmram)
    *(.ccmram*)

    . = ALIGN(4);
    _eccmram = .;
    __ccmram_end__ = _eccmram;
  } >CCMRAM AT> FLASH

  /* ---------------------------------------------------------------------
   * Anything the toolchain emits that has no business in the image.
   * --------------------------------------------------------------------- */
  /DISCARD/ :
  {
    *(.comment)
    *(.note.GNU-stack)
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }

  /* ---------------------------------------------------------------------
   * Layout invariants, checked at link time.
   * --------------------------------------------------------------------- */
  ASSERT(_estack == 0x20020000,
         "Stack top must be the end of the 128 KiB SRAM region")
  ASSERT(_sidata == LOADADDR(.data),
         "Data initialiser image address is inconsistent")
  ASSERT((_edata - _sdata) < LENGTH(RAM),
         ".data does not fit in SRAM")
  ASSERT((_ebss - _sbss) < LENGTH(RAM),
         ".bss does not fit in SRAM")
  ASSERT(_eccmram <= ORIGIN(CCMRAM) + LENGTH(CCMRAM),
         ".ccmram overflows the 64 KiB CCM region")
  ASSERT(SIZEOF(.isr_vector) == 98 * 4,
         "Vector table must have 98 words: initial MSP + 15 system + 82 IRQs")
  ASSERT(LOADADDR(.isr_vector) == ORIGIN(FLASH),
         "Vector table must start at 0x08000000")
  ASSERT(_end <= _Heap_Limit,
         "Static data leaves no room for the reserved heap and stack")
}
