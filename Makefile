# =============================================================================
#  second-gen-tea-picking-robot -- single entry-point build
# =============================================================================
#
#  Targets
#  -------
#    make help                 (default) list the targets
#    make host                 build the host firmware binary, plus the unit-test
#                              binary when firmware/Tests/*.c exists
#    make test                 build and run the host unit tests
#    make stm32                build build/stm32/tea_picking_robot.elf (+ .bin/.hex)
#    make stm32 RTOS=freertos  same, but with the FreeRTOS kernel and a GCC
#                              Cortex-M4F port
#    make size                 report the size of the STM32 image
#    make clean                remove build/
#
#  There is one Makefile on purpose.  `firmware/Tests/Makefile` and
#  `firmware/BSP/stm32f407/Makefile` would each need their own copy of the
#  include paths and the toolchain flags, and a flag that is right in one and
#  stale in the other produces a binary that behaves differently from the one
#  under test.  A single file makes "the tests build the same code the firmware
#  builds" a structural property rather than a convention.
#
# -----------------------------------------------------------------------------
#  WHY THIS MAKEFILE LOOKS LIKE IT DOES  (read before editing)
# -----------------------------------------------------------------------------
#
#  The build host for this port runs **GNU Make 3.80** with **cmd.exe** as the
#  recipe shell (`sh.exe` is not on PATH, so make falls back to `cmd.exe`).  Real
#  consequences, each of which has bitten this file already:
#
#   1. `$(info ...)` DOES NOT EXIST before Make 3.81.  Use `$(warning ...)` or
#      `$(error ...)`, both of which are 3.80 features.  A stray `$(info ...)`
#      silently expands to nothing instead of failing.
#   2. `$(realpath)`, `$(abspath)`, `$(or)`, `$(and)`, `$(file ...)` and
#      `.DEFAULT_GOAL` are all 3.81+.  None of them are used here.
#   3. The default goal is the *first* target in the file, so `help` is first.
#   4. Recipes run under `cmd.exe`, so directory creation is
#      `if not exist "dir" mkdir "dir"` and deletion is
#      `if exist "dir" rmdir /s /q "dir"`.  `mkdir -p` and `rm -rf` do not work.
#   5. Each recipe line is a separate `cmd.exe` invocation, so a `cd` does not
#      persist to the next line.  Every recipe below is written to be
#      directory-independent.
#   6. Make's own functions that *do* work in 3.80 and are used here:
#      `$(wildcard)`, `$(foreach)`, `$(if)`, `$(call)`, `$(eval)`, `$(patsubst)`,
#      `$(addprefix)`, `$(addsuffix)`, `$(basename)`, `$(subst)`, `$(filter-out)`,
#      `$(strip)`, `$(words)`, `$(sort)`, `$(dir)`, `$(notdir)`.
#   7. Recursive source discovery uses Make globbing one and two directory levels
#      deep (`dir/*.c` plus `dir/*/*.c`) rather than a `$(shell)`-driven
#      recursive walk: `dir /s /b` output on Windows carries stray carriage
#      returns that corrupt the file list.
#
# -----------------------------------------------------------------------------
#  THE DEFAULT `make stm32` IS BARE METAL, AND THAT IS DELIBERATE
# -----------------------------------------------------------------------------
#
#  `make stm32` with no `RTOS` argument builds against the **bare-metal /
#  cooperative OSAL backend** (`-DTP_OSAL_BAREMETAL=1`), not FreeRTOS.  That is
#  what makes a complete, linked ELF available on this machine today.
#
#  `make stm32 RTOS=freertos` needs a GCC Cortex-M4F port.  The only Cortex-M
#  port vendored in this tree is `portable/RVDS/ARM_CM3`, which is written for
#  the Arm (Keil) compiler and, being CM3, saves no floating-point context at
#  all - on a `-mfloat-abi=hard` build that silently corrupts s0-s31 across every
#  context switch.  The correct port, `portable/GCC/ARM_CM4F`, is not present on
#  this machine and there is no network to fetch it, so the freertos mode fails
#  with an actionable message instead of producing a binary that is wrong in a
#  way that only shows up as an occasional corrupt float.  Point
#  `FREERTOS_PORT_DIR` at a real `portable/GCC/ARM_CM4F` to enable it.
#
# =============================================================================

# -----------------------------------------------------------------------------
#  Toolchain
# -----------------------------------------------------------------------------
HOST_CC      ?= gcc
ARM_CC       ?= arm-none-eabi-gcc
ARM_OBJCOPY  ?= arm-none-eabi-objcopy
ARM_SIZE     ?= arm-none-eabi-size
ARM_NM       ?= arm-none-eabi-nm
ARM_OBJDUMP  ?= arm-none-eabi-objdump

# -----------------------------------------------------------------------------
#  Layout
# -----------------------------------------------------------------------------
FW           := firmware
BUILD        := build

HOST_BUILD   := $(BUILD)/host
HOST_OBJ_DIR := $(HOST_BUILD)/obj

# MinGW's `gcc -o foo` writes `foo.exe`, so the make target has to carry the
# suffix too or make would consider the target permanently out of date and
# relink on every single invocation.
ifeq ($(OS),Windows_NT)
HOST_EXE_SUFFIX := .exe
else
HOST_EXE_SUFFIX :=
endif

HOST_BIN     := $(HOST_BUILD)/tea_picking_robot_host$(HOST_EXE_SUFFIX)
TEST_BIN     := $(HOST_BUILD)/tea_picking_robot_tests$(HOST_EXE_SUFFIX)

STM32_BUILD   := $(BUILD)/stm32
STM32_OBJ_DIR := $(STM32_BUILD)/obj
STM32_ELF     := $(STM32_BUILD)/tea_picking_robot.elf
STM32_BIN     := $(STM32_BUILD)/tea_picking_robot.bin
STM32_HEX     := $(STM32_BUILD)/tea_picking_robot.hex
STM32_MAP     := $(STM32_BUILD)/tea_picking_robot.map

LDSCRIPT := $(FW)/Drivers/startup/STM32F407VET6_FLASH.ld
STARTUP  := $(FW)/Drivers/startup/startup_stm32f407xx.s

# Windows-path helper: `gcc` accepts forward slashes, `cmd.exe` mostly does, but
# `if exist`, `rmdir` and executing a program are safest with backslashes.
WINP = $(subst /,\,$(1))

# A blank line in a cmd.exe recipe.
#
# A bare `@echo.` does NOT work here, and the failure message is confusing:
#     process_begin: CreateProcess((null), echo., ...) failed.
# GNU Make's Windows fast path executes any recipe line containing no shell
# metacharacter *directly* via CreateProcess, and "echo." is not recognised as
# the echo builtin, so make tries to run a program literally called `echo.`.
# Appending `& rem` introduces a metacharacter, which routes the line through
# cmd.exe, where `echo(` prints an empty line and `rem` does nothing.
TP_BLANK_LINE := @echo(& rem

# =============================================================================
#  Default target
#
#  This must be the FIRST target *defined* in this file.  GNU Make 3.80 has no
#  .DEFAULT_GOAL, so the default goal is simply the first target it sees - and
#  the $(foreach ... $(eval ...)) block further down *defines targets*, one per
#  object file.  Putting `help` after that block made a bare `make` compile the
#  first host object instead of printing this text.  The recipe is expanded when
#  the target runs, not when it is parsed, so referencing variables that are
#  assigned further down the file is safe.
# =============================================================================
.PHONY: help
help:
	@echo =====================================================================
	@echo  second-gen-tea-picking-robot - firmware build
	@echo =====================================================================
	$(TP_BLANK_LINE)
	@echo  Targets:
	@echo    make host                  Host (PC) firmware binary + unit tests
	@echo    make test                  Build and run the host unit tests
	@echo    make stm32                 STM32F407VET6 ELF + .bin + .hex (bare metal)
	@echo    make stm32 RTOS=freertos   STM32F407VET6 with the FreeRTOS kernel
	@echo    make size                  Report the STM32 image size
	@echo    make clean                 Remove the build/ directory
	@echo    make help                  This text
	$(TP_BLANK_LINE)
	@echo  Outputs:
	@echo    $(HOST_BIN)
	@echo    $(TEST_BIN)     (when $(FW)/Tests/*.c exists)
	@echo    $(STM32_ELF)
	$(TP_BLANK_LINE)
	@echo  Notes:
	@echo    - `make stm32` defaults to RTOS=$(RTOS): the bare-metal /
	@echo      cooperative OSAL backend, so a complete ELF is produced here today.
	@echo    - `make stm32 RTOS=freertos` needs a GCC Cortex-M4F port at
	@echo      $(FREERTOS_PORT_DIR)
	@echo      which is not vendored on this machine; the build says how to get it.
	$(TP_BLANK_LINE)

# -----------------------------------------------------------------------------
#  Source discovery
#
#  The shared layers are the directories listed in the task brief.  `Core` is
#  listed separately for each target so that `main_stm32.c` can be excluded from
#  the host build and `main_host.c` from the target build: both files define
#  `main`, and a build that links the wrong one fails in a way that looks like a
#  duplicate-symbol problem in the wrong place.
# -----------------------------------------------------------------------------
SHARED_DIRS := $(FW)/Middleware $(FW)/Control $(FW)/Protocol \
               $(FW)/Application $(FW)/Safety $(FW)/Debug

# One and two directory levels, so a new `firmware/Middleware/<subsystem>/x.c`
# is picked up without editing this file.
shared_srcs = $(foreach d,$(1),$(wildcard $(d)/*.c) $(wildcard $(d)/*/*.c))

HOST_SRCS := $(wildcard $(FW)/Core/*.c) \
             $(call shared_srcs,$(SHARED_DIRS)) \
             $(wildcard $(FW)/BSP/host/*.c) \
             $(wildcard $(FW)/BSP/*.c)
HOST_SRCS := $(filter-out $(FW)/Core/main_stm32.c,$(HOST_SRCS))

STM32_C_SRCS := $(wildcard $(FW)/Core/*.c) \
                $(call shared_srcs,$(SHARED_DIRS)) \
                $(wildcard $(FW)/BSP/*.c) \
                $(wildcard $(FW)/BSP/stm32f407/*.c) \
                $(wildcard $(FW)/Drivers/startup/*.c) \
                $(wildcard $(FW)/Drivers/third_party/STM32F4xx_HAL_Driver/Src/*.c)
STM32_C_SRCS := $(filter-out $(FW)/Core/main_host.c,$(STM32_C_SRCS))

TEST_SRCS := $(wildcard $(FW)/Tests/*.c)

# -----------------------------------------------------------------------------
#  Include paths
# -----------------------------------------------------------------------------
HOST_INCLUDES := -I$(FW)/Application -I$(FW)/BSP -I$(FW)/BSP/host \
                 -I$(FW)/Core -I$(FW)/Control -I$(FW)/Debug \
                 -I$(FW)/Middleware/osal -I$(FW)/Middleware/utils \
                 -I$(FW)/Protocol -I$(FW)/Safety -I$(FW)/Tests

# `firmware/BSP/host` is deliberately absent: nothing in the target build may
# reach the simulator's headers, which is the layering rule that keeps the two
# builds the same program.
STM32_INCLUDES := -I$(FW)/Application -I$(FW)/BSP -I$(FW)/BSP/stm32f407 \
                  -I$(FW)/Core -I$(FW)/Control -I$(FW)/Debug \
                  -I$(FW)/Middleware/osal -I$(FW)/Middleware/utils \
                  -I$(FW)/Protocol -I$(FW)/Safety \
                  -I$(FW)/Drivers/startup \
                  -I$(FW)/Drivers/third_party/CMSIS/Include \
                  -I$(FW)/Drivers/third_party/CMSIS/Device/ST/STM32F4xx/Include \
                  -I$(FW)/Drivers/third_party/STM32F4xx_HAL_Driver/Inc

# -----------------------------------------------------------------------------
#  Warning policy
#
#  `-Wno-type-limits` is applied project-wide on purpose.  The defensive
#  argument check used throughout this codebase is
#
#      if (((int)id < 0) || (id >= TP_IO_COUNT)) { return TP_ERR_PARAM; }
#
#  which mirrors the reference backend's contract ("never write to an out-param
#  on failure").  On ARM the enumerator types are unsigned, so `(int)id < 0` is
#  provably dead and gcc emits -Wtype-limits for each of roughly twenty call
#  sites.  The alternative - rewriting every one of them to `(uint32_t)id >=
#  (uint32_t)TP_IO_COUNT` - is what the platform BSP does, but changing the
#  driver layer is not this port's call to make, and a build log with twenty
#  identical false positives is a build log nobody reads.  The check itself is
#  kept, and it is still effective: a negative value cast into an unsigned enum
#  arrives as a huge number and fails the second comparison.
#
#  The only remaining warnings in a `make stm32` build come from three vendored
#  HAL functions - `FLASH_MassErase`, `FLASH_OB_EnableWRP` and
#  `FLASH_OB_DisableWRP` in `stm32f4xx_hal_flash_ex.c` - each of which takes a
#  `Banks` parameter that is genuinely unused on a single-bank part like the
#  STM32F407VE (it exists for the dual-bank F42x/F43x).  Vendored files must stay
#  byte-identical to upstream, so they are not "fixed"; the warnings are left
#  visible rather than suppressed, because they are a true statement about the
#  vendor code and suppressing them would also hide a future real one.
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
#  Host (PC) flags
# -----------------------------------------------------------------------------
HOST_CFLAGS := -std=gnu11 -O2 -g -DTP_PLATFORM_HOST=1 -DTP_OSAL_BAREMETAL=1 \
               -Wall -Wno-type-limits -MMD -MP

# -----------------------------------------------------------------------------
#  ARM Cortex-M4F flags
#
#  `-nostartfiles` because Reset_Handler, the vector table and `__libc_init_array`
#  are all provided by firmware/Drivers/startup/startup_stm32f407xx.s: letting
#  newlib's crt0.o in as well would add a second, conflicting startup path.
#  `-lc`/`-lgcc` are still linked, so `__libc_init_array` resolves.
# -----------------------------------------------------------------------------
ARM_CPU     := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
ARM_DEFS    := -DUSE_HAL_DRIVER -DSTM32F407xx -DTP_PLATFORM_STM32F407=1
ARM_CFLAGS  := $(ARM_CPU) $(ARM_DEFS) $(STM32_INCLUDES) \
               -Os -g3 -ffunction-sections -fdata-sections \
               -Wall -Wextra -Wno-type-limits \
               -MMD -MP
# `-z noexecstack` marks the image as not requiring an executable stack, which is
# both the correct description of an MCU image and what silences binutils' "libc
# object is missing .note.GNU-stack" warning - that note is absent from some
# prebuilt newlib objects and cannot be fixed in a vendored file.
ARM_LDFLAGS := $(ARM_CPU) -nostartfiles -T$(LDSCRIPT) -Wl,--gc-sections \
               -Wl,-z,noexecstack -Wl,-Map=$(STM32_MAP)
ARM_LDLIBS  := -lm -lnosys

# -----------------------------------------------------------------------------
#  RTOS backend selection
#
#  Default: bare metal.  `RTOS=freertos` selects the FreeRTOS kernel and requires
#  a GCC Cortex-M4F port that is NOT vendored here; the check below fails early
#  with the exact command needed to fix it.
# -----------------------------------------------------------------------------
RTOS ?= baremetal

FREERTOS_DIR      ?= $(FW)/Drivers/third_party/FreeRTOS/Source
FREERTOS_PORT_DIR ?= $(FREERTOS_DIR)/portable/GCC/ARM_CM4F

ifeq ($(RTOS),freertos)

ifeq ($(wildcard $(FREERTOS_PORT_DIR)/port.c),)
# A single `$(error)` carrying a multi-line `define` is used rather than several
# `$(error)` calls: make aborts on the first one, so a sequence of them would
# print exactly one line of a message that is only useful as a whole.
define TP_FREERTOS_MISSING_PORT
======================================================================
 RTOS=freertos requires a GCC Cortex-M4F FreeRTOS port, and it is absent:
     $(FREERTOS_PORT_DIR)
 Nothing was compiled.

 The only Cortex-M port vendored in this tree is
     $(FREERTOS_DIR)/portable/RVDS/ARM_CM3
 which is (a) written for the Arm/Keil compiler, whose __asm block syntax and
 #pragma forms arm-none-eabi-gcc rejects, and (b) a Cortex-M3 port that saves no
 floating-point context.  This target is built with
     -mfpu=fpv4-sp-d16 -mfloat-abi=hard
 and the firmware uses 'float' throughout, so a CM3 port would corrupt s0-s31
 across every context switch - silently, and only under load.

 FIX IT by fetching the official kernel and pointing the build at its GCC port:

     git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git
     make stm32 RTOS=freertos \
          FREERTOS_DIR=/path/to/FreeRTOS-Kernel \
          FREERTOS_PORT_DIR=/path/to/FreeRTOS-Kernel/portable/GCC/ARM_CM4F

 (The checkout is also compatible with the V10.0.1 kernel vendored here if you
 would rather not replace it: copy portable/GCC/ARM_CM4F into
 $(FREERTOS_DIR)/portable/GCC/ and leave FREERTOS_DIR alone.)

 This build checked for
     $(FREERTOS_PORT_DIR)/port.c
     $(FREERTOS_PORT_DIR)/portmacro.h
 and neither exists.

 Run `make stm32` instead: it uses the bare-metal / cooperative OSAL backend and
 produces a complete, linked ELF on this machine today.
======================================================================
endef
$(error $(TP_FREERTOS_MISSING_PORT))
endif

ifeq ($(wildcard $(FREERTOS_PORT_DIR)/portmacro.h),)
$(error RTOS=freertos: $(FREERTOS_PORT_DIR) has port.c but no portmacro.h - the port directory is incomplete)
endif

FREERTOS_SRCS := $(wildcard $(FREERTOS_DIR)/*.c) \
                 $(wildcard $(FREERTOS_DIR)/portable/MemMang/heap_4.c) \
                 $(wildcard $(FREERTOS_PORT_DIR)/port.c)
FREERTOS_INCLUDES := -I$(FREERTOS_DIR)/include -I$(FREERTOS_PORT_DIR)
RTOS_DEFS := -DTP_OSAL_FREERTOS=1
RTOS_LDEXTRA :=
else
FREERTOS_SRCS :=
FREERTOS_INCLUDES :=
RTOS_DEFS := -DTP_OSAL_BAREMETAL=1
RTOS_LDEXTRA :=
endif

# -----------------------------------------------------------------------------
#  OSAL backend selection
#
#  `firmware/Middleware/osal` holds two complete implementations of the same API,
#  and **both are unconditional**: each force-defines its own selector macro
#  (`#if !defined(TP_OSAL_FREERTOS) / #define TP_OSAL_FREERTOS 1 / #endif`), so
#  compiling both would give two definitions of `osal_init`, `osal_task_create`
#  and the rest.  Exactly one is therefore selected here.
#
#  The host build always uses the bare-metal backend, which is what makes a PC
#  test deterministic: a preemptive kernel would make the timing of the unit
#  tests depend on the machine.
# -----------------------------------------------------------------------------
OSAL_BAREMETAL_SRC := $(FW)/Middleware/osal/osal_baremetal.c
OSAL_FREERTOS_SRC  := $(FW)/Middleware/osal/osal_freertos.c

HOST_SRCS := $(filter-out $(OSAL_FREERTOS_SRC),$(HOST_SRCS))

ifeq ($(RTOS),freertos)
STM32_C_SRCS := $(filter-out $(OSAL_BAREMETAL_SRC),$(STM32_C_SRCS))
else
STM32_C_SRCS := $(filter-out $(OSAL_FREERTOS_SRC),$(STM32_C_SRCS))
endif

# -----------------------------------------------------------------------------
#  Object lists
#
#  Object names are flattened (`firmware/Core/main_stm32.c` ->
#  `firmware_Core_main_stm32.o`) so that every object lands in one directory and
#  a single `mkdir` suffices - which matters because the recipe shell here is
#  `cmd.exe` and cannot do `mkdir -p`.
# -----------------------------------------------------------------------------
obj_name = $(1)/$(subst /,_,$(basename $(2))).o

HOST_OBJS  := $(foreach s,$(HOST_SRCS),$(call obj_name,$(HOST_OBJ_DIR),$(s)))
TEST_OBJS  := $(foreach s,$(TEST_SRCS),$(call obj_name,$(HOST_OBJ_DIR),$(s)))
STM32_OBJS := $(foreach s,$(STM32_C_SRCS),$(call obj_name,$(STM32_OBJ_DIR),$(s))) \
              $(foreach s,$(FREERTOS_SRCS),$(call obj_name,$(STM32_OBJ_DIR),$(s)))

# The unit-test binary links the test sources against every host source **except
# the host firmware's own main()**.  Both `firmware/Core/main_host.c` and
# `firmware/Tests/test_framework.c` define `main`, and a unit-test binary that
# linked the simulator's main would either fail to link or - worse, with a
# permissive linker - run the simulation instead of the tests.
TEST_HOST_SRCS := $(filter-out $(FW)/Core/main_host.c,$(HOST_SRCS))
TEST_HOST_OBJS := $(foreach s,$(TEST_HOST_SRCS),$(call obj_name,$(HOST_OBJ_DIR),$(s)))

# -----------------------------------------------------------------------------
#  Rule generation
#
#  One explicit rule per source file, generated with `$(foreach ... $(eval
#  $(call ...)))`.  A pattern rule would have to reconstruct the subdirectory
#  from the flattened object name, which is not expressible in make 3.80.
#
#  `$$@` and `$$<` are doubled so that the shell sees `$@`/`$<`... rather, so
#  that the *make* automatic variables survive one round of `$(eval)` expansion.
# -----------------------------------------------------------------------------
define HOST_RULE
$(call obj_name,$(HOST_OBJ_DIR),$(1)): $(1)
	@if not exist "$(call WINP,$(HOST_OBJ_DIR))" mkdir "$(call WINP,$(HOST_OBJ_DIR))"
	$(HOST_CC) $(HOST_CFLAGS) $(HOST_INCLUDES) -c $$< -o $$@
endef

define STM32_RULE
$(call obj_name,$(STM32_OBJ_DIR),$(1)): $(1)
	@if not exist "$(call WINP,$(STM32_OBJ_DIR))" mkdir "$(call WINP,$(STM32_OBJ_DIR))"
	$(ARM_CC) $(ARM_CFLAGS) $(RTOS_DEFS) $(FREERTOS_INCLUDES) -c $$< -o $$@
endef

$(foreach s,$(HOST_SRCS),$(eval $(call HOST_RULE,$(s))))
$(foreach s,$(TEST_SRCS),$(eval $(call HOST_RULE,$(s))))
$(foreach s,$(STM32_C_SRCS),$(eval $(call STM32_RULE,$(s))))
$(foreach s,$(FREERTOS_SRCS),$(eval $(call STM32_RULE,$(s))))

# The assembly startup file is its own rule: it is not C, so it gets no `-std`,
# no `-Os` and no include paths that could confuse the assembler.
$(STM32_OBJ_DIR)/$(subst /,_,$(basename $(STARTUP))).o: $(STARTUP)
	@if not exist "$(call WINP,$(STM32_OBJ_DIR))" mkdir "$(call WINP,$(STM32_OBJ_DIR))"
	$(ARM_CC) $(ARM_CPU) $(ARM_DEFS) -g3 -c $< -o $@

# -----------------------------------------------------------------------------
#  Header dependency tracking
#
#  `-MMD -MP` in the compile flags makes gcc emit a `.d` file next to every
#  object listing the headers it used.  Without this, editing `platform_api.h` or
#  `config.h` would leave stale objects behind and the build would "succeed"
#  while linking code compiled against the old interface - which is exactly the
#  class of bug that makes a `make clean` mandatory and therefore never run.
#
#  `-include` (with the leading dash) is silent about the .d files that do not
#  exist yet, which is every one of them on a fresh checkout.
# -----------------------------------------------------------------------------
-include $(HOST_OBJS:.o=.d) $(TEST_OBJS:.o=.d) $(STM32_OBJS:.o=.d)

# =============================================================================
#  host
# =============================================================================
.PHONY: host
host: $(HOST_BIN) host-tests-if-any
	$(TP_BLANK_LINE)
	@echo [host] built $(call WINP,$(HOST_BIN))

$(HOST_BIN): $(HOST_OBJS)
	$(HOST_CC) $(HOST_CFLAGS) -o $@ $(HOST_OBJS) -lm

# Building the unit-test binary is part of `host`, but only when tests exist.
.PHONY: host-tests-if-any
host-tests-if-any:
ifeq ($(strip $(TEST_SRCS)),)
	@echo [host] no sources in $(FW)/Tests, skipping the unit-test binary
else
	@$(MAKE) --no-print-directory $(TEST_BIN)
endif

$(TEST_BIN): $(TEST_OBJS) $(TEST_HOST_OBJS)
	$(HOST_CC) $(HOST_CFLAGS) -o $@ $(TEST_OBJS) $(TEST_HOST_OBJS) -lm

# =============================================================================
#  test
# =============================================================================
.PHONY: test
test:
ifeq ($(strip $(TEST_SRCS)),)
	@echo =====================================================================
	@echo  [test] SKIPPED - $(FW)/Tests contains no *.c sources yet.
	$(TP_BLANK_LINE)
	@echo  This target globs $(FW)/Tests/*.c, so as soon as the test suite
	@echo  lands it will build and run automatically, with `main` expected in
	@echo  $(FW)/Tests/test_main.c.  Nothing else needs to change.
	@echo =====================================================================
else
	@echo [test] $(words $(TEST_SRCS)) test source(s): $(notdir $(TEST_SRCS))
	@$(MAKE) --no-print-directory $(TEST_BIN)
	@echo [test] running $(call WINP,$(TEST_BIN))
	@"$(call WINP,$(TEST_BIN))"
	@echo [test] exit code 0 - all tests passed
endif

# =============================================================================
#  stm32
# =============================================================================
.PHONY: stm32
stm32: $(STM32_ELF) $(STM32_BIN) $(STM32_HEX)
	$(TP_BLANK_LINE)
	@echo [stm32] RTOS backend: $(RTOS)
	@$(ARM_SIZE) $(STM32_ELF)

$(STM32_ELF): $(STM32_OBJS) $(STM32_OBJ_DIR)/$(subst /,_,$(basename $(STARTUP))).o $(LDSCRIPT)
	@echo [stm32] linking $(notdir $@)
	$(ARM_CC) $(ARM_LDFLAGS) -o $@ $(STM32_OBJS) $(STM32_OBJ_DIR)/$(subst /,_,$(basename $(STARTUP))).o $(ARM_LDLIBS) $(RTOS_LDEXTRA)

$(STM32_BIN): $(STM32_ELF)
	$(ARM_OBJCOPY) -O binary $< $@
	@echo [stm32] wrote $@

$(STM32_HEX): $(STM32_ELF)
	$(ARM_OBJCOPY) -O ihex $< $@
	@echo [stm32] wrote $@

# =============================================================================
#  size / disassembly helpers
# =============================================================================
.PHONY: size
size: $(STM32_ELF)
	@echo --- arm-none-eabi-size ---
	@$(ARM_SIZE) $(STM32_ELF)

.PHONY: sections
sections: $(STM32_ELF)
	@echo --- section headers ---
	@$(ARM_OBJDUMP) -h $(STM32_ELF)

.PHONY: symbols
symbols: $(STM32_ELF)
	@$(ARM_NM) --print-size --size-sort $(STM32_ELF)

# =============================================================================
#  clean
# =============================================================================
.PHONY: clean
clean:
	@if exist "$(call WINP,$(BUILD))" rmdir /s /q "$(call WINP,$(BUILD))"
	@echo [clean] removed $(BUILD)/

.PHONY: distclean
distclean: clean
	@echo [distclean] nothing else to remove (vendored sources are never built in place)
