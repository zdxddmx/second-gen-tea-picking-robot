# =============================================================================
#  toolchain/arm-none-eabi.cmake
# =============================================================================
#
#  CMake toolchain file for the STM32F407VET6 target.
#
#  Usage (this is the "CMAKE_TOOLCHAIN_FILE style" the project asks for):
#
#      cmake -S . -B build/cmake-stm32 \
#            -DCMAKE_TOOLCHAIN_FILE=toolchain/arm-none-eabi.cmake \
#            -DCMAKE_BUILD_TYPE=MinSizeRel
#      cmake --build build/cmake-stm32
#
#  -----------------------------------------------------------------------------
#  THIS FILE HAS NOT BEEN EXECUTED
#  -----------------------------------------------------------------------------
#  cmake is not installed on the machine this port was written on
#  (`cmake --version` reports nothing; `Get-Command cmake` finds nothing) and
#  there is no network access to install it.  Everything in `CMakeLists.txt` and
#  in this directory is therefore written to be *faithful* to the root `Makefile`
#  - the same sources, the same flags, the same two targets - but it is
#  unverified.  The Makefile is the verified build.  Treat any CMake/CXX
#  compiler-identification mismatch as "check against the Makefile", not as
#  "the Makefile is wrong".
#  -----------------------------------------------------------------------------
#
#  What a toolchain file must do, and why each line is here:
#    - `CMAKE_SYSTEM_NAME` switches CMake from its host-native default to
#      cross-compiling mode, which is what makes `CMAKE_FIND_ROOT_PATH` and the
#      `try_compile` behaviour correct.
#    - `CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY` stops CMake from trying to
#      *link and run* a test executable during compiler identification.  A
#      Cortex-M binary cannot be executed on the build host, and without this the
#      compiler check fails with a confusing "cannot run test program" error.
#    - `CMAKE_C_FLAGS_INIT` / `CMAKE_EXE_LINKER_FLAGS_INIT` carry the CPU flags
#      so that the compiler-id check itself is done with the right ABI; a check
#      performed without `-mfloat-abi=hard` would succeed and then every real
#      compile would use the wrong ABI.
# =============================================================================

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# --- The compiler itself ------------------------------------------------------
# Override with -DTOOLCHAIN_PREFIX=/path/to/arm-none-eabi- on a machine where it
# is not on PATH.
set(TOOLCHAIN_PREFIX "arm-none-eabi-" CACHE STRING
    "Prefix of the Arm GNU toolchain executables")

set(CMAKE_C_COMPILER   "${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_PREFIX}gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PREFIX}g++")
set(CMAKE_OBJCOPY      "${TOOLCHAIN_PREFIX}objcopy" CACHE FILEPATH "")
set(CMAKE_SIZE         "${TOOLCHAIN_PREFIX}size"    CACHE FILEPATH "")

# --- Never link-and-run a probe binary ---------------------------------------
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# --- Cortex-M4F, hard float ---------------------------------------------------
# Identical to ARM_CPU in the root Makefile:
#   -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
set(TP_CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS_INIT        "${TP_CPU_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT      "${TP_CPU_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${TP_CPU_FLAGS}")

# `-nostartfiles`: Reset_Handler, the vector table, the `.data` copy and the
# `__libc_init_array` call all come from our own startup assembly.  Linking
# newlib's crt0.o as well would add a second, competing startup path.
# `-z noexecstack` is the correct description of an MCU image and silences
# binutils' "libc object is missing .note.GNU-stack" warning.
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "${CMAKE_EXE_LINKER_FLAGS_INIT} -nostartfiles -Wl,-z,noexecstack")

# --- Sizes are in hex-ish form for embedded reports --------------------------
set(CMAKE_SIZE_OUTPUT_FORMAT berkeley)

# --- Do not search the host system for libraries or headers -------------------
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# --- Tell CMake it may skip the usual host startup/ABI probing ---------------
set(CMAKE_C_COMPILER_WORKS   1)
set(CMAKE_CXX_COMPILER_WORKS 1)
set(CMAKE_ASM_COMPILER_WORKS 1)
