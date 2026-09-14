# =============================================================================
#  second-gen-tea-picking-robot - root CMake configuration
# =============================================================================
#
#  !! THIS FILE HAS NEVER BEEN RUN !!
#
#  `cmake` is not installed on the machine this port was written on
#  (`cmake --version` reports nothing and `Get-Command cmake` finds nothing) and
#  there is no network access to install it.  Every command in the verification
#  report was therefore produced with the root **Makefile**, which is the
#  verified build.
#
#  This file is written to be *faithful* to that Makefile - the same source
#  lists, the same include paths, the same compile and link flags, the same two
#  targets, the same RTOS guard - so that a machine with cmake produces the same
#  artefacts.  Where behaviour could plausibly differ it is called out in a
#  comment below.  If CMake and the Makefile ever disagree, the Makefile is right
#  and this file is the thing to fix.
#
#  -----------------------------------------------------------------------------
#  Equivalence table (CMake option -> Makefile variable)
#  -----------------------------------------------------------------------------
#    -DTP_BUILD_HOST=ON        -> make host
#    -DTP_BUILD_TESTS=ON       -> make test
#    -DTP_BUILD_STM32=ON       -> make stm32
#    -DTP_RTOS=freertos        -> make stm32 RTOS=freertos
#    -DTP_TESTS_DIR=...        -> the $(FW)/Tests glob
#    -DTOOLCHAIN_PREFIX=...    -> the ARM_CC/ARM_OBJCOPY/ARM_SIZE overrides
#
#  -----------------------------------------------------------------------------
#  Usage
#  -----------------------------------------------------------------------------
#    # host firmware + unit tests
#    cmake -S . -B build/cmake-host
#    cmake --build build/cmake-host
#    ctest --test-dir build/cmake-host
#
#    # STM32F407VET6
#    cmake -S . -B build/cmake-stm32 \
#          -DCMAKE_TOOLCHAIN_FILE=toolchain/arm-none-eabi.cmake \
#          -DTP_BUILD_HOST=OFF -DTP_BUILD_STM32=ON \
#          -DCMAKE_BUILD_TYPE=MinSizeRel
#    cmake --build build/cmake-stm32
#
#  -----------------------------------------------------------------------------
#  Deliberate differences from the Makefile, and why
#  -----------------------------------------------------------------------------
#    1. The Makefile discovers sources with `$(wildcard dir/*.c dir/*/*.c)`
#       (one and two levels, because GNU Make cannot glob recursively).  This
#       file uses `file(GLOB_RECURSE)` followed by the *same* exclusion list, so
#       it discovers a superset and then removes exactly the same files.  The
#       resulting source set is identical today; if a source is ever added three
#       directories deep, CMake finds it and the Makefile does not - which is
#       one more reason the CMake build is the convenient one and the Makefile
#       is the authoritative one.
#    2. `CONFIGURE_DEPENDS` is used on the globs so that adding a test file
#       re-runs CMake automatically.  The Makefile re-globs on every invocation
#       because make expands `$(wildcard)` at parse time, so the two behave the
#       same from the user's point of view.
#    3. The Makefile globs `firmware/Tests/*.c` and skips `make test` when the
#       glob is empty.  Here the `tea_picking_robot_tests` target is only created
#       when the glob is non-empty, and `ctest` reports "no tests" otherwise -
#       the same outcome, expressed the way CMake wants it.
# =============================================================================

cmake_minimum_required(VERSION 3.16)

project(tea_picking_robot
        VERSION 2.0.0
        DESCRIPTION "Second-generation tea-picking robot firmware"
        LANGUAGES C ASM)

# -----------------------------------------------------------------------------
#  Options - mirroring the Makefile's targets and variables
# -----------------------------------------------------------------------------
option(TP_BUILD_HOST  "Build the host (PC) firmware binary"       ON)
option(TP_BUILD_TESTS "Build the host unit-test binary"           ON)
option(TP_BUILD_STM32 "Build the STM32F407VET6 target"            OFF)
set(TP_RTOS "baremetal" CACHE STRING "OSAL backend: baremetal or freertos")
set_property(CACHE TP_RTOS PROPERTY STRINGS baremetal freertos)

set(TP_TESTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/firmware/Tests" CACHE PATH
    "Directory holding the host unit tests (the Makefile globs firmware/Tests/*.c)")

set(TP_FREERTOS_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/firmware/Drivers/third_party/FreeRTOS/Source"
    CACHE PATH "Vendored FreeRTOS kernel root")
set(TP_FREERTOS_PORT_DIR "${TP_FREERTOS_DIR}/portable/GCC/ARM_CM4F" CACHE PATH
    "GCC Cortex-M4F FreeRTOS port directory (NOT vendored on this machine)")

set(TP_HOST_BUILD_DIR "${CMAKE_BINARY_DIR}/host")
set(TP_STM32_BUILD_DIR "${CMAKE_BINARY_DIR}/stm32")

# -----------------------------------------------------------------------------
#  Source discovery
# -----------------------------------------------------------------------------
set(TP_FW "${CMAKE_CURRENT_SOURCE_DIR}/firmware")

# Shared layers, both host and target.  `Core` is listed separately for each
# target below so that exactly one of main_host.c / main_stm32.c is linked:
# both define `main`, and a build that links the wrong one fails in a way that
# looks like a duplicate-symbol problem in the wrong file.
set(TP_SHARED_DIRS
    "${TP_FW}/Middleware"
    "${TP_FW}/Control"
    "${TP_FW}/Protocol"
    "${TP_FW}/Application"
    "${TP_FW}/Safety"
    "${TP_FW}/Debug")

file(GLOB_RECURSE TP_SHARED_SRCS CONFIGURE_DEPENDS
     "${TP_FW}/Middleware/*.c"
     "${TP_FW}/Control/*.c"
     "${TP_FW}/Protocol/*.c"
     "${TP_FW}/Application/*.c"
     "${TP_FW}/Safety/*.c"
     "${TP_FW}/Debug/*.c")

# The OSAL has two complete backends and *both* force-define their own selector
# macro, so compiling both would give duplicate definitions of osal_init() and
# friends.  Exactly one is selected, exactly as the Makefile does.
set(TP_OSAL_BAREMETAL_SRC "${TP_FW}/Middleware/osal/osal_baremetal.c")
set(TP_OSAL_FREERTOS_SRC  "${TP_FW}/Middleware/osal/osal_freertos.c")

# -----------------------------------------------------------------------------
#  Include paths
# -----------------------------------------------------------------------------
set(TP_HOST_INCLUDES
    "${TP_FW}/Application" "${TP_FW}/BSP" "${TP_FW}/BSP/host"
    "${TP_FW}/Core" "${TP_FW}/Control" "${TP_FW}/Debug"
    "${TP_FW}/Middleware/osal" "${TP_FW}/Middleware/utils"
    "${TP_FW}/Protocol" "${TP_FW}/Safety" "${TP_TESTS_DIR}")

# `firmware/BSP/host` is deliberately absent from the target list: nothing in
# the target build may reach the simulator's headers.  That layering rule is what
# keeps the two builds the same program.
set(TP_STM32_INCLUDES
    "${TP_FW}/Application" "${TP_FW}/BSP" "${TP_FW}/BSP/stm32f407"
    "${TP_FW}/Core" "${TP_FW}/Control" "${TP_FW}/Debug"
    "${TP_FW}/Middleware/osal" "${TP_FW}/Middleware/utils"
    "${TP_FW}/Protocol" "${TP_FW}/Safety"
    "${TP_FW}/Drivers/startup"
    "${TP_FW}/Drivers/third_party/CMSIS/Include"
    "${TP_FW}/Drivers/third_party/CMSIS/Device/ST/STM32F4xx/Include"
    "${TP_FW}/Drivers/third_party/STM32F4xx_HAL_Driver/Inc")

# -----------------------------------------------------------------------------
#  Host (PC) build
# -----------------------------------------------------------------------------
if(TP_BUILD_HOST)

    file(GLOB_RECURSE TP_HOST_CORE_SRCS CONFIGURE_DEPENDS "${TP_FW}/Core/*.c")
    file(GLOB TP_BSP_ROOT_SRCS      CONFIGURE_DEPENDS "${TP_FW}/BSP/*.c")
    file(GLOB TP_BSP_HOST_SRCS      CONFIGURE_DEPENDS "${TP_FW}/BSP/host/*.c")

    set(TP_HOST_SRCS
        ${TP_HOST_CORE_SRCS}
        ${TP_SHARED_SRCS}
        ${TP_BSP_HOST_SRCS}
        ${TP_BSP_ROOT_SRCS})

    # The target entry point and the FreeRTOS OSAL backend are removed, exactly
    # as the Makefile filters them out.
    list(REMOVE_ITEM TP_HOST_SRCS
         "${TP_FW}/Core/main_stm32.c"
         "${TP_OSAL_FREERTOS_SRC}")

    add_executable(tea_picking_robot_host EXCLUDE_FROM_ALL ${TP_HOST_SRCS})
    target_include_directories(tea_picking_robot_host PRIVATE ${TP_HOST_INCLUDES})
    # Identical to HOST_CFLAGS in the Makefile, including -Wno-type-limits and
    # the reason for it (see the Makefile's "Warning policy" comment).
    target_compile_options(tea_picking_robot_host PRIVATE
        -std=gnu11 -O2 -g -Wall -Wno-type-limits)
    target_compile_definitions(tea_picking_robot_host PRIVATE
        TP_PLATFORM_HOST=1 TP_OSAL_BAREMETAL=1)
    target_link_libraries(tea_picking_robot_host PRIVATE m)

    set_target_properties(tea_picking_robot_host PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${TP_HOST_BUILD_DIR}")

    message(STATUS "TP host sources: ${TP_HOST_SRCS}")

    # --- Unit tests ----------------------------------------------------------
    file(GLOB TP_TEST_SRCS CONFIGURE_DEPENDS "${TP_TESTS_DIR}/*.c")

    if(TP_BUILD_TESTS AND TP_TEST_SRCS)
        # The test binary links the test sources *plus* every host source, so a
        # test exercises the production code rather than a copy of it.
        add_executable(tea_picking_robot_tests EXCLUDE_FROM_ALL
                       ${TP_TEST_SRCS} ${TP_HOST_SRCS})
        target_include_directories(tea_picking_robot_tests PRIVATE
                                   ${TP_HOST_INCLUDES})
        target_compile_options(tea_picking_robot_tests PRIVATE
            -std=gnu11 -O2 -g -Wall -Wno-type-limits)
        target_compile_definitions(tea_picking_robot_tests PRIVATE
            TP_PLATFORM_HOST=1 TP_OSAL_BAREMETAL=1)
        target_link_libraries(tea_picking_robot_tests PRIVATE m)
        set_target_properties(tea_picking_robot_tests PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY "${TP_HOST_BUILD_DIR}")

        enable_testing()
        add_test(NAME unit_tests
                 COMMAND tea_picking_robot_tests
                 WORKING_DIRECTORY "${TP_HOST_BUILD_DIR}")
    elseif(TP_BUILD_TESTS)
        # Same outcome as the Makefile's "test SKIPPED" message: the suite is not
        # there yet, so say so rather than configuring a target that cannot link.
        message(STATUS
            "TP: no sources in ${TP_TESTS_DIR} - the unit-test target is skipped, "
            "and 'make test' / ctest will report that there is nothing to run.")
    endif()
endif()

# -----------------------------------------------------------------------------
#  STM32F407VET6 build
# -----------------------------------------------------------------------------
if(TP_BUILD_STM32)

    if(NOT CMAKE_CROSSCOMPILING)
        message(FATAL_ERROR
            "TP_BUILD_STM32=ON requires the cross toolchain. Re-run with "
            "-DCMAKE_TOOLCHAIN_FILE=toolchain/arm-none-eabi.cmake")
    endif()

    # The toolchain support directory: it reports the resolved toolchain and
    # warns when objcopy/size are missing.
    add_subdirectory(toolchain)

    # --- RTOS guard: the same actionable failure as the Makefile -------------
    if(TP_RTOS STREQUAL "freertos")
        if(NOT EXISTS "${TP_FREERTOS_PORT_DIR}/port.c")
            message(FATAL_ERROR
                "================================================================\n"
                " RTOS=freertos requires a GCC Cortex-M4F FreeRTOS port, and it\n"
                " is absent:\n"
                "     ${TP_FREERTOS_PORT_DIR}\n"
                " Nothing was compiled.\n"
                "\n"
                " The only Cortex-M port vendored in this tree is\n"
                "     ${TP_FREERTOS_DIR}/portable/RVDS/ARM_CM3\n"
                " which is written for the Arm/Keil compiler and is a Cortex-M3\n"
                " port, so it saves no floating-point context. This target is\n"
                " built -mfpu=fpv4-sp-d16 -mfloat-abi=hard, so using it would\n"
                " corrupt s0-s31 across every context switch, silently.\n"
                "\n"
                " FIX IT:\n"
                "     git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git\n"
                "     cmake -S . -B build/cmake-stm32 \\\n"
                "           -DCMAKE_TOOLCHAIN_FILE=toolchain/arm-none-eabi.cmake \\\n"
                "           -DTP_BUILD_HOST=OFF -DTP_BUILD_STM32=ON \\\n"
                "           -DTP_RTOS=freertos \\\n"
                "           -DTP_FREERTOS_DIR=/path/to/FreeRTOS-Kernel \\\n"
                "           -DTP_FREERTOS_PORT_DIR=/path/to/FreeRTOS-Kernel/portable/GCC/ARM_CM4F\n"
                "\n"
                " Or configure without -DTP_RTOS=freertos and get the bare-metal\n"
                " image, which is complete and links today.\n"
                "================================================================")
        endif()
    endif()

    file(GLOB_RECURSE TP_STM32_CORE_SRCS CONFIGURE_DEPENDS "${TP_FW}/Core/*.c")
    file(GLOB TP_STM32_BSP_SRCS      CONFIGURE_DEPENDS "${TP_FW}/BSP/stm32f407/*.c")
    file(GLOB TP_STM32_BSPX_SRCS     CONFIGURE_DEPENDS "${TP_FW}/BSP/*.c")
    file(GLOB TP_STM32_HAL_SRCS      CONFIGURE_DEPENDS
         "${TP_FW}/Drivers/third_party/STM32F4xx_HAL_Driver/Src/*.c")

    set(TP_STM32_SRCS
        ${TP_STM32_CORE_SRCS}
        ${TP_SHARED_SRCS}
        ${TP_STM32_BSPX_SRCS}
        ${TP_STM32_BSP_SRCS}
        ${TP_STM32_HAL_SRCS})

    list(REMOVE_ITEM TP_STM32_SRCS "${TP_FW}/Core/main_host.c")

    if(TP_RTOS STREQUAL "freertos")
        list(REMOVE_ITEM TP_STM32_SRCS "${TP_OSAL_BAREMETAL_SRC}")
        file(GLOB TP_FREERTOS_SRCS CONFIGURE_DEPENDS
             "${TP_FREERTOS_DIR}/*.c"
             "${TP_FREERTOS_DIR}/portable/MemMang/heap_4.c"
             "${TP_FREERTOS_PORT_DIR}/port.c")
        list(APPEND TP_STM32_SRCS ${TP_FREERTOS_SRCS})
        set(TP_STM32_DEFS -DUSE_HAL_DRIVER -DSTM32F407xx
                          -DTP_PLATFORM_STM32F407=1 -DTP_OSAL_FREERTOS=1)
        set(TP_STM32_EXTRA_INCLUDES
            "${TP_FREERTOS_DIR}/include" "${TP_FREERTOS_PORT_DIR}")
    else()
        list(REMOVE_ITEM TP_STM32_SRCS "${TP_OSAL_FREERTOS_SRC}")
        set(TP_STM32_DEFS -DUSE_HAL_DRIVER -DSTM32F407xx
                          -DTP_PLATFORM_STM32F407=1 -DTP_OSAL_BAREMETAL=1)
        set(TP_STM32_EXTRA_INCLUDES "")
    endif()

    set(TP_STARTUP_SRC "${TP_FW}/Drivers/startup/startup_stm32f407xx.s")
    set(TP_LDSCRIPT    "${TP_FW}/Drivers/startup/STM32F407VET6_FLASH.ld")

    add_executable(tea_picking_robot.elf
                   ${TP_STM32_SRCS} ${TP_STARTUP_SRC})

    target_include_directories(tea_picking_robot.elf PRIVATE
        ${TP_STM32_INCLUDES} ${TP_STM32_EXTRA_INCLUDES})
    # Identical to ARM_CFLAGS in the Makefile.
    target_compile_options(tea_picking_robot.elf PRIVATE
        -Os -g3 -ffunction-sections -fdata-sections -Wall -Wextra -Wno-type-limits)
    target_compile_definitions(tea_picking_robot.elf PRIVATE ${TP_STM32_DEFS})
    # Identical to ARM_LDFLAGS: our own startup file, so no crt0.o.
    target_link_options(tea_picking_robot.elf PRIVATE
        -nostartfiles
        "-T${TP_LDSCRIPT}"
        -Wl,--gc-sections
        -Wl,-z,noexecstack
        "-Wl,-Map=${TP_STM32_BUILD_DIR}/tea_picking_robot.map")
    target_link_libraries(tea_picking_robot.elf PRIVATE m nosys)

    set_target_properties(tea_picking_robot.elf PROPERTIES
        OUTPUT_NAME "tea_picking_robot"
        SUFFIX ".elf"
        RUNTIME_OUTPUT_DIRECTORY "${TP_STM32_BUILD_DIR}")

    message(STATUS "TP STM32 RTOS backend: ${TP_RTOS}")

    # --- .bin and .hex, and the size report, mirroring the Makefile ----------
    find_program(TP_OBJCOPY_BIN objcopy HINTS "${CMAKE_OBJCOPY}")
    find_program(TP_SIZE_BIN    size    HINTS "${CMAKE_SIZE}")

    if(TP_OBJCOPY_BIN)
        add_custom_command(TARGET tea_picking_robot.elf POST_BUILD
            COMMAND "${TP_OBJCOPY_BIN}" -O binary "$<TARGET_FILE:tea_picking_robot.elf>"
                    "${TP_STM32_BUILD_DIR}/tea_picking_robot.bin"
            COMMAND "${TP_OBJCOPY_BIN}" -O ihex "$<TARGET_FILE:tea_picking_robot.elf>"
                    "${TP_STM32_BUILD_DIR}/tea_picking_robot.hex"
            COMMENT "TP: objcopy -> .bin and .hex")
    else()
        message(WARNING "TP: objcopy not found; .bin/.hex will not be produced")
    endif()

    if(TP_SIZE_BIN)
        add_custom_command(TARGET tea_picking_robot.elf POST_BUILD
            COMMAND "${TP_SIZE_BIN}" "$<TARGET_FILE:tea_picking_robot.elf>"
            COMMENT "TP: image size")
    else()
        message(WARNING "TP: size not found; no size report will be printed")
    endif()

    # `make size` / `make sections` / `make symbols` equivalents.
    find_program(TP_OBJDUMP_BIN objdump)
    find_program(TP_NM_BIN      nm)

    add_custom_target(tp_sections
        COMMAND "${TP_OBJDUMP_BIN}" -h "$<TARGET_FILE:tea_picking_robot.elf>"
        DEPENDS tea_picking_robot.elf
        COMMENT "TP: section headers")

    add_custom_target(tp_symbols
        COMMAND "${TP_NM_BIN}" --print-size --size-sort
                "$<TARGET_FILE:tea_picking_robot.elf>"
        DEPENDS tea_picking_robot.elf
        COMMENT "TP: symbol sizes")
endif()

# -----------------------------------------------------------------------------
#  A `tp_help` target, mirroring `make help`
# -----------------------------------------------------------------------------
add_custom_target(tp_help
    COMMAND ${CMAKE_COMMAND} -E echo "====================================================="
    COMMAND ${CMAKE_COMMAND} -E echo " tea-picking robot firmware - CMake targets"
    COMMAND ${CMAKE_COMMAND} -E echo "====================================================="
    COMMAND ${CMAKE_COMMAND} -E echo "  tea_picking_robot_host   host (PC) firmware binary"
    COMMAND ${CMAKE_COMMAND} -E echo "  tea_picking_robot_tests  host unit tests (when they exist)"
    COMMAND ${CMAKE_COMMAND} -E echo "  tea_picking_robot.elf    STM32F407VET6 image"
    COMMAND ${CMAKE_COMMAND} -E echo "  tp_sections              section headers of the ELF"
    COMMAND ${CMAKE_COMMAND} -E echo "  tp_symbols               symbol sizes of the ELF"
    COMMAND ${CMAKE_COMMAND} -E echo ""
    COMMAND ${CMAKE_COMMAND} -E echo " Configure with -DTP_BUILD_HOST=ON / -DTP_BUILD_STM32=ON"
    COMMENT "TP help")
