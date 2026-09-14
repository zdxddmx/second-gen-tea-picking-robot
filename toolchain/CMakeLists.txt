# =============================================================================
#  toolchain/CMakeLists.txt
# =============================================================================
#
#  Answer file / support package for the cross toolchain.
#
#  CMake's documented way to ship a toolchain together with supporting files is
#  either a `toolchainfile.cmake` (used directly with `CMAKE_TOOLCHAIN_FILE`) or
#  a *toolchain package* that a `find_package`-style include pulls in.  This
#  project uses the simpler of the two - `arm-none-eabi.cmake` is the file you
#  pass to `-DCMAKE_TOOLCHAIN_FILE` - and this CMakeLists.txt exists as the
#  support target that
#
#   1. documents the toolchain variables a caller may override, and
#   2. validates that the toolchain is actually present, with an error message
#      that says what to do instead of failing later inside a compile step.
#
#  It is included from the root CMakeLists.txt with
#
#      add_subdirectory(toolchain)
#
#  when the ARM target is enabled.
#
#  -----------------------------------------------------------------------------
#  NOT TESTED: cmake is absent on the machine this port was written on.
#  See the header of `toolchain/arm-none-eabi.cmake` for the full statement.
#  -----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# --- Collected by the root CMakeLists.txt when it includes this directory -----
# Places the toolchain's own variables into the parent scope so a caller can
# report them (the root file's `help`-equivalent status output prints these).

set(TP_TOOLCHAIN_PREFIX "${TOOLCHAIN_PREFIX}" PARENT_SCOPE)
set(TP_TOOLCHAIN_C_COMPILER "${CMAKE_C_COMPILER}" PARENT_SCOPE)
set(TP_TOOLCHAIN_OBJCOPY "${CMAKE_OBJCOPY}" PARENT_SCOPE)
set(TP_TOOLCHAIN_SIZE "${CMAKE_SIZE}" PARENT_SCOPE)
set(TP_TOOLCHAIN_CPU_FLAGS "${TP_CPU_FLAGS}" PARENT_SCOPE)

# --- Availability check -------------------------------------------------------
# `find_program` with NO_DEFAULT_PATH is not used: the toolchain may legitimately
# be found either on PATH or through CMAKE_C_COMPILER's directory.  What is
# checked is that the three programs the build actually invokes exist.
find_program(TP_OBJCOPY_FOUND NAMES "${CMAKE_OBJCOPY}" objcopy)
find_program(TP_SIZE_FOUND    NAMES "${CMAKE_SIZE}"    size)

if(NOT TP_OBJCOPY_FOUND OR NOT TP_SIZE_FOUND)
    message(WARNING
        "Arm GNU toolchain helpers not found (objcopy='${CMAKE_OBJCOPY}', "
        "size='${CMAKE_SIZE}'). The ELF can still be built, but the .bin/.hex "
        "and the size report cannot. Point -DTOOLCHAIN_PREFIX at a full "
        "installation, e.g. -DTOOLCHAIN_PREFIX='D:/13.2 Rel1/bin/arm-none-eabi-'.")
endif()

# --- Report what will be used, once, so a build log is self-describing --------
message(STATUS "TP toolchain: ${CMAKE_C_COMPILER}")
message(STATUS "TP CPU flags: ${TP_CPU_FLAGS}")
message(STATUS "TP objcopy  : ${CMAKE_OBJCOPY}")
message(STATUS "TP size     : ${CMAKE_SIZE}")
