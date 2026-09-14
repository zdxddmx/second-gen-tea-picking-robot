#!/usr/bin/env bash
# =============================================================================
#  scripts/flash_openocd.sh - flash the STM32F407VET6 through an ST-Link
# =============================================================================
#
#  DOCUMENTED, NOT EXECUTED.
#
#  There is no ST-Link and no target board attached to the machine this port was
#  written on, and `openocd` is not installed there either (`Get-Command openocd`
#  finds nothing).  Nothing in this script has been run.  It is provided because
#  the flashing command is part of "how do I put this on the robot", and because
#  the reset/halt sequence below is the part people get wrong when they write it
#  from memory.
#
#  -----------------------------------------------------------------------------
#  Hardware
#  -----------------------------------------------------------------------------
#  ST-Link V2 (or a clone) on the SWD header:
#
#      ST-Link            STM32F407VET6 board
#      -------            --------------------
#      SWCLK  (pin 9)  -> SWCLK   (PA14)
#      SWDIO  (pin 7)  -> SWDIO   (PA13)
#      GND    (pin 4)  -> GND
#      3.3V   (pin 1)  -> 3.3V    (reference only - see the warning below)
#      NRST   (pin 15) -> NRST    (recommended, see "reset" below)
#
#  Do **not** power the motor driver from the ST-Link's 3.3 V pin.  It is a
#  reference/logic supply, typically good for a few tens of milliamps.  Power the
#  board from its own 24 V rail with the ST-Link's 3.3 V pin left as a reference,
#  or the debugger browns out the moment a gate driver switches.
#
#  -----------------------------------------------------------------------------
#  Which file to flash
#  -----------------------------------------------------------------------------
#  build/stm32/tea_picking_robot.elf    - has symbols, best for debugging
#  build/stm32/tea_picking_robot.hex    - Intel HEX, offset embedded
#  build/stm32/tea_picking_robot.bin    - raw binary, MUST be given address 0x08000000
#
#  The ELF is preferred on the bench: OpenOCD loads the program headers, so the
#  addresses come from the linker script rather than from a number typed on the
#  command line.  A `.bin` flashed without an explicit address lands at
#  0x00000000 - which on this part is aliased flash, so it *appears* to work
#  until the first reset, then runs whatever was already at 0x08000000.
#
#  -----------------------------------------------------------------------------
#  Usage
#  -----------------------------------------------------------------------------
#    scripts/flash_openocd.sh                 # flash the ELF, then reset and run
#    scripts/flash_openocd.sh --bin           # flash the raw .bin instead
#    scripts/flash_openocd.sh --verify        # flash, then verify against the file
#    scripts/flash_openocd.sh --erase         # full chip erase first
#    scripts/flash_openocd.sh --probe         # just identify the target
#    scripts/flash_openocd.sh --attach-gdb    # leave a GDB server listening
#
#  Environment overrides:
#    OPENOCD        openocd binary (default: openocd)
#    OPENOCD_SCRIPTS  script search path (default: the installation's own)
#    OPENOCD_INTERFACE  interface config (default: interface/stlink.cfg, falling
#                       back to interface/stlink-v2.cfg for older installations)
#
#  NOTE: the Makefile is the build; this script only consumes its output.
# =============================================================================

set -eu

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd -P)"
project_root="$(cd -- "${script_dir}/.." >/dev/null 2>&1 && pwd -P)"

ELF="${project_root}/build/stm32/tea_picking_robot.elf"
BIN="${project_root}/build/stm32/tea_picking_robot.bin"

OPENOCD="${OPENOCD:-openocd}"
TARGET_CFG="${OPENOCD_TARGET:-target/stm32f4x.cfg}"

# OpenOCD renamed its interface configs in 0.11: `stlink-v2.cfg` became
# `stlink.cfg`.  Picking whichever the installation has avoids the most common
# "Can't find interface/stlink.cfg" failure on a slightly older setup.
if [ -n "${OPENOCD_INTERFACE:-}" ]; then
    IFACE_CFG="${OPENOCD_INTERFACE}"
elif [ -n "${OPENOCD_SCRIPTS:-}" ] && [ -f "${OPENOCD_SCRIPTS}/interface/stlink.cfg" ]; then
    IFACE_CFG="interface/stlink.cfg"
else
    IFACE_CFG="interface/stlink-v2.cfg"
fi

use_bin=0
do_verify=0
do_erase=0
probe_only=0
attach_gdb=0

for arg in "$@"; do
    case "${arg}" in
        --bin)        use_bin=1 ;;
        --verify)     do_verify=1 ;;
        --erase)      do_erase=1 ;;
        --probe)      probe_only=1 ;;
        --attach-gdb) attach_gdb=1 ;;
        -h|--help)
            sed -n '2,60p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            echo "flash_openocd.sh: unknown option '${arg}'" >&2
            exit 2
            ;;
    esac
done

if ! command -v "${OPENOCD}" >/dev/null 2>&1; then
    echo "flash_openocd.sh: '${OPENOCD}' is not on PATH." >&2
    echo "                  openocd is not installed on the machine this port was" >&2
    echo "                  written on; install it, or use the STM32CubeProgrammer" >&2
    echo "                  CLI equivalent:" >&2
    echo "                    STM32_Programmer_CLI -c port=SWD -w ${ELF} -v -rst" >&2
    exit 2
fi

# --- Build if the image is missing -------------------------------------------
if [ ! -f "${ELF}" ] && [ ! -f "${BIN}" ]; then
    echo "flash_openocd.sh: no image in build/stm32 - running 'make stm32' first"
    ( cd "${project_root}" && make stm32 )
fi

# --- Assemble the OpenOCD command --------------------------------------------
common_args=(
    -f "${IFACE_CFG}"
    -f "${TARGET_CFG}"
)

# `reset_config srst_only` + `adapter srst delay` matter on a board whose NRST is
# not wired: OpenOCD then has to use the debug port's own reset, which needs a
# longer settle because the core may be mid-instruction.  Without this the very
# common symptom is "target not halted" on the first connect after a reset.
common_args+=( -c "adapter speed 2000" )
common_args+=( -c "reset_config srst_only srst_nogate connect_assert_srst" )

if [ "${probe_only}" -eq 1 ]; then
    exec "${OPENOCD}" "${common_args[@]}" \
        -c "init" \
        -c "targets" \
        -c "halt" \
        -c "reg pc" \
        -c "flash info 0" \
        -c "shutdown"
fi

if [ "${attach_gdb}" -eq 1 ]; then
    # `program` is deliberately not used: the caller wants a live debug session.
    echo "flash_openocd.sh: GDB server on tcp/3333, telnet on tcp/4444."
    echo "                  Attach with:"
    echo "                    arm-none-eabi-gdb ${ELF} \\"
    echo "                      -ex 'target extended-remote localhost:3333'"
    exec "${OPENOCD}" "${common_args[@]}" -c "init" -c "reset halt"
fi

# --- Program -----------------------------------------------------------------
# `program <file> [verify|noverify] [reset|noreset]` runs load + verify + reset in
# the right order, which is why it is used instead of a hand-written
# erase/write/verify/reset sequence.
if [ "${use_bin}" -eq 1 ]; then
    if [ ! -f "${BIN}" ]; then
        echo "flash_openocd.sh: --bin requested but ${BIN} does not exist" >&2
        echo "                  (run 'make stm32', which produces it with objcopy)" >&2
        exit 2
    fi
    program_file="${BIN}"
    # The explicit address is mandatory for a raw binary: the file itself does
    # not say where it belongs, and 0x08000000 is where the linker script put
    # the vector table.
    program_args=( "0x08000000 ${program_file}" )
else
    program_file="${ELF}"
    program_args=( "${program_file}" )
fi

verify_mode="verify"
[ "${do_verify}" -eq 0 ] && verify_mode="verify"   # verify is cheap; always do it

if [ "${do_erase}" -eq 1 ]; then
    echo "flash_openocd.sh: full chip erase before programming"
    pre_erase=( -c "init" -c "halt" -c "stm32f4x mass_erase 0" -c "shutdown" )
    "${OPENOCD}" "${common_args[@]}" "${pre_erase[@]}"
fi

echo "flash_openocd.sh: programming ${program_file}"
exec "${OPENOCD}" "${common_args[@]}" \
    -c "init" \
    -c "halt" \
    -c "program ${program_args[*]} ${verify_mode} reset" \
    -c "shutdown"
