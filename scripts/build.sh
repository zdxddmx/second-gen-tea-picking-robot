#!/usr/bin/env bash
# =============================================================================
#  scripts/build.sh - thin wrapper around the root Makefile
# =============================================================================
#
#  This script deliberately contains **no build logic of its own**.  It resolves
#  the project root, forwards its arguments to `make`, and translates the
#  `make`-specific target names into the friendlier positional ones a CI job or a
#  colleague will reach for.  Anything it did beyond that would be a second
#  place where the build is defined, and the second place is always the one that
#  goes stale.
#
#  Usage
#  -----
#    scripts/build.sh                 # same as: make host
#    scripts/build.sh host            # host firmware binary + unit tests
#    scripts/build.sh test            # build and run the host unit tests
#    scripts/build.sh stm32           # STM32F407VET6 ELF + .bin + .hex (bare metal)
#    scripts/build.sh freertos        # make stm32 RTOS=freertos
#    scripts/build.sh size            # size of the STM32 image
#    scripts/build.sh clean
#    scripts/build.sh help
#    scripts/build.sh all             # clean, host, test, stm32
#    scripts/build.sh -- <make args>  # everything after `--` goes to make verbatim
#
#  Exit status is `make`'s, so this is safe to use as a CI step.
#
#  NOTE ON THE SHELL: this file is a POSIX shell script.  On the machine this
#  port was developed on, `make` itself falls back to `cmd.exe` because `sh.exe`
#  is not on PATH - which is why the *Makefile* uses cmd.exe recipes.  This
#  wrapper needs a real `bash`, and `scripts/build.ps1` exists for the case where
#  there is not one.  Neither the Makefile nor the build depends on either
#  script.
# =============================================================================

set -eu

# --- Locate the project root (the directory containing the Makefile) ----------
#
# Deliberately builtins only (`cd`, `pwd`, `printf`, `read`).  Calling `dirname`
# looks harmless and is not: on a Windows host where this script is run through
# an MSYS2 `bash.exe` started from a non-MSYS shell, `/usr/bin` is not on PATH and
# the very first line fails with
#     scripts/build.sh: line 39: dirname: command not found
# The `${BASH_SOURCE[0]%/*}` expansion does the same job with no external process.
self="${BASH_SOURCE[0]}"
case "${self}" in
    */*) script_dir="$(cd -- "${self%/*}" >/dev/null 2>&1 && pwd -P)" ;;
    *)   script_dir="$(pwd -P)" ;;
esac
project_root="$(cd -- "${script_dir}/.." >/dev/null 2>&1 && pwd -P)"

if [ ! -f "${project_root}/Makefile" ]; then
    printf 'build.sh: no Makefile in %s\n' "${project_root}" >&2
    exit 2
fi

cd "${project_root}"

# --- Which make? --------------------------------------------------------------
MAKE="${MAKE:-make}"
if ! command -v "${MAKE}" >/dev/null 2>&1; then
    printf "build.sh: '%s' not found on PATH. Set MAKE=/path/to/make.\n" "${MAKE}" >&2
    exit 2
fi

usage() {
    printf '%s\n' \
        'Usage: scripts/build.sh [action] [-- make-args...]' \
        '' \
        '  (no action)   same as: make host' \
        '  host          host (PC) firmware binary + unit tests' \
        '  test          build and run the host unit tests' \
        '  stm32         STM32F407VET6 ELF + .bin + .hex (bare metal)' \
        '  freertos      make stm32 RTOS=freertos' \
        '  size          size of the STM32 image' \
        '  sections      section headers of the STM32 ELF' \
        '  symbols       symbol sizes of the STM32 ELF' \
        '  clean         remove build/' \
        '  distclean     clean, plus anything else generated' \
        '  all           clean, host, test, stm32' \
        '  help          this text' \
        '  --            everything after this goes to make verbatim' \
        '' \
        'Exit status is make'"'"'s, so this is safe as a CI step.'
}

run_make() {
    printf '==> %s %s\n' "${MAKE}" "$*"
    "${MAKE}" "$@"
}

action="${1:-host}"
[ $# -gt 0 ] && shift

case "${action}" in
    host)
        run_make host
        ;;
    test)
        run_make test
        ;;
    stm32|target|firmware)
        run_make stm32
        ;;
    freertos|rtos)
        run_make stm32 RTOS=freertos
        ;;
    size)
        run_make size
        ;;
    sections)
        run_make sections
        ;;
    symbols)
        run_make symbols
        ;;
    clean)
        run_make clean
        ;;
    distclean)
        run_make distclean
        ;;
    help|-h|--help)
        usage
        ;;
    all)
        # Order matters: clean first so a stale object can never make a later
        # step look green.
        run_make clean
        run_make host
        run_make test
        run_make stm32
        ;;
    --)
        run_make "$@"
        ;;
    *)
        echo "build.sh: unknown action '${action}'" >&2
        echo >&2
        usage >&2
        exit 2
        ;;
esac

echo "==> ${action}: OK"
