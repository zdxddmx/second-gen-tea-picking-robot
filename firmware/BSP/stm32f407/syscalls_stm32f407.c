/**
 * @file    syscalls_stm32f407.c
 * @brief   Newlib system-call stubs backed by the debug UART and the linker
 *          script's heap.
 *
 * @details
 * `arm-none-eabi-gcc` links against newlib, whose `printf`, `malloc` and
 * `exit` reach for a handful of POSIX-shaped symbols that a bare-metal target
 * has to provide.  There are three ways to satisfy them:
 *
 *   1. `-lnosys` — link the "no operating system" stubs.  Every call returns an
 *      error, `malloc` always fails and `printf` goes nowhere.  Cheap, and
 *      useless the moment anything needs to allocate or print.
 *   2. A vendor `syscalls.c` — none is available in the vendored tree.
 *   3. Implement them, which is what this file does, because two of them are
 *      genuinely useful on this board:
 *      - `_write()` routes `printf` output to USART1, so a `printf` that
 *        somebody adds while debugging appears on the console instead of
 *        failing silently;
 *      - `_sbrk()` hands out the linker script's `._user_heap_stack`
 *        reservation, bounded by `_Heap_Limit` so the heap cannot grow into the
 *        stack.  Without that bound, a heap-heavy change would corrupt the
 *        stack and present as a random hard fault under load — the single worst
 *        class of bug to debug on a robot.
 *
 * Note that the firmware itself does **not** use `printf` or `malloc`: logging
 * goes through `tp_format.c` (which is deliberately allocation-free) and the
 * application uses static storage.  These functions exist so that the library
 * cannot be the reason something breaks, and so that the safety properties above
 * are explicit rather than inherited from `-lnosys`.
 *
 * @copyright MIT License, see LICENSE.
 */

#include "config.h"

#if defined(TP_PLATFORM_STM32F407)

#include <errno.h>
#include <stddef.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "stm32f4xx_hal.h"

#include "platform_stm32f407.h"

/* =========================================================================
 * Heap
 * ========================================================================= */

/** @brief First byte after `.bss`; provided by the linker script. */
extern char _end[];
/** @brief Upper bound of the heap: `_estack - _Min_Stack_Size`. */
extern char _Heap_Limit[];

/** @brief Current program break. */
static char *s_heap_ptr;

/**
 * @brief Extend the heap.
 *
 * @param[in] incr  Bytes requested, rounded up to an 8-byte boundary so every
 *                  returned pointer is correctly aligned for a `double` and for
 *                  the 8-byte alignment AAPCS requires at a public interface.
 * @return Previous break, or `(void *)-1` with `errno = ENOMEM`.
 */
void *_sbrk(ptrdiff_t incr)
{
    char *previous;
    char *next;
    uintptr_t rounded;

    if (s_heap_ptr == NULL) {
        s_heap_ptr = _end;
        /* Align the start so the first allocation is aligned too. */
        rounded = ((uintptr_t)s_heap_ptr + 7u) & ~(uintptr_t)7u;
        s_heap_ptr = (char *)rounded;
    }

    if (incr == 0) {
        return s_heap_ptr;
    }
    if (incr < 0) {
        /* Shrinking is not supported: newlib's `free` never shrinks the break,
         * so a call like this can only be a bug. */
        errno = EINVAL;
        return (void *)-1;
    }

    previous = s_heap_ptr;
    rounded = ((uintptr_t)previous + (uintptr_t)incr + 7u) & ~(uintptr_t)7u;
    next = (char *)rounded;

    /* Refuse rather than collide with the stack.  This is the check that turns a
     * silent memory-corruption bug into a clean, diagnosable allocation failure
     * that propagates back to the caller as a NULL pointer. */
    if (next > _Heap_Limit) {
        errno = ENOMEM;
        return (void *)-1;
    }

    s_heap_ptr = next;
    return previous;
}

/* =========================================================================
 * Character output
 * ========================================================================= */

/**
 * @brief Write to a file descriptor.
 *
 * Only stdout (1) and stderr (2) are meaningful, and both go to the debug UART
 * through the polling writer: `printf` is a debugging aid, and in that role it
 * must work before the platform is initialised and inside a fault handler.
 *
 * @param[in] file  Descriptor.
 * @param[in] ptr   Bytes.
 * @param[in] len   Count.
 * @return Bytes written, or -1.
 */
int _write(int file, char *ptr, int len)
{
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    }
    if (len <= 0) {
        return 0;
    }
    tp_stm32_debug_write_blocking(ptr, (uint32_t)len);
    return len;
}

/**
 * @brief Read from a file descriptor.
 *
 * There is no console input path in this design: commands arrive over the framing
 * protocol, never over a POSIX descriptor.  Returning 0 (end of file) is the
 * honest answer and cannot loop forever.
 *
 * @return 0.
 */
int _read(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    (void)len;
    return 0;
}

/** @brief Close a descriptor.  Always succeeds; nothing is open. */
int _close(int file)
{
    (void)file;
    return -1;
}

/** @brief Seek.  Not meaningful on a console; always fails. */
off_t _lseek(int file, off_t ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    errno = ESPIPE;
    return (off_t)-1;
}

/**
 * @brief `fstat`, which newlib's buffered stdio calls during initialisation.
 *
 * Reporting a character device is what makes `stdout` line-buffered rather than
 * block-buffered, so a `printf` appears immediately instead of waiting for 1 KiB
 * that a robot will never produce.
 *
 * @return 0 on success.
 */
int _fstat(int file, struct stat *st)
{
    (void)file;
    if (st == NULL) {
        errno = EFAULT;
        return -1;
    }
    st->st_mode = S_IFCHR;
    return 0;
}

/** @brief Whether a descriptor is a terminal.  Yes, in the sense above. */
int _isatty(int file)
{
    (void)file;
    return 1;
}

/**
 * @brief Process exit.
 *
 * There is no process to exit and nowhere to return to: reaching this means the
 * application called `exit()` or `abort()`, which is always a bug on a bare-metal
 * robot with live actuators.  Kill the drives and reset.
 */
void _exit(int status)
{
    (void)status;

    tp_platform_emergency_kill();
    tp_stm32_debug_puts_blocking("\r\nFATAL: _exit() called, resetting\r\n");

    __disable_irq();
    NVIC_SystemReset();
    for (;;) {
    }
}

/** @brief Kill the process.  Deferred to @ref _exit. */
void _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    _exit(1);
}

/** @brief Pretend to have a process id; some newlib paths request one. */
int _getpid(void)
{
    return 1;
}

#endif /* TP_PLATFORM_STM32F407 */
