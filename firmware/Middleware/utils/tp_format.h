/**
 * @file    tp_format.h
 * @brief   Tiny, dependency-free `snprintf` replacement for embedded use.
 *
 * @details
 * Why not use the C library `snprintf`?
 *
 *   - On Cortex-M, newlib's full `snprintf` pulls in roughly 6-10 kB of code
 *     and needs a heap for the `FILE` machinery used by the float paths.
 *     That is real flash and real RAM on an STM32F407 with 512 kB / 192 kB.
 *   - `newlib-nano` links a *stub* float formatter unless the application
 *     explicitly links the float variant, which silently prints `%f` as an
 *     empty string.  Debug logs that lie are worse than no logs.
 *   - A single, small, unit-tested formatter behaves identically on the host
 *     and on the target, so a log line reproduced in the PC simulator is
 *     byte-identical to the one produced by the firmware.
 *
 * Supported conversions: `%d %i %u %x %X %c %s %f %%`, with the `l`
 * (long) length modifier, `-` left-justify, `0` zero-pad and a decimal
 * width / precision.  Anything else is copied through verbatim rather than
 * crashing, which keeps a bad log call from taking the robot down.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_FORMAT_H
#define TP_FORMAT_H

#include <stdarg.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Format a string into a caller-supplied buffer.
 *
 * @param[out] dst     Destination buffer; always NUL-terminated on return
 *                     (unless @p dst is NULL or @p size is 0).
 * @param[in]  size    Capacity of @p dst in bytes, including the terminator.
 * @param[in]  fmt     Format string, see the file header for the subset that
 *                     is understood.
 * @param[in]  args    Argument list.
 * @return The number of characters that *would* have been written, excluding
 *         the terminator (i.e. `snprintf` semantics).  The caller can compare
 *         this against @p size to detect truncation.
 */
int tp_vsnprintf(char *dst, size_t size, const char *fmt, va_list args);

/**
 * @brief Variadic convenience wrapper around @ref tp_vsnprintf.
 *
 * @param[out] dst   Destination buffer.
 * @param[in]  size  Capacity of @p dst in bytes.
 * @param[in]  fmt   Format string.
 * @param[in]  ...   Arguments.
 * @return Characters that would have been written, excluding the terminator.
 */
int tp_snprintf(char *dst, size_t size, const char *fmt, ...);

/**
 * @brief Render a float with a fixed number of decimals.
 *
 * Exposed separately so the control and telemetry modules can produce a
 * deterministic textual representation without going through the full
 * formatter (useful for CSV output on the target).
 *
 * @param[out] dst     Destination buffer.
 * @param[in]  size    Capacity of @p dst.
 * @param[in]  value   Value to render.
 * @param[in]  decimals Number of fractional digits, clamped to 0..6.
 * @return Number of characters written, excluding the terminator.
 */
int tp_format_float(char *dst, size_t size, float value, int decimals);

#ifdef __cplusplus
}
#endif

#endif /* TP_FORMAT_H */
