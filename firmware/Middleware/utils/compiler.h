/**
 * @file    compiler.h
 * @brief   Toolchain abstraction: attributes, static assertions, size helpers.
 *
 * @details
 * The firmware is compiled by three different toolchains in this project:
 *
 *   - `gcc`               -> native host build used by the unit tests
 *   - `arm-none-eabi-gcc` -> STM32F407VET6 target build
 *   - `cl` (optional)     -> not supported, kept for reference only
 *
 * This header hides the small dialect differences so that no other source file
 * has to know which compiler is in use.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_COMPILER_H
#define TP_COMPILER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Static assertion
 * -------------------------------------------------------------------------- */

#if defined(__cplusplus) && (__cplusplus >= 201103L)
#define TP_STATIC_ASSERT(cond, msg) static_assert((cond), msg)
#elif defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
#define TP_STATIC_ASSERT(cond, msg) _Static_assert((cond), msg)
#else
/**
 * @brief Compile-time assertion for pre-C11 compilers.
 *
 * Declares an array whose length is 1 when @p cond holds and -1 otherwise,
 * which turns a violated invariant into a compile error rather than a silent
 * runtime surprise.
 */
#define TP_STATIC_ASSERT(cond, msg) \
    typedef char tp_static_assert_##__LINE__[(cond) ? 1 : -1]
#endif

/* --------------------------------------------------------------------------
 * Attributes
 * -------------------------------------------------------------------------- */

/** @brief Mark a function or variable as possibly unused. */
#define TP_UNUSED       __attribute__((unused))

/** @brief Force inlining of a very small, timing-critical helper. */
#define TP_ALWAYS_INLINE __attribute__((always_inline)) static inline

/** @brief Place a symbol in a specific linker section. */
#define TP_SECTION(name) __attribute__((section(name)))

/** @brief Hint the compiler that the function never returns. */
#define TP_NORETURN     __attribute__((noreturn))

/** @brief Align a symbol. */
#define TP_ALIGN(n)     __attribute__((aligned(n)))

/** @brief Weak symbol, overridable by a target-specific implementation. */
#define TP_WEAK         __attribute__((weak))

/** @brief Request a compile-time check that @p fmt matches the varargs. */
#define TP_PRINTF_LIKE(fmt_idx, arg_idx) \
    __attribute__((format(printf, fmt_idx, arg_idx)))

/* --------------------------------------------------------------------------
 * Small helpers
 * -------------------------------------------------------------------------- */

/** @brief Number of elements of a statically sized array. */
#define TP_ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/** @brief Minimum of two values, evaluated once each. */
#define TP_MIN(a, b) (((a) < (b)) ? (a) : (b))

/** @brief Maximum of two values, evaluated once each. */
#define TP_MAX(a, b) (((a) > (b)) ? (a) : (b))

/** @brief Clamp @p v into the inclusive range [lo, hi]. */
#define TP_CLAMP(v, lo, hi) (((v) < (lo)) ? (lo) : (((v) > (hi)) ? (hi) : (v)))

/** @brief Absolute value of a float expression, without a library call. */
#define TP_FABS(x) (((x) < 0.0f) ? -(x) : (x))

/** @brief Sign of a float expression as -1.0f, 0.0f or +1.0f. */
#define TP_FSIGN(x) (((x) > 0.0f) ? 1.0f : (((x) < 0.0f) ? -1.0f : 0.0f))

/** @brief Dead-band: map values inside +-@p band to zero. */
#define TP_DEADBAND(x, band) (((x) > (band) || (x) < -(band)) ? (x) : 0.0f)

/**
 * @brief Detect a non-finite float without linking against libm.
 *
 * IEEE-754 NaN is the only value that is not equal to itself; a float is
 * infinite when its exponent bits are all ones.  Keeping this inline avoids a
 * `isnan()` call in the 1 kHz control loop.
 */
#define TP_ISNAN(x) ((x) != (x))
#define TP_ISINF(x) (!TP_ISNAN(x) && ((x) > 3.402823466e38f || (x) < -3.402823466e38f))
#define TP_ISFINITE(x) (!TP_ISNAN(x) && !TP_ISINF(x))

/* --------------------------------------------------------------------------
 * Target-specific helpers
 * -------------------------------------------------------------------------- */

#if defined(__arm__) || defined(__thumb__)
/** @brief Busy-wait for a fixed number of CPU cycles (Cortex-M only). */
#define TP_NOP() __asm volatile("nop")
/** @brief Trigger a debug breakpoint (Cortex-M only). */
#define TP_BREAKPOINT() __asm volatile("bkpt #0")
#else
#define TP_NOP() ((void)0)
#define TP_BREAKPOINT() ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* TP_COMPILER_H */
