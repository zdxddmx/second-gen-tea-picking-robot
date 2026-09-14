/**
 * @file    test_framework.h
 * @brief   Minimal, dependency-free unit-test framework for the C firmware.
 *
 * @details
 * # Why not Unity, CMocka or Ceedling
 * The firmware has to be testable on a machine that has a C compiler and
 * nothing else.  Those frameworks are good, but each one adds a dependency
 * (a vendored source tree, a build generator, sometimes a Python tool) that a
 * reviewer has to install before they can run a single test — and in the
 * environment this project was developed in there was **no network access at
 * all**, so a missing dependency could not be fetched.  A 200-line header that
 * compiles with `gcc` is worth more here than a better framework nobody can run.
 *
 * The rest of the suite is designed to be portable to any of them later: tests
 * are plain `void(void)` functions with no framework-specific types in their
 * signatures.
 *
 * # Automatic registration
 * Each test registers itself with a `__attribute__((constructor))` function, so
 * adding a test file requires **no edit to any list**.  Forgetting to register a
 * new test is a real failure mode with table-based runners — the test exists,
 * passes locally when run by hand, and silently never runs in CI.
 *
 * @note The constructor attribute is a GCC/Clang extension and is host-only.
 *       Tests are never linked into the target firmware, so this is not a
 *       portability problem for the product; it is documented because it is the
 *       one place where the test tree differs from the firmware's own rules.
 *
 * # What is asserted
 * Normal input, boundary input, invalid input and exceptional state — the four
 * categories the project's own test plan requires.  The assertion macros
 * deliberately print the *values* as well as the expression, because a failing
 * test that says only "expected true" costs the reader ten minutes.
 *
 * @copyright MIT License, see LICENSE.
 */

#ifndef TP_TEST_FRAMEWORK_H
#define TP_TEST_FRAMEWORK_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Maximum number of registered test cases. */
#define TP_TEST_MAX_CASES 256u

/** @brief Signature of a test case. */
typedef void (*tp_test_fn_t)(void);

/**
 * @brief One registered test.
 */
typedef struct {
    const char *suite;  /**< Group name, usually the module under test.      */
    const char *name;   /**< Test name.                                      */
    tp_test_fn_t fn;    /**< Body.                                           */
} tp_test_case_t;

/**
 * @brief Register a test case.
 *
 * Called automatically by @ref TP_TEST_CASE; calling it by hand is possible but
 * unnecessary.
 *
 * @param[in] suite  Group name.
 * @param[in] name   Test name.
 * @param[in] fn     Body.
 */
void tp_test_register(const char *suite, const char *name, tp_test_fn_t fn);

/**
 * @brief Record a failed expectation.
 *
 * @param[in] file      Source file.
 * @param[in] line      Line number.
 * @param[in] expr      The expression that failed.
 * @param[in] detail    Extra text, or an empty string.
 */
void tp_test_fail(const char *file, int line, const char *expr,
                  const char *detail);

/**
 * @brief Number of failures recorded in the current test.
 *
 * @return Failure count.
 */
uint32_t tp_test_failures(void);

/**
 * @brief Whether the current test has failed.
 *
 * @return Non-zero when at least one expectation failed.
 */
int tp_test_has_failed(void);

/**
 * @brief Declare a test case.
 *
 * @param[in] suite  Group name (an identifier).
 * @param[in] name   Test name (an identifier).
 *
 * @code
 *   TP_TEST_CASE(pid, saturates_at_output_limit)
 *   {
 *       ... expectations ...
 *   }
 * @endcode
 */
#define TP_TEST_CASE(suite, name)                                              \
    static void suite##_##name(void);                                          \
    static void suite##_##name##_register(void) __attribute__((constructor));  \
    static void suite##_##name##_register(void)                                \
    {                                                                          \
        tp_test_register(#suite, #name, suite##_##name);                       \
    }                                                                          \
    static void suite##_##name(void)

/* -------------------------------------------------------------------------
 * Expectations
 * ------------------------------------------------------------------------- */

/** @brief Fail unconditionally with a message. */
#define TP_FAIL(msg) tp_test_fail(__FILE__, __LINE__, "TP_FAIL", (msg))

/** @brief Assert that @p cond is true. */
#define TP_ASSERT(cond)                                                        \
    do {                                                                       \
        if (!(cond)) {                                                         \
            tp_test_fail(__FILE__, __LINE__, #cond, "");                       \
        }                                                                      \
    } while (0)

/** @brief Assert that @p cond is true, printing an extra message on failure. */
#define TP_ASSERT_MSG(cond, msg)                                               \
    do {                                                                       \
        if (!(cond)) {                                                         \
            tp_test_fail(__FILE__, __LINE__, #cond, (msg));                    \
        }                                                                      \
    } while (0)

/** @brief Assert that two integers are equal. */
#define TP_ASSERT_EQ(a, b)                                                     \
    do {                                                                       \
        long long ta_ = (long long)(a);                                        \
        long long tb_ = (long long)(b);                                        \
        if (ta_ != tb_) {                                                      \
            char buf_[160];                                                    \
            (void)snprintf(buf_, sizeof(buf_),                                 \
                           "%s = %lld, expected %s = %lld", #a, ta_, #b, tb_); \
            tp_test_fail(__FILE__, __LINE__, #a " == " #b, buf_);              \
        }                                                                      \
    } while (0)

/** @brief Assert that two integers are different. */
#define TP_ASSERT_NE(a, b)                                                     \
    do {                                                                       \
        if ((long long)(a) == (long long)(b)) {                                \
            char buf_[160];                                                    \
            (void)snprintf(buf_, sizeof(buf_), "both are %lld",                \
                           (long long)(a));                                    \
            tp_test_fail(__FILE__, __LINE__, #a " != " #b, buf_);              \
        }                                                                      \
    } while (0)

/** @brief Assert that @p a > @p b. */
#define TP_ASSERT_GT(a, b)                                                     \
    do {                                                                       \
        if (!((a) > (b))) {                                                    \
            char buf_[160];                                                    \
            (void)snprintf(buf_, sizeof(buf_), "%s = %.6g, %s = %.6g",         \
                           #a, (double)(a), #b, (double)(b));                  \
            tp_test_fail(__FILE__, __LINE__, #a " > " #b, buf_);               \
        }                                                                      \
    } while (0)

/** @brief Assert that @p a < @p b. */
#define TP_ASSERT_LT(a, b)                                                     \
    do {                                                                       \
        if (!((a) < (b))) {                                                    \
            char buf_[160];                                                    \
            (void)snprintf(buf_, sizeof(buf_), "%s = %.6g, %s = %.6g",         \
                           #a, (double)(a), #b, (double)(b));                  \
            tp_test_fail(__FILE__, __LINE__, #a " < " #b, buf_);               \
        }                                                                      \
    } while (0)

/**
 * @brief Assert that two floats are within @p tol of each other.
 *
 * Uses an absolute tolerance rather than a relative one: every quantity in this
 * firmware has a natural absolute scale (milliradians, newtons, per-mille), and
 * a relative comparison is actively misleading near zero, which is exactly where
 * a control loop spends most of its settled time.
 */
#define TP_ASSERT_NEAR(a, b, tol)                                              \
    do {                                                                       \
        double da_ = (double)(a);                                              \
        double db_ = (double)(b);                                              \
        double dt_ = (double)(tol);                                            \
        double diff_ = (da_ > db_) ? (da_ - db_) : (db_ - da_);                \
        if (!(diff_ <= dt_)) {                                                 \
            char buf_[200];                                                    \
            (void)snprintf(buf_, sizeof(buf_),                                 \
                           "%s = %.9g, %s = %.9g, |diff| = %.9g > %.9g",       \
                           #a, da_, #b, db_, diff_, dt_);                      \
            tp_test_fail(__FILE__, __LINE__, #a " ~= " #b, buf_);              \
        }                                                                      \
    } while (0)

/** @brief Assert that a float is finite (not NaN and not infinite). */
#define TP_ASSERT_FINITE(x)                                                    \
    do {                                                                       \
        double dx_ = (double)(x);                                              \
        if (!((dx_ == dx_) && (dx_ < 1.0e300) && (dx_ > -1.0e300))) {          \
            char buf_[120];                                                    \
            (void)snprintf(buf_, sizeof(buf_), "%s = %.9g is not finite",      \
                           #x, dx_);                                           \
            tp_test_fail(__FILE__, __LINE__, "isfinite(" #x ")", buf_);        \
        }                                                                      \
    } while (0)

/** @brief Assert that two NUL-terminated strings are equal. */
#define TP_ASSERT_STR_EQ(a, b)                                                 \
    do {                                                                       \
        const char *sa_ = (a);                                                 \
        const char *sb_ = (b);                                                 \
        if ((sa_ == NULL) || (sb_ == NULL) || (strcmp(sa_, sb_) != 0)) {       \
            char buf_[220];                                                    \
            (void)snprintf(buf_, sizeof(buf_), "\"%s\" vs \"%s\"",             \
                           (sa_ != NULL) ? sa_ : "(null)",                     \
                           (sb_ != NULL) ? sb_ : "(null)");                    \
            tp_test_fail(__FILE__, __LINE__, #a " == " #b, buf_);              \
        }                                                                      \
    } while (0)

/**
 * @brief Access the registered test cases (used by the runner).
 *
 * @param[out] count  Receives the number of registered cases.
 * @return Pointer to the registration table.
 */
const tp_test_case_t *tp_test_cases(uint32_t *count);

/**
 * @brief Reset the failure state before a test runs.
 *
 * @param[in] suite  Group name, for the failure prefix.
 * @param[in] name   Test name, for the failure prefix.
 */
void tp_test_begin(const char *suite, const char *name);

#ifdef __cplusplus
}
#endif

#endif /* TP_TEST_FRAMEWORK_H */
