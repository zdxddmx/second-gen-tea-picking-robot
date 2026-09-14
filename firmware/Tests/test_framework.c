/**
 * @file    test_framework.c
 * @brief   Registration, failure accounting and the test runner's main().
 * @copyright MIT License, see LICENSE.
 */

#include "test_framework.h"

/* `qsort()` is used by the runner to order the registered cases by name, and it
 * is declared in <stdlib.h>.  Without this include the call is an implicit
 * declaration, which GCC 14 and later reject outright (it is an error, not a
 * warning, since C23 removed the implicit-int rule) - so the test binary could
 * not be built at all.  Added by the STM32 port work; the fix is the include,
 * not a cast. */
#include <stdlib.h>

static tp_test_case_t s_cases[TP_TEST_MAX_CASES];
static uint32_t       s_case_count;
static uint32_t       s_failures;
static uint32_t       s_total_failures;
static const char    *s_suite = "?";
static const char    *s_name = "?";

void tp_test_register(const char *suite, const char *name, tp_test_fn_t fn)
{
    if ((s_case_count >= TP_TEST_MAX_CASES) || (fn == NULL)) {
        return;
    }
    s_cases[s_case_count].suite = suite;
    s_cases[s_case_count].name = name;
    s_cases[s_case_count].fn = fn;
    s_case_count++;
}

const tp_test_case_t *tp_test_cases(uint32_t *count)
{
    if (count != NULL) {
        *count = s_case_count;
    }
    return s_cases;
}

void tp_test_begin(const char *suite, const char *name)
{
    s_suite = suite;
    s_name = name;
    s_failures = 0u;
}

void tp_test_fail(const char *file, int line, const char *expr,
                  const char *detail)
{
    s_failures++;
    s_total_failures++;
    (void)fprintf(stdout, "    FAIL %s.%s\n      %s:%d: %s%s%s\n",
                  s_suite, s_name, (file != NULL) ? file : "?", line,
                  (expr != NULL) ? expr : "?",
                  ((detail != NULL) && (detail[0] != '\0')) ? "  --  " : "",
                  ((detail != NULL) && (detail[0] != '\0')) ? detail : "");
}

uint32_t tp_test_failures(void)
{
    return s_failures;
}

int tp_test_has_failed(void)
{
    return (s_failures > 0u) ? 1 : 0;
}

/**
 * @brief Compare function used to sort the registration table.
 *
 * Tests run in suite-then-name order rather than in link order, so the output of
 * two builds can be diffed even if the object files are linked in a different
 * order.  Reproducible ordering in a test log is worth the twenty lines.
 *
 * @param[in] a  Left case.
 * @param[in] b  Right case.
 * @return Negative, zero or positive, as for `qsort`.
 */
static int test_compare(const void *a, const void *b)
{
    const tp_test_case_t *ca = (const tp_test_case_t *)a;
    const tp_test_case_t *cb = (const tp_test_case_t *)b;
    int r = strcmp(ca->suite, cb->suite);

    if (r != 0) {
        return r;
    }
    return strcmp(ca->name, cb->name);
}

/**
 * @brief Program entry point: run every registered test and report.
 *
 * @param[in] argc  Argument count.
 * @param[in] argv  Arguments; `--suite NAME` runs one group, `--list` prints the
 *                  registered cases without running them.
 * @return 0 when every test passed, 1 otherwise.
 */
int main(int argc, char **argv)
{
    uint32_t i;
    uint32_t passed = 0u;
    uint32_t failed = 0u;
    const char *only_suite = NULL;
    int list_only = 0;
    const char *current_suite = NULL;

    for (i = 1u; i < (uint32_t)argc; ++i) {
        if ((strcmp(argv[i], "--suite") == 0) && ((i + 1u) < (uint32_t)argc)) {
            only_suite = argv[++i];
        } else if (strcmp(argv[i], "--list") == 0) {
            list_only = 1;
        } else {
            (void)fprintf(stdout, "unknown argument: %s\n", argv[i]);
            return 2;
        }
    }

    qsort(s_cases, s_case_count, sizeof(s_cases[0]), test_compare);

    (void)printf("==============================================================\n");
    (void)printf(" tea-picking robot firmware - C unit tests\n");
    (void)printf(" %lu test case(s) registered\n", (unsigned long)s_case_count);
    if (only_suite != NULL) {
        (void)printf(" filter: suite '%s'\n", only_suite);
    }
    (void)printf("==============================================================\n");

    if (list_only != 0) {
        for (i = 0u; i < s_case_count; ++i) {
            (void)printf("  %s.%s\n", s_cases[i].suite, s_cases[i].name);
        }
        return 0;
    }

    for (i = 0u; i < s_case_count; ++i) {
        if ((only_suite != NULL) &&
            (strcmp(s_cases[i].suite, only_suite) != 0)) {
            continue;
        }
        if ((current_suite == NULL) ||
            (strcmp(current_suite, s_cases[i].suite) != 0)) {
            current_suite = s_cases[i].suite;
            (void)printf("\n[%s]\n", current_suite);
        }
        tp_test_begin(s_cases[i].suite, s_cases[i].name);
        s_cases[i].fn();
        if (tp_test_has_failed() != 0) {
            failed++;
        } else {
            passed++;
            (void)printf("    ok   %s\n", s_cases[i].name);
        }
    }

    (void)printf("\n==============================================================\n");
    (void)printf(" tests run    : %lu\n", (unsigned long)(passed + failed));
    (void)printf(" passed       : %lu\n", (unsigned long)passed);
    (void)printf(" failed       : %lu\n", (unsigned long)failed);
    (void)printf(" expectations : %lu failed\n", (unsigned long)s_total_failures);
    (void)printf(" RESULT: %s\n", (failed == 0u) ? "PASS" : "FAIL");
    (void)printf("==============================================================\n");
    return (failed == 0u) ? 0 : 1;
}
