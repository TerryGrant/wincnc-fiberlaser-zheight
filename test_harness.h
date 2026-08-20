/*
 * test_harness.h
 *
 * Minimal assertion harness: no external dependencies, so the tests build
 * with the same bare toolchain as the controller itself.
 *
 * Tests do not abort on first failure -- every check reports, so one run
 * shows the full picture.
 */

#ifndef TEST_HARNESS_H
#define TEST_HARNESS_H

#include <stdio.h>

static int tests_run = 0;
static int tests_failed = 0;
static const char *current_test = "";

#define TEST(name)                                                            \
    static void name(void);                                                   \
    static void run_##name(void) {                                            \
        current_test = #name;                                                 \
        tests_run++;                                                          \
        name();                                                               \
    }                                                                         \
    static void name(void)

#define RUN(name) run_##name()

/* Report a failure against the currently running test. */
#define FAILF(fmt, ...)                                                       \
    do {                                                                      \
        tests_failed++;                                                       \
        printf("FAIL %s\n     " fmt "\n", current_test, __VA_ARGS__);          \
    } while (0)

#define CHECK_EQ(actual, expected)                                            \
    do {                                                                      \
        long _a = (long)(actual);                                              \
        long _e = (long)(expected);                                           \
        if (_a != _e) {                                                       \
            FAILF("%s: expected %ld, got %ld", #actual, _e, _a);               \
        }                                                                     \
    } while (0)

/* Same as CHECK_EQ but tags the failure with a caller-supplied label,
 * for assertions inside loops where the line alone is ambiguous. */
#define CHECK_EQ_AT(actual, expected, label, idx)                             \
    do {                                                                      \
        long _a = (long)(actual);                                             \
        long _e = (long)(expected);                                           \
        if (_a != _e) {                                                       \
            FAILF("%s[%s=%d]: expected %ld, got %ld",                         \
                  #actual, (label), (int)(idx), _e, _a);                      \
        }                                                                     \
    } while (0)

#define CHECK_TRUE(cond)                                                      \
    do {                                                                      \
        if (!(cond)) {                                                        \
            FAILF("expected true: %s", #cond);                                \
        }                                                                     \
    } while (0)

#define CHECK_LE(a, b)                                                        \
    do {                                                                      \
        long _x = (long)(a);                                                  \
        long _y = (long)(b);                                                  \
        if (!(_x <= _y)) {                                                    \
            FAILF("%s <= %s: %ld is not <= %ld", #a, #b, _x, _y);             \
        }                                                                     \
    } while (0)

static int test_summary(void)
{
    printf("\n%d checks failed across %d tests\n", tests_failed, tests_run);
    return tests_failed == 0 ? 0 : 1;
}

#endif /* TEST_HARNESS_H */
