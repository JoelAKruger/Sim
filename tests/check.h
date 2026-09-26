#pragma once

#include <stdio.h>

#include "core/common.h"

// Minimal test support. Unlike assert, CHECK stays active in release builds.
static u32 check_failures;

#define CHECK(condition)                                                                  \
    do {                                                                                  \
        if (!(condition)) {                                                               \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
            check_failures++;                                                             \
        }                                                                                 \
    } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                                                   \
    do {                                                                                          \
        f64 check_actual = (f64)(actual);                                                         \
        f64 check_expected = (f64)(expected);                                                     \
        if (!(absolute(check_actual - check_expected) <= (f64)(tolerance))) {                     \
            fprintf(stderr, "%s:%d: CHECK_NEAR failed: %s = %.6g, expected %.6g +- %g\n",         \
                    __FILE__, __LINE__, #actual, check_actual, check_expected, (f64)(tolerance)); \
            check_failures++;                                                                     \
        }                                                                                         \
    } while (0)

static int report_checks(const char *suite)
{
    if (check_failures) {
        fprintf(stderr, "%s: %u check(s) failed\n", suite, check_failures);
        return 1;
    }
    printf("%s: ok\n", suite);
    return 0;
}
