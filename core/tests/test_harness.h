#pragma once

/* Minimal, dependency-free test harness: no vendored Unity/GoogleTest, just
 * enough to assert and report a pass/fail count. Keeps core/ buildable with
 * nothing but a C compiler. */

#include <math.h>
#include <stdio.h>

static int g_fc_test_failures = 0;
static int g_fc_test_count = 0;

#define FC_CHECK(cond)                                                              \
    do {                                                                            \
        g_fc_test_count++;                                                          \
        if (!(cond)) {                                                              \
            g_fc_test_failures++;                                                   \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
        }                                                                           \
    } while (0)

#define FC_CHECK_NEAR(a, b, tol)                                                    \
    do {                                                                            \
        g_fc_test_count++;                                                         \
        double _a = (double)(a), _b = (double)(b), _t = (double)(tol);              \
        if (fabs(_a - _b) > _t) {                                                   \
            g_fc_test_failures++;                                                   \
            fprintf(stderr, "FAIL %s:%d: %s ~= %s (%.6f vs %.6f, tol %.6f)\n",       \
                    __FILE__, __LINE__, #a, #b, _a, _b, _t);                        \
        }                                                                           \
    } while (0)

#define FC_TEST_MAIN_BEGIN() int main(void) {
#define FC_TEST_MAIN_END()                                                          \
    fprintf(stderr, "%d checks, %d failed\n", g_fc_test_count, g_fc_test_failures); \
    return g_fc_test_failures == 0 ? 0 : 1;                                         \
    }
