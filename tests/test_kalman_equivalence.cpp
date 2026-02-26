/*
 * test_kalman_equivalence.cpp
 * Same idea: run legacy C Kalman and C++ Kalman on same inputs,
 * verify outputs match.
 */

#include <cstdio>
#include <cmath>
#include "../legacy/kalman_filter.h"
#include "../include/kalman/KalmanFilter.hpp"

static const double EPS = 1e-10;

int main()
{
    /* noisy speed measurements (simulated sensor data) */
    double measurements[] = {98.5, 101.2, 99.8, 102.1, 100.3,
                              98.9, 101.7, 100.1, 99.5, 100.8};
    int n = sizeof(measurements) / sizeof(measurements[0]);

    /* Legacy C Kalman */
    Kalman_State legacy;
    Kalman_Init(&legacy, 100.0, 1.0, 0.1, 1.0);

    /* Modern C++ Kalman */
    KalmanFilter modern(100.0, 1.0, 0.1, 1.0);

    printf("Step | Measurement | Legacy est. | C++ est.    | Diff\n");
    printf("-----|-------------|-------------|-------------|----------\n");

    int passed = 0, failed = 0;

    for (int i = 0; i < n; i++) {
        double out_legacy = Kalman_Update(&legacy, measurements[i]);
        double out_modern = modern.update(measurements[i]);
        double diff       = fabs(out_legacy - out_modern);

        printf("%4d | %11.2f | %11.6f | %11.6f | %.2e\n",
               i, measurements[i], out_legacy, out_modern, diff);

        if (diff < EPS) passed++;
        else {
            printf("FAIL at step %d\n", i);
            failed++;
        }
    }

    printf("\nResults: %d passed, %d failed\n", passed, failed);

    if (failed == 0) {
        printf("PASS — Kalman migration is backwards compatible!\n");
        return 0;
    }
    return 1;
}
