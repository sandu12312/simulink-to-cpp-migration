/*
 * test_pid_equivalence.cpp
 *
 * Backwards compatibility test:
 * Run legacy C PID and new C++ PID with the same inputs,
 * check that outputs match within floating-point epsilon.
 *
 * If this test passes, the migration is safe.
 */

#include <cstdio>
#include <cmath>
#include "../legacy/pid_controller.h"
#include "../include/pid/PidController.hpp"

static const double KP  = 1.2;
static const double KI  = 0.5;
static const double KD  = 0.1;
static const double DT  = 0.01;      /* 10ms step, typical ADAS ECU */
static const double EPS = 1e-10;     /* acceptable floating-point diff */

int main()
{
    /* Setup legacy C PID */
    PID_State legacy;
    PID_Init(&legacy);

    /* Setup modern C++ PID */
    PidController modern(KP, KI, KD, DT);

    /* Simulate 100 control steps (1 second at 10ms) */
    double setpoint = 100.0;   /* target speed km/h */
    double measured = 80.0;    /* current speed */

    int passed = 0;
    int failed = 0;

    printf("Step | Legacy output | C++ output    | Diff\n");
    printf("-----|---------------|---------------|----------\n");

    for (int i = 0; i < 100; i++) {
        double out_legacy = PID_Step(&legacy, setpoint, measured, KP, KI, KD, DT);
        double out_modern = modern.step(setpoint, measured);
        double diff       = fabs(out_legacy - out_modern);

        if (i < 5) {  /* print first 5 rows */
            printf("%4d | %13.6f | %13.6f | %.2e\n",
                   i, out_legacy, out_modern, diff);
        }

        if (diff < EPS) {
            passed++;
        } else {
            printf("FAIL at step %d: legacy=%.10f cpp=%.10f diff=%.2e\n",
                   i, out_legacy, out_modern, diff);
            failed++;
        }

        /* simulate plant response: speed moves toward target */
        measured += out_legacy * 0.01;
    }

    printf("...\n\n");
    printf("Results: %d passed, %d failed\n", passed, failed);

    if (failed == 0) {
        printf("PASS — C++ output matches legacy C within epsilon %.0e\n", EPS);
        printf("Migration is backwards compatible!\n");
        return 0;
    } else {
        printf("FAIL — outputs differ, migration has a bug\n");
        return 1;
    }
}
