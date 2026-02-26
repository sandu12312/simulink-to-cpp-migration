/*
 * kalman_filter.h
 * Legacy 1D Kalman Filter - C style
 * Estimates true value from noisy sensor measurements
 * Used in ADAS for speed/distance estimation
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

typedef struct {
    double x;   /* estimated state (e.g. speed) */
    double P;   /* estimation error covariance */
    double Q;   /* process noise covariance */
    double R;   /* measurement noise covariance */
} Kalman_State;

/* extern "C" so C++ code can link against this C library without name mangling */
#ifdef __cplusplus
extern "C" {
#endif

void   Kalman_Init(Kalman_State* state, double initial_x, double initial_P,
                   double Q, double R);
double Kalman_Update(Kalman_State* state, double measurement);

#ifdef __cplusplus
}
#endif

#endif
