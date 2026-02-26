/*
 * kalman_filter.c
 * 1D Kalman Filter - legacy C implementation
 * Same algorithm will be re-implemented in C++ below
 */

#include "kalman_filter.h"

void Kalman_Init(Kalman_State* state, double initial_x, double initial_P,
                 double Q, double R)
{
    state->x = initial_x;
    state->P = initial_P;
    state->Q = Q;
    state->R = R;
}

double Kalman_Update(Kalman_State* state, double measurement)
{
    /* Predict step */
    state->P = state->P + state->Q;

    /* Update step */
    double K  = state->P / (state->P + state->R);  /* Kalman gain */
    state->x  = state->x + K * (measurement - state->x);
    state->P  = (1.0 - K) * state->P;

    return state->x;
}
