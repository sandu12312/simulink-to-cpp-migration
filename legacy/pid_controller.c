/*
 * pid_controller.c
 * Legacy PID - this is how Simulink Embedded Coder generates C code
 * Problems with this approach:
 *   - global state is hard to test
 *   - can't have two independent PID instances easily
 *   - no error handling, no encapsulation
 */

#include "pid_controller.h"

void PID_Init(PID_State* state)
{
    state->prev_error = 0.0;
    state->integral   = 0.0;
}

double PID_Step(PID_State* state, double setpoint, double measured,
                double kp, double ki, double kd, double dt)
{
    double error      = setpoint - measured;
    double derivative = (error - state->prev_error) / dt;

    state->integral  += error * dt;
    state->prev_error = error;

    return kp * error + ki * state->integral + kd * derivative;
}

void PID_Reset(PID_State* state)
{
    state->prev_error = 0.0;
    state->integral   = 0.0;
}
