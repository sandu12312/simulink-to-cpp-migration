/*
 * pid_controller.h
 * Legacy PID controller - C style, like Simulink Embedded Coder generates
 * Global state, no OOP, procedural
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/* Internal state - global, like Simulink RTW code */
typedef struct {
    double prev_error;
    double integral;
} PID_State;

/* extern "C" so C++ code can link against this C library without name mangling */
#ifdef __cplusplus
extern "C" {
#endif

/* Init must be called once before using */
void PID_Init(PID_State* state);

/* Call each control cycle (step function, like mdlOutputs in S-Function) */
double PID_Step(PID_State* state, double setpoint, double measured,
                double kp, double ki, double kd, double dt);

/* Reset state */
void PID_Reset(PID_State* state);

#ifdef __cplusplus
}
#endif

#endif
