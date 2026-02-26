/*
 * pid_sfunc_bridge.h
 * C-compatible header so legacy C code can call the C++ bridge
 */

#ifndef PID_SFUNC_BRIDGE_H
#define PID_SFUNC_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque handle - C code doesn't need to know what's inside */
typedef struct SFuncHandle SFuncHandle;

SFuncHandle* pid_bridge_create(double kp, double ki, double kd, double dt);
double       pid_bridge_step(SFuncHandle* h, double setpoint, double measured);
void         pid_bridge_destroy(SFuncHandle* h);

#ifdef __cplusplus
}
#endif

#endif
