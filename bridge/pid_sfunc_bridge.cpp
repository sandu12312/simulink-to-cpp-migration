/*
 * pid_sfunc_bridge.cpp
 *
 * S-Function Bridge: this is the key piece of the migration.
 *
 * In a real Simulink project, an S-Function is a C/C++ file that Simulink
 * calls at each simulation step. It has a fixed interface (mdlOutputs, etc.)
 * This file simulates that interface but calls our new C++ PidController
 * instead of the old C code.
 *
 * This allows the OLD Simulink model to keep working unchanged,
 * while the algorithm runs in the new C++ code underneath.
 * That's what "backwards compatibility via S-Functions" means.
 */

#include "../include/pid/PidController.hpp"
#include "pid_sfunc_bridge.h"

#include <cstring>   /* memset */
#include <cstdlib>   /* malloc/free */

/* The bridge holds a C++ object inside a void* (opaque handle)
 * because C code can't see C++ classes */
struct SFuncHandle {
    PidController* controller;
};

/* mdlInitializeSizes equivalent: called once at simulation start */
SFuncHandle* pid_bridge_create(double kp, double ki, double kd, double dt)
{
    SFuncHandle* h = new SFuncHandle();
    h->controller  = new PidController(kp, ki, kd, dt);
    return h;
}

/* mdlOutputs equivalent: called every simulation step */
double pid_bridge_step(SFuncHandle* h, double setpoint, double measured)
{
    return h->controller->step(setpoint, measured);
}

/* mdlTerminate equivalent: cleanup */
void pid_bridge_destroy(SFuncHandle* h)
{
    delete h->controller;
    delete h;
}
