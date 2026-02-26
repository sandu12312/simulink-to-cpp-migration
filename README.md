# Simulink to C++ Migration

Demonstrates migrating ADAS algorithms from Matlab Simulink-style C code to modern C++17, while keeping backwards compatibility through an S-Function bridge.

This is the exact workflow described in ADAS/Embedded SW job descriptions:
> "Migrate legacy features (Matlab Simulink based) to a new SW architecture (C++ based), ensure backwards compatibility via S-Functions"

---

## What's in here

```
legacy/              ← C code, same style as Simulink Embedded Coder output
  pid_controller.c   ← PID controller (global state, procedural)
  kalman_filter.c    ← 1D Kalman Filter (global state, procedural)

include/
  pid/PidController.hpp     ← same PID rewritten in C++17 (OOP, testable)
  kalman/KalmanFilter.hpp   ← same Kalman rewritten in C++17

bridge/
  pid_sfunc_bridge.cpp  ← S-Function style wrapper: C interface → C++ impl
  pid_sfunc_bridge.h    ← C-compatible header (extern "C")

tests/
  test_pid_equivalence.cpp     ← runs both, checks outputs are identical
  test_kalman_equivalence.cpp  ← same for Kalman
```

---

## Why migrate away from Simulink-generated C?

| Problem with legacy C | Solution in C++17 |
|---|---|
| Global state → can't have two instances | Each object owns its state |
| Hard to unit test | Instantiate and test freely |
| Parameters passed every step | Set once in constructor |
| No type safety | Typed interfaces |
| Harder to port to new platforms | Plain C++17, no Matlab dependency |

---

## How the S-Function bridge works

The bridge is the key piece. It lets the **old Simulink model keep calling C functions** (unchanged), while the actual algorithm runs in the new C++ code underneath.

```
Simulink model
     │
     │  calls C function (legacy interface)
     ▼
pid_sfunc_bridge.cpp   ←── this is the "S-Function"
     │
     │  creates/calls C++ PidController
     ▼
PidController.hpp      ←── new C++17 implementation
```

The bridge uses an opaque handle (`void*`) so C code doesn't need to know anything about C++ classes:

```c
// C code (legacy Simulink model) calls this:
SFuncHandle* h = pid_bridge_create(1.2, 0.5, 0.1, 0.01);
double output  = pid_bridge_step(h, setpoint, measured);
pid_bridge_destroy(h);
```

Internally, `pid_bridge_step` calls `h->controller->step(...)` on the C++ object.

---

## Build and run

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Expected output:
```
Step | Legacy output | C++ output    | Diff
-----|---------------|---------------|----------
   0 |     24.000000 |     24.000000 | 0.00e+00
   ...
PASS — C++ output matches legacy C within epsilon 1e-10
Migration is backwards compatible!
```

---

## Key concepts demonstrated

- **Simulink Embedded Coder** style: procedural C with explicit state structs
- **S-Function interface**: `mdlInitializeSizes`, `mdlOutputs`, `mdlTerminate` pattern
- **extern "C" bridge**: C++ code callable from C without name mangling
- **Backwards compatibility testing**: numerical equivalence within floating-point epsilon
- **OOP migration pattern**: same algorithm, different structure
