# KronMotion — PLCopen Motion Control Library

## Architecture Overview

KronMotion is a **PLCopen TC2 Part 1** compliant motion control library in baremetal C99.
It uses a **decoupled dual-task architecture**:

### Slow Task (~10ms) — `kronmotion.c` / `kronmotion.h`
- MC function blocks: `MC_Power`, `MC_Home`, `MC_MoveAbsolute`, `MC_MoveRelative`, `MC_MoveVelocity`, `MC_Halt`, `MC_Stop`, `MC_MoveAdditive`, `MC_MoveSuperimposed`
- Publishes commands via `AXIS_REF` cmd channel (`cmd_Cmd`, `cmd_TargetPos`, `cmd_TargetVel`, `cmd_Accel`, `cmd_Decel`, `cmd_Jerk`)
- Reads status via `AXIS_REF` sts channel (`sts_State`, `sts_AckSeq`, etc.)

### Fast Task (~1ms) — `kron_nc.c` / `kron_nc.h`
- NC engine: jerk-limited (S-curve) profile interpolation (minimum jerk enforced)
- CiA402 drive state machine
- Lock-free handshake: `cmd_Seq` / `sts_AckSeq` with atomic barriers
- Key functions:
  - `NC_ProcessOne()` — one axis, one cycle
  - `_nc_profile_pos()` — S-curve position profile (7-phase jerk-limited)
  - `_nc_profile_vel()` — S-curve velocity profile (jerk-limited ramp)
  - `_nc_decel_to_zero()` — jerk-limited deceleration to standstill
  - `_nc_stopping_distance()` — compute distance to stop from current (vel, acc) state
  - `_nc_latch_cmd()` — latch new command from slow task
  - `_nc_cia402_step()` — drive state machine

### Process Image — `kron_pi.h`
- `KRON_PROCESS_IMAGE` with `KRON_SERVO_SLOT` for EtherCAT/fieldbus PDO mapping
- Atomic macros: `KRON_LOAD_ACQ_U16`, `KRON_STORE_REL_U16`, `KRON_FETCH_ADD_U16`

## S-Curve Motion Profile

Triple integration: **jerk → acceleration → velocity → position**

Position mode (`_nc_profile_pos`) zones:
1. **Wrong-direction**: vel opposing target → ramp acc toward target, clamp to `min(dec, acc)`
2. **Decel zone**: `stopping_distance >= remaining` → three sub-phases (ramp-in, constant, ramp-out)
3. **Accel/Cruise zone**:
   - Above v_max → jerk-limited decel to v_max
   - At v_max → ramp acc to zero (cruise)
   - Below v_max → ramp acc up, anticipate ramp-down before v_max

Velocity safety clamp: only hard-clamp vel to v_max when acc is pushing vel further above v_max (not when decelerating).

## Key Data Structures

- `AXIS_REF` — shared between slow/fast task, contains cmd/sts channels and override factors
- `NC_AXIS` = `AXIS_REF*` + `NC_AXIS_INTERNAL` (profile state: target, v_max, acc, dec, jerk, cmd_pos/vel/acc)
- `NC_CMD_TYPE` — `NC_CMD_MOVE_ABS`, `NC_CMD_MOVE_REL`, `NC_CMD_MOVE_VEL`, `NC_CMD_HALT`, `NC_CMD_STOP`

## GUI Tool — `tool/main.cpp`

Analysis tool for motion profiles (not hardware simulation). Uses Dear ImGui + ImPlot.

- **NC thread** (1ms): runs `NC_ProcessOne`, checks anomalies, records plot data
- **Random thread**: fires random MoveAbs/MoveVel at random intervals/params
- **Main thread**: ImGui rendering, 2x2 plots (Pos|Acc / Vel|Jerk)
- Bypasses MC layer: directly publishes to `AXIS_REF` cmd channel
- Anomaly detection: vel/acc discontinuity, vel/acc overshoot, position settling
- Suppresses discontinuity checks for 2 cycles after CMD transition
- Logs saved to `tool/YYYY-MM-DD_HH-MM-SS.log` when random mode toggled OFF

## Build

```bash
mkdir build && cd build && cmake .. && cmake --build . -j$(nproc)
# Targets: kronmotion (static lib), kronmotion_tool (GUI), kronmotion_test
```

Dependencies fetched via CMake FetchContent: GLFW 3.4, ImGui v1.91.8, ImPlot v0.16.

## Coding Conventions

- C99 for library (kronmotion.c, kron_nc.c) — no libm, no dynamic allocation, baremetal compatible
- C++17 for tool (tool/main.cpp)
- Float helpers: `_NC_FABS`, `_NC_FMIN`, `_NC_FMAX`, `_NC_CLAMP`, `_NC_SIGN`
- Tolerances: `_NC_POS_EPS = 1e-4f`, `_NC_VEL_EPS = 1e-4f`
