# KronMotion — PLCopen Motion Control Library

## What this repo is

A **PLCopen TC2 Part 1 v2.0** motion control library in C99, built on top of the
**OnlineTrajectoryGenerator** core, which CMake fetches from
`github.com/fehimkus/OnlineTrajectoryGenerator` and compiles into the same static
library.  The specification itself is in the repo:
`PLCopen_Motion_Control_Part_1_version_2.0.pdf`.

The split is the one the core's own README asks for: **the core is the engine,
this is the controller.**  The core answers one question per scan and owns
nothing; the state machine, `Done` / `Busy` / `Active` / `CommandAborted`,
buffering, blending, `MC_Stop` priority, CiA402 and the process image are here.

## Architecture — single task

There is no slow/fast split and no `cmd_Seq` / `sts_AckSeq` handshake any more.
Everything runs in one cycle, in this order:

```c
void motion_task(void)                      /* every dt seconds, e.g. 1 ms */
{
    KronMotion_ReadInputs(axes, NUM_AXES);          /* fieldbus -> axis */
    plc_program();                                  /* MC_xxx_Call(...) */
    KronMotion_WriteOutputs(axes, NUM_AXES, dt);    /* core -> fieldbus */
}
```

This works because the core re-decides the whole trajectory every scan: a new
target, a new limit or a changed override takes effect on the next cycle,
jerk-limited, with no profile to re-plan.

### Files

| File | What it holds |
|---|---|
| `kronmotion.h/.c` | The `MC_xxx` function blocks.  Edge detection, input validation, output mapping — nothing else. |
| `kron_axis.h/.c` | `AXIS_REF`, the command queue, the PLCopen state diagram, blending, homing, the CiA402 drive state machine, the process image exchange. |
| `kron_pi.h/.c` | `KRON_PROCESS_IMAGE`, `KRON_SERVO_SLOT`, the `KRON_HAL_Driver` contract and the two globals. |
| `test/kronmotion_test.c` | The self test: 90 checks over 17 scenarios, all in simulation. |
| `error_codes.xml` | The `ErrorID` catalog. |
| `tool/main.cpp` | The old ImGui/ImPlot analysis tool.  **Not built** — it was written against the removed `kron_nc.c` and has not been ported. |

`kron_nc.c` / `kron_nc.h` (the old S-curve NC engine) are gone; the core replaces
them.

## How a command flows

1. An FB detects the rising edge of `Execute`, fills the `KRON_MOTION_REQ` that
   lives **inside its own instance**, and calls `KronAxis_Submit()`.
2. The axis either takes it (`mcAborting`), queues it, or rejects it with an
   `ErrorID`.  The request is a pointer, so the result always finds its way back
   to the FB that issued it and the axis never allocates.
3. Every cycle `KronAxis_WriteOutput()` turns the running request into one
   `MotionLimits` + `MotionCommand` pair and calls `GenerateTrajectory()` once.
   A superimposed motion gets a second, independent call.
4. The FB reads `req.State` back and maps it onto its outputs.

`KRON_REQ_STATE` is the whole protocol: `IDLE`, `BUFFERED`, `ACTIVE`, `DONE`,
`ABORTED`, `ERROR`.

## Decisions worth knowing

- **`double` everywhere.**  The core is `double`, and a `float` position
  accumulator drifts visibly on a long-travel axis running for hours.  PLCopen
  `REAL` is therefore mapped to `double` (`LREAL`) at the FB boundary too.
- **Limits fold together.**  `pick_limit()` takes the FB's value when it was
  given, falls back to the axis limit when it was not, and clamps an FB value
  above the axis limit rather than erroring (PLCopen 2.4.1 allows either).
- **Jerk 0 means trapezoidal**, expressed as a jerk 1000x the acceleration
  rather than a special case in the core.
- **A velocity bound of exactly 0 is degenerate in the core** — it holds the
  state instead of braking — so `build_limits()` floors it at `1e-9`.  That is
  what makes `VelFactor = 0.0` stop the axis without leaving its state
  (PLCopen 3.18 note 5).
- **Blending has no support in the core** (it always arrives at rest), so it is
  expressed as a **virtual target** one brake distance of the blend velocity
  beyond the real end position.  Coming to rest there means passing the real end
  position *at* the blend velocity.  The handover is the moment the set position
  crosses the real end position.  A blend whose next command reverses direction
  degrades to `mcBuffered`, because a reversal has to come to rest first.
- **Software limits** are handed to the core as `NegativeLimit` / `PositiveLimit`
  only when both enable flags are set; the core ignores an unordered pair, which
  is exactly how "disabled" is expressed.
- **`MC_Power.EnablePositive` / `EnableNegative`** bar motion one way.  Both
  false means the extended inputs are not in use and the axis is unrestricted —
  otherwise an FB instance that was never written would freeze every axis.  A
  command into a barred direction is not refused, it brakes and stays `Busy`:
  the permission is level sensitive and may come back.
- **An aborted FB reports on the cycle after the abort**, because its outputs are
  written when its own `_Call` runs.  This is ordinary IEC 61131-3 behaviour.

## Implemented function blocks

`MC_Power`, `MC_Home`, `MC_Stop`, `MC_Halt`, `MC_MoveAbsolute`,
`MC_MoveRelative`, `MC_MoveAdditive`, `MC_MoveSuperimposed`,
`MC_HaltSuperimposed`, `MC_MoveVelocity`, `MC_SetPosition`, `MC_SetOverride`,
`MC_ReadParameter`, `MC_ReadBoolParameter`, `MC_WriteParameter`,
`MC_WriteBoolParameter`, `MC_ReadActualPosition`, `MC_ReadActualVelocity`,
`MC_ReadActualTorque`, `MC_ReadStatus`, `MC_ReadMotionState`, `MC_ReadAxisInfo`,
`MC_ReadAxisError`, `MC_Reset`.

**Not implemented**: `MC_MoveContinuousAbsolute` / `MC_MoveContinuousRelative`
(they need a non-zero target velocity, which the core does not support yet),
`MC_TorqueControl`, the profile blocks, touch probe / cam switch, and everything
multi-axis (gearing, camming, `MC_CombineAxes`).  `MC_AXIS_SYNCHRONIZED_MOTION`
exists in the enum but nothing enters it.

## Build

```bash
cmake -B build -S .
cmake --build build -j
./build/kronmotion_test
```

The core is fetched with `FetchContent`.  Its own `CMakeLists.txt` requires Qt6
for its test rig, so `SOURCE_SUBDIR` points at a directory with no
`CMakeLists.txt`: FetchContent downloads the repository without configuring it,
and `TrajectoryGenerator.c` is compiled straight into `kronmotion`.

Targets: `kronmotion` (static lib), `kronmotion_test`.
`KRONMOTION_BUILD_TOOL` is off and refuses to configure.

## Coding conventions

- C99, no dynamic allocation, no recursion, fixed cost per cycle.  `libm` is
  used (the core calls `sqrt`), compiled with `-fno-math-errno`.
- Allman braces, 4 space indent, comments in English, lowercase, at the end of
  the line, one line each.
- Everything the code does because the standard says so carries the clause
  number in the comment.
- Claims about behaviour are backed by a check in `test/kronmotion_test.c`.
