# KronMotion

**PLCopen Motion Control Part 1 function blocks, in C99, on a scan-based
jerk-limited trajectory core.**

The motion core is [OnlineTrajectoryGenerator](https://github.com/fehimkus/OnlineTrajectoryGenerator):
it is handed where the axis is, what it may not exceed and where it is told to
go, and it answers with the state one cycle later. Nothing is planned up front.
KronMotion is everything on top of that — the PLCopen state diagram, the
function blocks and their `Done` / `Busy` / `Active` / `CommandAborted` outputs,
the command buffer and blending, `MC_Stop` priority, the CiA402 drive state
machine and the fieldbus process image.

CMake fetches the core and compiles it into the same static library.

## Use

One task, one cycle, in this order:

```c
#include "kronmotion.h"

static AXIS_REF        axes[NUM_AXES];
static MC_Power        power;
static MC_MoveAbsolute move;

void motion_task(void)                       /* every 1 ms */
{
    KronMotion_ReadInputs(axes, NUM_AXES);

    power.Enable = true;
    MC_Power_Call(&power, &axes[0]);

    move.Position     = 250.0;               /* [u]     */
    move.Velocity     = 300.0;               /* [u/s]   */
    move.Acceleration = 1500.0;              /* [u/s^2] */
    move.Deceleration = 1500.0;
    move.Jerk         = 30000.0;             /* [u/s^3] */
    move.Execute      = start_button;
    MC_MoveAbsolute_Call(&move, &axes[0]);

    KronMotion_WriteOutputs(axes, NUM_AXES, 0.001);
}
```

Set the axis up once at startup:

```c
AXIS_REF_Init(&axes[0], 0u, &Kron_PI.servo[0]);
axes[0].MaxVelocity     = 500.0;
axes[0].MaxAcceleration = 2000.0;
axes[0].MaxDeceleration = 2000.0;
axes[0].MaxJerk         = 40000.0;
axes[0].SwLimitNegative = -50.0;
axes[0].SwLimitPositive = 1000.0;
axes[0].EnableLimitNegative = true;
axes[0].EnableLimitPositive = true;
```

A block may leave `Velocity`, `Acceleration`, `Deceleration` or `Jerk` at zero,
and the axis limit is used instead. A value above the axis limit is clamped to
it.

## Function blocks

`MC_Power`, `MC_Home`, `MC_Stop`, `MC_Halt`, `MC_MoveAbsolute`,
`MC_MoveRelative`, `MC_MoveAdditive`, `MC_MoveSuperimposed`,
`MC_HaltSuperimposed`, `MC_MoveVelocity`, `MC_SetPosition`, `MC_SetOverride`,
`MC_ReadParameter`, `MC_ReadBoolParameter`, `MC_WriteParameter`,
`MC_WriteBoolParameter`, `MC_ReadActualPosition`, `MC_ReadActualVelocity`,
`MC_ReadActualTorque`, `MC_ReadStatus`, `MC_ReadMotionState`, `MC_ReadAxisInfo`,
`MC_ReadAxisError`, `MC_Reset`.

All six `MC_BUFFER_MODE` values are supported. `mcAborting` and `mcBuffered` are
exact. The four blending modes are built on a virtual target one brake distance
of the blend velocity beyond the real end position, so the axis passes the end
position at that velocity instead of stopping on it; a blend whose next command
reverses direction degrades to `mcBuffered`, because a reversal has to come to
rest first.

Not implemented: `MC_MoveContinuousAbsolute` / `MC_MoveContinuousRelative` (they
need a non-zero target velocity, which the core does not support yet),
`MC_TorqueControl`, the profile blocks, touch probe, digital cam switch, and
everything multi-axis.

## Build

```bash
cmake -B build -S .
cmake --build build -j
./build/kronmotion_test
```

Targets: `kronmotion` (static library), `kronmotion_test`. No Qt, no GUI, no
dependency beyond the C standard library and `libm`.

## What is verified

`test/kronmotion_test.c` runs 17 scenarios against a simulated axis and checks
90 statements of the specification: the state diagram transitions, output
exclusivity and the reset rules of 2.4.1, the axis limits never being exceeded,
`MC_Stop` refusing every other command while it holds the axis, the direction
permissions of `MC_Power`, buffered versus
blended handover, override including `VelFactor = 0.0`, the software limits, a
superimposed motion riding on a running one, and the `ErrorStop` / `MC_Reset`
path.

Not verified: hardware. Everything is simulation — no drive, no scan jitter, and
a drive that cannot follow the commanded jerk is unmodelled. The core's own open
issues (see its README) apply here unchanged.

## Licence

GPL-3.0, as is the core.
