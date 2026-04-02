/*===========================================================================
 * kronmotion.h  --  KronEditor PLCopen Motion Control Function Blocks
 * Specification: PLCopen TC2 Part 1 Version 2.0 (March 17, 2011)
 *
 * Architecture: Decoupled Dual-Task Motion
 *   Slow Task  (~10ms) — runs MC_xxx FBs.  FBs write to AXIS_REF cmd channel,
 *                         read from AXIS_REF sts channel.  No interpolation here.
 *   Fast Task  (~1ms)  — NC Engine (kron_nc.c) reads cmd channel, runs
 *                         trapezoidal profile, writes sts channel + process image.
 *
 * Lock-free handshake (cmd_Seq / sts_AckSeq):
 *   Slow Task: write cmd_* params, then KRON_FETCH_ADD_U16(&axis->cmd_Seq, 1)
 *   Fast Task: poll cmd_Seq != sts_AckSeq → latch new command →
 *              KRON_STORE_REL_U16(&axis->sts_AckSeq, latched_seq)
 *
 * Naming convention: XXX_Call(XXX *inst, AXIS_REF *axis)
 * B = Basic (mandatory per PLCopen compliance)
 * E = Extended (optional)
 *===========================================================================*/

#ifndef KRONMOTION_H
#define KRONMOTION_H

#include <stdbool.h>
#include <stdint.h>
#include "kron_pi.h"              /* KRON_SERVO_SLOT*, NC_CMD_TYPE, atomic macros */

/*===========================================================================
 * ENUMERATIONS
 *===========================================================================*/

typedef enum {
    mcAborting         = 0,
    mcBuffered         = 1,
    mcBlendingLow      = 2,
    mcBlendingPrevious = 3,
    mcBlendingNext     = 4,
    mcBlendingHigh     = 5
} MC_BUFFER_MODE;

typedef enum {
    mcPositiveDirection = 1,
    mcShortestWay       = 2,
    mcNegativeDirection = 3,
    mcCurrentDirection  = 4
} MC_DIRECTION;

typedef enum {
    mcImmediately = 0,
    mcQueued      = 1
} MC_EXECUTION_MODE;

typedef enum {
    mcCommandedValue = 0,
    mcSetValue       = 1,
    mcActualValue    = 2
} MC_SOURCE;

typedef enum {
    MC_AXIS_DISABLED            = 0,
    MC_AXIS_STANDSTILL          = 1,
    MC_AXIS_HOMING              = 2,
    MC_AXIS_STOPPING            = 3,
    MC_AXIS_DISCRETE_MOTION     = 4,
    MC_AXIS_CONTINUOUS_MOTION   = 5,
    MC_AXIS_SYNCHRONIZED_MOTION = 6,
    MC_AXIS_ERRORSTOP           = 7
} MC_AXIS_STATE;

/*===========================================================================
 * AXIS_REF — Axis Reference (Bridge between Slow Task FBs and NC Engine)
 *
 * Memory layout is split into three clear sections:
 *   1. Identity & link      — set once at init
 *   2. Command channel      — written by Slow Task, read by Fast Task
 *   3. Status channel       — written by Fast Task, read by Slow Task
 *
 * Slow Task MUST NOT write to sts_* fields.
 * Fast Task MUST NOT write to cmd_* fields.
 * Shared read-only fields (ActualPosition, etc.) are written by Fast Task only.
 *===========================================================================*/
typedef struct {

    /* ── 1. Identity & Hardware Link ────────────────────────────────────── */
    uint16_t          AxisNo;           /* Axis identifier (0-based)          */
    KRON_SERVO_SLOT  *slot;             /* Pointer into Kron_PI.servo[n]      */
                                        /* Set by generated plc.c at startup. */
                                        /* NULL in simulation mode.           */
    bool              Simulation;       /* TRUE: NC engine runs without hw    */

    /* ── Override factors (written by MC_SetOverride, read by NC engine) ── */
    float             VelFactor;        /* Velocity override  [0.0 .. 1.0]    */
    float             AccFactor;        /* Acc/dec override   [0.0 .. 1.0]    */
    float             JerkFactor;       /* Jerk override      [0.0 .. 1.0]    */

    /* ── Convenience actual values (written by NC each fast cycle) ──────── */
    /* Read by MC_ReadActualPosition, MC_ReadActualVelocity, etc.            */
    float             ActualPosition;   /* [u]                                */
    float             ActualVelocity;   /* [u/s], signed                      */
    float             ActualTorque;     /* [%rated], signed                   */

    /* ── Profile generator setpoints (written by NC each fast cycle) ────── */
    float             CommandedPosition;
    float             CommandedVelocity;

    /* ── Axis-level status (written by NC) ──────────────────────────────── */
    bool              IsHomed;
    bool              AxisWarning;
    uint16_t          AxisErrorID;

    /* ─────────────────────────────────────────────────────────────────────
     * COMMAND CHANNEL   (Slow Task writes → NC Engine reads)
     *
     * Protocol:
     *   1. Write all cmd_* param fields.
     *   2. KRON_FETCH_ADD_U16(&axis->cmd_Seq, 1u)  ← publish (RELEASE barrier)
     *
     * NC Engine detects new command when cmd_Seq != sts_AckSeq.
     * ───────────────────────────────────────────────────────────────────── */
    volatile uint16_t cmd_Seq;          /* Incremented by Slow Task to publish */
    NC_CMD_TYPE       cmd_Cmd;          /* Command type                        */
    float             cmd_TargetPos;    /* NC_CMD_MOVE_ABS / MOVE_REL / HOME   */
    float             cmd_TargetVel;    /* Maximum velocity [u/s]              */
    float             cmd_Accel;        /* Acceleration [u/s^2]                */
    float             cmd_Decel;        /* Deceleration [u/s^2]                */
    float             cmd_Jerk;         /* Jerk [u/s^3], 0 = trapezoidal      */
    float             cmd_HomePos;      /* Position set at home signal         */

    /* ─────────────────────────────────────────────────────────────────────
     * STATUS CHANNEL    (NC Engine writes → Slow Task reads)
     *
     * NC Engine:
     *   1. Latch cmd_* params internally.
     *   2. KRON_STORE_REL_U16(&axis->sts_AckSeq, latched_seq) ← acknowledge
     *   3. Update sts_Busy / sts_Done / sts_Error each fast cycle.
     *   4. Write sts_State each fast cycle (PLCopen state machine).
     *
     * Slow Task FBs read sts_* to report Busy/Done/Error/State.
     * ───────────────────────────────────────────────────────────────────── */
    volatile uint16_t sts_AckSeq;       /* Echo of cmd_Seq when NC latches cmd */
    volatile MC_AXIS_STATE sts_State;   /* PLCopen state, authoritative copy   */
    bool              sts_Busy;
    bool              sts_Done;
    bool              sts_Error;
    bool              sts_CommandAborted;
    uint16_t          sts_ErrorID;

    /* ── Drive diagnostics (written by NC each fast cycle, read-only) ──────── */
    /* Raw CiA402 statusword / controlword from the servo drive's PDO.          */
    uint16_t          drv_StatusWord;   /* 0x6041 — last received from drive  */
    uint16_t          drv_ControlWord;  /* 0x6040 — last sent to drive        */

    /* ── Slow-Task-only: abort coordination between concurrent FBs ───────── */
    /* When a new FB takes control it increments _ActiveToken.               */
    /* Each FB stores its own token at Execute↑ in _myToken (FB-private).   */
    /* If _myToken != _ActiveToken the FB was preempted → CommandAborted.   */
    uint16_t          _ActiveToken;

} AXIS_REF;

/*===========================================================================
 * AXIS_REF helpers
 *===========================================================================*/
void AXIS_REF_Init(AXIS_REF *axis, uint16_t axisNo, KRON_SERVO_SLOT *slot);

/* Inline used by FBs: publish a new command to the NC engine */
static inline void _axis_publish_cmd(AXIS_REF *axis, NC_CMD_TYPE cmd,
                                     float tgt, float vel,
                                     float acc, float dec, float jerk)
{
    axis->cmd_Cmd       = cmd;
    axis->cmd_TargetPos = tgt;
    axis->cmd_TargetVel = vel * axis->VelFactor;
    axis->cmd_Accel     = acc * axis->AccFactor;
    axis->cmd_Decel     = dec * axis->AccFactor;
    axis->cmd_Jerk      = jerk * axis->JerkFactor;
    KRON_FETCH_ADD_U16(&axis->cmd_Seq, 1u);   /* RELEASE barrier — publish */
}

/* Inline used by FBs: take exclusive axis control (abort other FBs) */
static inline uint16_t _axis_take_token(AXIS_REF *axis) {
    return ++axis->_ActiveToken;
}

/* Inline used by FBs: check if another FB has taken control */
static inline bool _axis_token_aborted(const AXIS_REF *axis, uint16_t my_token) {
    return my_token != axis->_ActiveToken;
}

/*===========================================================================
 * 3.1  MC_Power
 *===========================================================================*/
typedef struct {
    bool     Enable;
    bool     EnablePositive;
    bool     EnableNegative;

    bool     Status;
    bool     Valid;
    bool     Error;
    uint16_t ErrorID;

    bool     _prevEnable;
} MC_Power;

void MC_Power_Call(MC_Power *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.2  MC_Home
 *===========================================================================*/
typedef struct {
    bool           Execute;
    float          Position;
    MC_BUFFER_MODE BufferMode;

    bool     Done;
    bool     Busy;
    bool     Active;
    bool     CommandAborted;
    bool     Error;
    uint16_t ErrorID;

    bool     _prevExecute;
    uint16_t _myToken;
} MC_Home;

void MC_Home_Call(MC_Home *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.3  MC_Stop
 *===========================================================================*/
typedef struct {
    bool     Execute;
    float    Deceleration;
    float    Jerk;

    bool     Done;
    bool     Busy;
    bool     CommandAborted;
    bool     Error;
    uint16_t ErrorID;

    bool     _prevExecute;
} MC_Stop;

void MC_Stop_Call(MC_Stop *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.4  MC_Halt
 *===========================================================================*/
typedef struct {
    bool           Execute;
    float          Deceleration;
    float          Jerk;
    MC_BUFFER_MODE BufferMode;

    bool     Done;
    bool     Busy;
    bool     Active;
    bool     CommandAborted;
    bool     Error;
    uint16_t ErrorID;

    bool     _prevExecute;
    uint16_t _myToken;
} MC_Halt;

void MC_Halt_Call(MC_Halt *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.5  MC_MoveAbsolute
 *===========================================================================*/
typedef struct {
    bool           Execute;
    bool           ContinuousUpdate;
    float          Position;
    float          Velocity;
    float          Acceleration;
    float          Deceleration;
    float          Jerk;
    MC_DIRECTION   Direction;
    MC_BUFFER_MODE BufferMode;

    bool     Done;
    bool     Busy;
    bool     Active;
    bool     CommandAborted;
    bool     Error;
    uint16_t ErrorID;

    bool     _prevExecute;
    uint16_t _myToken;
} MC_MoveAbsolute;

void MC_MoveAbsolute_Call(MC_MoveAbsolute *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.6  MC_MoveRelative
 *===========================================================================*/
typedef struct {
    bool           Execute;
    bool           ContinuousUpdate;
    float          Distance;
    float          Velocity;
    float          Acceleration;
    float          Deceleration;
    float          Jerk;
    MC_BUFFER_MODE BufferMode;

    bool     Done;
    bool     Busy;
    bool     Active;
    bool     CommandAborted;
    bool     Error;
    uint16_t ErrorID;

    bool     _prevExecute;
    float    _targetPosition;
    uint16_t _myToken;
} MC_MoveRelative;

void MC_MoveRelative_Call(MC_MoveRelative *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.7  MC_MoveAdditive
 *===========================================================================*/
typedef struct {
    bool           Execute;
    bool           ContinuousUpdate;
    float          Distance;
    float          Velocity;
    float          Acceleration;
    float          Deceleration;
    float          Jerk;
    MC_BUFFER_MODE BufferMode;

    bool     Done;
    bool     Busy;
    bool     Active;
    bool     CommandAborted;
    bool     Error;
    uint16_t ErrorID;

    bool     _prevExecute;
    float    _targetPosition;
    uint16_t _myToken;
} MC_MoveAdditive;

void MC_MoveAdditive_Call(MC_MoveAdditive *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.8  MC_MoveSuperimposed
 *===========================================================================*/
typedef struct {
    bool     Execute;
    bool     ContinuousUpdate;
    float    Distance;
    float    VelocityDiff;
    float    Acceleration;
    float    Deceleration;
    float    Jerk;

    bool     Done;
    bool     Busy;
    bool     Active;
    bool     CommandAborted;
    bool     Error;
    uint16_t ErrorID;
    float    CoveredDistance;

    bool     _prevExecute;
    float    _coveredSoFar;
    uint16_t _myToken;
} MC_MoveSuperimposed;

void MC_MoveSuperimposed_Call(MC_MoveSuperimposed *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.9  MC_HaltSuperimposed
 *===========================================================================*/
typedef struct {
    bool     Execute;
    float    Deceleration;
    float    Jerk;

    bool     Done;
    bool     Busy;
    bool     Active;
    bool     CommandAborted;
    bool     Error;
    uint16_t ErrorID;

    bool     _prevExecute;
    uint16_t _myToken;
} MC_HaltSuperimposed;

void MC_HaltSuperimposed_Call(MC_HaltSuperimposed *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.10  MC_MoveVelocity
 *===========================================================================*/
typedef struct {
    bool           Execute;
    bool           ContinuousUpdate;
    float          Velocity;
    float          Acceleration;
    float          Deceleration;
    float          Jerk;
    MC_DIRECTION   Direction;
    MC_BUFFER_MODE BufferMode;

    bool     InVelocity;
    bool     Busy;
    bool     Active;
    bool     CommandAborted;
    bool     Error;
    uint16_t ErrorID;

    bool     _prevExecute;
    uint16_t _myToken;
} MC_MoveVelocity;

void MC_MoveVelocity_Call(MC_MoveVelocity *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.11  MC_MoveContinuousAbsolute
 *===========================================================================*/
typedef struct {
    bool           Execute;
    bool           ContinuousUpdate;
    float          Position;
    float          EndVelocity;
    float          Velocity;
    float          Acceleration;
    float          Deceleration;
    float          Jerk;
    MC_DIRECTION   Direction;
    MC_BUFFER_MODE BufferMode;

    bool     InEndVelocity;
    bool     Busy;
    bool     Active;
    bool     CommandAborted;
    bool     Error;
    uint16_t ErrorID;

    bool     _prevExecute;
    uint16_t _myToken;
} MC_MoveContinuousAbsolute;

void MC_MoveContinuousAbsolute_Call(MC_MoveContinuousAbsolute *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.12  MC_MoveContinuousRelative
 *===========================================================================*/
typedef struct {
    bool           Execute;
    bool           ContinuousUpdate;
    float          Distance;
    float          EndVelocity;
    float          Velocity;
    float          Acceleration;
    float          Deceleration;
    float          Jerk;
    MC_BUFFER_MODE BufferMode;

    bool     InEndVelocity;
    bool     Busy;
    bool     Active;
    bool     CommandAborted;
    bool     Error;
    uint16_t ErrorID;

    bool     _prevExecute;
    float    _targetPosition;
    uint16_t _myToken;
} MC_MoveContinuousRelative;

void MC_MoveContinuousRelative_Call(MC_MoveContinuousRelative *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.17  MC_SetPosition
 *===========================================================================*/
typedef struct {
    bool              Execute;
    float             Position;
    bool              Relative;
    MC_EXECUTION_MODE ExecutionMode;

    bool     Done;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;

    bool     _prevExecute;
} MC_SetPosition;

void MC_SetPosition_Call(MC_SetPosition *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.18  MC_SetOverride
 *===========================================================================*/
typedef struct {
    bool     Enable;
    float    VelFactor;
    float    AccFactor;
    float    JerkFactor;

    bool     Enabled;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;

    bool     _prevEnable;
} MC_SetOverride;

void MC_SetOverride_Call(MC_SetOverride *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.19  MC_ReadParameter / MC_ReadBoolParameter
 *===========================================================================*/
typedef struct {
    bool     Enable;
    int16_t  ParameterNumber;

    bool     Valid;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
    float    Value;

    bool     _prevEnable;
} MC_ReadParameter;

void MC_ReadParameter_Call(MC_ReadParameter *inst, AXIS_REF *axis);

typedef struct {
    bool     Enable;
    int16_t  ParameterNumber;

    bool     Valid;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
    bool     Value;

    bool     _prevEnable;
} MC_ReadBoolParameter;

void MC_ReadBoolParameter_Call(MC_ReadBoolParameter *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.20  MC_WriteParameter / MC_WriteBoolParameter
 *===========================================================================*/
typedef struct {
    bool              Execute;
    int16_t           ParameterNumber;
    float             Value;
    MC_EXECUTION_MODE ExecutionMode;

    bool     Done;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;

    bool     _prevExecute;
} MC_WriteParameter;

void MC_WriteParameter_Call(MC_WriteParameter *inst, AXIS_REF *axis);

typedef struct {
    bool              Execute;
    int16_t           ParameterNumber;
    bool              Value;
    MC_EXECUTION_MODE ExecutionMode;

    bool     Done;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;

    bool     _prevExecute;
} MC_WriteBoolParameter;

void MC_WriteBoolParameter_Call(MC_WriteBoolParameter *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.24  MC_ReadActualPosition
 *===========================================================================*/
typedef struct {
    bool     Enable;

    bool     Valid;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
    float    Position;

    bool     _prevEnable;
} MC_ReadActualPosition;

void MC_ReadActualPosition_Call(MC_ReadActualPosition *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.25  MC_ReadActualVelocity
 *===========================================================================*/
typedef struct {
    bool     Enable;

    bool     Valid;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
    float    Velocity;

    bool     _prevEnable;
} MC_ReadActualVelocity;

void MC_ReadActualVelocity_Call(MC_ReadActualVelocity *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.26  MC_ReadActualTorque
 *===========================================================================*/
typedef struct {
    bool     Enable;

    bool     Valid;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
    float    Torque;

    bool     _prevEnable;
} MC_ReadActualTorque;

void MC_ReadActualTorque_Call(MC_ReadActualTorque *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.27  MC_ReadStatus
 *===========================================================================*/
typedef struct {
    bool     Enable;

    bool     Valid;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
    bool     ErrorStop;
    bool     Disabled;
    bool     Stopping;
    bool     Homing;
    bool     Standstill;
    bool     DiscreteMotion;
    bool     ContinuousMotion;
    bool     SynchronizedMotion;

    bool     _prevEnable;
} MC_ReadStatus;

void MC_ReadStatus_Call(MC_ReadStatus *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.28  MC_ReadMotionState
 *===========================================================================*/
typedef struct {
    bool      Enable;
    MC_SOURCE Source;

    bool     Valid;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
    bool     ConstantVelocity;
    bool     Accelerating;
    bool     Decelerating;
    bool     DirectionPositive;
    bool     DirectionNegative;

    bool     _prevEnable;
    float    _prevVelocity;
} MC_ReadMotionState;

void MC_ReadMotionState_Call(MC_ReadMotionState *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.29  MC_ReadAxisInfo
 *===========================================================================*/
typedef struct {
    bool     Enable;

    bool     Valid;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
    bool     HomeAbsSwitch;
    bool     LimitSwitchPos;
    bool     LimitSwitchNeg;
    bool     Simulation;
    bool     CommunicationReady;
    bool     ReadyForPowerOn;
    bool     PowerOn;
    bool     IsHomed;
    bool     AxisWarning;

    bool     _prevEnable;
} MC_ReadAxisInfo;

void MC_ReadAxisInfo_Call(MC_ReadAxisInfo *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.30  MC_ReadAxisError
 *===========================================================================*/
typedef struct {
    bool     Enable;

    bool     Valid;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
    uint16_t AxisErrorID;

    bool     _prevEnable;
} MC_ReadAxisError;

void MC_ReadAxisError_Call(MC_ReadAxisError *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.31  MC_Reset
 *===========================================================================*/
typedef struct {
    bool     Execute;

    bool     Done;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;

    bool     _prevExecute;
} MC_Reset;

void MC_Reset_Call(MC_Reset *inst, AXIS_REF *axis);

#endif /* KRONMOTION_H */
