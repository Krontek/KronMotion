/*===========================================================================
 * kronmotion.h  --  PLCopen Motion Control function blocks
 * Specification: PLCopen TC2 Part 1, Version 2.0 (March 17, 2011)
 *
 * The function blocks are a thin, faithful surface over the axis engine in
 * kron_axis.c: an FB validates its inputs, hands a KRON_MOTION_REQ to the axis
 * on the rising edge of Execute, and maps the request's result back onto
 * Done / Busy / Active / CommandAborted / Error every cycle.  No FB owns any
 * motion of its own, which is what makes the exclusivity and abort rules of
 * chapter 2.4 come out right.
 *
 * Calling convention:  MC_Xxx_Call(MC_Xxx *inst, AXIS_REF *axis)
 * Call every instance every cycle, as IEC 61131-3 does.
 *
 *   B = Basic     (mandatory for PLCopen compliance)
 *   E = Extended  (optional)
 *
 * REAL is mapped to double: the trajectory core works in double, and a float
 * position accumulator drifts visibly on a long-travel axis running for hours.
 *===========================================================================*/

#ifndef KRONMOTION_H
#define KRONMOTION_H

#include <stdbool.h>
#include <stdint.h>

#include "kron_axis.h"

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * Shared FB private state
 *
 * Every Execute-driven FB carries one of these.  It holds the request the axis
 * writes its result into, plus the edge detection and the guarantee that a
 * terminal output is visible for at least one cycle (2.4.1, 'Output status').
 *===========================================================================*/
typedef struct {
    KRON_MOTION_REQ req;
    bool            prevExecute;
    bool            terminalShown;
} MC_FB_PRIVATE;

/*===========================================================================
 * 3.1  MC_Power — control the power stage
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool     Enable;            /* B */
    bool     EnablePositive;    /* E */
    bool     EnableNegative;    /* E */
    /* VAR_OUTPUT */
    bool     Status;            /* B  effective state of the power stage */
    bool     Valid;             /* E */
    bool     Error;             /* B */
    uint16_t ErrorID;           /* E */
} MC_Power;

void MC_Power_Call(MC_Power *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.2  MC_Home — search home sequence
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool           Execute;         /* B */
    double         Position;        /* B  absolute position at the reference signal [u] */
    MC_BUFFER_MODE BufferMode;      /* E */
    /* VAR_OUTPUT */
    bool           Done;            /* B */
    bool           Busy;            /* E */
    bool           Active;          /* E */
    bool           CommandAborted;  /* E */
    bool           Error;           /* B */
    uint16_t       ErrorID;         /* E */
    /* private */
    MC_FB_PRIVATE  _p;
} MC_Home;

void MC_Home_Call(MC_Home *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.3  MC_Stop — controlled stop, axis to 'Stopping', nothing else allowed
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool          Execute;          /* B */
    double        Deceleration;     /* E [u/s^2] */
    double        Jerk;             /* E [u/s^3] */
    /* VAR_OUTPUT */
    bool          Done;             /* B  zero velocity reached */
    bool          Busy;             /* E */
    bool          Active;           /* E  (Done and Active may both be set here) */
    bool          CommandAborted;   /* E  only power-off can abort a stop */
    bool          Error;            /* B */
    uint16_t      ErrorID;          /* E */
    /* private */
    MC_FB_PRIVATE _p;
} MC_Stop;

void MC_Stop_Call(MC_Stop *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.4  MC_Halt — controlled stop under normal conditions, abortable
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool           Execute;         /* B */
    double         Deceleration;    /* E */
    double         Jerk;            /* E */
    MC_BUFFER_MODE BufferMode;      /* E */
    /* VAR_OUTPUT */
    bool           Done;            /* B */
    bool           Busy;            /* E */
    bool           Active;          /* E */
    bool           CommandAborted;  /* E */
    bool           Error;           /* B */
    uint16_t       ErrorID;         /* E */
    /* private */
    MC_FB_PRIVATE  _p;
} MC_Halt;

void MC_Halt_Call(MC_Halt *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.5  MC_MoveAbsolute
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool           Execute;         /* B */
    bool           ContinuousUpdate;/* E */
    double         Position;        /* B [u] */
    double         Velocity;        /* B [u/s] maximum, not necessarily reached */
    double         Acceleration;    /* E [u/s^2] */
    double         Deceleration;    /* E [u/s^2] */
    double         Jerk;            /* E [u/s^3] */
    MC_DIRECTION   Direction;       /* B  ignored on a linear axis (3.5 note 2) */
    MC_BUFFER_MODE BufferMode;      /* E */
    /* VAR_OUTPUT */
    bool           Done;            /* B */
    bool           Busy;            /* E */
    bool           Active;          /* E */
    bool           CommandAborted;  /* E */
    bool           Error;           /* B */
    uint16_t       ErrorID;         /* E */
    /* private */
    MC_FB_PRIVATE  _p;
} MC_MoveAbsolute;

void MC_MoveAbsolute_Call(MC_MoveAbsolute *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.6  MC_MoveRelative
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool           Execute;
    bool           ContinuousUpdate;
    double         Distance;        /* B [u] relative to the set position at start */
    double         Velocity;
    double         Acceleration;
    double         Deceleration;
    double         Jerk;
    MC_BUFFER_MODE BufferMode;
    /* VAR_OUTPUT */
    bool           Done;
    bool           Busy;
    bool           Active;
    bool           CommandAborted;
    bool           Error;
    uint16_t       ErrorID;
    /* private */
    MC_FB_PRIVATE  _p;
} MC_MoveRelative;

void MC_MoveRelative_Call(MC_MoveRelative *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.7  MC_MoveAdditive — relative to the most recent commanded position
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool           Execute;
    bool           ContinuousUpdate;
    double         Distance;
    double         Velocity;
    double         Acceleration;
    double         Deceleration;
    double         Jerk;
    MC_BUFFER_MODE BufferMode;
    /* VAR_OUTPUT */
    bool           Done;
    bool           Busy;
    bool           Active;
    bool           CommandAborted;
    bool           Error;
    uint16_t       ErrorID;
    /* private */
    MC_FB_PRIVATE  _p;
} MC_MoveAdditive;

void MC_MoveAdditive_Call(MC_MoveAdditive *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.8  MC_MoveSuperimposed — an extra distance on top of the running motion
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool          Execute;
    bool          ContinuousUpdate;
    double        Distance;         /* B [u] superimposed on the ongoing motion */
    double        VelocityDiff;     /* E [u/s] velocity difference of the extra motion */
    double        Acceleration;
    double        Deceleration;
    double        Jerk;
    /* VAR_OUTPUT */
    bool          Done;
    bool          Busy;
    bool          Active;
    bool          CommandAborted;
    bool          Error;
    uint16_t      ErrorID;
    double        CoveredDistance;  /* E [u] contributed by this FB so far */
    /* private */
    MC_FB_PRIVATE _p;
} MC_MoveSuperimposed;

void MC_MoveSuperimposed_Call(MC_MoveSuperimposed *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.9  MC_HaltSuperimposed — stop the superimposed motion only
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool          Execute;
    double        Deceleration;
    double        Jerk;
    /* VAR_OUTPUT */
    bool          Done;
    bool          Busy;
    bool          Active;
    bool          CommandAborted;
    bool          Error;
    uint16_t      ErrorID;
    /* private */
    MC_FB_PRIVATE _p;
} MC_HaltSuperimposed;

void MC_HaltSuperimposed_Call(MC_HaltSuperimposed *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.10  MC_MoveVelocity — never ending motion at a velocity
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool           Execute;
    bool           ContinuousUpdate;
    double         Velocity;        /* B [u/s] signed */
    double         Acceleration;
    double         Deceleration;
    double         Jerk;
    MC_DIRECTION   Direction;       /* E  negative velocity * negative direction = positive */
    MC_BUFFER_MODE BufferMode;
    /* VAR_OUTPUT */
    bool           InVelocity;      /* B  commanded velocity reached */
    bool           Busy;
    bool           Active;
    bool           CommandAborted;
    bool           Error;
    uint16_t       ErrorID;
    /* private */
    MC_FB_PRIVATE  _p;
} MC_MoveVelocity;

void MC_MoveVelocity_Call(MC_MoveVelocity *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.17  MC_SetPosition — shift the coordinate system, no movement caused
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool              Execute;
    double            Position;     /* B [u] (a distance when Relative) */
    bool              Relative;     /* E */
    MC_EXECUTION_MODE ExecutionMode;/* E */
    /* VAR_OUTPUT */
    bool              Done;
    bool              Busy;
    bool              Error;
    uint16_t          ErrorID;
    /* private */
    bool              _prevExecute;
} MC_SetPosition;

void MC_SetPosition_Call(MC_SetPosition *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.18  MC_SetOverride — velocity / acceleration / jerk factors
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool     Enable;
    double   VelFactor;             /* B  0.0 .. 1.0 */
    double   AccFactor;             /* E  > 0.0       */
    double   JerkFactor;            /* E  > 0.0       */
    /* VAR_OUTPUT */
    bool     Enabled;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
} MC_SetOverride;

void MC_SetOverride_Call(MC_SetOverride *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.19 / 3.20  MC_ReadParameter, MC_WriteParameter and the boolean versions
 *
 * Parameter numbers follow Table 5.  Numbers above 999 are vendor specific.
 *===========================================================================*/
#define MC_PARAM_COMMANDED_POSITION      1
#define MC_PARAM_SW_LIMIT_POS            2
#define MC_PARAM_SW_LIMIT_NEG            3
#define MC_PARAM_ENABLE_LIMIT_POS        4
#define MC_PARAM_ENABLE_LIMIT_NEG        5
#define MC_PARAM_ENABLE_POS_LAG_MON      6
#define MC_PARAM_MAX_POSITION_LAG        7
#define MC_PARAM_MAX_VELOCITY_SYSTEM     8
#define MC_PARAM_MAX_VELOCITY_APPL       9
#define MC_PARAM_ACTUAL_VELOCITY        10
#define MC_PARAM_COMMANDED_VELOCITY     11
#define MC_PARAM_MAX_ACCELERATION_SYS   12
#define MC_PARAM_MAX_ACCELERATION_APPL  13
#define MC_PARAM_MAX_DECELERATION_SYS   14
#define MC_PARAM_MAX_DECELERATION_APPL  15
#define MC_PARAM_MAX_JERK_SYSTEM        16
#define MC_PARAM_MAX_JERK_APPL          17
/* vendor specific */
#define MC_PARAM_IN_POSITION_WINDOW   1000
#define MC_PARAM_IN_VELOCITY_WINDOW   1001
#define MC_PARAM_GEAR_RATIO           1002
#define MC_PARAM_ACTUAL_TORQUE        1003

typedef struct {
    bool     Enable;
    int32_t  ParameterNumber;
    bool     Valid;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
    double   Value;
} MC_ReadParameter;

void MC_ReadParameter_Call(MC_ReadParameter *inst, AXIS_REF *axis);

typedef struct {
    bool     Enable;
    int32_t  ParameterNumber;
    bool     Valid;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
    bool     Value;
} MC_ReadBoolParameter;

void MC_ReadBoolParameter_Call(MC_ReadBoolParameter *inst, AXIS_REF *axis);

typedef struct {
    bool              Execute;
    int32_t           ParameterNumber;
    double            Value;
    MC_EXECUTION_MODE ExecutionMode;
    bool              Done;
    bool              Busy;
    bool              Error;
    uint16_t          ErrorID;
    bool              _prevExecute;
} MC_WriteParameter;

void MC_WriteParameter_Call(MC_WriteParameter *inst, AXIS_REF *axis);

typedef struct {
    bool              Execute;
    int32_t           ParameterNumber;
    bool              Value;
    MC_EXECUTION_MODE ExecutionMode;
    bool              Done;
    bool              Busy;
    bool              Error;
    uint16_t          ErrorID;
    bool              _prevExecute;
} MC_WriteBoolParameter;

void MC_WriteBoolParameter_Call(MC_WriteBoolParameter *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.24 - 3.26  MC_ReadActualPosition / Velocity / Torque
 *===========================================================================*/
typedef struct {
    bool     Enable;
    bool     Valid;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
    double   Position;
} MC_ReadActualPosition;

void MC_ReadActualPosition_Call(MC_ReadActualPosition *inst, AXIS_REF *axis);

typedef struct {
    bool     Enable;
    bool     Valid;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
    double   Velocity;
} MC_ReadActualVelocity;

void MC_ReadActualVelocity_Call(MC_ReadActualVelocity *inst, AXIS_REF *axis);

typedef struct {
    bool     Enable;
    bool     Valid;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
    double   Torque;
} MC_ReadActualTorque;

void MC_ReadActualTorque_Call(MC_ReadActualTorque *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.27  MC_ReadStatus — the state diagram, as booleans
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
} MC_ReadStatus;

void MC_ReadStatus_Call(MC_ReadStatus *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.28  MC_ReadMotionState
 *===========================================================================*/
typedef struct {
    bool      Enable;
    MC_SOURCE Source;
    bool      Valid;
    bool      Busy;
    bool      Error;
    uint16_t  ErrorID;
    bool      ConstantVelocity;
    bool      Accelerating;
    bool      Decelerating;
    bool      DirectionPositive;
    bool      DirectionNegative;
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
    /* the digital inputs the axis watches, index into Kron_PI.di, -1 = none */
    int16_t  HomeSwitchInput;
    int16_t  LimitSwitchPosInput;
    int16_t  LimitSwitchNegInput;
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
} MC_ReadAxisError;

void MC_ReadAxisError_Call(MC_ReadAxisError *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.31  MC_Reset — 'ErrorStop' to 'Standstill' or 'Disabled'
 *===========================================================================*/
typedef struct {
    bool     Execute;
    bool     Done;
    bool     Busy;
    bool     Error;
    uint16_t ErrorID;
    bool     _prevExecute;
    bool     _terminalShown;
} MC_Reset;

void MC_Reset_Call(MC_Reset *inst, AXIS_REF *axis);

#ifdef __cplusplus
}
#endif

#endif /* KRONMOTION_H */
