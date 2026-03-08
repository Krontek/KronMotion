/*===========================================================================
 * KronMotion - PLCopen Motion Control Function Blocks
 * Specification: PLCopen TC2 Part 1 Version 2.0 (March 17, 2011)
 *
 * Baremetal C implementation compatible with ARM Cortex-M4.
 * No dynamic memory, no libm, no OS dependencies.
 * Requires C99 or later.
 *
 * Naming convention: XXX_Call(XXX *inst, AXIS_REF *axis)
 * TIME unit: uint32_t (ms or us — caller decides and must be consistent)
 * Position/Velocity/Acceleration units: user-defined [u], [u/s], [u/s^2]
 *
 * B = Basic (mandatory per PLCopen compliance)
 * E = Extended (optional)
 *===========================================================================*/

#ifndef KRONMOTION_H
#define KRONMOTION_H

#include <stdbool.h>
#include <stdint.h>
#define __int8_t_defined

/*===========================================================================
 * ENUMERATIONS
 *===========================================================================*/

/**
 * MC_BUFFER_MODE - Defines how a new motion command interacts with
 * an ongoing motion on the same axis. (Table 3, PLCopen Part 1 v2.0)
 */
typedef enum {
    mcAborting         = 0,  /* B - Abort current motion immediately (default) */
    mcBuffered         = 1,  /* E - Start after current motion is Done */
    mcBlendingLow      = 2,  /* E - Blend at lower velocity of both commands */
    mcBlendingPrevious = 3,  /* E - Blend at velocity of first command */
    mcBlendingNext     = 4,  /* E - Blend at velocity of second command */
    mcBlendingHigh     = 5   /* E - Blend at higher velocity of both commands */
} MC_BUFFER_MODE;

/**
 * MC_DIRECTION - Direction of motion for applicable function blocks.
 */
typedef enum {
    mcPositiveDirection = 1, /* B - Motion in positive direction */
    mcShortestWay       = 2, /* E - Shortest way (modulo/rotary axes) */
    mcNegativeDirection = 3, /* B - Motion in negative direction */
    mcCurrentDirection  = 4  /* E - Keep current direction */
} MC_DIRECTION;

/**
 * MC_EXECUTION_MODE - Execution timing for parameter write operations.
 */
typedef enum {
    mcImmediately = 0, /* E - Apply immediately, may affect ongoing motion */
    mcQueued      = 1  /* E - Queue: same as mcBuffered mode */
} MC_EXECUTION_MODE;

/**
 * MC_SOURCE - Data source selector for MC_ReadMotionState.
 */
typedef enum {
    mcCommandedValue = 0, /* E - Commanded (reference generator) value */
    mcSetValue       = 1, /* E - Set value (profile generator output) */
    mcActualValue    = 2  /* E - Actual value (from feedback) */
} MC_SOURCE;

/**
 * MC_AXIS_STATE - Internal axis state machine states.
 * See Figure 2: FB State Diagram in PLCopen Part 1 v2.0.
 */
typedef enum {
    MC_AXIS_DISABLED            = 0, /* Initial state: power off, no error */
    MC_AXIS_STANDSTILL          = 1, /* Power on, no motion active */
    MC_AXIS_HOMING              = 2, /* MC_Home is active */
    MC_AXIS_STOPPING            = 3, /* MC_Stop active (no other motion allowed) */
    MC_AXIS_DISCRETE_MOTION     = 4, /* MC_MoveAbsolute, MC_MoveRelative, MC_Halt, etc. */
    MC_AXIS_CONTINUOUS_MOTION   = 5, /* MC_MoveVelocity, MC_TorqueControl, etc. */
    MC_AXIS_SYNCHRONIZED_MOTION = 6, /* MC_GearIn, MC_CamIn (slave axis) */
    MC_AXIS_ERRORSTOP           = 7  /* Highest priority: axis error occurred */
} MC_AXIS_STATE;

/*===========================================================================
 * AXIS_REF - Axis Reference Data Structure
 *
 * Content is implementation dependent per PLCopen spec section 2.4.3.
 * This structure holds all data needed to represent one logical axis.
 * The actual hardware coupling is done outside this library.
 *===========================================================================*/
typedef struct {
    /* --- Identity --- */
    uint16_t      AxisNo;            /* Axis identifier (0-based) */

    /* --- State machine --- */
    MC_AXIS_STATE State;             /* Current PLCopen state machine state */

    /* --- Feedback (actual values from hardware) --- */
    float         ActualPosition;   /* Actual position [u] */
    float         ActualVelocity;   /* Actual velocity [u/s], signed */
    float         ActualTorque;     /* Actual torque / force, signed */

    /* --- Setpoints (profile generator outputs) --- */
    float         CommandedPosition; /* Commanded position [u] */
    float         CommandedVelocity; /* Commanded velocity [u/s] */

    /* --- Override factors (default 1.0, range 0.0..1.0) --- */
    float         VelFactor;        /* Velocity override factor */
    float         AccFactor;        /* Acceleration/deceleration override factor */
    float         JerkFactor;       /* Jerk override factor */

    /* --- Status flags --- */
    bool          PowerOn;          /* Power stage is switched ON */
    bool          IsHomed;          /* Absolute reference position is known */
    bool          Error;            /* Axis-level error is active */
    bool          Simulation;       /* TRUE if axis is running in simulation */

    /* --- Error information --- */
    uint16_t      AxisErrorID;      /* Axis error code (vendor/implementation specific) */

    /* --- Hardware I/O signals --- */
    bool          HomeAbsSwitch;    /* Home / absolute reference switch active */
    bool          LimitSwitchPos;   /* Positive hardware end switch active */
    bool          LimitSwitchNeg;   /* Negative hardware end switch active */
    bool          CommunicationReady; /* Drive communication is ready */
    bool          ReadyForPowerOn;  /* Drive is ready to be enabled */
    bool          AxisWarning;      /* Non-fatal warning present on axis */
} AXIS_REF;

/*===========================================================================
 * 3.1  MC_Power — Enable / Disable power stage
 *
 * 'Enable' is level-sensitive. When Enable=TRUE and axis is Disabled,
 * the state transitions to Standstill. When Enable=FALSE from any state
 * (except ErrorStop) the axis transitions to Disabled.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool     Enable;          /* B - Level: enable power stage */
    bool     EnablePositive;  /* E - Level: allow motion in positive direction */
    bool     EnableNegative;  /* E - Level: allow motion in negative direction */

    /* VAR_OUTPUT */
    bool     Status;          /* B - Effective state of power stage */
    bool     Valid;           /* E - Valid set of outputs available */
    bool     Error;           /* B - FB-level error */
    uint16_t ErrorID;         /* E - FB error identification */

    /* Internal */
    bool     _prevEnable;
} MC_Power;

void MC_Power_Call(MC_Power *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.2  MC_Home — Execute homing sequence
 *
 * Rising edge of Execute triggers the homing. When the reference signal
 * is detected 'Position' is set as the new absolute position.
 * Transitions axis: Standstill → Homing → Standstill (Done).
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool           Execute;    /* B - Rising edge starts homing */
    float          Position;   /* B - Absolute position set at home signal [u] */
    MC_BUFFER_MODE BufferMode; /* E - Buffer mode */

    /* VAR_OUTPUT */
    bool     Done;            /* B - Home reference found and set successfully */
    bool     Busy;            /* E - FB not finished */
    bool     Active;          /* E - FB has control of axis */
    bool     CommandAborted;  /* E - Command aborted by another command */
    bool     Error;           /* B - FB-level error */
    uint16_t ErrorID;         /* E - Error identification */

    /* Internal */
    bool     _prevExecute;
} MC_Home;

void MC_Home_Call(MC_Home *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.3  MC_Stop — Controlled emergency stop
 *
 * Transfers axis to 'Stopping' state. While Execute=TRUE, no other motion
 * command is accepted. When Done=TRUE AND Execute=FALSE, axis → Standstill.
 * This FB is intended for emergency / exception use.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool     Execute;      /* B - Rising edge triggers stop */
    float    Deceleration; /* E - Deceleration rate [u/s^2], always positive */
    float    Jerk;         /* E - Jerk limit [u/s^3], always positive */

    /* VAR_OUTPUT */
    bool     Done;            /* B - Zero velocity reached */
    bool     Busy;            /* E - FB not finished */
    bool     CommandAborted;  /* E - Aborted (only by power-off) */
    bool     Error;           /* B - FB-level error */
    uint16_t ErrorID;         /* E - Error identification */

    /* Internal */
    bool     _prevExecute;
} MC_Stop;

void MC_Stop_Call(MC_Stop *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.4  MC_Halt — Controlled stop returning to Standstill
 *
 * Stops the axis under normal conditions. Unlike MC_Stop, another motion
 * command CAN interrupt MC_Halt during deceleration.
 * Transitions axis: any motion state → DiscreteMotion → Standstill.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool           Execute;      /* B - Rising edge triggers halt */
    float          Deceleration; /* E - Deceleration [u/s^2] */
    float          Jerk;         /* E - Jerk [u/s^3] */
    MC_BUFFER_MODE BufferMode;   /* E - Buffer mode */

    /* VAR_OUTPUT */
    bool     Done;            /* B - Zero velocity reached */
    bool     Busy;            /* E - FB not finished */
    bool     Active;          /* E - FB has control of axis */
    bool     CommandAborted;  /* E - Aborted by another command */
    bool     Error;           /* B - FB-level error */
    uint16_t ErrorID;         /* E - Error identification */

    /* Internal */
    bool     _prevExecute;
} MC_Halt;

void MC_Halt_Call(MC_Halt *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.5  MC_MoveAbsolute — Move to absolute position
 *
 * Commanded motion to an absolute 'Position'. Completes with velocity=0.
 * Transitions: Standstill / DiscreteMotion → DiscreteMotion → Standstill.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool           Execute;          /* B - Rising edge starts motion */
    bool           ContinuousUpdate; /* E - Continuously update parameters */
    float          Position;         /* B - Target position [u] */
    float          Velocity;         /* B - Maximum velocity [u/s], positive */
    float          Acceleration;     /* E - Acceleration [u/s^2], always positive */
    float          Deceleration;     /* E - Deceleration [u/s^2], always positive */
    float          Jerk;             /* E - Jerk [u/s^3], always positive */
    MC_DIRECTION   Direction;        /* B - Direction (for modulo axes) */
    MC_BUFFER_MODE BufferMode;       /* E - Buffer mode */

    /* VAR_OUTPUT */
    bool     Done;            /* B - Target position reached */
    bool     Busy;            /* E - FB not finished */
    bool     Active;          /* E - FB has control of axis */
    bool     CommandAborted;  /* E - Aborted by another command */
    bool     Error;           /* B - FB-level error */
    uint16_t ErrorID;         /* E - Error identification */

    /* Internal */
    bool     _prevExecute;
} MC_MoveAbsolute;

void MC_MoveAbsolute_Call(MC_MoveAbsolute *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.6  MC_MoveRelative — Move relative distance from current set position
 *
 * 'Distance' is relative to the set position at time of Execute rising edge.
 * Completes with velocity=0.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool           Execute;          /* B - Rising edge starts motion */
    bool           ContinuousUpdate; /* E - Continuously update parameters */
    float          Distance;         /* B - Relative distance [u] */
    float          Velocity;         /* E - Maximum velocity [u/s] */
    float          Acceleration;     /* E - Acceleration [u/s^2] */
    float          Deceleration;     /* E - Deceleration [u/s^2] */
    float          Jerk;             /* E - Jerk [u/s^3] */
    MC_BUFFER_MODE BufferMode;       /* E - Buffer mode */

    /* VAR_OUTPUT */
    bool     Done;            /* B - Target distance reached */
    bool     Busy;            /* E - FB not finished */
    bool     Active;          /* E - FB has control of axis */
    bool     CommandAborted;  /* E - Aborted by another command */
    bool     Error;           /* B - FB-level error */
    uint16_t ErrorID;         /* E - Error identification */

    /* Internal */
    bool     _prevExecute;
    float    _targetPosition; /* Computed at Execute rising edge */
} MC_MoveRelative;

void MC_MoveRelative_Call(MC_MoveRelative *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.7  MC_MoveAdditive — Add relative distance to most recent commanded pos
 *
 * Adds 'Distance' to the most recent commanded position.
 * If axis is in ContinuousMotion, adds to set position at time of Execute.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool           Execute;          /* B - Rising edge starts motion */
    bool           ContinuousUpdate; /* E - Continuously update parameters */
    float          Distance;         /* B - Additional relative distance [u] */
    float          Velocity;         /* E - Maximum velocity [u/s] */
    float          Acceleration;     /* E - Acceleration [u/s^2] */
    float          Deceleration;     /* E - Deceleration [u/s^2] */
    float          Jerk;             /* E - Jerk [u/s^3] */
    MC_BUFFER_MODE BufferMode;       /* E - Buffer mode */

    /* VAR_OUTPUT */
    bool     Done;            /* B - Target distance reached */
    bool     Busy;            /* E - FB not finished */
    bool     Active;          /* E - FB has control of axis */
    bool     CommandAborted;  /* E - Aborted by another command */
    bool     Error;           /* B - FB-level error */
    uint16_t ErrorID;         /* E - Error identification */

    /* Internal */
    bool     _prevExecute;
    float    _targetPosition;
} MC_MoveAdditive;

void MC_MoveAdditive_Call(MC_MoveAdditive *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.8  MC_MoveSuperimposed — Superimpose relative distance on existing motion
 *
 * Adds a relative motion on top of an ongoing motion WITHOUT interrupting it.
 * 'VelocityDiff' is the additional velocity contribution.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool     Execute;          /* B - Rising edge starts superimposed motion */
    bool     ContinuousUpdate; /* E - Continuously update parameters */
    float    Distance;         /* B - Superimposed distance [u] */
    float    VelocityDiff;     /* E - Velocity difference of additional motion [u/s] */
    float    Acceleration;     /* E - Additional acceleration [u/s^2] */
    float    Deceleration;     /* E - Additional deceleration [u/s^2] */
    float    Jerk;             /* E - Additional jerk [u/s^3] */

    /* VAR_OUTPUT */
    bool     Done;            /* B - Superimposed distance covered */
    bool     Busy;            /* E - FB not finished */
    bool     Active;          /* E - FB has control */
    bool     CommandAborted;  /* E - Aborted by another command */
    bool     Error;           /* B - FB-level error */
    uint16_t ErrorID;         /* E - Error identification */
    float    CoveredDistance; /* E - Distance covered so far [u] */

    /* Internal */
    bool     _prevExecute;
    float    _coveredSoFar;
} MC_MoveSuperimposed;

void MC_MoveSuperimposed_Call(MC_MoveSuperimposed *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.9  MC_HaltSuperimposed — Stop all superimposed motions
 *
 * Halts any active superimposed motion. The underlying motion is NOT
 * interrupted.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool     Execute;      /* B - Rising edge triggers halt of superimposed motion */
    float    Deceleration; /* E - Deceleration [u/s^2] */
    float    Jerk;         /* E - Jerk [u/s^3] */

    /* VAR_OUTPUT */
    bool     Done;            /* B - Superimposed motion halted */
    bool     Busy;            /* E - FB not finished */
    bool     Active;          /* E - FB has control */
    bool     CommandAborted;  /* E - Aborted by another command */
    bool     Error;           /* B - FB-level error */
    uint16_t ErrorID;         /* E - Error identification */

    /* Internal */
    bool     _prevExecute;
} MC_HaltSuperimposed;

void MC_HaltSuperimposed_Call(MC_HaltSuperimposed *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.10  MC_MoveVelocity — Continuous velocity motion (never-ending)
 *
 * Commands axis to run at specified 'Velocity'. The motion does not stop
 * by itself — another FB must interrupt it.
 * Transitions: Standstill / DiscreteMotion → ContinuousMotion.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool           Execute;          /* B - Rising edge starts motion */
    bool           ContinuousUpdate; /* E - Continuously update parameters */
    float          Velocity;         /* B - Target velocity [u/s], signed */
    float          Acceleration;     /* E - Acceleration [u/s^2] */
    float          Deceleration;     /* E - Deceleration [u/s^2] */
    float          Jerk;             /* E - Jerk [u/s^3] */
    MC_DIRECTION   Direction;        /* E - Direction (1-of-3 values, not mcShortestWay) */
    MC_BUFFER_MODE BufferMode;       /* E - Buffer mode */

    /* VAR_OUTPUT */
    bool     InVelocity;      /* B - Commanded velocity reached */
    bool     Busy;            /* E - FB not finished */
    bool     Active;          /* E - FB has control of axis */
    bool     CommandAborted;  /* E - Aborted by another command */
    bool     Error;           /* B - FB-level error */
    uint16_t ErrorID;         /* E - Error identification */

    /* Internal */
    bool     _prevExecute;
} MC_MoveVelocity;

void MC_MoveVelocity_Call(MC_MoveVelocity *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.11  MC_MoveContinuousAbsolute — Move to absolute position, keep EndVelocity
 *
 * Like MC_MoveAbsolute but arrives at position with non-zero EndVelocity.
 * Axis continues in ContinuousMotion at EndVelocity after position is reached.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool           Execute;          /* B - Rising edge starts motion */
    bool           ContinuousUpdate; /* E - Continuously update parameters */
    float          Position;         /* B - Target position [u] */
    float          EndVelocity;      /* B - End velocity [u/s], signed */
    float          Velocity;         /* B - Maximum velocity [u/s] */
    float          Acceleration;     /* E - Acceleration [u/s^2] */
    float          Deceleration;     /* E - Deceleration [u/s^2] */
    float          Jerk;             /* E - Jerk [u/s^3] */
    MC_DIRECTION   Direction;        /* E - Direction */
    MC_BUFFER_MODE BufferMode;       /* E - Buffer mode */

    /* VAR_OUTPUT */
    bool     InEndVelocity;   /* B - Position reached and running at EndVelocity */
    bool     Busy;            /* E - FB not finished */
    bool     Active;          /* E - FB has control of axis */
    bool     CommandAborted;  /* E - Aborted by another command */
    bool     Error;           /* B - FB-level error */
    uint16_t ErrorID;         /* B - Error identification */

    /* Internal */
    bool     _prevExecute;
} MC_MoveContinuousAbsolute;

void MC_MoveContinuousAbsolute_Call(MC_MoveContinuousAbsolute *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.12  MC_MoveContinuousRelative — Move relative distance, keep EndVelocity
 *
 * Like MC_MoveRelative but arrives at target position with non-zero EndVelocity.
 * Axis continues in ContinuousMotion at EndVelocity after position is reached.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool           Execute;          /* B - Rising edge starts motion */
    bool           ContinuousUpdate; /* E - Continuously update parameters */
    float          Distance;         /* B - Relative distance [u] */
    float          EndVelocity;      /* B - End velocity [u/s], signed */
    float          Velocity;         /* B - Maximum velocity [u/s] */
    float          Acceleration;     /* E - Acceleration [u/s^2] */
    float          Deceleration;     /* E - Deceleration [u/s^2] */
    float          Jerk;             /* E - Jerk [u/s^3] */
    MC_BUFFER_MODE BufferMode;       /* E - Buffer mode */

    /* VAR_OUTPUT */
    bool     InEndVelocity;   /* B - Distance reached and running at EndVelocity */
    bool     Busy;            /* E - FB not finished */
    bool     Active;          /* E - FB has control of axis */
    bool     CommandAborted;  /* E - Aborted by another command */
    bool     Error;           /* B - FB-level error */
    uint16_t ErrorID;         /* B - Error identification */

    /* Internal */
    bool     _prevExecute;
    float    _targetPosition;
} MC_MoveContinuousRelative;

void MC_MoveContinuousRelative_Call(MC_MoveContinuousRelative *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.17  MC_SetPosition — Shift the axis coordinate system
 *
 * Shifts both set-point and actual position by the same value without
 * causing any physical movement. Used for re-calibration.
 * Relative=FALSE: sets actual position to 'Position' (absolute).
 * Relative=TRUE:  adds 'Position' to actual position (offset).
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool              Execute;        /* B - Rising edge applies position shift */
    float             Position;       /* B - New position or offset [u] */
    bool              Relative;       /* E - FALSE=absolute, TRUE=relative */
    MC_EXECUTION_MODE ExecutionMode;  /* E - mcImmediately or mcQueued */

    /* VAR_OUTPUT */
    bool     Done;    /* B - Position has been set */
    bool     Busy;    /* E - FB not finished */
    bool     Error;   /* B - FB-level error */
    uint16_t ErrorID; /* E - Error identification */

    /* Internal */
    bool     _prevExecute;
} MC_SetPosition;

void MC_SetPosition_Call(MC_SetPosition *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.18  MC_SetOverride — Set velocity/acceleration/jerk override factors
 *
 * 'Enable' is level-sensitive. While Enable=TRUE, override factors are
 * applied continuously. Default factor value is 1.0.
 * Range: 0.0..1.0 (values >1.0 are vendor-specific; <0.0 not allowed).
 * VelFactor=0.0 stops the axis without going to Standstill state.
 * AccFactor and JerkFactor must not be 0.0.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool     Enable;      /* B - Level: apply override factors */
    float    VelFactor;   /* B - Velocity override factor */
    float    AccFactor;   /* E - Acceleration/deceleration override factor */
    float    JerkFactor;  /* E - Jerk override factor */

    /* VAR_OUTPUT */
    bool     Enabled;     /* B - Override factors are set successfully */
    bool     Busy;        /* E - FB not finished */
    bool     Error;       /* B - FB-level error */
    uint16_t ErrorID;     /* E - Error identification */

    /* Internal */
    bool     _prevEnable;
} MC_SetOverride;

void MC_SetOverride_Call(MC_SetOverride *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.19  MC_ReadParameter — Read a REAL-valued axis parameter
 *
 * 'Enable' is level-sensitive. While Enable=TRUE 'Value' is continuously
 * updated. See Table 5 for standardized parameter numbers.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool     Enable;           /* B - Level: read parameter continuously */
    int16_t  ParameterNumber;  /* B - Parameter number (Table 5 in PLCopen spec) */

    /* VAR_OUTPUT */
    bool     Valid;   /* B - Valid output available */
    bool     Busy;    /* E - FB not finished */
    bool     Error;   /* B - FB-level error */
    uint16_t ErrorID; /* E - Error identification */
    float    Value;   /* B - Value of the parameter */

    /* Internal */
    bool     _prevEnable;
} MC_ReadParameter;

void MC_ReadParameter_Call(MC_ReadParameter *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.19  MC_ReadBoolParameter — Read a BOOL-valued axis parameter
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool     Enable;           /* B - Level: read parameter continuously */
    int16_t  ParameterNumber;  /* B - Parameter number */

    /* VAR_OUTPUT */
    bool     Valid;   /* B - Valid output available */
    bool     Busy;    /* E - FB not finished */
    bool     Error;   /* B - FB-level error */
    uint16_t ErrorID; /* E - Error identification */
    bool     Value;   /* B - Bool value of the parameter */

    /* Internal */
    bool     _prevEnable;
} MC_ReadBoolParameter;

void MC_ReadBoolParameter_Call(MC_ReadBoolParameter *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.20  MC_WriteParameter — Write a REAL-valued axis parameter
 *
 * Rising edge of Execute writes 'Value' to the parameter specified by
 * 'ParameterNumber'. See Table 5 (only R/W entries may be written).
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool              Execute;         /* B - Rising edge writes parameter */
    int16_t           ParameterNumber; /* B - Parameter number */
    float             Value;           /* B - New value to write */
    MC_EXECUTION_MODE ExecutionMode;   /* E - Immediate or queued */

    /* VAR_OUTPUT */
    bool     Done;    /* B - Parameter written successfully */
    bool     Busy;    /* E - FB not finished */
    bool     Error;   /* B - FB-level error */
    uint16_t ErrorID; /* E - Error identification */

    /* Internal */
    bool     _prevExecute;
} MC_WriteParameter;

void MC_WriteParameter_Call(MC_WriteParameter *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.20  MC_WriteBoolParameter — Write a BOOL-valued axis parameter
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool              Execute;         /* B - Rising edge writes parameter */
    int16_t           ParameterNumber; /* B - Parameter number */
    bool              Value;           /* B - New bool value to write */
    MC_EXECUTION_MODE ExecutionMode;   /* E - Immediate or queued */

    /* VAR_OUTPUT */
    bool     Done;    /* B - Parameter written successfully */
    bool     Busy;    /* E - FB not finished */
    bool     Error;   /* B - FB-level error */
    uint16_t ErrorID; /* E - Error identification */

    /* Internal */
    bool     _prevExecute;
} MC_WriteBoolParameter;

void MC_WriteBoolParameter_Call(MC_WriteBoolParameter *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.24  MC_ReadActualPosition — Read actual axis position
 *
 * 'Enable' is level-sensitive. Provides the actual position feedback.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool     Enable; /* B - Level: read position continuously */

    /* VAR_OUTPUT */
    bool     Valid;     /* B - Valid output available */
    bool     Busy;      /* E - FB not finished */
    bool     Error;     /* B - FB-level error */
    uint16_t ErrorID;   /* E - Error identification */
    float    Position;  /* B - Actual position [u] */

    /* Internal */
    bool     _prevEnable;
} MC_ReadActualPosition;

void MC_ReadActualPosition_Call(MC_ReadActualPosition *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.25  MC_ReadActualVelocity — Read actual axis velocity
 *
 * 'Enable' is level-sensitive. Output 'Velocity' can be signed.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool     Enable; /* B - Level: read velocity continuously */

    /* VAR_OUTPUT */
    bool     Valid;     /* B - Valid output available */
    bool     Busy;      /* E - FB not finished */
    bool     Error;     /* B - FB-level error */
    uint16_t ErrorID;   /* E - Error identification */
    float    Velocity;  /* B - Actual velocity [u/s], signed */

    /* Internal */
    bool     _prevEnable;
} MC_ReadActualVelocity;

void MC_ReadActualVelocity_Call(MC_ReadActualVelocity *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.26  MC_ReadActualTorque — Read actual axis torque / force
 *
 * 'Enable' is level-sensitive. Output 'Torque' can be signed.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool     Enable; /* B - Level: read torque continuously */

    /* VAR_OUTPUT */
    bool     Valid;    /* B - Valid output available */
    bool     Busy;     /* E - FB not finished */
    bool     Error;    /* B - FB-level error */
    uint16_t ErrorID;  /* E - Error identification */
    float    Torque;   /* B - Actual torque / force, signed */

    /* Internal */
    bool     _prevEnable;
} MC_ReadActualTorque;

void MC_ReadActualTorque_Call(MC_ReadActualTorque *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.27  MC_ReadStatus — Read detailed axis state diagram status
 *
 * Returns which state the axis is currently in. Multiple outputs can be
 * FALSE at the same time (only one state is active at a time).
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool     Enable; /* B - Level: read status continuously */

    /* VAR_OUTPUT */
    bool     Valid;              /* B - Valid set of outputs available */
    bool     Busy;               /* E - FB not finished */
    bool     Error;              /* B - FB-level error */
    uint16_t ErrorID;            /* E - Error identification */
    bool     ErrorStop;          /* B - Axis is in ErrorStop state */
    bool     Disabled;           /* B - Axis is in Disabled state */
    bool     Stopping;           /* B - Axis is in Stopping state */
    bool     Homing;             /* E - Axis is in Homing state */
    bool     Standstill;         /* B - Axis is in Standstill state */
    bool     DiscreteMotion;     /* E - Axis is in DiscreteMotion state */
    bool     ContinuousMotion;   /* E - Axis is in ContinuousMotion state */
    bool     SynchronizedMotion; /* E - Axis is in SynchronizedMotion state */

    /* Internal */
    bool     _prevEnable;
} MC_ReadStatus;

void MC_ReadStatus_Call(MC_ReadStatus *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.28  MC_ReadMotionState — Read motion details (accelerating, direction…)
 *
 * 'Source' selects whether commanded, set, or actual values are evaluated.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool      Enable;  /* B - Level: read motion state continuously */
    MC_SOURCE Source;  /* E - Data source selector */

    /* VAR_OUTPUT */
    bool     Valid;              /* B - Valid output available */
    bool     Busy;               /* E - FB not finished */
    bool     Error;              /* B - FB-level error */
    uint16_t ErrorID;            /* E - Error identification */
    bool     ConstantVelocity;   /* E - Velocity is constant (may be 0) */
    bool     Accelerating;       /* E - Absolute velocity is increasing */
    bool     Decelerating;       /* E - Absolute velocity is decreasing */
    bool     DirectionPositive;  /* E - Position is increasing */
    bool     DirectionNegative;  /* E - Position is decreasing */

    /* Internal */
    bool     _prevEnable;
    float    _prevVelocity;
} MC_ReadMotionState;

void MC_ReadMotionState_Call(MC_ReadMotionState *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.29  MC_ReadAxisInfo — Read axis hardware / mode information
 *
 * Provides static and dynamic axis properties: switches, communication,
 * power stage state, homing status, etc.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool     Enable; /* B - Level: read axis info continuously */

    /* VAR_OUTPUT */
    bool     Valid;              /* B - Valid output available */
    bool     Busy;               /* E - FB not finished */
    bool     Error;              /* B - FB-level error */
    uint16_t ErrorID;            /* E - Error identification */
    bool     HomeAbsSwitch;      /* E - Home/abs-reference switch active */
    bool     LimitSwitchPos;     /* E - Positive hardware end switch active */
    bool     LimitSwitchNeg;     /* E - Negative hardware end switch active */
    bool     Simulation;         /* E - Axis is in simulation mode */
    bool     CommunicationReady; /* E - Network/drive communication ready */
    bool     ReadyForPowerOn;    /* E - Drive ready to be enabled */
    bool     PowerOn;            /* E - Power stage is switched ON */
    bool     IsHomed;            /* E - Absolute position reference is known */
    bool     AxisWarning;        /* E - Non-fatal warning on axis */

    /* Internal */
    bool     _prevEnable;
} MC_ReadAxisInfo;

void MC_ReadAxisInfo_Call(MC_ReadAxisInfo *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.30  MC_ReadAxisError — Read axis-level error code
 *
 * Presents axis errors NOT related to Function Block instances
 * (e.g. following error, over-temperature, drive fault).
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool     Enable; /* B - Level: read axis error continuously */

    /* VAR_OUTPUT */
    bool     Valid;        /* B - Valid output available */
    bool     Busy;         /* E - FB not finished */
    bool     Error;        /* B - FB-level error */
    uint16_t ErrorID;      /* B - FB error identification */
    uint16_t AxisErrorID;  /* B - Axis error code (vendor/implementation specific) */

    /* Internal */
    bool     _prevEnable;
} MC_ReadAxisError;

void MC_ReadAxisError_Call(MC_ReadAxisError *inst, AXIS_REF *axis);

/*===========================================================================
 * 3.31  MC_Reset — Reset axis from ErrorStop state
 *
 * Clears all internal axis-related errors and transitions from ErrorStop
 * to Standstill (if MC_Power.Enable=TRUE) or Disabled.
 * Does NOT affect FB instance error outputs.
 *===========================================================================*/
typedef struct {
    /* VAR_INPUT */
    bool     Execute; /* B - Rising edge resets axis errors */

    /* VAR_OUTPUT */
    bool     Done;    /* B - Axis reset, Standstill or Disabled reached */
    bool     Busy;    /* E - FB not finished */
    bool     Error;   /* B - FB-level error */
    uint16_t ErrorID; /* E - Error identification */

    /* Internal */
    bool     _prevExecute;
} MC_Reset;

void MC_Reset_Call(MC_Reset *inst, AXIS_REF *axis);

/*===========================================================================
 * AXIS_REF helper — Initialize axis structure to safe defaults
 *===========================================================================*/
void AXIS_REF_Init(AXIS_REF *axis, uint16_t axisNo);

#endif /* KRONMOTION_H */
