/*===========================================================================
 * kron_axis.h  --  KronMotion axis engine
 *
 * The motion layer that sits on top of the OnlineTrajectoryGenerator core
 * (TrajectoryGenerator.h, fetched by CMake).  The core answers one question
 * per scan and owns nothing; everything a motion controller is — the PLCopen
 * state diagram, Done / Busy / Active / CommandAborted, the command buffer,
 * blending, MC_Stop priority, the CiA402 drive state machine and the process
 * image — lives here.
 *
 * Single task.  There is no slow/fast split and no cmd/sts handshake: the
 * MC_xxx function blocks, the trajectory core and the fieldbus exchange all
 * run in the same cycle, in this order:
 *
 *     void motion_task(void)            // every KRON_CYCLE seconds
 *     {
 *         KronMotion_ReadInputs(axes, NUM_AXES);   // fieldbus -> axis
 *         plc_program();                           // MC_xxx_Call(...)
 *         KronMotion_WriteOutputs(axes, NUM_AXES, dt);  // core -> fieldbus
 *     }
 *
 * Because the core re-decides the whole trajectory every scan, a command may
 * change on any cycle: a new target, a new velocity limit or an override takes
 * effect immediately and jerk-limited, with no profile to re-plan.
 *
 * Units are the axis' user units [u]; all values are double.
 *===========================================================================*/

#ifndef KRON_AXIS_H
#define KRON_AXIS_H

#include <stdbool.h>
#include <stdint.h>

#include "kron_pi.h"                 /* process image, CiA402 slot, HAL      */
#include "TrajectoryGenerator.h"     /* the core: GenerateTrajectory()        */

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * PLCopen enumerations (Part 1 v2.0, chapter 2)
 *===========================================================================*/
typedef enum {
    mcAborting         = 0,   /* start immediately, clear the buffer          */
    mcBuffered         = 1,   /* start when the running command is Done       */
    mcBlendingLow      = 2,   /* blend at the lowest velocity of both FBs     */
    mcBlendingPrevious = 3,   /* blend at the velocity of the running FB      */
    mcBlendingNext     = 4,   /* blend at the velocity of the queued FB       */
    mcBlendingHigh     = 5    /* blend at the highest velocity of both FBs    */
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

/* The state diagram of Figure 2.  The axis is always in exactly one state. */
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
 * Error codes
 *
 * Block local, as in error_codes.xml: code 1 in MC_Stop is unrelated to code 1
 * in MC_MoveAbsolute.  Codes 1..99 are raised by the FB or by the submit check,
 * codes 100+ are axis errors propagated from the drive.
 *===========================================================================*/
#define MC_ERR_NONE          0u
#define MC_ERR_PARAM         1u   /* a parameter is missing or out of range   */
#define MC_ERR_STATE         2u   /* not allowed in the axis' current state   */
#define MC_ERR_NOT_HOMED     3u   /* absolute move on an unreferenced axis    */
#define MC_ERR_QUEUE_FULL    4u   /* the command buffer has no room left      */
#define MC_ERR_STOP_ACTIVE   5u   /* MC_Stop holds the axis, see 3.3 note 2   */
#define MC_ERR_AXIS        100u   /* drive fault / axis error                 */

/*===========================================================================
 * KRON_MOTION_REQ — one motion command
 *
 * A request lives inside the function block instance that issued it, so the
 * axis never allocates and the result always finds its way back to the right
 * FB: the axis writes State / ErrorID into the request, the FB reads them.
 *
 * The FB fills the input half at the rising edge of Execute and calls
 * KronAxis_Submit().  With ContinuousUpdate it may keep writing the parameter
 * fields while the request is ACTIVE; the core picks them up on the next scan.
 *===========================================================================*/
typedef enum {
    KRON_MOTION_NONE = 0,
    KRON_MOTION_HOME,
    KRON_MOTION_HALT,
    KRON_MOTION_STOP,
    KRON_MOTION_MOVE_ABSOLUTE,
    KRON_MOTION_MOVE_RELATIVE,
    KRON_MOTION_MOVE_ADDITIVE,
    KRON_MOTION_MOVE_VELOCITY,
    KRON_MOTION_SUPERIMPOSED,
    KRON_MOTION_HALT_SUPERIMPOSED
} KRON_MOTION_KIND;

typedef enum {
    KRON_REQ_IDLE = 0,    /* not submitted, or already collected by the FB    */
    KRON_REQ_BUFFERED,    /* in the queue, waiting for the running command    */
    KRON_REQ_ACTIVE,      /* this request owns the axis                        */
    KRON_REQ_DONE,        /* finished successfully                             */
    KRON_REQ_ABORTED,     /* another command took the axis                     */
    KRON_REQ_ERROR        /* rejected or failed, see ErrorID                   */
} KRON_REQ_STATE;

typedef struct KRON_MOTION_REQ {
    /* ── input half: written by the FB ──────────────────────────────────── */
    KRON_MOTION_KIND Kind;
    double           Position;        /* [u]   absolute target / distance     */
    double           Velocity;        /* [u/s] signed for MOVE_VELOCITY       */
    double           Acceleration;    /* [u/s^2] always positive              */
    double           Deceleration;    /* [u/s^2] always positive              */
    double           Jerk;            /* [u/s^3] always positive, 0 = axis    */
    MC_DIRECTION     Direction;
    MC_BUFFER_MODE   BufferMode;
    bool             ContinuousUpdate;

    /* ── output half: written by the axis ───────────────────────────────── */
    KRON_REQ_STATE   State;
    uint16_t         ErrorID;
    bool             InVelocity;      /* set velocity equals the commanded one */
    double           CoveredDistance; /* superimposed contribution so far [u]  */

    /* ── axis private ───────────────────────────────────────────────────── */
    double           _target;         /* absolute target resolved at start [u] */
    bool             _resolved;       /* _target is valid                      */
} KRON_MOTION_REQ;

/*===========================================================================
 * AXIS_REF — the axis
 *
 * Held by the application (usually the generated plc.c), passed to every FB.
 * Limits and scaling are configured once at startup; everything else is
 * written by KronAxis_ReadInput / KronAxis_WriteOutput.
 *===========================================================================*/
#ifndef KRON_MOTION_QUEUE_DEPTH
#  define KRON_MOTION_QUEUE_DEPTH 4
#endif

typedef struct {

    /* ── identity and hardware link ─────────────────────────────────────── */
    uint16_t          AxisNo;
    KRON_SERVO_SLOT  *slot;             /* into Kron_PI.servo[n], NULL = none */
    bool              Simulation;       /* run the axis without a drive       */

    /* ── axis limits, configured at startup ─────────────────────────────── */
    double            MaxVelocity;      /* [u/s]   0 = FB must supply its own  */
    double            MaxAcceleration;  /* [u/s^2]                             */
    double            MaxDeceleration;  /* [u/s^2]                             */
    double            MaxJerk;          /* [u/s^3] 0 = effectively trapezoidal */
    double            SwLimitNegative;  /* [u] software position limits        */
    double            SwLimitPositive;  /* [u]                                 */
    bool              EnableLimitNegative;
    bool              EnableLimitPositive;
    bool              EnablePosLagMonitoring;
    double            MaxPositionLag;   /* [u] following error that trips the axis */
    double            InPositionWindow; /* [u]   a move is Done inside this    */
    double            InVelocityWindow; /* [u/s] InVelocity inside this        */

    /* ── override factors, written by MC_SetOverride ────────────────────── */
    double            VelFactor;        /* [0.0 .. 1.0], 1.0 at init           */
    double            AccFactor;
    double            JerkFactor;

    /* ── scaling metadata ───────────────────────────────────────────────── */
    double            GearRatio;        /* user units per motor revolution     */
    KRON_ENCODER_TYPE EncoderType;
    double            PositionOffset;   /* [u] coordinate shift, MC_SetPosition */

    /* ── actual values, from the feedback each cycle ────────────────────── */
    double            ActualPosition;   /* [u]                                 */
    double            ActualVelocity;   /* [u/s] signed                        */
    double            ActualTorque;     /* [% of rated] signed                 */

    /* ── set values, the trajectory core's answer for this cycle ────────── */
    MotionState       Set;              /* Position / Velocity / Acceleration  */
    double            SetJerk;          /* [u/s^3] what was really applied     */
    double            BrakeDistance;    /* [u] distance to standstill from here*/
    double            CommandedPosition;/* the target of the running command   */
    double            CommandedVelocity;

    /* ── superimposed generator, runs beside the main one ───────────────── */
    MotionState       Super;            /* offset, its own little trajectory   */
    double            SuperTarget;      /* [u] relative to the start of the FB */

    /* ── state ──────────────────────────────────────────────────────────── */
    MC_AXIS_STATE     State;
    bool              PowerEnabled;     /* MC_Power.Enable                     */
    bool              EnablePositive;   /* MC_Power.EnablePositive             */
    bool              EnableNegative;   /* MC_Power.EnableNegative             */
    bool              PowerStatus;      /* effective state of the power stage  */
    bool              IsHomed;
    bool              AxisWarning;
    bool              AxisError;
    uint16_t          AxisErrorID;

    /* ── drive diagnostics ──────────────────────────────────────────────── */
    uint16_t          drv_StatusWord;   /* 0x6041, last received               */
    uint16_t          drv_ControlWord;  /* 0x6040, last sent                   */

    /* ── command queue ──────────────────────────────────────────────────── */
    KRON_MOTION_REQ  *Active;
    KRON_MOTION_REQ  *Buffer[KRON_MOTION_QUEUE_DEPTH];
    uint8_t           BufferCount;
    KRON_MOTION_REQ  *Superimposed;     /* runs in parallel with Active        */

    /* ── engine private ─────────────────────────────────────────────────── */
    bool              StopActive;       /* MC_Stop holds the axis (3.3 note 2) */
    uint8_t           HomingPhase;      /* CiA402 mode 6 sub-state, 0 = idle   */
    double            HomePosition;     /* MC_Home.Position                    */
    double            LastCommandedPos; /* base for MC_MoveAdditive            */
    double            PrevSetVelocity;  /* for MC_ReadMotionState              */
    bool              OperationEnabled; /* drive is in Operation Enabled       */

} AXIS_REF;

/*===========================================================================
 * Axis API
 *===========================================================================*/

/* Zero the axis, put the override factors back to 1.0 and the windows to a
 * usable default.  Call once at startup, before setting the limits. */
void AXIS_REF_Init(AXIS_REF *axis, uint16_t axisNo, KRON_SERVO_SLOT *slot);

/* Hand a request to the axis.  Returns MC_ERR_NONE when it was accepted, in
 * which case req->State tells whether it went ACTIVE or into the buffer.  On a
 * rejection the error code is returned and also written to req->ErrorID. */
uint16_t KronAxis_Submit(AXIS_REF *axis, KRON_MOTION_REQ *req);

/* Drop a request the FB no longer wants (Execute fell before it started).
 * A request that is already ACTIVE keeps running - PLCopen 2.4.1: the falling
 * edge of Execute does not influence the execution of the FB. */
void KronAxis_Cancel(AXIS_REF *axis, KRON_MOTION_REQ *req);

/* Start of cycle: fieldbus image -> actual values, drive status, fault check. */
void KronAxis_ReadInput(AXIS_REF *axis);

/* End of cycle: state machine, one scan of the trajectory core, CiA402 step,
 * set values -> fieldbus image.  dt is the cycle time in seconds. */
void KronAxis_WriteOutput(AXIS_REF *axis, double dt);

/* MC_Stop holds the axis in 'Stopping' while its Execute is high; this is how
 * the FB gives it back when Execute falls and the stop ramp is finished. */
void KronAxis_ReleaseStop(AXIS_REF *axis, KRON_MOTION_REQ *req);

/* MC_Reset: clear the axis errors and leave 'ErrorStop'.  Returns true once the
 * axis has reached 'Standstill' or 'Disabled'. */
bool KronAxis_Reset(AXIS_REF *axis);

/* MC_SetPosition: shift the coordinate system by the same amount on the set and
 * the actual side, so no movement is caused. */
void KronAxis_SetPosition(AXIS_REF *axis, double position, bool relative);

/* The whole array, with the HAL exchange around it. */
void KronMotion_ReadInputs(AXIS_REF *axes, uint16_t count);
void KronMotion_WriteOutputs(AXIS_REF *axes, uint16_t count, double dt);

#ifdef __cplusplus
}
#endif

#endif /* KRON_AXIS_H */
