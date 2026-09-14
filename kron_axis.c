/*===========================================================================
 * kron_axis.c  --  KronMotion axis engine
 *
 * One cycle of one axis: read the feedback, step the CiA402 drive state
 * machine, walk the PLCopen state diagram, call the trajectory core once for
 * the running command and once more for a superimposed motion, and push the
 * result into the process image.
 *
 * The core (TrajectoryGenerator.c) is pure and keeps nothing between calls, so
 * every decision it makes is made from the axis state held here.  Nothing is
 * planned ahead: a changed target, a changed limit or a changed override is
 * simply what the next scan is handed.
 *
 * C99.  No dynamic allocation, no recursion, fixed cost per cycle.
 *===========================================================================*/

#include "kron_axis.h"

#include <math.h>
#include <string.h>

/*===========================================================================
 * Local helpers
 *===========================================================================*/
#define KM_POS_EPS   1e-9    /* [u]   below this a position difference is nothing */
#define KM_VEL_EPS   1e-9    /* [u/s] below this the axis is standing still       */

/* a jerk this much above the acceleration is trapezoidal for every practical
 * purpose - the core needs jerk >= acc, dec and refuses to degenerate */
#define KM_TRAPEZOIDAL_JERK_RATIO 1000.0

static double dmin(double a, double b) { return (b < a) ? b : a; }
static double dmax(double a, double b) { return (a < b) ? b : a; }

/* the FB value if it was given, otherwise the axis limit; an FB value above the
 * axis limit is limited by the system (PLCopen 2.4.1, 'inputs exceeding
 * application limits') rather than turned into an error */
static double pick_limit(double from_fb, double axis_limit)
{
    if (from_fb > 0.0)
    {
        if (axis_limit > 0.0 && from_fb > axis_limit)
        {
            return axis_limit;
        }
        return from_fb;
    }
    return axis_limit;
}

/*===========================================================================
 * CiA402 statusword / controlword
 *===========================================================================*/
#define CIA402_SW_FAULT           0x0008u
#define CIA402_SW_WARNING         0x0080u
#define CIA402_SW_TARGET_REACHED  0x0400u
#define CIA402_SW_HOMING_ATTAINED 0x1000u

#define CIA402_CW_DISABLE   0x0000u   /* disable voltage    */
#define CIA402_CW_SO        0x0006u   /* shutdown           */
#define CIA402_CW_EOA       0x0007u   /* switch on          */
#define CIA402_CW_OE        0x000Fu   /* enable operation   */
#define CIA402_CW_FACK      0x0080u   /* fault reset        */
#define CIA402_CW_HM_START  0x0010u   /* bit 4, start homing */

#define CIA402_MODE_CSP  8
#define CIA402_MODE_HM   6

static bool cia402_op_enabled(uint16_t sw)   { return (sw & 0x006Fu) == 0x0027u; }
static bool cia402_switched_on(uint16_t sw)  { return (sw & 0x006Fu) == 0x0023u; }
static bool cia402_ready_to_so(uint16_t sw)  { return (sw & 0x006Fu) == 0x0021u; }
static bool cia402_not_ready(uint16_t sw)    { return (sw & 0x004Fu) == 0x0000u; }
static bool cia402_fault(uint16_t sw)        { return (sw & CIA402_SW_FAULT) != 0u; }

/*===========================================================================
 * Request bookkeeping
 *===========================================================================*/
static bool req_is_position_move(const KRON_MOTION_REQ *req)
{
    return req->Kind == KRON_MOTION_MOVE_ABSOLUTE ||
           req->Kind == KRON_MOTION_MOVE_RELATIVE ||
           req->Kind == KRON_MOTION_MOVE_ADDITIVE;
}

static bool req_is_superimposed(const KRON_MOTION_REQ *req)
{
    return req->Kind == KRON_MOTION_SUPERIMPOSED ||
           req->Kind == KRON_MOTION_HALT_SUPERIMPOSED;
}

/* finish a request with a result, and let go of the axis */
static void req_finish(AXIS_REF *axis, KRON_MOTION_REQ *req, KRON_REQ_STATE result, uint16_t errorID)
{
    if (req == NULL)
    {
        return;
    }

    req->State   = result;
    req->ErrorID = errorID;

    if (axis->Active == req)
    {
        axis->Active = NULL;
    }
    if (axis->Superimposed == req)
    {
        axis->Superimposed = NULL;
    }
}

/* every queued request loses the axis; the buffer is cleared (2.4.2) */
static void buffer_clear(AXIS_REF *axis, KRON_REQ_STATE result, uint16_t errorID)
{
    uint8_t i;

    for (i = 0; i < axis->BufferCount; i++)
    {
        if (axis->Buffer[i] != NULL)
        {
            axis->Buffer[i]->State   = result;
            axis->Buffer[i]->ErrorID = errorID;
            axis->Buffer[i] = NULL;
        }
    }
    axis->BufferCount = 0;
}

/* pull the head of the queue out, shifting the rest down */
static KRON_MOTION_REQ *buffer_pop(AXIS_REF *axis)
{
    KRON_MOTION_REQ *head;
    uint8_t i;

    if (axis->BufferCount == 0u)
    {
        return NULL;
    }

    head = axis->Buffer[0];

    for (i = 1u; i < axis->BufferCount; i++)
    {
        axis->Buffer[i - 1u] = axis->Buffer[i];
    }
    axis->BufferCount--;
    axis->Buffer[axis->BufferCount] = NULL;

    return head;
}

/*===========================================================================
 * Target resolution
 *
 * A relative distance becomes an absolute target the moment the request takes
 * the axis, not when it was submitted - a buffered move is relative to where
 * the axis is when it actually starts (3.6).
 *===========================================================================*/
static double resolve_target(AXIS_REF *axis, const KRON_MOTION_REQ *req, double base)
{
    switch (req->Kind)
    {
        case KRON_MOTION_MOVE_ABSOLUTE:
            return req->Position;                           // already absolute

        case KRON_MOTION_MOVE_RELATIVE:
            return base + req->Position;                    // from where we are now

        case KRON_MOTION_MOVE_ADDITIVE:
            return axis->LastCommandedPos + req->Position;  // from the last commanded position (3.7)

        default:
            return base;
    }
}

static void activate(AXIS_REF *axis, KRON_MOTION_REQ *req)
{
    req->State      = KRON_REQ_ACTIVE;
    req->ErrorID    = MC_ERR_NONE;
    req->InVelocity = false;

    if (req_is_position_move(req))
    {
        req->_target   = resolve_target(axis, req, axis->Set.Position);
        req->_resolved = true;
        axis->CommandedPosition = req->_target;
        axis->LastCommandedPos  = req->_target;
        axis->State = MC_AXIS_DISCRETE_MOTION;
    }
    else if (req->Kind == KRON_MOTION_MOVE_VELOCITY)
    {
        req->_resolved = false;
        axis->State = MC_AXIS_CONTINUOUS_MOTION;
    }
    else if (req->Kind == KRON_MOTION_HALT)
    {
        req->_resolved = false;
        axis->State = MC_AXIS_DISCRETE_MOTION;        // 3.4: Halt runs in DiscreteMotion
    }
    else if (req->Kind == KRON_MOTION_STOP)
    {
        req->_resolved = false;
        axis->State = MC_AXIS_STOPPING;
    }
    else if (req->Kind == KRON_MOTION_HOME)
    {
        req->_resolved   = false;
        axis->State      = MC_AXIS_HOMING;
        axis->HomingPhase = 1u;
    }

    axis->Active = req;
}

/*===========================================================================
 * KronAxis_Submit
 *===========================================================================*/
static uint16_t submit_check(AXIS_REF *axis, const KRON_MOTION_REQ *req)
{
    /* MC_Stop owns the axis while its Execute is high and refuses everything
     * else (3.3 note 2).  The stop itself may be re-triggered. */
    if (axis->StopActive && req->Kind != KRON_MOTION_STOP)
    {
        return MC_ERR_STOP_ACTIVE;
    }

    if (axis->State == MC_AXIS_ERRORSTOP)
    {
        return MC_ERR_STATE;              // nothing is accepted until MC_Reset
    }

    if (axis->State == MC_AXIS_DISABLED)
    {
        return MC_ERR_STATE;              // power is off
    }

    if (axis->State == MC_AXIS_HOMING && req->Kind != KRON_MOTION_STOP)
    {
        return MC_ERR_STATE;              // the homing sequence owns the axis
    }

    if (req->Kind == KRON_MOTION_MOVE_ABSOLUTE && !axis->IsHomed &&
        axis->EncoderType == KRON_ENC_INCREMENTAL)
    {
        return MC_ERR_NOT_HOMED;          // an absolute target means nothing yet
    }

    /* a move needs a velocity and an acceleration from somewhere: either the FB
     * gave them or the axis carries them */
    if (req_is_position_move(req) || req->Kind == KRON_MOTION_MOVE_VELOCITY ||
        req->Kind == KRON_MOTION_SUPERIMPOSED)
    {
        if (pick_limit(fabs(req->Velocity), axis->MaxVelocity) <= 0.0)
        {
            return MC_ERR_PARAM;
        }
    }

    if (pick_limit(req->Acceleration, axis->MaxAcceleration) <= 0.0 ||
        pick_limit(req->Deceleration, axis->MaxDeceleration) <= 0.0)
    {
        return MC_ERR_PARAM;
    }

    if (req->Acceleration < 0.0 || req->Deceleration < 0.0 || req->Jerk < 0.0)
    {
        return MC_ERR_PARAM;              // 2.4.1 sign rules: these are magnitudes
    }

    return MC_ERR_NONE;
}

uint16_t KronAxis_Submit(AXIS_REF *axis, KRON_MOTION_REQ *req)
{
    uint16_t err;

    if (axis == NULL || req == NULL)
    {
        return MC_ERR_PARAM;
    }

    err = submit_check(axis, req);
    if (err != MC_ERR_NONE)
    {
        req->State   = KRON_REQ_ERROR;
        req->ErrorID = err;
        return err;
    }

    req->ErrorID         = MC_ERR_NONE;
    req->InVelocity      = false;
    req->CoveredDistance = 0.0;
    req->_resolved       = false;

    /* a superimposed motion runs beside the running command instead of taking
     * it over, and only one of them can be active (3.8 note 2) */
    if (req_is_superimposed(req))
    {
        if (req->Kind == KRON_MOTION_HALT_SUPERIMPOSED)
        {
            if (axis->Superimposed != NULL && axis->Superimposed->Kind == KRON_MOTION_SUPERIMPOSED)
            {
                req_finish(axis, axis->Superimposed, KRON_REQ_ABORTED, MC_ERR_NONE);
            }
        }
        else
        {
            req_finish(axis, axis->Superimposed, KRON_REQ_ABORTED, MC_ERR_NONE);
            axis->Super.Position     = 0.0;   // a fresh superimposed distance starts at zero
            axis->SuperTarget        = req->Position;
        }

        if (axis->State == MC_AXIS_STANDSTILL && req->Kind == KRON_MOTION_SUPERIMPOSED)
        {
            axis->State = MC_AXIS_DISCRETE_MOTION;   // 3.8: acts like MC_MoveRelative here
        }

        req->State         = KRON_REQ_ACTIVE;
        axis->Superimposed = req;
        return MC_ERR_NONE;
    }

    /* MC_Stop aborts whatever is running and cannot itself be buffered (Table 4) */
    if (req->Kind == KRON_MOTION_STOP)
    {
        req_finish(axis, axis->Active, KRON_REQ_ABORTED, MC_ERR_NONE);
        req_finish(axis, axis->Superimposed, KRON_REQ_ABORTED, MC_ERR_NONE);
        buffer_clear(axis, KRON_REQ_ABORTED, MC_ERR_NONE);
        activate(axis, req);
        return MC_ERR_NONE;
    }

    if (req->BufferMode == mcAborting || axis->Active == NULL)
    {
        /* the buffer is cleared and the command affects the axis immediately */
        req_finish(axis, axis->Active, KRON_REQ_ABORTED, MC_ERR_NONE);
        buffer_clear(axis, KRON_REQ_ABORTED, MC_ERR_NONE);

        /* an aborting command other than MC_MoveSuperimposed takes the
         * superimposed motion down with it (3.8 note 1) */
        req_finish(axis, axis->Superimposed, KRON_REQ_ABORTED, MC_ERR_NONE);

        activate(axis, req);
        return MC_ERR_NONE;
    }

    if (axis->BufferCount >= (uint8_t)KRON_MOTION_QUEUE_DEPTH)
    {
        req->State   = KRON_REQ_ERROR;
        req->ErrorID = MC_ERR_QUEUE_FULL;
        return MC_ERR_QUEUE_FULL;
    }

    req->State = KRON_REQ_BUFFERED;
    axis->Buffer[axis->BufferCount] = req;
    axis->BufferCount++;

    return MC_ERR_NONE;
}

void KronAxis_Cancel(AXIS_REF *axis, KRON_MOTION_REQ *req)
{
    uint8_t i;
    uint8_t w = 0u;

    if (axis == NULL || req == NULL || req->State != KRON_REQ_BUFFERED)
    {
        return;                            // an active request keeps running (2.4.1)
    }

    for (i = 0u; i < axis->BufferCount; i++)
    {
        if (axis->Buffer[i] != req)
        {
            axis->Buffer[w] = axis->Buffer[i];
            w++;
        }
    }

    for (i = w; i < axis->BufferCount; i++)
    {
        axis->Buffer[i] = NULL;
    }
    axis->BufferCount = w;
    req->State = KRON_REQ_IDLE;
}

/*===========================================================================
 * AXIS_REF_Init
 *===========================================================================*/
void AXIS_REF_Init(AXIS_REF *axis, uint16_t axisNo, KRON_SERVO_SLOT *slot)
{
    if (axis == NULL)
    {
        return;
    }

    memset(axis, 0, sizeof(AXIS_REF));

    axis->AxisNo = axisNo;
    axis->slot   = slot;
    axis->State  = MC_AXIS_DISABLED;

    axis->VelFactor  = 1.0;                // 3.18 note 3: the default is 1.0
    axis->AccFactor  = 1.0;
    axis->JerkFactor = 1.0;

    axis->InPositionWindow = 1e-4;         // the same defaults the core carries
    axis->InVelocityWindow = 1e-3;

    axis->GearRatio   = 1.0;
    axis->EncoderType = slot != NULL ? slot->encoder_type : KRON_ENC_INCREMENTAL;
}

/*===========================================================================
 * Limits for one scan
 *===========================================================================*/
static void build_limits(const AXIS_REF *axis, const KRON_MOTION_REQ *req,
                         MotionLimits *lim, bool with_software_limits)
{
    double vel_factor  = axis->VelFactor  >= 0.0 ? axis->VelFactor  : 0.0;
    double acc_factor  = axis->AccFactor  >  0.0 ? axis->AccFactor  : 1.0;
    double jerk_factor = axis->JerkFactor >  0.0 ? axis->JerkFactor : 1.0;

    MotionLimitsInit(lim);

    if (req != NULL)
    {
        lim->MaxVelocity     = pick_limit(fabs(req->Velocity), axis->MaxVelocity)     * vel_factor;
        lim->MaxAcceleration = pick_limit(req->Acceleration,   axis->MaxAcceleration) * acc_factor;
        lim->MaxDeceleration = pick_limit(req->Deceleration,   axis->MaxDeceleration) * acc_factor;
        lim->Jerk            = pick_limit(req->Jerk,           axis->MaxJerk)         * jerk_factor;
    }
    else
    {
        /* no command: whatever the axis itself allows, used to come to rest */
        lim->MaxVelocity     = axis->MaxVelocity     * vel_factor;
        lim->MaxAcceleration = axis->MaxAcceleration * acc_factor;
        lim->MaxDeceleration = axis->MaxDeceleration * acc_factor;
        lim->Jerk            = axis->MaxJerk         * jerk_factor;
    }

    /* nothing may come out as zero or the core has no room to move at all.
     * A velocity bound of exactly zero is the degenerate case the core cannot
     * act on - it holds the state instead of braking - so a VelFactor of 0.0,
     * which has to stop the axis without leaving its state (3.18 note 5), is
     * expressed as a bound just above zero. */
    if (lim->MaxVelocity     <= 0.0) { lim->MaxVelocity     = 1e-9; }
    if (lim->MaxAcceleration <= 0.0) { lim->MaxAcceleration = 1.0; }
    if (lim->MaxDeceleration <= 0.0) { lim->MaxDeceleration = lim->MaxAcceleration; }

    if (lim->Jerk <= 0.0)
    {
        /* no jerk limit was asked for anywhere: run trapezoidal, which is a jerk
         * far above the accelerations rather than a special case in the core */
        lim->Jerk = KM_TRAPEZOIDAL_JERK_RATIO * dmax(lim->MaxAcceleration, lim->MaxDeceleration);
    }

    /* the core ignores the pair unless it is ordered, which is exactly how a
     * disabled limit switch is expressed - nothing special to branch on */
    if (with_software_limits && axis->EnableLimitNegative && axis->EnableLimitPositive)
    {
        lim->NegativeLimit = axis->SwLimitNegative;
        lim->PositiveLimit = axis->SwLimitPositive;
    }

    lim->InPositionWindow = axis->InPositionWindow > 0.0 ? axis->InPositionWindow : 1e-4;
    lim->InVelocityWindow = axis->InVelocityWindow > 0.0 ? axis->InVelocityWindow : 1e-3;
}

/*===========================================================================
 * Blending
 *
 * The core always arrives at rest, so a blend is expressed as a target that
 * sits one brake distance of the blend velocity beyond the real end position:
 * coming to rest there means passing the real end position at the blend
 * velocity, jerk-limited all the way and with the core's own guarantees intact.
 * The handover happens when the set position crosses the real end position.
 *===========================================================================*/
static double preview_target(const AXIS_REF *axis, const KRON_MOTION_REQ *req, double base)
{
    switch (req->Kind)
    {
        case KRON_MOTION_MOVE_ABSOLUTE:  return req->Position;
        case KRON_MOTION_MOVE_RELATIVE:  return base + req->Position;
        case KRON_MOTION_MOVE_ADDITIVE:  return axis->LastCommandedPos + req->Position;
        default:                         return base;
    }
}

static double blend_velocity(const AXIS_REF *axis,
                             const KRON_MOTION_REQ *current,
                             const KRON_MOTION_REQ *next)
{
    double v_current = pick_limit(fabs(current->Velocity), axis->MaxVelocity);
    double v_next    = pick_limit(fabs(next->Velocity),    axis->MaxVelocity);

    switch (next->BufferMode)
    {
        case mcBlendingLow:      return dmin(v_current, v_next);
        case mcBlendingPrevious: return v_current;
        case mcBlendingNext:     return v_next;
        case mcBlendingHigh:     return dmax(v_current, v_next);
        default:                 return 0.0;                    // not a blend
    }
}

/* the target handed to the core this scan: the real one, or the virtual one a
 * blend needs.  *blending says whether the handover has to be watched for. */
static double effective_target(const AXIS_REF *axis, const KRON_MOTION_REQ *req,
                               const MotionLimits *lim, bool *blending)
{
    const KRON_MOTION_REQ *next;
    double v_blend;
    double next_target;
    double direction;

    *blending = false;

    if (axis->BufferCount == 0u || !req_is_position_move(req))
    {
        return req->_target;
    }

    next = axis->Buffer[0];
    if (next->BufferMode < mcBlendingLow)
    {
        return req->_target;               // aborting or plain buffered
    }

    if (!req_is_position_move(next) && next->Kind != KRON_MOTION_MOVE_VELOCITY)
    {
        return req->_target;               // a halt or a stop is never blended into
    }

    v_blend = blend_velocity(axis, req, next);
    if (v_blend <= 0.0)
    {
        return req->_target;
    }

    direction = req->_target - axis->Set.Position;
    if (fabs(direction) < KM_POS_EPS)
    {
        return req->_target;
    }
    direction = direction > 0.0 ? 1.0 : -1.0;

    /* a blend only makes sense if the next command carries on the same way;
     * a reversal has to come to rest first, so it degrades to 'Buffered' */
    if (next->Kind != KRON_MOTION_MOVE_VELOCITY)
    {
        next_target = preview_target(axis, next, req->_target);
        if ((next_target - req->_target) * direction <= 0.0)
        {
            return req->_target;
        }
    }
    else if (next->Velocity * direction <= 0.0)
    {
        return req->_target;
    }

    *blending = true;

    return req->_target + direction * BrakeDistance(v_blend, 0.0, lim->MaxDeceleration, lim->Jerk);
}

/*===========================================================================
 * Homing
 *
 * Delegated to the drive (CiA402 mode 6).  Without a drive the reference is
 * simply declared: the set position becomes MC_Home.Position.
 *===========================================================================*/
static void run_homing(AXIS_REF *axis)
{
    KRON_SERVO_SLOT *slot = axis->slot;
    uint16_t sw;

    axis->Set.Velocity     = 0.0;
    axis->Set.Acceleration = 0.0;

    if (axis->Simulation || slot == NULL || !slot->present)
    {
        axis->Set.Position = axis->HomePosition;
        axis->IsHomed      = true;
        axis->HomingPhase  = 0u;
        axis->CommandedPosition = axis->HomePosition;
        axis->LastCommandedPos  = axis->HomePosition;
        req_finish(axis, axis->Active, KRON_REQ_DONE, MC_ERR_NONE);
        axis->State = MC_AXIS_STANDSTILL;
        return;
    }

    sw = slot->status_word;

    switch (axis->HomingPhase)
    {
        case 1u:                                        // ask the drive for homing mode
            slot->mode_of_operation = CIA402_MODE_HM;
            slot->control_word      = CIA402_CW_OE;
            if (slot->mode_display == CIA402_MODE_HM)
            {
                axis->HomingPhase = 2u;
            }
            break;

        case 2u:                                        // raise the start bit
            slot->mode_of_operation = CIA402_MODE_HM;
            slot->control_word      = CIA402_CW_OE | CIA402_CW_HM_START;
            axis->HomingPhase = 3u;
            break;

        case 3u:                                        // wait for homing attained
            slot->mode_of_operation = CIA402_MODE_HM;
            slot->control_word      = CIA402_CW_OE | CIA402_CW_HM_START;

            if ((sw & CIA402_SW_HOMING_ATTAINED) != 0u && (sw & CIA402_SW_TARGET_REACHED) != 0u)
            {
                axis->Set.Position      = axis->ActualPosition;
                axis->CommandedPosition = axis->ActualPosition;
                axis->LastCommandedPos  = axis->ActualPosition;
                axis->IsHomed           = true;

                slot->mode_of_operation = CIA402_MODE_CSP;
                slot->control_word      = CIA402_CW_OE;

                axis->HomingPhase = 0u;
                req_finish(axis, axis->Active, KRON_REQ_DONE, MC_ERR_NONE);
                axis->State = MC_AXIS_STANDSTILL;
            }
            break;

        default:
            axis->HomingPhase = 0u;
            break;
    }
}

/*===========================================================================
 * CiA402 drive state machine
 *===========================================================================*/
static bool cia402_step(AXIS_REF *axis)
{
    KRON_SERVO_SLOT *slot = axis->slot;
    uint16_t sw;

    if (axis->Simulation || slot == NULL || !slot->present)
    {
        axis->OperationEnabled = axis->PowerEnabled;     // no drive to argue with
        return axis->OperationEnabled;
    }

    sw = slot->status_word;
    axis->drv_StatusWord   = sw;
    axis->AxisWarning      = (sw & CIA402_SW_WARNING) != 0u;
    axis->OperationEnabled = cia402_op_enabled(sw);

    if (cia402_fault(sw))
    {
        slot->control_word   = CIA402_CW_FACK;           // one reset attempt
        axis->drv_ControlWord = slot->control_word;
        axis->AxisError      = true;
        axis->AxisErrorID    = MC_ERR_AXIS;
        axis->OperationEnabled = false;
        return false;
    }

    if (!axis->PowerEnabled)
    {
        slot->control_word    = CIA402_CW_DISABLE;
        axis->drv_ControlWord = slot->control_word;
        return false;
    }

    /* the mode has to be in place before the drive is enabled; while homing the
     * homing sequence owns it */
    if (axis->HomingPhase == 0u)
    {
        slot->mode_of_operation = CIA402_MODE_CSP;
    }

    if (cia402_not_ready(sw))
    {
        slot->control_word = CIA402_CW_DISABLE;          // still initialising
    }
    else if (cia402_ready_to_so(sw))
    {
        slot->control_word = CIA402_CW_EOA;
    }
    else if (cia402_switched_on(sw) || axis->OperationEnabled)
    {
        slot->control_word = CIA402_CW_OE;
    }
    else
    {
        slot->control_word = CIA402_CW_SO;               // switch-on disabled
    }

    axis->drv_ControlWord = slot->control_word;

    return axis->OperationEnabled;
}

/*===========================================================================
 * Process image
 *===========================================================================*/
void KronAxis_ReadInput(AXIS_REF *axis)
{
    const KRON_SERVO_SLOT *slot;
    double counts_per_unit;
    double vel_raw_per_unit;

    if (axis == NULL)
    {
        return;
    }

    slot = axis->slot;

    if (axis->Simulation || slot == NULL || !slot->present)
    {
        /* the axis follows its own set values perfectly */
        axis->ActualPosition = axis->Set.Position + axis->Super.Position;
        axis->ActualVelocity = axis->Set.Velocity + axis->Super.Velocity;
        axis->ActualTorque   = 0.0;
        return;
    }

    counts_per_unit  = slot->counts_per_unit  > 0.0f ? (double)slot->counts_per_unit  : 1.0;
    vel_raw_per_unit = slot->vel_raw_per_unit > 0.0f ? (double)slot->vel_raw_per_unit : 1.0;

    axis->ActualPosition = (double)slot->actual_pos_raw / counts_per_unit + axis->PositionOffset;
    axis->ActualVelocity = (double)slot->actual_vel_raw    / vel_raw_per_unit;
    axis->ActualTorque   = (double)slot->actual_torque_raw * 0.1;    // per-mille -> %
    axis->drv_StatusWord = slot->status_word;
}

static void write_process_image(const AXIS_REF *axis)
{
    KRON_SERVO_SLOT *slot = axis->slot;
    double counts_per_unit;

    if (slot == NULL || !slot->present)
    {
        return;
    }

    counts_per_unit = slot->counts_per_unit > 0.0f ? (double)slot->counts_per_unit : 1.0;

    slot->target_pos_raw = (int32_t)((axis->Set.Position + axis->Super.Position -
                                      axis->PositionOffset) * counts_per_unit);
    slot->target_vel_raw = (int32_t)((axis->Set.Velocity + axis->Super.Velocity) *
                                     (double)slot->vel_raw_per_unit);
}

/*===========================================================================
 * The superimposed generator
 *
 * A second, independent trajectory whose position is added to the main one.
 * It works in its own frame, starting at zero when the FB starts, so its
 * position is exactly the covered distance the FB has to report.
 *===========================================================================*/
static void run_superimposed(AXIS_REF *axis, double dt)
{
    KRON_MOTION_REQ *req = axis->Superimposed;
    MotionLimits lim;
    MotionCommand cmd;
    TrajectoryStep step;

    if (req == NULL)
    {
        if (fabs(axis->Super.Position) > KM_POS_EPS ||
            fabs(axis->Super.Velocity) > KM_VEL_EPS)
        {
            /* nothing owns the offset any more: leave it where it is, it is
             * already part of the position the axis is standing on */
            axis->Super.Velocity     = 0.0;
            axis->Super.Acceleration = 0.0;
        }
        return;
    }

    build_limits(axis, req, &lim, false);        // the offset has no software limits of its own

    if (req->Kind == KRON_MOTION_HALT_SUPERIMPOSED)
    {
        cmd.Type   = MOTION_COMMAND_VELOCITY;
        cmd.Target = 0.0;
    }
    else
    {
        if (req->ContinuousUpdate)
        {
            axis->SuperTarget = req->Position;
        }
        cmd.Type   = MOTION_COMMAND_POSITION;
        cmd.Target = axis->SuperTarget;
    }

    step = GenerateTrajectory(&axis->Super, &lim, &cmd, dt);
    axis->Super = step.State;

    req->CoveredDistance = axis->Super.Position;

    if (req->Kind == KRON_MOTION_HALT_SUPERIMPOSED)
    {
        if (step.InVelocity)
        {
            req_finish(axis, req, KRON_REQ_DONE, MC_ERR_NONE);
        }
    }
    else if (step.InPosition)
    {
        req_finish(axis, req, KRON_REQ_DONE, MC_ERR_NONE);
    }
}

/*===========================================================================
 * Direction permissions
 *
 * MC_Power carries EnablePositive / EnableNegative, which permit motion in one
 * direction only.  Both false means the extended inputs are not in use and the
 * axis is unrestricted - otherwise an FB instance that was never written would
 * freeze every axis.  A command into a barred direction is not refused: the
 * permission is level sensitive and may come back, so the axis simply brakes
 * and the FB stays Busy.
 *===========================================================================*/
static bool direction_barred(const AXIS_REF *axis, double direction)
{
    if (axis->EnablePositive == axis->EnableNegative)
    {
        return false;                      // not configured, nothing is barred
    }

    if (direction > 0.0)
    {
        return !axis->EnablePositive;
    }
    if (direction < 0.0)
    {
        return !axis->EnableNegative;
    }

    return false;
}

/*===========================================================================
 * The main generator: one scan for the running command
 *===========================================================================*/
static void run_motion(AXIS_REF *axis, double dt)
{
    KRON_MOTION_REQ *req = axis->Active;
    MotionLimits lim;
    MotionCommand cmd;
    TrajectoryStep step;
    bool blending = false;
    bool barred = false;
    double direction;

    if (req == NULL)
    {
        /* no command owns the axis.  If it still carries motion - an abort, a
         * power loss, an error - bring it to rest instead of dropping it. */
        if (fabs(axis->Set.Velocity) <= KM_VEL_EPS && fabs(axis->Set.Acceleration) <= KM_VEL_EPS)
        {
            axis->Set.Velocity     = 0.0;
            axis->Set.Acceleration = 0.0;
            axis->CommandedVelocity = 0.0;
            return;
        }

        build_limits(axis, NULL, &lim, true);
        cmd.Type   = MOTION_COMMAND_VELOCITY;
        cmd.Target = 0.0;

        step = GenerateTrajectory(&axis->Set, &lim, &cmd, dt);
        axis->Set           = step.State;
        axis->SetJerk       = step.Jerk;
        axis->BrakeDistance = step.BrakeDistance;
        axis->CommandedVelocity = 0.0;
        return;
    }

    build_limits(axis, req, &lim, true);

    switch (req->Kind)
    {
        case KRON_MOTION_MOVE_ABSOLUTE:
        case KRON_MOTION_MOVE_RELATIVE:
        case KRON_MOTION_MOVE_ADDITIVE:
            /* with ContinuousUpdate the FB keeps writing _target, so the
             * commanded position has to be re-read rather than latched once */
            axis->CommandedPosition = req->_target;
            axis->LastCommandedPos  = req->_target;
            cmd.Type   = MOTION_COMMAND_POSITION;
            cmd.Target = effective_target(axis, req, &lim, &blending);
            axis->CommandedVelocity = lim.MaxVelocity;
            break;

        case KRON_MOTION_MOVE_VELOCITY:
            cmd.Type   = MOTION_COMMAND_VELOCITY;
            cmd.Target = req->Velocity;                  // already signed by the FB
            if (cmd.Target >  lim.MaxVelocity) { cmd.Target =  lim.MaxVelocity; }
            if (cmd.Target < -lim.MaxVelocity) { cmd.Target = -lim.MaxVelocity; }
            axis->CommandedVelocity = cmd.Target;
            break;

        case KRON_MOTION_HALT:
        case KRON_MOTION_STOP:
            cmd.Type   = MOTION_COMMAND_VELOCITY;
            cmd.Target = 0.0;
            axis->CommandedVelocity = 0.0;
            break;

        default:
            return;
    }

    if (cmd.Type == MOTION_COMMAND_POSITION)
    {
        direction = cmd.Target - axis->Set.Position;
    }
    else
    {
        direction = cmd.Target;
    }

    barred = direction_barred(axis, direction);

    if (barred)
    {
        cmd.Type   = MOTION_COMMAND_VELOCITY;
        cmd.Target = 0.0;
        axis->CommandedVelocity = 0.0;
    }

    step = GenerateTrajectory(&axis->Set, &lim, &cmd, dt);

    direction = step.State.Position - axis->Set.Position;

    axis->Set           = step.State;
    axis->SetJerk       = step.Jerk;
    axis->BrakeDistance = step.BrakeDistance;

    /* ── completion ─────────────────────────────────────────────────────── */
    switch (req->Kind)
    {
        case KRON_MOTION_MOVE_ABSOLUTE:
        case KRON_MOTION_MOVE_RELATIVE:
        case KRON_MOTION_MOVE_ADDITIVE:
            if (barred)
            {
                break;                      // waiting for the permission, not finished
            }
            if (blending)
            {
                /* the handover is the moment the set position crosses the real
                 * end position - the axis is on it, still moving, and the next
                 * command takes over without the velocity ever reaching zero */
                double remaining = req->_target - axis->Set.Position;
                if (fabs(remaining) <= lim.InPositionWindow || remaining * direction < 0.0)
                {
                    req_finish(axis, req, KRON_REQ_DONE, MC_ERR_NONE);
                }
            }
            else if (step.InPosition)
            {
                req_finish(axis, req, KRON_REQ_DONE, MC_ERR_NONE);
                axis->State = MC_AXIS_STANDSTILL;
            }
            break;

        case KRON_MOTION_MOVE_VELOCITY:
            /* the axis standing still because the direction is barred is not
             * the commanded velocity being reached */
            req->InVelocity = step.InVelocity != 0 && !barred;
            /* a never ending motion: it only finishes to let a buffered command
             * through, and InVelocity is what releases it (Table 4) */
            if (req->InVelocity && axis->BufferCount > 0u)
            {
                req_finish(axis, req, KRON_REQ_DONE, MC_ERR_NONE);
            }
            break;

        case KRON_MOTION_HALT:
            if (step.InVelocity)                        // the commanded velocity is zero
            {
                req_finish(axis, req, KRON_REQ_DONE, MC_ERR_NONE);
                axis->State = MC_AXIS_STANDSTILL;
                axis->CommandedPosition = axis->Set.Position;
                axis->LastCommandedPos  = axis->Set.Position;
            }
            break;

        case KRON_MOTION_STOP:
            if (step.InVelocity)
            {
                /* Done is set as soon as the velocity is zero, but the axis is
                 * held in 'Stopping' until Execute goes low (3.3) */
                req->State   = KRON_REQ_DONE;
                req->ErrorID = MC_ERR_NONE;
                axis->CommandedPosition = axis->Set.Position;
                axis->LastCommandedPos  = axis->Set.Position;
            }
            break;

        default:
            break;
    }
}

/*===========================================================================
 * State diagram transitions that do not belong to a command
 *===========================================================================*/
static void enter_errorstop(AXIS_REF *axis)
{
    /* every FB waiting on this axis is told it cannot do its job (2.2.2) */
    req_finish(axis, axis->Active, KRON_REQ_ERROR, MC_ERR_AXIS);
    req_finish(axis, axis->Superimposed, KRON_REQ_ERROR, MC_ERR_AXIS);
    buffer_clear(axis, KRON_REQ_ERROR, MC_ERR_AXIS);

    axis->StopActive  = false;
    axis->HomingPhase = 0u;
    axis->State       = MC_AXIS_ERRORSTOP;
}

static void enter_disabled(AXIS_REF *axis)
{
    req_finish(axis, axis->Active, KRON_REQ_ABORTED, MC_ERR_NONE);
    req_finish(axis, axis->Superimposed, KRON_REQ_ABORTED, MC_ERR_NONE);
    buffer_clear(axis, KRON_REQ_ABORTED, MC_ERR_NONE);

    axis->HomingPhase = 0u;
    axis->State       = MC_AXIS_DISABLED;
}

void KronAxis_ReleaseStop(AXIS_REF *axis, KRON_MOTION_REQ *req)
{
    if (axis == NULL)
    {
        return;
    }

    axis->StopActive = false;

    if (axis->Active == req)
    {
        axis->Active = NULL;
    }

    if (axis->State == MC_AXIS_STOPPING)
    {
        axis->State = MC_AXIS_STANDSTILL;        // 3.3: Done AND NOT Execute
    }
}

bool KronAxis_Reset(AXIS_REF *axis)
{
    if (axis == NULL)
    {
        return false;
    }

    axis->AxisError   = false;
    axis->AxisErrorID = MC_ERR_NONE;

    if (axis->State == MC_AXIS_ERRORSTOP)
    {
        /* Note 3 / Note 4: where the reset lands depends on the power stage */
        axis->State = (axis->PowerEnabled && axis->PowerStatus) ? MC_AXIS_STANDSTILL
                                                                : MC_AXIS_DISABLED;
        axis->Set.Position     = axis->ActualPosition;
        axis->Set.Velocity     = 0.0;
        axis->Set.Acceleration = 0.0;
        axis->CommandedPosition = axis->Set.Position;
        axis->LastCommandedPos  = axis->Set.Position;
    }

    return axis->State == MC_AXIS_STANDSTILL || axis->State == MC_AXIS_DISABLED;
}

void KronAxis_SetPosition(AXIS_REF *axis, double position, bool relative)
{
    double shift;

    if (axis == NULL)
    {
        return;
    }

    /* both sides move by the same amount, so the following error is untouched
     * and no movement is caused (3.17) */
    shift = relative ? position : (position - axis->ActualPosition);

    axis->PositionOffset    += shift;
    axis->ActualPosition    += shift;
    axis->Set.Position      += shift;
    axis->CommandedPosition += shift;
    axis->LastCommandedPos  += shift;

    if (axis->Active != NULL && axis->Active->_resolved)
    {
        axis->Active->_target += shift;          // the target rides along with the frame
    }
}

/*===========================================================================
 * KronAxis_WriteOutput — one cycle
 *===========================================================================*/
void KronAxis_WriteOutput(AXIS_REF *axis, double dt)
{
    bool op_enabled;

    if (axis == NULL || dt <= 0.0)
    {
        return;
    }

    axis->PrevSetVelocity = axis->Set.Velocity;

    /* ── 1. the drive ───────────────────────────────────────────────────── */
    op_enabled = cia402_step(axis);
    axis->PowerStatus = op_enabled;

    /* following error: the axis cannot keep up with what it is being told */
    if (axis->EnablePosLagMonitoring && axis->MaxPositionLag > 0.0 && !axis->Simulation &&
        axis->State != MC_AXIS_DISABLED && axis->State != MC_AXIS_ERRORSTOP)
    {
        if (fabs(axis->Set.Position + axis->Super.Position - axis->ActualPosition) >
            axis->MaxPositionLag)
        {
            axis->AxisError   = true;
            axis->AxisErrorID = MC_ERR_AXIS;
        }
    }

    /* ── 2. an axis error beats everything (Note 1 of the state diagram) ── */
    if (axis->AxisError && axis->State != MC_AXIS_ERRORSTOP)
    {
        enter_errorstop(axis);
    }

    /* power lost while the axis was enabled and moving is an axis error (3.1) */
    if (!op_enabled && axis->PowerEnabled && axis->State != MC_AXIS_ERRORSTOP &&
        axis->State != MC_AXIS_DISABLED)
    {
        axis->AxisError   = true;
        axis->AxisErrorID = MC_ERR_AXIS;
        enter_errorstop(axis);
    }

    /* ── 3. power ───────────────────────────────────────────────────────── */
    if (axis->State != MC_AXIS_ERRORSTOP)
    {
        if (!axis->PowerEnabled)
        {
            if (axis->State != MC_AXIS_DISABLED)
            {
                enter_disabled(axis);        // Note 2: from any state
            }
        }
        else if (axis->State == MC_AXIS_DISABLED && op_enabled)
        {
            axis->State            = MC_AXIS_STANDSTILL;   // Note 5
            axis->Set.Position     = axis->ActualPosition; // take over where the axis stands
            axis->Set.Velocity     = 0.0;
            axis->Set.Acceleration = 0.0;
            axis->CommandedPosition = axis->Set.Position;
            axis->LastCommandedPos  = axis->Set.Position;
        }
    }

    /* ── 4. the motion itself ───────────────────────────────────────────── */
    if (axis->State == MC_AXIS_DISABLED)
    {
        /* the set values follow the feedback so enabling never jumps */
        axis->Set.Position     = axis->ActualPosition;
        axis->Set.Velocity     = 0.0;
        axis->Set.Acceleration = 0.0;
    }
    else if (axis->State == MC_AXIS_HOMING)
    {
        run_homing(axis);
    }
    else
    {
        run_motion(axis, dt);
        run_superimposed(axis, dt);
    }

    /* ── 5. a finished command lets the next one in ─────────────────────── */
    if (axis->Active == NULL && axis->BufferCount > 0u &&
        axis->State != MC_AXIS_ERRORSTOP && axis->State != MC_AXIS_DISABLED && !axis->StopActive)
    {
        activate(axis, buffer_pop(axis));
    }

    /* ── 6. nothing left to do: stand still ─────────────────────────────── */
    if (axis->Active == NULL && axis->Superimposed == NULL && axis->BufferCount == 0u &&
        (axis->State == MC_AXIS_DISCRETE_MOTION || axis->State == MC_AXIS_CONTINUOUS_MOTION))
    {
        if (fabs(axis->Set.Velocity) <= axis->InVelocityWindow)
        {
            axis->State = MC_AXIS_STANDSTILL;
        }
    }

    /* ── 7. out to the fieldbus ─────────────────────────────────────────── */
    write_process_image(axis);
}

/*===========================================================================
 * The whole array
 *===========================================================================*/
void KronMotion_ReadInputs(AXIS_REF *axes, uint16_t count)
{
    uint16_t i;

    HAL_Read_Inputs();

    for (i = 0u; i < count; i++)
    {
        KronAxis_ReadInput(&axes[i]);
    }
}

void KronMotion_WriteOutputs(AXIS_REF *axes, uint16_t count, double dt)
{
    uint16_t i;

    for (i = 0u; i < count; i++)
    {
        KronAxis_WriteOutput(&axes[i], dt);
    }

    HAL_Write_Outputs();
}
