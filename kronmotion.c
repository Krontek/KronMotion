/*===========================================================================
 * kronmotion.c  --  PLCopen Motion Control function blocks
 *
 * Each FB does three things and nothing else: detect the edge on Execute (or
 * the level on Enable), hand a request to the axis engine, and map the result
 * the engine wrote back onto the outputs.  The motion itself belongs to
 * kron_axis.c, and the trajectory to the core underneath it.
 *
 * The output rules of 2.4.1 are implemented once, in fb_collect(), so every
 * block gets the same behaviour: Done / CommandAborted / Error are mutually
 * exclusive, they survive at least one cycle even if Execute was dropped
 * early, and they are cleared on the falling edge of Execute after that.
 *===========================================================================*/

#include "kronmotion.h"

#include <math.h>
#include <string.h>

/*===========================================================================
 * Shared FB plumbing
 *===========================================================================*/
typedef struct {
    bool     Done;
    bool     Busy;
    bool     Active;
    bool     CommandAborted;
    bool     Error;
    uint16_t ErrorID;
} MC_FB_RESULT;

static bool fb_rising(MC_FB_PRIVATE *p, bool execute)
{
    bool rising = execute && !p->prevExecute;
    p->prevExecute = execute;
    return rising;
}

/* The request state, as PLCopen outputs.  Terminal states are latched for one
 * cycle and then follow the falling edge of Execute. */
static MC_FB_RESULT fb_collect(MC_FB_PRIVATE *p, bool execute)
{
    MC_FB_RESULT r;
    bool terminal = false;

    memset(&r, 0, sizeof(r));

    switch (p->req.State)
    {
        case KRON_REQ_BUFFERED:
            r.Busy = true;
            break;

        case KRON_REQ_ACTIVE:
            r.Busy   = true;
            r.Active = true;
            break;

        case KRON_REQ_DONE:
            r.Done   = true;
            terminal = true;
            break;

        case KRON_REQ_ABORTED:
            r.CommandAborted = true;
            terminal = true;
            break;

        case KRON_REQ_ERROR:
            r.Error   = true;
            r.ErrorID = p->req.ErrorID;
            terminal  = true;
            break;

        default:
            break;
    }

    if (!terminal)
    {
        p->terminalShown = false;
        return r;
    }

    if (!execute && p->terminalShown)
    {
        memset(&r, 0, sizeof(r));        // the outputs have been seen, let them go
        p->req.State = KRON_REQ_IDLE;
        p->terminalShown = false;
    }
    else
    {
        p->terminalShown = true;         // guaranteed for at least one cycle
    }

    return r;
}

/* Enable-driven blocks all behave the same way (2.4.1, 'Enable and Valid'). */
static void fb_enable_outputs(bool enable, bool ok, bool *valid, bool *busy,
                              bool *error, uint16_t *errorID, uint16_t err)
{
    if (!enable)
    {
        *valid = false;
        *busy  = false;
        *error = false;
        *errorID = MC_ERR_NONE;
        return;
    }

    *busy  = true;
    *error = !ok;
    *errorID = ok ? MC_ERR_NONE : err;
    *valid = ok;                          // Valid and Error are mutually exclusive
}

/*===========================================================================
 * 3.1  MC_Power
 *===========================================================================*/
void MC_Power_Call(MC_Power *inst, AXIS_REF *axis)
{
    if (inst == NULL)
    {
        return;
    }

    if (axis == NULL)
    {
        inst->Status  = false;
        inst->Valid   = false;
        inst->Error   = true;
        inst->ErrorID = MC_ERR_PARAM;
        return;
    }

    /* level sensitive, every cycle: the axis engine walks the CiA402 sequence
     * and reports back what the power stage really does */
    axis->PowerEnabled   = inst->Enable;
    axis->EnablePositive = inst->EnablePositive;
    axis->EnableNegative = inst->EnableNegative;

    inst->Status  = axis->PowerStatus;
    inst->Error   = axis->AxisError;
    inst->ErrorID = axis->AxisError ? axis->AxisErrorID : MC_ERR_NONE;
    inst->Valid   = !axis->AxisError;
}

/*===========================================================================
 * 3.2  MC_Home
 *===========================================================================*/
void MC_Home_Call(MC_Home *inst, AXIS_REF *axis)
{
    MC_FB_RESULT r;

    if (inst == NULL || axis == NULL)
    {
        return;
    }

    if (fb_rising(&inst->_p, inst->Execute))
    {
        memset(&inst->_p.req, 0, sizeof(inst->_p.req));
        inst->_p.req.Kind       = KRON_MOTION_HOME;
        inst->_p.req.Position   = inst->Position;
        inst->_p.req.BufferMode = inst->BufferMode;

        axis->HomePosition = inst->Position;
        (void)KronAxis_Submit(axis, &inst->_p.req);
        inst->_p.terminalShown = false;
    }
    else if (!inst->Execute && inst->_p.req.State == KRON_REQ_BUFFERED)
    {
        KronAxis_Cancel(axis, &inst->_p.req);
    }

    r = fb_collect(&inst->_p, inst->Execute);

    inst->Done           = r.Done;
    inst->Busy           = r.Busy;
    inst->Active         = r.Active;
    inst->CommandAborted = r.CommandAborted;
    inst->Error          = r.Error;
    inst->ErrorID        = r.ErrorID;
}

/*===========================================================================
 * 3.3  MC_Stop
 *===========================================================================*/
void MC_Stop_Call(MC_Stop *inst, AXIS_REF *axis)
{
    MC_FB_RESULT r;
    bool stop_done;

    if (inst == NULL || axis == NULL)
    {
        return;
    }

    if (fb_rising(&inst->_p, inst->Execute))
    {
        memset(&inst->_p.req, 0, sizeof(inst->_p.req));
        inst->_p.req.Kind         = KRON_MOTION_STOP;
        inst->_p.req.Deceleration = inst->Deceleration;
        inst->_p.req.Jerk         = inst->Jerk;

        /* the axis is locked before the request goes in, so the submit check of
         * every other FB sees the lock from this cycle on (3.3 note 2) */
        axis->StopActive = true;
        if (KronAxis_Submit(axis, &inst->_p.req) != MC_ERR_NONE)
        {
            axis->StopActive = false;
        }
        inst->_p.terminalShown = false;
    }

    stop_done = inst->_p.req.State == KRON_REQ_DONE;

    /* 'As soon as Done is SET and Execute is FALSE the axis goes to Standstill' */
    if (stop_done && !inst->Execute)
    {
        KronAxis_ReleaseStop(axis, &inst->_p.req);
    }

    r = fb_collect(&inst->_p, inst->Execute);

    inst->Done           = r.Done;
    inst->Busy           = r.Busy;
    /* the one block where Active and Done may be set together (2.4.1) */
    inst->Active         = r.Active || (r.Done && axis->StopActive);
    inst->CommandAborted = r.CommandAborted;
    inst->Error          = r.Error;
    inst->ErrorID        = r.ErrorID;
}

/*===========================================================================
 * 3.4  MC_Halt
 *===========================================================================*/
void MC_Halt_Call(MC_Halt *inst, AXIS_REF *axis)
{
    MC_FB_RESULT r;

    if (inst == NULL || axis == NULL)
    {
        return;
    }

    if (fb_rising(&inst->_p, inst->Execute))
    {
        memset(&inst->_p.req, 0, sizeof(inst->_p.req));
        inst->_p.req.Kind         = KRON_MOTION_HALT;
        inst->_p.req.Deceleration = inst->Deceleration;
        inst->_p.req.Jerk         = inst->Jerk;
        inst->_p.req.BufferMode   = inst->BufferMode;

        (void)KronAxis_Submit(axis, &inst->_p.req);
        inst->_p.terminalShown = false;
    }
    else if (!inst->Execute && inst->_p.req.State == KRON_REQ_BUFFERED)
    {
        KronAxis_Cancel(axis, &inst->_p.req);
    }

    r = fb_collect(&inst->_p, inst->Execute);

    inst->Done           = r.Done;
    inst->Busy           = r.Busy;
    inst->Active         = r.Active;
    inst->CommandAborted = r.CommandAborted;
    inst->Error          = r.Error;
    inst->ErrorID        = r.ErrorID;
}

/*===========================================================================
 * 3.5 - 3.7  the three discrete moves
 *
 * They differ only in what 'Position' means, which the axis engine resolves
 * when the request actually takes the axis.
 *===========================================================================*/
static void move_call(MC_FB_PRIVATE *p, AXIS_REF *axis, KRON_MOTION_KIND kind,
                      bool execute, bool continuous_update,
                      double position, double velocity,
                      double acceleration, double deceleration, double jerk,
                      MC_DIRECTION direction, MC_BUFFER_MODE buffer_mode,
                      MC_FB_RESULT *out)
{
    if (fb_rising(p, execute))
    {
        memset(&p->req, 0, sizeof(p->req));
        p->req.Kind             = kind;
        p->req.Position         = position;
        p->req.Velocity         = velocity;
        p->req.Acceleration     = acceleration;
        p->req.Deceleration     = deceleration;
        p->req.Jerk             = jerk;
        p->req.Direction        = direction;
        p->req.BufferMode       = buffer_mode;
        p->req.ContinuousUpdate = continuous_update;

        (void)KronAxis_Submit(axis, &p->req);
        p->terminalShown = false;
    }
    else if (continuous_update &&
             (p->req.State == KRON_REQ_ACTIVE || p->req.State == KRON_REQ_BUFFERED))
    {
        /* 2.4.1: with ContinuousUpdate the parameters may keep changing; the
         * core re-decides from them on the next scan, nothing is re-planned */
        p->req.Position     = position;
        p->req.Velocity     = velocity;
        p->req.Acceleration = acceleration;
        p->req.Deceleration = deceleration;
        p->req.Jerk         = jerk;

        if (kind == KRON_MOTION_MOVE_ABSOLUTE && p->req.State == KRON_REQ_ACTIVE)
        {
            p->req._target = position;
        }
    }
    else if (!execute && p->req.State == KRON_REQ_BUFFERED)
    {
        KronAxis_Cancel(axis, &p->req);
    }

    *out = fb_collect(p, execute);
}

void MC_MoveAbsolute_Call(MC_MoveAbsolute *inst, AXIS_REF *axis)
{
    MC_FB_RESULT r;

    if (inst == NULL || axis == NULL)
    {
        return;
    }

    move_call(&inst->_p, axis, KRON_MOTION_MOVE_ABSOLUTE,
              inst->Execute, inst->ContinuousUpdate,
              inst->Position, inst->Velocity,
              inst->Acceleration, inst->Deceleration, inst->Jerk,
              inst->Direction, inst->BufferMode, &r);

    inst->Done           = r.Done;
    inst->Busy           = r.Busy;
    inst->Active         = r.Active;
    inst->CommandAborted = r.CommandAborted;
    inst->Error          = r.Error;
    inst->ErrorID        = r.ErrorID;
}

void MC_MoveRelative_Call(MC_MoveRelative *inst, AXIS_REF *axis)
{
    MC_FB_RESULT r;

    if (inst == NULL || axis == NULL)
    {
        return;
    }

    move_call(&inst->_p, axis, KRON_MOTION_MOVE_RELATIVE,
              inst->Execute, inst->ContinuousUpdate,
              inst->Distance, inst->Velocity,
              inst->Acceleration, inst->Deceleration, inst->Jerk,
              mcCurrentDirection, inst->BufferMode, &r);

    inst->Done           = r.Done;
    inst->Busy           = r.Busy;
    inst->Active         = r.Active;
    inst->CommandAborted = r.CommandAborted;
    inst->Error          = r.Error;
    inst->ErrorID        = r.ErrorID;
}

void MC_MoveAdditive_Call(MC_MoveAdditive *inst, AXIS_REF *axis)
{
    MC_FB_RESULT r;

    if (inst == NULL || axis == NULL)
    {
        return;
    }

    move_call(&inst->_p, axis, KRON_MOTION_MOVE_ADDITIVE,
              inst->Execute, inst->ContinuousUpdate,
              inst->Distance, inst->Velocity,
              inst->Acceleration, inst->Deceleration, inst->Jerk,
              mcCurrentDirection, inst->BufferMode, &r);

    inst->Done           = r.Done;
    inst->Busy           = r.Busy;
    inst->Active         = r.Active;
    inst->CommandAborted = r.CommandAborted;
    inst->Error          = r.Error;
    inst->ErrorID        = r.ErrorID;
}

/*===========================================================================
 * 3.8  MC_MoveSuperimposed
 *===========================================================================*/
void MC_MoveSuperimposed_Call(MC_MoveSuperimposed *inst, AXIS_REF *axis)
{
    MC_FB_RESULT r;

    if (inst == NULL || axis == NULL)
    {
        return;
    }

    if (fb_rising(&inst->_p, inst->Execute))
    {
        memset(&inst->_p.req, 0, sizeof(inst->_p.req));
        inst->_p.req.Kind             = KRON_MOTION_SUPERIMPOSED;
        inst->_p.req.Position         = inst->Distance;
        inst->_p.req.Velocity         = inst->VelocityDiff;
        inst->_p.req.Acceleration     = inst->Acceleration;
        inst->_p.req.Deceleration     = inst->Deceleration;
        inst->_p.req.Jerk             = inst->Jerk;
        inst->_p.req.ContinuousUpdate = inst->ContinuousUpdate;

        (void)KronAxis_Submit(axis, &inst->_p.req);
        inst->_p.terminalShown = false;
    }
    else if (inst->ContinuousUpdate && inst->_p.req.State == KRON_REQ_ACTIVE)
    {
        inst->_p.req.Position     = inst->Distance;
        inst->_p.req.Velocity     = inst->VelocityDiff;
        inst->_p.req.Acceleration = inst->Acceleration;
        inst->_p.req.Deceleration = inst->Deceleration;
        inst->_p.req.Jerk         = inst->Jerk;
    }

    inst->CoveredDistance = inst->_p.req.CoveredDistance;

    r = fb_collect(&inst->_p, inst->Execute);

    inst->Done           = r.Done;
    inst->Busy           = r.Busy;
    inst->Active         = r.Active;
    inst->CommandAborted = r.CommandAborted;
    inst->Error          = r.Error;
    inst->ErrorID        = r.ErrorID;
}

/*===========================================================================
 * 3.9  MC_HaltSuperimposed
 *===========================================================================*/
void MC_HaltSuperimposed_Call(MC_HaltSuperimposed *inst, AXIS_REF *axis)
{
    MC_FB_RESULT r;

    if (inst == NULL || axis == NULL)
    {
        return;
    }

    if (fb_rising(&inst->_p, inst->Execute))
    {
        memset(&inst->_p.req, 0, sizeof(inst->_p.req));
        inst->_p.req.Kind         = KRON_MOTION_HALT_SUPERIMPOSED;
        inst->_p.req.Deceleration = inst->Deceleration;
        inst->_p.req.Jerk         = inst->Jerk;

        (void)KronAxis_Submit(axis, &inst->_p.req);
        inst->_p.terminalShown = false;
    }

    r = fb_collect(&inst->_p, inst->Execute);

    inst->Done           = r.Done;
    inst->Busy           = r.Busy;
    inst->Active         = r.Active;
    inst->CommandAborted = r.CommandAborted;
    inst->Error          = r.Error;
    inst->ErrorID        = r.ErrorID;
}

/*===========================================================================
 * 3.10  MC_MoveVelocity
 *===========================================================================*/
static double velocity_with_direction(double velocity, MC_DIRECTION direction,
                                      double current_velocity)
{
    switch (direction)
    {
        case mcNegativeDirection:
            return -velocity;                 // 'negative velocity * negative direction = positive'

        case mcCurrentDirection:
            return current_velocity < 0.0 ? -fabs(velocity) : fabs(velocity);

        case mcPositiveDirection:
        default:
            return velocity;                  // the sign the application gave
    }
}

void MC_MoveVelocity_Call(MC_MoveVelocity *inst, AXIS_REF *axis)
{
    MC_FB_RESULT r;
    bool in_velocity;
    double target;

    if (inst == NULL || axis == NULL)
    {
        return;
    }

    target = velocity_with_direction(inst->Velocity, inst->Direction, axis->Set.Velocity);

    if (fb_rising(&inst->_p, inst->Execute))
    {
        memset(&inst->_p.req, 0, sizeof(inst->_p.req));
        inst->_p.req.Kind             = KRON_MOTION_MOVE_VELOCITY;
        inst->_p.req.Velocity         = target;
        inst->_p.req.Acceleration     = inst->Acceleration;
        inst->_p.req.Deceleration     = inst->Deceleration;
        inst->_p.req.Jerk             = inst->Jerk;
        inst->_p.req.Direction        = inst->Direction;
        inst->_p.req.BufferMode       = inst->BufferMode;
        inst->_p.req.ContinuousUpdate = inst->ContinuousUpdate;

        (void)KronAxis_Submit(axis, &inst->_p.req);
        inst->_p.terminalShown = false;
    }
    else if (inst->ContinuousUpdate &&
             (inst->_p.req.State == KRON_REQ_ACTIVE || inst->_p.req.State == KRON_REQ_BUFFERED))
    {
        inst->_p.req.Velocity     = target;
        inst->_p.req.Acceleration = inst->Acceleration;
        inst->_p.req.Deceleration = inst->Deceleration;
        inst->_p.req.Jerk         = inst->Jerk;
    }
    else if (!inst->Execute && inst->_p.req.State == KRON_REQ_BUFFERED)
    {
        KronAxis_Cancel(axis, &inst->_p.req);
    }

    /* 'Inxxx' is updated as long as the FB has control of the axis, and is
     * reset when the block is aborted (2.4.1, 3.10 note 2) */
    in_velocity = inst->_p.req.InVelocity &&
                  (inst->_p.req.State == KRON_REQ_ACTIVE || inst->_p.req.State == KRON_REQ_DONE);

    r = fb_collect(&inst->_p, inst->Execute);

    inst->InVelocity     = in_velocity && !r.CommandAborted && !r.Error;
    inst->Busy           = r.Busy;
    inst->Active         = r.Active;
    inst->CommandAborted = r.CommandAborted;
    inst->Error          = r.Error;
    inst->ErrorID        = r.ErrorID;
}

/*===========================================================================
 * 3.17  MC_SetPosition
 *===========================================================================*/
void MC_SetPosition_Call(MC_SetPosition *inst, AXIS_REF *axis)
{
    bool rising;

    if (inst == NULL)
    {
        return;
    }

    if (axis == NULL)
    {
        inst->Done    = false;
        inst->Busy    = false;
        inst->Error   = true;
        inst->ErrorID = MC_ERR_PARAM;
        return;
    }

    rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (rising)
    {
        /* mcQueued would wait for the running motion; the shift itself causes
         * no movement, so only the state check can fail here */
        if (axis->State == MC_AXIS_ERRORSTOP)
        {
            inst->Done    = false;
            inst->Error   = true;
            inst->ErrorID = MC_ERR_STATE;
            inst->Busy    = false;
            return;
        }

        KronAxis_SetPosition(axis, inst->Position, inst->Relative);

        inst->Done    = true;
        inst->Busy    = false;
        inst->Error   = false;
        inst->ErrorID = MC_ERR_NONE;
        return;
    }

    if (!inst->Execute)
    {
        inst->Done    = false;
        inst->Busy    = false;
        inst->Error   = false;
        inst->ErrorID = MC_ERR_NONE;
    }
}

/*===========================================================================
 * 3.18  MC_SetOverride
 *===========================================================================*/
void MC_SetOverride_Call(MC_SetOverride *inst, AXIS_REF *axis)
{
    bool ok;

    if (inst == NULL || axis == NULL)
    {
        return;
    }

    if (!inst->Enable)
    {
        /* 'If RESET it should keep the last value' - the axis keeps its factors */
        inst->Enabled = false;
        inst->Busy    = false;
        inst->Error   = false;
        inst->ErrorID = MC_ERR_NONE;
        return;
    }

    /* note 4: below zero is not allowed anywhere, and zero is not allowed for
     * the acceleration and the jerk - zero velocity factor is legal and stops
     * the axis without leaving its state (note 5) */
    ok = inst->VelFactor >= 0.0 && inst->AccFactor > 0.0 && inst->JerkFactor > 0.0;

    if (ok)
    {
        axis->VelFactor  = inst->VelFactor;
        axis->AccFactor  = inst->AccFactor;
        axis->JerkFactor = inst->JerkFactor;
    }

    inst->Enabled = ok;
    inst->Busy    = true;
    inst->Error   = !ok;
    inst->ErrorID = ok ? MC_ERR_NONE : MC_ERR_PARAM;
}

/*===========================================================================
 * 3.19 / 3.20  parameter access
 *===========================================================================*/
static bool param_read(const AXIS_REF *axis, int32_t pn, double *value)
{
    switch (pn)
    {
        case MC_PARAM_COMMANDED_POSITION:     *value = axis->CommandedPosition;  return true;
        case MC_PARAM_SW_LIMIT_POS:           *value = axis->SwLimitPositive;    return true;
        case MC_PARAM_SW_LIMIT_NEG:           *value = axis->SwLimitNegative;    return true;
        case MC_PARAM_MAX_POSITION_LAG:       *value = axis->MaxPositionLag;     return true;
        case MC_PARAM_MAX_VELOCITY_SYSTEM:    *value = axis->MaxVelocity;        return true;
        case MC_PARAM_MAX_VELOCITY_APPL:      *value = axis->MaxVelocity;        return true;
        case MC_PARAM_ACTUAL_VELOCITY:        *value = axis->ActualVelocity;     return true;
        case MC_PARAM_COMMANDED_VELOCITY:     *value = axis->CommandedVelocity;  return true;
        case MC_PARAM_MAX_ACCELERATION_SYS:   *value = axis->MaxAcceleration;    return true;
        case MC_PARAM_MAX_ACCELERATION_APPL:  *value = axis->MaxAcceleration;    return true;
        case MC_PARAM_MAX_DECELERATION_SYS:   *value = axis->MaxDeceleration;    return true;
        case MC_PARAM_MAX_DECELERATION_APPL:  *value = axis->MaxDeceleration;    return true;
        case MC_PARAM_MAX_JERK_SYSTEM:        *value = axis->MaxJerk;            return true;
        case MC_PARAM_MAX_JERK_APPL:          *value = axis->MaxJerk;            return true;
        case MC_PARAM_IN_POSITION_WINDOW:     *value = axis->InPositionWindow;   return true;
        case MC_PARAM_IN_VELOCITY_WINDOW:     *value = axis->InVelocityWindow;   return true;
        case MC_PARAM_GEAR_RATIO:             *value = axis->GearRatio;          return true;
        case MC_PARAM_ACTUAL_TORQUE:          *value = axis->ActualTorque;       return true;
        default:                                                                 return false;
    }
}

static bool param_write(AXIS_REF *axis, int32_t pn, double value)
{
    switch (pn)
    {
        case MC_PARAM_SW_LIMIT_POS:           axis->SwLimitPositive  = value; return true;
        case MC_PARAM_SW_LIMIT_NEG:           axis->SwLimitNegative  = value; return true;
        case MC_PARAM_MAX_POSITION_LAG:       axis->MaxPositionLag   = value; return true;
        case MC_PARAM_MAX_VELOCITY_APPL:      axis->MaxVelocity      = value; return true;
        case MC_PARAM_MAX_ACCELERATION_APPL:  axis->MaxAcceleration  = value; return true;
        case MC_PARAM_MAX_DECELERATION_APPL:  axis->MaxDeceleration  = value; return true;
        case MC_PARAM_MAX_JERK_APPL:          axis->MaxJerk          = value; return true;
        case MC_PARAM_IN_POSITION_WINDOW:     axis->InPositionWindow = value; return true;
        case MC_PARAM_IN_VELOCITY_WINDOW:     axis->InVelocityWindow = value; return true;
        case MC_PARAM_GEAR_RATIO:             axis->GearRatio        = value; return true;
        default:                                                              return false;
    }
}

static bool bool_param_read(const AXIS_REF *axis, int32_t pn, bool *value)
{
    switch (pn)
    {
        case MC_PARAM_ENABLE_LIMIT_POS:   *value = axis->EnableLimitPositive;    return true;
        case MC_PARAM_ENABLE_LIMIT_NEG:   *value = axis->EnableLimitNegative;    return true;
        case MC_PARAM_ENABLE_POS_LAG_MON: *value = axis->EnablePosLagMonitoring; return true;
        default:                                                                 return false;
    }
}

static bool bool_param_write(AXIS_REF *axis, int32_t pn, bool value)
{
    switch (pn)
    {
        case MC_PARAM_ENABLE_LIMIT_POS:   axis->EnableLimitPositive    = value; return true;
        case MC_PARAM_ENABLE_LIMIT_NEG:   axis->EnableLimitNegative    = value; return true;
        case MC_PARAM_ENABLE_POS_LAG_MON: axis->EnablePosLagMonitoring = value; return true;
        default:                                                                return false;
    }
}

void MC_ReadParameter_Call(MC_ReadParameter *inst, AXIS_REF *axis)
{
    bool ok;

    if (inst == NULL || axis == NULL)
    {
        return;
    }

    ok = inst->Enable && param_read(axis, inst->ParameterNumber, &inst->Value);

    fb_enable_outputs(inst->Enable, ok, &inst->Valid, &inst->Busy,
                      &inst->Error, &inst->ErrorID, MC_ERR_PARAM);
}

void MC_ReadBoolParameter_Call(MC_ReadBoolParameter *inst, AXIS_REF *axis)
{
    bool ok;

    if (inst == NULL || axis == NULL)
    {
        return;
    }

    ok = inst->Enable && bool_param_read(axis, inst->ParameterNumber, &inst->Value);

    fb_enable_outputs(inst->Enable, ok, &inst->Valid, &inst->Busy,
                      &inst->Error, &inst->ErrorID, MC_ERR_PARAM);
}

void MC_WriteParameter_Call(MC_WriteParameter *inst, AXIS_REF *axis)
{
    bool rising;

    if (inst == NULL || axis == NULL)
    {
        return;
    }

    rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (rising)
    {
        bool ok = param_write(axis, inst->ParameterNumber, inst->Value);
        inst->Done    = ok;
        inst->Error   = !ok;
        inst->ErrorID = ok ? MC_ERR_NONE : MC_ERR_PARAM;
        inst->Busy    = false;
    }
    else if (!inst->Execute)
    {
        inst->Done    = false;
        inst->Busy    = false;
        inst->Error   = false;
        inst->ErrorID = MC_ERR_NONE;
    }
}

void MC_WriteBoolParameter_Call(MC_WriteBoolParameter *inst, AXIS_REF *axis)
{
    bool rising;

    if (inst == NULL || axis == NULL)
    {
        return;
    }

    rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (rising)
    {
        bool ok = bool_param_write(axis, inst->ParameterNumber, inst->Value);
        inst->Done    = ok;
        inst->Error   = !ok;
        inst->ErrorID = ok ? MC_ERR_NONE : MC_ERR_PARAM;
        inst->Busy    = false;
    }
    else if (!inst->Execute)
    {
        inst->Done    = false;
        inst->Busy    = false;
        inst->Error   = false;
        inst->ErrorID = MC_ERR_NONE;
    }
}

/*===========================================================================
 * 3.24 - 3.26  the actual values
 *===========================================================================*/
void MC_ReadActualPosition_Call(MC_ReadActualPosition *inst, AXIS_REF *axis)
{
    if (inst == NULL || axis == NULL)
    {
        return;
    }

    if (inst->Enable)
    {
        inst->Position = axis->ActualPosition;
    }

    fb_enable_outputs(inst->Enable, !axis->AxisError, &inst->Valid, &inst->Busy,
                      &inst->Error, &inst->ErrorID, axis->AxisErrorID);
}

void MC_ReadActualVelocity_Call(MC_ReadActualVelocity *inst, AXIS_REF *axis)
{
    if (inst == NULL || axis == NULL)
    {
        return;
    }

    if (inst->Enable)
    {
        inst->Velocity = axis->ActualVelocity;
    }

    fb_enable_outputs(inst->Enable, !axis->AxisError, &inst->Valid, &inst->Busy,
                      &inst->Error, &inst->ErrorID, axis->AxisErrorID);
}

void MC_ReadActualTorque_Call(MC_ReadActualTorque *inst, AXIS_REF *axis)
{
    if (inst == NULL || axis == NULL)
    {
        return;
    }

    if (inst->Enable)
    {
        inst->Torque = axis->ActualTorque;
    }

    fb_enable_outputs(inst->Enable, !axis->AxisError, &inst->Valid, &inst->Busy,
                      &inst->Error, &inst->ErrorID, axis->AxisErrorID);
}

/*===========================================================================
 * 3.27  MC_ReadStatus
 *===========================================================================*/
void MC_ReadStatus_Call(MC_ReadStatus *inst, AXIS_REF *axis)
{
    if (inst == NULL || axis == NULL)
    {
        return;
    }

    if (!inst->Enable)
    {
        memset(inst, 0, sizeof(*inst));
        return;
    }

    inst->ErrorStop          = axis->State == MC_AXIS_ERRORSTOP;
    inst->Disabled           = axis->State == MC_AXIS_DISABLED;
    inst->Stopping           = axis->State == MC_AXIS_STOPPING;
    inst->Homing             = axis->State == MC_AXIS_HOMING;
    inst->Standstill         = axis->State == MC_AXIS_STANDSTILL;
    inst->DiscreteMotion     = axis->State == MC_AXIS_DISCRETE_MOTION;
    inst->ContinuousMotion   = axis->State == MC_AXIS_CONTINUOUS_MOTION;
    inst->SynchronizedMotion = axis->State == MC_AXIS_SYNCHRONIZED_MOTION;

    inst->Enable = true;
    fb_enable_outputs(true, true, &inst->Valid, &inst->Busy,
                      &inst->Error, &inst->ErrorID, MC_ERR_NONE);
}

/*===========================================================================
 * 3.28  MC_ReadMotionState
 *===========================================================================*/
void MC_ReadMotionState_Call(MC_ReadMotionState *inst, AXIS_REF *axis)
{
    double velocity;
    double acceleration;

    if (inst == NULL || axis == NULL)
    {
        return;
    }

    if (!inst->Enable)
    {
        inst->Valid = false;
        inst->Busy  = false;
        inst->Error = false;
        inst->ErrorID = MC_ERR_NONE;
        inst->ConstantVelocity  = false;
        inst->Accelerating      = false;
        inst->Decelerating      = false;
        inst->DirectionPositive = false;
        inst->DirectionNegative = false;
        return;
    }

    switch (inst->Source)
    {
        case mcActualValue:
            velocity     = axis->ActualVelocity;
            acceleration = 0.0;                       // no filtered actual acceleration is kept
            break;

        case mcCommandedValue:
            velocity     = axis->CommandedVelocity;
            acceleration = 0.0;
            break;

        case mcSetValue:
        default:
            velocity     = axis->Set.Velocity;
            acceleration = axis->Set.Acceleration;
            break;
    }

    /* accelerating and decelerating are about the magnitude of the velocity,
     * not the sign of the acceleration (3.28) */
    inst->Accelerating      = velocity * acceleration >  0.0;
    inst->Decelerating      = velocity * acceleration <  0.0;
    inst->ConstantVelocity  = !inst->Accelerating && !inst->Decelerating;
    inst->DirectionPositive = velocity >  axis->InVelocityWindow;
    inst->DirectionNegative = velocity < -axis->InVelocityWindow;

    fb_enable_outputs(true, !axis->AxisError, &inst->Valid, &inst->Busy,
                      &inst->Error, &inst->ErrorID, axis->AxisErrorID);
}

/*===========================================================================
 * 3.29  MC_ReadAxisInfo
 *===========================================================================*/
void MC_ReadAxisInfo_Call(MC_ReadAxisInfo *inst, AXIS_REF *axis)
{
    if (inst == NULL || axis == NULL)
    {
        return;
    }

    if (!inst->Enable)
    {
        inst->Valid = false;
        inst->Busy  = false;
        inst->Error = false;
        inst->ErrorID = MC_ERR_NONE;
        return;
    }

    inst->HomeAbsSwitch  = inst->HomeSwitchInput     >= 0 &&
                           inst->HomeSwitchInput     <  KRON_MAX_DI &&
                           Kron_PI.di[inst->HomeSwitchInput];
    inst->LimitSwitchPos = inst->LimitSwitchPosInput >= 0 &&
                           inst->LimitSwitchPosInput <  KRON_MAX_DI &&
                           Kron_PI.di[inst->LimitSwitchPosInput];
    inst->LimitSwitchNeg = inst->LimitSwitchNegInput >= 0 &&
                           inst->LimitSwitchNegInput <  KRON_MAX_DI &&
                           Kron_PI.di[inst->LimitSwitchNegInput];

    inst->Simulation         = axis->Simulation;
    inst->CommunicationReady = axis->Simulation ||
                               (axis->slot != NULL && axis->slot->present);
    inst->ReadyForPowerOn    = inst->CommunicationReady && !axis->AxisError;
    inst->PowerOn            = axis->PowerStatus;
    inst->IsHomed            = axis->IsHomed;
    inst->AxisWarning        = axis->AxisWarning;

    fb_enable_outputs(true, true, &inst->Valid, &inst->Busy,
                      &inst->Error, &inst->ErrorID, MC_ERR_NONE);
}

/*===========================================================================
 * 3.30  MC_ReadAxisError
 *===========================================================================*/
void MC_ReadAxisError_Call(MC_ReadAxisError *inst, AXIS_REF *axis)
{
    if (inst == NULL || axis == NULL)
    {
        return;
    }

    if (inst->Enable)
    {
        inst->AxisErrorID = axis->AxisErrorID;
    }

    /* this block reports the axis error, so an axis error does not make it
     * invalid - only the FB's own problems do */
    fb_enable_outputs(inst->Enable, true, &inst->Valid, &inst->Busy,
                      &inst->Error, &inst->ErrorID, MC_ERR_NONE);
}

/*===========================================================================
 * 3.31  MC_Reset
 *===========================================================================*/
void MC_Reset_Call(MC_Reset *inst, AXIS_REF *axis)
{
    bool rising;

    if (inst == NULL || axis == NULL)
    {
        return;
    }

    rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (rising)
    {
        inst->Busy = true;
        inst->_terminalShown = false;
    }

    if (inst->Busy)
    {
        if (KronAxis_Reset(axis))
        {
            inst->Done    = true;
            inst->Busy    = false;
            inst->Error   = false;
            inst->ErrorID = MC_ERR_NONE;
        }
        else
        {
            inst->Error   = true;
            inst->ErrorID = axis->AxisErrorID;
            inst->Busy    = false;
        }
    }

    if (inst->Done || inst->Error)
    {
        if (!inst->Execute && inst->_terminalShown)
        {
            inst->Done    = false;
            inst->Error   = false;
            inst->ErrorID = MC_ERR_NONE;
            inst->_terminalShown = false;
        }
        else
        {
            inst->_terminalShown = true;
        }
    }
}
