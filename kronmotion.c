/*===========================================================================
 * kronmotion.c  --  KronEditor PLCopen Motion Control Function Blocks
 * Specification: PLCopen TC2 Part 1 Version 2.0 (March 17, 2011)
 *
 * Slow Task implementation (~10ms).
 * FBs ONLY communicate through AXIS_REF cmd/sts channels.
 * Interpolation is NOT done here — that is the NC Engine's job (kron_nc.c).
 *
 * cmd/sts handshake protocol:
 *   Slow Task (this file):
 *     1. Write cmd_* param fields.
 *     2. KRON_FETCH_ADD_U16(&axis->cmd_Seq, 1u)  ← publish (RELEASE barrier)
 *   NC Engine (kron_nc.c):
 *     1. Detect cmd_Seq != sts_AckSeq → latch cmd_*.
 *     2. KRON_STORE_REL_U16(&axis->sts_AckSeq, latched_seq)  ← acknowledge.
 *     3. Update sts_Busy / sts_Done / sts_Error / sts_State each fast cycle.
 *===========================================================================*/

#include "kronmotion.h"
#include <string.h>  /* memset */

/* ── Error codes (vendor-defined range 0x8000) ─────────────────────────── */
#define _MC_ERR_STATE     0x8001u   /* Wrong axis state for this command   */
#define _MC_ERR_PARAM     0x8002u   /* Invalid parameter (e.g. Velocity=0) */
#define _MC_ERR_NOT_HOMED 0x8003u   /* Axis not homed for absolute move    */

/* ── Internal convenience macros ─────────────────────────────────────────── */
#define _FB_ERR(inst, code) \
    do { (inst)->Error = true; (inst)->ErrorID = (code); \
         (inst)->Busy  = false; } while (0)

#define _AXIS_SAFE(axis, inst) \
    do { if (!(axis)) { _FB_ERR(inst, _MC_ERR_PARAM); return; } } while (0)

/*===========================================================================
 * AXIS_REF_Init — Reset axis to power-up defaults
 *===========================================================================*/
void AXIS_REF_Init(AXIS_REF *axis, uint16_t axisNo, KRON_SERVO_SLOT *slot)
{
    memset(axis, 0, sizeof(AXIS_REF));
    axis->AxisNo      = axisNo;
    axis->slot        = slot;
    axis->VelFactor   = 1.0f;
    axis->AccFactor   = 1.0f;
    axis->JerkFactor  = 1.0f;
    /* sts_State starts at MC_AXIS_DISABLED (0) — already done by memset */
}

/*===========================================================================
 * 3.1  MC_Power
 *
 * Level-sensitive: Enable HIGH → publish NC_CMD_POWER_ON if not already on.
 *                  Enable LOW  → publish NC_CMD_POWER_OFF.
 * NC Engine transitions the PLCopen state machine; Status mirrors sts_State.
 *===========================================================================*/
void MC_Power_Call(MC_Power *inst, AXIS_REF *axis)
{
    /* MC_Power has no Busy field — check axis pointer manually */
    if (!axis) { inst->Error = true; inst->ErrorID = _MC_ERR_PARAM; return; }

    bool rising  = inst->Enable  && !inst->_prevEnable;
    bool falling = !inst->Enable &&  inst->_prevEnable;
    inst->_prevEnable = inst->Enable;

    /* Reflect axis errors */
    if (axis->sts_Error) {
        inst->Error   = true;
        inst->ErrorID = axis->sts_ErrorID;
        inst->Status  = false;
        inst->Valid   = false;
        return;
    }
    inst->Error = false;

    if (rising) {
        axis->cmd_Cmd = NC_CMD_POWER_ON;
        KRON_FETCH_ADD_U16(&axis->cmd_Seq, 1u);
    } else if (falling) {
        axis->cmd_Cmd = NC_CMD_POWER_OFF;
        KRON_FETCH_ADD_U16(&axis->cmd_Seq, 1u);
    }

    /* Status = TRUE while NC reports axis is in Standstill or above */
    MC_AXIS_STATE st = KRON_LOAD_ACQ_U16((volatile uint16_t *)&axis->sts_State);
    inst->Status = (st >= MC_AXIS_STANDSTILL);
    inst->Valid  = inst->Status;
}

/*===========================================================================
 * 3.2  MC_Home
 *===========================================================================*/
void MC_Home_Call(MC_Home *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    /* Reflect axis errors */
    if (axis->sts_Error) {
        inst->Error   = true;
        inst->ErrorID = axis->sts_ErrorID;
        inst->Busy    = false;
        inst->Active  = false;
        return;
    }

    bool rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (rising) {
        MC_AXIS_STATE st = KRON_LOAD_ACQ_U16((volatile uint16_t *)&axis->sts_State);
        if (st != MC_AXIS_STANDSTILL && st != MC_AXIS_HOMING) {
            _FB_ERR(inst, _MC_ERR_STATE);
            return;
        }
        inst->_myToken = _axis_take_token(axis);
        axis->cmd_Cmd      = NC_CMD_HOME;
        axis->cmd_TargetPos = inst->Position;
        KRON_FETCH_ADD_U16(&axis->cmd_Seq, 1u);
        inst->Busy  = true;
        inst->Active = false;
        inst->Done  = false;
        inst->Error = false;
        inst->CommandAborted = false;
    }

    if (!inst->Busy) return;

    /* Check abort by another FB */
    if (_axis_token_aborted(axis, inst->_myToken)) {
        inst->Busy           = false;
        inst->Active         = false;
        inst->CommandAborted = true;
        inst->Done           = false;
        return;
    }

    inst->Active = axis->sts_Busy;

    if (axis->sts_Done) {
        inst->Done   = true;
        inst->Busy   = false;
        inst->Active = false;
        return;
    }
    if (axis->sts_Error) {
        inst->Error   = true;
        inst->ErrorID = axis->sts_ErrorID;
        inst->Busy    = false;
        inst->Active  = false;
    }
}

/*===========================================================================
 * 3.3  MC_Stop
 *
 * Non-abortable: once active, Execute must stay HIGH until Done.
 *===========================================================================*/
void MC_Stop_Call(MC_Stop *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (rising) {
        if (inst->Deceleration <= 0.0f) {
            _FB_ERR(inst, _MC_ERR_PARAM);
            return;
        }
        axis->cmd_Cmd   = NC_CMD_STOP;
        axis->cmd_Decel = inst->Deceleration;
        axis->cmd_Jerk  = inst->Jerk;
        KRON_FETCH_ADD_U16(&axis->cmd_Seq, 1u);
        inst->Busy  = true;
        inst->Done  = false;
        inst->Error = false;
        return;
    }

    if (!inst->Busy) return;

    if (axis->sts_Error) {
        inst->Error   = true;
        inst->ErrorID = axis->sts_ErrorID;
        inst->Busy    = false;
        return;
    }
    if (axis->sts_Done) {
        inst->Done = true;
        inst->Busy = false;
    }
}

/*===========================================================================
 * 3.4  MC_Halt
 *===========================================================================*/
void MC_Halt_Call(MC_Halt *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (axis->sts_Error) {
        inst->Error   = true;
        inst->ErrorID = axis->sts_ErrorID;
        inst->Busy    = false;
        inst->Active  = false;
        return;
    }

    if (rising) {
        if (inst->Deceleration <= 0.0f) {
            _FB_ERR(inst, _MC_ERR_PARAM);
            return;
        }
        inst->_myToken = _axis_take_token(axis);
        axis->cmd_Cmd   = NC_CMD_HALT;
        axis->cmd_Decel = inst->Deceleration;
        axis->cmd_Jerk  = inst->Jerk;
        KRON_FETCH_ADD_U16(&axis->cmd_Seq, 1u);
        inst->Busy           = true;
        inst->Active         = false;
        inst->Done           = false;
        inst->Error          = false;
        inst->CommandAborted = false;
        return;
    }

    if (!inst->Busy) return;

    if (_axis_token_aborted(axis, inst->_myToken)) {
        inst->Busy           = false;
        inst->Active         = false;
        inst->CommandAborted = true;
        return;
    }

    inst->Active = axis->sts_Busy;

    if (axis->sts_Done) {
        inst->Done   = true;
        inst->Busy   = false;
        inst->Active = false;
    } else if (axis->sts_Error) {
        inst->Error   = true;
        inst->ErrorID = axis->sts_ErrorID;
        inst->Busy    = false;
        inst->Active  = false;
    }
}

/*===========================================================================
 * 3.5  MC_MoveAbsolute
 *===========================================================================*/
void MC_MoveAbsolute_Call(MC_MoveAbsolute *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (axis->sts_Error) {
        inst->Error   = true;
        inst->ErrorID = axis->sts_ErrorID;
        inst->Busy    = false;
        inst->Active  = false;
        return;
    }

    if (rising) {
        MC_AXIS_STATE st = KRON_LOAD_ACQ_U16((volatile uint16_t *)&axis->sts_State);
        if (st != MC_AXIS_STANDSTILL && st != MC_AXIS_DISCRETE_MOTION &&
            st != MC_AXIS_CONTINUOUS_MOTION) {
            _FB_ERR(inst, _MC_ERR_STATE);
            return;
        }
        if (!axis->IsHomed) {
            _FB_ERR(inst, _MC_ERR_NOT_HOMED);
            return;
        }
        if (inst->Velocity <= 0.0f || inst->Acceleration <= 0.0f || inst->Deceleration <= 0.0f) {
            _FB_ERR(inst, _MC_ERR_PARAM);
            return;
        }
        inst->_myToken = _axis_take_token(axis);
        _axis_publish_cmd(axis, NC_CMD_MOVE_ABS,
                          inst->Position, inst->Velocity,
                          inst->Acceleration, inst->Deceleration, inst->Jerk);
        inst->Busy           = true;
        inst->Active         = false;
        inst->Done           = false;
        inst->Error          = false;
        inst->CommandAborted = false;
        return;
    }

    /* ContinuousUpdate: republish while Busy */
    if (inst->Busy && inst->ContinuousUpdate && inst->Execute) {
        _axis_publish_cmd(axis, NC_CMD_MOVE_ABS,
                          inst->Position, inst->Velocity,
                          inst->Acceleration, inst->Deceleration, inst->Jerk);
    }

    if (!inst->Busy) return;

    if (_axis_token_aborted(axis, inst->_myToken)) {
        inst->Busy           = false;
        inst->Active         = false;
        inst->CommandAborted = true;
        return;
    }

    inst->Active = axis->sts_Busy;

    if (axis->sts_Done) {
        inst->Done   = true;
        inst->Busy   = false;
        inst->Active = false;
    } else if (axis->sts_Error) {
        inst->Error   = true;
        inst->ErrorID = axis->sts_ErrorID;
        inst->Busy    = false;
        inst->Active  = false;
    }
}

/*===========================================================================
 * 3.6  MC_MoveRelative
 *===========================================================================*/
void MC_MoveRelative_Call(MC_MoveRelative *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (axis->sts_Error) {
        inst->Error   = true;
        inst->ErrorID = axis->sts_ErrorID;
        inst->Busy    = false;
        inst->Active  = false;
        return;
    }

    if (rising) {
        MC_AXIS_STATE st = KRON_LOAD_ACQ_U16((volatile uint16_t *)&axis->sts_State);
        if (st != MC_AXIS_STANDSTILL && st != MC_AXIS_DISCRETE_MOTION &&
            st != MC_AXIS_CONTINUOUS_MOTION) {
            _FB_ERR(inst, _MC_ERR_STATE);
            return;
        }
        if (inst->Velocity <= 0.0f || inst->Acceleration <= 0.0f || inst->Deceleration <= 0.0f) {
            _FB_ERR(inst, _MC_ERR_PARAM);
            return;
        }
        /* Absolute target is resolved here for ContinuousUpdate */
        inst->_targetPosition = axis->CommandedPosition + inst->Distance;
        inst->_myToken = _axis_take_token(axis);
        _axis_publish_cmd(axis, NC_CMD_MOVE_REL,
                          inst->_targetPosition, inst->Velocity,
                          inst->Acceleration, inst->Deceleration, inst->Jerk);
        inst->Busy           = true;
        inst->Active         = false;
        inst->Done           = false;
        inst->Error          = false;
        inst->CommandAborted = false;
        return;
    }

    if (inst->Busy && inst->ContinuousUpdate && inst->Execute) {
        _axis_publish_cmd(axis, NC_CMD_MOVE_REL,
                          inst->_targetPosition, inst->Velocity,
                          inst->Acceleration, inst->Deceleration, inst->Jerk);
    }

    if (!inst->Busy) return;

    if (_axis_token_aborted(axis, inst->_myToken)) {
        inst->Busy           = false;
        inst->Active         = false;
        inst->CommandAborted = true;
        return;
    }

    inst->Active = axis->sts_Busy;

    if (axis->sts_Done) {
        inst->Done   = true;
        inst->Busy   = false;
        inst->Active = false;
    } else if (axis->sts_Error) {
        inst->Error   = true;
        inst->ErrorID = axis->sts_ErrorID;
        inst->Busy    = false;
        inst->Active  = false;
    }
}

/*===========================================================================
 * 3.7  MC_MoveAdditive
 *===========================================================================*/
void MC_MoveAdditive_Call(MC_MoveAdditive *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (axis->sts_Error) {
        inst->Error   = true;
        inst->ErrorID = axis->sts_ErrorID;
        inst->Busy    = false;
        inst->Active  = false;
        return;
    }

    if (rising) {
        MC_AXIS_STATE st = KRON_LOAD_ACQ_U16((volatile uint16_t *)&axis->sts_State);
        if (st != MC_AXIS_STANDSTILL && st != MC_AXIS_DISCRETE_MOTION) {
            _FB_ERR(inst, _MC_ERR_STATE);
            return;
        }
        if (inst->Velocity <= 0.0f || inst->Acceleration <= 0.0f || inst->Deceleration <= 0.0f) {
            _FB_ERR(inst, _MC_ERR_PARAM);
            return;
        }
        /* Additive: add Distance on top of the NC engine's current commanded pos */
        inst->_targetPosition = axis->CommandedPosition + inst->Distance;
        inst->_myToken = _axis_take_token(axis);
        /* NC_CMD_MOVE_ADD: NC engine adds this on top of in-flight motion */
        _axis_publish_cmd(axis, NC_CMD_MOVE_ADD,
                          inst->_targetPosition, inst->Velocity,
                          inst->Acceleration, inst->Deceleration, inst->Jerk);
        inst->Busy           = true;
        inst->Active         = false;
        inst->Done           = false;
        inst->Error          = false;
        inst->CommandAborted = false;
        return;
    }

    if (inst->Busy && inst->ContinuousUpdate && inst->Execute) {
        _axis_publish_cmd(axis, NC_CMD_MOVE_ADD,
                          inst->_targetPosition, inst->Velocity,
                          inst->Acceleration, inst->Deceleration, inst->Jerk);
    }

    if (!inst->Busy) return;

    if (_axis_token_aborted(axis, inst->_myToken)) {
        inst->Busy           = false;
        inst->Active         = false;
        inst->CommandAborted = true;
        return;
    }

    inst->Active = axis->sts_Busy;

    if (axis->sts_Done) {
        inst->Done   = true;
        inst->Busy   = false;
        inst->Active = false;
    } else if (axis->sts_Error) {
        inst->Error   = true;
        inst->ErrorID = axis->sts_ErrorID;
        inst->Busy    = false;
        inst->Active  = false;
    }
}

/*===========================================================================
 * 3.8  MC_MoveSuperimposed
 *
 * Superimposed moves are added on top of the primary motion.
 * The NC engine accumulates a superimposed offset; CoveredDistance reflects
 * total distance covered by this FB.
 *===========================================================================*/
void MC_MoveSuperimposed_Call(MC_MoveSuperimposed *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (axis->sts_Error) {
        inst->Error   = true;
        inst->ErrorID = axis->sts_ErrorID;
        inst->Busy    = false;
        inst->Active  = false;
        return;
    }

    if (rising) {
        MC_AXIS_STATE st = KRON_LOAD_ACQ_U16((volatile uint16_t *)&axis->sts_State);
        if (st == MC_AXIS_DISABLED || st == MC_AXIS_ERRORSTOP || st == MC_AXIS_STOPPING) {
            _FB_ERR(inst, _MC_ERR_STATE);
            return;
        }
        inst->_coveredSoFar = 0.0f;
        inst->_myToken = _axis_take_token(axis);
        axis->cmd_Cmd       = NC_CMD_MOVE_ADD;
        axis->cmd_TargetPos = inst->Distance;
        axis->cmd_TargetVel = inst->VelocityDiff * axis->VelFactor;
        axis->cmd_Accel     = inst->Acceleration * axis->AccFactor;
        axis->cmd_Decel     = inst->Deceleration * axis->AccFactor;
        KRON_FETCH_ADD_U16(&axis->cmd_Seq, 1u);
        inst->Busy           = true;
        inst->Active         = false;
        inst->Done           = false;
        inst->Error          = false;
        inst->CommandAborted = false;
        return;
    }

    if (!inst->Busy) return;

    if (_axis_token_aborted(axis, inst->_myToken)) {
        inst->Busy           = false;
        inst->Active         = false;
        inst->CommandAborted = true;
        return;
    }

    inst->Active = axis->sts_Busy;

    /* CoveredDistance: NC engine should write this into sts_* future extension;
       for now derive from commanded position delta */
    inst->CoveredDistance = axis->CommandedPosition;

    if (axis->sts_Done) {
        inst->Done   = true;
        inst->Busy   = false;
        inst->Active = false;
    } else if (axis->sts_Error) {
        inst->Error   = true;
        inst->ErrorID = axis->sts_ErrorID;
        inst->Busy    = false;
        inst->Active  = false;
    }
}

/*===========================================================================
 * 3.9  MC_HaltSuperimposed
 *===========================================================================*/
void MC_HaltSuperimposed_Call(MC_HaltSuperimposed *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (axis->sts_Error) {
        inst->Error   = true;
        inst->ErrorID = axis->sts_ErrorID;
        inst->Busy    = false;
        inst->Active  = false;
        return;
    }

    if (rising) {
        inst->_myToken = _axis_take_token(axis);
        axis->cmd_Cmd   = NC_CMD_HALT;
        axis->cmd_Decel = inst->Deceleration;
        axis->cmd_Jerk  = inst->Jerk;
        KRON_FETCH_ADD_U16(&axis->cmd_Seq, 1u);
        inst->Busy           = true;
        inst->Active         = false;
        inst->Done           = false;
        inst->Error          = false;
        inst->CommandAborted = false;
        return;
    }

    if (!inst->Busy) return;

    if (_axis_token_aborted(axis, inst->_myToken)) {
        inst->Busy           = false;
        inst->Active         = false;
        inst->CommandAborted = true;
        return;
    }

    inst->Active = axis->sts_Busy;

    if (axis->sts_Done) {
        inst->Done   = true;
        inst->Busy   = false;
        inst->Active = false;
    } else if (axis->sts_Error) {
        inst->Error   = true;
        inst->ErrorID = axis->sts_ErrorID;
        inst->Busy    = false;
        inst->Active  = false;
    }
}

/*===========================================================================
 * 3.10  MC_MoveVelocity
 *===========================================================================*/
void MC_MoveVelocity_Call(MC_MoveVelocity *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (axis->sts_Error) {
        inst->Error      = true;
        inst->ErrorID    = axis->sts_ErrorID;
        inst->Busy       = false;
        inst->Active     = false;
        inst->InVelocity = false;
        return;
    }

    if (rising) {
        MC_AXIS_STATE st = KRON_LOAD_ACQ_U16((volatile uint16_t *)&axis->sts_State);
        if (st == MC_AXIS_DISABLED || st == MC_AXIS_ERRORSTOP || st == MC_AXIS_STOPPING) {
            _FB_ERR(inst, _MC_ERR_STATE);
            return;
        }
        if (inst->Acceleration <= 0.0f || inst->Deceleration <= 0.0f) {
            _FB_ERR(inst, _MC_ERR_PARAM);
            return;
        }
        float vel = inst->Velocity;
        if (inst->Direction == mcNegativeDirection) vel = -vel;
        inst->_myToken = _axis_take_token(axis);
        _axis_publish_cmd(axis, NC_CMD_MOVE_VEL,
                          0.0f, vel,
                          inst->Acceleration, inst->Deceleration, inst->Jerk);
        inst->Busy           = true;
        inst->Active         = false;
        inst->InVelocity     = false;
        inst->Error          = false;
        inst->CommandAborted = false;
        return;
    }

    if (inst->Busy && inst->ContinuousUpdate && inst->Execute) {
        float vel = inst->Velocity;
        if (inst->Direction == mcNegativeDirection) vel = -vel;
        _axis_publish_cmd(axis, NC_CMD_MOVE_VEL,
                          0.0f, vel,
                          inst->Acceleration, inst->Deceleration, inst->Jerk);
    }

    if (!inst->Busy) return;

    if (_axis_token_aborted(axis, inst->_myToken)) {
        inst->Busy           = false;
        inst->Active         = false;
        inst->InVelocity     = false;
        inst->CommandAborted = true;
        return;
    }

    inst->Active     = axis->sts_Busy;
    inst->InVelocity = axis->sts_Done;   /* NC sets sts_Done when target vel reached */

    if (axis->sts_Error) {
        inst->Error      = true;
        inst->ErrorID    = axis->sts_ErrorID;
        inst->Busy       = false;
        inst->Active     = false;
        inst->InVelocity = false;
    }
}

/*===========================================================================
 * 3.11  MC_MoveContinuousAbsolute
 *===========================================================================*/
void MC_MoveContinuousAbsolute_Call(MC_MoveContinuousAbsolute *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (axis->sts_Error) {
        inst->Error         = true;
        inst->ErrorID       = axis->sts_ErrorID;
        inst->Busy          = false;
        inst->Active        = false;
        inst->InEndVelocity = false;
        return;
    }

    if (rising) {
        MC_AXIS_STATE st = KRON_LOAD_ACQ_U16((volatile uint16_t *)&axis->sts_State);
        if (st != MC_AXIS_STANDSTILL && st != MC_AXIS_DISCRETE_MOTION &&
            st != MC_AXIS_CONTINUOUS_MOTION) {
            _FB_ERR(inst, _MC_ERR_STATE);
            return;
        }
        if (!axis->IsHomed) { _FB_ERR(inst, _MC_ERR_NOT_HOMED); return; }
        if (inst->Velocity <= 0.0f || inst->Acceleration <= 0.0f || inst->Deceleration <= 0.0f) {
            _FB_ERR(inst, _MC_ERR_PARAM);
            return;
        }
        inst->_myToken = _axis_take_token(axis);
        _axis_publish_cmd(axis, NC_CMD_MOVE_ABS,
                          inst->Position, inst->Velocity,
                          inst->Acceleration, inst->Deceleration, inst->Jerk);
        inst->Busy           = true;
        inst->Active         = false;
        inst->InEndVelocity  = false;
        inst->Error          = false;
        inst->CommandAborted = false;
        return;
    }

    if (inst->Busy && inst->ContinuousUpdate && inst->Execute) {
        _axis_publish_cmd(axis, NC_CMD_MOVE_ABS,
                          inst->Position, inst->Velocity,
                          inst->Acceleration, inst->Deceleration, inst->Jerk);
    }

    if (!inst->Busy) return;

    if (_axis_token_aborted(axis, inst->_myToken)) {
        inst->Busy = false; inst->Active = false; inst->CommandAborted = true;
        return;
    }

    inst->Active = axis->sts_Busy;
    /* InEndVelocity: axis reached position and is now at EndVelocity */
    inst->InEndVelocity = axis->sts_Done;

    if (axis->sts_Error) {
        inst->Error = true; inst->ErrorID = axis->sts_ErrorID;
        inst->Busy = false; inst->Active = false; inst->InEndVelocity = false;
    }
}

/*===========================================================================
 * 3.12  MC_MoveContinuousRelative
 *===========================================================================*/
void MC_MoveContinuousRelative_Call(MC_MoveContinuousRelative *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (axis->sts_Error) {
        inst->Error = true; inst->ErrorID = axis->sts_ErrorID;
        inst->Busy = false; inst->Active = false; inst->InEndVelocity = false;
        return;
    }

    if (rising) {
        MC_AXIS_STATE st = KRON_LOAD_ACQ_U16((volatile uint16_t *)&axis->sts_State);
        if (st != MC_AXIS_STANDSTILL && st != MC_AXIS_DISCRETE_MOTION &&
            st != MC_AXIS_CONTINUOUS_MOTION) {
            _FB_ERR(inst, _MC_ERR_STATE);
            return;
        }
        if (inst->Velocity <= 0.0f || inst->Acceleration <= 0.0f || inst->Deceleration <= 0.0f) {
            _FB_ERR(inst, _MC_ERR_PARAM);
            return;
        }
        inst->_targetPosition = axis->CommandedPosition + inst->Distance;
        inst->_myToken = _axis_take_token(axis);
        _axis_publish_cmd(axis, NC_CMD_MOVE_REL,
                          inst->_targetPosition, inst->Velocity,
                          inst->Acceleration, inst->Deceleration, inst->Jerk);
        inst->Busy = true; inst->Active = false; inst->InEndVelocity = false;
        inst->Error = false; inst->CommandAborted = false;
        return;
    }

    if (inst->Busy && inst->ContinuousUpdate && inst->Execute) {
        _axis_publish_cmd(axis, NC_CMD_MOVE_REL,
                          inst->_targetPosition, inst->Velocity,
                          inst->Acceleration, inst->Deceleration, inst->Jerk);
    }

    if (!inst->Busy) return;

    if (_axis_token_aborted(axis, inst->_myToken)) {
        inst->Busy = false; inst->Active = false; inst->CommandAborted = true;
        return;
    }

    inst->Active = axis->sts_Busy;
    inst->InEndVelocity = axis->sts_Done;

    if (axis->sts_Error) {
        inst->Error = true; inst->ErrorID = axis->sts_ErrorID;
        inst->Busy = false; inst->Active = false; inst->InEndVelocity = false;
    }
}

/*===========================================================================
 * 3.17  MC_SetPosition
 *
 * Immediately redefines the axis origin — no motion, no command to NC engine
 * except for a special NC_CMD_HOME with the given position.
 *===========================================================================*/
void MC_SetPosition_Call(MC_SetPosition *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (axis->sts_Error) {
        inst->Error = true; inst->ErrorID = axis->sts_ErrorID;
        inst->Busy = false;
        return;
    }

    if (rising) {
        MC_AXIS_STATE st = KRON_LOAD_ACQ_U16((volatile uint16_t *)&axis->sts_State);
        if (st != MC_AXIS_STANDSTILL) {
            _FB_ERR(inst, _MC_ERR_STATE);
            return;
        }
        float new_pos = inst->Relative
                        ? axis->ActualPosition + inst->Position
                        : inst->Position;
        axis->cmd_Cmd       = NC_CMD_HOME;
        axis->cmd_TargetPos  = new_pos;
        axis->cmd_HomePos    = new_pos;
        KRON_FETCH_ADD_U16(&axis->cmd_Seq, 1u);
        inst->Busy  = true;
        inst->Done  = false;
        inst->Error = false;
        return;
    }

    if (!inst->Busy) return;

    if (axis->sts_Done) {
        inst->Done = true;
        inst->Busy = false;
    } else if (axis->sts_Error) {
        inst->Error = true; inst->ErrorID = axis->sts_ErrorID;
        inst->Busy = false;
    }
}

/*===========================================================================
 * 3.18  MC_SetOverride
 *
 * Level-sensitive. Continuously writes override factors to the axis.
 * The NC engine applies these each fast cycle.
 *===========================================================================*/
void MC_SetOverride_Call(MC_SetOverride *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    if (axis->sts_Error) {
        inst->Error = true; inst->ErrorID = axis->sts_ErrorID;
        inst->Enabled = false;
        return;
    }

    bool rising  = inst->Enable  && !inst->_prevEnable;
    bool falling = !inst->Enable &&  inst->_prevEnable;
    inst->_prevEnable = inst->Enable;

    if (rising) {
        /* Clamp to [0, 1] */
        float vf = inst->VelFactor  < 0.0f ? 0.0f : (inst->VelFactor  > 1.0f ? 1.0f : inst->VelFactor);
        float af = inst->AccFactor  < 0.0f ? 0.0f : (inst->AccFactor  > 1.0f ? 1.0f : inst->AccFactor);
        float jf = inst->JerkFactor < 0.0f ? 0.0f : (inst->JerkFactor > 1.0f ? 1.0f : inst->JerkFactor);
        axis->VelFactor  = (vf > 0.0f) ? vf : 0.01f;
        axis->AccFactor  = (af > 0.0f) ? af : 0.01f;
        axis->JerkFactor = (jf > 0.0f) ? jf : 0.01f;
        inst->Enabled = true;
        inst->Error   = false;
    } else if (falling) {
        axis->VelFactor  = 1.0f;
        axis->AccFactor  = 1.0f;
        axis->JerkFactor = 1.0f;
        inst->Enabled = false;
    }

    (void)falling;  /* suppress unused-variable warning if needed */
}

/*===========================================================================
 * 3.19  MC_ReadParameter / MC_ReadBoolParameter
 *
 * PLCopen parameter numbers mapped to AXIS_REF fields:
 *   0 = CommandedPosition
 *   1 = CommandedVelocity
 *   2 = ActualPosition
 *   3 = ActualVelocity
 *   4 = ActualTorque
 *   5 = VelFactor
 *   6 = AccFactor
 *   7 = JerkFactor
 *===========================================================================*/
void MC_ReadParameter_Call(MC_ReadParameter *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising  = inst->Enable  && !inst->_prevEnable;
    bool falling = !inst->Enable &&  inst->_prevEnable;
    inst->_prevEnable = inst->Enable;

    if (falling) { inst->Valid = false; inst->Busy = false; return; }
    if (!inst->Enable) return;
    if (rising)  { inst->Busy = true; inst->Valid = false; inst->Error = false; }

    if (axis->sts_Error) {
        inst->Error = true; inst->ErrorID = axis->sts_ErrorID;
        inst->Valid = false; inst->Busy = false;
        return;
    }

    float val = 0.0f;
    switch (inst->ParameterNumber) {
        case 0:  val = axis->CommandedPosition; break;
        case 1:  val = axis->CommandedVelocity; break;
        case 2:  val = axis->ActualPosition;    break;
        case 3:  val = axis->ActualVelocity;    break;
        case 4:  val = axis->ActualTorque;      break;
        case 5:  val = axis->VelFactor;         break;
        case 6:  val = axis->AccFactor;         break;
        case 7:  val = axis->JerkFactor;        break;
        default: inst->Error = true; inst->ErrorID = _MC_ERR_PARAM; return;
    }
    inst->Value = val;
    inst->Valid = true;
    inst->Busy  = false;
    inst->Error = false;
}

void MC_ReadBoolParameter_Call(MC_ReadBoolParameter *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising  = inst->Enable  && !inst->_prevEnable;
    bool falling = !inst->Enable &&  inst->_prevEnable;
    inst->_prevEnable = inst->Enable;

    if (falling) { inst->Valid = false; inst->Busy = false; return; }
    if (!inst->Enable) return;
    if (rising)  { inst->Busy = true; inst->Valid = false; inst->Error = false; }

    if (axis->sts_Error) {
        inst->Error = true; inst->ErrorID = axis->sts_ErrorID;
        inst->Valid = false; inst->Busy = false;
        return;
    }

    bool val = false;
    switch (inst->ParameterNumber) {
        case 10: val = axis->IsHomed;      break;
        case 11: val = axis->Simulation;   break;
        case 12: val = axis->AxisWarning;  break;
        default: inst->Error = true; inst->ErrorID = _MC_ERR_PARAM; return;
    }
    inst->Value = val;
    inst->Valid = true;
    inst->Busy  = false;
    inst->Error = false;
}

/*===========================================================================
 * 3.20  MC_WriteParameter / MC_WriteBoolParameter
 *===========================================================================*/
void MC_WriteParameter_Call(MC_WriteParameter *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (!rising) return;
    inst->Busy = true; inst->Done = false; inst->Error = false;

    if (axis->sts_Error) {
        _FB_ERR(inst, axis->sts_ErrorID);
        return;
    }

    switch (inst->ParameterNumber) {
        case 5:  axis->VelFactor  = inst->Value; break;
        case 6:  axis->AccFactor  = inst->Value; break;
        case 7:  axis->JerkFactor = inst->Value; break;
        default: _FB_ERR(inst, _MC_ERR_PARAM);   return;
    }
    inst->Done = true;
    inst->Busy = false;
}

void MC_WriteBoolParameter_Call(MC_WriteBoolParameter *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (!rising) return;
    inst->Busy = true; inst->Done = false; inst->Error = false;

    if (axis->sts_Error) {
        _FB_ERR(inst, axis->sts_ErrorID);
        return;
    }

    switch (inst->ParameterNumber) {
        case 11: axis->Simulation = inst->Value; break;
        default: _FB_ERR(inst, _MC_ERR_PARAM);   return;
    }
    inst->Done = true;
    inst->Busy = false;
}

/*===========================================================================
 * 3.24  MC_ReadActualPosition
 *===========================================================================*/
void MC_ReadActualPosition_Call(MC_ReadActualPosition *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising  = inst->Enable && !inst->_prevEnable;
    bool falling = !inst->Enable && inst->_prevEnable;
    inst->_prevEnable = inst->Enable;

    if (falling) { inst->Valid = false; return; }
    if (!inst->Enable) return;
    if (rising) { inst->Busy = true; inst->Error = false; }

    if (axis->sts_Error) {
        inst->Error = true; inst->ErrorID = axis->sts_ErrorID;
        inst->Valid = false; return;
    }
    inst->Position = axis->ActualPosition;
    inst->Valid    = true;
    inst->Busy     = false;
}

/*===========================================================================
 * 3.25  MC_ReadActualVelocity
 *===========================================================================*/
void MC_ReadActualVelocity_Call(MC_ReadActualVelocity *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising  = inst->Enable && !inst->_prevEnable;
    bool falling = !inst->Enable && inst->_prevEnable;
    inst->_prevEnable = inst->Enable;

    if (falling) { inst->Valid = false; return; }
    if (!inst->Enable) return;
    if (rising) { inst->Busy = true; inst->Error = false; }

    if (axis->sts_Error) {
        inst->Error = true; inst->ErrorID = axis->sts_ErrorID;
        inst->Valid = false; return;
    }
    inst->Velocity = axis->ActualVelocity;
    inst->Valid    = true;
    inst->Busy     = false;
}

/*===========================================================================
 * 3.26  MC_ReadActualTorque
 *===========================================================================*/
void MC_ReadActualTorque_Call(MC_ReadActualTorque *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising  = inst->Enable && !inst->_prevEnable;
    bool falling = !inst->Enable && inst->_prevEnable;
    inst->_prevEnable = inst->Enable;

    if (falling) { inst->Valid = false; return; }
    if (!inst->Enable) return;
    if (rising) { inst->Busy = true; inst->Error = false; }

    if (axis->sts_Error) {
        inst->Error = true; inst->ErrorID = axis->sts_ErrorID;
        inst->Valid = false; return;
    }
    inst->Torque = axis->ActualTorque;
    inst->Valid  = true;
    inst->Busy   = false;
}

/*===========================================================================
 * 3.27  MC_ReadStatus
 *===========================================================================*/
void MC_ReadStatus_Call(MC_ReadStatus *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising  = inst->Enable && !inst->_prevEnable;
    bool falling = !inst->Enable && inst->_prevEnable;
    inst->_prevEnable = inst->Enable;

    if (falling) { inst->Valid = false; return; }
    if (!inst->Enable) return;
    if (rising) { inst->Busy = true; inst->Error = false; }

    MC_AXIS_STATE st = (MC_AXIS_STATE)KRON_LOAD_ACQ_U16((volatile uint16_t *)&axis->sts_State);

    inst->ErrorStop         = (st == MC_AXIS_ERRORSTOP);
    inst->Disabled          = (st == MC_AXIS_DISABLED);
    inst->Stopping          = (st == MC_AXIS_STOPPING);
    inst->Homing            = (st == MC_AXIS_HOMING);
    inst->Standstill        = (st == MC_AXIS_STANDSTILL);
    inst->DiscreteMotion    = (st == MC_AXIS_DISCRETE_MOTION);
    inst->ContinuousMotion  = (st == MC_AXIS_CONTINUOUS_MOTION);
    inst->SynchronizedMotion= (st == MC_AXIS_SYNCHRONIZED_MOTION);

    if (axis->sts_Error) {
        inst->Error = true; inst->ErrorID = axis->sts_ErrorID;
        inst->Valid = false;
    } else {
        inst->Valid = true;
        inst->Error = false;
    }
    inst->Busy = false;
}

/*===========================================================================
 * 3.28  MC_ReadMotionState
 *===========================================================================*/
void MC_ReadMotionState_Call(MC_ReadMotionState *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising  = inst->Enable && !inst->_prevEnable;
    bool falling = !inst->Enable && inst->_prevEnable;
    inst->_prevEnable = inst->Enable;

    if (falling) { inst->Valid = false; return; }
    if (!inst->Enable) return;
    if (rising) { inst->Busy = true; inst->Error = false; }

    float vel = (inst->Source == mcActualValue) ? axis->ActualVelocity
                                                 : axis->CommandedVelocity;
    float prev = inst->_prevVelocity;
    inst->_prevVelocity = vel;

    float eps = 1e-4f;
    inst->ConstantVelocity  = (vel > eps || vel < -eps) && ((vel - prev) < eps) && ((prev - vel) < eps);
    inst->Accelerating      = (vel > prev + eps);
    inst->Decelerating      = (vel < prev - eps);
    inst->DirectionPositive = (vel >  eps);
    inst->DirectionNegative = (vel < -eps);

    if (axis->sts_Error) {
        inst->Error = true; inst->ErrorID = axis->sts_ErrorID;
        inst->Valid = false;
    } else {
        inst->Valid = true;
        inst->Error = false;
    }
    inst->Busy = false;
}

/*===========================================================================
 * 3.29  MC_ReadAxisInfo
 *===========================================================================*/
void MC_ReadAxisInfo_Call(MC_ReadAxisInfo *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising  = inst->Enable && !inst->_prevEnable;
    bool falling = !inst->Enable && inst->_prevEnable;
    inst->_prevEnable = inst->Enable;

    if (falling) { inst->Valid = false; return; }
    if (!inst->Enable) return;
    if (rising) { inst->Busy = true; inst->Error = false; }

    MC_AXIS_STATE st = (MC_AXIS_STATE)KRON_LOAD_ACQ_U16((volatile uint16_t *)&axis->sts_State);

    /* Drive status from process image slot if available */
    KRON_SERVO_SLOT *slot = axis->slot;
    if (slot && slot->present) {
        /* CiA402 statusword bit decoding */
        uint16_t sw = slot->status_word;
        inst->LimitSwitchPos  = (sw & (1u << 11)) != 0;
        inst->LimitSwitchNeg  = (sw & (1u << 12)) != 0;
        inst->HomeAbsSwitch   = (sw & (1u <<  4)) != 0;
        inst->PowerOn         = (sw & 0x0027u) == 0x0027u;  /* Operation enabled */
        inst->ReadyForPowerOn = (sw & 0x0007u) == 0x0001u;  /* Ready to switch on */
        inst->CommunicationReady = true;
    } else {
        /* Simulation or no slot: derive from state */
        inst->PowerOn         = (st >= MC_AXIS_STANDSTILL);
        inst->ReadyForPowerOn = (st == MC_AXIS_DISABLED);
        inst->CommunicationReady = axis->Simulation;
        inst->LimitSwitchPos  = false;
        inst->LimitSwitchNeg  = false;
        inst->HomeAbsSwitch   = false;
    }

    inst->Simulation  = axis->Simulation;
    inst->IsHomed     = axis->IsHomed;
    inst->AxisWarning = axis->AxisWarning;

    if (axis->sts_Error) {
        inst->Error = true; inst->ErrorID = axis->sts_ErrorID;
        inst->Valid = false;
    } else {
        inst->Valid = true;
        inst->Error = false;
    }
    inst->Busy = false;
}

/*===========================================================================
 * 3.30  MC_ReadAxisError
 *===========================================================================*/
void MC_ReadAxisError_Call(MC_ReadAxisError *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising  = inst->Enable && !inst->_prevEnable;
    bool falling = !inst->Enable && inst->_prevEnable;
    inst->_prevEnable = inst->Enable;

    if (falling) { inst->Valid = false; return; }
    if (!inst->Enable) return;
    if (rising) { inst->Busy = true; inst->Error = false; }

    inst->AxisErrorID = axis->AxisErrorID;
    inst->Valid       = true;
    inst->Error       = false;
    inst->Busy        = false;
}

/*===========================================================================
 * 3.31  MC_Reset
 *
 * Rising edge: clears ErrorStop state — NC engine acknowledges by clearing
 * sts_Error and moving to Disabled state.
 *===========================================================================*/
void MC_Reset_Call(MC_Reset *inst, AXIS_REF *axis)
{
    _AXIS_SAFE(axis, inst);

    bool rising = inst->Execute && !inst->_prevExecute;
    inst->_prevExecute = inst->Execute;

    if (rising) {
        /* Only valid from ErrorStop */
        MC_AXIS_STATE st = KRON_LOAD_ACQ_U16((volatile uint16_t *)&axis->sts_State);
        if (st != MC_AXIS_ERRORSTOP) {
            _FB_ERR(inst, _MC_ERR_STATE);
            return;
        }
        /* Clear axis-level error state so NC engine re-enables */
        axis->AxisErrorID = 0;
        axis->AxisWarning = false;
        /* Publish a special POWER_OFF to let NC engine reset CiA402 state */
        axis->cmd_Cmd = NC_CMD_POWER_OFF;
        KRON_FETCH_ADD_U16(&axis->cmd_Seq, 1u);
        inst->Busy  = true;
        inst->Done  = false;
        inst->Error = false;
        return;
    }

    if (!inst->Busy) return;

    /* Wait for NC to clear error */
    if (!axis->sts_Error) {
        inst->Done = true;
        inst->Busy = false;
    }
}

