/*===========================================================================
 * kron_nc.c  --  KronEditor NC Engine (Fast Task, ~1ms cycle)
 *
 * Implements:
 *   - Trapezoidal velocity profile interpolation (no libm)
 *   - CiA402 (DS-402) drive state machine
 *   - cmd/sts handshake with Slow Task AXIS_REF
 *   - HAL_Read_Inputs / HAL_Write_Outputs orchestration
 *
 * All operations are O(N_axes) per cycle, no dynamic allocation.
 * Baremetal C99.  GCC/Clang only (atomic intrinsics).
 *===========================================================================*/

#include "kron_nc.h"
#include <string.h>  /* memset */

/*===========================================================================
 * Float helpers — no libm
 *===========================================================================*/
#define _NC_FABS(x)        ((x) <  0.0f ? -(x) : (x))
#define _NC_FMIN(a,b)      ((a) < (b) ? (a) : (b))
#define _NC_FMAX(a,b)      ((a) > (b) ? (a) : (b))
#define _NC_CLAMP(x,lo,hi) _NC_FMIN(_NC_FMAX((x),(lo)),(hi))
#define _NC_SIGN(x)        ((x) > 0.0f ? 1.0f : ((x) < 0.0f ? -1.0f : 0.0f))

#define _NC_POS_EPS   1e-4f   /* Position close-enough threshold [u]   */
#define _NC_VEL_EPS   1e-4f   /* Velocity close-enough threshold [u/s] */

/*===========================================================================
 * CiA402 statusword / controlword bitmasks
 *===========================================================================*/
/* Statusword (0x6041) state decode masks */
#define CIA402_SW_RTSO   0x0001u  /* Ready to switch on   */
#define CIA402_SW_SO     0x0002u  /* Switched on          */
#define CIA402_SW_OE     0x0004u  /* Operation enabled    */
#define CIA402_SW_FAULT  0x0008u  /* Fault                */
#define CIA402_SW_VE     0x0010u  /* Voltage enabled      */
#define CIA402_SW_QS     0x0020u  /* Quick stop           */
#define CIA402_SW_SOD    0x0040u  /* Switch on disabled   */
#define CIA402_SW_WARN   0x0080u  /* Warning              */

/* Controlword (0x6040) commands */
#define CIA402_CW_SO     0x0006u  /* Shutdown             */
#define CIA402_CW_EOA    0x0007u  /* Switch on            */
#define CIA402_CW_OE     0x000Fu  /* Enable operation     */
#define CIA402_CW_FACK   0x0080u  /* Fault reset          */
#define CIA402_CW_QS     0x0006u  /* Quick stop           */
#define CIA402_CW_DISABLE 0x0000u /* Disable voltage      */

/* CiA402 Modes of Operation (0x6060) */
#define CIA402_MODE_CSP  8   /* Cyclic Synchronous Position */
#define CIA402_MODE_CSV  9   /* Cyclic Synchronous Velocity */
#define CIA402_MODE_PP   1   /* Profile Position            */
#define CIA402_MODE_PV   3   /* Profile Velocity            */
#define CIA402_MODE_HM   6   /* Homing mode                 */

/* Extract CiA402 state from statusword */
static inline bool _cia402_op_enabled(uint16_t sw)
{
    return (sw & 0x006Fu) == 0x0027u;
}
static inline bool _cia402_fault(uint16_t sw)
{
    return (sw & CIA402_SW_FAULT) != 0;
}
static inline bool _cia402_switched_on(uint16_t sw)
{
    return (sw & 0x006Fu) == 0x0023u;
}
static inline bool _cia402_ready_to_so(uint16_t sw)
{
    return (sw & 0x006Fu) == 0x0021u;
}
static inline bool _cia402_not_ready(uint16_t sw)
{
    return (sw & 0x004Fu) == 0x0000u;
}

/*===========================================================================
 * NC_Init
 *===========================================================================*/
void NC_Init(NC_AXIS *nc, AXIS_REF *ref)
{
    memset(&nc->priv, 0, sizeof(NC_AXIS_INTERNAL));
    nc->ref = ref;
    nc->priv.latched_cmd = NC_CMD_NONE;
}

/*===========================================================================
 * _nc_latch_cmd — check for new command from Slow Task
 *
 * Reads cmd_Seq (ACQUIRE).  If different from latched_seq, latches all
 * cmd_* params and acknowledges by writing sts_AckSeq (RELEASE).
 * Returns true if a new command was latched.
 *===========================================================================*/
static bool _nc_latch_cmd(NC_AXIS *nc)
{
    AXIS_REF *ref = nc->ref;
    uint16_t cur_seq = KRON_LOAD_ACQ_U16(&ref->cmd_Seq);
    if (cur_seq == nc->priv.latched_seq)
        return false;

    nc->priv.latched_seq = cur_seq;
    nc->priv.latched_cmd = ref->cmd_Cmd;
    nc->priv.target_pos  = ref->cmd_TargetPos;
    nc->priv.target_vel  = ref->cmd_TargetVel;
    nc->priv.v_max       = ref->cmd_TargetVel;
    nc->priv.acc         = ref->cmd_Accel;
    nc->priv.dec         = ref->cmd_Decel;
    nc->priv.in_velocity = false;

    /* Acknowledge: NC has latched the command */
    KRON_STORE_REL_U16(&ref->sts_AckSeq, cur_seq);
    return true;
}

/*===========================================================================
 * _nc_cia402_step — drive CiA402 state machine for one axis, one cycle.
 *
 * Writes ref->slot->control_word and mode_of_operation to request the
 * desired drive state.  Reads slot->status_word to detect actual state.
 *
 * Returns true when drive is in Operation Enabled state.
 *===========================================================================*/
static bool _nc_cia402_step(NC_AXIS *nc)
{
    AXIS_REF       *ref  = nc->ref;
    KRON_SERVO_SLOT *slot = ref->slot;

    /* Simulation mode: pretend drive is always enabled */
    if (ref->Simulation || !slot || !slot->present) {
        nc->priv.op_enabled = nc->priv.power_requested;
        return nc->priv.op_enabled;
    }

    uint16_t sw = slot->status_word;
    bool op_en  = _cia402_op_enabled(sw);
    bool fault  = _cia402_fault(sw);

    nc->priv.op_enabled = op_en;

    if (fault) {
        /* Attempt fault reset once */
        slot->control_word = CIA402_CW_FACK;
        ref->sts_Error     = true;
        ref->sts_ErrorID   = 0x8010u;  /* Vendor: drive fault */
        ref->AxisErrorID   = 0x8010u;
        ref->sts_State     = MC_AXIS_ERRORSTOP;
        return false;
    }

    /* Check if we are being asked to power on or off */
    bool want_on = nc->priv.power_requested;

    if (!want_on) {
        slot->control_word = CIA402_CW_DISABLE;
        return false;
    }

    /* Step through CiA402 sequence: Not Ready → Switch-on Disabled
     * → Ready to Switch On → Switched On → Operation Enabled             */
    if (_cia402_not_ready(sw)) {
        /* Drive initializing — nothing to do, wait */
        slot->control_word = CIA402_CW_DISABLE;
    } else if (_cia402_ready_to_so(sw)) {
        slot->control_word = CIA402_CW_EOA;    /* Switch on → Switched on */
    } else if (_cia402_switched_on(sw)) {
        slot->control_word = CIA402_CW_OE;     /* Enable operation */
    } else if (op_en) {
        slot->control_word = CIA402_CW_OE;     /* Keep enabled */
        slot->mode_of_operation = CIA402_MODE_CSP;
    } else {
        /* Switch-on disabled or other intermediate — send Shutdown */
        slot->control_word = CIA402_CW_SO;
    }

    return op_en;
}

/*===========================================================================
 * _nc_profile_pos — trapezoidal position profile, one step.
 *
 * Moves cmd_pos toward target at v_max with acc/dec ramps.
 * Returns true when at target and velocity is zero.
 *===========================================================================*/
static bool _nc_profile_pos(NC_AXIS_INTERNAL *p, float dt)
{
    float remaining = p->target_pos - p->cmd_pos;
    float dir = (remaining >= 0.0f) ? 1.0f : -1.0f;
    float abs_rem = _NC_FABS(remaining);

    /* Project current velocity onto direction of motion */
    float abs_vel = p->cmd_vel * dir;
    if (abs_vel < 0.0f) abs_vel = 0.0f;  /* wrong-direction clamp */

    /* Arrival */
    if (abs_rem < _NC_POS_EPS && abs_vel < _NC_VEL_EPS) {
        p->cmd_vel = 0.0f;
        p->cmd_pos = p->target_pos;
        return true;
    }

    /* Braking distance: d = v² / (2·dec) */
    float brake_dist = (abs_vel * abs_vel) / (2.0f * p->dec + 1e-9f);
    float new_abs_vel;

    if (brake_dist >= abs_rem) {
        /* Decelerate */
        new_abs_vel = abs_vel - p->dec * dt;
        if (new_abs_vel < 0.0f) new_abs_vel = 0.0f;
    } else {
        /* Accelerate or hold */
        new_abs_vel = abs_vel + p->acc * dt;
        if (new_abs_vel > p->v_max) new_abs_vel = p->v_max;
    }

    float new_vel = new_abs_vel * dir;
    float new_pos = p->cmd_pos + new_vel * dt;

    /* Don't overshoot */
    if (dir > 0.0f && new_pos > p->target_pos) { new_pos = p->target_pos; new_vel = 0.0f; }
    if (dir < 0.0f && new_pos < p->target_pos) { new_pos = p->target_pos; new_vel = 0.0f; }

    p->cmd_vel = new_vel;
    p->cmd_pos = new_pos;
    return false;
}

/*===========================================================================
 * _nc_profile_vel — trapezoidal velocity ramp, one step.
 *
 * Ramps cmd_vel toward target_vel.
 * Returns true when velocity has been reached.
 *===========================================================================*/
static bool _nc_profile_vel(NC_AXIS_INTERNAL *p, float dt)
{
    float diff = p->target_vel - p->cmd_vel;
    float new_vel;

    if (_NC_FABS(diff) < _NC_VEL_EPS) {
        new_vel = p->target_vel;
        p->cmd_pos += new_vel * dt;
        p->cmd_vel  = new_vel;
        return true;
    }

    if (diff > 0.0f) {
        new_vel = p->cmd_vel + p->acc * dt;
        if (new_vel > p->target_vel) new_vel = p->target_vel;
    } else {
        new_vel = p->cmd_vel - p->dec * dt;
        if (new_vel < p->target_vel) new_vel = p->target_vel;
    }

    p->cmd_pos += new_vel * dt;
    p->cmd_vel  = new_vel;
    return false;
}

/*===========================================================================
 * _nc_decel_to_zero — decelerate to standstill, one step.
 *
 * Returns true when stopped.
 *===========================================================================*/
static bool _nc_decel_to_zero(NC_AXIS_INTERNAL *p, float dt)
{
    if (_NC_FABS(p->cmd_vel) < _NC_VEL_EPS) {
        p->cmd_vel = 0.0f;
        return true;
    }
    float dir = _NC_SIGN(p->cmd_vel);
    float new_vel = p->cmd_vel - dir * p->dec * dt;
    if (dir > 0.0f && new_vel < 0.0f) new_vel = 0.0f;
    if (dir < 0.0f && new_vel > 0.0f) new_vel = 0.0f;
    p->cmd_pos += new_vel * dt;
    p->cmd_vel  = new_vel;
    return _NC_FABS(new_vel) < _NC_VEL_EPS;
}

/*===========================================================================
 * _nc_write_pi — push NC output (target_pos_raw, control_word) to process image
 *===========================================================================*/
static void _nc_write_pi(NC_AXIS *nc)
{
    KRON_SERVO_SLOT *slot = nc->ref->slot;
    if (!slot || !slot->present) return;

    float cpu = slot->counts_per_unit > 0.0f ? slot->counts_per_unit : 1.0f;
    slot->target_pos_raw = (int32_t)(nc->priv.cmd_pos * cpu);
    slot->target_vel_raw = (int32_t)(nc->priv.cmd_vel * slot->vel_raw_per_unit);
    /* control_word already set by _nc_cia402_step */
}

/*===========================================================================
 * _nc_read_pi — pull actual values from process image into AXIS_REF
 *===========================================================================*/
static void _nc_read_pi(NC_AXIS *nc)
{
    AXIS_REF        *ref  = nc->ref;
    KRON_SERVO_SLOT *slot = ref->slot;

    if (ref->Simulation || !slot || !slot->present) {
        /* Simulation: actuals = commanded */
        ref->ActualPosition = nc->priv.cmd_pos;
        ref->ActualVelocity = nc->priv.cmd_vel;
        ref->ActualTorque   = 0.0f;
        return;
    }

    float cpu = slot->counts_per_unit > 0.0f ? slot->counts_per_unit : 1.0f;
    ref->ActualPosition = (float)slot->actual_pos_raw   / cpu;
    ref->ActualVelocity = (float)slot->actual_vel_raw   / (slot->vel_raw_per_unit > 0.0f ? slot->vel_raw_per_unit : 1.0f);
    ref->ActualTorque   = (float)slot->actual_torque_raw * 0.1f; /* per-mille → % */
}

/*===========================================================================
 * NC_ProcessOne — main per-axis logic for one fast cycle
 *===========================================================================*/
void NC_ProcessOne(NC_AXIS *nc, float dt)
{
    AXIS_REF         *ref  = nc->ref;
    NC_AXIS_INTERNAL *p    = &nc->priv;

    /* ── 1. Run CiA402 state machine ──────────────────────────────────────── */
    bool op_en = _nc_cia402_step(nc);

    /* ── 2. Latch new command if Slow Task published one ──────────────────── */
    bool new_cmd = _nc_latch_cmd(nc);

    if (new_cmd) {
        switch (p->latched_cmd) {
            case NC_CMD_POWER_ON:
                p->power_requested = true;
                ref->sts_State     = MC_AXIS_DISABLED;  /* NC will step to Standstill after op_en */
                ref->sts_Error     = false;
                ref->sts_ErrorID   = 0;
                break;

            case NC_CMD_POWER_OFF:
                p->power_requested = false;
                ref->sts_State     = MC_AXIS_DISABLED;
                ref->sts_Busy      = false;
                ref->sts_Done      = false;
                ref->sts_Error     = false;
                p->cmd_vel         = 0.0f;
                break;

            case NC_CMD_MOVE_ABS:
            case NC_CMD_MOVE_REL:
            case NC_CMD_MOVE_ADD:
                /* target_pos already latched from cmd_TargetPos */
                ref->sts_State  = MC_AXIS_DISCRETE_MOTION;
                ref->sts_Busy   = true;
                ref->sts_Done   = false;
                ref->sts_Error  = false;
                break;

            case NC_CMD_MOVE_VEL:
                ref->sts_State  = MC_AXIS_CONTINUOUS_MOTION;
                ref->sts_Busy   = true;
                ref->sts_Done   = false;
                ref->sts_Error  = false;
                p->in_velocity  = false;
                break;

            case NC_CMD_HALT:
                ref->sts_State  = MC_AXIS_STOPPING;
                ref->sts_Busy   = true;
                ref->sts_Done   = false;
                ref->sts_Error  = false;
                break;

            case NC_CMD_STOP:
                ref->sts_State  = MC_AXIS_STOPPING;
                ref->sts_Busy   = true;
                ref->sts_Done   = false;
                ref->sts_Error  = false;
                p->cmd_vel      = ref->ActualVelocity; /* decel from actual */
                break;

            case NC_CMD_HOME:
                ref->sts_State  = MC_AXIS_HOMING;
                ref->sts_Busy   = true;
                ref->sts_Done   = false;
                ref->sts_Error  = false;
                break;

            case NC_CMD_NONE:
            default:
                break;
        }
    }

    /* ── 3. If drive not enabled, hold position and wait ──────────────────── */
    if (!op_en && !ref->Simulation) {
        /* Axis is disabled or fault — update sts_State if powered off cleanly */
        if (!p->power_requested) {
            ref->sts_State = MC_AXIS_DISABLED;
        }
        _nc_read_pi(nc);
        ref->CommandedPosition = p->cmd_pos;
        ref->CommandedVelocity = p->cmd_vel;
        return;
    }

    /* Drive enabled: transition from DISABLED → STANDSTILL once op_en */
    if (ref->sts_State == MC_AXIS_DISABLED && op_en) {
        p->cmd_pos         = ref->ActualPosition;
        p->cmd_vel         = 0.0f;
        ref->sts_State     = MC_AXIS_STANDSTILL;
        ref->sts_Busy      = false;
        ref->sts_Done      = false;
    }

    /* ── 4. Run motion profile for current state ──────────────────────────── */
    switch (ref->sts_State) {

        case MC_AXIS_STANDSTILL:
        case MC_AXIS_DISABLED:
            /* Hold still */
            p->cmd_vel = 0.0f;
            ref->sts_Busy = false;
            break;

        case MC_AXIS_DISCRETE_MOTION: {
            bool done = _nc_profile_pos(p, dt);
            ref->sts_Busy = !done;
            if (done) {
                ref->sts_Done  = true;
                ref->sts_State = MC_AXIS_STANDSTILL;
                ref->sts_Busy  = false;
            }
            break;
        }

        case MC_AXIS_CONTINUOUS_MOTION: {
            bool at_vel = _nc_profile_vel(p, dt);
            ref->sts_Busy = true;
            if (at_vel && !p->in_velocity) {
                p->in_velocity = true;
                ref->sts_Done  = true;   /* "InVelocity" signal to Slow Task */
            }
            break;
        }

        case MC_AXIS_STOPPING: {
            float dec = (p->dec > _NC_VEL_EPS) ? p->dec : 1000.0f;
            bool stopped;
            {
                NC_AXIS_INTERNAL tmp = *p;
                tmp.dec = dec;
                stopped = _nc_decel_to_zero(&tmp, dt);
                p->cmd_pos = tmp.cmd_pos;
                p->cmd_vel = tmp.cmd_vel;
            }
            ref->sts_Busy = !stopped;
            if (stopped) {
                ref->sts_Done  = true;
                ref->sts_State = MC_AXIS_STANDSTILL;
                ref->sts_Busy  = false;
            }
            break;
        }

        case MC_AXIS_HOMING: {
            /* Simple homing: snap commanded position to cmd_HomePos,
             * mark axis as homed.  Hardware homing sequences can extend this. */
            p->cmd_pos        = ref->cmd_HomePos;
            p->cmd_vel        = 0.0f;
            ref->IsHomed      = true;
            ref->sts_Done     = true;
            ref->sts_Busy     = false;
            ref->sts_State    = MC_AXIS_STANDSTILL;
            break;
        }

        case MC_AXIS_ERRORSTOP:
            p->cmd_vel = 0.0f;
            ref->sts_Busy = false;
            break;

        case MC_AXIS_SYNCHRONIZED_MOTION:
            /* Future: Gearing / Camming — not implemented yet */
            break;

        default:
            break;
    }

    /* ── 5. Apply superimposed offset (if any) ────────────────────────────── */
    if (_NC_FABS(p->superimposed_offset) > _NC_VEL_EPS) {
        float step = p->superimposed_vel * dt;
        if (_NC_FABS(step) > _NC_FABS(p->superimposed_offset))
            step = p->superimposed_offset;
        p->cmd_pos                += step;
        p->superimposed_offset    -= step;
    }

    /* ── 6. Write commanded values back to AXIS_REF ───────────────────────── */
    ref->CommandedPosition = p->cmd_pos;
    ref->CommandedVelocity = p->cmd_vel;

    /* ── 7. Read actual values from process image ─────────────────────────── */
    _nc_read_pi(nc);

    /* ── 8. Push commanded values to process image output ────────────────── */
    _nc_write_pi(nc);

    /* ── 9. Warning flag: following error check ───────────────────────────── */
    if (ref->slot && ref->slot->present) {
        int32_t fe = ref->slot->following_error_raw;
        if (fe < 0) fe = -fe;
        ref->AxisWarning = (fe > 10000);  /* vendor threshold: 10k counts */
    }
}

/*===========================================================================
 * NC_ProcessAxes — main Fast Task entry point
 *===========================================================================*/
void NC_ProcessAxes(NC_AXIS *axes, uint16_t count, float dt)
{
    /* Read all fieldbus inputs first (PDO → process image) */
    HAL_Read_Inputs();

    /* Run each axis */
    for (uint16_t i = 0; i < count; i++) {
        if (axes[i].ref)
            NC_ProcessOne(&axes[i], dt);
    }

    /* Write all fieldbus outputs last (process image → PDO) */
    HAL_Write_Outputs();
}
