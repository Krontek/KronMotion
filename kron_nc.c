/*===========================================================================
 * kron_nc.c  --  KronEditor NC Engine (Fast Task, ~1ms cycle)
 *
 * Implements:
 *   - UNIFIED motion generator: single function for all motion types
 *     (position, velocity, stop) with shared S-curve velocity ramp
 *   - CiA402 (DS-402) drive state machine
 *   - cmd/sts handshake with Slow Task AXIS_REF
 *   - HAL_Read_Inputs / HAL_Write_Outputs orchestration
 *
 * Architecture:
 *   All motion types share a single velocity ramp function.
 *   - Position mode: computes a dynamic target velocity based on
 *     remaining distance (decelerates naturally as target approaches)
 *   - Velocity mode: target velocity is the desired velocity (fixed)
 *   - Stop mode: target velocity is zero
 *   Transitions between modes are perfectly smooth because the ramp
 *   function continues from the current kinematic state (pos/vel/acc).
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
#define CIA402_SW_TARGET_REACHED  0x0400u  /* Bit 10              */
#define CIA402_SW_HOMING_ATTAINED 0x1000u  /* Bit 12              */

/* Controlword (0x6040) commands */
#define CIA402_CW_SO     0x0006u  /* Shutdown             */
#define CIA402_CW_EOA    0x0007u  /* Switch on            */
#define CIA402_CW_OE     0x000Fu  /* Enable operation     */
#define CIA402_CW_FACK   0x0080u  /* Fault reset          */
#define CIA402_CW_QS     0x0006u  /* Quick stop           */
#define CIA402_CW_DISABLE 0x0000u /* Disable voltage      */
#define CIA402_CW_HM_START 0x0010u /* Bit 4: start homing  */

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
    nc->priv.gen_mode    = NC_GEN_IDLE;
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
    nc->priv.v_max       = _NC_FABS(ref->cmd_TargetVel);
    nc->priv.acc         = ref->cmd_Accel;
    nc->priv.dec         = ref->cmd_Decel;
    nc->priv.jerk        = ref->cmd_Jerk;

    /* Defensive: enforce minimum values and relationships.
     * MC layer validates these, but tool/test code may bypass MC. */
    if (nc->priv.acc  < _NC_VEL_EPS) nc->priv.acc  = 1000.0f;
    if (nc->priv.dec  < _NC_VEL_EPS) nc->priv.dec  = 1000.0f;
    if (nc->priv.jerk < 0.0f)        nc->priv.jerk = 0.0f;
    if (nc->priv.jerk > 0.0f) {
        float min_jerk = _NC_FMAX(nc->priv.acc, nc->priv.dec);
        if (nc->priv.jerk < min_jerk)
            nc->priv.jerk = min_jerk;
    }

    /* Reset S-curve ramp state for the new command */
    nc->priv.in_velocity      = false;
    nc->priv.cruise_ramp_down = false;
    nc->priv.decel_committed  = false;

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

    /* Mirror raw PDO words into AXIS_REF for monitoring */
    ref->drv_StatusWord  = sw;

    if (fault) {
        /* Attempt fault reset once */
        slot->control_word = CIA402_CW_FACK;
        ref->sts_Error     = true;
        ref->sts_ErrorID   = 100u;  /* Vendor: drive fault */
        ref->AxisErrorID   = 100u;
        ref->sts_State     = MC_AXIS_ERRORSTOP;
        return false;
    }

    /* Check if we are being asked to power on or off */
    bool want_on = nc->priv.power_requested;

    if (!want_on) {
        slot->control_word = CIA402_CW_DISABLE;
        return false;
    }

    /* Set mode of operation EARLY — many drives require this before enabling.
     * CSP (Cyclic Synchronous Position) is the default for NC-style control.
     * During drive-delegated homing the homing state machine owns the mode. */
    if (nc->priv.homing_phase == 0)
        slot->mode_of_operation = CIA402_MODE_CSP;

    /* Step through CiA402 sequence: Not Ready → Switch-on Disabled
     * → Ready to Switch On → Switched On → Operation Enabled             */
    if (_cia402_not_ready(sw)) {
        slot->control_word = CIA402_CW_DISABLE;
    } else if (_cia402_ready_to_so(sw)) {
        slot->control_word = CIA402_CW_EOA;    /* Switch on → Switched on */
    } else if (_cia402_switched_on(sw)) {
        slot->control_word = CIA402_CW_OE;     /* Enable operation */
    } else if (op_en) {
        slot->control_word = CIA402_CW_OE;     /* Keep enabled */
    } else {
        /* Switch-on disabled or other intermediate — send Shutdown */
        slot->control_word = CIA402_CW_SO;
    }

    ref->drv_ControlWord = slot->control_word;
    return op_en;
}

/*===========================================================================
 * SHARED MATH: Stopping distance, square root, inverse stopping distance
 *===========================================================================*/

/* Forward declaration */
static float _nc_fsqrt(float x);

/*---------------------------------------------------------------------------
 * _nc_stopping_distance — compute distance needed to stop from current state
 *
 * Given (abs_vel, abs_acc, dec, jerk), returns how far we travel if we
 * start braking NOW and come to a complete stop.
 *
 * Two cases:
 *   jerk > 0: S-curve stop (ramp acc to 0, then jerk-limited decel)
 *   jerk = 0: Trapezoidal stop = v² / (2·dec)
 *---------------------------------------------------------------------------*/
static float _nc_stopping_distance(float abs_vel, float acc_in_dir,
                                   float dec, float jerk)
{
    if (abs_vel < _NC_VEL_EPS && _NC_FABS(acc_in_dir) < _NC_VEL_EPS)
        return 0.0f;

    if (jerk <= 0.0f) {
        /* Trapezoidal: d = v² / (2·dec) */
        return (abs_vel * abs_vel) / (2.0f * dec + 1e-9f);
    }

    float d = 0.0f;
    float v = abs_vel;

    /* Phase A: if currently accelerating (acc > 0), must ramp acc down to 0
     * before we can start decelerating.  Velocity increases during this. */
    if (acc_in_dir > 0.0f) {
        float a  = acc_in_dir;
        float t_a = a / jerk;
        d += v * t_a + (a * a * a) / (3.0f * jerk * jerk);
        v += (a * a) / (2.0f * jerk);
    }

    /* Phase B: from (v, a=0) do a full S-curve decel to zero. */
    float dv_ramp = (dec * dec) / (2.0f * jerk);
    float dv_both = 2.0f * dv_ramp;

    if (v <= dv_both + _NC_VEL_EPS) {
        /* Triangular decel: never reach full dec.
         * Two symmetric jerk phases: d = v * t_half
         * where a_peak = sqrt(v * jerk), t_half = a_peak / jerk. */
        float a_peak = _nc_fsqrt(v * jerk);
        if (a_peak < 1e-6f) return d;
        float t_half = a_peak / jerk;
        d += v * t_half;
    } else {
        /* Full 3-sub-phase decel */
        float t_ramp = dec / jerk;
        d += v * t_ramp - dv_ramp * t_ramp / 3.0f;
        v -= dv_ramp;
        float d_b3 = dv_ramp * t_ramp / 3.0f;
        float v_b3_entry = dv_ramp;
        float v_b2 = v - v_b3_entry;
        if (v_b2 > 0.0f) {
            float t_b2 = v_b2 / dec;
            d += (v - v_b2 * 0.5f) * t_b2;
        }
        d += d_b3;
    }

    /* Correction for existing deceleration */
    if (acc_in_dir < 0.0f) {
        float ed = _NC_FMIN(-acc_in_dir, dec);
        float savings = abs_vel * ed / (2.0f * jerk)
                      - (ed * ed * ed) / (6.0f * jerk * jerk);
        if (savings > 0.0f) d -= savings;
        if (d < 0.0f) d = 0.0f;
    }

    return d;
}

/*---------------------------------------------------------------------------
 * _nc_fsqrt — fast square root without libm
 *---------------------------------------------------------------------------*/
static float _nc_fsqrt(float x)
{
    if (x <= 0.0f) return 0.0f;
    union { float f; uint32_t i; } u = { .f = x };
    u.i = 0x5f3759df - (u.i >> 1);
    float inv = u.f;
    inv = inv * (1.5f - 0.5f * x * inv * inv);
    inv = inv * (1.5f - 0.5f * x * inv * inv);  /* 2nd iteration: ~22-bit accuracy */
    return x * inv;
}

/*---------------------------------------------------------------------------
 * _nc_vel_for_dist — inverse stopping distance (Newton's method)
 *
 * Given a distance, returns the maximum velocity from which we can stop
 * within that distance (starting with zero acceleration).
 * Used by position mode to compute the dynamic target velocity.
 *---------------------------------------------------------------------------*/
static float _nc_vel_for_dist(float dist, float dec, float jerk)
{
    if (dist < _NC_POS_EPS) return 0.0f;

    /* Initial estimate: trapezoidal (upper bound) */
    float v = _nc_fsqrt(2.0f * dec * dist);

    if (jerk <= 0.0f) return v;

    /* Newton iterations: find v such that stopping_distance(v,0,dec,jerk) = dist */
    for (int i = 0; i < 5; i++) {
        float d = _nc_stopping_distance(v, 0.0f, dec, jerk);
        float err = d - dist;
        if (_NC_FABS(err) < _NC_POS_EPS * 0.1f) break;
        /* Derivative: dd/dv ≈ v/dec + dec/(2·jerk) for full profile */
        float dddv = v / (dec + 1e-9f) + dec / (2.0f * jerk + 1e-9f);
        if (dddv < 1e-6f) break;
        v -= err / dddv;
        if (v < 0.0f) v = 0.0f;
    }

    return v;
}

/*===========================================================================
 * UNIFIED MOTION GENERATOR
 *
 * Single entry point for all motion types.  Components:
 *
 *   1. _nc_select_limit  — choose acc or dec based on speed magnitude change
 *   2. _nc_vel_ramp      — S-curve/trapezoidal velocity ramp (shared core)
 *   3. _nc_brake_scurve  — direct S-curve braking for position mode
 *   4. _nc_motion_step   — dispatches on gen_mode, computes target velocity
 *
 * Velocity/stop modes use the shared vel_ramp.  Position mode braking
 * uses _nc_brake_scurve which computes jerk directly from stopping
 * distance comparison, avoiding the chattering that occurs when vel_ramp
 * tracks a moving velocity target.
 *===========================================================================*/

/*---------------------------------------------------------------------------
 * _nc_brake_scurve — Direct S-curve braking for position mode
 *
 * Bypasses vel_ramp to avoid jerk chattering that occurs when vel_ramp
 * tracks a moving v_limit target.  Three monotonic phases:
 *
 *   1. If still accelerating toward target → ramp acc to zero
 *   2. Build deceleration toward limit (ramp-in) or hold at limit
 *   3. When velocity low enough for triangular ramp-out → release
 *      decel (positive jerk only, NEVER re-tighten)
 *
 * Key design rules:
 *   - Jerk direction NEVER reverses during braking (no chattering)
 *   - No stopping-distance feedback loop during execution
 *   - Phase transitions are one-way: ramp-in → hold → ramp-out
 *
 * Updates cmd_acc, cmd_vel, cmd_pos.
 *---------------------------------------------------------------------------*/
static void _nc_brake_scurve(NC_AXIS_INTERNAL *p, float abs_rem,
                              float abs_vel, float dir, float dt)
{
    float j       = p->jerk;
    float dec     = p->dec;
    float old_vel = p->cmd_vel;
    float old_acc = p->cmd_acc;
    float acc_toward = old_acc * dir;  /* + = toward target, - = braking */
    float cur_decel  = _NC_FMAX(-acc_toward, 0.0f);

    float new_acc;

    /* ── 1. Still accelerating toward target: ramp acc to zero first ── */
    if (acc_toward > _NC_VEL_EPS) {
        new_acc = old_acc - dir * j * dt;
        /* Don't overshoot zero */
        if ((new_acc * dir) < 0.0f) new_acc = 0.0f;
        goto integrate;
    }

    /* ── 2. Ramp-out: velocity low enough to stop by releasing decel ── */
    /*    Entry: abs_vel ≤ cur_decel² / (2·j)                          */
    /*    Once here, ONLY release decel — never re-tighten.             */
    /*    The margin stays constant (vel and threshold decrease at the   */
    /*    same rate), so no oscillation at the boundary.                 */
    {
        float ramp_out_vel = cur_decel * cur_decel / (2.0f * j + 1e-12f);

        if (abs_vel <= ramp_out_vel + _NC_VEL_EPS && cur_decel > _NC_VEL_EPS) {
            new_acc = old_acc + dir * j * dt;
            /* Don't cross into acceleration */
            if (new_acc * dir > 0.0f) new_acc = 0.0f;
            goto integrate;
        }
    }

    /* ── 3. Build or hold deceleration — NEVER ease ─────────────────── */
    /*    No stopping-distance feedback here to avoid chattering.        */
    /*    The commitment decision was made by the caller; here we just   */
    /*    execute the 3-phase profile deterministically.                  */
    if (cur_decel < dec - _NC_VEL_EPS) {
        /* Ramp-in: build deceleration toward limit */
        new_acc = old_acc - dir * j * dt;
        if (-new_acc * dir > dec) new_acc = -dir * dec;
    } else {
        /* At or above limit: hold at dec */
        new_acc = -dir * dec;
    }

integrate:;
    {
        float new_vel = old_vel + new_acc * dt;

        /* Prevent velocity reversal through zero */
        if ((old_vel > _NC_VEL_EPS && new_vel < 0.0f) ||
            (old_vel < -_NC_VEL_EPS && new_vel > 0.0f)) {
            new_vel = 0.0f;
            new_acc = -old_vel / (dt + 1e-12f);
        }

        p->cmd_pos += 0.5f * (old_vel + new_vel) * dt;
        p->cmd_vel  = new_vel;
        p->cmd_acc  = new_acc;
    }
}

/*---------------------------------------------------------------------------
 * _nc_select_limit — PLCopen-compliant acc/dec selection
 *
 * Rule: acceleration when speed magnitude is increasing,
 *       deceleration when speed magnitude is decreasing.
 *
 * At standstill, any motion counts as acceleration.
 * When velocity and goal are in opposite directions (reversal),
 * the first phase is deceleration (slowing to zero).
 *---------------------------------------------------------------------------*/
static float _nc_select_limit(const NC_AXIS_INTERNAL *p, float goal_vel)
{
    float abs_cur = _NC_FABS(p->cmd_vel);
    float abs_tgt = _NC_FABS(goal_vel);

    /* At or near standstill: starting motion = acceleration */
    if (abs_cur < _NC_VEL_EPS)
        return p->acc;

    /* Same direction: compare magnitudes */
    if ((p->cmd_vel >= 0.0f) == (goal_vel >= 0.0f) || abs_tgt < _NC_VEL_EPS) {
        return (abs_tgt > abs_cur + _NC_VEL_EPS) ? p->acc : p->dec;
    }

    /* Opposite directions: must slow down first */
    return p->dec;
}

/*---------------------------------------------------------------------------
 * _nc_vel_ramp — Unified S-curve / trapezoidal velocity ramp
 *
 * Ramps cmd_vel toward goal_vel using the correct acc/dec limit.
 * Handles:
 *   - Trapezoidal (jerk=0): instant acceleration changes
 *   - S-curve (jerk>0): jerk-limited smooth ramps
 *   - Inherited acceleration from previous commands
 *   - Ramp-in (building acceleration) and ramp-out (reducing for arrival)
 *   - Commitment flag to prevent oscillation at the arrival boundary
 *
 * Updates: cmd_acc, cmd_vel, cmd_pos
 *---------------------------------------------------------------------------*/
static void _nc_vel_ramp(NC_AXIS_INTERNAL *p, float goal_vel, float dt)
{
    float old_vel  = p->cmd_vel;
    float diff     = goal_vel - old_vel;
    float abs_diff = _NC_FABS(diff);

    /* ── Already at goal with zero acceleration? ───────────────────── */
    if (abs_diff < _NC_VEL_EPS && _NC_FABS(p->cmd_acc) < _NC_VEL_EPS) {
        p->cmd_vel = goal_vel;
        p->cmd_acc = 0.0f;
        p->cmd_pos += goal_vel * dt;
        return;
    }

    float limit = _nc_select_limit(p, goal_vel);
    float j     = p->jerk;

    /* ══════════════════════════════════════════════════════════════════
     * TRAPEZOIDAL (jerk = 0): instant acceleration changes
     * ══════════════════════════════════════════════════════════════════ */
    if (j <= 0.0f) {
        float step = limit * dt;
        float new_vel;
        if (abs_diff <= step) {
            new_vel = goal_vel;
        } else {
            new_vel = old_vel + _NC_SIGN(diff) * step;
        }
        p->cmd_acc = (new_vel - old_vel) / (dt + 1e-12f);
        p->cmd_pos += 0.5f * (old_vel + new_vel) * dt;
        p->cmd_vel = new_vel;
        return;
    }

    /* ══════════════════════════════════════════════════════════════════
     * S-CURVE (jerk > 0): jerk-limited acceleration ramp
     * ══════════════════════════════════════════════════════════════════ */

    /* Handle edge case: at goal velocity but acceleration non-zero.
     * Ramp acceleration to zero smoothly. */
    if (abs_diff < _NC_VEL_EPS) {
        float sign_acc = _NC_SIGN(p->cmd_acc);
        float new_acc = p->cmd_acc - sign_acc * j * dt;
        /* Don't let acc cross zero */
        if (sign_acc > 0.0f && new_acc < 0.0f) new_acc = 0.0f;
        if (sign_acc < 0.0f && new_acc > 0.0f) new_acc = 0.0f;
        float new_vel = old_vel + new_acc * dt;
        p->cmd_pos += 0.5f * (old_vel + new_vel) * dt;
        p->cmd_vel = new_vel;
        p->cmd_acc = new_acc;
        return;
    }

    float dir        = _NC_SIGN(diff);     /* +1 = need faster, -1 = need slower */
    float acc        = p->cmd_acc;
    float acc_toward = acc * dir;           /* positive when acc helps reach goal */

    /* Distance (in velocity space) to ramp current acc back to zero:
     * dv_ramp_down = a² / (2·j) */
    float dv_ramp_down = (acc_toward > _NC_VEL_EPS)
        ? (acc * acc) / (2.0f * j)
        : 0.0f;

    /* Anticipation margin: discrete-time compensation */
    float margin = _NC_FABS(acc) * dt * 2.0f;

    /* ── Commitment: once velocity-to-goal ≤ ramp-down distance,
     *    commit to reducing acceleration for smooth arrival.
     *    Un-commit if goal moved significantly away. ──────────────── */
    if (!p->cruise_ramp_down &&
        acc_toward >= 0.0f &&
        abs_diff <= dv_ramp_down + margin + _NC_VEL_EPS) {
        p->cruise_ramp_down = true;
    }
    if (p->cruise_ramp_down &&
        abs_diff > dv_ramp_down + margin * 5.0f + _NC_VEL_EPS * 4.0f) {
        /* Goal moved away (ContinuousUpdate or mode change) — un-commit */
        p->cruise_ramp_down = false;
    }

    float new_acc;

    if (acc_toward < -_NC_VEL_EPS) {
        /* Acc opposes desired direction (inherited from previous command).
         * Ramp it toward zero using jerk — don't build in new direction yet. */
        new_acc = acc + dir * j * dt;
        if ((new_acc * dir) > 0.0f) new_acc = 0.0f;
        p->cruise_ramp_down = false;
    } else if (p->cruise_ramp_down) {
        /* Committed to ramp-out: reduce acc to arrive at goal smoothly */
        new_acc = acc - dir * j * dt;
        if ((acc * dir > 0.0f) && (new_acc * dir < 0.0f)) {
            new_acc = 0.0f;
            p->cruise_ramp_down = false;
        }
    } else {
        /* Ramp-in: build acceleration toward limit */
        new_acc = acc + dir * j * dt;
    }

    /* Clamp acceleration magnitude to limit.
     * If inherited |acc| exceeds limit, ramp down smoothly rather than
     * hard-clamping (prevents acceleration discontinuity). */
    if (new_acc * dir > 0.0f && _NC_FABS(new_acc) > limit) {
        if (_NC_FABS(acc) > limit + _NC_VEL_EPS) {
            /* Inherited above limit — ramp toward limit */
            new_acc = acc - dir * j * dt;
            if (_NC_FABS(new_acc) < limit) new_acc = dir * limit;
        } else {
            new_acc = dir * limit;
        }
    }

    /* ── Integrate: acc → vel ──────────────────────────────────────── */
    float new_vel = old_vel + new_acc * dt;

    /* Velocity overshoot clamp (safety net).
     * Compute the exact acceleration that yields goal_vel this cycle
     * instead of hard-zeroing acc (which causes acc discontinuity). */
    if ((dir > 0.0f && new_vel > goal_vel) ||
        (dir < 0.0f && new_vel < goal_vel)) {
        new_vel = goal_vel;
        new_acc = (goal_vel - old_vel) / (dt + 1e-12f);
        p->cruise_ramp_down = false;
    }

    /* Prevent velocity reversal through zero when stopping.
     * Compute exact acc to bring vel to zero instead of hard-zeroing. */
    if (_NC_FABS(goal_vel) < _NC_VEL_EPS) {
        if ((old_vel > _NC_VEL_EPS && new_vel < -_NC_VEL_EPS) ||
            (old_vel < -_NC_VEL_EPS && new_vel > _NC_VEL_EPS)) {
            new_vel = 0.0f;
            new_acc = -old_vel / (dt + 1e-12f);
        }
    }

    /* ── Integrate: vel → pos (trapezoidal for accuracy) ──────────── */
    p->cmd_pos += 0.5f * (old_vel + new_vel) * dt;
    p->cmd_vel = new_vel;
    p->cmd_acc = new_acc;
}

/*---------------------------------------------------------------------------
 * _nc_motion_step — Unified motion generator entry point
 *
 * Computes the effective target velocity based on gen_mode, then feeds
 * it to the shared velocity ramp.
 *
 * Returns true when motion is complete:
 *   POSITION → at target position with zero velocity
 *   VELOCITY → at target velocity (InVelocity)
 *   STOP     → at zero velocity (stopped)
 *---------------------------------------------------------------------------*/
static bool _nc_motion_step(NC_AXIS_INTERNAL *p, float dt)
{
    float eff_vel;

    switch (p->gen_mode) {

    /* ═══════════════════════════════════════════════════════════════════
     * VELOCITY MODE — Infinite trajectory
     *
     * Target velocity is fixed. Ramp to it and hold.
     * ═══════════════════════════════════════════════════════════════════ */
    case NC_GEN_VELOCITY: {
        eff_vel = p->target_vel;

        /* At target velocity? */
        if (_NC_FABS(eff_vel - p->cmd_vel) < _NC_VEL_EPS &&
            _NC_FABS(p->cmd_acc) < _NC_VEL_EPS) {
            p->cmd_vel = eff_vel;
            p->cmd_acc = 0.0f;
            p->cmd_pos += eff_vel * dt;
            p->in_velocity = true;
            return true;
        }
        break;
    }

    /* ═══════════════════════════════════════════════════════════════════
     * STOP MODE — Decelerate to standstill
     *
     * Target velocity is zero. Decelerate from current state.
     * ═══════════════════════════════════════════════════════════════════ */
    case NC_GEN_STOP: {
        eff_vel = 0.0f;

        /* Stopped? */
        if (_NC_FABS(p->cmd_vel) < _NC_VEL_EPS &&
            _NC_FABS(p->cmd_acc) < _NC_VEL_EPS) {
            p->cmd_vel = 0.0f;
            p->cmd_acc = 0.0f;
            return true;
        }
        break;
    }

    /* ═══════════════════════════════════════════════════════════════════
     * POSITION MODE — Finite trajectory
     *
     * Dynamic target velocity based on remaining distance:
     *   - Far from target: eff_vel = v_max (cruise toward target)
     *   - Close to target: eff_vel = v_limit (decelerate to arrive)
     *   - Wrong direction: eff_vel = 0 (stop first, then reverse)
     * ═══════════════════════════════════════════════════════════════════ */
    case NC_GEN_POSITION: {
        float remaining = p->target_pos - p->cmd_pos;
        float dir       = _NC_SIGN(remaining);
        float abs_rem   = _NC_FABS(remaining);

        /* ── Arrival check ─────────────────────────────────────────── */
        if (abs_rem < _NC_POS_EPS && _NC_FABS(p->cmd_vel) < _NC_VEL_EPS) {
            p->cmd_pos = p->target_pos;
            p->cmd_vel = 0.0f;
            p->cmd_acc = 0.0f;
            p->decel_committed = false;
            return true;
        }

        /* ── Wrong-direction velocity: decelerate to zero first ──── */
        float vel_toward = p->cmd_vel * dir;   /* positive = toward target */
        if (vel_toward < -_NC_VEL_EPS) {
            eff_vel = 0.0f;
            break;
        }

        /* ── Compute stopping distance from current state ────────── */
        float abs_vel       = _NC_FMAX(vel_toward, 0.0f);
        float acc_toward    = p->cmd_acc * dir;
        float acc_for_stop  = _NC_FMAX(acc_toward, 0.0f);
        float stop_dist     = _nc_stopping_distance(abs_vel, acc_for_stop,
                                                     p->dec, p->jerk);
        stop_dist += abs_vel * dt * 3.0f;  /* 3-cycle safety margin */

        /* ── Commit to braking when stopping distance ≥ remaining ── */
        if (!p->decel_committed &&
            stop_dist >= abs_rem - _NC_POS_EPS) {
            p->decel_committed = true;
        }

        /* Un-commit if stopped but not yet at target (overshoot recovery) */
        if (p->decel_committed &&
            abs_vel < _NC_VEL_EPS &&
            abs_rem > _NC_POS_EPS * 2.0f) {
            p->decel_committed = false;
        }

        if (p->decel_committed && p->jerk > 0.0f) {
            /* S-curve braking: use direct jerk control to avoid
             * vel_ramp chattering with moving velocity target. */
            _nc_brake_scurve(p, abs_rem, abs_vel, dir, dt);
            return false;  /* skip vel_ramp below */
        } else if (p->decel_committed) {
            /* Trapezoidal braking: v_limit approach works fine */
            float v_limit = _nc_vel_for_dist(abs_rem, p->dec, 0.0f);
            eff_vel = v_limit * dir;
        } else {
            /* Cruise zone: limit to safe velocity that can stop in time */
            float v_safe = _NC_FMIN(p->v_max,
                                     _nc_vel_for_dist(abs_rem, p->dec, p->jerk));
            eff_vel = v_safe * dir;
        }
        break;
    }

    default:
        /* IDLE or unknown — hold still */
        p->cmd_vel = 0.0f;
        p->cmd_acc = 0.0f;
        return true;
    }

    /* ── Feed effective target velocity to the shared velocity ramp ──── */
    _nc_vel_ramp(p, eff_vel, dt);

    return false;
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

    /* ── 1. Run CiA402 state machine ──────────────────────────────────── */
    bool op_en = _nc_cia402_step(nc);

    /* ── 2. Latch new command if Slow Task published one ──────────────── */
    bool new_cmd = _nc_latch_cmd(nc);

    if (new_cmd) {
        /* MC_Stop lock: while StopActive, reject all motion commands.
           Only POWER_OFF and STOP (re-trigger) are allowed through. */
        if (ref->StopActive &&
            p->latched_cmd != NC_CMD_POWER_OFF &&
            p->latched_cmd != NC_CMD_STOP) {
            /* Silently discard — axis stays in STOPPING */
            new_cmd = false;
        }
    }

    if (new_cmd) {
        switch (p->latched_cmd) {
            case NC_CMD_POWER_ON:
                p->power_requested = true;
                p->gen_mode        = NC_GEN_IDLE;
                ref->sts_State     = MC_AXIS_DISABLED;
                ref->sts_Error     = false;
                ref->sts_ErrorID   = 0;
                break;

            case NC_CMD_POWER_OFF:
                p->power_requested = false;
                p->gen_mode        = NC_GEN_IDLE;
                ref->sts_State     = MC_AXIS_DISABLED;
                ref->sts_Busy      = false;
                ref->sts_Done      = false;
                ref->sts_Error     = false;
                p->cmd_vel         = 0.0f;
                p->cmd_acc         = 0.0f;
                break;

            case NC_CMD_MOVE_ABS:
            case NC_CMD_MOVE_REL:
            case NC_CMD_MOVE_ADD:
                p->gen_mode     = NC_GEN_POSITION;
                ref->sts_State  = MC_AXIS_DISCRETE_MOTION;
                ref->sts_Busy   = true;
                ref->sts_Done   = false;
                ref->sts_Error  = false;
                break;

            case NC_CMD_MOVE_VEL:
                p->gen_mode     = NC_GEN_VELOCITY;
                ref->sts_State  = MC_AXIS_CONTINUOUS_MOTION;
                ref->sts_Busy   = true;
                ref->sts_Done   = false;
                ref->sts_Error  = false;
                p->in_velocity  = false;
                break;

            case NC_CMD_HALT:
                p->gen_mode     = NC_GEN_STOP;
                ref->sts_State  = MC_AXIS_STOPPING;
                ref->sts_Busy   = true;
                ref->sts_Done   = false;
                ref->sts_Error  = false;
                break;

            case NC_CMD_STOP:
                p->gen_mode     = NC_GEN_STOP;
                ref->sts_State  = MC_AXIS_STOPPING;
                ref->sts_Busy   = true;
                ref->sts_Done   = false;
                ref->sts_Error  = false;
                p->cmd_vel      = ref->ActualVelocity; /* decel from actual */
                p->cmd_acc      = 0.0f;                /* fresh decel start */
                break;

            case NC_CMD_HOME:
                p->gen_mode     = NC_GEN_IDLE;
                ref->sts_State  = MC_AXIS_HOMING;
                ref->sts_Busy   = true;
                ref->sts_Done   = false;
                ref->sts_Error  = false;
                p->homing_phase = 1;
                break;

            case NC_CMD_NONE:
            default:
                break;
        }
    }

    /* ── 3. If drive not enabled, hold position and wait ──────────────── */
    if (!op_en && !ref->Simulation) {
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
        p->cmd_acc         = 0.0f;
        p->gen_mode        = NC_GEN_IDLE;
        ref->sts_State     = MC_AXIS_STANDSTILL;
        ref->sts_Busy      = false;
        ref->sts_Done      = false;
        if (ref->Simulation ||
            ref->EncoderType == KRON_ENC_ABSOLUTE_ST ||
            ref->EncoderType == KRON_ENC_ABSOLUTE_MT) {
            ref->IsHomed = true;
        }
    }

    /* ── 4. Run motion profile for current state ──────────────────────── */
    switch (ref->sts_State) {

        case MC_AXIS_STANDSTILL:
        case MC_AXIS_DISABLED:
            /* Hold still */
            p->cmd_vel = 0.0f;
            p->cmd_acc = 0.0f;
            ref->sts_Busy = false;
            break;

        case MC_AXIS_DISCRETE_MOTION: {
            /* UNIFIED GENERATOR: Position mode */
            bool done = _nc_motion_step(p, dt);
            ref->sts_Busy = !done;
            if (done) {
                ref->sts_Done  = true;
                ref->sts_State = MC_AXIS_STANDSTILL;
                ref->sts_Busy  = false;
                p->gen_mode    = NC_GEN_IDLE;
            }
            break;
        }

        case MC_AXIS_CONTINUOUS_MOTION: {
            /* UNIFIED GENERATOR: Velocity mode */
            bool at_vel = _nc_motion_step(p, dt);
            ref->sts_Busy = true;
            if (at_vel && !p->in_velocity) {
                p->in_velocity = true;
                ref->sts_Done  = true;   /* "InVelocity" signal to Slow Task */
            }
            break;
        }

        case MC_AXIS_STOPPING: {
            /* UNIFIED GENERATOR: Stop mode */
            if (p->dec < _NC_VEL_EPS) p->dec = 1000.0f;
            bool stopped = _nc_motion_step(p, dt);
            ref->sts_Busy = !stopped;
            if (stopped) {
                ref->sts_Done = true;
                if (!ref->StopActive) {
                    ref->sts_State = MC_AXIS_STANDSTILL;
                    ref->sts_Busy  = false;
                    p->gen_mode    = NC_GEN_IDLE;
                } else {
                    /* Hold position, stay in STOPPING */
                    p->cmd_vel = 0.0f;
                    p->cmd_acc = 0.0f;
                }
            }
            break;
        }

        case MC_AXIS_HOMING: {
            /* ── Simulation / no-slot: instant snap ──── */
            if (ref->Simulation || !ref->slot || !ref->slot->present) {
                p->cmd_pos     = ref->cmd_HomePos;
                p->cmd_vel     = 0.0f;
                p->cmd_acc     = 0.0f;
                ref->IsHomed   = true;
                ref->sts_Done  = true;
                ref->sts_Busy  = false;
                ref->sts_State = MC_AXIS_STANDSTILL;
                p->homing_phase = 0;
                p->gen_mode    = NC_GEN_IDLE;
                break;
            }

            /* ── CiA 402 drive-delegated homing (Mode 6) ──────────── */
            KRON_SERVO_SLOT *hslot = ref->slot;
            uint16_t hsw = hslot->status_word;

            /* Fault during homing → error */
            if (_cia402_fault(hsw)) {
                ref->sts_Error   = true;
                ref->sts_ErrorID = 100u;
                ref->sts_State   = MC_AXIS_ERRORSTOP;
                ref->sts_Busy    = false;
                p->homing_phase  = 0;
                p->gen_mode      = NC_GEN_IDLE;
                break;
            }

            switch (p->homing_phase) {
                case 1:
                    hslot->mode_of_operation = CIA402_MODE_HM;
                    hslot->control_word      = CIA402_CW_OE;
                    if (hslot->mode_display == CIA402_MODE_HM)
                        p->homing_phase = 2;
                    break;

                case 2:
                    hslot->mode_of_operation = CIA402_MODE_HM;
                    hslot->control_word      = CIA402_CW_OE | CIA402_CW_HM_START;
                    p->homing_phase = 3;
                    break;

                case 3: {
                    hslot->mode_of_operation = CIA402_MODE_HM;
                    hslot->control_word      = CIA402_CW_OE | CIA402_CW_HM_START;

                    bool attained = (hsw & CIA402_SW_HOMING_ATTAINED) != 0;
                    bool reached  = (hsw & CIA402_SW_TARGET_REACHED)  != 0;

                    if (attained && reached) {
                        _nc_read_pi(nc);
                        p->cmd_pos  = ref->ActualPosition;
                        p->cmd_vel  = 0.0f;
                        p->cmd_acc  = 0.0f;
                        ref->IsHomed = true;

                        hslot->mode_of_operation = CIA402_MODE_CSP;
                        hslot->control_word      = CIA402_CW_OE;

                        ref->sts_Done  = true;
                        ref->sts_Busy  = false;
                        ref->sts_State = MC_AXIS_STANDSTILL;
                        p->homing_phase = 0;
                        p->gen_mode    = NC_GEN_IDLE;
                    }
                    break;
                }

                default:
                    p->homing_phase = 0;
                    break;
            }
            break;
        }

        case MC_AXIS_ERRORSTOP:
            p->cmd_vel  = 0.0f;
            p->cmd_acc  = 0.0f;
            p->gen_mode = NC_GEN_IDLE;
            ref->sts_Busy = false;
            break;

        case MC_AXIS_SYNCHRONIZED_MOTION:
            /* Future: Gearing / Camming — not implemented yet */
            break;

        default:
            break;
    }

    /* ── 5. Apply superimposed offset (if any) ────────────────────────── */
    if (_NC_FABS(p->superimposed_offset) > _NC_VEL_EPS) {
        float step = p->superimposed_vel * dt;
        if (_NC_FABS(step) > _NC_FABS(p->superimposed_offset))
            step = p->superimposed_offset;
        p->cmd_pos                += step;
        p->superimposed_offset    -= step;
    }

    /* ── 6. Write commanded values back to AXIS_REF ───────────────────── */
    ref->CommandedPosition = p->cmd_pos;
    ref->CommandedVelocity = p->cmd_vel;

    /* ── 7. Read actual values from process image ─────────────────────── */
    _nc_read_pi(nc);

    /* ── 8. Push commanded values to process image output ────────────── */
    _nc_write_pi(nc);

    /* ── 9. Warning flag: following error check ───────────────────────── */
    if (ref->slot && ref->slot->present) {
        int32_t fe = ref->slot->following_error_raw;
        if (fe < 0) fe = -fe;
        ref->AxisWarning = (fe > 10000);
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
