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
    nc->priv.jerk        = ref->cmd_Jerk;
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

    /* Mirror raw PDO words into AXIS_REF for monitoring */
    ref->drv_StatusWord  = sw;

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

    /* Set mode of operation EARLY — many drives require this before enabling.
     * CSP (Cyclic Synchronous Position) is the default for NC-style control.
     * During drive-delegated homing the homing state machine owns the mode. */
    if (nc->priv.homing_phase == 0)
        slot->mode_of_operation = CIA402_MODE_CSP;

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
    } else {
        /* Switch-on disabled or other intermediate — send Shutdown */
        slot->control_word = CIA402_CW_SO;
    }

    ref->drv_ControlWord = slot->control_word;
    return op_en;
}

/*===========================================================================
 * S-CURVE (JERK-LIMITED) MOTION PROFILE GENERATOR
 *
 * Triple integration:  jerk → acceleration → velocity → position
 *
 * When jerk > 0:  Full 7-phase S-curve with smooth acceleration ramps.
 * When jerk = 0:  Falls back to trapezoidal profile (instant acc changes).
 *
 * The online (cycle-by-cycle) approach evaluates the stopping distance
 * including the current acceleration ramp-down each cycle, so it handles
 * ContinuousUpdate, on-the-fly velocity changes, and preemption naturally.
 *
 * Stopping distance with jerk:
 *   Phase A: ramp current acceleration to zero → d_A = |v|·t_A + ½·a·t_A²
 *            where t_A = |a| / j
 *   Phase B: jerk-limited deceleration from v_after_A to zero
 *            d_B ≈ v² / (2·dec)  for trapezoidal component
 *            + v·a/(2·j)         for S-curve ramp-in/out overhead
 *   Total is computed by _nc_stopping_distance() below.
 *===========================================================================*/

/* Forward declaration — defined below */
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
     * before we can start decelerating.  Velocity increases during this.
     *
     * Integral:  ∫₀^t_a (v + a·τ − j·τ²/2) dτ
     *          = v·t_a + a·t_a²/2 − j·t_a³/6
     *          = v·t_a + a³/(3·j²)
     *
     * BUG FIX: previous code used a³/(6·j²) — off by factor 2. */
    if (acc_in_dir > 0.0f) {
        float a  = acc_in_dir;
        float t_a = a / jerk;
        d += v * t_a + (a * a * a) / (3.0f * jerk * jerk);
        v += (a * a) / (2.0f * jerk);          /* velocity after ramp  */
    }

    /* Phase B: from (v, a=0) do a full S-curve decel to zero.
     *
     * S-curve decel has 3 sub-phases:
     *   B1: jerk-  → |acc| grows from 0 to dec         t1 = dec/j
     *       Δv1 = dec²/(2·j)
     *
     *   B2: constant decel at −dec                      t2 = Δv2/dec
     *       Δv2 = v − Δv1 − Δv3  (remainder)
     *
     *   B3: jerk+  → |acc| shrinks from dec back to 0   t3 = dec/j
     *       Δv3 = dec²/(2·j)
     *
     * If v is so small that Δv1 + Δv3 > v, we never reach full dec
     * and do a triangular decel profile. */

    float dv_ramp = (dec * dec) / (2.0f * jerk);  /* vel consumed by one ramp */
    float dv_both = 2.0f * dv_ramp;                /* both ramps combined      */

    if (v <= dv_both + _NC_VEL_EPS) {
        /* Triangular decel: never reach full dec.
         * Peak decel a_peak = sqrt(v · j).
         * Distance ≈ v · 2·t_half · 2/3  where t_half = a_peak/j */
        float a_peak = _nc_fsqrt(v * jerk);
        if (a_peak < 1e-6f) return d;
        float t_half = a_peak / jerk;
        d += v * t_half * 1.333333f;
    } else {
        /* Full 3-sub-phase decel */
        float t_ramp = dec / jerk;

        /* B1: ramp-in (jerk−).
         * ∫₀^t_ramp (v − j·τ²/2) dτ = v·t_ramp − j·t_ramp³/6
         *   = v·t_ramp − dv_ramp·t_ramp/3                        */
        d += v * t_ramp - dv_ramp * t_ramp / 3.0f;
        v -= dv_ramp;

        /* B3: ramp-out (jerk+).  d_b3 = dv_ramp·t_ramp/3 = dec³/(6·j²) */
        float d_b3 = dv_ramp * t_ramp / 3.0f;
        float v_b3_entry = dv_ramp;  /* velocity when B3 starts */

        /* B2: constant decel */
        float v_b2 = v - v_b3_entry;  /* velocity consumed at constant dec */
        if (v_b2 > 0.0f) {
            float t_b2 = v_b2 / dec;
            d += (v - v_b2 * 0.5f) * t_b2;  /* average vel × time */
        }

        /* B3: ramp-out */
        d += d_b3;
    }

    /* ── Correction for existing deceleration ──────────────────────────
     *
     * When acc_in_dir < 0 the axis is already decelerating.  The Phase B
     * estimate above assumed decel starts from zero — it overestimates.
     *
     * Approximate savings from having existing decel magnitude |ed|:
     *   savings ≈ v · ed / (2·j)  −  ed³ / (6·j²)
     *
     * Derivation: removing the portion of B1 ramp-in from 0→ed while
     * accounting for the higher velocity entering B2.
     * Tested within ~2-3 % of exact for ed ∈ [0, dec].               */
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
 *
 * Uses one step of Newton-Raphson after a bit-hack initial estimate.
 * Good to ~0.1% accuracy — sufficient for jerk planning.
 *---------------------------------------------------------------------------*/
static float _nc_fsqrt(float x)
{
    if (x <= 0.0f) return 0.0f;
    /* Quake-style initial estimate */
    union { float f; uint32_t i; } u = { .f = x };
    u.i = 0x5f3759df - (u.i >> 1);     /* inverse sqrt estimate */
    float inv = u.f;
    inv = inv * (1.5f - 0.5f * x * inv * inv);  /* one Newton step */
    return x * inv;                     /* x · (1/√x) = √x */
}

/*===========================================================================
 * _nc_profile_vel — Jerk-limited velocity ramp, one step.
 *
 * Ramps cmd_vel toward target_vel using S-curve acceleration.
 * When jerk=0, falls back to trapezoidal (instant acceleration change).
 *
 * Updates: cmd_acc, cmd_vel, cmd_pos.
 * Returns true when target velocity has been reached (within tolerance).
 *===========================================================================*/
static bool _nc_profile_vel(NC_AXIS_INTERNAL *p, float dt)
{
    float diff = p->target_vel - p->cmd_vel;

    /* Arrived at target velocity? */
    if (_NC_FABS(diff) < _NC_VEL_EPS && _NC_FABS(p->cmd_acc) < _NC_VEL_EPS) {
        p->cmd_vel = p->target_vel;
        p->cmd_acc = 0.0f;
        p->cmd_pos += p->cmd_vel * dt;
        return true;
    }

    float dir = (diff >= 0.0f) ? 1.0f : -1.0f;  /* +1 = speed up, -1 = slow down */
    float abs_diff = _NC_FABS(diff);
    float j = p->jerk;

    if (j <= 0.0f) {
        /*--- Trapezoidal: instant acceleration changes ---*/
        float limit = (dir > 0.0f) ? p->acc : p->dec;
        float new_vel = p->cmd_vel + dir * limit * dt;
        if ((dir > 0.0f && new_vel > p->target_vel) ||
            (dir < 0.0f && new_vel < p->target_vel))
            new_vel = p->target_vel;
        p->cmd_acc = (new_vel - p->cmd_vel) / (dt + 1e-12f);
        p->cmd_vel = new_vel;
        p->cmd_pos += p->cmd_vel * dt;
        return _NC_FABS(p->target_vel - p->cmd_vel) < _NC_VEL_EPS;
    }

    /*--- S-curve: jerk-limited acceleration ramp ---*/
    float acc = p->cmd_acc;
    float acc_limit = (dir > 0.0f) ? p->acc : p->dec;

    /* Distance (in velocity space) to decelerate current acc to zero:
     * dv_ramp_down = a²/(2·j) */
    float acc_toward = acc * dir;  /* positive when acc helps reach target */
    float dv_ramp_down = (acc_toward > 0.0f)
        ? (acc * acc) / (2.0f * j)
        : 0.0f;

    /* Anticipation margin: account for velocity added this cycle plus
     * discrete-time lag (same idea as _nc_profile_pos stopping margin) */
    float margin = _NC_FABS(acc) * dt * 2.0f;

    float new_acc;
    if (acc_toward < -_NC_VEL_EPS) {
        /* Acc is opposing desired direction (inherited from previous cmd).
         * Ramp it toward zero with jerk — don't use ramp-down/up logic. */
        new_acc = acc + dir * j * dt;
        /* Don't let it cross zero and build in desired direction yet —
         * that's handled by the normal branch once acc_toward >= 0. */
        if ((new_acc * dir) > 0.0f)
            new_acc = 0.0f;
    } else if (abs_diff <= dv_ramp_down + margin + _NC_VEL_EPS) {
        /* Must start reducing acceleration to arrive at target vel smoothly */
        new_acc = acc - dir * j * dt;
        /* Don't overshoot zero (would push vel away from target) */
        if ((acc * dir > 0.0f) && (new_acc * dir < 0.0f))
            new_acc = 0.0f;
    } else {
        /* Can still increase acceleration toward acc_limit */
        new_acc = acc + dir * j * dt;
    }

    /* Clamp acceleration only in the desired direction.
     * When acc is opposing (being ramped toward zero by jerk above),
     * don't hard-clamp — let jerk do the work smoothly. */
    if (new_acc * dir > 0.0f) {
        if (_NC_FABS(new_acc) > acc_limit)
            new_acc = dir * acc_limit;
    }

    /* Integrate: acc → vel */
    float new_vel = p->cmd_vel + new_acc * dt;

    /* Velocity overshoot clamp (safety net — ramp-down should prevent this) */
    if ((dir > 0.0f && new_vel > p->target_vel) ||
        (dir < 0.0f && new_vel < p->target_vel)) {
        new_vel = p->target_vel;
        new_acc = 0.0f;
    }

    /* Integrate: vel → pos */
    p->cmd_pos += 0.5f * (p->cmd_vel + new_vel) * dt;  /* trapezoidal integration */
    p->cmd_vel = new_vel;
    p->cmd_acc = new_acc;

    return false;
}

/*===========================================================================
 * _nc_profile_pos — Jerk-limited position profile, one step.
 *
 * Moves cmd_pos toward target_pos with S-curve acceleration/deceleration.
 * Acceleration is smoothly ramped by jerk (when jerk > 0).
 *
 * Algorithm:
 *   1. Compute stopping distance from current (vel, acc) state.
 *   2. If remaining distance <= stopping distance → decelerate (reverse jerk).
 *   3. Else → accelerate toward v_max (apply jerk toward acc limit).
 *   4. Near target, snap to exact position.
 *
 * Updates: cmd_acc, cmd_vel, cmd_pos.
 * Returns true when at target position with zero velocity.
 *===========================================================================*/
static bool _nc_profile_pos(NC_AXIS_INTERNAL *p, float dt)
{
    float remaining = p->target_pos - p->cmd_pos;
    float dir = (remaining >= 0.0f) ? 1.0f : -1.0f;
    float abs_rem = _NC_FABS(remaining);

    /* Current velocity and acceleration projected onto direction of travel */
    float vel = p->cmd_vel * dir;    /* positive = toward target */
    float acc = p->cmd_acc * dir;    /* positive = accelerating toward target */

    /* ── Arrival check ─────────────────────────────────────────────────── */
    if (abs_rem < _NC_POS_EPS && _NC_FABS(vel) < _NC_VEL_EPS) {
        p->cmd_pos = p->target_pos;
        p->cmd_vel = 0.0f;
        p->cmd_acc = 0.0f;
        return true;
    }

    float j = p->jerk;

    if (j <= 0.0f) {
        /*--- Trapezoidal: instant acceleration changes ---*/
        float abs_vel = _NC_FMAX(vel, 0.0f);
        float brake_dist = (abs_vel * abs_vel) / (2.0f * p->dec + 1e-9f);
        float new_abs_vel;

        if (vel < -_NC_VEL_EPS) {
            /* Wrong direction: decelerate then reverse */
            new_abs_vel = _NC_FABS(vel) - p->dec * dt;
            if (new_abs_vel < 0.0f) new_abs_vel = 0.0f;
            float nv = -new_abs_vel * dir;  /* keep wrong direction, reducing */
            p->cmd_acc = (nv - p->cmd_vel) / (dt + 1e-12f);
            p->cmd_pos += 0.5f * (p->cmd_vel + nv) * dt;
            p->cmd_vel = nv;
            return false;
        } else if (brake_dist >= abs_rem) {
            new_abs_vel = abs_vel - p->dec * dt;
            if (new_abs_vel < 0.0f) new_abs_vel = 0.0f;
        } else if (abs_vel > p->v_max) {
            new_abs_vel = abs_vel - p->dec * dt;
            if (new_abs_vel < p->v_max) new_abs_vel = p->v_max;
        } else {
            new_abs_vel = abs_vel + p->acc * dt;
            if (new_abs_vel > p->v_max) new_abs_vel = p->v_max;
        }

        float nv = new_abs_vel * dir;
        float np = p->cmd_pos + 0.5f * (p->cmd_vel + nv) * dt;

        if (dir > 0.0f && np > p->target_pos) { np = p->target_pos; nv = 0.0f; }
        if (dir < 0.0f && np < p->target_pos) { np = p->target_pos; nv = 0.0f; }

        p->cmd_acc = (nv - p->cmd_vel) / (dt + 1e-12f);
        p->cmd_vel = nv;
        p->cmd_pos = np;
        return false;
    }

    /*--- S-curve position mode ---*/

    /* Handle wrong-direction velocity: must decelerate first.
     * Cap at min(dec, acc) so there's no discontinuity when vel crosses
     * zero and the normal branch takes over (which clamps to acc).
     *
     * Also cap so that when vel crosses zero, the acc ramp-down to zero
     * doesn't overshoot v_max.  Ramp-down from a to 0 adds dv = a²/(2j)
     * velocity.  Need: a ≤ sqrt(2 · j · v_max). */
    if (vel < -_NC_VEL_EPS) {
        float limit = _NC_FMIN(p->dec, p->acc);
        float vmax_limit = _nc_fsqrt(2.0f * j * p->v_max);
        limit = _NC_FMIN(limit, vmax_limit);
        float new_acc = acc + j * dt;
        if (new_acc > limit) new_acc = limit;
        float nv = (vel + new_acc * dt) * dir;
        p->cmd_pos += 0.5f * (p->cmd_vel + nv) * dt;
        p->cmd_vel = nv;
        p->cmd_acc = new_acc * dir;
        return false;
    }

    /* Positive velocity toward target (or zero) */
    float abs_vel = _NC_FMAX(vel, 0.0f);

    /* Compute stopping distance from current state.
     * Pass signed acc so the estimate accounts for existing deceleration.
     * Add small margin to compensate for discrete-time integration error. */
    float stop_dist = _nc_stopping_distance(abs_vel, acc, p->dec, j);
    stop_dist += abs_vel * dt * 3.0f;  /* ~3 cycle margin */

    float new_acc_dir;  /* acceleration in direction of motion */

    if (stop_dist >= abs_rem - _NC_POS_EPS) {
        /*--- DECELERATION ZONE ---
         * Need to slow down. Three sub-decisions:
         * 1. If acc > 0: first ramp acc down to 0 (apply negative jerk)
         * 2. If acc ≤ 0 and v > a²/(2j): ramp-in (build deceleration)
         * 3. If acc ≤ 0 and v ≤ a²/(2j): ramp-out (reduce deceleration) */

        if (acc > _NC_VEL_EPS) {
            /* Still accelerating — ramp down to zero first */
            new_acc_dir = acc - j * dt;
            if (new_acc_dir < 0.0f) new_acc_dir = 0.0f;
        } else {
            /* Decelerating: decide ramp-in vs ramp-out.
             *
             * Ramp-out condition: velocity is low enough that ramping
             * acceleration from current value to zero will bring v to 0.
             *   During ramp-out:  Δv = |a|² / (2·j)
             *   So ramp-out when:  v ≤ |a|² / (2·j)                   */
            float abs_a = _NC_FABS(acc);
            float dv_ramp_out = (abs_a * abs_a) / (2.0f * j);

            if (abs_vel <= dv_ramp_out + _NC_VEL_EPS * 2.0f) {
                /* RAMP-OUT: reduce |deceleration| → arrive at v=0 smoothly */
                new_acc_dir = acc + j * dt;
                if (new_acc_dir > 0.0f) new_acc_dir = 0.0f;
            } else {
                /* RAMP-IN: build deceleration magnitude */
                new_acc_dir = acc - j * dt;
                if (new_acc_dir < -p->dec) new_acc_dir = -p->dec;
            }
        }
    } else {
        /*--- ACCELERATION / CRUISE ZONE ---*/
        if (abs_vel > p->v_max + _NC_VEL_EPS) {
            /* Above v_max (e.g. new command lowered v_max mid-motion):
             * jerk-limited deceleration toward v_max, same ramp-in/out
             * logic as decel zone but targeting v_max instead of zero. */
            float vel_excess = abs_vel - p->v_max;
            float abs_a = (acc < -_NC_VEL_EPS) ? -acc : 0.0f;
            float dv_ramp_out = (abs_a * abs_a) / (2.0f * j + 1e-9f);

            if (vel_excess <= dv_ramp_out + _NC_VEL_EPS) {
                /* Ramp-out: ease off deceleration to arrive at v_max */
                new_acc_dir = acc + j * dt;
                if (new_acc_dir > 0.0f) new_acc_dir = 0.0f;
            } else {
                /* Ramp-in: build deceleration */
                new_acc_dir = acc - j * dt;
                if (new_acc_dir < -p->dec) new_acc_dir = -p->dec;
            }
        } else if (abs_vel >= p->v_max - _NC_VEL_EPS) {
            /* At cruise speed — ramp acceleration to zero */
            if (acc > _NC_VEL_EPS) {
                new_acc_dir = acc - j * dt;
                if (new_acc_dir < 0.0f) new_acc_dir = 0.0f;
            } else if (acc < -_NC_VEL_EPS) {
                /* Negative acc (from above-v_max decel): ramp up to 0 */
                new_acc_dir = acc + j * dt;
                if (new_acc_dir > 0.0f) new_acc_dir = 0.0f;
            } else {
                new_acc_dir = 0.0f;
            }
        } else {
            /* Below v_max */
            if (acc < -_NC_VEL_EPS) {
                /* Negative acc (from above-v_max decel): ramp toward 0 first */
                new_acc_dir = acc + j * dt;
                if (new_acc_dir > 0.0f) new_acc_dir = 0.0f;
            } else {
                /* Normal acceleration toward v_max */
                new_acc_dir = acc + j * dt;
                if (new_acc_dir > p->acc) new_acc_dir = p->acc;

                /* Don't overshoot v_max — check if we need to ramp down acc */
                float vel_headroom = p->v_max - abs_vel;
                float dv_ramp_down = (new_acc_dir * new_acc_dir) / (2.0f * j);
                if (vel_headroom <= dv_ramp_down + _NC_VEL_EPS) {
                    new_acc_dir = acc - j * dt;
                    if (new_acc_dir < 0.0f) new_acc_dir = 0.0f;
                }
            }
        }
    }

    /* ── Integrate state ──────────────────────────────────────────────── */
    float real_acc = new_acc_dir * dir;           /* back to world frame */
    float new_vel = p->cmd_vel + real_acc * dt;

    /* No velocity safety clamp — the S-curve profile code handles v_max
     * via the cruise-entry ramp-down and the "above v_max" branch.
     * Small temporary overshoot during cruise entry is normal S-curve
     * behavior (vel settles to v_max as acc ramps to 0). */

    /* Ensure velocity doesn't reverse past zero while decelerating */
    if (vel >= 0.0f && new_vel * dir < -_NC_VEL_EPS) {
        new_vel = 0.0f;
        real_acc = 0.0f;
    }

    float new_pos = p->cmd_pos + 0.5f * (p->cmd_vel + new_vel) * dt;

    /* Overshoot clamp */
    if (dir > 0.0f && new_pos > p->target_pos) {
        new_pos = p->target_pos; new_vel = 0.0f; real_acc = 0.0f;
    }
    if (dir < 0.0f && new_pos < p->target_pos) {
        new_pos = p->target_pos; new_vel = 0.0f; real_acc = 0.0f;
    }

    p->cmd_acc = real_acc;
    p->cmd_vel = new_vel;
    p->cmd_pos = new_pos;
    return false;
}

/*===========================================================================
 * _nc_decel_to_zero — Jerk-limited deceleration to standstill, one step.
 *
 * Used by MC_Halt / MC_Stop states (STOPPING).
 * Brings velocity to zero smoothly when jerk > 0.
 *
 * Returns true when stopped.
 *===========================================================================*/
static bool _nc_decel_to_zero(NC_AXIS_INTERNAL *p, float dt)
{
    float vel = p->cmd_vel;
    float acc = p->cmd_acc;

    /* Already stopped? */
    if (_NC_FABS(vel) < _NC_VEL_EPS && _NC_FABS(acc) < _NC_VEL_EPS) {
        p->cmd_vel = 0.0f;
        p->cmd_acc = 0.0f;
        return true;
    }

    float j = p->jerk;

    if (j <= 0.0f) {
        /*--- Trapezoidal: instant deceleration ---*/
        float sign = _NC_SIGN(vel);
        float new_vel = vel - sign * p->dec * dt;
        if (sign > 0.0f && new_vel < 0.0f) new_vel = 0.0f;
        if (sign < 0.0f && new_vel > 0.0f) new_vel = 0.0f;
        p->cmd_pos += 0.5f * (vel + new_vel) * dt;
        p->cmd_acc = (new_vel - vel) / (dt + 1e-12f);
        p->cmd_vel = new_vel;
        return _NC_FABS(new_vel) < _NC_VEL_EPS;
    }

    /*--- S-curve deceleration ---*/
    float sign = _NC_SIGN(vel);
    float abs_vel = _NC_FABS(vel);
    float a_toward_zero = -acc * sign;  /* positive when decelerating correctly */

    /* Distance/velocity to ramp current decel back to zero:
     * dv = a²/(2·j) */
    float dv_ramp_out = (a_toward_zero > 0.0f)
        ? (a_toward_zero * a_toward_zero) / (2.0f * j)
        : 0.0f;

    float new_acc;
    if (abs_vel <= dv_ramp_out + _NC_VEL_EPS * 2.0f) {
        /* Close to stop — ramp acceleration back toward zero */
        new_acc = acc + sign * j * dt;
        /* Don't let acc overshoot zero */
        if (sign > 0.0f && new_acc > 0.0f) new_acc = 0.0f;
        if (sign < 0.0f && new_acc < 0.0f) new_acc = 0.0f;
    } else {
        /* Build deceleration (increase |acc| opposing velocity) */
        new_acc = acc - sign * j * dt;
        /* Clamp magnitude to dec limit */
        if (_NC_FABS(new_acc) > p->dec) new_acc = -sign * p->dec;
    }

    float new_vel = vel + new_acc * dt;

    /* Don't reverse through zero */
    if (sign > 0.0f && new_vel < 0.0f) { new_vel = 0.0f; new_acc = 0.0f; }
    if (sign < 0.0f && new_vel > 0.0f) { new_vel = 0.0f; new_acc = 0.0f; }

    p->cmd_pos += 0.5f * (vel + new_vel) * dt;
    p->cmd_vel = new_vel;
    p->cmd_acc = new_acc;

    return _NC_FABS(new_vel) < _NC_VEL_EPS && _NC_FABS(new_acc) < _NC_VEL_EPS;
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
                p->cmd_acc         = 0.0f;
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
                p->cmd_acc      = 0.0f;                /* fresh decel start */
                break;

            case NC_CMD_HOME:
                ref->sts_State  = MC_AXIS_HOMING;
                ref->sts_Busy   = true;
                ref->sts_Done   = false;
                ref->sts_Error  = false;
                p->homing_phase = 1;  /* start CiA402 homing sequence */
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
        p->cmd_acc         = 0.0f;
        ref->sts_State     = MC_AXIS_STANDSTILL;
        ref->sts_Busy      = false;
        ref->sts_Done      = false;
        /* Absolute encoders are inherently homed — position is valid at power-on */
        if (ref->EncoderType == KRON_ENC_ABSOLUTE_ST ||
            ref->EncoderType == KRON_ENC_ABSOLUTE_MT) {
            ref->IsHomed = true;
        }
    }

    /* ── 4. Run motion profile for current state ──────────────────────────── */
    switch (ref->sts_State) {

        case MC_AXIS_STANDSTILL:
        case MC_AXIS_DISABLED:
            /* Hold still */
            p->cmd_vel = 0.0f;
            p->cmd_acc = 0.0f;
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
            if (p->dec < _NC_VEL_EPS) p->dec = 1000.0f;  /* safety default */
            bool stopped = _nc_decel_to_zero(p, dt);
            ref->sts_Busy = !stopped;
            if (stopped) {
                ref->sts_Done  = true;
                ref->sts_State = MC_AXIS_STANDSTILL;
                ref->sts_Busy  = false;
            }
            break;
        }

        case MC_AXIS_HOMING: {
            /* ── Simulation / no-slot: instant snap (legacy behaviour) ──── */
            if (ref->Simulation || !ref->slot || !ref->slot->present) {
                p->cmd_pos     = ref->cmd_HomePos;
                p->cmd_vel     = 0.0f;
                p->cmd_acc     = 0.0f;
                ref->IsHomed   = true;
                ref->sts_Done  = true;
                ref->sts_Busy  = false;
                ref->sts_State = MC_AXIS_STANDSTILL;
                p->homing_phase = 0;
                break;
            }

            /* ── CiA 402 drive-delegated homing (Mode 6) ──────────────── */
            KRON_SERVO_SLOT *hslot = ref->slot;
            uint16_t hsw = hslot->status_word;

            /* Fault during homing → error */
            if (_cia402_fault(hsw)) {
                ref->sts_Error   = true;
                ref->sts_ErrorID = 0x8010u;
                ref->sts_State   = MC_AXIS_ERRORSTOP;
                ref->sts_Busy    = false;
                p->homing_phase  = 0;
                break;
            }

            switch (p->homing_phase) {
                case 1:
                    /* Phase 1: request Homing Mode (mode 6) */
                    hslot->mode_of_operation = CIA402_MODE_HM;
                    hslot->control_word      = CIA402_CW_OE;  /* enabled, no start yet */
                    if (hslot->mode_display == CIA402_MODE_HM)
                        p->homing_phase = 2;
                    break;

                case 2:
                    /* Phase 2: assert Homing Start (controlword bit 4) */
                    hslot->mode_of_operation = CIA402_MODE_HM;
                    hslot->control_word      = CIA402_CW_OE | CIA402_CW_HM_START;
                    /* Transition to waiting once the drive clears Homing Attained
                     * (some drives clear bit 12 on start, others don't — safe
                     * to advance immediately and let phase 3 detect completion). */
                    p->homing_phase = 3;
                    break;

                case 3: {
                    /* Phase 3: wait for Homing Attained (bit 12) + Target Reached (bit 10) */
                    hslot->mode_of_operation = CIA402_MODE_HM;
                    hslot->control_word      = CIA402_CW_OE | CIA402_CW_HM_START;

                    bool attained = (hsw & CIA402_SW_HOMING_ATTAINED) != 0;
                    bool reached  = (hsw & CIA402_SW_TARGET_REACHED)  != 0;

                    if (attained && reached) {
                        /* Homing complete — sync NC position from drive actual */
                        _nc_read_pi(nc);
                        p->cmd_pos  = ref->ActualPosition;
                        p->cmd_vel  = 0.0f;
                        p->cmd_acc  = 0.0f;
                        ref->IsHomed = true;

                        /* Switch back to CSP mode */
                        hslot->mode_of_operation = CIA402_MODE_CSP;
                        hslot->control_word      = CIA402_CW_OE;

                        ref->sts_Done  = true;
                        ref->sts_Busy  = false;
                        ref->sts_State = MC_AXIS_STANDSTILL;
                        p->homing_phase = 0;
                    }
                    break;
                }

                default:
                    /* Unexpected phase — reset */
                    p->homing_phase = 0;
                    break;
            }
            break;
        }

        case MC_AXIS_ERRORSTOP:
            p->cmd_vel = 0.0f;
            p->cmd_acc = 0.0f;
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
