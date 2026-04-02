/*===========================================================================
 * kron_nc.h  --  KronEditor NC (Numerical Control) Engine Interface
 *
 * Runs in the Fast Task (~1ms strict cycle).
 *
 * Responsibilities:
 *   1. Call HAL_Read_Inputs()  at start of cycle (fieldbus PDO → Kron_PI)
 *   2. For each axis: run trapezoidal profile interpolation
 *   3. Manage CiA402 (DS-402) state machine per axis
 *   4. Write AXIS_REF.sts_* so Slow Task FBs can read status
 *   5. Call HAL_Write_Outputs() at end of cycle (Kron_PI → fieldbus PDO)
 *
 * Dependencies: kron_pi.h, kronmotion.h
 *
 * Usage in Fast Task entry point:
 *
 *   void fast_task_entry(void)   // called every 1ms by RTOS / timer ISR
 *   {
 *       NC_ProcessAxes(g_axes, NUM_AXES, 0.001f);
 *   }
 *
 *===========================================================================*/

#ifndef KRON_NC_H
#define KRON_NC_H

#include <stdint.h>
#include <stdbool.h>
#include "kron_pi.h"
#include "kronmotion.h"

/*===========================================================================
 * NC_AXIS_INTERNAL
 *
 * NC Engine private state per axis.  Not visible to Slow Task.
 * Embedded inside NC_AXIS (below).
 *===========================================================================*/
typedef struct {
    /* ── Profile constraints (latched from command channel) ────────────── */
    float   target_pos;         /* Absolute position goal [u]              */
    float   target_vel;         /* Target velocity (signed, vel mode) [u/s]*/
    float   v_max;              /* Profile velocity limit [u/s]            */
    float   acc;                /* Max acceleration [u/s^2]                */
    float   dec;                /* Max deceleration [u/s^2]                */
    float   jerk;               /* Jerk limit [u/s^3], 0 = trapezoidal    */

    /* ── S-curve integrator state (triple integration: jerk→acc→vel→pos) */
    float   cmd_pos;            /* Current commanded position [u]          */
    float   cmd_vel;            /* Current commanded velocity [u/s]        */
    float   cmd_acc;            /* Current commanded acceleration [u/s^2]  */

    /* ── Latched command from Slow Task ─────────────────────────────── */
    uint16_t    latched_seq;    /* cmd_Seq value when we latched the cmd   */
    NC_CMD_TYPE latched_cmd;    /* Command type latched from AXIS_REF      */

    /* ── Velocity mode flag ─────────────────────────────────────────── */
    bool    in_velocity;        /* True when cmd_vel has reached target_vel */

    /* ── CiA402 control state ───────────────────────────────────────── */
    bool    power_requested;    /* MC_Power Enable = TRUE acknowledged      */
    bool    op_enabled;         /* Drive is in Operation Enabled state      */

    /* ── Superimposed offset accumulator ────────────────────────────── */
    float   superimposed_offset;
    float   superimposed_vel;

} NC_AXIS_INTERNAL;

/*===========================================================================
 * NC_AXIS
 *
 * Pairing of user-visible AXIS_REF (shared with Slow Task) and the
 * NC engine's private internal state.
 *
 * Allocated by the generated plc.c (or application); passed to NC_Init /
 * NC_ProcessAxes as an array.
 *===========================================================================*/
typedef struct {
    AXIS_REF         *ref;      /* Pointer to the shared AXIS_REF struct  */
    NC_AXIS_INTERNAL  priv;     /* NC engine private state (not shared)   */
} NC_AXIS;

/*===========================================================================
 * API
 *===========================================================================*/

/*
 * NC_Init — call once at startup for each axis.
 *   nc   : NC_AXIS to initialize
 *   ref  : shared AXIS_REF for this axis (must already be AXIS_REF_Init'd)
 */
void NC_Init(NC_AXIS *nc, AXIS_REF *ref);

/*
 * NC_ProcessAxes — call every Fast Task cycle.
 *   axes     : array of NC_AXIS
 *   count    : number of axes
 *   dt       : cycle time in seconds (e.g. 0.001f for 1ms)
 *
 * Internally calls HAL_Read_Inputs() then HAL_Write_Outputs().
 */
void NC_ProcessAxes(NC_AXIS *axes, uint16_t count, float dt);

/*
 * NC_ProcessOne — process a single axis (called by NC_ProcessAxes).
 * Exposed for unit testing.
 */
void NC_ProcessOne(NC_AXIS *nc, float dt);

#endif /* KRON_NC_H */
