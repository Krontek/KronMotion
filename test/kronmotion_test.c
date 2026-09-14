/*===========================================================================
 * kronmotion_test.c  --  KronMotion self test
 *
 * Drives the function blocks exactly as a PLC program would: every instance is
 * called every cycle, and the axis engine runs between the calls.  The axis is
 * in simulation, so the feedback follows the set values perfectly and what is
 * measured here is the motion layer, not a drive.
 *
 * Every check is on behaviour the standard states: where the axis ends up, in
 * which state, which output is set, and that the limits handed in are never
 * exceeded.
 *===========================================================================*/

#include "kronmotion.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define CYCLE 0.001                       /* 1 ms, the task this runs in */

static int failures = 0;
static int checks   = 0;

static void check(int condition, const char *what)
{
    checks++;
    if (!condition)
    {
        failures++;
        printf("  FAIL  %s\n", what);
    }
}

static void check_near(double got, double want, double tolerance, const char *what)
{
    checks++;
    if (fabs(got - want) > tolerance)
    {
        failures++;
        printf("  FAIL  %s: got %.9f, wanted %.9f (+/- %g)\n", what, got, want, tolerance);
    }
}

/*===========================================================================
 * A test axis and the cycle it runs in
 *===========================================================================*/
static AXIS_REF axis;

/* the worst values seen since the last reset, to test the limits are held */
static double peak_velocity;
static double peak_acceleration;
static double peak_jerk;

static void axis_setup(void)
{
    AXIS_REF_Init(&axis, 0u, NULL);

    axis.Simulation      = true;
    axis.EncoderType     = KRON_ENC_ABSOLUTE_MT;   /* no homing needed for absolute moves */
    axis.IsHomed         = true;

    axis.MaxVelocity     = 500.0;
    axis.MaxAcceleration = 2000.0;
    axis.MaxDeceleration = 2000.0;
    axis.MaxJerk         = 40000.0;

    axis.InPositionWindow = 1e-3;
    axis.InVelocityWindow = 1e-3;

    peak_velocity = peak_acceleration = peak_jerk = 0.0;
}

static void cycle(void)
{
    KronAxis_ReadInput(&axis);
}

static void cycle_end(void)
{
    KronAxis_WriteOutput(&axis, CYCLE);

    if (fabs(axis.Set.Velocity)     > peak_velocity)     { peak_velocity     = fabs(axis.Set.Velocity); }
    if (fabs(axis.Set.Acceleration) > peak_acceleration) { peak_acceleration = fabs(axis.Set.Acceleration); }
    if (fabs(axis.SetJerk)          > peak_jerk)         { peak_jerk         = fabs(axis.SetJerk); }
}

/* bring the axis up: MC_Power until the state machine reports Standstill */
static void power_on(MC_Power *pwr)
{
    int i;

    pwr->Enable = true;

    for (i = 0; i < 10; i++)
    {
        cycle();
        MC_Power_Call(pwr, &axis);
        cycle_end();
    }
}

/*===========================================================================
 * 1. MC_Power brings the axis from Disabled to Standstill
 *===========================================================================*/
static void test_power(void)
{
    MC_Power pwr;

    printf("MC_Power\n");
    axis_setup();
    memset(&pwr, 0, sizeof(pwr));

    check(axis.State == MC_AXIS_DISABLED, "the initial state is Disabled");

    power_on(&pwr);

    check(pwr.Status, "Status follows the power stage");
    check(!pwr.Error, "no error on a clean power on");
    check(axis.State == MC_AXIS_STANDSTILL, "Enable = TRUE in Disabled gives Standstill");

    pwr.Enable = false;
    cycle();
    MC_Power_Call(&pwr, &axis);
    cycle_end();

    check(axis.State == MC_AXIS_DISABLED, "Enable = FALSE gives Disabled");
}

/*===========================================================================
 * 2. MC_MoveAbsolute lands on the target, inside every limit it was given
 *===========================================================================*/
static void test_move_absolute(void)
{
    MC_Power pwr;
    MC_MoveAbsolute move;
    int i;

    printf("MC_MoveAbsolute\n");
    axis_setup();
    memset(&pwr, 0, sizeof(pwr));
    memset(&move, 0, sizeof(move));
    power_on(&pwr);

    move.Position     = 250.0;
    move.Velocity     = 300.0;
    move.Acceleration = 1500.0;
    move.Deceleration = 1500.0;
    move.Jerk         = 30000.0;
    move.Execute      = true;

    for (i = 0; i < 5000 && !move.Done; i++)
    {
        cycle();
        MC_Power_Call(&pwr, &axis);
        MC_MoveAbsolute_Call(&move, &axis);
        cycle_end();

        if (i == 0)
        {
            check(move.Busy && move.Active, "Busy and Active on the first cycle");
            check(axis.State == MC_AXIS_DISCRETE_MOTION, "the axis is in DiscreteMotion");
        }
    }

    check(move.Done, "Done is set when the target is reached");
    check(!move.Busy && !move.Active, "Busy and Active are reset by Done");
    check(!move.Error && !move.CommandAborted, "the outputs are mutually exclusive");
    check_near(axis.Set.Position, 250.0, 1e-3, "the axis lands on the target");
    check(axis.State == MC_AXIS_STANDSTILL, "the axis is back in Standstill");

    check(peak_velocity     <= 300.0   + 1e-6, "the commanded velocity is never exceeded");
    check(peak_acceleration <= 1500.0  + 1e-6, "the commanded acceleration is never exceeded");
    check(peak_jerk         <= 30000.0 + 1e-6, "the commanded jerk is never exceeded");

    /* Done follows the falling edge of Execute (2.4.1) */
    move.Execute = false;
    cycle();
    MC_MoveAbsolute_Call(&move, &axis);
    cycle_end();
    check(!move.Done, "Done is reset on the falling edge of Execute");
}

/*===========================================================================
 * 3. The axis limit wins over an FB that asks for more (2.4.1)
 *===========================================================================*/
static void test_axis_limit(void)
{
    MC_Power pwr;
    MC_MoveAbsolute move;
    int i;

    printf("axis limits\n");
    axis_setup();
    memset(&pwr, 0, sizeof(pwr));
    memset(&move, 0, sizeof(move));
    power_on(&pwr);

    move.Position     = 1000.0;
    move.Velocity     = 5000.0;           /* far above the axis' 500 */
    move.Acceleration = 99999.0;
    move.Deceleration = 99999.0;
    move.Jerk         = 999999.0;
    move.Execute      = true;

    for (i = 0; i < 20000 && !move.Done; i++)
    {
        cycle();
        MC_MoveAbsolute_Call(&move, &axis);
        cycle_end();
    }

    check(move.Done, "the move finishes");
    check(peak_velocity     <= axis.MaxVelocity     + 1e-6, "MaxVelocity holds");
    check(peak_acceleration <= axis.MaxAcceleration + 1e-6, "MaxAcceleration holds");
    check(peak_jerk         <= axis.MaxJerk         + 1e-6, "MaxJerk holds");
}

/*===========================================================================
 * 4. MC_MoveVelocity reaches the velocity and holds it; MC_Halt stops it
 *===========================================================================*/
static void test_move_velocity_and_halt(void)
{
    MC_Power pwr;
    MC_MoveVelocity jog;
    MC_Halt halt;
    int i;

    printf("MC_MoveVelocity + MC_Halt\n");
    axis_setup();
    memset(&pwr, 0, sizeof(pwr));
    memset(&jog, 0, sizeof(jog));
    memset(&halt, 0, sizeof(halt));
    power_on(&pwr);

    jog.Velocity     = -200.0;
    jog.Acceleration = 1000.0;
    jog.Deceleration = 1000.0;
    jog.Jerk         = 20000.0;
    jog.Direction    = mcPositiveDirection;
    jog.Execute      = true;

    for (i = 0; i < 5000 && !jog.InVelocity; i++)
    {
        cycle();
        MC_MoveVelocity_Call(&jog, &axis);
        MC_Halt_Call(&halt, &axis);
        cycle_end();
    }

    check(jog.InVelocity, "InVelocity is set when the commanded velocity is reached");
    check(jog.Busy && jog.Active, "a never ending motion stays Busy and Active");
    check(axis.State == MC_AXIS_CONTINUOUS_MOTION, "the axis is in ContinuousMotion");
    check_near(axis.Set.Velocity, -200.0, 1e-6, "the set velocity is the commanded one");

    /* hold it for a while: it must not wander off the commanded velocity */
    for (i = 0; i < 500; i++)
    {
        cycle();
        MC_MoveVelocity_Call(&jog, &axis);
        MC_Halt_Call(&halt, &axis);
        cycle_end();
        check_near(axis.Set.Velocity, -200.0, 1e-6, "the velocity is held");
        checks--;                          /* one check for the whole loop, not 500 */
    }
    checks++;

    /* MC_Halt aborts the jog and brings the axis to Standstill */
    halt.Deceleration = 1000.0;
    halt.Jerk         = 20000.0;
    halt.Execute      = true;

    for (i = 0; i < 5000 && !halt.Done; i++)
    {
        cycle();
        MC_MoveVelocity_Call(&jog, &axis);
        MC_Halt_Call(&halt, &axis);
        cycle_end();
    }

    check(halt.Done, "MC_Halt reports Done at zero velocity");
    check(jog.CommandAborted, "the aborted MC_MoveVelocity reports CommandAborted");
    check(!jog.InVelocity, "InVelocity is reset when the block is aborted");
    check(axis.State == MC_AXIS_STANDSTILL, "Halt ends in Standstill");
    check_near(axis.Set.Velocity, 0.0, 1e-6, "the axis stands still");
}

/*===========================================================================
 * 5. MC_MoveVelocity honours Direction: negative * negative = positive
 *===========================================================================*/
static void test_direction(void)
{
    MC_Power pwr;
    MC_MoveVelocity jog;
    int i;

    printf("MC_MoveVelocity direction\n");
    axis_setup();
    memset(&pwr, 0, sizeof(pwr));
    memset(&jog, 0, sizeof(jog));
    power_on(&pwr);

    jog.Velocity     = -100.0;
    jog.Direction    = mcNegativeDirection;
    jog.Acceleration = 1000.0;
    jog.Deceleration = 1000.0;
    jog.Jerk         = 20000.0;
    jog.Execute      = true;

    for (i = 0; i < 5000 && !jog.InVelocity; i++)
    {
        cycle();
        MC_MoveVelocity_Call(&jog, &axis);
        cycle_end();
    }

    check_near(axis.Set.Velocity, 100.0, 1e-6,
               "negative velocity with negative direction runs positive");
}

/*===========================================================================
 * 6. MC_Stop holds the axis and refuses everything else (3.3 note 2)
 *===========================================================================*/
static void test_stop_priority(void)
{
    MC_Power pwr;
    MC_MoveVelocity jog;
    MC_Stop stop;
    MC_MoveAbsolute blocked;
    int i;

    printf("MC_Stop priority\n");
    axis_setup();
    memset(&pwr, 0, sizeof(pwr));
    memset(&jog, 0, sizeof(jog));
    memset(&stop, 0, sizeof(stop));
    memset(&blocked, 0, sizeof(blocked));
    power_on(&pwr);

    jog.Velocity     = 300.0;
    jog.Acceleration = 1500.0;
    jog.Deceleration = 1500.0;
    jog.Jerk         = 30000.0;
    jog.Execute      = true;

    for (i = 0; i < 5000 && !jog.InVelocity; i++)
    {
        cycle();
        MC_MoveVelocity_Call(&jog, &axis);
        cycle_end();
    }

    stop.Deceleration = 2000.0;
    stop.Jerk         = 40000.0;
    stop.Execute      = true;

    for (i = 0; i < 5000 && !stop.Done; i++)
    {
        cycle();
        MC_MoveVelocity_Call(&jog, &axis);
        MC_Stop_Call(&stop, &axis);
        cycle_end();

        if (i == 0)
        {
            check(axis.State == MC_AXIS_STOPPING, "the axis goes to Stopping");
        }
        if (i == 1)
        {
            /* the jog's own outputs are written when its FB is called, so the
             * abort shows on the cycle after the stop took the axis */
            check(jog.CommandAborted, "the running motion is aborted by the stop");
        }
    }

    check(stop.Done, "Done is set at zero velocity");
    check(stop.Active, "Active and Done are both set while Execute is high");
    check(axis.State == MC_AXIS_STOPPING, "the axis is held in Stopping while Execute is high");

    /* a motion command issued while the stop holds the axis must be refused */
    blocked.Position     = 10.0;
    blocked.Velocity     = 100.0;
    blocked.Acceleration = 1000.0;
    blocked.Deceleration = 1000.0;
    blocked.Jerk         = 20000.0;
    blocked.Execute      = true;

    cycle();
    MC_Stop_Call(&stop, &axis);
    MC_MoveAbsolute_Call(&blocked, &axis);
    cycle_end();

    check(blocked.Error, "a motion command is refused while MC_Stop holds the axis");
    check(blocked.ErrorID == MC_ERR_STOP_ACTIVE, "and says why");
    check(axis.State == MC_AXIS_STOPPING, "the axis stays in Stopping");

    /* Done AND NOT Execute releases it */
    stop.Execute = false;
    cycle();
    MC_Stop_Call(&stop, &axis);
    cycle_end();

    check(axis.State == MC_AXIS_STANDSTILL, "Done and Execute low gives Standstill");
}

/*===========================================================================
 * 7. mcBuffered: the second move starts when the first is Done
 *===========================================================================*/
static void test_buffered(void)
{
    MC_Power pwr;
    MC_MoveAbsolute first;
    MC_MoveRelative second;
    int i;
    bool saw_both_busy = false;

    printf("mcBuffered\n");
    axis_setup();
    memset(&pwr, 0, sizeof(pwr));
    memset(&first, 0, sizeof(first));
    memset(&second, 0, sizeof(second));
    power_on(&pwr);

    first.Position     = 100.0;
    first.Velocity     = 300.0;
    first.Acceleration = 2000.0;
    first.Deceleration = 2000.0;
    first.Jerk         = 40000.0;
    first.Execute      = true;

    second.Distance     = 50.0;
    second.Velocity     = 200.0;
    second.Acceleration = 2000.0;
    second.Deceleration = 2000.0;
    second.Jerk         = 40000.0;
    second.BufferMode   = mcBuffered;
    second.Execute      = true;

    for (i = 0; i < 20000 && !second.Done; i++)
    {
        cycle();
        MC_MoveAbsolute_Call(&first, &axis);
        MC_MoveRelative_Call(&second, &axis);
        cycle_end();

        if (first.Active && second.Busy && !second.Active)
        {
            saw_both_busy = true;          /* the second one waits in the buffer */
        }
        check(!(first.Active && second.Active), "only one FB is Active at a time");
        checks--;
    }
    checks++;

    check(saw_both_busy, "the buffered FB is Busy but not Active while it waits");
    check(second.Done, "the buffered move finishes");
    check_near(axis.Set.Position, 150.0, 1e-3, "a relative move starts from where the first ended");
    check_near(axis.Set.Velocity, 0.0, 1e-3, "mcBuffered comes to a stop between the moves");
}

/*===========================================================================
 * 8. Blending does not stop between the two moves
 *===========================================================================*/
static void test_blending(void)
{
    MC_Power pwr;
    MC_MoveAbsolute first;
    MC_MoveAbsolute second;
    int i;
    double slowest = 1e30;
    bool past_first = false;

    printf("mcBlendingLow\n");
    axis_setup();
    memset(&pwr, 0, sizeof(pwr));
    memset(&first, 0, sizeof(first));
    memset(&second, 0, sizeof(second));
    power_on(&pwr);

    first.Position     = 100.0;
    first.Velocity     = 300.0;
    first.Acceleration = 2000.0;
    first.Deceleration = 2000.0;
    first.Jerk         = 40000.0;
    first.Execute      = true;

    second.Position     = 300.0;
    second.Velocity     = 200.0;
    second.Acceleration = 2000.0;
    second.Deceleration = 2000.0;
    second.Jerk         = 40000.0;
    second.BufferMode   = mcBlendingLow;
    second.Execute      = true;

    for (i = 0; i < 20000 && !second.Done; i++)
    {
        cycle();
        MC_MoveAbsolute_Call(&first, &axis);
        MC_MoveAbsolute_Call(&second, &axis);
        cycle_end();

        /* the velocity at the moment the first end position is crossed is the
         * one the blend is about */
        if (!past_first && axis.Set.Position >= 100.0)
        {
            past_first = true;
            slowest = fabs(axis.Set.Velocity);
        }
    }

    check(first.Done, "the first move reports Done at its end position");
    check(second.Done, "the blended move finishes");
    check_near(axis.Set.Position, 300.0, 1e-3, "the axis lands on the second target");
    check(past_first, "the axis crossed the first end position");
    check(slowest > 1.0, "the axis does not stop at the first end position");
    check(peak_velocity <= 300.0 + 1e-6, "the velocity limit still holds across the blend");
}

/*===========================================================================
 * 9. Aborting: a second move takes the axis immediately
 *===========================================================================*/
static void test_aborting(void)
{
    MC_Power pwr;
    MC_MoveAbsolute first;
    MC_MoveAbsolute second;
    int i;

    printf("mcAborting\n");
    axis_setup();
    memset(&pwr, 0, sizeof(pwr));
    memset(&first, 0, sizeof(first));
    memset(&second, 0, sizeof(second));
    power_on(&pwr);

    first.Position     = 400.0;
    first.Velocity     = 300.0;
    first.Acceleration = 2000.0;
    first.Deceleration = 2000.0;
    first.Jerk         = 40000.0;
    first.Execute      = true;

    for (i = 0; i < 100; i++)
    {
        cycle();
        MC_MoveAbsolute_Call(&first, &axis);
        cycle_end();
    }

    check(first.Active, "the first move owns the axis");

    second.Position     = 50.0;            /* the other way round, mid flight */
    second.Velocity     = 200.0;
    second.Acceleration = 2000.0;
    second.Deceleration = 2000.0;
    second.Jerk         = 40000.0;
    second.BufferMode   = mcAborting;
    second.Execute      = true;

    for (i = 0; i < 20000 && !second.Done; i++)
    {
        cycle();
        MC_MoveAbsolute_Call(&first, &axis);
        MC_MoveAbsolute_Call(&second, &axis);
        cycle_end();

        if (i == 1)
        {
            check(first.CommandAborted, "the running move reports CommandAborted");
            check(!first.Done, "an interrupted move never reports Done");
        }
    }

    check(second.Done, "the aborting move finishes");
    check_near(axis.Set.Position, 50.0, 1e-3, "on its own target, having turned around");
}

/*===========================================================================
 * 10. MC_SetOverride scales the motion; VelFactor 0 stops without a state change
 *===========================================================================*/
static void test_override(void)
{
    MC_Power pwr;
    MC_MoveVelocity jog;
    MC_SetOverride ovr;
    int i;

    printf("MC_SetOverride\n");
    axis_setup();
    memset(&pwr, 0, sizeof(pwr));
    memset(&jog, 0, sizeof(jog));
    memset(&ovr, 0, sizeof(ovr));
    power_on(&pwr);

    ovr.Enable     = true;
    ovr.VelFactor  = 0.5;
    ovr.AccFactor  = 1.0;
    ovr.JerkFactor = 1.0;

    jog.Velocity     = 400.0;
    jog.Acceleration = 2000.0;
    jog.Deceleration = 2000.0;
    jog.Jerk         = 40000.0;
    jog.Execute      = true;

    for (i = 0; i < 5000 && !jog.InVelocity; i++)
    {
        cycle();
        MC_SetOverride_Call(&ovr, &axis);
        MC_MoveVelocity_Call(&jog, &axis);
        cycle_end();
    }

    check(ovr.Enabled, "the override is accepted");
    check_near(axis.Set.Velocity, 200.0, 1e-3, "half the override gives half the velocity");

    /* note 5: VelFactor 0.0 stops the axis without bringing it to Standstill */
    ovr.VelFactor = 0.0;

    for (i = 0; i < 5000 && fabs(axis.Set.Velocity) > 1e-3; i++)
    {
        cycle();
        MC_SetOverride_Call(&ovr, &axis);
        MC_MoveVelocity_Call(&jog, &axis);
        cycle_end();
    }

    check_near(axis.Set.Velocity, 0.0, 1e-3, "VelFactor 0.0 stops the axis");
    check(axis.State == MC_AXIS_CONTINUOUS_MOTION, "but it does not leave ContinuousMotion");
}

/*===========================================================================
 * 11. Software limits: a velocity command comes to rest on the limit
 *===========================================================================*/
static void test_software_limits(void)
{
    MC_Power pwr;
    MC_MoveVelocity jog;
    int i;
    double worst_breach = 0.0;

    printf("software limits\n");
    axis_setup();
    memset(&pwr, 0, sizeof(pwr));
    memset(&jog, 0, sizeof(jog));

    axis.SwLimitNegative     = -50.0;
    axis.SwLimitPositive     = 100.0;
    axis.EnableLimitNegative = true;
    axis.EnableLimitPositive = true;

    power_on(&pwr);

    jog.Velocity     = 400.0;
    jog.Acceleration = 2000.0;
    jog.Deceleration = 2000.0;
    jog.Jerk         = 40000.0;
    jog.Execute      = true;

    for (i = 0; i < 20000; i++)
    {
        cycle();
        MC_MoveVelocity_Call(&jog, &axis);
        cycle_end();

        if (axis.Set.Position - 100.0 > worst_breach)
        {
            worst_breach = axis.Set.Position - 100.0;
        }
    }

    check(worst_breach < 0.1, "the axis does not run through the positive limit");
    check_near(axis.Set.Velocity, 0.0, 1e-3, "it comes to rest there");
    check(axis.Set.Position <= 100.0 + 0.1, "and stays on the limit");
}

/*===========================================================================
 * 12. MC_MoveSuperimposed rides on top of the running motion
 *===========================================================================*/
static void test_superimposed(void)
{
    MC_Power pwr;
    MC_MoveAbsolute move;
    MC_MoveSuperimposed super;
    int i;

    printf("MC_MoveSuperimposed\n");
    axis_setup();
    memset(&pwr, 0, sizeof(pwr));
    memset(&move, 0, sizeof(move));
    memset(&super, 0, sizeof(super));
    power_on(&pwr);

    move.Position     = 200.0;
    move.Velocity     = 200.0;
    move.Acceleration = 2000.0;
    move.Deceleration = 2000.0;
    move.Jerk         = 40000.0;
    move.Execute      = true;

    for (i = 0; i < 100; i++)
    {
        cycle();
        MC_MoveAbsolute_Call(&move, &axis);
        cycle_end();
    }

    super.Distance     = 10.0;
    super.VelocityDiff = 50.0;
    super.Acceleration = 1000.0;
    super.Deceleration = 1000.0;
    super.Jerk         = 20000.0;
    super.Execute      = true;

    for (i = 0; i < 20000 && !(move.Done && super.Done); i++)
    {
        cycle();
        MC_MoveAbsolute_Call(&move, &axis);
        MC_MoveSuperimposed_Call(&super, &axis);
        cycle_end();
    }

    check(move.Done, "the underlying move is not interrupted");
    check(super.Done, "the superimposed move finishes");
    check_near(super.CoveredDistance, 10.0, 1e-3, "CoveredDistance is the contributed distance");
    check_near(axis.Set.Position + axis.Super.Position, 210.0, 1e-3,
               "the axis ends on the target plus the superimposed distance");
}

/*===========================================================================
 * 13. A drive fault takes the axis to ErrorStop, and MC_Reset brings it back
 *===========================================================================*/
static void test_error_and_reset(void)
{
    MC_Power pwr;
    MC_MoveAbsolute move;
    MC_Reset reset;
    MC_ReadAxisError read_error;
    int i;

    printf("ErrorStop and MC_Reset\n");
    axis_setup();
    memset(&pwr, 0, sizeof(pwr));
    memset(&move, 0, sizeof(move));
    memset(&reset, 0, sizeof(reset));
    memset(&read_error, 0, sizeof(read_error));
    power_on(&pwr);

    move.Position     = 300.0;
    move.Velocity     = 300.0;
    move.Acceleration = 2000.0;
    move.Deceleration = 2000.0;
    move.Jerk         = 40000.0;
    move.Execute      = true;

    for (i = 0; i < 100; i++)
    {
        cycle();
        MC_MoveAbsolute_Call(&move, &axis);
        cycle_end();
    }

    /* the drive faults mid move */
    axis.AxisError   = true;
    axis.AxisErrorID = MC_ERR_AXIS;

    cycle();
    MC_MoveAbsolute_Call(&move, &axis);
    read_error.Enable = true;
    MC_ReadAxisError_Call(&read_error, &axis);
    cycle_end();

    check(axis.State == MC_AXIS_ERRORSTOP, "an axis error gives ErrorStop");

    cycle();
    MC_MoveAbsolute_Call(&move, &axis);
    cycle_end();

    check(move.Error, "the running FB reports Error, not CommandAborted");
    check(move.ErrorID == MC_ERR_AXIS, "with the axis error code");
    check(read_error.AxisErrorID == MC_ERR_AXIS, "MC_ReadAxisError reports it too");

    /* nothing is accepted until the reset */
    {
        MC_MoveAbsolute refused;
        memset(&refused, 0, sizeof(refused));
        refused.Position     = 10.0;
        refused.Velocity     = 100.0;
        refused.Acceleration = 1000.0;
        refused.Deceleration = 1000.0;
        refused.Jerk         = 20000.0;
        refused.Execute      = true;

        cycle();
        MC_MoveAbsolute_Call(&refused, &axis);
        cycle_end();

        check(refused.Error && refused.ErrorID == MC_ERR_STATE,
              "a motion command in ErrorStop is refused");
    }

    reset.Execute = true;
    cycle();
    MC_Reset_Call(&reset, &axis);
    cycle_end();

    check(reset.Done, "MC_Reset reports Done");
    check(axis.State == MC_AXIS_STANDSTILL, "with power on the reset lands in Standstill");
}

/*===========================================================================
 * 14. MC_SetPosition shifts the frame without moving the axis
 *===========================================================================*/
static void test_set_position(void)
{
    MC_Power pwr;
    MC_SetPosition set;
    MC_ReadActualPosition read;
    double before;

    printf("MC_SetPosition\n");
    axis_setup();
    memset(&pwr, 0, sizeof(pwr));
    memset(&set, 0, sizeof(set));
    memset(&read, 0, sizeof(read));
    power_on(&pwr);

    before = axis.Set.Position;

    set.Position = 1000.0;
    set.Relative = false;
    set.Execute  = true;

    cycle();
    MC_SetPosition_Call(&set, &axis);
    read.Enable = true;
    MC_ReadActualPosition_Call(&read, &axis);
    cycle_end();

    check(set.Done, "MC_SetPosition reports Done");
    check_near(axis.Set.Position, 1000.0, 1e-9, "the set position is the new one");
    check_near(axis.Set.Position - axis.ActualPosition, before - before, 1e-9,
               "the following error is untouched");
    check(axis.State == MC_AXIS_STANDSTILL, "no movement is caused");
}

/*===========================================================================
 * 15. MC_ReadStatus and MC_ReadMotionState report what the axis is doing
 *===========================================================================*/
static void test_read_blocks(void)
{
    MC_Power pwr;
    MC_MoveVelocity jog;
    MC_ReadStatus status;
    MC_ReadMotionState motion;
    int i;

    printf("MC_ReadStatus / MC_ReadMotionState\n");
    axis_setup();
    memset(&pwr, 0, sizeof(pwr));
    memset(&jog, 0, sizeof(jog));
    memset(&status, 0, sizeof(status));
    memset(&motion, 0, sizeof(motion));
    power_on(&pwr);

    status.Enable = true;
    motion.Enable = true;
    motion.Source = mcSetValue;

    cycle();
    MC_ReadStatus_Call(&status, &axis);
    cycle_end();

    check(status.Valid && status.Standstill, "Standstill is reported");

    jog.Velocity     = 200.0;
    jog.Acceleration = 1000.0;
    jog.Deceleration = 1000.0;
    jog.Jerk         = 20000.0;
    jog.Execute      = true;

    for (i = 0; i < 20; i++)
    {
        cycle();
        MC_MoveVelocity_Call(&jog, &axis);
        status.Enable = true;
        MC_ReadStatus_Call(&status, &axis);
        motion.Enable = true;
        motion.Source = mcSetValue;
        MC_ReadMotionState_Call(&motion, &axis);
        cycle_end();
    }

    check(status.ContinuousMotion, "ContinuousMotion is reported while jogging");
    check(motion.Accelerating, "Accelerating is reported while speeding up");
    check(motion.DirectionPositive, "the direction is positive");
    check(!motion.DirectionNegative, "and not negative");
}

/*===========================================================================
 * 16. MC_Power's direction permissions bar motion one way (3.1)
 *===========================================================================*/
static void test_direction_permission(void)
{
    MC_Power pwr;
    MC_MoveVelocity jog;
    int i;

    printf("MC_Power direction permissions\n");
    axis_setup();
    memset(&pwr, 0, sizeof(pwr));
    memset(&jog, 0, sizeof(jog));

    pwr.EnablePositive = false;
    pwr.EnableNegative = true;              /* only negative motion is permitted */
    power_on(&pwr);

    jog.Velocity     = 200.0;               /* asks for the barred direction */
    jog.Acceleration = 2000.0;
    jog.Deceleration = 2000.0;
    jog.Jerk         = 40000.0;
    jog.Execute      = true;

    for (i = 0; i < 500; i++)
    {
        cycle();
        MC_Power_Call(&pwr, &axis);
        MC_MoveVelocity_Call(&jog, &axis);
        cycle_end();
    }

    check_near(axis.Set.Velocity, 0.0, 1e-6, "a barred direction does not move");
    check(jog.Busy && !jog.InVelocity, "the FB stays Busy, waiting for the permission");

    /* granting it lets the same command through, no re-trigger needed */
    pwr.EnablePositive = true;

    for (i = 0; i < 5000 && !jog.InVelocity; i++)
    {
        cycle();
        MC_Power_Call(&pwr, &axis);
        MC_MoveVelocity_Call(&jog, &axis);
        cycle_end();
    }

    check(jog.InVelocity, "granting the permission releases the motion");
    check_near(axis.Set.Velocity, 200.0, 1e-6, "at the commanded velocity");
}

/*===========================================================================
 * 17. Nothing is accepted while the power is off
 *===========================================================================*/
static void test_disabled_refuses(void)
{
    MC_MoveAbsolute move;
    MC_Stop stop;

    printf("commands in Disabled\n");
    axis_setup();
    memset(&move, 0, sizeof(move));
    memset(&stop, 0, sizeof(stop));

    move.Position     = 10.0;
    move.Velocity     = 100.0;
    move.Acceleration = 1000.0;
    move.Deceleration = 1000.0;
    move.Jerk         = 20000.0;
    move.Execute      = true;

    stop.Deceleration = 1000.0;
    stop.Jerk         = 20000.0;
    stop.Execute      = true;

    cycle();
    MC_MoveAbsolute_Call(&move, &axis);
    MC_Stop_Call(&stop, &axis);
    cycle_end();

    check(move.Error && move.ErrorID == MC_ERR_STATE, "a move in Disabled is refused");
    check(stop.Error && stop.ErrorID == MC_ERR_STATE, "so is MC_Stop");
    check(axis.State == MC_AXIS_DISABLED, "and the axis stays Disabled");
}

/*===========================================================================
 * main
 *===========================================================================*/
int main(void)
{
    printf("KronMotion self test\n\n");

    test_power();
    test_move_absolute();
    test_axis_limit();
    test_move_velocity_and_halt();
    test_direction();
    test_stop_priority();
    test_buffered();
    test_blending();
    test_aborting();
    test_override();
    test_software_limits();
    test_superimposed();
    test_error_and_reset();
    test_set_position();
    test_read_blocks();
    test_direction_permission();
    test_disabled_refuses();

    printf("\n%d checks, %d failed\n", checks, failures);

    return failures == 0 ? 0 : 1;
}
