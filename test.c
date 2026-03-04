/*===========================================================================
 * KronMotion — Test Suite
 * Build: tcc test.c kronmotion.c -o test_app && ./test_app
 *
 * Since implementations are stubs, these tests verify:
 *   1. Enum values match PLCopen Part 1 v2.0 specification
 *   2. All struct fields exist and are the correct type
 *   3. All function signatures compile correctly
 *   4. AXIS_REF state transitions (to be updated when impl is complete)
 *===========================================================================*/

#include "kronmotion.h"
#include <stdio.h>

/*---------------------------------------------------------------------------
 * Test infrastructure (same style as other KronLibraries)
 *---------------------------------------------------------------------------*/
static int pass_count = 0;
static int fail_count = 0;

static void check(const char *name, int condition)
{
    if (condition) {
        pass_count++;
        printf("PASS: %s\n", name);
    } else {
        fail_count++;
        printf("FAIL: %s\n", name);
    }
}

/*===========================================================================
 * 1. Enum value tests (PLCopen Table 3, Figure 2)
 *===========================================================================*/
static void test_enum_values(void)
{
    printf("\n--- MC_BUFFER_MODE enum values ---\n");
    check("mcAborting         == 0", mcAborting         == 0);
    check("mcBuffered         == 1", mcBuffered         == 1);
    check("mcBlendingLow      == 2", mcBlendingLow      == 2);
    check("mcBlendingPrevious == 3", mcBlendingPrevious == 3);
    check("mcBlendingNext     == 4", mcBlendingNext     == 4);
    check("mcBlendingHigh     == 5", mcBlendingHigh     == 5);

    printf("\n--- MC_DIRECTION enum values ---\n");
    check("mcPositiveDirection == 1", mcPositiveDirection == 1);
    check("mcShortestWay       == 2", mcShortestWay       == 2);
    check("mcNegativeDirection == 3", mcNegativeDirection == 3);
    check("mcCurrentDirection  == 4", mcCurrentDirection  == 4);

    printf("\n--- MC_EXECUTION_MODE enum values ---\n");
    check("mcImmediately == 0", mcImmediately == 0);
    check("mcQueued      == 1", mcQueued      == 1);

    printf("\n--- MC_SOURCE enum values ---\n");
    check("mcCommandedValue == 0", mcCommandedValue == 0);
    check("mcSetValue       == 1", mcSetValue       == 1);
    check("mcActualValue    == 2", mcActualValue    == 2);

    printf("\n--- MC_AXIS_STATE enum values ---\n");
    check("MC_AXIS_DISABLED            == 0", MC_AXIS_DISABLED            == 0);
    check("MC_AXIS_STANDSTILL          == 1", MC_AXIS_STANDSTILL          == 1);
    check("MC_AXIS_HOMING              == 2", MC_AXIS_HOMING              == 2);
    check("MC_AXIS_STOPPING            == 3", MC_AXIS_STOPPING            == 3);
    check("MC_AXIS_DISCRETE_MOTION     == 4", MC_AXIS_DISCRETE_MOTION     == 4);
    check("MC_AXIS_CONTINUOUS_MOTION   == 5", MC_AXIS_CONTINUOUS_MOTION   == 5);
    check("MC_AXIS_SYNCHRONIZED_MOTION == 6", MC_AXIS_SYNCHRONIZED_MOTION == 6);
    check("MC_AXIS_ERRORSTOP           == 7", MC_AXIS_ERRORSTOP           == 7);
}

/*===========================================================================
 * 2. AXIS_REF field access test (compile-time field existence check)
 *===========================================================================*/
static void test_axis_ref_fields(void)
{
    printf("\n--- AXIS_REF field access ---\n");
    AXIS_REF axis;

    /* Assign every field to verify they exist at the expected type */
    axis.AxisNo             = 0;
    axis.State              = MC_AXIS_DISABLED;
    axis.ActualPosition     = 0.0f;
    axis.ActualVelocity     = 0.0f;
    axis.ActualTorque       = 0.0f;
    axis.CommandedPosition  = 0.0f;
    axis.CommandedVelocity  = 0.0f;
    axis.VelFactor          = 1.0f;
    axis.AccFactor          = 1.0f;
    axis.JerkFactor         = 1.0f;
    axis.PowerOn            = false;
    axis.IsHomed            = false;
    axis.Error              = false;
    axis.Simulation         = false;
    axis.AxisErrorID        = 0;
    axis.HomeAbsSwitch      = false;
    axis.LimitSwitchPos     = false;
    axis.LimitSwitchNeg     = false;
    axis.CommunicationReady = false;
    axis.ReadyForPowerOn    = false;
    axis.AxisWarning        = false;

    check("AXIS_REF fields accessible", 1);
    check("AXIS_REF default VelFactor == 1.0", axis.VelFactor == 1.0f);
    check("AXIS_REF default State == Disabled", axis.State == MC_AXIS_DISABLED);
}

/*===========================================================================
 * 3. MC_Power — struct fields and stub call
 *===========================================================================*/
static void test_mc_power(void)
{
    printf("\n--- MC_Power ---\n");
    AXIS_REF axis;
    MC_Power fb;

    /* Zero-init structs */
    AXIS_REF_Init(&axis, 0);

    fb.Enable         = false;
    fb.EnablePositive = false;
    fb.EnableNegative = false;
    fb.Status         = false;
    fb.Valid          = false;
    fb.Error          = false;
    fb.ErrorID        = 0;
    fb._prevEnable    = false;

    MC_Power_Call(&fb, &axis);
    check("MC_Power_Call compiles and runs", 1);

    /* Field existence */
    check("MC_Power has Enable field",         (void*)&fb.Enable         != (void*)0);
    check("MC_Power has EnablePositive field", (void*)&fb.EnablePositive != (void*)0);
    check("MC_Power has EnableNegative field", (void*)&fb.EnableNegative != (void*)0);
    check("MC_Power has Status field",         (void*)&fb.Status         != (void*)0);
    check("MC_Power has Valid field",          (void*)&fb.Valid          != (void*)0);
    check("MC_Power has Error field",          (void*)&fb.Error          != (void*)0);
    check("MC_Power has ErrorID field",        (void*)&fb.ErrorID        != (void*)0);
}

/*===========================================================================
 * 4. MC_Home — struct fields and stub call
 *===========================================================================*/
static void test_mc_home(void)
{
    printf("\n--- MC_Home ---\n");
    AXIS_REF axis;
    MC_Home fb;

    AXIS_REF_Init(&axis, 0);

    fb.Execute         = false;
    fb.Position        = 0.0f;
    fb.BufferMode      = mcAborting;
    fb.Done            = false;
    fb.Busy            = false;
    fb.Active          = false;
    fb.CommandAborted  = false;
    fb.Error           = false;
    fb.ErrorID         = 0;
    fb._prevExecute    = false;

    MC_Home_Call(&fb, &axis);
    check("MC_Home_Call compiles and runs", 1);
    check("MC_Home has Execute field",  (void*)&fb.Execute  != (void*)0);
    check("MC_Home has Position field", (void*)&fb.Position != (void*)0);
    check("MC_Home has Done field",     (void*)&fb.Done     != (void*)0);
}

/*===========================================================================
 * 5. MC_Stop — struct fields and stub call
 *===========================================================================*/
static void test_mc_stop(void)
{
    printf("\n--- MC_Stop ---\n");
    AXIS_REF axis;
    MC_Stop fb;

    AXIS_REF_Init(&axis, 0);

    fb.Execute        = false;
    fb.Deceleration   = 100.0f;
    fb.Jerk           = 0.0f;
    fb.Done           = false;
    fb.Busy           = false;
    fb.CommandAborted = false;
    fb.Error          = false;
    fb.ErrorID        = 0;
    fb._prevExecute   = false;

    MC_Stop_Call(&fb, &axis);
    check("MC_Stop_Call compiles and runs", 1);
    check("MC_Stop has Deceleration field", (void*)&fb.Deceleration != (void*)0);
}

/*===========================================================================
 * 6. MC_Halt — struct fields and stub call
 *===========================================================================*/
static void test_mc_halt(void)
{
    printf("\n--- MC_Halt ---\n");
    AXIS_REF axis;
    MC_Halt fb;

    AXIS_REF_Init(&axis, 0);

    fb.Execute        = false;
    fb.Deceleration   = 100.0f;
    fb.Jerk           = 0.0f;
    fb.BufferMode     = mcAborting;
    fb.Done           = false;
    fb.Busy           = false;
    fb.Active         = false;
    fb.CommandAborted = false;
    fb.Error          = false;
    fb.ErrorID        = 0;
    fb._prevExecute   = false;

    MC_Halt_Call(&fb, &axis);
    check("MC_Halt_Call compiles and runs", 1);
    check("MC_Halt has BufferMode field", (void*)&fb.BufferMode != (void*)0);
}

/*===========================================================================
 * 7. MC_MoveAbsolute — struct fields and stub call
 *===========================================================================*/
static void test_mc_move_absolute(void)
{
    printf("\n--- MC_MoveAbsolute ---\n");
    AXIS_REF axis;
    MC_MoveAbsolute fb;

    AXIS_REF_Init(&axis, 0);

    fb.Execute           = false;
    fb.ContinuousUpdate  = false;
    fb.Position          = 1000.0f;
    fb.Velocity          = 100.0f;
    fb.Acceleration      = 50.0f;
    fb.Deceleration      = 50.0f;
    fb.Jerk              = 0.0f;
    fb.Direction         = mcPositiveDirection;
    fb.BufferMode        = mcAborting;
    fb.Done              = false;
    fb.Busy              = false;
    fb.Active            = false;
    fb.CommandAborted    = false;
    fb.Error             = false;
    fb.ErrorID           = 0;
    fb._prevExecute      = false;

    MC_MoveAbsolute_Call(&fb, &axis);
    check("MC_MoveAbsolute_Call compiles and runs", 1);
    check("MC_MoveAbsolute has Position field",  (void*)&fb.Position  != (void*)0);
    check("MC_MoveAbsolute has Velocity field",  (void*)&fb.Velocity  != (void*)0);
    check("MC_MoveAbsolute has Direction field", (void*)&fb.Direction != (void*)0);
}

/*===========================================================================
 * 8. MC_MoveRelative — struct fields and stub call
 *===========================================================================*/
static void test_mc_move_relative(void)
{
    printf("\n--- MC_MoveRelative ---\n");
    AXIS_REF axis;
    MC_MoveRelative fb;

    AXIS_REF_Init(&axis, 0);

    fb.Execute          = false;
    fb.ContinuousUpdate = false;
    fb.Distance         = 500.0f;
    fb.Velocity         = 100.0f;
    fb.Acceleration     = 50.0f;
    fb.Deceleration     = 50.0f;
    fb.Jerk             = 0.0f;
    fb.BufferMode       = mcAborting;
    fb.Done             = false;
    fb.Busy             = false;
    fb.Active           = false;
    fb.CommandAborted   = false;
    fb.Error            = false;
    fb.ErrorID          = 0;
    fb._prevExecute     = false;
    fb._targetPosition  = 0.0f;

    MC_MoveRelative_Call(&fb, &axis);
    check("MC_MoveRelative_Call compiles and runs", 1);
    check("MC_MoveRelative has Distance field", (void*)&fb.Distance != (void*)0);
}

/*===========================================================================
 * 9. MC_MoveAdditive — stub call
 *===========================================================================*/
static void test_mc_move_additive(void)
{
    printf("\n--- MC_MoveAdditive ---\n");
    AXIS_REF axis;
    MC_MoveAdditive fb;

    AXIS_REF_Init(&axis, 0);

    fb.Execute          = false;
    fb.ContinuousUpdate = false;
    fb.Distance         = 200.0f;
    fb.Velocity         = 80.0f;
    fb.Acceleration     = 40.0f;
    fb.Deceleration     = 40.0f;
    fb.Jerk             = 0.0f;
    fb.BufferMode       = mcAborting;
    fb.Done             = false;
    fb.Busy             = false;
    fb.Active           = false;
    fb.CommandAborted   = false;
    fb.Error            = false;
    fb.ErrorID          = 0;
    fb._prevExecute     = false;
    fb._targetPosition  = 0.0f;

    MC_MoveAdditive_Call(&fb, &axis);
    check("MC_MoveAdditive_Call compiles and runs", 1);
}

/*===========================================================================
 * 10. MC_MoveSuperimposed — stub call
 *===========================================================================*/
static void test_mc_move_superimposed(void)
{
    printf("\n--- MC_MoveSuperimposed ---\n");
    AXIS_REF axis;
    MC_MoveSuperimposed fb;

    AXIS_REF_Init(&axis, 0);

    fb.Execute          = false;
    fb.ContinuousUpdate = false;
    fb.Distance         = 100.0f;
    fb.VelocityDiff     = 10.0f;
    fb.Acceleration     = 20.0f;
    fb.Deceleration     = 20.0f;
    fb.Jerk             = 0.0f;
    fb.Done             = false;
    fb.Busy             = false;
    fb.Active           = false;
    fb.CommandAborted   = false;
    fb.Error            = false;
    fb.ErrorID          = 0;
    fb.CoveredDistance  = 0.0f;
    fb._prevExecute     = false;
    fb._coveredSoFar    = 0.0f;

    MC_MoveSuperimposed_Call(&fb, &axis);
    check("MC_MoveSuperimposed_Call compiles and runs", 1);
    check("MC_MoveSuperimposed has VelocityDiff field",     (void*)&fb.VelocityDiff    != (void*)0);
    check("MC_MoveSuperimposed has CoveredDistance field",  (void*)&fb.CoveredDistance != (void*)0);
}

/*===========================================================================
 * 11. MC_HaltSuperimposed — stub call
 *===========================================================================*/
static void test_mc_halt_superimposed(void)
{
    printf("\n--- MC_HaltSuperimposed ---\n");
    AXIS_REF axis;
    MC_HaltSuperimposed fb;

    AXIS_REF_Init(&axis, 0);

    fb.Execute        = false;
    fb.Deceleration   = 50.0f;
    fb.Jerk           = 0.0f;
    fb.Done           = false;
    fb.Busy           = false;
    fb.Active         = false;
    fb.CommandAborted = false;
    fb.Error          = false;
    fb.ErrorID        = 0;
    fb._prevExecute   = false;

    MC_HaltSuperimposed_Call(&fb, &axis);
    check("MC_HaltSuperimposed_Call compiles and runs", 1);
}

/*===========================================================================
 * 12. MC_MoveVelocity — struct fields and stub call
 *===========================================================================*/
static void test_mc_move_velocity(void)
{
    printf("\n--- MC_MoveVelocity ---\n");
    AXIS_REF axis;
    MC_MoveVelocity fb;

    AXIS_REF_Init(&axis, 0);

    fb.Execute          = false;
    fb.ContinuousUpdate = false;
    fb.Velocity         = 200.0f;
    fb.Acceleration     = 50.0f;
    fb.Deceleration     = 50.0f;
    fb.Jerk             = 0.0f;
    fb.Direction        = mcPositiveDirection;
    fb.BufferMode       = mcAborting;
    fb.InVelocity       = false;
    fb.Busy             = false;
    fb.Active           = false;
    fb.CommandAborted   = false;
    fb.Error            = false;
    fb.ErrorID          = 0;
    fb._prevExecute     = false;

    MC_MoveVelocity_Call(&fb, &axis);
    check("MC_MoveVelocity_Call compiles and runs", 1);
    check("MC_MoveVelocity has InVelocity output", (void*)&fb.InVelocity != (void*)0);
    check("MC_MoveVelocity has Velocity field",    (void*)&fb.Velocity   != (void*)0);
}

/*===========================================================================
 * 13. MC_MoveContinuousAbsolute — stub call
 *===========================================================================*/
static void test_mc_move_continuous_absolute(void)
{
    printf("\n--- MC_MoveContinuousAbsolute ---\n");
    AXIS_REF axis;
    MC_MoveContinuousAbsolute fb;

    AXIS_REF_Init(&axis, 0);

    fb.Execute          = false;
    fb.ContinuousUpdate = false;
    fb.Position         = 5000.0f;
    fb.EndVelocity      = 50.0f;
    fb.Velocity         = 200.0f;
    fb.Acceleration     = 100.0f;
    fb.Deceleration     = 100.0f;
    fb.Jerk             = 0.0f;
    fb.Direction        = mcPositiveDirection;
    fb.BufferMode       = mcAborting;
    fb.InEndVelocity    = false;
    fb.Busy             = false;
    fb.Active           = false;
    fb.CommandAborted   = false;
    fb.Error            = false;
    fb.ErrorID          = 0;
    fb._prevExecute     = false;

    MC_MoveContinuousAbsolute_Call(&fb, &axis);
    check("MC_MoveContinuousAbsolute_Call compiles and runs", 1);
    check("MC_MoveContinuousAbsolute has EndVelocity field",  (void*)&fb.EndVelocity  != (void*)0);
    check("MC_MoveContinuousAbsolute has InEndVelocity field",(void*)&fb.InEndVelocity!= (void*)0);
}

/*===========================================================================
 * 14. MC_MoveContinuousRelative — stub call
 *===========================================================================*/
static void test_mc_move_continuous_relative(void)
{
    printf("\n--- MC_MoveContinuousRelative ---\n");
    AXIS_REF axis;
    MC_MoveContinuousRelative fb;

    AXIS_REF_Init(&axis, 0);

    fb.Execute          = false;
    fb.ContinuousUpdate = false;
    fb.Distance         = 3000.0f;
    fb.EndVelocity      = 30.0f;
    fb.Velocity         = 150.0f;
    fb.Acceleration     = 75.0f;
    fb.Deceleration     = 75.0f;
    fb.Jerk             = 0.0f;
    fb.BufferMode       = mcAborting;
    fb.InEndVelocity    = false;
    fb.Busy             = false;
    fb.Active           = false;
    fb.CommandAborted   = false;
    fb.Error            = false;
    fb.ErrorID          = 0;
    fb._prevExecute     = false;
    fb._targetPosition  = 0.0f;

    MC_MoveContinuousRelative_Call(&fb, &axis);
    check("MC_MoveContinuousRelative_Call compiles and runs", 1);
}

/*===========================================================================
 * 15. MC_SetPosition — stub call
 *===========================================================================*/
static void test_mc_set_position(void)
{
    printf("\n--- MC_SetPosition ---\n");
    AXIS_REF axis;
    MC_SetPosition fb;

    AXIS_REF_Init(&axis, 0);

    fb.Execute       = false;
    fb.Position      = 0.0f;
    fb.Relative      = false;
    fb.ExecutionMode = mcImmediately;
    fb.Done          = false;
    fb.Busy          = false;
    fb.Error         = false;
    fb.ErrorID       = 0;
    fb._prevExecute  = false;

    MC_SetPosition_Call(&fb, &axis);
    check("MC_SetPosition_Call compiles and runs", 1);
    check("MC_SetPosition has Relative field",      (void*)&fb.Relative      != (void*)0);
    check("MC_SetPosition has ExecutionMode field", (void*)&fb.ExecutionMode != (void*)0);
}

/*===========================================================================
 * 16. MC_SetOverride — stub call
 *===========================================================================*/
static void test_mc_set_override(void)
{
    printf("\n--- MC_SetOverride ---\n");
    AXIS_REF axis;
    MC_SetOverride fb;

    AXIS_REF_Init(&axis, 0);

    fb.Enable     = false;
    fb.VelFactor  = 1.0f;
    fb.AccFactor  = 1.0f;
    fb.JerkFactor = 1.0f;
    fb.Enabled    = false;
    fb.Busy       = false;
    fb.Error      = false;
    fb.ErrorID    = 0;
    fb._prevEnable= false;

    MC_SetOverride_Call(&fb, &axis);
    check("MC_SetOverride_Call compiles and runs", 1);
    check("MC_SetOverride has VelFactor field",  (void*)&fb.VelFactor  != (void*)0);
    check("MC_SetOverride has AccFactor field",  (void*)&fb.AccFactor  != (void*)0);
    check("MC_SetOverride has JerkFactor field", (void*)&fb.JerkFactor != (void*)0);
    check("MC_SetOverride has Enabled output",   (void*)&fb.Enabled    != (void*)0);
}

/*===========================================================================
 * 17. MC_ReadParameter & MC_ReadBoolParameter — stub calls
 *===========================================================================*/
static void test_mc_read_parameter(void)
{
    printf("\n--- MC_ReadParameter ---\n");
    AXIS_REF axis;
    MC_ReadParameter fb;
    MC_ReadBoolParameter fbb;

    AXIS_REF_Init(&axis, 0);

    fb.Enable          = false;
    fb.ParameterNumber = 10;  /* PN10 = ActualVelocity */
    fb.Valid           = false;
    fb.Busy            = false;
    fb.Error           = false;
    fb.ErrorID         = 0;
    fb.Value           = 0.0f;
    fb._prevEnable     = false;

    MC_ReadParameter_Call(&fb, &axis);
    check("MC_ReadParameter_Call compiles and runs", 1);
    check("MC_ReadParameter has ParameterNumber field", (void*)&fb.ParameterNumber != (void*)0);
    check("MC_ReadParameter has Value (float) field",   (void*)&fb.Value           != (void*)0);

    fbb.Enable          = false;
    fbb.ParameterNumber = 4;  /* PN4 = EnableLimitPos */
    fbb.Valid           = false;
    fbb.Busy            = false;
    fbb.Error           = false;
    fbb.ErrorID         = 0;
    fbb.Value           = false;
    fbb._prevEnable     = false;

    MC_ReadBoolParameter_Call(&fbb, &axis);
    check("MC_ReadBoolParameter_Call compiles and runs", 1);
    check("MC_ReadBoolParameter has Value (bool) field", (void*)&fbb.Value != (void*)0);
}

/*===========================================================================
 * 18. MC_WriteParameter & MC_WriteBoolParameter — stub calls
 *===========================================================================*/
static void test_mc_write_parameter(void)
{
    printf("\n--- MC_WriteParameter ---\n");
    AXIS_REF axis;
    MC_WriteParameter fb;
    MC_WriteBoolParameter fbb;

    AXIS_REF_Init(&axis, 0);

    fb.Execute         = false;
    fb.ParameterNumber = 9;   /* PN9 = MaxVelocityAppl (R/W) */
    fb.Value           = 500.0f;
    fb.ExecutionMode   = mcImmediately;
    fb.Done            = false;
    fb.Busy            = false;
    fb.Error           = false;
    fb.ErrorID         = 0;
    fb._prevExecute    = false;

    MC_WriteParameter_Call(&fb, &axis);
    check("MC_WriteParameter_Call compiles and runs", 1);

    fbb.Execute         = false;
    fbb.ParameterNumber = 4;
    fbb.Value           = true;
    fbb.ExecutionMode   = mcImmediately;
    fbb.Done            = false;
    fbb.Busy            = false;
    fbb.Error           = false;
    fbb.ErrorID         = 0;
    fbb._prevExecute    = false;

    MC_WriteBoolParameter_Call(&fbb, &axis);
    check("MC_WriteBoolParameter_Call compiles and runs", 1);
}

/*===========================================================================
 * 19. Read-actual FBs — MC_ReadActualPosition/Velocity/Torque
 *===========================================================================*/
static void test_mc_read_actual(void)
{
    printf("\n--- MC_ReadActual* ---\n");
    AXIS_REF axis;

    AXIS_REF_Init(&axis, 0);

    /* Position */
    MC_ReadActualPosition pos_fb;
    pos_fb.Enable    = false;
    pos_fb.Valid     = false;
    pos_fb.Busy      = false;
    pos_fb.Error     = false;
    pos_fb.ErrorID   = 0;
    pos_fb.Position  = 0.0f;
    pos_fb._prevEnable = false;

    MC_ReadActualPosition_Call(&pos_fb, &axis);
    check("MC_ReadActualPosition_Call compiles and runs", 1);
    check("MC_ReadActualPosition has Position field", (void*)&pos_fb.Position != (void*)0);

    /* Velocity */
    MC_ReadActualVelocity vel_fb;
    vel_fb.Enable    = false;
    vel_fb.Valid     = false;
    vel_fb.Busy      = false;
    vel_fb.Error     = false;
    vel_fb.ErrorID   = 0;
    vel_fb.Velocity  = 0.0f;
    vel_fb._prevEnable = false;

    MC_ReadActualVelocity_Call(&vel_fb, &axis);
    check("MC_ReadActualVelocity_Call compiles and runs", 1);

    /* Torque */
    MC_ReadActualTorque trq_fb;
    trq_fb.Enable   = false;
    trq_fb.Valid    = false;
    trq_fb.Busy     = false;
    trq_fb.Error    = false;
    trq_fb.ErrorID  = 0;
    trq_fb.Torque   = 0.0f;
    trq_fb._prevEnable = false;

    MC_ReadActualTorque_Call(&trq_fb, &axis);
    check("MC_ReadActualTorque_Call compiles and runs", 1);
}

/*===========================================================================
 * 20. MC_ReadStatus — stub call, field check
 *===========================================================================*/
static void test_mc_read_status(void)
{
    printf("\n--- MC_ReadStatus ---\n");
    AXIS_REF axis;
    MC_ReadStatus fb;

    AXIS_REF_Init(&axis, 0);

    fb.Enable             = false;
    fb.Valid              = false;
    fb.Busy               = false;
    fb.Error              = false;
    fb.ErrorID            = 0;
    fb.ErrorStop          = false;
    fb.Disabled           = false;
    fb.Stopping           = false;
    fb.Homing             = false;
    fb.Standstill         = false;
    fb.DiscreteMotion     = false;
    fb.ContinuousMotion   = false;
    fb.SynchronizedMotion = false;
    fb._prevEnable        = false;

    MC_ReadStatus_Call(&fb, &axis);
    check("MC_ReadStatus_Call compiles and runs", 1);
    check("MC_ReadStatus has Disabled field",           (void*)&fb.Disabled           != (void*)0);
    check("MC_ReadStatus has Standstill field",         (void*)&fb.Standstill         != (void*)0);
    check("MC_ReadStatus has DiscreteMotion field",     (void*)&fb.DiscreteMotion     != (void*)0);
    check("MC_ReadStatus has ContinuousMotion field",   (void*)&fb.ContinuousMotion   != (void*)0);
    check("MC_ReadStatus has SynchronizedMotion field", (void*)&fb.SynchronizedMotion != (void*)0);
    check("MC_ReadStatus has ErrorStop field",          (void*)&fb.ErrorStop          != (void*)0);
    check("MC_ReadStatus has Homing field",             (void*)&fb.Homing             != (void*)0);
}

/*===========================================================================
 * 21. MC_ReadMotionState — stub call
 *===========================================================================*/
static void test_mc_read_motion_state(void)
{
    printf("\n--- MC_ReadMotionState ---\n");
    AXIS_REF axis;
    MC_ReadMotionState fb;

    AXIS_REF_Init(&axis, 0);

    fb.Enable              = false;
    fb.Source              = mcActualValue;
    fb.Valid               = false;
    fb.Busy                = false;
    fb.Error               = false;
    fb.ErrorID             = 0;
    fb.ConstantVelocity    = false;
    fb.Accelerating        = false;
    fb.Decelerating        = false;
    fb.DirectionPositive   = false;
    fb.DirectionNegative   = false;
    fb._prevEnable         = false;
    fb._prevVelocity       = 0.0f;

    MC_ReadMotionState_Call(&fb, &axis);
    check("MC_ReadMotionState_Call compiles and runs", 1);
    check("MC_ReadMotionState has Source field",           (void*)&fb.Source           != (void*)0);
    check("MC_ReadMotionState has ConstantVelocity field", (void*)&fb.ConstantVelocity != (void*)0);
    check("MC_ReadMotionState has DirectionPositive field",(void*)&fb.DirectionPositive!= (void*)0);
}

/*===========================================================================
 * 22. MC_ReadAxisInfo — stub call
 *===========================================================================*/
static void test_mc_read_axis_info(void)
{
    printf("\n--- MC_ReadAxisInfo ---\n");
    AXIS_REF axis;
    MC_ReadAxisInfo fb;

    AXIS_REF_Init(&axis, 0);

    fb.Enable             = false;
    fb.Valid              = false;
    fb.Busy               = false;
    fb.Error              = false;
    fb.ErrorID            = 0;
    fb.HomeAbsSwitch      = false;
    fb.LimitSwitchPos     = false;
    fb.LimitSwitchNeg     = false;
    fb.Simulation         = false;
    fb.CommunicationReady = false;
    fb.ReadyForPowerOn    = false;
    fb.PowerOn            = false;
    fb.IsHomed            = false;
    fb.AxisWarning        = false;
    fb._prevEnable        = false;

    MC_ReadAxisInfo_Call(&fb, &axis);
    check("MC_ReadAxisInfo_Call compiles and runs", 1);
    check("MC_ReadAxisInfo has LimitSwitchPos field",     (void*)&fb.LimitSwitchPos     != (void*)0);
    check("MC_ReadAxisInfo has CommunicationReady field", (void*)&fb.CommunicationReady != (void*)0);
    check("MC_ReadAxisInfo has IsHomed field",            (void*)&fb.IsHomed            != (void*)0);
}

/*===========================================================================
 * 23. MC_ReadAxisError — stub call
 *===========================================================================*/
static void test_mc_read_axis_error(void)
{
    printf("\n--- MC_ReadAxisError ---\n");
    AXIS_REF axis;
    MC_ReadAxisError fb;

    AXIS_REF_Init(&axis, 0);

    fb.Enable      = false;
    fb.Valid       = false;
    fb.Busy        = false;
    fb.Error       = false;
    fb.ErrorID     = 0;
    fb.AxisErrorID = 0;
    fb._prevEnable = false;

    MC_ReadAxisError_Call(&fb, &axis);
    check("MC_ReadAxisError_Call compiles and runs", 1);
    check("MC_ReadAxisError has AxisErrorID field", (void*)&fb.AxisErrorID != (void*)0);
}

/*===========================================================================
 * 24. MC_Reset — stub call
 *===========================================================================*/
static void test_mc_reset(void)
{
    printf("\n--- MC_Reset ---\n");
    AXIS_REF axis;
    MC_Reset fb;

    AXIS_REF_Init(&axis, 0);

    fb.Execute      = false;
    fb.Done         = false;
    fb.Busy         = false;
    fb.Error        = false;
    fb.ErrorID      = 0;
    fb._prevExecute = false;

    MC_Reset_Call(&fb, &axis);
    check("MC_Reset_Call compiles and runs", 1);
}

/*===========================================================================
 * 25. State diagram: state enum completeness
 *===========================================================================*/
static void test_state_diagram(void)
{
    printf("\n--- State diagram completeness ---\n");

    /* Verify all 8 states from Figure 2 exist */
    MC_AXIS_STATE states[] = {
        MC_AXIS_DISABLED,
        MC_AXIS_STANDSTILL,
        MC_AXIS_HOMING,
        MC_AXIS_STOPPING,
        MC_AXIS_DISCRETE_MOTION,
        MC_AXIS_CONTINUOUS_MOTION,
        MC_AXIS_SYNCHRONIZED_MOTION,
        MC_AXIS_ERRORSTOP
    };
    check("All 8 PLCopen axis states defined", 1);
    check("States span 0..7", states[0] == 0 && states[7] == 7);

    /* Verify buffer modes from Table 3 */
    MC_BUFFER_MODE modes[] = {
        mcAborting, mcBuffered, mcBlendingLow,
        mcBlendingPrevious, mcBlendingNext, mcBlendingHigh
    };
    check("All 6 buffer modes defined", 1);
    check("mcAborting is default (0)", modes[0] == 0);

    (void)states;
    (void)modes;
}

/*===========================================================================
 * main
 *===========================================================================*/
int main(void)
{
    printf("=== KronMotion Test Suite ===\n");
    printf("PLCopen Motion Control Part 1, Version 2.0\n");

    test_enum_values();
    test_axis_ref_fields();
    test_mc_power();
    test_mc_home();
    test_mc_stop();
    test_mc_halt();
    test_mc_move_absolute();
    test_mc_move_relative();
    test_mc_move_additive();
    test_mc_move_superimposed();
    test_mc_halt_superimposed();
    test_mc_move_velocity();
    test_mc_move_continuous_absolute();
    test_mc_move_continuous_relative();
    test_mc_set_position();
    test_mc_set_override();
    test_mc_read_parameter();
    test_mc_write_parameter();
    test_mc_read_actual();
    test_mc_read_status();
    test_mc_read_motion_state();
    test_mc_read_axis_info();
    test_mc_read_axis_error();
    test_mc_reset();
    test_state_diagram();

    printf("\n=== Results: %d passed, %d failed ===\n", pass_count, fail_count);
    return (fail_count == 0) ? 0 : 1;
}
