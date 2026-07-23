/**
 * @file ambar_actuator.c
 * @brief Final software-geometry motion authority for the AMBAR airbrake.
 *
 * The existing PCB has no HOME/FULL inputs. HOME is declared by the operator
 * only while the mechanism is manually closed and motor energy is off. That
 * command writes XACTUAL/XTARGET to zero without moving. FULL is the configured
 * three-rotation software target. XACTUAL is TMC5240 ramp-generator state, not
 * encoder feedback or independent proof of physical position.
 */

#include "ambar_actuator.h"

#include "main.h"
#include "rocket_protocol.h"
#include "tmc5240.h"

#include <limits.h>
#include <stddef.h>

#define AMBAR_ACTUATOR_POSITION_TOLERANCE_STEPS          100L
#define AMBAR_ACTUATOR_ENDPOINT_POSITION_TOLERANCE_STEPS \
    AMBAR_ACTUATOR_POSITION_TOLERANCE_STEPS
#define AMBAR_ACTUATOR_TRAVEL_TIMEOUT_MS                 8000UL
#define AMBAR_ACTUATOR_STALL_PROGRESS_STEPS              100L
#define AMBAR_ACTUATOR_STALL_TIMEOUT_MS                  2500UL

static void ambar_latch_fault(AmbarActuatorState_t *state,
                              uint32_t inhibit_flag,
                              bool driver_fault);

/* -------------------- Numeric and state helpers -------------------- */

static float ambar_clamp_fraction(float value)
{
    if (!(value > 0.0f))
    {
        return 0.0f;
    }
    if (value > 1.0f)
    {
        return 1.0f;
    }
    return value;
}

static int32_t ambar_saturate_i64(int64_t value)
{
    if (value > INT32_MAX)
    {
        return INT32_MAX;
    }
    if (value < INT32_MIN)
    {
        return INT32_MIN;
    }
    return (int32_t)value;
}

static uint32_t ambar_step_distance(int32_t first, int32_t second)
{
    int64_t difference = (int64_t)first - (int64_t)second;
    if (difference < 0)
    {
        difference = -difference;
    }
    return difference > UINT32_MAX ? UINT32_MAX : (uint32_t)difference;
}

static int32_t ambar_full_position(const AmbarActuatorState_t *state)
{
    int64_t position;

    if (state == NULL)
    {
        return 0;
    }
    position = (int64_t)state->home_position_steps
        + (state->direction_inverted != 0U
            ? -(int64_t)state->max_extension_steps
            : (int64_t)state->max_extension_steps);
    return ambar_saturate_i64(position);
}

static void ambar_travel_range(const AmbarActuatorState_t *state,
                               int32_t *lower,
                               int32_t *upper)
{
    const int32_t full = ambar_full_position(state);

    if (state == NULL || lower == NULL || upper == NULL)
    {
        return;
    }
    if (full < state->home_position_steps)
    {
        *lower = full;
        *upper = state->home_position_steps;
    }
    else
    {
        *lower = state->home_position_steps;
        *upper = full;
    }
}

static void ambar_clear_automatic_motion(AmbarActuatorState_t *state)
{
    if (state == NULL)
    {
        return;
    }
    state->automatic_motion_active = 0U;
    state->automatic_deployment_latched = 0U;
    state->automatic_retract_active = 0U;
    state->automatic_retract_started_ms = 0U;
}

static void ambar_begin_motion_watchdog(AmbarActuatorState_t *state)
{
    const uint32_t now = HAL_GetTick();

    state->motion_started_ms = now;
    state->motion_origin_position_steps = state->actual_position_steps;
    state->last_progress_position_steps = state->actual_position_steps;
    state->last_progress_ms = now;
}

static bool ambar_motion_watchdog_ok(AmbarActuatorState_t *state)
{
    const uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - state->motion_started_ms)
        > AMBAR_ACTUATOR_TRAVEL_TIMEOUT_MS)
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
        return false;
    }

    if (ambar_step_distance(state->actual_position_steps,
                            state->last_progress_position_steps)
        >= (uint32_t)AMBAR_ACTUATOR_STALL_PROGRESS_STEPS)
    {
        state->last_progress_position_steps = state->actual_position_steps;
        state->last_progress_ms = now;
    }
    else if ((uint32_t)(now - state->last_progress_ms)
             > AMBAR_ACTUATOR_STALL_TIMEOUT_MS)
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
        return false;
    }
    return true;
}

/* -------------------- Direct hardware gates -------------------- */

static HAL_StatusTypeDef ambar_write_target(AmbarActuatorState_t *state,
                                             int32_t target_steps)
{
    if (state == NULL)
    {
        return HAL_ERROR;
    }
    if (state->command_valid != 0U
        && state->commanded_position_steps == target_steps)
    {
        return HAL_OK;
    }
    if (TMC5240_SetTargetPosition(target_steps) != HAL_OK)
    {
        return HAL_ERROR;
    }
    state->commanded_position_steps = target_steps;
    state->command_valid = 1U;
    return HAL_OK;
}

static bool ambar_set_driver_enabled(AmbarActuatorState_t *state, uint8_t enabled)
{
    uint32_t interrupt_mask;
    bool enable_allowed = false;

    if (enabled == 0U)
    {
        TMC5240_SetDriverEnabled(0U);
        if (state != NULL)
        {
            state->driver_enabled = false;
        }
        return false;
    }
    if (state == NULL)
    {
        TMC5240_SetDriverEnabled(0U);
        return false;
    }

    interrupt_mask = __get_PRIMASK();
    __disable_irq();
#if AMBAR_FEATURE_ACTUATOR
    enable_allowed = state->motor_driver_ok
        && state->config_valid
        && state->homed
        && !state->estop_latched
        && !state->fault_latched
        && state->limit_state_initialized != 0U
        && state->limits_plausible != 0U
        && state->limit_irq_pending == 0U;
#endif
    TMC5240_SetDriverEnabled(enable_allowed ? 1U : 0U);
    state->driver_enabled = enable_allowed;
    __DMB();
    __set_PRIMASK(interrupt_mask);
    return enable_allowed;
}

static uint32_t ambar_compile_time_inhibit_flags(void)
{
    uint32_t flags = AMBAR_ACTUATOR_INHIBIT_NONE;
#if AMBAR_FEATURE_ACTUATOR == 0
    flags |= AMBAR_ACTUATOR_INHIBIT_BUILD_FLAG;
#endif
    return flags;
}

static uint32_t ambar_base_hardware_gates(const AmbarActuatorState_t *state,
                                          bool require_home)
{
    uint32_t flags = ambar_compile_time_inhibit_flags();

    if (state == NULL || !state->motor_driver_ok)
    {
        flags |= AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT;
    }
    if (state == NULL || !state->config_valid)
    {
        flags |= AMBAR_ACTUATOR_INHIBIT_CONFIG_INVALID;
    }
    if (state == NULL || state->estop_latched)
    {
        flags |= AMBAR_ACTUATOR_INHIBIT_ESTOP;
    }
    if (state == NULL || state->fault_latched)
    {
        flags |= AMBAR_ACTUATOR_INHIBIT_DIAG_FAULT;
    }
    if (require_home && (state == NULL || !state->homed))
    {
        flags |= AMBAR_ACTUATOR_INHIBIT_NOT_HOMED;
        flags |= AMBAR_ACTUATOR_INHIBIT_SOFTWARE_HOME_NOT_READY;
    }
    if (require_home
        && (state == NULL || state->limit_state_initialized == 0U))
    {
        flags |= AMBAR_ACTUATOR_INHIBIT_SOFTWARE_HOME_NOT_READY;
    }
    if (require_home && state != NULL && state->homed
        && state->limits_plausible == 0U)
    {
        flags |= AMBAR_ACTUATOR_INHIBIT_GEOMETRY_FAULT;
    }
    return flags;
}

static uint32_t ambar_common_motion_gates(const AmbarActuatorState_t *state)
{
    return ambar_base_hardware_gates(state, true);
}

static uint32_t ambar_bench_motion_gates(const AmbarActuatorState_t *state)
{
    uint32_t flags = ambar_common_motion_gates(state);
#if AMBAR_FEATURE_EFFECTIVE_BENCH_COMMANDS == 0
    flags |= AMBAR_ACTUATOR_INHIBIT_BENCH_FLAG;
#endif
    return flags;
}

static void ambar_latch_fault(AmbarActuatorState_t *state,
                              uint32_t inhibit_flag,
                              bool driver_fault)
{
    if (state != NULL)
    {
        if (driver_fault)
        {
            state->motor_driver_ok = false;
        }
        state->fault_latched = true;
        state->homing_active = 0U;
        state->manual_request_pending = 0U;
        state->hil_override_mode = AMBAR_ACTUATOR_HIL_OVERRIDE_OFF;
        state->hil_endpoint_reached = 0U;
        ambar_clear_automatic_motion(state);
        state->command_valid = 0U;
        state->machine_state = AMBAR_ACTUATOR_STATE_FAULT;
        state->last_inhibit_flags |= inhibit_flag;
    }
    ambar_set_driver_enabled(state, 0U);
}

/* -------------------- Software geometry and stroke sequence -------------------- */

static bool ambar_position_in_software_range(
    const AmbarActuatorState_t *state)
{
    int32_t lower = 0;
    int32_t upper = 0;
    int64_t padded_lower;
    int64_t padded_upper;

    if (state == NULL || !state->config_valid || !state->homed)
    {
        return false;
    }
    ambar_travel_range(state, &lower, &upper);
    padded_lower = (int64_t)lower
        - AMBAR_ACTUATOR_ENDPOINT_POSITION_TOLERANCE_STEPS;
    padded_upper = (int64_t)upper
        + AMBAR_ACTUATOR_ENDPOINT_POSITION_TOLERANCE_STEPS;
    return (int64_t)state->actual_position_steps >= padded_lower
        && (int64_t)state->actual_position_steps <= padded_upper;
}

static void ambar_update_software_geometry(AmbarActuatorState_t *state)
{
    uint8_t at_home;
    uint8_t at_full;
    AmbarActuatorStrokeSequence_t sequence;

    if (state == NULL)
    {
        return;
    }

    state->limit_irq_pending = 0U;
    if (!state->config_valid || !state->homed)
    {
        state->home_limit_active = 0U;
        state->full_limit_active = 0U;
        state->home_limit_raw = 0U;
        state->full_limit_raw = 0U;
        state->limit_sample_started = 0U;
        state->limit_state_initialized = 0U;
        state->limits_plausible = 0U;
        return;
    }

    at_home = ambar_step_distance(state->actual_position_steps,
                                  state->home_position_steps)
            <= (uint32_t)AMBAR_ACTUATOR_ENDPOINT_POSITION_TOLERANCE_STEPS
        ? 1U : 0U;
    at_full = ambar_step_distance(state->actual_position_steps,
                                  ambar_full_position(state))
            <= (uint32_t)AMBAR_ACTUATOR_ENDPOINT_POSITION_TOLERANCE_STEPS
        ? 1U : 0U;

    state->home_limit_active = at_home;
    state->full_limit_active = at_full;
    state->home_limit_raw = at_home;
    state->full_limit_raw = at_full;
    state->limit_sample_started = 1U;
    state->limit_state_initialized = 1U;
    state->limits_plausible =
        ambar_position_in_software_range(state)
        && !(at_home != 0U && at_full != 0U);

    sequence =
        (AmbarActuatorStrokeSequence_t)state->endpoint_sequence_state;
    if (at_home != 0U)
    {
        if (sequence == AMBAR_ACTUATOR_STROKE_SEQUENCE_LEFT_FULL)
        {
            state->endpoint_sequence_state =
                AMBAR_ACTUATOR_STROKE_SEQUENCE_COMPLETE;
            state->endpoint_sequence_verified = 1U;
        }
        else if (sequence == AMBAR_ACTUATOR_STROKE_SEQUENCE_UNKNOWN)
        {
            state->endpoint_sequence_state =
                AMBAR_ACTUATOR_STROKE_SEQUENCE_AT_HOME;
        }
    }
    else if (sequence == AMBAR_ACTUATOR_STROKE_SEQUENCE_AT_HOME
             || sequence == AMBAR_ACTUATOR_STROKE_SEQUENCE_COMPLETE)
    {
        state->endpoint_sequence_state =
            AMBAR_ACTUATOR_STROKE_SEQUENCE_LEFT_HOME;
        state->endpoint_sequence_verified = 0U;
    }

    sequence =
        (AmbarActuatorStrokeSequence_t)state->endpoint_sequence_state;
    if (at_full != 0U
        && sequence == AMBAR_ACTUATOR_STROKE_SEQUENCE_LEFT_HOME)
    {
        state->endpoint_sequence_state =
            AMBAR_ACTUATOR_STROKE_SEQUENCE_AT_FULL;
    }
    else if (at_full == 0U
             && sequence == AMBAR_ACTUATOR_STROKE_SEQUENCE_AT_FULL)
    {
        state->endpoint_sequence_state =
            AMBAR_ACTUATOR_STROKE_SEQUENCE_LEFT_FULL;
    }

    if (state->limits_plausible == 0U && !state->fault_latched)
    {
        ambar_latch_fault(state,
                          AMBAR_ACTUATOR_INHIBIT_GEOMETRY_FAULT,
                          false);
    }
}

static void ambar_update_driver_snapshot(AmbarActuatorState_t *state)
{
    state->diag0_level =
        HAL_GPIO_ReadPin(MOTOR_DIAG0_GPIO_Port, MOTOR_DIAG0_Pin) == GPIO_PIN_SET
        ? 1U : 0U;
    state->diag1_level =
        HAL_GPIO_ReadPin(MOTOR_DIAG1_GPIO_Port, MOTOR_DIAG1_Pin) == GPIO_PIN_SET
        ? 1U : 0U;

    if (state->motor_driver_ok
        && (TMC5240_ReadActualPosition(&state->actual_position_steps) != HAL_OK
            || TMC5240_ReadDriverStatus(&state->last_driver_status) != HAL_OK
            || TMC5240_DriverStatusHasHardFault(state->last_driver_status) != 0U))
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
    }
}

/* -------------------- Completion helpers -------------------- */

static bool ambar_restore_normal_motion_limits(AmbarActuatorState_t *state)
{
    if (state->normal_motion_limits_applied != 0U)
    {
        return true;
    }
    if (TMC5240_SetMotionLimits(state->max_velocity_steps_per_s,
                                state->max_accel_steps_per_s2) != HAL_OK)
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
        return false;
    }
    state->normal_motion_limits_applied = 1U;
    return true;
}

/*
 * Operator-only software HOME declaration. The caller has already established
 * that the mechanism is manually closed, unarmed, idle, and energy-off.
 * This never commands motion: DRV_ENN is asserted before XACTUAL/XTARGET=HOME.
 */
static bool ambar_declare_software_home(AmbarActuatorState_t *state)
{
    ambar_set_driver_enabled(state, 0U);
    if (TMC5240_Stop() != HAL_OK
        || TMC5240_SetActualPosition(state->home_position_steps) != HAL_OK
        || TMC5240_SetTargetPosition(state->home_position_steps) != HAL_OK)
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
        return false;
    }
    state->actual_position_steps = state->home_position_steps;
    state->requested_position_steps = state->home_position_steps;
    state->commanded_position_steps = state->home_position_steps;
    state->command_valid = 1U;
    state->homed = true;
    state->homing_active = 0U;
    state->manual_request_pending = 0U;
    state->hil_override_mode = AMBAR_ACTUATOR_HIL_OVERRIDE_OFF;
    state->hil_endpoint_reached = 0U;
    ambar_clear_automatic_motion(state);
    state->endpoint_sequence_verified = 0U;
    state->endpoint_sequence_state =
        AMBAR_ACTUATOR_STROKE_SEQUENCE_AT_HOME;
    state->machine_state = AMBAR_ACTUATOR_STATE_READY;
    ambar_update_software_geometry(state);
    return !state->fault_latched
        && ambar_restore_normal_motion_limits(state);
}

/*
 * Powered travel back to HOME completes from XACTUAL tolerance only. It does
 * not rewrite XACTUAL, because doing so would hide the ramp-generator evidence
 * collected during the stroke.
 */
static bool ambar_complete_return_home(AmbarActuatorState_t *state)
{
    if (ambar_step_distance(state->actual_position_steps,
                            state->home_position_steps)
        > (uint32_t)AMBAR_ACTUATOR_ENDPOINT_POSITION_TOLERANCE_STEPS)
    {
        ambar_latch_fault(state,
                          AMBAR_ACTUATOR_INHIBIT_GEOMETRY_FAULT,
                          false);
        return false;
    }

    ambar_set_driver_enabled(state, 0U);
    if (TMC5240_Stop() != HAL_OK
        || TMC5240_SetTargetPosition(state->home_position_steps) != HAL_OK)
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
        return false;
    }
    state->requested_position_steps = state->home_position_steps;
    state->commanded_position_steps = state->home_position_steps;
    state->command_valid = 1U;
    state->homing_active = 0U;
    state->manual_request_pending = 0U;
    state->automatic_motion_active = 0U;
    state->automatic_retract_active = 0U;
    state->automatic_retract_started_ms = 0U;
    ambar_update_software_geometry(state);

    if (state->hil_override_mode == AMBAR_ACTUATOR_HIL_OVERRIDE_FORCE_HOME)
    {
        state->hil_endpoint_reached = 1U;
        state->machine_state = AMBAR_ACTUATOR_STATE_HIL_OVERRIDE;
    }
    else
    {
        state->automatic_deployment_latched = 0U;
        state->machine_state = AMBAR_ACTUATOR_STATE_READY;
    }
    return !state->fault_latched
        && ambar_restore_normal_motion_limits(state);
}

static bool ambar_complete_nonendpoint_target(AmbarActuatorState_t *state)
{
    ambar_set_driver_enabled(state, 0U);
    if (TMC5240_Stop() != HAL_OK)
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
        return false;
    }
    state->manual_request_pending = 0U;
    state->automatic_motion_active = 0U;
    state->machine_state = AMBAR_ACTUATOR_STATE_READY;
    return true;
}

static bool ambar_complete_full(AmbarActuatorState_t *state, bool hil)
{
    if (ambar_step_distance(state->actual_position_steps,
                            ambar_full_position(state))
        > (uint32_t)AMBAR_ACTUATOR_ENDPOINT_POSITION_TOLERANCE_STEPS)
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_LIMIT_FAULT, false);
        return false;
    }
    ambar_set_driver_enabled(state, 0U);
    if (TMC5240_Stop() != HAL_OK)
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
        return false;
    }
    state->requested_position_steps = ambar_full_position(state);
    state->commanded_position_steps = state->requested_position_steps;
    state->command_valid = 1U;
    state->manual_request_pending = 0U;
    state->automatic_motion_active = 0U;
    ambar_update_software_geometry(state);
    if (state->full_limit_active == 0U)
    {
        ambar_latch_fault(state,
                          AMBAR_ACTUATOR_INHIBIT_GEOMETRY_FAULT,
                          false);
        return false;
    }
    if (hil)
    {
        state->hil_endpoint_reached = 1U;
        state->machine_state = AMBAR_ACTUATOR_STATE_HIL_OVERRIDE;
    }
    else
    {
        state->automatic_deployment_latched = 1U;
        state->machine_state = AMBAR_ACTUATOR_STATE_READY;
    }
    return true;
}

/* -------------------- Configuration and pure flight mapping -------------------- */

AmbarActuatorConfig_t AmbarActuator_DefaultConfig(void)
{
    AmbarActuatorConfig_t config;

#if AMBAR_ACTUATOR_USE_PROTOTYPE_PROFILE
    config.home_position_steps = 0;
    config.max_extension_steps = 200 * 256 * 3; /* exactly 153,600 counts */
    config.max_velocity_steps_per_s = 200000U;
    config.max_accel_steps_per_s2 = 20000U;
    config.hold_current_ma = 1600U;
    config.run_current_ma = 3000U;
#else
    config.home_position_steps = 0;
    config.max_extension_steps = 10000;
    config.max_velocity_steps_per_s = 1000U;
    config.max_accel_steps_per_s2 = 1000U;
    config.hold_current_ma = 100U;
    config.run_current_ma = 200U;
#endif
    config.direction_inverted = (uint8_t)AMBAR_ACTUATOR_DIRECTION_INVERTED;
    config.require_diag_for_home = 0U;
    return config;
}

AmbarActuatorState_t AmbarActuator_DefaultState(void)
{
    AmbarActuatorState_t state = {0};

    state.home_position_steps = 0;
    state.max_extension_steps = 10000;
    state.requested_position_steps = 0;
    state.actual_position_steps = 0;
    state.commanded_position_steps = 0;
    state.max_velocity_steps_per_s = 1000U;
    state.max_accel_steps_per_s2 = 1000U;
    state.hold_current_ma = 100U;
    state.run_current_ma = 200U;
    state.machine_state = AMBAR_ACTUATOR_STATE_DISABLED;
    state.last_inhibit_flags = ambar_compile_time_inhibit_flags()
        | AMBAR_ACTUATOR_INHIBIT_NOT_HOMED
        | AMBAR_ACTUATOR_INHIBIT_SOFTWARE_HOME_NOT_READY;
    state.endpoint_sequence_state =
        AMBAR_ACTUATOR_STROKE_SEQUENCE_UNKNOWN;
    state.hil_override_mode = AMBAR_ACTUATOR_HIL_OVERRIDE_OFF;
    return state;
}

bool AmbarActuator_ApplyConfig(AmbarActuatorState_t *state,
                               const AmbarActuatorConfig_t *config)
{
    bool geometry_changed;

    if (state == NULL || config == NULL
        || config->max_extension_steps <= 0
        || config->max_velocity_steps_per_s == 0U
        || config->max_accel_steps_per_s2 == 0U
        || config->run_current_ma == 0U)
    {
        if (state != NULL)
        {
            state->config_valid = false;
            state->machine_state = AMBAR_ACTUATOR_STATE_DISABLED;
        }
        return false;
    }

    geometry_changed = state->config_valid
        && (state->home_position_steps != config->home_position_steps
            || state->max_extension_steps != config->max_extension_steps
            || state->direction_inverted != (config->direction_inverted ? 1U : 0U));

    state->home_position_steps = config->home_position_steps;
    state->max_extension_steps = config->max_extension_steps;
    state->requested_position_steps = config->home_position_steps;
    state->max_velocity_steps_per_s = config->max_velocity_steps_per_s;
    state->max_accel_steps_per_s2 = config->max_accel_steps_per_s2;
    state->hold_current_ma = config->hold_current_ma;
    state->run_current_ma = config->run_current_ma;
    state->direction_inverted = config->direction_inverted ? 1U : 0U;
    state->config_valid = true;
    state->command_valid = 0U;
    state->normal_motion_limits_applied = 0U;
    ambar_clear_automatic_motion(state);
    if (geometry_changed)
    {
        state->homed = false;
    }
    if (!state->homed)
    {
        state->home_limit_active = 0U;
        state->full_limit_active = 0U;
        state->home_limit_raw = 0U;
        state->full_limit_raw = 0U;
        state->limit_sample_started = 0U;
        state->limit_state_initialized = 0U;
        state->limits_plausible = 0U;
        state->endpoint_sequence_verified = 0U;
        state->endpoint_sequence_state =
            AMBAR_ACTUATOR_STROKE_SEQUENCE_UNKNOWN;
    }

    if (state->motor_driver_ok
        && (TMC5240_SetCurrentLimits(state->hold_current_ma,
                                     state->run_current_ma) != HAL_OK
            || TMC5240_SetMotionLimits(state->max_velocity_steps_per_s,
                                       state->max_accel_steps_per_s2) != HAL_OK))
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
        return false;
    }
    state->normal_motion_limits_applied = state->motor_driver_ok ? 1U : 0U;
    state->machine_state = AMBAR_ACTUATOR_STATE_IDLE;
    ambar_update_software_geometry(state);
    return true;
}

AmbarActuatorDecision_t AmbarActuator_Evaluate(const AmbarFlightOutput_t *flight_output,
                                               const AmbarActuatorState_t *state)
{
    AmbarActuatorDecision_t decision;
    float deploy_fraction;
    int32_t travel;

    decision.driver_enabled = false;
    decision.target_position_steps =
        state != NULL ? state->home_position_steps : 0;
    decision.inhibit_flags = ambar_common_motion_gates(state);
    decision.machine_state =
        state != NULL ? state->machine_state : AMBAR_ACTUATOR_STATE_DISABLED;

    if (flight_output == NULL || flight_output->airbrake_command.inhibit)
    {
        decision.inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_FLIGHT_COMMAND;
    }
    if (decision.inhibit_flags != AMBAR_ACTUATOR_INHIBIT_NONE)
    {
        return decision;
    }

    deploy_fraction =
        ambar_clamp_fraction(flight_output->airbrake_command.deploy_fraction);
    if (!(deploy_fraction > 0.0f))
    {
        return decision;
    }
    travel = (int32_t)(deploy_fraction
                       * (float)state->max_extension_steps + 0.5f);
    if (state->direction_inverted != 0U)
    {
        travel = -travel;
    }
    decision.target_position_steps =
        ambar_saturate_i64((int64_t)state->home_position_steps + travel);
    decision.driver_enabled = travel != 0;
    return decision;
}

/* -------------------- Cooperative motion services -------------------- */

static AmbarActuatorDecision_t ambar_decision_for_state(
    const AmbarActuatorState_t *state)
{
    AmbarActuatorDecision_t decision;
    decision.driver_enabled = false;
    decision.target_position_steps =
        state != NULL ? state->requested_position_steps : 0;
    decision.inhibit_flags = ambar_common_motion_gates(state);
    decision.machine_state =
        state != NULL ? state->machine_state : AMBAR_ACTUATOR_STATE_DISABLED;
    return decision;
}

static AmbarActuatorDecision_t ambar_service_hil_override(
    AmbarActuatorState_t *state)
{
    AmbarActuatorDecision_t decision = ambar_decision_for_state(state);
    const bool force_full =
        state->hil_override_mode == AMBAR_ACTUATOR_HIL_OVERRIDE_FORCE_FULL;
    const int32_t target = force_full
        ? ambar_full_position(state) : state->home_position_steps;

    decision.target_position_steps = target;
    decision.machine_state = AMBAR_ACTUATOR_STATE_HIL_OVERRIDE;
    decision.inhibit_flags = ambar_common_motion_gates(state);

#if AMBAR_FEATURE_EFFECTIVE_HIL_OVERRIDE == 0
    decision.inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_HIL_PROFILE;
    ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_HIL_PROFILE, false);
    return ambar_decision_for_state(state);
#else
    if (decision.inhibit_flags != AMBAR_ACTUATOR_INHIBIT_NONE)
    {
        ambar_set_driver_enabled(state, 0U);
        return decision;
    }
    if (state->hil_endpoint_reached != 0U)
    {
        ambar_set_driver_enabled(state, 0U);
        return decision;
    }
    if (ambar_step_distance(state->actual_position_steps, target)
        <= (uint32_t)AMBAR_ACTUATOR_ENDPOINT_POSITION_TOLERANCE_STEPS)
    {
        if (force_full)
        {
            if (!ambar_complete_full(state, true))
            {
                if (!state->fault_latched)
                {
                    ambar_latch_fault(state,
                                      AMBAR_ACTUATOR_INHIBIT_GEOMETRY_FAULT,
                                      false);
                }
            }
        }
        else if (!ambar_complete_return_home(state))
        {
            if (!state->fault_latched)
            {
                ambar_latch_fault(state,
                                  AMBAR_ACTUATOR_INHIBIT_GEOMETRY_FAULT,
                                  false);
            }
        }
        return ambar_decision_for_state(state);
    }
    if (!ambar_motion_watchdog_ok(state))
    {
        return ambar_decision_for_state(state);
    }
    state->requested_position_steps = target;
    if (ambar_write_target(state, target) != HAL_OK)
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
        return ambar_decision_for_state(state);
    }
    state->machine_state = AMBAR_ACTUATOR_STATE_HIL_OVERRIDE;
    decision.driver_enabled = ambar_set_driver_enabled(state, 1U);
    return decision;
#endif
}

static AmbarActuatorDecision_t ambar_service_manual(AmbarActuatorState_t *state)
{
    AmbarActuatorDecision_t decision = ambar_decision_for_state(state);
    const int32_t target = state->requested_position_steps;
    const int32_t full = ambar_full_position(state);
    const bool is_home = target == state->home_position_steps;
    const bool is_full = target == full;

    decision.target_position_steps = target;
    decision.inhibit_flags =
        is_home ? ambar_common_motion_gates(state)
                : ambar_bench_motion_gates(state);

    if (decision.inhibit_flags != AMBAR_ACTUATOR_INHIBIT_NONE)
    {
        ambar_set_driver_enabled(state, 0U);
        return decision;
    }
    if (ambar_step_distance(state->actual_position_steps, target)
        <= (uint32_t)AMBAR_ACTUATOR_ENDPOINT_POSITION_TOLERANCE_STEPS)
    {
        if (is_home)
        {
            (void)ambar_complete_return_home(state);
        }
        else if (is_full)
        {
            (void)ambar_complete_full(state, false);
        }
        else
        {
            (void)ambar_complete_nonendpoint_target(state);
        }
        return ambar_decision_for_state(state);
    }
    if (!ambar_motion_watchdog_ok(state))
    {
        return ambar_decision_for_state(state);
    }
    if (ambar_write_target(state, target) != HAL_OK)
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
        return ambar_decision_for_state(state);
    }
    state->machine_state =
        is_home ? AMBAR_ACTUATOR_STATE_RETRACTING
                : AMBAR_ACTUATOR_STATE_MOVING;
    decision.machine_state = state->machine_state;
    decision.driver_enabled = ambar_set_driver_enabled(state, 1U);
    return decision;
}

#if AMBAR_FEATURE_AUTOMATIC_ACTUATION
static bool ambar_flight_has_critical_retract_inhibit(
    const AmbarFlightOutput_t *flight_output)
{
    const uint32_t critical =
        AMBAR_INHIBIT_ESTIMATOR_UNHEALTHY | AMBAR_INHIBIT_NOT_ARMED;
    return flight_output == NULL
        || !flight_output->armed
        || !flight_output->estimate.healthy
        || (flight_output->airbrake_command.inhibit_flags & critical) != 0U;
}

static bool ambar_flight_requests_safe_retract(
    const AmbarFlightOutput_t *flight_output)
{
    const uint32_t safe_reasons =
        AMBAR_INHIBIT_DESCENDING | AMBAR_INHIBIT_APOGEE_ON_TARGET;
    if (ambar_flight_has_critical_retract_inhibit(flight_output)
        || !(flight_output->airbrake_command.deploy_fraction <= 0.0f))
    {
        return false;
    }
    return flight_output->phase == AMBAR_PHASE_RECOVERY
        || (flight_output->airbrake_command.inhibit_flags & safe_reasons) != 0U;
}

static AmbarActuatorDecision_t ambar_service_automatic_retract(
    AmbarActuatorState_t *state)
{
    AmbarActuatorDecision_t decision = ambar_decision_for_state(state);

    decision.target_position_steps = state->home_position_steps;
    decision.inhibit_flags = ambar_common_motion_gates(state);
    if (decision.inhibit_flags != AMBAR_ACTUATOR_INHIBIT_NONE)
    {
        ambar_clear_automatic_motion(state);
        ambar_set_driver_enabled(state, 0U);
        return decision;
    }
    if (ambar_step_distance(state->actual_position_steps,
                            state->home_position_steps)
        <= (uint32_t)AMBAR_ACTUATOR_ENDPOINT_POSITION_TOLERANCE_STEPS)
    {
        (void)ambar_complete_return_home(state);
        return ambar_decision_for_state(state);
    }
    if (state->automatic_retract_active == 0U)
    {
        state->automatic_retract_active = 1U;
        state->automatic_retract_started_ms = HAL_GetTick();
        ambar_begin_motion_watchdog(state);
    }
    if (!ambar_motion_watchdog_ok(state))
    {
        return ambar_decision_for_state(state);
    }
    state->requested_position_steps = state->home_position_steps;
    if (ambar_write_target(state, state->home_position_steps) != HAL_OK)
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
        return ambar_decision_for_state(state);
    }
    state->automatic_motion_active = 1U;
    state->machine_state = AMBAR_ACTUATOR_STATE_RETRACTING;
    decision.machine_state = state->machine_state;
    decision.driver_enabled = ambar_set_driver_enabled(state, 1U);
    return decision;
}

static AmbarActuatorDecision_t ambar_service_automatic_deployment(
    AmbarActuatorState_t *state,
    const AmbarFlightOutput_t *flight_output)
{
    AmbarActuatorDecision_t decision = AmbarActuator_Evaluate(flight_output, state);
    const int32_t full = ambar_full_position(state);

    if (!decision.driver_enabled)
    {
        state->automatic_motion_active = 0U;
        ambar_set_driver_enabled(state, 0U);
        state->machine_state = state->homed
            ? AMBAR_ACTUATOR_STATE_READY : AMBAR_ACTUATOR_STATE_IDLE;
        decision.machine_state = state->machine_state;
        return decision;
    }

    if (ambar_step_distance(state->actual_position_steps,
                            decision.target_position_steps)
        <= (uint32_t)AMBAR_ACTUATOR_ENDPOINT_POSITION_TOLERANCE_STEPS)
    {
        if (decision.target_position_steps == full)
        {
            (void)ambar_complete_full(state, false);
        }
        else
        {
            state->automatic_deployment_latched = 1U;
            (void)ambar_complete_nonendpoint_target(state);
        }
        return ambar_decision_for_state(state);
    }
    if (state->automatic_motion_active == 0U)
    {
        state->automatic_motion_active = 1U;
        ambar_begin_motion_watchdog(state);
    }
    if (!ambar_motion_watchdog_ok(state))
    {
        return ambar_decision_for_state(state);
    }
    state->requested_position_steps = decision.target_position_steps;
    if (ambar_write_target(state, decision.target_position_steps) != HAL_OK)
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
        return ambar_decision_for_state(state);
    }
    state->machine_state = AMBAR_ACTUATOR_STATE_MOVING;
    decision.machine_state = state->machine_state;
    decision.driver_enabled = ambar_set_driver_enabled(state, 1U);
    if (decision.driver_enabled)
    {
        state->automatic_deployment_latched = 1U;
    }
    return decision;
}
#endif

AmbarActuatorDecision_t AmbarActuator_Task(AmbarActuatorState_t *state,
                                           const AmbarFlightOutput_t *flight_output)
{
    AmbarActuatorDecision_t decision;

    if (state == NULL)
    {
        ambar_set_driver_enabled(NULL, 0U);
        return ambar_decision_for_state(NULL);
    }

    ambar_update_driver_snapshot(state);
    if (!state->fault_latched)
    {
        ambar_update_software_geometry(state);
    }

    if (state->estop_latched)
    {
        state->machine_state = AMBAR_ACTUATOR_STATE_ESTOP;
        ambar_set_driver_enabled(state, 0U);
        state->last_inhibit_flags |= ambar_common_motion_gates(state);
        decision = ambar_decision_for_state(state);
        decision.inhibit_flags |= state->last_inhibit_flags;
        return decision;
    }
    if (state->fault_latched)
    {
        state->machine_state = AMBAR_ACTUATOR_STATE_FAULT;
        ambar_set_driver_enabled(state, 0U);
        state->last_inhibit_flags |= ambar_common_motion_gates(state);
        decision = ambar_decision_for_state(state);
        decision.inhibit_flags |= state->last_inhibit_flags;
        return decision;
    }
    if (state->hil_override_mode != AMBAR_ACTUATOR_HIL_OVERRIDE_OFF)
    {
        decision = ambar_service_hil_override(state);
    }
    else if (state->manual_request_pending != 0U)
    {
        ambar_clear_automatic_motion(state);
        decision = ambar_service_manual(state);
    }
#if !AMBAR_FEATURE_AUTOMATIC_ACTUATION
    else
    {
        (void)flight_output;
        /*
         * Replay controller output remains observable but never becomes physical
         * demand in HIL.  Only the separately labelled override can energize it.
         */
        ambar_set_driver_enabled(state, 0U);
        state->machine_state = state->homed
            ? AMBAR_ACTUATOR_STATE_READY : AMBAR_ACTUATOR_STATE_IDLE;
        decision = ambar_decision_for_state(state);
        decision.inhibit_flags = ambar_common_motion_gates(state);
    }
#else
    else if (state->automatic_deployment_latched != 0U
             && ambar_flight_requests_safe_retract(flight_output))
    {
        decision = ambar_service_automatic_retract(state);
    }
    else
    {
        if (state->automatic_deployment_latched != 0U
            && ambar_flight_has_critical_retract_inhibit(flight_output))
        {
            ambar_clear_automatic_motion(state);
        }
        decision = ambar_service_automatic_deployment(state, flight_output);
    }
#endif
    if (state->fault_latched || state->estop_latched)
    {
        state->last_inhibit_flags |= decision.inhibit_flags;
        decision.inhibit_flags = state->last_inhibit_flags;
    }
    else
    {
        state->last_inhibit_flags = decision.inhibit_flags;
    }
    decision.machine_state = state->machine_state;
    return decision;
}

/* -------------------- Explicit safety and operator commands -------------------- */

void AmbarActuator_EStop(AmbarActuatorState_t *state)
{
    if (state != NULL)
    {
        state->estop_latched = true;
        state->homing_active = 0U;
        state->manual_request_pending = 0U;
        state->hil_override_mode = AMBAR_ACTUATOR_HIL_OVERRIDE_OFF;
        state->hil_endpoint_reached = 0U;
        ambar_clear_automatic_motion(state);
        state->command_valid = 0U;
        state->machine_state = AMBAR_ACTUATOR_STATE_ESTOP;
        state->last_inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_ESTOP;
    }
    ambar_set_driver_enabled(state, 0U);
    (void)TMC5240_Stop();
}

bool AmbarActuator_RequestHome(AmbarActuatorState_t *state)
{
    uint32_t gates;

    if (state == NULL)
    {
        return false;
    }
    gates = ambar_base_hardware_gates(state, false);
    if (gates != AMBAR_ACTUATOR_INHIBIT_NONE
        || state->driver_enabled
        || state->homing_active != 0U
        || state->manual_request_pending != 0U
        || state->hil_override_mode != AMBAR_ACTUATOR_HIL_OVERRIDE_OFF
        || state->automatic_motion_active != 0U
        || state->automatic_deployment_latched != 0U
        || state->automatic_retract_active != 0U)
    {
        state->last_inhibit_flags = gates;
        return false;
    }

    /*
     * This is an operator declaration, not a seek. Assert DRV_ENN first and
     * write the current manually closed position as software zero.
     */
    return ambar_declare_software_home(state);
}

bool AmbarActuator_RequestRetract(AmbarActuatorState_t *state)
{
    if (state == NULL
        || ambar_common_motion_gates(state) != AMBAR_ACTUATOR_INHIBIT_NONE)
    {
        if (state != NULL)
        {
            state->last_inhibit_flags = ambar_common_motion_gates(state);
        }
        return false;
    }
    state->hil_override_mode = AMBAR_ACTUATOR_HIL_OVERRIDE_OFF;
    ambar_clear_automatic_motion(state);
    state->requested_position_steps = state->home_position_steps;
    state->manual_request_pending = 1U;
    state->manual_request_started_ms = HAL_GetTick();
    state->machine_state = AMBAR_ACTUATOR_STATE_RETRACTING;
    ambar_begin_motion_watchdog(state);
    return true;
}

bool AmbarActuator_RequestKnownFullRecoveryRetract(
    AmbarActuatorState_t *state)
{
#if !AMBAR_BUILD_IS_CONTINUOUS_HIL
    if (state != NULL)
    {
        state->last_inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_HIL_PROFILE;
    }
    return false;
#else
    uint32_t gates;
    uint32_t driver_status = 0U;
    int32_t reset_actual_steps = 0;
    int32_t reset_target_steps = 0;
    int32_t full_position_steps;

    if (state == NULL)
    {
        return false;
    }

    /*
     * This is deliberately narrower than HOME. It is authorized only for the
     * observed reset-zero mismatch where the operator independently knows the
     * mechanism is physically at configured FULL. No already-established
     * coordinate system may be overwritten by this exceptional command.
     */
    gates = ambar_base_hardware_gates(state, false);
    if (gates != AMBAR_ACTUATOR_INHIBIT_NONE
        || state->driver_enabled
        || state->homed
        || state->homing_active != 0U
        || state->manual_request_pending != 0U
        || state->hil_override_mode != AMBAR_ACTUATOR_HIL_OVERRIDE_OFF
        || state->automatic_motion_active != 0U
        || state->automatic_deployment_latched != 0U
        || state->automatic_retract_active != 0U
        || state->home_limit_active != 0U
        || state->full_limit_active != 0U
        || state->limit_state_initialized != 0U
        || state->limits_plausible != 0U
        || state->endpoint_sequence_state
               != AMBAR_ACTUATOR_STROKE_SEQUENCE_UNKNOWN)
    {
        if (state->driver_enabled || state->homed
            || state->home_limit_active != 0U
            || state->full_limit_active != 0U
            || state->limit_state_initialized != 0U
            || state->limits_plausible != 0U
            || state->endpoint_sequence_state
                   != AMBAR_ACTUATOR_STROKE_SEQUENCE_UNKNOWN)
        {
            gates |= AMBAR_ACTUATOR_INHIBIT_GEOMETRY_FAULT;
        }
        state->last_inhibit_flags = gates;
        return false;
    }

    /* Read both ramp registers and DRV_STATUS fresh before changing either. */
    if (TMC5240_ReadActualPosition(&reset_actual_steps) != HAL_OK
        || TMC5240_ReadTargetPosition(&reset_target_steps) != HAL_OK
        || TMC5240_ReadDriverStatus(&driver_status) != HAL_OK
        || TMC5240_DriverStatusHasHardFault(driver_status) != 0U)
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
        return false;
    }
    state->actual_position_steps = reset_actual_steps;
    state->last_driver_status = driver_status;
    if (ambar_step_distance(reset_actual_steps, 0)
            > (uint32_t)AMBAR_ACTUATOR_ENDPOINT_POSITION_TOLERANCE_STEPS
        || ambar_step_distance(reset_target_steps, 0)
            > (uint32_t)AMBAR_ACTUATOR_ENDPOINT_POSITION_TOLERANCE_STEPS)
    {
        state->last_inhibit_flags = AMBAR_ACTUATOR_INHIBIT_GEOMETRY_FAULT;
        return false;
    }

    full_position_steps = ambar_full_position(state);

    /*
     * DRV_ENN remains asserted throughout. The main loop is cooperative, so
     * committing the software state only after both register writes makes the
     * re-label atomic with respect to every motion-authority caller.
     */
    ambar_set_driver_enabled(state, 0U);
    if (TMC5240_Stop() != HAL_OK)
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
        return false;
    }
    if (TMC5240_SetActualPosition(full_position_steps) != HAL_OK)
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
        return false;
    }
    if (TMC5240_SetTargetPosition(full_position_steps) != HAL_OK)
    {
        /* Best-effort rollback; a latched fault keeps energy off either way. */
        (void)TMC5240_SetTargetPosition(reset_target_steps);
        (void)TMC5240_SetActualPosition(reset_actual_steps);
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
        return false;
    }

    state->actual_position_steps = full_position_steps;
    state->requested_position_steps = full_position_steps;
    state->commanded_position_steps = full_position_steps;
    state->command_valid = 1U;
    state->homed = true;
    state->endpoint_sequence_verified = 0U;
    state->endpoint_sequence_state =
        AMBAR_ACTUATOR_STROKE_SEQUENCE_LEFT_HOME;
    state->machine_state = AMBAR_ACTUATOR_STATE_READY;
    ambar_update_software_geometry(state);
    if (state->fault_latched
        || state->full_limit_active == 0U
        || state->limits_plausible == 0U
        || state->endpoint_sequence_state
               != AMBAR_ACTUATOR_STROKE_SEQUENCE_AT_FULL)
    {
        if (!state->fault_latched)
        {
            ambar_latch_fault(state,
                              AMBAR_ACTUATOR_INHIBIT_GEOMETRY_FAULT,
                              false);
        }
        return false;
    }

    state->last_inhibit_flags = AMBAR_ACTUATOR_INHIBIT_NONE;
    if (!AmbarActuator_RequestRetract(state))
    {
        ambar_latch_fault(state,
                          AMBAR_ACTUATOR_INHIBIT_GEOMETRY_FAULT,
                          false);
        return false;
    }
    return true;
#endif
}

bool AmbarActuator_RequestBenchMove(AmbarActuatorState_t *state,
                                    int32_t target_steps)
{
    int32_t lower = 0;
    int32_t upper = 0;

    if (state == NULL
        || ambar_bench_motion_gates(state) != AMBAR_ACTUATOR_INHIBIT_NONE)
    {
        if (state != NULL)
        {
            state->last_inhibit_flags = ambar_bench_motion_gates(state);
        }
        return false;
    }
    ambar_travel_range(state, &lower, &upper);
    if (target_steps < lower || target_steps > upper)
    {
        state->last_inhibit_flags = AMBAR_ACTUATOR_INHIBIT_CONFIG_INVALID;
        return false;
    }
    state->hil_override_mode = AMBAR_ACTUATOR_HIL_OVERRIDE_OFF;
    ambar_clear_automatic_motion(state);
    state->requested_position_steps = target_steps;
    state->manual_request_pending = 1U;
    state->manual_request_started_ms = HAL_GetTick();
    state->machine_state = target_steps == state->home_position_steps
        ? AMBAR_ACTUATOR_STATE_RETRACTING : AMBAR_ACTUATOR_STATE_MOVING;
    ambar_begin_motion_watchdog(state);
    return true;
}

bool AmbarActuator_SetHilOverride(AmbarActuatorState_t *state,
                                  AmbarActuatorHilOverride_t mode)
{
    if (state == NULL
        || mode < AMBAR_ACTUATOR_HIL_OVERRIDE_OFF
        || mode > AMBAR_ACTUATOR_HIL_OVERRIDE_FORCE_HOME)
    {
        return false;
    }
    if (mode == AMBAR_ACTUATOR_HIL_OVERRIDE_OFF)
    {
        return AmbarActuator_StopAndCancel(state);
    }
#if AMBAR_FEATURE_EFFECTIVE_HIL_OVERRIDE == 0
    state->last_inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_HIL_PROFILE;
    return false;
#else
    if (ambar_common_motion_gates(state) != AMBAR_ACTUATOR_INHIBIT_NONE
        || state->homing_active != 0U
        || state->manual_request_pending != 0U)
    {
        state->last_inhibit_flags = ambar_common_motion_gates(state);
        return false;
    }
    if (mode == AMBAR_ACTUATOR_HIL_OVERRIDE_FORCE_FULL
        && (state->home_limit_active == 0U
            || ambar_step_distance(state->actual_position_steps,
                                   state->home_position_steps)
                   > (uint32_t)AMBAR_ACTUATOR_ENDPOINT_POSITION_TOLERANCE_STEPS))
    {
        state->last_inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_GEOMETRY_FAULT;
        return false;
    }
    if (mode == AMBAR_ACTUATOR_HIL_OVERRIDE_FORCE_HOME
        && state->endpoint_sequence_state
               != AMBAR_ACTUATOR_STROKE_SEQUENCE_AT_FULL)
    {
        state->last_inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_GEOMETRY_FAULT;
        return false;
    }

    ambar_set_driver_enabled(state, 0U);
    if (TMC5240_Stop() != HAL_OK)
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
        return false;
    }
    state->manual_request_pending = 0U;
    ambar_clear_automatic_motion(state);
    state->hil_override_mode = (uint8_t)mode;
    state->hil_endpoint_reached = 0U;
    state->machine_state = AMBAR_ACTUATOR_STATE_HIL_OVERRIDE;
    state->requested_position_steps =
        mode == AMBAR_ACTUATOR_HIL_OVERRIDE_FORCE_FULL
        ? ambar_full_position(state) : state->home_position_steps;
    state->command_valid = 0U;
    if (mode == AMBAR_ACTUATOR_HIL_OVERRIDE_FORCE_FULL)
    {
        state->endpoint_sequence_verified = 0U;
        state->endpoint_sequence_state =
            AMBAR_ACTUATOR_STROKE_SEQUENCE_AT_HOME;
    }
    ambar_begin_motion_watchdog(state);
    return true;
#endif
}

bool AmbarActuator_StopAndCancel(AmbarActuatorState_t *state)
{
    HAL_StatusTypeDef status;

    if (state == NULL)
    {
        ambar_set_driver_enabled(NULL, 0U);
        return false;
    }
    state->homing_active = 0U;
    state->manual_request_pending = 0U;
    state->hil_override_mode = AMBAR_ACTUATOR_HIL_OVERRIDE_OFF;
    state->hil_endpoint_reached = 0U;
    ambar_clear_automatic_motion(state);
    ambar_set_driver_enabled(state, 0U);
    status = state->motor_driver_ok ? TMC5240_Stop() : HAL_OK;
    if (status != HAL_OK)
    {
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT, true);
        return false;
    }
    if (state->motor_driver_ok && !ambar_restore_normal_motion_limits(state))
    {
        return false;
    }
    state->requested_position_steps = state->actual_position_steps;
    state->commanded_position_steps = state->actual_position_steps;
    state->command_valid = state->motor_driver_ok ? 1U : 0U;
    if (!state->estop_latched && !state->fault_latched)
    {
        state->machine_state = state->homed
            ? AMBAR_ACTUATOR_STATE_READY : AMBAR_ACTUATOR_STATE_IDLE;
    }
    return true;
}

bool AmbarActuator_IsReadyForFlight(const AmbarActuatorState_t *state)
{
    return state != NULL
        && state->home_limit_active != 0U
        && state->full_limit_active == 0U
        && state->manual_request_pending == 0U
        && state->homing_active == 0U
        && state->hil_override_mode == AMBAR_ACTUATOR_HIL_OVERRIDE_OFF
        && state->automatic_motion_active == 0U
        && state->automatic_deployment_latched == 0U
        && state->automatic_retract_active == 0U
        && !state->driver_enabled
        && ambar_step_distance(state->actual_position_steps,
                               state->home_position_steps)
               <= (uint32_t)AMBAR_ACTUATOR_POSITION_TOLERANCE_STEPS
        && ambar_common_motion_gates(state) == AMBAR_ACTUATOR_INHIBIT_NONE;
}

uint16_t AmbarActuator_GetProtocolStatus(const AmbarActuatorState_t *state)
{
    uint16_t status = 0U;

    if (state != NULL)
    {
        if (state->home_limit_active != 0U)
        {
            status |= ROCKET_ACTUATOR_STATUS_SOFTWARE_HOME;
        }
        if (state->full_limit_active != 0U)
        {
            status |= ROCKET_ACTUATOR_STATUS_SOFTWARE_FULL;
        }
        if (state->limits_plausible != 0U)
        {
            status |= ROCKET_ACTUATOR_STATUS_GEOMETRY_VALID;
        }
        if (state->hil_override_mode != AMBAR_ACTUATOR_HIL_OVERRIDE_OFF)
        {
            status |= ROCKET_ACTUATOR_STATUS_OVERRIDE_ACTIVE;
        }
        status |= (uint16_t)(
            ((uint16_t)state->hil_override_mode
             << ROCKET_ACTUATOR_STATUS_OVERRIDE_SHIFT)
            & ROCKET_ACTUATOR_STATUS_OVERRIDE_MASK);
        if (state->endpoint_sequence_verified != 0U)
        {
            status |= ROCKET_ACTUATOR_STATUS_STROKE_VERIFIED;
        }
    }
#if AMBAR_BUILD_IS_CONTINUOUS_HIL
    status |= ROCKET_ACTUATOR_STATUS_CONTINUOUS_HIL;
#endif
#if AMBAR_BUILD_IS_VARIABLE_HIL
    status |= ROCKET_ACTUATOR_STATUS_VARIABLE_HIL;
#endif
    return status;
}

void AmbarActuator_HandleExtiPin(AmbarActuatorState_t *state,
                                 uint16_t GPIO_Pin)
{
    if (state == NULL)
    {
        return;
    }
    if (GPIO_Pin == MOTOR_DIAG0_Pin || GPIO_Pin == MOTOR_DIAG1_Pin)
    {
        ++state->diag_event_count;
        state->diag0_level =
            HAL_GPIO_ReadPin(MOTOR_DIAG0_GPIO_Port, MOTOR_DIAG0_Pin)
                    == GPIO_PIN_SET
            ? 1U : 0U;
        state->diag1_level =
            HAL_GPIO_ReadPin(MOTOR_DIAG1_GPIO_Port, MOTOR_DIAG1_Pin)
                    == GPIO_PIN_SET
            ? 1U : 0U;
        ambar_latch_fault(state, AMBAR_ACTUATOR_INHIBIT_DIAG_FAULT, true);
    }
}
