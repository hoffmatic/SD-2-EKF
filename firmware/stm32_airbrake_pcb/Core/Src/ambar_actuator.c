/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 18:31:46 -04:00
 *
 * What this file does:
 *   This file is the airbrake actuator safety layer. It receives either the flight airbrake command or a direct bench command and decides whether the TMC5240 motor driver is allowed to move.
 *
 * Process flow:
 *   Startup creates a disabled actuator state. Config values are applied from safe defaults or flash. Each scheduler pass reads TMC status, checks diagnostic pins, checks build flags, and either keeps the driver disabled or writes a bounded target position.
 *
 * Main variables and what can be changed:
 *   AMBAR_ENABLE_ACTUATOR and AMBAR_ENABLE_ACTUATOR_BENCH are the compile-time motion locks. max_extension_steps, current limits, velocity, and acceleration must be measured before flight.
 *
 * Assumptions:
 *   The PCB has TMC DIAG0 and DIAG1 signals but no clearly named separate home switch. Therefore the HOME command in this firmware declares the current bench position as home only when bench motion is explicitly enabled.
 *
 * What is missing:
 *   Real flight-ready homing, measured current conversion, mechanism end-stop characterization, and emergency-retract hardware validation are still required.
 */

#include "ambar_actuator.h"

#include "main.h"
#include "tmc5240.h"

#include <stddef.h>

/*
 * ===================== AMBAR BENCH-GATED EXPANSION - UPDATED FILE =====================
 *
 * The old actuator code only calculated a theoretical target.  This version adds
 * a small state machine and TMC5240 register calls, but the default build still
 * cannot move hardware because AMBAR_ENABLE_ACTUATOR and AMBAR_ENABLE_ACTUATOR_BENCH
 * both default to 0.
 */

static float ambar_actuator_clamp(float value, float lower, float upper)
{
    if (value < lower)
    {
        return lower;
    }

    if (value > upper)
    {
        return upper;
    }

    return value;
}

static int32_t ambar_clamp_steps(int32_t value, int32_t lower, int32_t upper)
{
    if (value < lower)
    {
        return lower;
    }
    if (value > upper)
    {
        return upper;
    }
    return value;
}

static uint32_t ambar_compile_time_inhibit_flags(void)
{
    uint32_t flags = AMBAR_ACTUATOR_INHIBIT_NONE;

#if AMBAR_ENABLE_ACTUATOR == 0
    flags |= AMBAR_ACTUATOR_INHIBIT_BUILD_FLAG;
#endif

#if AMBAR_ENABLE_ACTUATOR_BENCH == 0
    flags |= AMBAR_ACTUATOR_INHIBIT_BENCH_FLAG;
#endif

    return flags;
}

static void ambar_update_driver_snapshot(AmbarActuatorState_t *state)
{
    if (state == NULL)
    {
        return;
    }

    state->diag0_level =
        (HAL_GPIO_ReadPin(MOTOR_DIAG0_GPIO_Port, MOTOR_DIAG0_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    state->diag1_level =
        (HAL_GPIO_ReadPin(MOTOR_DIAG1_GPIO_Port, MOTOR_DIAG1_Pin) == GPIO_PIN_SET) ? 1U : 0U;

    if (state->motor_driver_ok)
    {
        (void)TMC5240_ReadActualPosition(&state->actual_position_steps);
        (void)TMC5240_ReadDriverStatus(&state->last_driver_status);
    }
}

static uint32_t ambar_common_motion_gates(const AmbarActuatorState_t *state)
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
    if (state == NULL || !state->homed)
    {
        flags |= AMBAR_ACTUATOR_INHIBIT_NOT_HOMED;
    }
    if (state != NULL
        && (state->machine_state == AMBAR_ACTUATOR_STATE_FAULT
            || state->machine_state == AMBAR_ACTUATOR_STATE_ESTOP))
    {
        flags |= AMBAR_ACTUATOR_INHIBIT_DIAG_FAULT;
    }

    return flags;
}

AmbarActuatorConfig_t AmbarActuator_DefaultConfig(void)
{
    /*
     * Bench placeholders only.  These values are intentionally gentle and must
     * be replaced with measured travel/current/speed limits before any real use.
     */
    AmbarActuatorConfig_t config;

    config.home_position_steps = 0;
    config.max_extension_steps = 10000;
    config.max_velocity_steps_per_s = 1000U;
    config.max_accel_steps_per_s2 = 1000U;
    config.hold_current_ma = 100U;
    config.run_current_ma = 200U;
    config.direction_inverted = 0U;
    config.require_diag_for_home = 0U;

    return config;
}

AmbarActuatorState_t AmbarActuator_DefaultState(void)
{
    AmbarActuatorState_t state;

    state.homed = false;
    state.motor_driver_ok = false;
    state.config_valid = false;
    state.estop_latched = false;
    state.home_position_steps = 0;
    state.max_extension_steps = 10000;
    state.requested_position_steps = 0;
    state.actual_position_steps = 0;
    state.max_velocity_steps_per_s = 1000U;
    state.max_accel_steps_per_s2 = 1000U;
    state.hold_current_ma = 100U;
    state.run_current_ma = 200U;
    state.machine_state = AMBAR_ACTUATOR_STATE_DISABLED;
    state.last_inhibit_flags = ambar_compile_time_inhibit_flags();
    state.last_driver_status = 0U;
    state.diag_event_count = 0U;
    state.diag0_level = 0U;
    state.diag1_level = 0U;
    state.direction_inverted = 0U;
    state.manual_request_pending = 0U;

    return state;
}

bool AmbarActuator_ApplyConfig(AmbarActuatorState_t *state,
                               const AmbarActuatorConfig_t *config)
{
    if (state == NULL || config == NULL)
    {
        return false;
    }

    if (config->max_extension_steps <= 0
        || config->max_velocity_steps_per_s == 0U
        || config->max_accel_steps_per_s2 == 0U
        || config->run_current_ma == 0U)
    {
        state->config_valid = false;
        state->machine_state = AMBAR_ACTUATOR_STATE_DISABLED;
        return false;
    }

    state->home_position_steps = config->home_position_steps;
    state->max_extension_steps = config->max_extension_steps;
    state->requested_position_steps = config->home_position_steps;
    state->max_velocity_steps_per_s = config->max_velocity_steps_per_s;
    state->max_accel_steps_per_s2 = config->max_accel_steps_per_s2;
    state->hold_current_ma = config->hold_current_ma;
    state->run_current_ma = config->run_current_ma;
    state->direction_inverted = config->direction_inverted ? 1U : 0U;
    state->config_valid = true;

    if (state->motor_driver_ok)
    {
        (void)TMC5240_SetCurrentLimits(state->hold_current_ma, state->run_current_ma);
        (void)TMC5240_SetMotionLimits(state->max_velocity_steps_per_s,
                                      state->max_accel_steps_per_s2);
    }

    state->machine_state = AMBAR_ACTUATOR_STATE_IDLE;
    return true;
}

AmbarActuatorDecision_t AmbarActuator_Evaluate(const AmbarFlightOutput_t *flight_output,
                                               const AmbarActuatorState_t *state)
{
    AmbarActuatorDecision_t decision;

    decision.driver_enabled = false;
    decision.target_position_steps = (state != NULL) ? state->home_position_steps : 0;
    decision.inhibit_flags = ambar_common_motion_gates(state);
    decision.machine_state =
        (state != NULL) ? state->machine_state : AMBAR_ACTUATOR_STATE_DISABLED;

    if (flight_output == NULL || flight_output->airbrake_command.inhibit)
    {
        decision.inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_FLIGHT_COMMAND;
    }

    if (decision.inhibit_flags != AMBAR_ACTUATOR_INHIBIT_NONE)
    {
        return decision;
    }

    const float deploy_fraction =
        ambar_actuator_clamp(flight_output->airbrake_command.deploy_fraction, 0.0f, 1.0f);
    int32_t travel = (int32_t)(deploy_fraction * (float)state->max_extension_steps + 0.5f);

    if (state->direction_inverted != 0U)
    {
        travel = -travel;
    }

    decision.target_position_steps = state->home_position_steps + travel;
    decision.driver_enabled = true;
    return decision;
}

AmbarActuatorDecision_t AmbarActuator_Task(AmbarActuatorState_t *state,
                                           const AmbarFlightOutput_t *flight_output)
{
    AmbarActuatorDecision_t decision;

    decision.driver_enabled = false;
    decision.target_position_steps = 0;
    decision.inhibit_flags = AMBAR_ACTUATOR_INHIBIT_CONFIG_INVALID;
    decision.machine_state = AMBAR_ACTUATOR_STATE_DISABLED;

    if (state == NULL)
    {
        TMC5240_SetDriverEnabled(0U);
        return decision;
    }

    ambar_update_driver_snapshot(state);

    if (state->estop_latched)
    {
        state->machine_state = AMBAR_ACTUATOR_STATE_ESTOP;
        state->last_inhibit_flags = ambar_common_motion_gates(state);
        TMC5240_SetDriverEnabled(0U);
        (void)TMC5240_Stop();
        decision.inhibit_flags = state->last_inhibit_flags;
        decision.machine_state = state->machine_state;
        return decision;
    }

    if (state->manual_request_pending != 0U)
    {
        decision.target_position_steps = ambar_clamp_steps(
            state->requested_position_steps,
            state->home_position_steps,
            state->home_position_steps + state->max_extension_steps);
        decision.inhibit_flags = ambar_common_motion_gates(state);
        decision.machine_state = state->machine_state;

        if (decision.inhibit_flags == AMBAR_ACTUATOR_INHIBIT_NONE)
        {
            (void)TMC5240_SetMotionLimits(state->max_velocity_steps_per_s,
                                          state->max_accel_steps_per_s2);
            if (TMC5240_SetTargetPosition(decision.target_position_steps) == HAL_OK)
            {
                TMC5240_SetDriverEnabled(1U);
                decision.driver_enabled = true;
                state->machine_state =
                    (decision.target_position_steps == state->home_position_steps)
                    ? AMBAR_ACTUATOR_STATE_RETRACTING
                    : AMBAR_ACTUATOR_STATE_MOVING;
            }
            else
            {
                decision.inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT;
                state->machine_state = AMBAR_ACTUATOR_STATE_FAULT;
            }
        }
        else
        {
            TMC5240_SetDriverEnabled(0U);
        }

        state->last_inhibit_flags = decision.inhibit_flags;
        decision.machine_state = state->machine_state;
        return decision;
    }

    decision = AmbarActuator_Evaluate(flight_output, state);
    if (decision.driver_enabled)
    {
        (void)TMC5240_SetMotionLimits(state->max_velocity_steps_per_s,
                                      state->max_accel_steps_per_s2);
        if (TMC5240_SetTargetPosition(decision.target_position_steps) == HAL_OK)
        {
            TMC5240_SetDriverEnabled(1U);
            state->machine_state = AMBAR_ACTUATOR_STATE_MOVING;
        }
        else
        {
            decision.driver_enabled = false;
            decision.inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT;
            state->machine_state = AMBAR_ACTUATOR_STATE_FAULT;
            TMC5240_SetDriverEnabled(0U);
        }
    }
    else
    {
        state->machine_state =
            state->homed ? AMBAR_ACTUATOR_STATE_READY : AMBAR_ACTUATOR_STATE_IDLE;
        TMC5240_SetDriverEnabled(0U);
    }

    state->last_inhibit_flags = decision.inhibit_flags;
    decision.machine_state = state->machine_state;
    return decision;
}

void AmbarActuator_EStop(AmbarActuatorState_t *state)
{
    if (state != NULL)
    {
        state->estop_latched = true;
        state->machine_state = AMBAR_ACTUATOR_STATE_ESTOP;
        state->last_inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_ESTOP;
    }

    TMC5240_SetDriverEnabled(0U);
    (void)TMC5240_Stop();
}

bool AmbarActuator_RequestHome(AmbarActuatorState_t *state)
{
    if (state == NULL)
    {
        return false;
    }

    /*
     * With no separate home switch net, homing is bench-declare-current-position.
     * It is still blocked by the compile-time motion flags so a flight build
     * cannot accidentally mark an unknown mechanism as homed.
     */
    if (ambar_compile_time_inhibit_flags() != AMBAR_ACTUATOR_INHIBIT_NONE
        || !state->motor_driver_ok
        || !state->config_valid
        || state->estop_latched)
    {
        state->last_inhibit_flags = ambar_common_motion_gates(state);
        return false;
    }

    state->machine_state = AMBAR_ACTUATOR_STATE_HOMING;
    (void)TMC5240_SetActualPosition(state->home_position_steps);
    state->actual_position_steps = state->home_position_steps;
    state->requested_position_steps = state->home_position_steps;
    state->homed = true;
    state->manual_request_pending = 0U;
    state->machine_state = AMBAR_ACTUATOR_STATE_READY;
    return true;
}

bool AmbarActuator_RequestRetract(AmbarActuatorState_t *state)
{
    if (state == NULL)
    {
        return false;
    }

    state->requested_position_steps = state->home_position_steps;
    state->manual_request_pending = 1U;
    state->machine_state = AMBAR_ACTUATOR_STATE_RETRACTING;
    return true;
}

bool AmbarActuator_RequestBenchMove(AmbarActuatorState_t *state, int32_t target_steps)
{
    if (state == NULL)
    {
        return false;
    }

    state->requested_position_steps = target_steps;
    state->manual_request_pending = 1U;
    state->machine_state = AMBAR_ACTUATOR_STATE_MOVING;
    return true;
}

void AmbarActuator_HandleExtiPin(AmbarActuatorState_t *state, uint16_t GPIO_Pin)
{
    if (state == NULL)
    {
        return;
    }

    if (GPIO_Pin == MOTOR_DIAG0_Pin || GPIO_Pin == MOTOR_DIAG1_Pin)
    {
        ++state->diag_event_count;
        state->diag0_level =
            (HAL_GPIO_ReadPin(MOTOR_DIAG0_GPIO_Port, MOTOR_DIAG0_Pin) == GPIO_PIN_SET) ? 1U : 0U;
        state->diag1_level =
            (HAL_GPIO_ReadPin(MOTOR_DIAG1_GPIO_Port, MOTOR_DIAG1_Pin) == GPIO_PIN_SET) ? 1U : 0U;

        /*
         * Latch a fault instead of trying to interpret every possible TMC DIAG
         * meaning in the ISR path.  Bench work can later decide which DIAG source
         * is stall, switch, or warning for the final mechanism.
         */
        state->machine_state = AMBAR_ACTUATOR_STATE_FAULT;
        state->last_inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_DIAG_FAULT;
        TMC5240_SetDriverEnabled(0U);
    }
}
