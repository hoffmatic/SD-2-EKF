/**
 * @file ambar_actuator.c
 * @brief Final gated state machine between flight intent and TMC5240 motion.
 *
 * OVERVIEW
 * --------
 *   Final airbrake motion authority. It converts the [ARCH-4] flight request or
 *   an explicit bench request into bounded TMC5240 targets only after [ARCH-5]
 *   hardware and state gates pass.
 *
 * HOW IT WORKS
 * ------------
 *   Startup is energy-off. Each bounded [ARCH-1] scheduler pass reads TMC status,
 *   checks diagnostic/build/config/HOME gates, services manual motion, then
 *   services automatic deployment. A successful nonzero automatic deployment
 *   alone can authorize a later bounded return to HOME.
 *
 * SECTION MAP
 * -----------
 *   1. Numeric/state helpers and direct hardware gates
 *   2. Configuration, initialization, and command evaluation
 *   3. Manual motion and bounded automatic return-to-HOME service
 *   4. Cooperative task, ESTOP/stop, HOME, and bench commands
 *
 * CONFIGURATION
 * -------------
 *   AMBAR_FEATURE_ACTUATOR and AMBAR_FEATURE_BENCH_ACTUATOR_COMMANDS are the
 *   compile-time locks. Travel, current, ramp limits, progress distance, and
 *   retract timeout must be validated on the loaded mechanism.
 *
 * SAFETY ASSUMPTIONS
 * ------------------
 *   XACTUAL is open-loop ramp-generator state, not encoder feedback. The PCB has
 *   DIAG0/DIAG1 but no dedicated home switch, so HOME is an explicit operator
 *   declaration available only through the bench-gated path.
 *
 * REMAINING FLIGHT WORK
 * ---------------------
 *   Real homing, measured current conversion, independent end stops, and loaded
 *   automatic-retract validation are still required before flight use.
 */

#include "ambar_actuator.h"

#include "main.h"
#include "tmc5240.h"

#include <stddef.h>

#define AMBAR_ACTUATOR_POSITION_TOLERANCE_STEPS 100L
#define AMBAR_ACTUATOR_MANUAL_TIMEOUT_MS        8000UL
#define AMBAR_ACTUATOR_AUTO_RETRACT_TIMEOUT_MS  8000UL
#define AMBAR_ACTUATOR_STALL_PROGRESS_STEPS     100L
#define AMBAR_ACTUATOR_STALL_TIMEOUT_MS         2500UL

/* -------------------- Numeric and state helpers -------------------- */

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

static uint32_t ambar_step_distance(int32_t first, int32_t second)
{
    int64_t difference = (int64_t)first - (int64_t)second;
    if (difference < 0)
    {
        difference = -difference;
    }
    return difference > UINT32_MAX ? UINT32_MAX : (uint32_t)difference;
}

static void ambar_clear_automatic_motion_history(AmbarActuatorState_t *state)
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

static bool ambar_flight_has_critical_retract_inhibit(
    const AmbarFlightOutput_t *flight_output)
{
    const uint32_t critical_flags =
        AMBAR_INHIBIT_ESTIMATOR_UNHEALTHY | AMBAR_INHIBIT_NOT_ARMED;

    return flight_output == NULL
        || !flight_output->armed
        || !flight_output->estimate.healthy
        || (flight_output->airbrake_command.inhibit_flags & critical_flags) != 0U;
}

static bool ambar_flight_requests_safe_retract(
    const AmbarFlightOutput_t *flight_output)
{
    const uint32_t safe_retract_reasons =
        AMBAR_INHIBIT_DESCENDING | AMBAR_INHIBIT_APOGEE_ON_TARGET;

    if (ambar_flight_has_critical_retract_inhibit(flight_output)
        || !(flight_output->airbrake_command.deploy_fraction <= 0.0f))
    {
        return false;
    }

    return flight_output->phase == AMBAR_PHASE_RECOVERY
        || (flight_output->airbrake_command.inhibit_flags & safe_retract_reasons) != 0U;
}

/* -------------------- TMC5240 command and hardware-gate helpers -------------------- */

static HAL_StatusTypeDef ambar_write_target_if_changed(AmbarActuatorState_t *state,
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

    /*
     * MOTOR_DIAG interrupts can latch a fault at any point in the cooperative
     * actuator task.  Make release of active-low DRV_ENN the final, atomic gate:
     * a fault that already ran is observed here; a pending fault runs as soon as
     * PRIMASK is restored and asserts DRV_ENN again in the ISR.
     */
    interrupt_mask = __get_PRIMASK();
    __disable_irq();
#if AMBAR_FEATURE_ACTUATOR
    enable_allowed = state->motor_driver_ok
        && state->config_valid
        && state->homed
        && !state->estop_latched
        && !state->fault_latched;
#endif
    TMC5240_SetDriverEnabled(enable_allowed ? 1U : 0U);
    state->driver_enabled = enable_allowed;
    __DMB();
    __set_PRIMASK(interrupt_mask);
    return enable_allowed;
}

static void ambar_manual_travel_range(const AmbarActuatorState_t *state,
                                      int32_t *lower,
                                      int32_t *upper)
{
    int64_t end;

    if (state == NULL || lower == NULL || upper == NULL)
    {
        return;
    }

    end = (int64_t)state->home_position_steps
        + (state->direction_inverted != 0U
            ? -(int64_t)state->max_extension_steps
            : (int64_t)state->max_extension_steps);
    if (end > INT32_MAX)
    {
        end = INT32_MAX;
    }
    else if (end < INT32_MIN)
    {
        end = INT32_MIN;
    }

    if (end < state->home_position_steps)
    {
        *lower = (int32_t)end;
        *upper = state->home_position_steps;
    }
    else
    {
        *lower = state->home_position_steps;
        *upper = (int32_t)end;
    }
}

static uint32_t ambar_compile_time_inhibit_flags(void)
{
    uint32_t flags = AMBAR_ACTUATOR_INHIBIT_NONE;

#if AMBAR_FEATURE_ACTUATOR == 0
    flags |= AMBAR_ACTUATOR_INHIBIT_BUILD_FLAG;
#endif

    return flags;
}

static uint32_t ambar_bench_compile_time_inhibit_flags(void)
{
    uint32_t flags = ambar_compile_time_inhibit_flags();

#if AMBAR_FEATURE_BENCH_ACTUATOR_COMMANDS == 0
    flags |= AMBAR_ACTUATOR_INHIBIT_BENCH_FLAG;
#endif

    return flags;
}

static void ambar_latch_driver_fault(AmbarActuatorState_t *state)
{
    if (state != NULL)
    {
        state->motor_driver_ok = false;
        state->fault_latched = true;
        state->manual_request_pending = 0U;
        ambar_clear_automatic_motion_history(state);
        state->command_valid = 0U;
        state->machine_state = AMBAR_ACTUATOR_STATE_FAULT;
        state->last_inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT;
    }

    /* DRV_ENN is the final hardware gate; assert it immediately on SPI loss. */
    ambar_set_driver_enabled(state, 0U);
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
        if (TMC5240_ReadActualPosition(&state->actual_position_steps) != HAL_OK
            || TMC5240_ReadDriverStatus(&state->last_driver_status) != HAL_OK
            || TMC5240_DriverStatusHasHardFault(state->last_driver_status) != 0U)
        {
            ambar_latch_driver_fault(state);
        }
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
    if (state == NULL || state->fault_latched)
    {
        flags |= AMBAR_ACTUATOR_INHIBIT_DIAG_FAULT;
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

static uint32_t ambar_manual_motion_gates(const AmbarActuatorState_t *state)
{
    uint32_t flags = ambar_common_motion_gates(state);

    flags |= ambar_bench_compile_time_inhibit_flags();
    return flags;
}

/* -------------------- Public configuration and initialization -------------------- */

AmbarActuatorConfig_t AmbarActuator_DefaultConfig(void)
{
    /*
     * The presentation branch mirrors the earlier prototype that moved on this
     * hardware; the fallback branch remains a gentle non-motion placeholder.
     */
    AmbarActuatorConfig_t config;

#if AMBAR_FEATURE_PRESENTATION_MOTION
    /* Known-moving prototype profile: 3 rev at 200*256 counts/revolution. */
    config.home_position_steps = 0;
    config.max_extension_steps = 200 * 256 * 3;
    config.max_velocity_steps_per_s = 200000U;
    config.max_accel_steps_per_s2 = 20000U;
    config.hold_current_ma = 1600U; /* uncalibrated mapping -> IHOLD 16 */
    config.run_current_ma = 3000U;  /* uncalibrated mapping -> IRUN 31 */
#else
    config.home_position_steps = 0;
    config.max_extension_steps = 10000;
    config.max_velocity_steps_per_s = 1000U;
    config.max_accel_steps_per_s2 = 1000U;
    config.hold_current_ma = 100U;
    config.run_current_ma = 200U;
#endif
    config.direction_inverted = AMBAR_FEATURE_PRESENTATION_MOTION
        ? (uint8_t)AMBAR_PRESENTATION_ACTUATOR_DIRECTION_INVERTED
        : 0U;
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
    state.fault_latched = false;
    state.driver_enabled = false;
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
    state.last_inhibit_flags = ambar_compile_time_inhibit_flags();
    state.last_driver_status = 0U;
    state.diag_event_count = 0U;
    state.diag0_level = 0U;
    state.diag1_level = 0U;
    state.direction_inverted = 0U;
    state.manual_request_pending = 0U;
    state.command_valid = 0U;
    state.manual_request_started_ms = 0U;
    state.automatic_motion_active = 0U;
    state.automatic_deployment_latched = 0U;
    state.automatic_retract_active = 0U;
    state.automatic_retract_started_ms = 0U;
    state.last_progress_position_steps = 0;
    state.last_progress_ms = 0U;

    return state;
}

bool AmbarActuator_ApplyConfig(AmbarActuatorState_t *state,
                               const AmbarActuatorConfig_t *config)
{
    bool geometry_changed;

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
    ambar_clear_automatic_motion_history(state);
    if (geometry_changed)
    {
        state->homed = false;
    }

    if (state->motor_driver_ok)
    {
        if (TMC5240_SetCurrentLimits(state->hold_current_ma,
                                     state->run_current_ma) != HAL_OK
            || TMC5240_SetMotionLimits(state->max_velocity_steps_per_s,
                                       state->max_accel_steps_per_s2) != HAL_OK)
        {
            ambar_latch_driver_fault(state);
            return false;
        }
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

    /* A zero or non-finite request must never energize an at-HOME actuator. */
    if (!(deploy_fraction > 0.0f))
    {
        return decision;
    }

    int32_t travel = (int32_t)(deploy_fraction * (float)state->max_extension_steps + 0.5f);

    if (travel == 0)
    {
        return decision;
    }

    if (state->direction_inverted != 0U)
    {
        travel = -travel;
    }

    decision.target_position_steps = state->home_position_steps + travel;
    decision.driver_enabled = true;
    return decision;
}

/* -------------------- Bounded automatic return-to-HOME state -------------------- */

static AmbarActuatorDecision_t ambar_service_automatic_retract(
    AmbarActuatorState_t *state)
{
    AmbarActuatorDecision_t decision;
    uint32_t current_error;
    uint32_t now;

    decision.driver_enabled = false;
    decision.target_position_steps = state->home_position_steps;
    decision.inhibit_flags = ambar_common_motion_gates(state);
    decision.machine_state = state->machine_state;

    if (decision.inhibit_flags != AMBAR_ACTUATOR_INHIBIT_NONE)
    {
        ambar_clear_automatic_motion_history(state);
        ambar_set_driver_enabled(state, 0U);
        state->machine_state = state->homed
            ? AMBAR_ACTUATOR_STATE_READY
            : AMBAR_ACTUATOR_STATE_IDLE;
        decision.machine_state = state->machine_state;
        return decision;
    }

    now = HAL_GetTick();
    current_error = ambar_step_distance(state->actual_position_steps,
                                        state->home_position_steps);

    if (current_error <= (uint32_t)AMBAR_ACTUATOR_POSITION_TOLERANCE_STEPS)
    {
        /* HOME reached: leave XTARGET at HOME, remove energy, and consume the latch. */
        if (ambar_write_target_if_changed(state, state->home_position_steps) != HAL_OK)
        {
            decision.inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT;
            ambar_latch_driver_fault(state);
            decision.machine_state = state->machine_state;
            return decision;
        }
        state->requested_position_steps = state->home_position_steps;
        ambar_clear_automatic_motion_history(state);
        state->machine_state = AMBAR_ACTUATOR_STATE_READY;
        ambar_set_driver_enabled(state, 0U);
        decision.machine_state = state->machine_state;
        return decision;
    }

    if (state->automatic_retract_active == 0U)
    {
        state->automatic_retract_active = 1U;
        state->automatic_retract_started_ms = now;
        state->last_progress_position_steps = state->actual_position_steps;
        state->last_progress_ms = now;
    }
    else
    {
        const uint32_t previous_error = ambar_step_distance(
            state->last_progress_position_steps,
            state->home_position_steps);

        if ((uint32_t)(now - state->automatic_retract_started_ms)
            > AMBAR_ACTUATOR_AUTO_RETRACT_TIMEOUT_MS)
        {
            decision.inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT;
            ambar_latch_driver_fault(state);
            decision.machine_state = state->machine_state;
            return decision;
        }

        if (previous_error > current_error
            && previous_error - current_error
               >= (uint32_t)AMBAR_ACTUATOR_STALL_PROGRESS_STEPS)
        {
            state->last_progress_position_steps = state->actual_position_steps;
            state->last_progress_ms = now;
        }
        else if ((uint32_t)(now - state->last_progress_ms)
                 > AMBAR_ACTUATOR_STALL_TIMEOUT_MS)
        {
            decision.inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT;
            ambar_latch_driver_fault(state);
            decision.machine_state = state->machine_state;
            return decision;
        }
    }

    if (ambar_write_target_if_changed(state, state->home_position_steps) != HAL_OK)
    {
        decision.inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT;
        ambar_latch_driver_fault(state);
        decision.machine_state = state->machine_state;
        return decision;
    }

    state->requested_position_steps = state->home_position_steps;
    state->automatic_motion_active = 1U;
    state->machine_state = AMBAR_ACTUATOR_STATE_RETRACTING;
    decision.driver_enabled = ambar_set_driver_enabled(state, 1U);
    if (!decision.driver_enabled)
    {
        decision.inhibit_flags |= ambar_common_motion_gates(state);
    }
    decision.machine_state = state->machine_state;
    return decision;
}

/* -------------------- Cooperative actuator task ([ARCH-1], [ARCH-5]) -------------------- */

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
        ambar_set_driver_enabled(state, 0U);
        return decision;
    }

    ambar_update_driver_snapshot(state);

    if (state->estop_latched)
    {
        state->machine_state = AMBAR_ACTUATOR_STATE_ESTOP;
        state->last_inhibit_flags = ambar_common_motion_gates(state);
        ambar_set_driver_enabled(state, 0U);
        decision.inhibit_flags = state->last_inhibit_flags;
        decision.machine_state = state->machine_state;
        return decision;
    }

    if (state->fault_latched)
    {
        state->machine_state = AMBAR_ACTUATOR_STATE_FAULT;
        state->last_inhibit_flags = ambar_common_motion_gates(state);
        ambar_set_driver_enabled(state, 0U);
        decision.inhibit_flags = state->last_inhibit_flags;
        decision.machine_state = state->machine_state;
        return decision;
    }

    if (state->manual_request_pending != 0U)
    {
        /* Manual authority consumes any one-shot automatic-return permission. */
        ambar_clear_automatic_motion_history(state);
        int32_t lower = state->home_position_steps;
        int32_t upper = state->home_position_steps;
        ambar_manual_travel_range(state, &lower, &upper);
        decision.target_position_steps = ambar_clamp_steps(
            state->requested_position_steps,
            lower,
            upper);
        decision.inhibit_flags = ambar_manual_motion_gates(state);
        decision.machine_state = state->machine_state;

        if (decision.inhibit_flags == AMBAR_ACTUATOR_INHIBIT_NONE)
        {
            if ((uint32_t)(HAL_GetTick() - state->manual_request_started_ms)
                > AMBAR_ACTUATOR_MANUAL_TIMEOUT_MS)
            {
                decision.inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT;
                ambar_latch_driver_fault(state);
            }
            else if (ambar_step_distance(state->actual_position_steps,
                                         decision.target_position_steps)
                     <= (uint32_t)AMBAR_ACTUATOR_POSITION_TOLERANCE_STEPS)
            {
                state->manual_request_pending = 0U;
                state->machine_state = AMBAR_ACTUATOR_STATE_READY;
                ambar_set_driver_enabled(state, 0U);
                decision.driver_enabled = false;
            }
            else if (ambar_write_target_if_changed(state,
                                                    decision.target_position_steps) == HAL_OK)
            {
                state->machine_state =
                    (decision.target_position_steps == state->home_position_steps)
                    ? AMBAR_ACTUATOR_STATE_RETRACTING
                    : AMBAR_ACTUATOR_STATE_MOVING;
                decision.driver_enabled = ambar_set_driver_enabled(state, 1U);
                if (!decision.driver_enabled)
                {
                    decision.inhibit_flags |= ambar_manual_motion_gates(state);
                }
            }
            else
            {
                decision.inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT;
                ambar_latch_driver_fault(state);
            }
        }
        else
        {
            ambar_set_driver_enabled(state, 0U);
        }

        state->last_inhibit_flags = decision.inhibit_flags;
        decision.machine_state = state->machine_state;
        return decision;
    }

    if (state->automatic_deployment_latched != 0U
        && ambar_flight_requests_safe_retract(flight_output))
    {
        decision = ambar_service_automatic_retract(state);
        state->last_inhibit_flags = decision.inhibit_flags;
        return decision;
    }

    if (state->automatic_deployment_latched != 0U
        && ambar_flight_has_critical_retract_inhibit(flight_output))
    {
        /* Never resume later from stale history after health or ARM authority is lost. */
        ambar_clear_automatic_motion_history(state);
    }

    decision = AmbarActuator_Evaluate(flight_output, state);
    if (decision.driver_enabled)
    {
        const uint32_t now = HAL_GetTick();
        const uint32_t target_error = ambar_step_distance(
            state->actual_position_steps,
            decision.target_position_steps);

        /* A new positive flight command cancels an in-progress return timer. */
        state->automatic_retract_active = 0U;
        state->automatic_retract_started_ms = 0U;

        if (state->automatic_motion_active == 0U
            || target_error <= (uint32_t)AMBAR_ACTUATOR_POSITION_TOLERANCE_STEPS)
        {
            state->automatic_motion_active = 1U;
            state->last_progress_position_steps = state->actual_position_steps;
            state->last_progress_ms = now;
        }
        else if (ambar_step_distance(state->actual_position_steps,
                                     state->last_progress_position_steps)
                 >= (uint32_t)AMBAR_ACTUATOR_STALL_PROGRESS_STEPS)
        {
            state->last_progress_position_steps = state->actual_position_steps;
            state->last_progress_ms = now;
        }
        else if ((uint32_t)(now - state->last_progress_ms)
                 > AMBAR_ACTUATOR_STALL_TIMEOUT_MS)
        {
            decision.driver_enabled = false;
            decision.inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT;
            ambar_latch_driver_fault(state);
        }

        if (decision.driver_enabled
            && ambar_write_target_if_changed(state,
                                             decision.target_position_steps) == HAL_OK)
        {
            state->machine_state = AMBAR_ACTUATOR_STATE_MOVING;
            state->requested_position_steps = decision.target_position_steps;
            decision.driver_enabled = ambar_set_driver_enabled(state, 1U);
            if (decision.driver_enabled)
            {
                /* This is the sole source of autonomous retract eligibility. */
                state->automatic_deployment_latched = 1U;
            }
            else
            {
                decision.inhibit_flags |= ambar_common_motion_gates(state);
            }
        }
        else if (decision.driver_enabled)
        {
            decision.driver_enabled = false;
            decision.inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT;
            ambar_latch_driver_fault(state);
        }
    }
    else
    {
        state->automatic_motion_active = 0U;
        state->automatic_retract_active = 0U;
        state->automatic_retract_started_ms = 0U;
        state->machine_state =
            state->homed ? AMBAR_ACTUATOR_STATE_READY : AMBAR_ACTUATOR_STATE_IDLE;
        ambar_set_driver_enabled(state, 0U);
    }

    state->last_inhibit_flags = decision.inhibit_flags;
    decision.machine_state = state->machine_state;
    return decision;
}

/* -------------------- Explicit energy-off and bench commands -------------------- */

void AmbarActuator_EStop(AmbarActuatorState_t *state)
{
    if (state != NULL)
    {
        state->estop_latched = true;
        state->manual_request_pending = 0U;
        ambar_clear_automatic_motion_history(state);
        state->command_valid = 0U;
        state->machine_state = AMBAR_ACTUATOR_STATE_ESTOP;
        state->last_inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_ESTOP;
    }

    ambar_set_driver_enabled(state, 0U);
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
    if (ambar_bench_compile_time_inhibit_flags() != AMBAR_ACTUATOR_INHIBIT_NONE
        || !state->motor_driver_ok
        || !state->config_valid
        || state->estop_latched
        || state->fault_latched
        || state->driver_enabled
        || state->manual_request_pending != 0U
        || state->automatic_deployment_latched != 0U
        || state->automatic_retract_active != 0U)
    {
        state->last_inhibit_flags = ambar_manual_motion_gates(state);
        return false;
    }

    state->machine_state = AMBAR_ACTUATOR_STATE_HOMING;
    ambar_set_driver_enabled(state, 0U);
    if (TMC5240_SetActualPosition(state->home_position_steps) != HAL_OK
        || TMC5240_SetTargetPosition(state->home_position_steps) != HAL_OK)
    {
        state->homed = false;
        ambar_latch_driver_fault(state);
        return false;
    }
    state->actual_position_steps = state->home_position_steps;
    state->requested_position_steps = state->home_position_steps;
    state->commanded_position_steps = state->home_position_steps;
    state->homed = true;
    state->manual_request_pending = 0U;
    ambar_clear_automatic_motion_history(state);
    state->command_valid = 1U;
    state->machine_state = AMBAR_ACTUATOR_STATE_READY;
    return true;
}

bool AmbarActuator_RequestRetract(AmbarActuatorState_t *state)
{
    if (state == NULL
        || ambar_manual_motion_gates(state) != AMBAR_ACTUATOR_INHIBIT_NONE)
    {
        if (state != NULL)
        {
            state->last_inhibit_flags = ambar_manual_motion_gates(state);
        }
        return false;
    }

    ambar_clear_automatic_motion_history(state);
    state->requested_position_steps = state->home_position_steps;
    state->manual_request_pending = 1U;
    state->manual_request_started_ms = HAL_GetTick();
    state->machine_state = AMBAR_ACTUATOR_STATE_RETRACTING;
    return true;
}

bool AmbarActuator_RequestBenchMove(AmbarActuatorState_t *state, int32_t target_steps)
{
    int32_t lower;
    int32_t upper;

    if (state == NULL
        || ambar_manual_motion_gates(state) != AMBAR_ACTUATOR_INHIBIT_NONE)
    {
        if (state != NULL)
        {
            state->last_inhibit_flags = ambar_manual_motion_gates(state);
        }
        return false;
    }

    ambar_manual_travel_range(state, &lower, &upper);
    if (target_steps < lower || target_steps > upper)
    {
        state->last_inhibit_flags = AMBAR_ACTUATOR_INHIBIT_CONFIG_INVALID;
        return false;
    }

    ambar_clear_automatic_motion_history(state);
    state->requested_position_steps = target_steps;
    state->manual_request_pending = 1U;
    state->manual_request_started_ms = HAL_GetTick();
    state->machine_state = AMBAR_ACTUATOR_STATE_MOVING;
    return true;
}

bool AmbarActuator_StopAndCancel(AmbarActuatorState_t *state)
{
    HAL_StatusTypeDef status;

    if (state == NULL)
    {
        ambar_set_driver_enabled(state, 0U);
        return false;
    }

    state->manual_request_pending = 0U;
    ambar_clear_automatic_motion_history(state);
    /* Energy-off is immediate even if the following SPI stop command times out. */
    ambar_set_driver_enabled(state, 0U);
    status = state->motor_driver_ok ? TMC5240_Stop() : HAL_OK;

    if (status != HAL_OK)
    {
        ambar_latch_driver_fault(state);
        return false;
    }

    state->requested_position_steps = state->actual_position_steps;
    state->commanded_position_steps = state->actual_position_steps;
    state->command_valid = state->motor_driver_ok ? 1U : 0U;
    if (!state->estop_latched && !state->fault_latched)
    {
        state->machine_state = state->homed
            ? AMBAR_ACTUATOR_STATE_READY
            : AMBAR_ACTUATOR_STATE_IDLE;
    }
    return true;
}

bool AmbarActuator_IsReadyForFlight(const AmbarActuatorState_t *state)
{
    return state != NULL
        && state->manual_request_pending == 0U
        && state->automatic_motion_active == 0U
        && state->automatic_deployment_latched == 0U
        && state->automatic_retract_active == 0U
        && !state->driver_enabled
        && ambar_step_distance(state->actual_position_steps,
                               state->home_position_steps)
           <= (uint32_t)AMBAR_ACTUATOR_POSITION_TOLERANCE_STEPS
        && ambar_common_motion_gates(state) == AMBAR_ACTUATOR_INHIBIT_NONE;
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
        state->fault_latched = true;
        state->motor_driver_ok = false;
        state->manual_request_pending = 0U;
        ambar_clear_automatic_motion_history(state);
        state->command_valid = 0U;
        state->machine_state = AMBAR_ACTUATOR_STATE_FAULT;
        state->last_inhibit_flags |= AMBAR_ACTUATOR_INHIBIT_DIAG_FAULT;
        ambar_set_driver_enabled(state, 0U);
    }
}
