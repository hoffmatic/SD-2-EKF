/*
 * Host regression tests for the bounded automatic return-to-HOME state.
 *
 * The production actuator source is compiled unchanged against small HAL and
 * TMC5240 fakes. Tests exercise [ARCH-5] decisions without energizing hardware.
 */

#include "ambar_actuator.h"
#include "main.h"
#include "rocket_protocol.h"
#include "tmc5240.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

GPIO_TypeDef g_fake_motor_diag_port;

static uint32_t s_tick_ms;
static int32_t s_actual_steps;
static int32_t s_target_steps;
static bool s_driver_enabled;
static uint32_t s_enable_calls;
static uint32_t s_target_write_calls;
static uint32_t s_stop_calls;
static HAL_StatusTypeDef s_target_status;
static HAL_StatusTypeDef s_position_read_status;
static HAL_StatusTypeDef s_driver_status_read_status;
static uint32_t s_driver_status;

#define CHECK(condition)                                                        \
    do                                                                          \
    {                                                                           \
        if (!(condition))                                                       \
        {                                                                       \
            (void)printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            return false;                                                       \
        }                                                                       \
    } while (0)

/* -------------------- HAL and TMC5240 fakes -------------------- */

uint32_t HAL_GetTick(void)
{
    return s_tick_ms;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    (void)port;
    (void)pin;
    return GPIO_PIN_RESET;
}

void TMC5240_SetDriverEnabled(uint8_t enabled)
{
    s_driver_enabled = enabled != 0U;
    if (enabled != 0U)
    {
        ++s_enable_calls;
    }
}

HAL_StatusTypeDef TMC5240_SetCurrentLimits(uint16_t hold_current_ma,
                                           uint16_t run_current_ma)
{
    (void)hold_current_ma;
    (void)run_current_ma;
    return HAL_OK;
}

HAL_StatusTypeDef TMC5240_SetMotionLimits(uint32_t max_velocity_steps_per_s,
                                          uint32_t max_accel_steps_per_s2)
{
    (void)max_velocity_steps_per_s;
    (void)max_accel_steps_per_s2;
    return HAL_OK;
}

HAL_StatusTypeDef TMC5240_SetActualPosition(int32_t position_steps)
{
    s_actual_steps = position_steps;
    return HAL_OK;
}

HAL_StatusTypeDef TMC5240_SetTargetPosition(int32_t target_steps)
{
    ++s_target_write_calls;
    if (s_target_status == HAL_OK)
    {
        s_target_steps = target_steps;
    }
    return s_target_status;
}

HAL_StatusTypeDef TMC5240_ReadActualPosition(int32_t *position_steps)
{
    if (s_position_read_status == HAL_OK && position_steps != NULL)
    {
        *position_steps = s_actual_steps;
    }
    return s_position_read_status;
}

HAL_StatusTypeDef TMC5240_ReadTargetPosition(int32_t *position_steps)
{
    if (s_position_read_status == HAL_OK && position_steps != NULL)
    {
        *position_steps = s_target_steps;
    }
    return s_position_read_status;
}

HAL_StatusTypeDef TMC5240_ReadDriverStatus(uint32_t *driver_status)
{
    if (s_driver_status_read_status == HAL_OK && driver_status != NULL)
    {
        *driver_status = s_driver_status;
    }
    return s_driver_status_read_status;
}

uint8_t TMC5240_DriverStatusHasHardFault(uint32_t driver_status)
{
    return driver_status != 0U ? 1U : 0U;
}

HAL_StatusTypeDef TMC5240_Stop(void)
{
    ++s_stop_calls;
    s_target_steps = s_actual_steps;
    return HAL_OK;
}

/* -------------------- Test fixtures -------------------- */

static void fake_reset(void)
{
    s_tick_ms = 0U;
    s_actual_steps = 0;
    s_target_steps = 0;
    s_driver_enabled = false;
    s_enable_calls = 0U;
    s_target_write_calls = 0U;
    s_stop_calls = 0U;
    s_target_status = HAL_OK;
    s_position_read_status = HAL_OK;
    s_driver_status_read_status = HAL_OK;
    s_driver_status = 0U;
}

static bool ready_actuator(AmbarActuatorState_t *state)
{
    AmbarActuatorConfig_t config;

    fake_reset();
    *state = AmbarActuator_DefaultState();
    state->motor_driver_ok = true;
    config = AmbarActuator_DefaultConfig();
    config.home_position_steps = 0;
    config.max_extension_steps = 10000;
    config.max_velocity_steps_per_s = 20000U;
    config.max_accel_steps_per_s2 = 10000U;
    config.hold_current_ma = 100U;
    config.run_current_ma = 200U;
    config.direction_inverted = 0U;

    CHECK(AmbarActuator_ApplyConfig(state, &config));
    CHECK(AmbarActuator_RequestHome(state));
    CHECK(AmbarActuator_IsReadyForFlight(state));

    s_enable_calls = 0U;
    s_target_write_calls = 0U;
    s_stop_calls = 0U;
    return true;
}

static AmbarFlightOutput_t flight_output(float deploy_fraction,
                                          bool inhibit,
                                          uint32_t inhibit_flags,
                                          AmbarFlightPhase_t phase)
{
    AmbarFlightOutput_t output;

    memset(&output, 0, sizeof(output));
    output.estimate.healthy = true;
    output.armed = true;
    output.phase = phase;
    output.airbrake_command.deploy_fraction = deploy_fraction;
    output.airbrake_command.inhibit = inhibit;
    output.airbrake_command.inhibit_flags = inhibit_flags;
    return output;
}

static bool start_automatic_deployment(AmbarActuatorState_t *state,
                                        float deploy_fraction)
{
    const AmbarFlightOutput_t output = flight_output(
        deploy_fraction,
        false,
        AMBAR_INHIBIT_NONE,
        AMBAR_PHASE_AIRBRAKE_ACTIVE);
    const AmbarActuatorDecision_t decision = AmbarActuator_Task(state, &output);

    CHECK(decision.driver_enabled);
    CHECK(state->driver_enabled);
    CHECK(state->automatic_deployment_latched != 0U);
    CHECK(state->machine_state == AMBAR_ACTUATOR_STATE_MOVING);
    CHECK(s_target_steps > state->home_position_steps);
    return true;
}

static AmbarFlightOutput_t recovery_output(void)
{
    return flight_output(
        0.0f,
        true,
        AMBAR_INHIBIT_NOT_IN_COAST | AMBAR_INHIBIT_DESCENDING,
        AMBAR_PHASE_RECOVERY);
}

/* -------------------- Regression cases -------------------- */

static bool test_deploy_then_recovery_retracts_and_stops_at_home(void)
{
    AmbarActuatorState_t state;
    AmbarActuatorDecision_t decision;
    const AmbarFlightOutput_t recovery = recovery_output();

    CHECK(ready_actuator(&state));
    CHECK(start_automatic_deployment(&state, 0.5f));
    s_actual_steps = 5000;
    s_tick_ms = 100U;
    s_enable_calls = 0U;

    decision = AmbarActuator_Task(&state, &recovery);
    CHECK(decision.driver_enabled);
    CHECK(state.automatic_retract_active != 0U);
    CHECK(state.machine_state == AMBAR_ACTUATOR_STATE_RETRACTING);
    CHECK(s_target_steps == state.home_position_steps);
    CHECK(s_enable_calls == 1U);

    s_actual_steps = 2500;
    s_tick_ms = 300U;
    decision = AmbarActuator_Task(&state, &recovery);
    CHECK(decision.driver_enabled);
    CHECK(!state.fault_latched);

    s_actual_steps = state.home_position_steps;
    s_tick_ms = 500U;
    decision = AmbarActuator_Task(&state, &recovery);
    CHECK(!decision.driver_enabled);
    CHECK(!s_driver_enabled);
    CHECK(state.machine_state == AMBAR_ACTUATOR_STATE_READY);
    CHECK(state.automatic_deployment_latched == 0U);
    CHECK(state.automatic_retract_active == 0U);
    CHECK(state.requested_position_steps == state.home_position_steps);
    return true;
}

static bool test_at_home_completion_never_reenergizes(void)
{
    AmbarActuatorState_t state;
    const AmbarFlightOutput_t recovery = recovery_output();
    AmbarActuatorDecision_t decision;

    CHECK(ready_actuator(&state));
    CHECK(start_automatic_deployment(&state, 0.5f));
    s_actual_steps = state.home_position_steps;
    s_enable_calls = 0U;

    decision = AmbarActuator_Task(&state, &recovery);
    CHECK(!decision.driver_enabled);
    CHECK(s_enable_calls == 0U);
    CHECK(s_target_steps == state.home_position_steps);
    CHECK(state.machine_state == AMBAR_ACTUATOR_STATE_READY);
    CHECK(state.automatic_deployment_latched == 0U);
    return true;
}

static bool test_unhealthy_and_not_armed_refuse_powered_retract(void)
{
    AmbarActuatorState_t state;
    AmbarFlightOutput_t blocked;
    AmbarActuatorDecision_t decision;

    CHECK(ready_actuator(&state));
    CHECK(start_automatic_deployment(&state, 0.5f));
    s_actual_steps = 5000;
    s_enable_calls = 0U;
    blocked = recovery_output();
    blocked.estimate.healthy = false;
    blocked.airbrake_command.inhibit_flags |= AMBAR_INHIBIT_ESTIMATOR_UNHEALTHY;
    decision = AmbarActuator_Task(&state, &blocked);
    CHECK(!decision.driver_enabled);
    CHECK(!s_driver_enabled);
    CHECK(s_enable_calls == 0U);
    CHECK(state.automatic_deployment_latched == 0U);

    CHECK(ready_actuator(&state));
    CHECK(start_automatic_deployment(&state, 0.5f));
    s_actual_steps = 5000;
    s_enable_calls = 0U;
    blocked = recovery_output();
    blocked.armed = false;
    blocked.airbrake_command.inhibit_flags |= AMBAR_INHIBIT_NOT_ARMED;
    decision = AmbarActuator_Task(&state, &blocked);
    CHECK(!decision.driver_enabled);
    CHECK(!s_driver_enabled);
    CHECK(s_enable_calls == 0U);
    CHECK(state.automatic_deployment_latched == 0U);
    return true;
}

static bool test_retract_stall_times_out_and_latches_fault(void)
{
    AmbarActuatorState_t state;
    const AmbarFlightOutput_t recovery = recovery_output();
    AmbarActuatorDecision_t decision;
    CHECK(ready_actuator(&state));
    CHECK(start_automatic_deployment(&state, 1.0f));
    s_actual_steps = 10000;
    decision = AmbarActuator_Task(&state, &recovery);
    CHECK(decision.driver_enabled);

    s_tick_ms = 2501U;
    decision = AmbarActuator_Task(&state, &recovery);
    CHECK(!decision.driver_enabled);
    CHECK((decision.inhibit_flags & AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT) != 0U);
    CHECK(state.fault_latched);
    CHECK(!state.motor_driver_ok);
    CHECK(state.machine_state == AMBAR_ACTUATOR_STATE_FAULT);
    CHECK(!s_driver_enabled);
    return true;
}

static bool test_retract_total_timeout_wins_even_with_progress(void)
{
    AmbarActuatorState_t state;
    const AmbarFlightOutput_t recovery = recovery_output();
    AmbarActuatorDecision_t decision;
    CHECK(ready_actuator(&state));
    CHECK(start_automatic_deployment(&state, 1.0f));
    s_actual_steps = 10000;
    decision = AmbarActuator_Task(&state, &recovery);
    CHECK(decision.driver_enabled);

    s_tick_ms = 2000U;
    s_actual_steps = 9800;
    CHECK(AmbarActuator_Task(&state, &recovery).driver_enabled);
    s_tick_ms = 4000U;
    s_actual_steps = 9600;
    CHECK(AmbarActuator_Task(&state, &recovery).driver_enabled);
    s_tick_ms = 6000U;
    s_actual_steps = 9400;
    CHECK(AmbarActuator_Task(&state, &recovery).driver_enabled);
    s_tick_ms = 8001U;
    s_actual_steps = 9200;
    decision = AmbarActuator_Task(&state, &recovery);

    CHECK(!decision.driver_enabled);
    CHECK(state.fault_latched);
    CHECK(state.machine_state == AMBAR_ACTUATOR_STATE_FAULT);
    CHECK(!s_driver_enabled);
    return true;
}

static bool test_startup_and_zero_command_have_no_unintended_motion(void)
{
    AmbarActuatorState_t state;
    AmbarFlightOutput_t output;
    AmbarActuatorDecision_t decision;

    fake_reset();
    state = AmbarActuator_DefaultState();
    output = recovery_output();
    decision = AmbarActuator_Task(&state, &output);
    CHECK(!decision.driver_enabled);
    CHECK(s_enable_calls == 0U);

    CHECK(ready_actuator(&state));
    output = recovery_output();
    decision = AmbarActuator_Task(&state, &output);
    CHECK(!decision.driver_enabled);
    CHECK(s_enable_calls == 0U);

    output = flight_output(0.0f, false, AMBAR_INHIBIT_NONE, AMBAR_PHASE_COAST);
    decision = AmbarActuator_Task(&state, &output);
    CHECK(!decision.driver_enabled);
    CHECK(s_enable_calls == 0U);
    CHECK(state.automatic_deployment_latched == 0U);
    return true;
}

static bool test_energy_off_stop_clears_automatic_permission(void)
{
    AmbarActuatorState_t state;

    CHECK(ready_actuator(&state));
    CHECK(start_automatic_deployment(&state, 0.5f));
    s_actual_steps = 2000;
    CHECK(AmbarActuator_StopAndCancel(&state));
    CHECK(!s_driver_enabled);
    CHECK(state.automatic_deployment_latched == 0U);
    CHECK(state.automatic_retract_active == 0U);
    CHECK(s_stop_calls == 1U);
    return true;
}

static bool test_driver_fault_during_retract_removes_energy(void)
{
    AmbarActuatorState_t state;
    const AmbarFlightOutput_t recovery = recovery_output();
    AmbarActuatorDecision_t decision;

    CHECK(ready_actuator(&state));
    CHECK(start_automatic_deployment(&state, 0.5f));
    s_actual_steps = 5000;
    CHECK(AmbarActuator_Task(&state, &recovery).driver_enabled);

    /* A new hard-fault bit must win over an already authorized return move. */
    s_driver_status = 1U;
    decision = AmbarActuator_Task(&state, &recovery);
    CHECK(!decision.driver_enabled);
    CHECK(!s_driver_enabled);
    CHECK(state.fault_latched);
    CHECK(!state.motor_driver_ok);
    CHECK(state.machine_state == AMBAR_ACTUATOR_STATE_FAULT);
    CHECK(state.automatic_deployment_latched == 0U);
    CHECK(state.automatic_retract_active == 0U);
    return true;
}

static bool test_automatic_profile_rejects_hil_override(void)
{
    AmbarActuatorState_t state;

    CHECK(ready_actuator(&state));
    CHECK(!AmbarActuator_SetHilOverride(
        &state, AMBAR_ACTUATOR_HIL_OVERRIDE_FORCE_FULL));
    CHECK(!s_driver_enabled);
    CHECK(state.hil_override_mode == AMBAR_ACTUATOR_HIL_OVERRIDE_OFF);
    CHECK((state.last_inhibit_flags
           & AMBAR_ACTUATOR_INHIBIT_HIL_PROFILE) != 0U);
    return true;
}

static bool test_automatic_profile_rejects_known_full_recovery(void)
{
    AmbarActuatorState_t state;

    CHECK(ready_actuator(&state));
    CHECK(!AmbarActuator_RequestKnownFullRecoveryRetract(&state));
    CHECK(!s_driver_enabled);
    CHECK((state.last_inhibit_flags
           & AMBAR_ACTUATOR_INHIBIT_HIL_PROFILE) != 0U);
    return true;
}

static bool test_fraction_mapping_and_profile_identity(void)
{
    AmbarActuatorState_t state;
    const AmbarFlightOutput_t output = flight_output(
        0.5f,
        false,
        AMBAR_INHIBIT_NONE,
        AMBAR_PHASE_AIRBRAKE_ACTIVE);
    const uint16_t expected_profile =
#if AMBAR_BUILD_IS_VARIABLE_HIL
        ROCKET_ACTUATOR_STATUS_VARIABLE_HIL;
#else
        0U;
#endif

    CHECK(ready_actuator(&state));
    const AmbarActuatorDecision_t decision = AmbarActuator_Task(&state, &output);
    CHECK(decision.driver_enabled);
    CHECK(decision.target_position_steps == 5000);
    CHECK(s_target_steps == 5000);
    CHECK((AmbarActuator_GetProtocolStatus(&state)
           & ROCKET_ACTUATOR_STATUS_VARIABLE_HIL) == expected_profile);
    CHECK((AmbarActuator_GetProtocolStatus(&state)
           & ROCKET_ACTUATOR_STATUS_CONTINUOUS_HIL) == 0U);
    return true;
}

int main(void)
{
    static const struct
    {
        const char *name;
        bool (*run)(void);
    } tests[] = {
        {"deploy -> recovery -> HOME", test_deploy_then_recovery_retracts_and_stops_at_home},
        {"at-HOME completion is energy-off", test_at_home_completion_never_reenergizes},
        {"unhealthy/not-armed retract refusal", test_unhealthy_and_not_armed_refuse_powered_retract},
        {"stall timeout latches fault", test_retract_stall_times_out_and_latches_fault},
        {"absolute retract timeout", test_retract_total_timeout_wins_even_with_progress},
        {"no startup/zero-command motion", test_startup_and_zero_command_have_no_unintended_motion},
        {"energy-off stop clears permission", test_energy_off_stop_clears_automatic_permission},
        {"driver fault during retract is energy-off", test_driver_fault_during_retract_removes_energy},
        {"automatic profile rejects HIL override", test_automatic_profile_rejects_hil_override},
        {"automatic profile rejects known-FULL recovery",
         test_automatic_profile_rejects_known_full_recovery},
        {"fraction mapping and profile identity", test_fraction_mapping_and_profile_identity},
    };
    uint32_t passed = 0U;

    for (uint32_t index = 0U; index < (uint32_t)(sizeof(tests) / sizeof(tests[0])); ++index)
    {
        if (!tests[index].run())
        {
            (void)printf("Test failed: %s\n", tests[index].name);
            return 1;
        }
        ++passed;
        (void)printf("PASS %s\n", tests[index].name);
    }

    (void)printf("%lu actuator automatic-retract tests passed\n", (unsigned long)passed);
    return 0;
}
