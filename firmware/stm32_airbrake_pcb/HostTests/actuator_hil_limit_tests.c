/*
 * Native host tests for switch-free CONTINUOUS_HIL actuator geometry.
 *
 * The production actuator source is compiled unchanged with the HIL profile.
 * HAL/TMC functions are faked, so these tests never access or move hardware.
 */

#include "ambar_actuator.h"
#include "main.h"
#include "rocket_protocol.h"
#include "tmc5240.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

GPIO_TypeDef g_fake_motor_diag_port;

static uint32_t s_tick_ms;
static int32_t s_actual_steps;
static int32_t s_target_steps;
static bool s_driver_enabled;
static uint32_t s_enable_calls;
static uint32_t s_set_actual_calls;
static uint32_t s_target_calls;
static uint32_t s_stop_calls;
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
    ++s_set_actual_calls;
    s_actual_steps = position_steps;
    return HAL_OK;
}

HAL_StatusTypeDef TMC5240_SetTargetPosition(int32_t target_steps)
{
    ++s_target_calls;
    s_target_steps = target_steps;
    return HAL_OK;
}

HAL_StatusTypeDef TMC5240_ReadActualPosition(int32_t *position_steps)
{
    if (position_steps == NULL)
    {
        return HAL_ERROR;
    }
    *position_steps = s_actual_steps;
    return HAL_OK;
}

HAL_StatusTypeDef TMC5240_ReadTargetPosition(int32_t *position_steps)
{
    if (position_steps == NULL)
    {
        return HAL_ERROR;
    }
    *position_steps = s_target_steps;
    return HAL_OK;
}

HAL_StatusTypeDef TMC5240_ReadDriverStatus(uint32_t *driver_status)
{
    if (driver_status == NULL)
    {
        return HAL_ERROR;
    }
    *driver_status = s_driver_status;
    return HAL_OK;
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

static void fake_reset(void)
{
    s_tick_ms = 0U;
    s_actual_steps = 0;
    s_target_steps = 0;
    s_driver_enabled = false;
    s_enable_calls = 0U;
    s_set_actual_calls = 0U;
    s_target_calls = 0U;
    s_stop_calls = 0U;
    s_driver_status = 0U;
}

static bool apply_test_config(AmbarActuatorState_t *state,
                              uint8_t direction_inverted)
{
    AmbarActuatorConfig_t config = AmbarActuator_DefaultConfig();

    CHECK(config.home_position_steps == 0);
    CHECK(config.max_extension_steps == 153600);
    config.direction_inverted = direction_inverted;
    *state = AmbarActuator_DefaultState();
    state->motor_driver_ok = true;
    CHECK(AmbarActuator_ApplyConfig(state, &config));
    return true;
}

static bool ready_at_home(AmbarActuatorState_t *state)
{
    fake_reset();
    CHECK(apply_test_config(state, 0U));
    CHECK(AmbarActuator_RequestHome(state));
    CHECK(AmbarActuator_IsReadyForFlight(state));
    s_enable_calls = 0U;
    s_stop_calls = 0U;
    s_target_calls = 0U;
    return true;
}

static bool complete_force_full(AmbarActuatorState_t *state)
{
    CHECK(AmbarActuator_SetHilOverride(
        state, AMBAR_ACTUATOR_HIL_OVERRIDE_FORCE_FULL));
    CHECK(AmbarActuator_Task(state, NULL).driver_enabled);
    CHECK(s_driver_enabled);
    CHECK(s_target_steps == 153600);

    s_tick_ms += 100U;
    s_actual_steps = 50000;
    CHECK(AmbarActuator_Task(state, NULL).driver_enabled);

    s_tick_ms += 100U;
    s_actual_steps = 153550;
    CHECK(!AmbarActuator_Task(state, NULL).driver_enabled);
    CHECK(!state->fault_latched);
    CHECK(state->hil_endpoint_reached != 0U);
    CHECK(state->full_limit_active != 0U);
    CHECK(state->endpoint_sequence_state
          == AMBAR_ACTUATOR_STROKE_SEQUENCE_AT_FULL);
    return true;
}

static bool complete_force_home(AmbarActuatorState_t *state)
{
    const uint32_t set_actual_before_return = s_set_actual_calls;

    CHECK(AmbarActuator_SetHilOverride(
        state, AMBAR_ACTUATOR_HIL_OVERRIDE_FORCE_HOME));
    CHECK(AmbarActuator_Task(state, NULL).driver_enabled);
    CHECK(s_driver_enabled);
    CHECK(s_target_steps == 0);

    s_tick_ms += 100U;
    s_actual_steps = 80000;
    CHECK(AmbarActuator_Task(state, NULL).driver_enabled);

    s_tick_ms += 100U;
    s_actual_steps = 50;
    CHECK(!AmbarActuator_Task(state, NULL).driver_enabled);
    CHECK(!state->fault_latched);
    CHECK(state->hil_endpoint_reached != 0U);
    CHECK(state->home_limit_active != 0U);
    CHECK(state->endpoint_sequence_verified != 0U);
    CHECK(state->endpoint_sequence_state
          == AMBAR_ACTUATOR_STROKE_SEQUENCE_COMPLETE);
    CHECK(s_actual_steps == 50);
    CHECK(s_set_actual_calls == set_actual_before_return);
    CHECK(s_target_steps == 0);
    return true;
}

static bool test_boot_never_energizes_or_zeros(void)
{
    AmbarActuatorState_t state;

    fake_reset();
    s_actual_steps = 42000;
    CHECK(apply_test_config(&state, 0U));
    (void)AmbarActuator_Task(&state, NULL);

    CHECK(!s_driver_enabled);
    CHECK(s_enable_calls == 0U);
    CHECK(s_set_actual_calls == 0U);
    CHECK(!state.homed);
    CHECK(state.home_limit_active == 0U);
    CHECK((AmbarActuator_GetProtocolStatus(&state)
           & ROCKET_ACTUATOR_STATUS_CONTINUOUS_HIL) != 0U);
    return true;
}

static bool test_home_is_energy_off_software_zero(void)
{
    AmbarActuatorState_t state;
    uint16_t status;

    fake_reset();
    s_actual_steps = 42000;
    CHECK(apply_test_config(&state, 0U));
    CHECK(AmbarActuator_RequestHome(&state));

    CHECK(!s_driver_enabled);
    CHECK(s_enable_calls == 0U);
    CHECK(s_stop_calls == 1U);
    CHECK(s_set_actual_calls == 1U);
    CHECK(s_actual_steps == 0);
    CHECK(s_target_steps == 0);
    CHECK(state.homed);
    CHECK(state.machine_state == AMBAR_ACTUATOR_STATE_READY);
    status = AmbarActuator_GetProtocolStatus(&state);
    CHECK((status & ROCKET_ACTUATOR_STATUS_SOFTWARE_HOME) != 0U);
    CHECK((status & ROCKET_ACTUATOR_STATUS_SOFTWARE_FULL) == 0U);
    CHECK((status & ROCKET_ACTUATOR_STATUS_GEOMETRY_VALID) != 0U);
    return true;
}

static bool test_complete_software_stroke(void)
{
    AmbarActuatorState_t state;
    uint16_t status;

    CHECK(ready_at_home(&state));
    CHECK(complete_force_full(&state));
    CHECK(complete_force_home(&state));

    status = AmbarActuator_GetProtocolStatus(&state);
    CHECK((status & ROCKET_ACTUATOR_STATUS_SOFTWARE_HOME) != 0U);
    CHECK((status & ROCKET_ACTUATOR_STATUS_SOFTWARE_FULL) == 0U);
    CHECK((status & ROCKET_ACTUATOR_STATUS_GEOMETRY_VALID) != 0U);
    CHECK((status & ROCKET_ACTUATOR_STATUS_STROKE_VERIFIED) != 0U);
    CHECK((status & ROCKET_ACTUATOR_STATUS_OVERRIDE_ACTIVE) != 0U);

    CHECK(AmbarActuator_SetHilOverride(
        &state, AMBAR_ACTUATOR_HIL_OVERRIDE_OFF));
    CHECK(AmbarActuator_IsReadyForFlight(&state));
    CHECK(state.homed);
    CHECK(s_actual_steps == 50);
    return true;
}

static bool test_full_target_is_signed_three_rotations(void)
{
    AmbarActuatorState_t state;

    CHECK(ready_at_home(&state));
    CHECK(AmbarActuator_SetHilOverride(
        &state, AMBAR_ACTUATOR_HIL_OVERRIDE_FORCE_FULL));
    CHECK(AmbarActuator_Task(&state, NULL).driver_enabled);
    CHECK(s_target_steps == 153600);

    fake_reset();
    CHECK(apply_test_config(&state, 1U));
    CHECK(AmbarActuator_RequestHome(&state));
    CHECK(AmbarActuator_SetHilOverride(
        &state, AMBAR_ACTUATOR_HIL_OVERRIDE_FORCE_FULL));
    CHECK(AmbarActuator_Task(&state, NULL).driver_enabled);
    CHECK(s_target_steps == -153600);
    return true;
}

static bool test_override_requires_declared_home_and_full_order(void)
{
    AmbarActuatorState_t state;

    fake_reset();
    CHECK(apply_test_config(&state, 0U));
    CHECK(!AmbarActuator_SetHilOverride(
        &state, AMBAR_ACTUATOR_HIL_OVERRIDE_FORCE_FULL));
    CHECK(!s_driver_enabled);

    CHECK(AmbarActuator_RequestHome(&state));
    CHECK(!AmbarActuator_SetHilOverride(
        &state, AMBAR_ACTUATOR_HIL_OVERRIDE_FORCE_HOME));
    CHECK(!s_driver_enabled);
    return true;
}

static bool test_known_full_recovery_relabels_then_uses_bounded_retract(void)
{
    AmbarActuatorState_t state;
    AmbarActuatorDecision_t decision;

    fake_reset();
    CHECK(apply_test_config(&state, 0U));
    CHECK(!state.homed);
    CHECK(s_actual_steps == 0);
    CHECK(s_target_steps == 0);

    CHECK(AmbarActuator_RequestKnownFullRecoveryRetract(&state));
    CHECK(!s_driver_enabled);
    CHECK(s_enable_calls == 0U);
    CHECK(s_actual_steps == 153600);
    CHECK(s_target_steps == 153600);
    CHECK(s_set_actual_calls == 1U);
    CHECK(state.homed);
    CHECK(state.full_limit_active != 0U);
    CHECK(state.home_limit_active == 0U);
    CHECK(state.limits_plausible != 0U);
    CHECK(state.endpoint_sequence_state
          == AMBAR_ACTUATOR_STROKE_SEQUENCE_AT_FULL);
    CHECK(state.manual_request_pending != 0U);
    CHECK(state.machine_state == AMBAR_ACTUATOR_STATE_RETRACTING);

    decision = AmbarActuator_Task(&state, NULL);
    CHECK(decision.driver_enabled);
    CHECK(s_driver_enabled);
    CHECK(s_target_steps == 0);

    s_tick_ms += 100U;
    s_actual_steps = 80000;
    CHECK(AmbarActuator_Task(&state, NULL).driver_enabled);

    s_tick_ms += 100U;
    s_actual_steps = 50;
    decision = AmbarActuator_Task(&state, NULL);
    CHECK(!decision.driver_enabled);
    CHECK(!s_driver_enabled);
    CHECK(!state.fault_latched);
    CHECK(state.manual_request_pending == 0U);
    CHECK(state.home_limit_active != 0U);
    CHECK(state.full_limit_active == 0U);
    CHECK(state.endpoint_sequence_verified != 0U);
    CHECK(state.endpoint_sequence_state
          == AMBAR_ACTUATOR_STROKE_SEQUENCE_COMPLETE);
    CHECK(s_target_steps == 0);
    CHECK(s_actual_steps == 50);
    CHECK(s_set_actual_calls == 1U);
    return true;
}

static bool test_known_full_recovery_rejects_any_nonreset_or_known_geometry(void)
{
    AmbarActuatorState_t state;

    fake_reset();
    CHECK(apply_test_config(&state, 0U));
    s_actual_steps = 101;
    CHECK(!AmbarActuator_RequestKnownFullRecoveryRetract(&state));
    CHECK(!state.homed);
    CHECK(!s_driver_enabled);
    CHECK(s_set_actual_calls == 0U);
    CHECK((state.last_inhibit_flags
           & AMBAR_ACTUATOR_INHIBIT_GEOMETRY_FAULT) != 0U);

    fake_reset();
    CHECK(apply_test_config(&state, 0U));
    s_target_steps = -101;
    CHECK(!AmbarActuator_RequestKnownFullRecoveryRetract(&state));
    CHECK(!state.homed);
    CHECK(!s_driver_enabled);
    CHECK(s_set_actual_calls == 0U);

    CHECK(ready_at_home(&state));
    const uint32_t set_actual_before_rejected_recovery = s_set_actual_calls;
    CHECK(!AmbarActuator_RequestKnownFullRecoveryRetract(&state));
    CHECK(state.homed);
    CHECK(!s_driver_enabled);
    CHECK(s_set_actual_calls == set_actual_before_rejected_recovery);
    return true;
}

static bool test_no_progress_timeout_is_fail_off(void)
{
    AmbarActuatorState_t state;

    CHECK(ready_at_home(&state));
    CHECK(AmbarActuator_SetHilOverride(
        &state, AMBAR_ACTUATOR_HIL_OVERRIDE_FORCE_FULL));
    CHECK(AmbarActuator_Task(&state, NULL).driver_enabled);

    s_tick_ms = state.motion_started_ms + 2501U;
    CHECK(!AmbarActuator_Task(&state, NULL).driver_enabled);
    CHECK(state.fault_latched);
    CHECK(!state.motor_driver_ok);
    CHECK(state.hil_override_mode == AMBAR_ACTUATOR_HIL_OVERRIDE_OFF);
    CHECK(!s_driver_enabled);
    return true;
}

static bool test_off_estop_and_driver_fault_remove_energy(void)
{
    AmbarActuatorState_t state;

    CHECK(ready_at_home(&state));
    CHECK(AmbarActuator_SetHilOverride(
        &state, AMBAR_ACTUATOR_HIL_OVERRIDE_FORCE_FULL));
    CHECK(AmbarActuator_Task(&state, NULL).driver_enabled);
    CHECK(AmbarActuator_SetHilOverride(
        &state, AMBAR_ACTUATOR_HIL_OVERRIDE_OFF));
    CHECK(!s_driver_enabled);
    CHECK(state.homed);

    CHECK(ready_at_home(&state));
    CHECK(AmbarActuator_SetHilOverride(
        &state, AMBAR_ACTUATOR_HIL_OVERRIDE_FORCE_FULL));
    CHECK(AmbarActuator_Task(&state, NULL).driver_enabled);
    AmbarActuator_EStop(&state);
    CHECK(!s_driver_enabled);
    CHECK(state.estop_latched);
    CHECK(state.hil_override_mode == AMBAR_ACTUATOR_HIL_OVERRIDE_OFF);

    CHECK(ready_at_home(&state));
    CHECK(AmbarActuator_SetHilOverride(
        &state, AMBAR_ACTUATOR_HIL_OVERRIDE_FORCE_FULL));
    CHECK(AmbarActuator_Task(&state, NULL).driver_enabled);
    s_driver_status = 1U;
    CHECK(!AmbarActuator_Task(&state, NULL).driver_enabled);
    CHECK(state.fault_latched);
    CHECK(!state.motor_driver_ok);
    CHECK(state.hil_override_mode == AMBAR_ACTUATOR_HIL_OVERRIDE_OFF);
    return true;
}

int main(void)
{
    static const struct
    {
        const char *name;
        bool (*run)(void);
    } tests[] = {
        {"boot is energy-off and does not zero",
         test_boot_never_energizes_or_zeros},
        {"HOME is energy-off software zero",
         test_home_is_energy_off_software_zero},
        {"software 0-FULL-0 stroke", test_complete_software_stroke},
        {"signed three-rotation target", test_full_target_is_signed_three_rotations},
        {"override ordering gates",
         test_override_requires_declared_home_and_full_order},
        {"known-FULL recovery uses bounded RETRACT",
         test_known_full_recovery_relabels_then_uses_bounded_retract},
        {"known-FULL recovery rejects non-reset/known geometry",
         test_known_full_recovery_rejects_any_nonreset_or_known_geometry},
        {"no-progress timeout is fail-off", test_no_progress_timeout_is_fail_off},
        {"OFF/ESTOP/DRV_STATUS are fail-off",
         test_off_estop_and_driver_fault_remove_energy},
    };
    uint32_t passed = 0U;

    for (uint32_t index = 0U;
         index < (uint32_t)(sizeof(tests) / sizeof(tests[0]));
         ++index)
    {
        if (!tests[index].run())
        {
            (void)printf("Test failed: %s\n", tests[index].name);
            return 1;
        }
        ++passed;
        (void)printf("PASS %s\n", tests[index].name);
    }

    (void)printf("%lu switch-free continuous-HIL actuator tests passed\n",
                 (unsigned long)passed);
    return 0;
}
