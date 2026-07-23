/* Host tests for the pure VARIABLE_HIL arm and shutdown policy. */

#include "ambar_variable_hil_policy.h"

#include <stdbool.h>
#include <stdio.h>

#define CHECK(condition)                                                        \
    do                                                                          \
    {                                                                           \
        if (!(condition))                                                       \
        {                                                                       \
            (void)printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            return false;                                                       \
        }                                                                       \
    } while (0)

static AmbarVariableHilArmInputs_t ready_arm_inputs(void)
{
    const AmbarVariableHilArmInputs_t inputs = {
        .simulation_active = true,
        .simulation_sample_valid = true,
        .simulation_stale = false,
        .simulation_sample_age_ms = 100U,
        .simulation_timeout_ms = 100U,
        .config_upload_accepted = true,
        .actuator_ready = true
    };
    return inputs;
}

static bool test_arm_requires_fresh_simulation(void)
{
    AmbarVariableHilArmInputs_t inputs = ready_arm_inputs();

    CHECK(AmbarVariableHilPolicy_CheckArm(NULL)
          == AMBAR_VARIABLE_HIL_ARM_BLOCKED_FRESH_SIMULATION);
    inputs.simulation_active = false;
    CHECK(AmbarVariableHilPolicy_CheckArm(&inputs)
          == AMBAR_VARIABLE_HIL_ARM_BLOCKED_FRESH_SIMULATION);
    inputs = ready_arm_inputs();
    inputs.simulation_sample_valid = false;
    CHECK(AmbarVariableHilPolicy_CheckArm(&inputs)
          == AMBAR_VARIABLE_HIL_ARM_BLOCKED_FRESH_SIMULATION);
    inputs = ready_arm_inputs();
    inputs.simulation_stale = true;
    CHECK(AmbarVariableHilPolicy_CheckArm(&inputs)
          == AMBAR_VARIABLE_HIL_ARM_BLOCKED_FRESH_SIMULATION);
    inputs = ready_arm_inputs();
    inputs.simulation_sample_age_ms = 101U;
    CHECK(AmbarVariableHilPolicy_CheckArm(&inputs)
          == AMBAR_VARIABLE_HIL_ARM_BLOCKED_FRESH_SIMULATION);
    return true;
}

static bool test_arm_requires_uploaded_config_and_ready_actuator(void)
{
    AmbarVariableHilArmInputs_t inputs = ready_arm_inputs();

    inputs.config_upload_accepted = false;
    CHECK(AmbarVariableHilPolicy_CheckArm(&inputs)
          == AMBAR_VARIABLE_HIL_ARM_BLOCKED_CONFIG);
    inputs = ready_arm_inputs();
    inputs.actuator_ready = false;
    CHECK(AmbarVariableHilPolicy_CheckArm(&inputs)
          == AMBAR_VARIABLE_HIL_ARM_BLOCKED_ACTUATOR);
    inputs = ready_arm_inputs();
    CHECK(AmbarVariableHilPolicy_CheckArm(&inputs)
          == AMBAR_VARIABLE_HIL_ARM_ALLOWED);
    return true;
}

static bool common_shutdown_actions_are_safe(
    const AmbarVariableHilShutdownActions_t *actions)
{
    return actions->stop_simulation
        && actions->invalidate_sample
        && actions->clear_barometer_pending
        && actions->clear_correlated_state
        && actions->disarm_flight
        && actions->stop_and_cancel_motion
        && actions->reset_flight_on_pad;
}

static bool test_stale_input_cleanup(void)
{
    const AmbarVariableHilShutdownActions_t actions =
        AmbarVariableHilPolicy_ShutdownActions(
            AMBAR_VARIABLE_HIL_SHUTDOWN_STALE_INPUT);

    CHECK(common_shutdown_actions_are_safe(&actions));
    CHECK(actions.mark_simulation_stale);
    CHECK(!actions.clear_config_upload);
    return true;
}

static bool test_usb_loss_cleanup_revokes_uploaded_config(void)
{
    const AmbarVariableHilShutdownActions_t actions =
        AmbarVariableHilPolicy_ShutdownActions(
            AMBAR_VARIABLE_HIL_SHUTDOWN_USB_LOSS);

    CHECK(common_shutdown_actions_are_safe(&actions));
    CHECK(actions.clear_config_upload);
    CHECK(!actions.mark_simulation_stale);
    return true;
}

static bool test_orderly_stop_preserves_config_but_removes_motion_authority(void)
{
    const AmbarVariableHilShutdownActions_t stop_actions =
        AmbarVariableHilPolicy_ShutdownActions(
            AMBAR_VARIABLE_HIL_SHUTDOWN_SIM_STOP);
    const AmbarVariableHilShutdownActions_t end_actions =
        AmbarVariableHilPolicy_ShutdownActions(
            AMBAR_VARIABLE_HIL_SHUTDOWN_END_OF_STREAM);

    CHECK(common_shutdown_actions_are_safe(&stop_actions));
    CHECK(common_shutdown_actions_are_safe(&end_actions));
    CHECK(!stop_actions.clear_config_upload);
    CHECK(!end_actions.clear_config_upload);
    CHECK(!stop_actions.mark_simulation_stale);
    CHECK(!end_actions.mark_simulation_stale);
    return true;
}

int main(void)
{
    unsigned passed = 0U;
    const unsigned total = 5U;

    passed += test_arm_requires_fresh_simulation() ? 1U : 0U;
    passed += test_arm_requires_uploaded_config_and_ready_actuator() ? 1U : 0U;
    passed += test_stale_input_cleanup() ? 1U : 0U;
    passed += test_usb_loss_cleanup_revokes_uploaded_config() ? 1U : 0U;
    passed += test_orderly_stop_preserves_config_but_removes_motion_authority()
        ? 1U : 0U;

    (void)printf("VARIABLE_HIL policy tests: %u/%u passed\n", passed, total);
    return passed == total ? 0 : 1;
}
