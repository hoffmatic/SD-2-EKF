/**
 * @file ambar_variable_hil_policy.c
 * @brief Pure VARIABLE_HIL arm and fail-safe cleanup decisions.
 */

#include "ambar_variable_hil_policy.h"

#include <string.h>

AmbarVariableHilArmDecision_t AmbarVariableHilPolicy_CheckArm(
    const AmbarVariableHilArmInputs_t *inputs)
{
    if (inputs == NULL
        || !inputs->simulation_active
        || !inputs->simulation_sample_valid
        || inputs->simulation_stale
        || inputs->simulation_sample_age_ms > inputs->simulation_timeout_ms)
    {
        return AMBAR_VARIABLE_HIL_ARM_BLOCKED_FRESH_SIMULATION;
    }
    if (!inputs->config_upload_accepted)
    {
        return AMBAR_VARIABLE_HIL_ARM_BLOCKED_CONFIG;
    }
    if (!inputs->actuator_ready)
    {
        return AMBAR_VARIABLE_HIL_ARM_BLOCKED_ACTUATOR;
    }
    return AMBAR_VARIABLE_HIL_ARM_ALLOWED;
}

AmbarVariableHilShutdownActions_t AmbarVariableHilPolicy_ShutdownActions(
    AmbarVariableHilShutdownReason_t reason)
{
    AmbarVariableHilShutdownActions_t actions;
    (void)memset(&actions, 0, sizeof(actions));

    switch (reason)
    {
    case AMBAR_VARIABLE_HIL_SHUTDOWN_SIM_STOP:
    case AMBAR_VARIABLE_HIL_SHUTDOWN_END_OF_STREAM:
    case AMBAR_VARIABLE_HIL_SHUTDOWN_STALE_INPUT:
    case AMBAR_VARIABLE_HIL_SHUTDOWN_USB_LOSS:
        actions.stop_simulation = true;
        actions.invalidate_sample = true;
        actions.clear_barometer_pending = true;
        actions.clear_correlated_state = true;
        actions.disarm_flight = true;
        actions.stop_and_cancel_motion = true;
        actions.reset_flight_on_pad = true;
        break;

    default:
        /* An invalid reason is fail-safe and receives the strongest cleanup. */
        actions.stop_simulation = true;
        actions.invalidate_sample = true;
        actions.clear_barometer_pending = true;
        actions.clear_correlated_state = true;
        actions.clear_config_upload = true;
        actions.mark_simulation_stale = true;
        actions.disarm_flight = true;
        actions.stop_and_cancel_motion = true;
        actions.reset_flight_on_pad = true;
        return actions;
    }

    actions.clear_config_upload =
        reason == AMBAR_VARIABLE_HIL_SHUTDOWN_USB_LOSS;
    actions.mark_simulation_stale =
        reason == AMBAR_VARIABLE_HIL_SHUTDOWN_STALE_INPUT;
    return actions;
}
