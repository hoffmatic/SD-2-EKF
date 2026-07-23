/**
 * @file ambar_variable_hil_policy.h
 * @brief Pure safety policy for the causal VARIABLE_HIL session.
 *
 * This module deliberately contains no HAL or driver calls.  The application
 * consumes its decisions and performs the requested shutdown operations, while
 * host tests exercise the exact policy without USB or motor hardware.
 */

#ifndef AMBAR_VARIABLE_HIL_POLICY_H
#define AMBAR_VARIABLE_HIL_POLICY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    AMBAR_VARIABLE_HIL_ARM_ALLOWED = 0,
    AMBAR_VARIABLE_HIL_ARM_BLOCKED_FRESH_SIMULATION = 1,
    AMBAR_VARIABLE_HIL_ARM_BLOCKED_CONFIG = 2,
    AMBAR_VARIABLE_HIL_ARM_BLOCKED_ACTUATOR = 3
} AmbarVariableHilArmDecision_t;

typedef struct
{
    bool simulation_active;
    bool simulation_sample_valid;
    bool simulation_stale;
    uint32_t simulation_sample_age_ms;
    uint32_t simulation_timeout_ms;
    bool config_upload_accepted;
    bool actuator_ready;
} AmbarVariableHilArmInputs_t;

typedef enum
{
    AMBAR_VARIABLE_HIL_SHUTDOWN_SIM_STOP = 0,
    AMBAR_VARIABLE_HIL_SHUTDOWN_END_OF_STREAM = 1,
    AMBAR_VARIABLE_HIL_SHUTDOWN_STALE_INPUT = 2,
    AMBAR_VARIABLE_HIL_SHUTDOWN_USB_LOSS = 3
} AmbarVariableHilShutdownReason_t;

typedef struct
{
    bool stop_simulation;
    bool invalidate_sample;
    bool clear_barometer_pending;
    bool clear_correlated_state;
    bool clear_config_upload;
    bool mark_simulation_stale;
    bool disarm_flight;
    bool stop_and_cancel_motion;
    bool reset_flight_on_pad;
} AmbarVariableHilShutdownActions_t;

/** Evaluate every precondition which must be true before VARIABLE_HIL arms. */
AmbarVariableHilArmDecision_t AmbarVariableHilPolicy_CheckArm(
    const AmbarVariableHilArmInputs_t *inputs);

/** Return the complete cleanup contract for a session-ending event. */
AmbarVariableHilShutdownActions_t AmbarVariableHilPolicy_ShutdownActions(
    AmbarVariableHilShutdownReason_t reason);

#ifdef __cplusplus
}
#endif

#endif /* AMBAR_VARIABLE_HIL_POLICY_H */
