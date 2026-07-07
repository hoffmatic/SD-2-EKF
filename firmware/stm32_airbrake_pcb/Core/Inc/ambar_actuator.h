/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 17:44:48 -04:00
 *
 * What this file does:
 *   This is the public interface for the airbrake motor safety gate. It names the reasons the motor is not allowed to move and describes the data passed into that safety decision.
 *
 * Process flow:
 *   The flight layer creates a desired deployment amount. The actuator layer checks the build flag, homing state, motor-driver health, and flight inhibit flags before any motor target can be trusted.
 *
 * Main variables and what can be changed:
 *   AMBAR_ENABLE_ACTUATOR is the main switch and should stay 0 until bench testing is complete. home_position_steps and max_extension_steps must come from real mechanism measurements.
 *
 * Assumptions:
 *   The motor is unsafe unless proven otherwise. A step target is meaningless until the mechanism has been homed.
 *
 * What is missing:
 *   Homing, limit and stall handling, current limits, step calibration, emergency retract, and actual motion register writes are not implemented yet.
 */

#ifndef AMBAR_ACTUATOR_H
#define AMBAR_ACTUATOR_H

/*
 * ===================== AMBAR EKF PCB INTEGRATION - NEW FILE =====================
 *
 * This is a safety gate between the flight command and the TMC5240 motor driver.
 * The first PCB EKF build must not move hardware by accident, so motion is
 * compile-time disabled unless AMBAR_ENABLE_ACTUATOR is explicitly set to 1.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "ambar_flight.h"

#include <stdbool.h>
#include <stdint.h>

#ifndef AMBAR_ENABLE_ACTUATOR
#define AMBAR_ENABLE_ACTUATOR 0
#endif

#ifndef AMBAR_ENABLE_ACTUATOR_BENCH
#define AMBAR_ENABLE_ACTUATOR_BENCH 0
#endif

typedef enum
{
    /* All gates passed.  This is not expected until actuator bench validation. */
    AMBAR_ACTUATOR_INHIBIT_NONE = 0U,

    /* AMBAR_ENABLE_ACTUATOR is 0, so this build cannot enable movement. */
    AMBAR_ACTUATOR_INHIBIT_BUILD_FLAG = 1U << 0U,

    /* Home position is unknown; target_steps would be unsafe. */
    AMBAR_ACTUATOR_INHIBIT_NOT_HOMED = 1U << 1U,

    /* The TMC5240 did not pass bring-up or reported a driver issue. */
    AMBAR_ACTUATOR_INHIBIT_DRIVER_FAULT = 1U << 2U,

    /* The flight layer itself says deployment is inhibited. */
    AMBAR_ACTUATOR_INHIBIT_FLIGHT_COMMAND = 1U << 3U,

    /* Bench movement flag is off, so direct bench commands are refused. */
    AMBAR_ACTUATOR_INHIBIT_BENCH_FLAG = 1U << 4U,

    /* Stored actuator settings failed validation. */
    AMBAR_ACTUATOR_INHIBIT_CONFIG_INVALID = 1U << 5U,

    /* Emergency stop latch is active. */
    AMBAR_ACTUATOR_INHIBIT_ESTOP = 1U << 6U,

    /* TMC diagnostic pins or driver-status bits reported a fault. */
    AMBAR_ACTUATOR_INHIBIT_DIAG_FAULT = 1U << 7U
} AmbarActuatorInhibitFlags_t;

typedef enum
{
    AMBAR_ACTUATOR_STATE_DISABLED = 0,
    AMBAR_ACTUATOR_STATE_IDLE = 1,
    AMBAR_ACTUATOR_STATE_HOMING = 2,
    AMBAR_ACTUATOR_STATE_READY = 3,
    AMBAR_ACTUATOR_STATE_MOVING = 4,
    AMBAR_ACTUATOR_STATE_RETRACTING = 5,
    AMBAR_ACTUATOR_STATE_FAULT = 6,
    AMBAR_ACTUATOR_STATE_ESTOP = 7
} AmbarActuatorMachineState_t;

typedef struct
{
    /*
     * BEGIN AMBAR BENCH-GATED EXPANSION - ACTUATOR SETTINGS
     *
     * These settings are saved in flash config and are intentionally required
     * before motion can be enabled.  The defaults are bench placeholders, not
     * flight-qualified mechanism numbers.
     */
    int32_t home_position_steps;
    int32_t max_extension_steps;
    uint32_t max_velocity_steps_per_s;
    uint32_t max_accel_steps_per_s2;
    uint16_t hold_current_ma;
    uint16_t run_current_ma;
    uint8_t direction_inverted;
    uint8_t require_diag_for_home;
    /* END AMBAR BENCH-GATED EXPANSION - ACTUATOR SETTINGS */
} AmbarActuatorConfig_t;

typedef struct
{
    /* True only after a future homing routine establishes home_position_steps. */
    bool homed;

    /* True when the TMC5240 answered the bring-up check. */
    bool motor_driver_ok;

    /* True only after AmbarActuator_ApplyConfig accepts travel/current limits. */
    bool config_valid;

    /* Latched immediately by ESTOP or serious driver/diag fault. */
    bool estop_latched;

    /* Retracted/home position in motor steps. */
    int32_t home_position_steps;

    /* Maximum extension travel in motor steps from home. */
    int32_t max_extension_steps;

    /* Latest requested/actual positions used for telemetry and command ACKs. */
    int32_t requested_position_steps;
    int32_t actual_position_steps;

    uint32_t max_velocity_steps_per_s;
    uint32_t max_accel_steps_per_s2;
    uint16_t hold_current_ma;
    uint16_t run_current_ma;

    AmbarActuatorMachineState_t machine_state;
    uint32_t last_inhibit_flags;
    uint32_t last_driver_status;
    uint32_t diag_event_count;
    uint8_t diag0_level;
    uint8_t diag1_level;
    uint8_t direction_inverted;
    uint8_t manual_request_pending;
} AmbarActuatorState_t;

typedef struct
{
    /* True means it would be safe to release DRV_ENN for motion. */
    bool driver_enabled;

    /* Safe target step position after applying deployment fraction and travel. */
    int32_t target_position_steps;

    /* Bitfield explaining every safety gate that blocked motion. */
    uint32_t inhibit_flags;

    /* State machine value included in telemetry. */
    AmbarActuatorMachineState_t machine_state;
} AmbarActuatorDecision_t;

AmbarActuatorConfig_t AmbarActuator_DefaultConfig(void);

/* Conservative default: unhomed, driver not OK, retracted at step zero. */
AmbarActuatorState_t AmbarActuator_DefaultState(void);

bool AmbarActuator_ApplyConfig(AmbarActuatorState_t *state,
                               const AmbarActuatorConfig_t *config);

/* Evaluate safety gates and map deploy_fraction to target steps if allowed. */
AmbarActuatorDecision_t AmbarActuator_Evaluate(const AmbarFlightOutput_t *flight_output,
                                               const AmbarActuatorState_t *state);

/* Cooperative state-machine task called from the main flight scheduler. */
AmbarActuatorDecision_t AmbarActuator_Task(AmbarActuatorState_t *state,
                                           const AmbarFlightOutput_t *flight_output);

/* Explicit bench/control commands.  They remain inhibited unless gates pass. */
void AmbarActuator_EStop(AmbarActuatorState_t *state);
bool AmbarActuator_RequestHome(AmbarActuatorState_t *state);
bool AmbarActuator_RequestRetract(AmbarActuatorState_t *state);
bool AmbarActuator_RequestBenchMove(AmbarActuatorState_t *state, int32_t target_steps);

/* Interrupt callback hook for MOTOR_DIAG0/MOTOR_DIAG1 edges. */
void AmbarActuator_HandleExtiPin(AmbarActuatorState_t *state, uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif /* AMBAR_ACTUATOR_H */
