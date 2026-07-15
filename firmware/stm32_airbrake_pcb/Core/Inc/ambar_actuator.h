/**
 * @file ambar_actuator.h
 * @brief Types, safety states, and API for the final airbrake motion authority.
 *
 * OVERVIEW
 * --------
 *   Public types and commands for the final airbrake motion-safety authority.
 *   See CODE_GUIDE.md [ARCH-4] for the flight request and [ARCH-5] for the
 *   gated TMC5240 command path.
 *
 * HOW IT WORKS
 * ------------
 *   The flight layer creates a bounded deployment request. The actuator task
 *   applies hardware gates before deployment, remembers only a deployment that
 *   actually passed those gates, and can then make one bounded automatic return
 *   to HOME for healthy/armed on-target, descent, or recovery conditions.
 *
 * CONFIGURATION
 * -------------
 *   AMBAR_FEATURE_PRESENTATION_MOTION in ambar_features.h identifies the guarded
 *   demo build. Geometry, current, velocity, and timeout assumptions still need
 *   loaded-mechanism validation.
 *
 * SAFETY ASSUMPTIONS
 * ------------------
 *   The motor is unsafe unless proven otherwise. A step target is meaningless
 *   until HOME is established. DISARM, disconnect, ESTOP, and faults always
 *   remove energy instead of attempting a powered return.
 *
 * REMAINING FLIGHT WORK
 * ---------------------
 *   HOME is operator-declared because there is no dedicated home switch. Final
 *   current calibration, independent end stops, and loaded retract validation
 *   remain required before flight use.
 */

#ifndef AMBAR_ACTUATOR_H
#define AMBAR_ACTUATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ambar_features.h"
#include "ambar_flight.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    /* All gates passed.  This is not expected until actuator bench validation. */
    AMBAR_ACTUATOR_INHIBIT_NONE = 0U,

    /* AMBAR_FEATURE_ACTUATOR is 0, so this build cannot enable movement. */
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

/* -------------------- Persistent/runtime actuator configuration -------------------- */

typedef struct
{
    /*
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
} AmbarActuatorConfig_t;

/* -------------------- Cooperative motion state ([ARCH-1], [ARCH-5]) -------------------- */

typedef struct
{
    /* True only after a future homing routine establishes home_position_steps. */
    bool homed;

    /* True when the TMC5240 answered the bring-up check. */
    volatile bool motor_driver_ok;

    /* True only after AmbarActuator_ApplyConfig accepts travel/current limits. */
    bool config_valid;

    /* Latched immediately by ESTOP or serious driver/diag fault. */
    bool estop_latched;

    /* Latched until reboot after SPI, DRV_STATUS, timeout, or DIAG failure. */
    volatile bool fault_latched;

    /* Software mirror of the active-low DRV_ENN output state. */
    volatile bool driver_enabled;

    /* Retracted/home position in motor steps. */
    int32_t home_position_steps;

    /* Maximum extension travel in motor steps from home. */
    int32_t max_extension_steps;

    /* Latest requested/actual positions used for telemetry and command ACKs. */
    int32_t requested_position_steps;
    int32_t actual_position_steps;
    int32_t commanded_position_steps;

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
    uint8_t command_valid;
    uint32_t manual_request_started_ms;
    /* True while the automatic deployment/retract task needs frequent service. */
    uint8_t automatic_motion_active;

    /*
     * Set only after a nonzero automatic deployment target passed every hardware
     * gate and DRV_ENN was released. It is the one-shot permission for an
     * autonomous return to HOME; startup and manual moves never set it.
     */
    uint8_t automatic_deployment_latched;

    /* True only during the bounded powered return initiated from flight output. */
    uint8_t automatic_retract_active;
    uint32_t automatic_retract_started_ms;

    /* Position/time checkpoint shared by automatic stall-progress monitoring. */
    int32_t last_progress_position_steps;
    uint32_t last_progress_ms;
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

/* -------------------- Configuration and cooperative task API -------------------- */

AmbarActuatorConfig_t AmbarActuator_DefaultConfig(void);

/* Conservative default: unhomed, driver not OK, retracted at step zero. */
AmbarActuatorState_t AmbarActuator_DefaultState(void);

bool AmbarActuator_ApplyConfig(AmbarActuatorState_t *state,
                               const AmbarActuatorConfig_t *config);

/*
 * Evaluate deployment only. Automatic return-to-HOME policy is stateful and is
 * therefore applied by AmbarActuator_Task(), not by this pure mapping helper.
 */
AmbarActuatorDecision_t AmbarActuator_Evaluate(const AmbarFlightOutput_t *flight_output,
                                               const AmbarActuatorState_t *state);

/* Cooperative state-machine task called from the main flight scheduler. */
AmbarActuatorDecision_t AmbarActuator_Task(AmbarActuatorState_t *state,
                                           const AmbarFlightOutput_t *flight_output);

/* -------------------- Explicit safety and bench commands -------------------- */

/* Explicit commands remain inhibited unless their documented gates pass. */
void AmbarActuator_EStop(AmbarActuatorState_t *state);
bool AmbarActuator_RequestHome(AmbarActuatorState_t *state);
bool AmbarActuator_RequestRetract(AmbarActuatorState_t *state);
bool AmbarActuator_RequestBenchMove(AmbarActuatorState_t *state, int32_t target_steps);

/*
 * Energy-off stop used by DISARM/SIM_STOP/USB timeout. It cancels any pending
 * manual request and holds XTARGET at the latest known position before DRV_ENN
 * is asserted. This is deliberately different from powered RETRACT.
 */
bool AmbarActuator_StopAndCancel(AmbarActuatorState_t *state);

/* True only when HOME and every runtime actuator gate are ready for ARM. */
bool AmbarActuator_IsReadyForFlight(const AmbarActuatorState_t *state);

/* Interrupt callback hook for MOTOR_DIAG0/MOTOR_DIAG1 edges. */
void AmbarActuator_HandleExtiPin(AmbarActuatorState_t *state, uint16_t GPIO_Pin);

#ifdef __cplusplus
}
#endif

#endif /* AMBAR_ACTUATOR_H */
