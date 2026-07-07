/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 17:44:48 -04:00
 *
 * What this file does:
 *   This header describes the flight-control layer above the EKF. It names flight phases, inhibit reasons, controller settings, and the output snapshot used by telemetry and actuator safety.
 *
 * Process flow:
 *   Sensor updates feed the EKF. The flight layer watches altitude, speed, and acceleration to decide phase, then computes a deployment request or explains why deployment is inhibited.
 *
 * Main variables and what can be changed:
 *   Phase thresholds and controller values can be tuned. Target apogee, tolerance, minimum deployment altitude, and minimum flight time are expected to change as the vehicle is characterized.
 *
 * Assumptions:
 *   Deployment can only be considered during coast. Any fault or descent should force a safe retracted command.
 *
 * What is missing:
 *   There is no launch arming switch, flight-mode command protocol, drag table, or actuator-effectiveness model yet.
 */

#ifndef AMBAR_FLIGHT_H
#define AMBAR_FLIGHT_H

/*
 * ===================== AMBAR EKF PCB INTEGRATION - NEW FILE =====================
 *
 * This header wraps the EKF in the flight logic the PCB firmware needs:
 * phase tracking, apogee-error control, inhibit flags, and a single output
 * snapshot for telemetry and the actuator safety gate.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "ambar_ekf.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    /* Board is stationary on the pad; estimator should stay near zero altitude. */
    AMBAR_PHASE_PAD_IDLE = 0,

    /* High acceleration after liftoff.  Airbrakes must stay inhibited. */
    AMBAR_PHASE_BOOST = 1,

    /* Motor burnout/coast.  This is the only phase where deployment can be allowed. */
    AMBAR_PHASE_COAST = 2,

    /* Coast plus a nonzero airbrake command. */
    AMBAR_PHASE_AIRBRAKE_ACTIVE = 3,

    /* Descending/recovery state; deployment command is forced safe. */
    AMBAR_PHASE_RECOVERY = 4,

    /* Estimator or sensor health fault. */
    AMBAR_PHASE_FAULT = 5
} AmbarFlightPhase_t;

typedef enum
{
    /* Bitfield explaining why the command is zeroed. */
    AMBAR_INHIBIT_NONE = 0U,
    AMBAR_INHIBIT_ESTIMATOR_UNHEALTHY = 1U << 0U,
    AMBAR_INHIBIT_NOT_IN_COAST = 1U << 1U,
    AMBAR_INHIBIT_BELOW_MINIMUM_ALTITUDE = 1U << 2U,
    AMBAR_INHIBIT_BEFORE_MINIMUM_FLIGHT_TIME = 1U << 3U,
    AMBAR_INHIBIT_DESCENDING = 1U << 4U,
    AMBAR_INHIBIT_APOGEE_ON_TARGET = 1U << 5U,

    /* Added for bench-gated firmware: airbrakes need an explicit ARM command. */
    AMBAR_INHIBIT_NOT_ARMED = 1U << 6U
} AmbarInhibitFlags_t;

typedef enum
{
    /*
     * Ballistic prediction ignores drag.  It is useful as a comparison because
     * the first EKF integration used this simple upper-bound estimate.
     */
    AMBAR_APOGEE_MODE_BALLISTIC = 0,

    /*
     * Drag prediction numerically coasts the vertical state upward with a simple
     * v^2 drag term.  It is still vertical-only, not a full attitude/wind model.
     */
    AMBAR_APOGEE_MODE_DRAG = 1
} AmbarApogeePredictionMode_t;

typedef struct
{
    /* Altitude/acceleration thresholds used to leave pad idle. */
    float liftoff_altitude_m;
    float liftoff_acceleration_mps2;

    /* Minimum boost dwell prevents a short vibration spike from entering coast. */
    float minimum_boost_time_s;

    /* Coast detection after acceleration drops near/under this value. */
    float burnout_acceleration_mps2;

    /* Require upward velocity before allowing coast airbrake logic. */
    float minimum_coast_velocity_mps;

    /* Descent threshold for recovery detection. */
    float recovery_descent_velocity_mps;
} AmbarFlightPhaseConfig_t;

typedef struct
{
    /* Desired apogee and deadband around it. */
    float target_apogee_m;
    float apogee_tolerance_m;

    /* Apogee error that maps to full deployment before clamping. */
    float full_deployment_error_m;

    /* Mechanical/software cap on requested deployment fraction. */
    float maximum_deploy_fraction;

    /* Deployment is inhibited below this altitude and before this flight time. */
    float minimum_deploy_altitude_m;
    float minimum_flight_time_s;
} AmbarAirbrakeControllerConfig_t;

typedef struct
{
    /*
     * BEGIN AMBAR BENCH-GATED EXPANSION - DRAG APOGEE SETTINGS
     *
     * These values let bench/replay work switch between the old ballistic
     * predictor and a simple drag-aware coast estimate without changing the EKF
     * state.  They are deliberately plain floats so they can be saved in the
     * flash configuration record.
     */
    AmbarApogeePredictionMode_t mode;
    float vehicle_mass_kg;
    float drag_area_m2;
    float air_density_kgpm3;
    float time_step_s;
    float max_predict_time_s;
    float actuator_effectiveness;
    /* END AMBAR BENCH-GATED EXPANSION - DRAG APOGEE SETTINGS */
} AmbarApogeePredictorConfig_t;

typedef struct
{
    /* 0.0 fully retracted, 1.0 fully deployed, before actuator step mapping. */
    float deploy_fraction;

    /* Values used by telemetry to explain the command decision. */
    float predicted_apogee_m;
    float target_apogee_m;

    /* inhibit=true means deploy_fraction must be treated as zero by hardware. */
    bool inhibit;
    uint32_t inhibit_flags;
} AmbarAirbrakeCommand_t;

typedef struct
{
    AmbarEkfConfig_t estimator;
    AmbarFlightPhaseConfig_t phase;
    AmbarAirbrakeControllerConfig_t controller;
    AmbarApogeePredictorConfig_t apogee;
} AmbarFlightConfig_t;

typedef struct
{
    /* Single telemetry/control snapshot produced by AmbarFlight_GetOutput(). */
    AmbarNavigationEstimate_t estimate;
    AmbarEstimatorHealth_t health;
    AmbarAirbrakeCommand_t airbrake_command;
    AmbarFlightPhase_t phase;

    /*
     * Extra comparison values for telemetry and replay tuning.  The command uses
     * estimate.predicted_apogee_m, while these fields show what each predictor
     * route produced.
     */
    float ballistic_apogee_m;
    float drag_apogee_m;
    bool armed;
} AmbarFlightOutput_t;

/* Create default estimator, phase, and controller tuning values. */
AmbarFlightConfig_t AmbarFlight_DefaultConfig(void);

/* Initialize the singleton flight layer used by the embedded application. */
void AmbarFlight_Init(const AmbarFlightConfig_t *config);

/* Reset estimator/phase tracking when the board is stationary on the pad. */
void AmbarFlight_ResetOnPad(float timestamp_s);

/* Explicit arming gate controlled by radio commands or bench code. */
void AmbarFlight_SetArmed(bool armed);
bool AmbarFlight_IsArmed(void);

/* Feed sensor updates into the estimator and phase/controller logic. */
bool AmbarFlight_UpdateImu(float timestamp_s, float vertical_acceleration_mps2);
bool AmbarFlight_UpdateBarometer(float barometer_altitude_agl_m, float barometer_stddev_m);

/* Read the latest estimate, command, health, and phase as one stable snapshot. */
AmbarFlightOutput_t AmbarFlight_GetOutput(void);

/* Human-readable phase string for telemetry messages. */
const char *AmbarFlight_PhaseName(AmbarFlightPhase_t phase);

#ifdef __cplusplus
}
#endif

#endif /* AMBAR_FLIGHT_H */
