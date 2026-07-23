/*
 * AMBAR FLIGHT ESTIMATION AND DEPLOYMENT POLICY - PUBLIC INTERFACE
 *
 * Purpose and ownership
 *   Defines the phase vocabulary, inhibit bits, tuning blocks, requested
 *   airbrake command, and combined output snapshot above the vertical EKF.
 *   The implementation owns one embedded flight-computer instance; callers feed
 *   sensor updates and read copies of its latest decision state.
 *
 * Decision flow
 *   Sensor samples update the EKF and phase machine.  The selected apogee model
 *   predicts the coast result.  Target error is converted to a bounded 0..1
 *   deploy_fraction only after arming, phase, altitude, time, direction, and
 *   estimator-health gates pass.  See CODE_GUIDE.md [ARCH-4].  The actuator
 *   layer remains final authority over physical motion; this API is a request.
 *
 * Section map
 *   1. Flight phases, inhibit bits, and predictor mode
 *   2. Estimator/phase/controller/predictor configuration
 *   3. Command and output snapshots
 *   4. Lifecycle, arming, sensor-update, and read-only output API
 *
 * Safety and assumptions
 *   Coast is the only deployable phase, and every inhibit forces a zero request.
 *   Predictor and controller parameters must be replaced or validated using the
 *   actual vehicle and representative replay/HIL data.  The forward model is
 *   vertical-only; it is not an aerodynamic table or six-degree-of-freedom model.
 */

#ifndef AMBAR_FLIGHT_H
#define AMBAR_FLIGHT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ambar_ekf.h"

#include <stdbool.h>
#include <stdint.h>

/* ===================== PHASES, INHIBITS, AND PREDICTOR MODE ===================== */

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

/* The provisional M5 aerodynamic model is defined at these deployment points. */
#define AMBAR_DEPLOYMENT_CDA_POINT_COUNT 5U
#define AMBAR_PREDICTIVE_BISECTION_ITERATIONS 8U

typedef enum
{
    /* Retained as a deterministic fallback and comparison controller. */
    AMBAR_CONTROL_MODE_PROPORTIONAL = 0,

    /* Solve the deploy fraction from retracted/full authority predictions. */
    AMBAR_CONTROL_MODE_PREDICTIVE = 1
} AmbarAirbrakeControlMode_t;

typedef enum
{
    AMBAR_FLIGHT_CONFIG_VALID = 0U,
    AMBAR_FLIGHT_CONFIG_INVALID_TARGET = 1U << 0U,
    AMBAR_FLIGHT_CONFIG_INVALID_TOLERANCE = 1U << 1U,
    AMBAR_FLIGHT_CONFIG_INVALID_DEADBAND = 1U << 2U,
    AMBAR_FLIGHT_CONFIG_INVALID_HYSTERESIS = 1U << 3U,
    AMBAR_FLIGHT_CONFIG_INVALID_CONTROLLER_LIMIT = 1U << 4U,
    AMBAR_FLIGHT_CONFIG_INVALID_CONTROLLER_RATE = 1U << 5U,
    AMBAR_FLIGHT_CONFIG_INVALID_CONTROLLER_MODE = 1U << 6U,
    AMBAR_FLIGHT_CONFIG_INVALID_MASS = 1U << 7U,
    AMBAR_FLIGHT_CONFIG_INVALID_CDA = 1U << 8U,
    AMBAR_FLIGHT_CONFIG_INVALID_ATMOSPHERE = 1U << 9U,
    AMBAR_FLIGHT_CONFIG_INVALID_PREDICTOR_TIMING = 1U << 10U,
    AMBAR_FLIGHT_CONFIG_INVALID_ACTUATOR_MODEL = 1U << 11U,
    AMBAR_FLIGHT_CONFIG_INVALID_PREDICTOR_MODE = 1U << 12U,
    AMBAR_FLIGHT_CONFIG_INVALID_CALIBRATION = 1U << 13U
} AmbarFlightConfigValidationFlags_t;

/* ===================== FLIGHT CONFIGURATION ===================== */

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
    /* Desired apogee and the 100 ft mission acceptance band. */
    float target_apogee_m;

    /*
     * Legacy compatibility mirror of mission_tolerance_m.  Existing telemetry
     * configuration records still refer to this name; new configuration paths
     * must keep the two values equal.
     */
    float apogee_tolerance_m;
    float mission_tolerance_m;

    /* The controller acts outside this narrower 10 ft deadband. */
    float control_deadband_m;

    /* Suppress command chatter smaller than two percent of full travel. */
    float deployment_hysteresis_fraction;

    /* Predictive fraction solver cadence.  The provisional M5 value is 20 Hz. */
    float predictive_update_period_s;
    AmbarAirbrakeControlMode_t control_mode;

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
     * These values let bench/replay work switch between the old ballistic
     * predictor and a simple drag-aware coast estimate without changing the EKF
     * state.  They are deliberately plain floats so they can be saved in the
     * flash configuration record.
     */
    AmbarApogeePredictionMode_t mode;

    /*
     * Legacy configuration mirrors retained for the current flash/telemetry
     * record.  New writers must mirror coast_mass_kg, baseline_drag_area_m2,
     * and sea_level_air_density_kgpm3 into these three fields.
     */
    float vehicle_mass_kg;
    float drag_area_m2;
    float air_density_kgpm3;

    /* Versioned, provisional M5 coast model inputs. */
    uint32_t calibration_version;
    float coast_mass_kg;
    float baseline_drag_area_m2;
    float deployment_drag_area_m2[AMBAR_DEPLOYMENT_CDA_POINT_COUNT];
    float sea_level_air_density_kgpm3;
    float density_scale_height_m;
    float launch_site_elevation_m;
    float time_step_s;
    float max_predict_time_s;

    /* Command-to-motion delay and measured loaded stroke rates. */
    float actuator_delay_s;
    float actuator_open_rate_fraction_per_s;
    float actuator_close_rate_fraction_per_s;

    /* Gain used only by the proportional fallback/comparison controller. */
    float actuator_effectiveness;
} AmbarApogeePredictorConfig_t;

/* ===================== COMMAND AND OUTPUT SNAPSHOTS ===================== */

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
    float closed_predicted_apogee_m;
    float full_predicted_apogee_m;
    AmbarAirbrakeControlMode_t controller_mode_used;
    bool predictive_solution_valid;
    bool target_reachable;
    bool armed;
} AmbarFlightOutput_t;

/* ===================== PUBLIC API ===================== */

/* Create default estimator, phase, and controller tuning values. */
AmbarFlightConfig_t AmbarFlight_DefaultConfig(void);

/* Explicit opt-in profile for provisional M5 VARIABLE_HIL studies. */
AmbarFlightConfig_t AmbarFlight_M5VariableHilConfig(void);

/* Validate and transactionally apply only the control/predictor tuning blocks. */
bool AmbarFlight_ValidateControlConfig(
    const AmbarAirbrakeControllerConfig_t *controller,
    const AmbarApogeePredictorConfig_t *apogee,
    uint32_t *invalid_flags);
bool AmbarFlight_ApplyControlConfig(
    const AmbarAirbrakeControllerConfig_t *controller,
    const AmbarApogeePredictorConfig_t *apogee,
    uint32_t *invalid_flags);

/* Copy the active configuration without exposing mutable module storage. */
void AmbarFlight_GetConfigSnapshot(AmbarFlightConfig_t *config_out);

/*
 * Pure coast prediction helper used by calibration/replay tools.  Passing the
 * same current and target fraction evaluates a fixed-deployment trajectory.
 */
float AmbarFlight_PredictApogee(
    float altitude_m,
    float velocity_mps,
    float current_deploy_fraction,
    float target_deploy_fraction,
    const AmbarApogeePredictorConfig_t *config);

/*
 * Pure predictive-controller helpers used by the production command path and
 * host regression tests.  The solver brackets against the supplied closed/full
 * forecasts, performs exactly AMBAR_PREDICTIVE_BISECTION_ITERATIONS bounded
 * iterations when reachable, and reports endpoint saturation only when the
 * target lies outside that bracket.
 */
bool AmbarFlight_SolvePredictiveFraction(
    float altitude_m,
    float velocity_mps,
    float current_deploy_fraction,
    float closed_predicted_apogee_m,
    float full_predicted_apogee_m,
    const AmbarAirbrakeControllerConfig_t *controller,
    const AmbarApogeePredictorConfig_t *predictor,
    float *fraction_out,
    bool *authority_saturation_out);

/* Preserve the prior command for a sub-threshold non-saturated change. */
float AmbarFlight_ApplyDeploymentHysteresis(
    float requested_fraction,
    float previous_fraction,
    float hysteresis_fraction,
    bool authority_saturation);

/* Initialize the singleton flight layer used by the embedded application. */
void AmbarFlight_Init(const AmbarFlightConfig_t *config);

/* Reset estimator/phase tracking when the board is stationary on the pad. */
void AmbarFlight_ResetOnPad(float timestamp_s);

/* Explicit arming gate controlled by radio commands or bench code. */
void AmbarFlight_SetArmed(bool armed);
bool AmbarFlight_IsArmed(void);

/* Supply measured/host XACTUAL deployment; invalid selects the rate-model fallback. */
void AmbarFlight_SetActuatorFraction(float deploy_fraction, bool valid);

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
