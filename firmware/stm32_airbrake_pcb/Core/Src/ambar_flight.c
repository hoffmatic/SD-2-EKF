/*
 * AMBAR FLIGHT ESTIMATION AND DEPLOYMENT POLICY - IMPLEMENTATION
 *
 * Purpose
 *   Combines the vertical EKF, flight-phase state machine, forward apogee
 *   prediction, explicit arming, and airbrake command policy.  This module
 *   produces a requested deploy_fraction and inhibit explanation; it never
 *   enables the motor driver or writes a TMC5240 register.
 *
 * Data/control flow
 *   IMU updates propagate the EKF and advance PAD_IDLE -> BOOST -> COAST ->
 *   AIRBRAKE_ACTIVE -> RECOVERY.  Barometer updates correct the EKF at their
 *   own rate.  GetOutput selects ballistic or drag-aware apogee, evaluates all
 *   deployment gates, and returns one telemetry/control snapshot.  AmbarApp
 *   passes that request to AmbarActuator, which applies the independent hardware
 *   safety gates described in CODE_GUIDE.md [ARCH-4] and [ARCH-5].
 *
 * Section map
 *   1. Configuration constant and singleton runtime state
 *   2. Phase-machine helpers
 *   3. Ballistic and drag-aware apogee predictors
 *   4. Command policy and output assembly
 *   5. Defaults and lifecycle
 *   6. Sensor-update and snapshot API
 *
 * Safety and assumptions
 *   Deployment is authorized only while armed, healthy, ascending, sufficiently
 *   high/late, and in coast.  Any inhibit produces a zero request.  The M5
 *   predictor values are explicit provisional calibration inputs, not
 *   flight-qualified measurements.  The predictor is vertical-only and does
 *   not model attitude, wind, or six-degree-of-freedom motion.
 */

#include "ambar_flight.h"
#include "ambar_features.h"

#include <stddef.h>
#include <string.h>
#include <math.h>

/* ===================== CONFIGURATION AND MODULE STATE ===================== */

/*
 * A single radio/scheduler stall can make one IMU timestamp exceed the EKF
 * gate.  Keep that sample rejected and the command inhibited, but only latch
 * the phase machine into FAULT after a sustained IMU outage.
 */
#define AMBAR_PHASE_FAULT_CONSECUTIVE_IMU_REJECTIONS 10U

/* 3379 ft passive M5 reference, 3000 ft target, and provisional calibration. */
#define AMBAR_M5_CALIBRATION_VERSION 20260719UL
#define AMBAR_M5_COAST_MASS_KG 3.27328f
#define AMBAR_M5_BASELINE_CDA_M2 0.002596784f
#define AMBAR_M5_CDA_25_M2 0.004898250f
#define AMBAR_M5_CDA_50_M2 0.007199716f
#define AMBAR_M5_CDA_75_M2 0.009501182f
#define AMBAR_M5_CDA_100_M2 0.011802649f
#define AMBAR_M5_DENSITY_SCALE_HEIGHT_M 8500.0f
#define AMBAR_M5_LAUNCH_SITE_ELEVATION_M 7.0104f
#define AMBAR_M5_ACTUATOR_DELAY_S 0.10f
#define AMBAR_M5_OPEN_RATE_FRACTION_PER_S 0.864f
#define AMBAR_M5_CLOSE_RATE_FRACTION_PER_S 0.844f
#define AMBAR_M5_MINIMUM_BOOST_TIME_S 1.80f

typedef struct
{
    /* Combined estimator, phase machine, and controller runtime state. */
    AmbarFlightConfig_t config;
    AmbarEkf_t ekf;
    AmbarFlightPhase_t phase;

    /* Launch time is captured at liftoff and used for boost/coast inhibits. */
    float launch_timestamp_s;
    bool has_launch_timestamp;

    /* Latest sensor timestamp seen by the flight layer. */
    float latest_timestamp_s;
    uint32_t consecutive_imu_rejections;

    /*
     * Arming is intentionally separate from phase tracking.  The estimator can
     * detect launch and coast while the airbrake command remains inhibited until
     * an explicit ARM command is accepted.
     */
    bool armed;
    float last_ballistic_apogee_m;
    float last_drag_apogee_m;
    float last_closed_apogee_m;
    float last_full_apogee_m;
    float last_command_fraction;
    float estimated_deploy_fraction;
    float last_control_update_s;
    float next_control_update_s;
    float last_actuator_model_timestamp_s;
    bool has_control_update;
    bool has_actuator_model_timestamp;
    bool actuator_fraction_feedback_valid;
    bool predictive_solution_valid;
    bool target_reachable;
    AmbarAirbrakeControlMode_t controller_mode_used;
#if AMBAR_BUILD_IS_VARIABLE_HIL
    /* One causal controller snapshot is computed for each accepted sensor set. */
    AmbarFlightOutput_t cached_output;
    bool cached_output_valid;
    bool output_dirty;
#endif
} AmbarFlightComputer_t;

static AmbarFlightComputer_t s_flight;

/* ===================== PHASE-MACHINE HELPERS ===================== */

static float ambar_clamp(float value, float lower, float upper)
{
    /* Shared clamp for bounded deployment fractions and controller outputs. */
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

static bool ambar_float_in_range(float value, float lower, float upper)
{
    return isfinite(value) && value >= lower && value <= upper;
}

static void ambar_controller_runtime_reset(void)
{
    s_flight.last_closed_apogee_m = 0.0f;
    s_flight.last_full_apogee_m = 0.0f;
    s_flight.last_command_fraction = 0.0f;
    s_flight.estimated_deploy_fraction = 0.0f;
    s_flight.last_control_update_s = 0.0f;
    s_flight.next_control_update_s = 0.0f;
    s_flight.last_actuator_model_timestamp_s = s_flight.latest_timestamp_s;
    s_flight.has_control_update = false;
    s_flight.has_actuator_model_timestamp = false;
    s_flight.actuator_fraction_feedback_valid = false;
    s_flight.predictive_solution_valid = false;
    s_flight.target_reachable = false;
    s_flight.controller_mode_used = AMBAR_CONTROL_MODE_PROPORTIONAL;
}

static void ambar_phase_reset(void)
{
    /* Reset phase state whenever the pad reference/estimator is reset. */
    s_flight.phase = AMBAR_PHASE_PAD_IDLE;
    s_flight.launch_timestamp_s = 0.0f;
    s_flight.has_launch_timestamp = false;
    s_flight.consecutive_imu_rejections = 0U;
}

static void ambar_phase_update(float timestamp_s,
                               const AmbarNavigationEstimate_t *estimate,
                               float vertical_acceleration_mps2)
{
#if AMBAR_FEATURE_AUTO_FLIGHT_PHASES == 0
    /* Presentation mode: estimator runs, but no sensor transient can leave pad. */
    (void)timestamp_s;
    (void)estimate;
    (void)vertical_acceleration_mps2;
    s_flight.phase = AMBAR_PHASE_PAD_IDLE;
    s_flight.launch_timestamp_s = 0.0f;
    s_flight.has_launch_timestamp = false;
    return;
#endif

    if (estimate == NULL)
    {
        return;
    }

    if (!estimate->healthy)
    {
        s_flight.phase = AMBAR_PHASE_FAULT;
        return;
    }

    switch (s_flight.phase)
    {
    case AMBAR_PHASE_PAD_IDLE:
        /*
         * Liftoff can be seen by altitude or acceleration. Using both makes the
         * state machine less fragile during bench testing and early sensor work.
         */
        if (estimate->altitude_agl_m > s_flight.config.phase.liftoff_altitude_m
            || vertical_acceleration_mps2 > s_flight.config.phase.liftoff_acceleration_mps2)
        {
            s_flight.phase = AMBAR_PHASE_BOOST;
            s_flight.launch_timestamp_s = timestamp_s;
            s_flight.has_launch_timestamp = true;
        }
        break;

    case AMBAR_PHASE_BOOST:
        /*
         * Burnout/coast requires time since launch, low acceleration, and still
         * rising velocity. This avoids declaring coast during motor startup
         * transients.
         */
        if (s_flight.has_launch_timestamp
            && timestamp_s - s_flight.launch_timestamp_s >= s_flight.config.phase.minimum_boost_time_s
            && vertical_acceleration_mps2 < s_flight.config.phase.burnout_acceleration_mps2
            && estimate->vertical_velocity_mps > s_flight.config.phase.minimum_coast_velocity_mps)
        {
            s_flight.phase = AMBAR_PHASE_COAST;
        }
        break;

    case AMBAR_PHASE_COAST:
    case AMBAR_PHASE_AIRBRAKE_ACTIVE:
        if (estimate->vertical_velocity_mps <= s_flight.config.phase.recovery_descent_velocity_mps)
        {
            s_flight.phase = AMBAR_PHASE_RECOVERY;
        }
        break;

    case AMBAR_PHASE_RECOVERY:
    case AMBAR_PHASE_FAULT:
    default:
        break;
    }
}

/* ===================== APOGEE PREDICTORS ===================== */

static float ambar_ballistic_apogee(float altitude_m, float velocity_mps)
{
    /*
     * Keep the original no-drag estimate available for comparison.  It is an
     * upper-bound coast estimate and is useful while validating the drag model.
     */
    if (velocity_mps <= 0.0f)
    {
        return altitude_m;
    }

    return altitude_m
         + (velocity_mps * velocity_mps) / (2.0f * AMBAR_STANDARD_GRAVITY_MPS2);
}

static float ambar_cda_for_fraction(float deployment_fraction,
                                    const AmbarApogeePredictorConfig_t *config)
{
    const float bounded = ambar_clamp(deployment_fraction, 0.0f, 1.0f);
    const float scaled = bounded * (float)(AMBAR_DEPLOYMENT_CDA_POINT_COUNT - 1U);
    uint32_t lower_index = (uint32_t)scaled;

    if (lower_index >= AMBAR_DEPLOYMENT_CDA_POINT_COUNT - 1U)
    {
        return config->deployment_drag_area_m2[AMBAR_DEPLOYMENT_CDA_POINT_COUNT - 1U];
    }

    const float local_fraction = scaled - (float)lower_index;
    const float lower = config->deployment_drag_area_m2[lower_index];
    const float upper = config->deployment_drag_area_m2[lower_index + 1U];
    return lower + local_fraction * (upper - lower);
}

static float ambar_legacy_drag_apogee(float altitude_m,
                                      float velocity_mps,
                                      const AmbarApogeePredictorConfig_t *config)
{
    float mass_kg = config->vehicle_mass_kg;
    float drag_area_m2 = config->drag_area_m2;
    float air_density = config->air_density_kgpm3;
    float dt_s = config->time_step_s;
    float max_time_s = config->max_predict_time_s;

    if (!isfinite(mass_kg) || mass_kg < 0.1f)
    {
        mass_kg = 5.0f;
    }
    if (!isfinite(drag_area_m2) || drag_area_m2 < 0.0f)
    {
        drag_area_m2 = 0.0f;
    }
    if (!isfinite(air_density) || air_density <= 0.0f)
    {
        air_density = 1.225f;
    }
    if (!isfinite(dt_s) || dt_s < 0.005f || dt_s > 0.100f)
    {
        dt_s = 0.02f;
    }
    if (!isfinite(max_time_s) || max_time_s < 1.0f || max_time_s > 60.0f)
    {
        max_time_s = 30.0f;
    }

    const float drag_k = 0.5f * air_density * drag_area_m2 / mass_kg;
    float altitude = altitude_m;
    float velocity = velocity_mps;
    for (float elapsed_s = 0.0f;
         elapsed_s < max_time_s && velocity > 0.0f;
         elapsed_s += dt_s)
    {
        const float acceleration = -AMBAR_STANDARD_GRAVITY_MPS2
                                 - drag_k * velocity * velocity;
        altitude += velocity * dt_s
                  + 0.5f * acceleration * dt_s * dt_s;
        velocity += acceleration * dt_s;
    }

    if (!isfinite(altitude) || altitude < altitude_m)
    {
        return altitude_m;
    }
    return altitude;
}

float AmbarFlight_PredictApogee(float altitude_m,
                                float velocity_mps,
                                float current_deploy_fraction,
                                float target_deploy_fraction,
                                const AmbarApogeePredictorConfig_t *config)
{
    /*
     * Vertical M5 coast model.  Each step uses altitude-dependent exponential
     * density and total CdA interpolated between the measured deployment
     * stations.  A command first waits the configured delay, then moves at the
     * measured opening/closing rate.  Equal current/target fractions are the
     * fixed-deployment calibration path.
     */
    if (config == NULL || !isfinite(altitude_m) || !isfinite(velocity_mps))
    {
        return 0.0f;
    }
    if (velocity_mps <= 0.0f)
    {
        return altitude_m;
    }

    /* Version zero is the unchanged Normal-build predictor contract. */
    if (config->calibration_version == 0U)
    {
        return ambar_legacy_drag_apogee(altitude_m, velocity_mps, config);
    }

    const float dt_s = config->time_step_s;
    const float max_time_s = config->max_predict_time_s;
    if (!isfinite(current_deploy_fraction) || !isfinite(target_deploy_fraction))
    {
        return ambar_ballistic_apogee(altitude_m, velocity_mps);
    }
    float altitude = altitude_m;
    float velocity = velocity_mps;
    float deployment = ambar_clamp(current_deploy_fraction, 0.0f, 1.0f);
    const float target = ambar_clamp(target_deploy_fraction, 0.0f, 1.0f);

    if (!ambar_float_in_range(config->coast_mass_kg, 0.1f, 100.0f)
        || !ambar_float_in_range(config->sea_level_air_density_kgpm3, 0.1f, 2.0f)
        || !ambar_float_in_range(config->density_scale_height_m, 1000.0f, 20000.0f)
        || !ambar_float_in_range(config->launch_site_elevation_m, -500.0f, 10000.0f)
        || !ambar_float_in_range(dt_s, 0.001f, 0.100f)
        || !ambar_float_in_range(max_time_s, 1.0f, 120.0f)
        || !ambar_float_in_range(config->actuator_delay_s, 0.0f, 5.0f)
        || !ambar_float_in_range(config->actuator_open_rate_fraction_per_s,
                                 0.01f, 10.0f)
        || !ambar_float_in_range(config->actuator_close_rate_fraction_per_s,
                                 0.01f, 10.0f))
    {
        return ambar_ballistic_apogee(altitude_m, velocity_mps);
    }
    for (uint32_t index = 0U;
         index < AMBAR_DEPLOYMENT_CDA_POINT_COUNT;
         ++index)
    {
        if (!ambar_float_in_range(config->deployment_drag_area_m2[index],
                                  0.000001f, 1.0f))
        {
            return ambar_ballistic_apogee(altitude_m, velocity_mps);
        }
    }

    /*
     * Density changes only by a few ten-thousandths per 20 ms coast step.
     * Evaluate expf once, then advance the exact exponential ratio with its
     * second-order series. This removes hundreds of expensive transcendental
     * calls per forecast while keeping the relative truncation error negligible
     * at the validated dt/velocity/scale-height bounds.
     */
    float density = config->sea_level_air_density_kgpm3
                  * expf(-(config->launch_site_elevation_m + altitude)
                         / config->density_scale_height_m);

    for (float elapsed_s = 0.0f;
         elapsed_s < max_time_s && velocity > 0.0f;
         elapsed_s += dt_s)
    {
        if (elapsed_s + 1.0e-6f >= config->actuator_delay_s)
        {
            const float rate = target >= deployment
                             ? config->actuator_open_rate_fraction_per_s
                             : config->actuator_close_rate_fraction_per_s;
            const float maximum_change = rate * dt_s;
            deployment += ambar_clamp(target - deployment,
                                      -maximum_change,
                                      maximum_change);
        }

        const float drag_area_m2 = ambar_cda_for_fraction(deployment, config);
        const float drag_k = 0.5f * density * drag_area_m2
                           / config->coast_mass_kg;
        const float acceleration = -AMBAR_STANDARD_GRAVITY_MPS2
                                 - drag_k * velocity * velocity;
        const float next_velocity = velocity + acceleration * dt_s;

        if (next_velocity <= 0.0f)
        {
            const float time_to_apogee_s = ambar_clamp(
                velocity / -acceleration, 0.0f, dt_s);
            altitude += velocity * time_to_apogee_s
                      + 0.5f * acceleration
                      * time_to_apogee_s * time_to_apogee_s;
            velocity = 0.0f;
            break;
        }

        const float altitude_change_m = velocity * dt_s
                                      + 0.5f * acceleration * dt_s * dt_s;
        altitude += altitude_change_m;
        const float density_exponent = altitude_change_m
                                     / config->density_scale_height_m;
        density *= 1.0f - density_exponent
                 + 0.5f * density_exponent * density_exponent;
        velocity = next_velocity;
    }

    if (!isfinite(altitude) || altitude < altitude_m)
    {
        return altitude_m;
    }
    return altitude;
}

static void ambar_update_actuator_estimate(void)
{
    if (s_flight.actuator_fraction_feedback_valid)
    {
        s_flight.last_actuator_model_timestamp_s = s_flight.latest_timestamp_s;
        s_flight.has_actuator_model_timestamp = true;
        return;
    }

    if (!s_flight.has_actuator_model_timestamp)
    {
        s_flight.last_actuator_model_timestamp_s = s_flight.latest_timestamp_s;
        s_flight.has_actuator_model_timestamp = true;
        return;
    }

    float elapsed_s = s_flight.latest_timestamp_s
                    - s_flight.last_actuator_model_timestamp_s;
    if (!isfinite(elapsed_s) || elapsed_s <= 0.0f)
    {
        return;
    }
    elapsed_s = ambar_clamp(elapsed_s, 0.0f, 0.25f);

    const float target = s_flight.last_command_fraction;
    const float rate = target >= s_flight.estimated_deploy_fraction
                     ? s_flight.config.apogee.actuator_open_rate_fraction_per_s
                     : s_flight.config.apogee.actuator_close_rate_fraction_per_s;
    const float maximum_change = rate * elapsed_s;
    s_flight.estimated_deploy_fraction += ambar_clamp(
        target - s_flight.estimated_deploy_fraction,
        -maximum_change,
        maximum_change);
    s_flight.estimated_deploy_fraction = ambar_clamp(
        s_flight.estimated_deploy_fraction, 0.0f, 1.0f);
    s_flight.last_actuator_model_timestamp_s = s_flight.latest_timestamp_s;
}

static float ambar_selected_apogee(const AmbarNavigationEstimate_t *estimate)
{
    if (estimate == NULL)
    {
        return 0.0f;
    }

    ambar_update_actuator_estimate();
    s_flight.last_ballistic_apogee_m =
        ambar_ballistic_apogee(estimate->altitude_agl_m,
                               estimate->vertical_velocity_mps);
#if AMBAR_BUILD_IS_VARIABLE_HIL
    s_flight.last_closed_apogee_m = AmbarFlight_PredictApogee(
        estimate->altitude_agl_m,
        estimate->vertical_velocity_mps,
        s_flight.estimated_deploy_fraction,
        0.0f,
        &s_flight.config.apogee);
    if (fabsf(s_flight.estimated_deploy_fraction) <= 1.0e-6f)
    {
        /* At HOME the current-deployment and closed predictions are identical. */
        s_flight.last_drag_apogee_m = s_flight.last_closed_apogee_m;
    }
    else
    {
        s_flight.last_drag_apogee_m = AmbarFlight_PredictApogee(
            estimate->altitude_agl_m,
            estimate->vertical_velocity_mps,
            s_flight.estimated_deploy_fraction,
            s_flight.estimated_deploy_fraction,
            &s_flight.config.apogee);
    }
#else
    s_flight.last_drag_apogee_m = AmbarFlight_PredictApogee(
        estimate->altitude_agl_m,
        estimate->vertical_velocity_mps,
        s_flight.estimated_deploy_fraction,
        s_flight.estimated_deploy_fraction,
        &s_flight.config.apogee);
    s_flight.last_closed_apogee_m = AmbarFlight_PredictApogee(
        estimate->altitude_agl_m,
        estimate->vertical_velocity_mps,
        s_flight.estimated_deploy_fraction,
        0.0f,
        &s_flight.config.apogee);
#endif
    s_flight.last_full_apogee_m = AmbarFlight_PredictApogee(
        estimate->altitude_agl_m,
        estimate->vertical_velocity_mps,
        s_flight.estimated_deploy_fraction,
        s_flight.config.controller.maximum_deploy_fraction,
        &s_flight.config.apogee);
    s_flight.target_reachable =
        s_flight.last_closed_apogee_m >= s_flight.config.controller.target_apogee_m
        && s_flight.last_full_apogee_m <= s_flight.config.controller.target_apogee_m;

    if (s_flight.config.apogee.mode == AMBAR_APOGEE_MODE_DRAG)
    {
        /* Retracted apogee is the relevant no-action forecast for the gate. */
        return s_flight.last_closed_apogee_m;
    }

    return s_flight.last_ballistic_apogee_m;
}

/* ===================== COMMAND POLICY AND OUTPUT ASSEMBLY ===================== */

static float ambar_proportional_fraction(float apogee_error_m)
{
    float effectiveness = s_flight.config.apogee.actuator_effectiveness;
    if (!isfinite(effectiveness) || effectiveness <= 0.0f)
    {
        effectiveness = 1.0f;
    }

    return ambar_clamp(
        (apogee_error_m / s_flight.config.controller.full_deployment_error_m)
        * effectiveness,
        0.0f,
        s_flight.config.controller.maximum_deploy_fraction);
}

bool AmbarFlight_SolvePredictiveFraction(
    float altitude_m,
    float velocity_mps,
    float current_deploy_fraction,
    float closed_predicted_apogee_m,
    float full_predicted_apogee_m,
    const AmbarAirbrakeControllerConfig_t *controller,
    const AmbarApogeePredictorConfig_t *predictor,
    float *fraction_out,
    bool *authority_saturation_out)
{
    if (controller == NULL || predictor == NULL || fraction_out == NULL
        || authority_saturation_out == NULL
        || predictor->mode != AMBAR_APOGEE_MODE_DRAG
        || !isfinite(altitude_m)
        || !isfinite(velocity_mps)
        || !isfinite(current_deploy_fraction)
        || !isfinite(closed_predicted_apogee_m)
        || !isfinite(full_predicted_apogee_m)
        || closed_predicted_apogee_m + 0.01f < full_predicted_apogee_m)
    {
        return false;
    }

    const float target_apogee_m = controller->target_apogee_m;
    const float maximum_fraction = controller->maximum_deploy_fraction;
    *authority_saturation_out = false;

    /* Saturation is allowed only when the target lies outside current authority. */
    if (target_apogee_m >= closed_predicted_apogee_m)
    {
        *fraction_out = 0.0f;
        *authority_saturation_out = true;
        return true;
    }
    if (target_apogee_m <= full_predicted_apogee_m)
    {
        *fraction_out = maximum_fraction;
        *authority_saturation_out = true;
        return true;
    }

    float lower_fraction = 0.0f;
    float upper_fraction = maximum_fraction;
    for (uint32_t iteration = 0U;
         iteration < AMBAR_PREDICTIVE_BISECTION_ITERATIONS;
         ++iteration)
    {
        const float middle_fraction =
            0.5f * (lower_fraction + upper_fraction);
        const float middle_apogee_m = AmbarFlight_PredictApogee(
            altitude_m,
            velocity_mps,
            current_deploy_fraction,
            middle_fraction,
            predictor);

        if (!isfinite(middle_apogee_m))
        {
            return false;
        }
        if (middle_apogee_m > target_apogee_m)
        {
            lower_fraction = middle_fraction;
        }
        else
        {
            upper_fraction = middle_fraction;
        }
    }

    *fraction_out = 0.5f * (lower_fraction + upper_fraction);
    return true;
}

float AmbarFlight_ApplyDeploymentHysteresis(
    float requested_fraction,
    float previous_fraction,
    float hysteresis_fraction,
    bool authority_saturation)
{
    if (!isfinite(requested_fraction)
        || !isfinite(previous_fraction)
        || !isfinite(hysteresis_fraction)
        || hysteresis_fraction < 0.0f)
    {
        return requested_fraction;
    }
    if (!authority_saturation
        && fabsf(requested_fraction - previous_fraction) < hysteresis_fraction)
    {
        return previous_fraction;
    }
    return requested_fraction;
}

static bool ambar_predictive_fraction(const AmbarNavigationEstimate_t *estimate,
                                      float *fraction_out,
                                      bool *authority_saturation_out)
{
    if (estimate == NULL)
    {
        return false;
    }
    return AmbarFlight_SolvePredictiveFraction(
        estimate->altitude_agl_m,
        estimate->vertical_velocity_mps,
        s_flight.estimated_deploy_fraction,
        s_flight.last_closed_apogee_m,
        s_flight.last_full_apogee_m,
        &s_flight.config.controller,
        &s_flight.config.apogee,
        fraction_out,
        authority_saturation_out);
}

static AmbarAirbrakeCommand_t ambar_compute_command(const AmbarNavigationEstimate_t *estimate)
{
    /*
     * Convert a predicted-apogee error into a bounded deployment request, then
     * explain every reason that request must be inhibited.
     */
    AmbarAirbrakeCommand_t command;
    memset(&command, 0, sizeof(command));

    command.target_apogee_m = s_flight.config.controller.target_apogee_m;
    command.inhibit = true;
    command.inhibit_flags = AMBAR_INHIBIT_NONE;

    if (estimate == NULL)
    {
        command.inhibit_flags = AMBAR_INHIBIT_ESTIMATOR_UNHEALTHY;
        return command;
    }

    command.predicted_apogee_m = estimate->predicted_apogee_m;

    if (!s_flight.armed)
    {
        command.inhibit_flags |= AMBAR_INHIBIT_NOT_ARMED;
    }

    if (!estimate->healthy)
    {
        command.inhibit_flags |= AMBAR_INHIBIT_ESTIMATOR_UNHEALTHY;
    }

    if (s_flight.phase != AMBAR_PHASE_COAST
        && s_flight.phase != AMBAR_PHASE_AIRBRAKE_ACTIVE)
    {
        command.inhibit_flags |= AMBAR_INHIBIT_NOT_IN_COAST;
    }

    if (estimate->altitude_agl_m < s_flight.config.controller.minimum_deploy_altitude_m)
    {
        command.inhibit_flags |= AMBAR_INHIBIT_BELOW_MINIMUM_ALTITUDE;
    }

    if (s_flight.latest_timestamp_s - s_flight.launch_timestamp_s
        < s_flight.config.controller.minimum_flight_time_s)
    {
        /*
         * Time inhibit is intentionally independent of phase.  It prevents an
         * early state-machine mistake from commanding deployment right after launch.
         */
        command.inhibit_flags |= AMBAR_INHIBIT_BEFORE_MINIMUM_FLIGHT_TIME;
    }

    if (estimate->vertical_velocity_mps <= 0.0f)
    {
        command.inhibit_flags |= AMBAR_INHIBIT_DESCENDING;
    }

    const float apogee_error_m =
        estimate->predicted_apogee_m - s_flight.config.controller.target_apogee_m;
    if (apogee_error_m <= s_flight.config.controller.control_deadband_m)
    {
        command.inhibit_flags |= AMBAR_INHIBIT_APOGEE_ON_TARGET;
    }

    command.inhibit = command.inhibit_flags != AMBAR_INHIBIT_NONE;
    if (command.inhibit)
    {
        /*
         * Any inhibit flag means safe/retracted. The actuator module receives
         * this command later and applies an additional hardware safety gate.
         */
        command.deploy_fraction = 0.0f;
        s_flight.last_command_fraction = 0.0f;
        s_flight.has_control_update = false;
        s_flight.predictive_solution_valid = false;
        s_flight.controller_mode_used = AMBAR_CONTROL_MODE_PROPORTIONAL;
        return command;
    }

    if (s_flight.config.controller.control_mode == AMBAR_CONTROL_MODE_PREDICTIVE
        && s_flight.has_control_update
        && s_flight.latest_timestamp_s + 1.0e-6f
           < s_flight.next_control_update_s)
    {
        command.deploy_fraction = s_flight.last_command_fraction;
        command.inhibit = false;
        return command;
    }

    float requested_fraction = 0.0f;
    bool authority_saturation = false;
    bool predictive_valid = false;
    if (s_flight.config.controller.control_mode == AMBAR_CONTROL_MODE_PREDICTIVE)
    {
        predictive_valid = ambar_predictive_fraction(
            estimate, &requested_fraction, &authority_saturation);
    }

    if (predictive_valid)
    {
        s_flight.controller_mode_used = AMBAR_CONTROL_MODE_PREDICTIVE;
        requested_fraction = AmbarFlight_ApplyDeploymentHysteresis(
            requested_fraction,
            s_flight.last_command_fraction,
            s_flight.config.controller.deployment_hysteresis_fraction,
            authority_saturation);
    }
    else
    {
        /* A bad/non-monotonic predictor falls back to the evaluated P controller. */
        requested_fraction = ambar_proportional_fraction(apogee_error_m);
        s_flight.controller_mode_used = AMBAR_CONTROL_MODE_PROPORTIONAL;
    }

    command.deploy_fraction = ambar_clamp(
        requested_fraction,
        0.0f,
        s_flight.config.controller.maximum_deploy_fraction);
    s_flight.last_command_fraction = command.deploy_fraction;
    s_flight.last_control_update_s = s_flight.latest_timestamp_s;
    if (!s_flight.has_control_update)
    {
        s_flight.next_control_update_s = s_flight.latest_timestamp_s
                                       + s_flight.config.controller.predictive_update_period_s;
    }
    else
    {
        do
        {
            s_flight.next_control_update_s +=
                s_flight.config.controller.predictive_update_period_s;
        }
        while (s_flight.next_control_update_s
               <= s_flight.latest_timestamp_s + 1.0e-6f);
    }
    s_flight.has_control_update = true;
    s_flight.predictive_solution_valid = predictive_valid;

    return command;
}

static AmbarFlightOutput_t ambar_build_output(void)
{
    /* Assemble one consistent snapshot for telemetry and actuator safety logic. */
    AmbarFlightOutput_t output;
    memset(&output, 0, sizeof(output));

    output.estimate = AmbarEkf_GetEstimate(&s_flight.ekf);
    output.estimate.predicted_apogee_m = ambar_selected_apogee(&output.estimate);
    output.health = AmbarEkf_GetHealth(&s_flight.ekf);
    output.phase = s_flight.phase;
    output.airbrake_command = ambar_compute_command(&output.estimate);
    output.ballistic_apogee_m = s_flight.last_ballistic_apogee_m;
    output.drag_apogee_m = s_flight.last_drag_apogee_m;
    output.closed_predicted_apogee_m = s_flight.last_closed_apogee_m;
    output.full_predicted_apogee_m = s_flight.last_full_apogee_m;
    output.controller_mode_used = s_flight.controller_mode_used;
    output.predictive_solution_valid = s_flight.predictive_solution_valid;
    output.target_reachable = s_flight.target_reachable;
    output.armed = s_flight.armed;

    return output;
}

/* ===================== DEFAULTS AND LIFECYCLE ===================== */

bool AmbarFlight_ValidateControlConfig(
    const AmbarAirbrakeControllerConfig_t *controller,
    const AmbarApogeePredictorConfig_t *apogee,
    uint32_t *invalid_flags)
{
    uint32_t flags = AMBAR_FLIGHT_CONFIG_VALID;

    if (controller == NULL || apogee == NULL)
    {
        flags = AMBAR_FLIGHT_CONFIG_INVALID_TARGET
              | AMBAR_FLIGHT_CONFIG_INVALID_MASS;
    }
    else
    {
        if (!ambar_float_in_range(controller->target_apogee_m, 100.0f, 10000.0f))
        {
            flags |= AMBAR_FLIGHT_CONFIG_INVALID_TARGET;
        }
        if (!ambar_float_in_range(controller->apogee_tolerance_m, 0.1f, 1000.0f)
            || !ambar_float_in_range(controller->mission_tolerance_m, 0.1f, 1000.0f))
        {
            flags |= AMBAR_FLIGHT_CONFIG_INVALID_TOLERANCE;
        }
        if (!ambar_float_in_range(controller->control_deadband_m, 0.0f, 1000.0f)
            || controller->control_deadband_m > controller->mission_tolerance_m)
        {
            flags |= AMBAR_FLIGHT_CONFIG_INVALID_DEADBAND;
        }
        if (!ambar_float_in_range(controller->deployment_hysteresis_fraction,
                                  0.0f, 0.25f))
        {
            flags |= AMBAR_FLIGHT_CONFIG_INVALID_HYSTERESIS;
        }
        if (!ambar_float_in_range(controller->full_deployment_error_m, 0.1f, 3000.0f)
            || !ambar_float_in_range(controller->maximum_deploy_fraction, 0.01f, 1.0f)
            || !ambar_float_in_range(controller->minimum_deploy_altitude_m, 0.0f, 5000.0f)
            || !ambar_float_in_range(controller->minimum_flight_time_s, 0.0f, 60.0f))
        {
            flags |= AMBAR_FLIGHT_CONFIG_INVALID_CONTROLLER_LIMIT;
        }
        if (!ambar_float_in_range(controller->predictive_update_period_s, 0.01f, 1.0f))
        {
            flags |= AMBAR_FLIGHT_CONFIG_INVALID_CONTROLLER_RATE;
        }
        if (controller->control_mode != AMBAR_CONTROL_MODE_PROPORTIONAL
            && controller->control_mode != AMBAR_CONTROL_MODE_PREDICTIVE)
        {
            flags |= AMBAR_FLIGHT_CONFIG_INVALID_CONTROLLER_MODE;
        }

        if (!ambar_float_in_range(apogee->coast_mass_kg, 0.1f, 100.0f)
            || !ambar_float_in_range(apogee->vehicle_mass_kg, 0.1f, 100.0f))
        {
            flags |= AMBAR_FLIGHT_CONFIG_INVALID_MASS;
        }
        if (!ambar_float_in_range(apogee->baseline_drag_area_m2, 0.000001f, 1.0f)
            || !ambar_float_in_range(apogee->drag_area_m2, 0.0f, 1.0f))
        {
            flags |= AMBAR_FLIGHT_CONFIG_INVALID_CDA;
        }
        else
        {
            float previous_cda = 0.0f;
            for (uint32_t index = 0U;
                 index < AMBAR_DEPLOYMENT_CDA_POINT_COUNT;
                 ++index)
            {
                const float cda = apogee->deployment_drag_area_m2[index];
                if (!ambar_float_in_range(cda, 0.000001f, 1.0f)
                    || (index > 0U && cda < previous_cda))
                {
                    flags |= AMBAR_FLIGHT_CONFIG_INVALID_CDA;
                }
                previous_cda = cda;
            }
            const float baseline_tolerance =
                0.05f * apogee->baseline_drag_area_m2 + 0.000001f;
            if (fabsf(apogee->deployment_drag_area_m2[0]
                      - apogee->baseline_drag_area_m2) > baseline_tolerance)
            {
                flags |= AMBAR_FLIGHT_CONFIG_INVALID_CDA;
            }
        }
        if (!ambar_float_in_range(apogee->sea_level_air_density_kgpm3, 0.1f, 2.0f)
            || !ambar_float_in_range(apogee->air_density_kgpm3, 0.1f, 2.0f)
            || !ambar_float_in_range(apogee->density_scale_height_m, 1000.0f, 20000.0f)
            || !ambar_float_in_range(apogee->launch_site_elevation_m, -500.0f, 10000.0f))
        {
            flags |= AMBAR_FLIGHT_CONFIG_INVALID_ATMOSPHERE;
        }
        if (!ambar_float_in_range(apogee->time_step_s, 0.001f, 0.100f)
            || !ambar_float_in_range(apogee->max_predict_time_s, 1.0f, 120.0f))
        {
            flags |= AMBAR_FLIGHT_CONFIG_INVALID_PREDICTOR_TIMING;
        }
        if (!ambar_float_in_range(apogee->actuator_delay_s, 0.0f, 5.0f)
            || !ambar_float_in_range(apogee->actuator_open_rate_fraction_per_s,
                                     0.01f, 10.0f)
            || !ambar_float_in_range(apogee->actuator_close_rate_fraction_per_s,
                                     0.01f, 10.0f)
            || !ambar_float_in_range(apogee->actuator_effectiveness, 0.01f, 10.0f))
        {
            flags |= AMBAR_FLIGHT_CONFIG_INVALID_ACTUATOR_MODEL;
        }
        if (apogee->mode != AMBAR_APOGEE_MODE_BALLISTIC
            && apogee->mode != AMBAR_APOGEE_MODE_DRAG)
        {
            flags |= AMBAR_FLIGHT_CONFIG_INVALID_PREDICTOR_MODE;
        }
        if (controller->control_mode == AMBAR_CONTROL_MODE_PREDICTIVE
            && apogee->calibration_version == 0U)
        {
            flags |= AMBAR_FLIGHT_CONFIG_INVALID_CALIBRATION;
        }
    }

    if (invalid_flags != NULL)
    {
        *invalid_flags = flags;
    }
    return flags == AMBAR_FLIGHT_CONFIG_VALID;
}

AmbarFlightConfig_t AmbarFlight_DefaultConfig(void)
{
    /*
     * Conservative first-pass defaults.  They are easy to override in a test build
     * by passing a custom AmbarFlightConfig_t to AmbarFlight_Init().
     */
    AmbarFlightConfig_t config;
    memset(&config, 0, sizeof(config));

    config.estimator = AmbarEkf_DefaultConfig();

    config.phase.liftoff_altitude_m = 3.0f;
    config.phase.liftoff_acceleration_mps2 = 15.0f;
    config.phase.minimum_boost_time_s = 0.50f;
    config.phase.burnout_acceleration_mps2 = -2.0f;
    config.phase.minimum_coast_velocity_mps = 20.0f;
    config.phase.recovery_descent_velocity_mps = -5.0f;

    config.controller.target_apogee_m = AMBAR_TARGET_APOGEE_M;
    config.controller.apogee_tolerance_m = AMBAR_TARGET_TOLERANCE_M;
    config.controller.mission_tolerance_m = AMBAR_TARGET_TOLERANCE_M;
    config.controller.control_deadband_m = AMBAR_TARGET_TOLERANCE_M;
    config.controller.deployment_hysteresis_fraction = 0.0f;
    config.controller.predictive_update_period_s = 0.05f;
    config.controller.control_mode = AMBAR_CONTROL_MODE_PROPORTIONAL;
    config.controller.full_deployment_error_m = 250.0f * AMBAR_FEET_TO_METERS;
    config.controller.maximum_deploy_fraction = 1.0f;
    config.controller.minimum_deploy_altitude_m = 200.0f * AMBAR_FEET_TO_METERS;
    config.controller.minimum_flight_time_s = 1.0f;

    /*
     * Preserve the pre-VARIABLE_HIL Normal-build predictor and proportional
     * controller exactly.  The M5 calibration is an explicit opt-in factory
     * below; rebuilding Normal must not change its control law or aero values.
     */
    config.apogee.mode = AMBAR_APOGEE_MODE_DRAG;
    config.apogee.vehicle_mass_kg = 5.0f;
    config.apogee.drag_area_m2 = 0.012f;
    config.apogee.air_density_kgpm3 = 1.225f;
    config.apogee.calibration_version = 0U;
    config.apogee.coast_mass_kg = 5.0f;
    config.apogee.baseline_drag_area_m2 = 0.012f;
    config.apogee.deployment_drag_area_m2[0] = 0.012f;
    config.apogee.deployment_drag_area_m2[1] = 0.012f;
    config.apogee.deployment_drag_area_m2[2] = 0.012f;
    config.apogee.deployment_drag_area_m2[3] = 0.012f;
    config.apogee.deployment_drag_area_m2[4] = 0.012f;
    config.apogee.sea_level_air_density_kgpm3 = 1.225f;
    config.apogee.density_scale_height_m = AMBAR_M5_DENSITY_SCALE_HEIGHT_M;
    config.apogee.launch_site_elevation_m = AMBAR_M5_LAUNCH_SITE_ELEVATION_M;
    config.apogee.time_step_s = 0.02f;
    config.apogee.max_predict_time_s = 30.0f;
    config.apogee.actuator_delay_s = 0.0f;
    config.apogee.actuator_open_rate_fraction_per_s = 1.0f;
    config.apogee.actuator_close_rate_fraction_per_s = 1.0f;
    config.apogee.actuator_effectiveness = 1.0f;

    return config;
}

AmbarFlightConfig_t AmbarFlight_M5VariableHilConfig(void)
{
    /*
     * Explicit provisional M5 opt-in.  Coast mass is the unchanged 3.0 kg
     * RocketPy dry-mass placeholder plus the unchanged 0.27328 kg J420R casing.
     * Baseline CdA uses the RocketPy passive fit Cd=0.526464844 that reproduces
     * 3379 ft without changing mass or motor. Full-deployment incremental CdA
     * comes from 219.286 N at 197.206 m/s and rho=1.225 kg/m^3; the 25/50/75
     * percent stations are provisional linear load-point interpolation.
     */
    AmbarFlightConfig_t config = AmbarFlight_DefaultConfig();

    /* 1.69 s screening maximum J420R burn plus 0.10 s enable margin. */
    config.phase.minimum_boost_time_s = AMBAR_M5_MINIMUM_BOOST_TIME_S;
    config.controller.target_apogee_m = AMBAR_TARGET_APOGEE_M;
    config.controller.apogee_tolerance_m = AMBAR_TARGET_TOLERANCE_M;
    config.controller.mission_tolerance_m = AMBAR_TARGET_TOLERANCE_M;
    config.controller.control_deadband_m = 10.0f * AMBAR_FEET_TO_METERS;
    config.controller.deployment_hysteresis_fraction = 0.02f;
    config.controller.predictive_update_period_s = 0.05f;
    config.controller.control_mode = AMBAR_CONTROL_MODE_PREDICTIVE;

    config.apogee.calibration_version = AMBAR_M5_CALIBRATION_VERSION;
    config.apogee.vehicle_mass_kg = AMBAR_M5_COAST_MASS_KG;
    config.apogee.drag_area_m2 = AMBAR_M5_BASELINE_CDA_M2;
    config.apogee.air_density_kgpm3 = 1.225f;
    config.apogee.coast_mass_kg = AMBAR_M5_COAST_MASS_KG;
    config.apogee.baseline_drag_area_m2 = AMBAR_M5_BASELINE_CDA_M2;
    config.apogee.deployment_drag_area_m2[0] = AMBAR_M5_BASELINE_CDA_M2;
    config.apogee.deployment_drag_area_m2[1] = AMBAR_M5_CDA_25_M2;
    config.apogee.deployment_drag_area_m2[2] = AMBAR_M5_CDA_50_M2;
    config.apogee.deployment_drag_area_m2[3] = AMBAR_M5_CDA_75_M2;
    config.apogee.deployment_drag_area_m2[4] = AMBAR_M5_CDA_100_M2;
    config.apogee.sea_level_air_density_kgpm3 = 1.225f;
    config.apogee.density_scale_height_m = AMBAR_M5_DENSITY_SCALE_HEIGHT_M;
    config.apogee.launch_site_elevation_m = AMBAR_M5_LAUNCH_SITE_ELEVATION_M;
    config.apogee.actuator_delay_s = AMBAR_M5_ACTUATOR_DELAY_S;
    config.apogee.actuator_open_rate_fraction_per_s =
        AMBAR_M5_OPEN_RATE_FRACTION_PER_S;
    config.apogee.actuator_close_rate_fraction_per_s =
        AMBAR_M5_CLOSE_RATE_FRACTION_PER_S;
    return config;
}

void AmbarFlight_Init(const AmbarFlightConfig_t *config)
{
    /* Initialize singleton flight state.  The embedded app uses one flight computer. */
    const AmbarFlightConfig_t defaults = AmbarFlight_DefaultConfig();
    memset(&s_flight, 0, sizeof(s_flight));
    s_flight.config = (config != NULL) ? *config : defaults;
    if (!AmbarFlight_ValidateControlConfig(&s_flight.config.controller,
                                           &s_flight.config.apogee,
                                           NULL))
    {
        /* Preserve estimator/phase overrides, but never run invalid control tuning. */
        s_flight.config.controller = defaults.controller;
        s_flight.config.apogee = defaults.apogee;
    }
    AmbarEkf_Init(&s_flight.ekf, &s_flight.config.estimator);
    ambar_phase_reset();
    ambar_controller_runtime_reset();
    s_flight.armed = false;
#if AMBAR_BUILD_IS_VARIABLE_HIL
    s_flight.cached_output_valid = false;
    s_flight.output_dirty = true;
#endif
}

bool AmbarFlight_ApplyControlConfig(
    const AmbarAirbrakeControllerConfig_t *controller,
    const AmbarApogeePredictorConfig_t *apogee,
    uint32_t *invalid_flags)
{
    if (!AmbarFlight_ValidateControlConfig(controller, apogee, invalid_flags))
    {
        return false;
    }

    /* Validate first, then commit both tuning blocks as one transaction. */
    AmbarAirbrakeControllerConfig_t controller_candidate = *controller;
    AmbarApogeePredictorConfig_t apogee_candidate = *apogee;
    controller_candidate.apogee_tolerance_m =
        controller_candidate.mission_tolerance_m;
    apogee_candidate.vehicle_mass_kg = apogee_candidate.coast_mass_kg;
    apogee_candidate.drag_area_m2 = apogee_candidate.baseline_drag_area_m2;
    apogee_candidate.air_density_kgpm3 =
        apogee_candidate.sea_level_air_density_kgpm3;
    s_flight.config.controller = controller_candidate;
    s_flight.config.apogee = apogee_candidate;
    ambar_controller_runtime_reset();
#if AMBAR_BUILD_IS_VARIABLE_HIL
    s_flight.output_dirty = true;
#endif
    return true;
}

void AmbarFlight_GetConfigSnapshot(AmbarFlightConfig_t *config_out)
{
    if (config_out != NULL)
    {
        *config_out = s_flight.config;
    }
}

void AmbarFlight_ResetOnPad(float timestamp_s)
{
    /* Reset both estimator state and the phase machine at the same timestamp. */
    AmbarEkf_ResetOnPad(&s_flight.ekf, timestamp_s);
    ambar_phase_reset();
    s_flight.latest_timestamp_s = timestamp_s;
    ambar_controller_runtime_reset();
#if AMBAR_BUILD_IS_VARIABLE_HIL
    s_flight.output_dirty = true;
#endif
}

void AmbarFlight_SetArmed(bool armed)
{
    /*
     * Radio ARM/DISARM only affects command authorization.  It does not reset the
     * EKF or force a phase change, so bench testing can watch the same estimator
     * output with deployment inhibited or allowed.
     */
    s_flight.armed = armed;
#if AMBAR_BUILD_IS_VARIABLE_HIL
    s_flight.output_dirty = true;
#endif
}

bool AmbarFlight_IsArmed(void)
{
    return s_flight.armed;
}

void AmbarFlight_SetActuatorFraction(float deploy_fraction, bool valid)
{
    if (valid && isfinite(deploy_fraction))
    {
        s_flight.estimated_deploy_fraction = ambar_clamp(
            deploy_fraction, 0.0f, 1.0f);
        s_flight.actuator_fraction_feedback_valid = true;
        s_flight.last_actuator_model_timestamp_s = s_flight.latest_timestamp_s;
        s_flight.has_actuator_model_timestamp = true;
    }
    else
    {
        s_flight.actuator_fraction_feedback_valid = false;
        s_flight.last_actuator_model_timestamp_s = s_flight.latest_timestamp_s;
        s_flight.has_actuator_model_timestamp = true;
    }
}

/* ===================== SENSOR UPDATES AND OUTPUT API ===================== */

bool AmbarFlight_UpdateImu(float timestamp_s, float vertical_acceleration_mps2)
{
    /*
     * IMU updates drive both estimator propagation and phase transitions because
     * acceleration/velocity are the fastest indicators of launch and coast.
     */
    s_flight.latest_timestamp_s = timestamp_s;
#if AMBAR_BUILD_IS_VARIABLE_HIL
    s_flight.output_dirty = true;
#endif

    const bool propagated =
        AmbarEkf_PropagateImu(&s_flight.ekf, timestamp_s, vertical_acceleration_mps2);

    if (!propagated)
    {
        if (s_flight.consecutive_imu_rejections <
            AMBAR_PHASE_FAULT_CONSECUTIVE_IMU_REJECTIONS)
        {
            ++s_flight.consecutive_imu_rejections;
        }

        /*
         * The EKF already marks this sample unhealthy, so all actuator output is
         * inhibited immediately.  Delay only the permanent phase latch so one
         * recoverable timing/sensor miss cannot strand the system in FAULT.
         */
        if (s_flight.consecutive_imu_rejections >=
            AMBAR_PHASE_FAULT_CONSECUTIVE_IMU_REJECTIONS)
        {
            AmbarNavigationEstimate_t failed_estimate =
                AmbarEkf_GetEstimate(&s_flight.ekf);
            ambar_phase_update(timestamp_s,
                               &failed_estimate,
                               vertical_acceleration_mps2);
        }

        return false;
    }

    s_flight.consecutive_imu_rejections = 0U;
    AmbarNavigationEstimate_t estimate = AmbarEkf_GetEstimate(&s_flight.ekf);
    ambar_phase_update(timestamp_s, &estimate, vertical_acceleration_mps2);

#if !AMBAR_BUILD_IS_VARIABLE_HIL
    AmbarFlightOutput_t output = ambar_build_output();
    if (!output.airbrake_command.inhibit
        && output.airbrake_command.deploy_fraction > 0.0f
        && s_flight.phase == AMBAR_PHASE_COAST)
    {
        s_flight.phase = AMBAR_PHASE_AIRBRAKE_ACTIVE;
    }
#endif

    return propagated;
}

bool AmbarFlight_UpdateBarometer(float barometer_altitude_agl_m, float barometer_stddev_m)
{
    /* Barometer updates only correct the EKF; phase changes happen on IMU ticks. */
    const bool updated = AmbarEkf_UpdateBarometer(&s_flight.ekf,
                                                  barometer_altitude_agl_m,
                                                  barometer_stddev_m);
#if AMBAR_BUILD_IS_VARIABLE_HIL
    s_flight.output_dirty = true;
#endif
    return updated;
}

AmbarFlightOutput_t AmbarFlight_GetOutput(void)
{
    /* Copy the latest decision state; this call does not advance the estimator. */
#if AMBAR_BUILD_IS_VARIABLE_HIL
    if (!s_flight.cached_output_valid || s_flight.output_dirty)
    {
        s_flight.cached_output = ambar_build_output();
        if (!s_flight.cached_output.airbrake_command.inhibit
            && s_flight.cached_output.airbrake_command.deploy_fraction > 0.0f
            && s_flight.phase == AMBAR_PHASE_COAST)
        {
            s_flight.phase = AMBAR_PHASE_AIRBRAKE_ACTIVE;
            s_flight.cached_output.phase = s_flight.phase;
        }
        s_flight.cached_output_valid = true;
        s_flight.output_dirty = false;
    }
    return s_flight.cached_output;
#else
    return ambar_build_output();
#endif
}

const char *AmbarFlight_PhaseName(AmbarFlightPhase_t phase)
{
    /* Stable labels used by logs and operator-facing telemetry. */
    switch (phase)
    {
    case AMBAR_PHASE_PAD_IDLE:
        return "PadIdle";
    case AMBAR_PHASE_BOOST:
        return "Boost";
    case AMBAR_PHASE_COAST:
        return "Coast";
    case AMBAR_PHASE_AIRBRAKE_ACTIVE:
        return "AirbrakeActive";
    case AMBAR_PHASE_RECOVERY:
        return "Recovery";
    case AMBAR_PHASE_FAULT:
        return "Fault";
    default:
        return "Unknown";
    }
}
