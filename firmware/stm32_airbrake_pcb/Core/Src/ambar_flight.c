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
 *   high/late, and in coast.  Any inhibit produces a zero request.  Default
 *   thresholds, vehicle mass, drag area, air density, and effectiveness are
 *   first-pass replay values, not flight-qualified measurements.  The predictor
 *   is vertical-only and does not model attitude, wind, or changing atmosphere.
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

static float ambar_drag_apogee(float altitude_m,
                               float velocity_mps,
                               const AmbarApogeePredictorConfig_t *config)
{
    /*
     * This is intentionally still a vertical-only coast predictor.  It numerically
     * steps upward velocity until the vehicle stops rising:
     *   acceleration = -gravity - k * velocity^2
     *   k = 0.5 * air_density * drag_area / mass
     *
     * The model is simple enough for the STM32 and good enough for replay tuning,
     * but it still needs real flight/bench data before being trusted for flight.
     */
    if (config == NULL || velocity_mps <= 0.0f)
    {
        return altitude_m;
    }

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

    for (float t = 0.0f; t < max_time_s && velocity > 0.0f; t += dt_s)
    {
        const float accel = -AMBAR_STANDARD_GRAVITY_MPS2 - drag_k * velocity * velocity;
        altitude += velocity * dt_s + 0.5f * accel * dt_s * dt_s;
        velocity += accel * dt_s;
    }

    if (!isfinite(altitude) || altitude < altitude_m)
    {
        altitude = altitude_m;
    }

    return altitude;
}

static float ambar_selected_apogee(const AmbarNavigationEstimate_t *estimate)
{
    if (estimate == NULL)
    {
        return 0.0f;
    }

    s_flight.last_ballistic_apogee_m =
        ambar_ballistic_apogee(estimate->altitude_agl_m,
                               estimate->vertical_velocity_mps);
    s_flight.last_drag_apogee_m =
        ambar_drag_apogee(estimate->altitude_agl_m,
                          estimate->vertical_velocity_mps,
                          &s_flight.config.apogee);

    if (s_flight.config.apogee.mode == AMBAR_APOGEE_MODE_DRAG)
    {
        return s_flight.last_drag_apogee_m;
    }

    return s_flight.last_ballistic_apogee_m;
}

/* ===================== COMMAND POLICY AND OUTPUT ASSEMBLY ===================== */

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
    if (apogee_error_m <= s_flight.config.controller.apogee_tolerance_m)
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
        return command;
    }

    float effectiveness = s_flight.config.apogee.actuator_effectiveness;
    if (!isfinite(effectiveness) || effectiveness <= 0.0f)
    {
        effectiveness = 1.0f;
    }

    command.deploy_fraction = ambar_clamp(
        (apogee_error_m / s_flight.config.controller.full_deployment_error_m) * effectiveness,
        0.0f,
        s_flight.config.controller.maximum_deploy_fraction
    );
    /*
     * Linear first-pass controller: if predicted apogee is above target, deploy
     * proportionally.  Later flight tests can replace this without changing the
     * estimator API.
     */

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
    output.armed = s_flight.armed;

    return output;
}

/* ===================== DEFAULTS AND LIFECYCLE ===================== */

AmbarFlightConfig_t AmbarFlight_DefaultConfig(void)
{
    /*
     * Conservative first-pass defaults.  They are easy to override in a test build
     * by passing a custom AmbarFlightConfig_t to AmbarFlight_Init().
     */
    AmbarFlightConfig_t config;

    config.estimator = AmbarEkf_DefaultConfig();

    config.phase.liftoff_altitude_m = 3.0f;
    config.phase.liftoff_acceleration_mps2 = 15.0f;
    config.phase.minimum_boost_time_s = 0.50f;
    config.phase.burnout_acceleration_mps2 = -2.0f;
    config.phase.minimum_coast_velocity_mps = 20.0f;
    config.phase.recovery_descent_velocity_mps = -5.0f;

    config.controller.target_apogee_m = AMBAR_TARGET_APOGEE_M;
    config.controller.apogee_tolerance_m = AMBAR_TARGET_TOLERANCE_M;
    config.controller.full_deployment_error_m = 250.0f * AMBAR_FEET_TO_METERS;
    config.controller.maximum_deploy_fraction = 1.0f;
    config.controller.minimum_deploy_altitude_m = 200.0f * AMBAR_FEET_TO_METERS;
    config.controller.minimum_flight_time_s = 1.0f;

    /*
     * These are placeholders for replay and bench tuning.  The mode defaults to
     * drag because this pass is meant to expose the predictor, but the values are
     * conservative and must be replaced with measured vehicle data.
     */
    config.apogee.mode = AMBAR_APOGEE_MODE_DRAG;
    config.apogee.vehicle_mass_kg = 5.0f;
    config.apogee.drag_area_m2 = 0.012f;
    config.apogee.air_density_kgpm3 = 1.225f;
    config.apogee.time_step_s = 0.02f;
    config.apogee.max_predict_time_s = 30.0f;
    config.apogee.actuator_effectiveness = 1.0f;

    return config;
}

void AmbarFlight_Init(const AmbarFlightConfig_t *config)
{
    /* Initialize singleton flight state.  The embedded app uses one flight computer. */
    memset(&s_flight, 0, sizeof(s_flight));
    s_flight.config = (config != NULL) ? *config : AmbarFlight_DefaultConfig();
    AmbarEkf_Init(&s_flight.ekf, &s_flight.config.estimator);
    ambar_phase_reset();
    s_flight.armed = false;
}

void AmbarFlight_ResetOnPad(float timestamp_s)
{
    /* Reset both estimator state and the phase machine at the same timestamp. */
    AmbarEkf_ResetOnPad(&s_flight.ekf, timestamp_s);
    ambar_phase_reset();
    s_flight.latest_timestamp_s = timestamp_s;
}

void AmbarFlight_SetArmed(bool armed)
{
    /*
     * Radio ARM/DISARM only affects command authorization.  It does not reset the
     * EKF or force a phase change, so bench testing can watch the same estimator
     * output with deployment inhibited or allowed.
     */
    s_flight.armed = armed;
}

bool AmbarFlight_IsArmed(void)
{
    return s_flight.armed;
}

/* ===================== SENSOR UPDATES AND OUTPUT API ===================== */

bool AmbarFlight_UpdateImu(float timestamp_s, float vertical_acceleration_mps2)
{
    /*
     * IMU updates drive both estimator propagation and phase transitions because
     * acceleration/velocity are the fastest indicators of launch and coast.
     */
    s_flight.latest_timestamp_s = timestamp_s;

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

    AmbarFlightOutput_t output = ambar_build_output();
    if (!output.airbrake_command.inhibit
        && output.airbrake_command.deploy_fraction > 0.0f
        && s_flight.phase == AMBAR_PHASE_COAST)
    {
        s_flight.phase = AMBAR_PHASE_AIRBRAKE_ACTIVE;
    }

    return propagated;
}

bool AmbarFlight_UpdateBarometer(float barometer_altitude_agl_m, float barometer_stddev_m)
{
    /* Barometer updates only correct the EKF; phase changes happen on IMU ticks. */
    return AmbarEkf_UpdateBarometer(&s_flight.ekf,
                                    barometer_altitude_agl_m,
                                    barometer_stddev_m);
}

AmbarFlightOutput_t AmbarFlight_GetOutput(void)
{
    /* Copy the latest decision state; this call does not advance the estimator. */
    return ambar_build_output();
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
