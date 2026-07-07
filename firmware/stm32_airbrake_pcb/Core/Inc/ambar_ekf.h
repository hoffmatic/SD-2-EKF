/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 17:44:48 -04:00
 *
 * What this file does:
 *   This header describes the vertical Extended Kalman Filter. It estimates altitude, vertical speed, predicted apogee, accelerometer bias, barometer bias, and health.
 *
 * Process flow:
 *   IMU prediction moves altitude and velocity forward in time. Barometer correction pulls altitude back toward pressure altitude. Health counters track rejected IMU timestamps and pressure samples.
 *
 * Main variables and what can be changed:
 *   AmbarEkfConfig_t is the tuning area. Noise values, initial variances, timestamp limits, and the barometer gate can be changed after bench and replay testing.
 *
 * Assumptions:
 *   IMU acceleration entering the filter is already pointed along rocket vertical and has pad static acceleration removed.
 *
 * What is missing:
 *   This is vertical-only. It does not estimate attitude, horizontal motion, drag, wind, or full vehicle dynamics.
 */

#ifndef AMBAR_EKF_H
#define AMBAR_EKF_H

/*
 * ===================== AMBAR EKF PCB INTEGRATION - NEW FILE =====================
 *
 * This header is the plain-C interface for the vertical Extended Kalman Filter
 * added for the airbrake PCB firmware. It intentionally avoids C++ features,
 * heap allocation, and dynamic matrix libraries so the code is predictable in
 * the STM32CubeIDE C project.
 *
 * EKF state order:
 *   0: altitude above launch pad, meters
 *   1: vertical velocity, meters/second, positive upward
 *   2: accelerometer bias, meters/second^2
 *   3: barometer altitude bias, meters
 *
 * Sensor convention:
 *   The IMU input must already be converted into launch-frame vertical
 *   acceleration with gravity removed. The board/sensor layer owns that
 *   conversion because it depends on how the PCB is mounted in the rocket.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define AMBAR_STANDARD_GRAVITY_MPS2 9.80665f
#define AMBAR_FEET_TO_METERS        0.3048f
#define AMBAR_METERS_TO_FEET        (1.0f / AMBAR_FEET_TO_METERS)
#define AMBAR_TARGET_APOGEE_M       (3000.0f * AMBAR_FEET_TO_METERS)
#define AMBAR_TARGET_TOLERANCE_M    (100.0f * AMBAR_FEET_TO_METERS)

typedef struct
{
    /* Reject IMU samples with dt smaller than this to avoid zero-time updates. */
    float min_dt_s;

    /* Reject IMU samples with dt larger than this because the model drift is too high. */
    float max_dt_s;

    /* White acceleration noise expected from the vertical acceleration input. */
    float accel_noise_stddev_mps2;

    /* Slow random-walk rate for the estimated accelerometer bias. */
    float accel_bias_random_walk_mps2_per_root_s;

    /* Slow random-walk rate for the estimated barometer altitude bias. */
    float barometer_bias_random_walk_m_per_root_s;

    /* Initial covariance values after pad reset.  Larger means less confidence. */
    float initial_altitude_variance_m2;
    float initial_velocity_variance_m2ps2;
    float initial_accel_bias_variance_m2ps4;
    float initial_barometer_bias_variance_m2;

    /* Barometer innovation gate in sigma.  Outliers beyond this are rejected. */
    float barometer_innovation_gate_sigma;
} AmbarEkfConfig_t;

typedef struct
{
    /* Estimated altitude above launch pad in meters. */
    float altitude_agl_m;

    /* Estimated vertical velocity in m/s, positive upward. */
    float vertical_velocity_mps;

    /* Last accepted vertical acceleration input in m/s^2. */
    float vertical_acceleration_mps2;

    /* Ballistic apogee prediction using the current altitude and upward velocity. */
    float predicted_apogee_m;

    /* Diagonal covariance entries exposed for telemetry/debugging. */
    float altitude_variance_m2;
    float velocity_variance_m2ps2;

    /* Estimated difference between barometer altitude and EKF altitude. */
    float barometer_bias_m;

    /* initialized=false means ResetOnPad has not completed yet. */
    bool initialized;

    /* healthy=false means enough rejected samples or invalid inputs have occurred. */
    bool healthy;
} AmbarNavigationEstimate_t;

typedef struct
{
    /* Mirrors the estimator state so telemetry can report estimator readiness. */
    bool initialized;
    bool healthy;

    /* Rejection counters help diagnose bad timestamps, sensor faults, or outliers. */
    uint32_t rejected_imu_samples;
    uint32_t rejected_barometer_samples;

    /* Most recent barometer innovation, both in meters and normalized sigma. */
    float last_barometer_innovation_m;
    float last_barometer_innovation_sigma;
} AmbarEstimatorHealth_t;

typedef struct
{
    /* Configuration copied in at init so the EKF can run without global constants. */
    AmbarEkfConfig_t config;

    /* State vector: altitude, velocity, accel bias, baro bias. */
    float state[4];

    /* Full 4x4 covariance matrix for the state vector above. */
    float covariance[4][4];

    /* Runtime flags and timestamp tracking. */
    bool initialized;
    bool healthy;
    bool has_last_imu_timestamp;
    float last_imu_timestamp_s;

    /* Last acceleration accepted by propagation, used for telemetry output. */
    float last_vertical_acceleration_mps2;

    /* Fault counters and last innovation values exposed through GetHealth(). */
    uint32_t rejected_imu_samples;
    uint32_t rejected_barometer_samples;
    float last_barometer_innovation_m;
    float last_barometer_innovation_sigma;
} AmbarEkf_t;

AmbarEkfConfig_t AmbarEkf_DefaultConfig(void);

/* Initialize a caller-owned EKF object.  Pass NULL config to use defaults. */
void AmbarEkf_Init(AmbarEkf_t *ekf, const AmbarEkfConfig_t *config);

/* Reset state/covariance at the pad and arm timestamp tracking. */
void AmbarEkf_ResetOnPad(AmbarEkf_t *ekf, float timestamp_s);

/* Prediction step driven by pad-zeroed vertical acceleration. */
bool AmbarEkf_PropagateImu(AmbarEkf_t *ekf,
                           float timestamp_s,
                           float vertical_acceleration_mps2);

/* Correction step driven by barometer altitude above the pad. */
bool AmbarEkf_UpdateBarometer(AmbarEkf_t *ekf,
                              float barometer_altitude_agl_m,
                              float barometer_stddev_m);

/* Copy out estimate/health snapshots without exposing mutable EKF internals. */
AmbarNavigationEstimate_t AmbarEkf_GetEstimate(const AmbarEkf_t *ekf);
AmbarEstimatorHealth_t AmbarEkf_GetHealth(const AmbarEkf_t *ekf);

#ifdef __cplusplus
}
#endif

#endif /* AMBAR_EKF_H */
