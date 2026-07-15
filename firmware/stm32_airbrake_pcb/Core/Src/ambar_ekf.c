/*
 * AMBAR VERTICAL EXTENDED KALMAN FILTER - IMPLEMENTATION
 *
 * Purpose
 *   Estimates pad-relative altitude, upward velocity, accelerometer bias, and
 *   barometer-altitude bias.  It is deliberately a small, deterministic
 *   four-state filter with fixed-size matrices and no heap allocation.
 *
 * Data flow
 *   RocketSensors supplies pad-zeroed vertical acceleration in SI units.
 *   AmbarEkf_PropagateImu() advances state and covariance at the IMU rate;
 *   AmbarEkf_UpdateBarometer() applies the slower pressure-altitude correction.
 *   AmbarFlight reads copy-out estimate and health snapshots, then owns phase,
 *   drag prediction, and deployment decisions.  See CODE_GUIDE.md [ARCH-2]
 *   and [ARCH-4].
 *
 * Section map
 *   1. State ordering and fixed-size matrix helpers
 *   2. Default tuning and lifecycle
 *   3. IMU prediction
 *   4. Barometer correction
 *   5. Read-only estimate and health snapshots
 *
 * Safety and assumptions
 *   Positive is upward; acceleration must already be aligned to the rocket
 *   vertical axis with the stationary pad value removed.  Invalid numbers,
 *   implausible sample intervals, and gated barometer innovations are rejected.
 *   The default covariance/noise values are first-pass replay settings and need
 *   validation from real sensor logs.  This module is vertical-only: it does
 *   not estimate attitude, wind, horizontal motion, or aerodynamic drag.
 */

#include "ambar_ekf.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* ===================== STATE MODEL AND MATRIX HELPERS ===================== */

enum
{
    /*
     * Keep these indexes in one enum so all state/covariance references use the
     * same ordering.  Changing this order requires updating the dynamics and
     * measurement model comments in ambar_ekf.h.
     */
    AMBAR_EKF_ALTITUDE = 0,
    AMBAR_EKF_VELOCITY = 1,
    AMBAR_EKF_ACCEL_BIAS = 2,
    AMBAR_EKF_BARO_BIAS = 3,
    AMBAR_EKF_STATE_COUNT = 4
};

static bool ambar_is_finite(float value)
{
    /*
     * Small wrapper makes the EKF rejection policy easy to read.  NaN/Inf values
     * usually mean a sensor read failed upstream or a pressure conversion broke.
     */
    return isfinite(value);
}

static float ambar_ballistic_apogee(float altitude_m, float vertical_velocity_mps)
{
    /*
     * First PCB integration uses a ballistic upper-bound apogee estimate.
     * This is conservative and simple; later flight-data work can replace this
     * with a drag-aware predictor without changing the EKF state itself.
     */
    if (vertical_velocity_mps <= 0.0f)
    {
        return altitude_m;
    }

    return altitude_m
         + (vertical_velocity_mps * vertical_velocity_mps)
             / (2.0f * AMBAR_STANDARD_GRAVITY_MPS2);
}

static void ambar_matrix_identity(float matrix[4][4])
{
    /* Build a 4x4 identity matrix for transition/correction matrices. */
    memset(matrix, 0, sizeof(float) * 16U);

    for (int i = 0; i < AMBAR_EKF_STATE_COUNT; ++i)
    {
        matrix[i][i] = 1.0f;
    }
}

static void ambar_matrix_transpose(const float input[4][4], float output[4][4])
{
    for (int row = 0; row < AMBAR_EKF_STATE_COUNT; ++row)
    {
        for (int column = 0; column < AMBAR_EKF_STATE_COUNT; ++column)
        {
            output[row][column] = input[column][row];
        }
    }
}

static void ambar_matrix_multiply(const float left[4][4],
                                  const float right[4][4],
                                  float output[4][4])
{
    /*
     * Fixed-size matrix multiply.  The local result buffer allows output to alias
     * either input without corrupting the calculation.
     */
    float result[4][4] = {0};

    for (int row = 0; row < AMBAR_EKF_STATE_COUNT; ++row)
    {
        for (int column = 0; column < AMBAR_EKF_STATE_COUNT; ++column)
        {
            float sum = 0.0f;

            for (int inner = 0; inner < AMBAR_EKF_STATE_COUNT; ++inner)
            {
                sum += left[row][inner] * right[inner][column];
            }

            result[row][column] = sum;
        }
    }

    memcpy(output, result, sizeof(result));
}

static void ambar_matrix_add(const float left[4][4],
                             const float right[4][4],
                             float output[4][4])
{
    for (int row = 0; row < AMBAR_EKF_STATE_COUNT; ++row)
    {
        for (int column = 0; column < AMBAR_EKF_STATE_COUNT; ++column)
        {
            output[row][column] = left[row][column] + right[row][column];
        }
    }
}

static void ambar_matrix_subtract(const float left[4][4],
                                  const float right[4][4],
                                  float output[4][4])
{
    for (int row = 0; row < AMBAR_EKF_STATE_COUNT; ++row)
    {
        for (int column = 0; column < AMBAR_EKF_STATE_COUNT; ++column)
        {
            output[row][column] = left[row][column] - right[row][column];
        }
    }
}

static void ambar_symmetrize_covariance(AmbarEkf_t *ekf)
{
    /*
     * Floating-point roundoff can make P slightly asymmetric.  The covariance
     * should be symmetric by definition, so average mirrored terms after updates.
     */
    for (int row = 0; row < AMBAR_EKF_STATE_COUNT; ++row)
    {
        for (int column = row + 1; column < AMBAR_EKF_STATE_COUNT; ++column)
        {
            const float average =
                0.5f * (ekf->covariance[row][column] + ekf->covariance[column][row]);
            ekf->covariance[row][column] = average;
            ekf->covariance[column][row] = average;
        }
    }
}

/* ===================== CONFIGURATION AND LIFECYCLE ===================== */

AmbarEkfConfig_t AmbarEkf_DefaultConfig(void)
{
    /*
     * First-pass tuning values for bench/initial flight replay.  Treat these as
     * measurable knobs: log telemetry, compare against replay data, then tune.
     */
    AmbarEkfConfig_t config;

    config.min_dt_s = 0.0005f;
    config.max_dt_s = 0.0500f;
    config.accel_noise_stddev_mps2 = 3.0f;
    config.accel_bias_random_walk_mps2_per_root_s = 0.08f;
    config.barometer_bias_random_walk_m_per_root_s = 0.03f;
    config.initial_altitude_variance_m2 = 4.0f;
    config.initial_velocity_variance_m2ps2 = 25.0f;
    config.initial_accel_bias_variance_m2ps4 = 4.0f;
    config.initial_barometer_bias_variance_m2 = 4.0f;
    config.barometer_innovation_gate_sigma = 5.0f;

    return config;
}

void AmbarEkf_Init(AmbarEkf_t *ekf, const AmbarEkfConfig_t *config)
{
    /* Caller owns the EKF object; no heap allocation is used on the STM32. */
    if (ekf == NULL)
    {
        return;
    }

    memset(ekf, 0, sizeof(*ekf));
    ekf->config = (config != NULL) ? *config : AmbarEkf_DefaultConfig();
}

void AmbarEkf_ResetOnPad(AmbarEkf_t *ekf, float timestamp_s)
{
    /*
     * Pad reset defines altitude=0 and velocity=0.  Bias states also start at
     * zero, but their covariance lets later measurements estimate them.
     */
    if (ekf == NULL)
    {
        return;
    }

    memset(ekf->state, 0, sizeof(ekf->state));
    memset(ekf->covariance, 0, sizeof(ekf->covariance));

    ekf->covariance[AMBAR_EKF_ALTITUDE][AMBAR_EKF_ALTITUDE] =
        ekf->config.initial_altitude_variance_m2;
    ekf->covariance[AMBAR_EKF_VELOCITY][AMBAR_EKF_VELOCITY] =
        ekf->config.initial_velocity_variance_m2ps2;
    ekf->covariance[AMBAR_EKF_ACCEL_BIAS][AMBAR_EKF_ACCEL_BIAS] =
        ekf->config.initial_accel_bias_variance_m2ps4;
    ekf->covariance[AMBAR_EKF_BARO_BIAS][AMBAR_EKF_BARO_BIAS] =
        ekf->config.initial_barometer_bias_variance_m2;

    ekf->initialized = true;
    ekf->healthy = ambar_is_finite(timestamp_s);
    ekf->has_last_imu_timestamp = ekf->healthy;
    ekf->last_imu_timestamp_s = timestamp_s;
    ekf->last_vertical_acceleration_mps2 = 0.0f;
    ekf->rejected_imu_samples = 0U;
    ekf->rejected_barometer_samples = 0U;
    ekf->last_barometer_innovation_m = 0.0f;
    ekf->last_barometer_innovation_sigma = 0.0f;
}

/* ===================== IMU PREDICTION ===================== */

bool AmbarEkf_PropagateImu(AmbarEkf_t *ekf,
                           float timestamp_s,
                           float vertical_acceleration_mps2)
{
    if (ekf == NULL)
    {
        return false;
    }

    if (!ambar_is_finite(timestamp_s) || !ambar_is_finite(vertical_acceleration_mps2))
    {
        ekf->healthy = false;
        ++ekf->rejected_imu_samples;
        return false;
    }

    if (!ekf->initialized)
    {
        AmbarEkf_ResetOnPad(ekf, timestamp_s);
        ekf->last_vertical_acceleration_mps2 = vertical_acceleration_mps2;
        return true;
    }

    if (!ekf->has_last_imu_timestamp)
    {
        ekf->has_last_imu_timestamp = true;
        ekf->last_imu_timestamp_s = timestamp_s;
        return true;
    }

    const float dt_s = timestamp_s - ekf->last_imu_timestamp_s;
    ekf->last_imu_timestamp_s = timestamp_s;

    if (dt_s < ekf->config.min_dt_s || dt_s > ekf->config.max_dt_s)
    {
        /*
         * Bad timestamps are usually scheduler stalls or duplicate samples.  Do
         * not integrate them because velocity/altitude would jump incorrectly.
         */
        ekf->healthy = false;
        ++ekf->rejected_imu_samples;
        return false;
    }

    const float corrected_accel_mps2 =
        vertical_acceleration_mps2 - ekf->state[AMBAR_EKF_ACCEL_BIAS];
    ekf->last_vertical_acceleration_mps2 = corrected_accel_mps2;

    /*
     * Constant-acceleration kinematics over one IMU interval. The acceleration
     * bias state is subtracted before integration so the barometer can slowly
     * teach the filter about persistent IMU offset.
     */
    ekf->state[AMBAR_EKF_ALTITUDE] +=
        ekf->state[AMBAR_EKF_VELOCITY] * dt_s
      + 0.5f * corrected_accel_mps2 * dt_s * dt_s;
    ekf->state[AMBAR_EKF_VELOCITY] += corrected_accel_mps2 * dt_s;

    float transition[4][4];
    ambar_matrix_identity(transition);
    transition[AMBAR_EKF_ALTITUDE][AMBAR_EKF_VELOCITY] = dt_s;
    transition[AMBAR_EKF_ALTITUDE][AMBAR_EKF_ACCEL_BIAS] = -0.5f * dt_s * dt_s;
    transition[AMBAR_EKF_VELOCITY][AMBAR_EKF_ACCEL_BIAS] = -dt_s;

    const float accel_variance =
        ekf->config.accel_noise_stddev_mps2 * ekf->config.accel_noise_stddev_mps2;

    float process_noise[4][4] = {0};
    /*
     * Acceleration noise is integrated into altitude and velocity uncertainty.
     * Bias random walks are separate diagonal terms because they drift slowly.
     */
    process_noise[AMBAR_EKF_ALTITUDE][AMBAR_EKF_ALTITUDE] =
        0.25f * dt_s * dt_s * dt_s * dt_s * accel_variance;
    process_noise[AMBAR_EKF_ALTITUDE][AMBAR_EKF_VELOCITY] =
        0.5f * dt_s * dt_s * dt_s * accel_variance;
    process_noise[AMBAR_EKF_VELOCITY][AMBAR_EKF_ALTITUDE] =
        process_noise[AMBAR_EKF_ALTITUDE][AMBAR_EKF_VELOCITY];
    process_noise[AMBAR_EKF_VELOCITY][AMBAR_EKF_VELOCITY] =
        dt_s * dt_s * accel_variance;
    process_noise[AMBAR_EKF_ACCEL_BIAS][AMBAR_EKF_ACCEL_BIAS] =
        ekf->config.accel_bias_random_walk_mps2_per_root_s
      * ekf->config.accel_bias_random_walk_mps2_per_root_s
      * dt_s;
    process_noise[AMBAR_EKF_BARO_BIAS][AMBAR_EKF_BARO_BIAS] =
        ekf->config.barometer_bias_random_walk_m_per_root_s
      * ekf->config.barometer_bias_random_walk_m_per_root_s
      * dt_s;

    /*
     * Covariance prediction:
     *   P = F P F^T + Q
     *
     * Q injects uncertainty from accelerometer noise and slow bias wandering.
     * Without it the filter becomes overconfident and will reject valid sensor
     * corrections later in flight.
     */
    float transition_transpose[4][4];
    float temp[4][4];
    float predicted[4][4];

    ambar_matrix_transpose(transition, transition_transpose);
    ambar_matrix_multiply(transition, ekf->covariance, temp);
    ambar_matrix_multiply(temp, transition_transpose, predicted);
    ambar_matrix_add(predicted, process_noise, ekf->covariance);
    ambar_symmetrize_covariance(ekf);

    ekf->healthy = true;
    return true;
}

/* ===================== BAROMETER CORRECTION ===================== */

bool AmbarEkf_UpdateBarometer(AmbarEkf_t *ekf,
                              float barometer_altitude_agl_m,
                              float barometer_stddev_m)
{
    if (ekf == NULL || !ekf->initialized)
    {
        return false;
    }

    if (!ambar_is_finite(barometer_altitude_agl_m)
        || !ambar_is_finite(barometer_stddev_m)
        || barometer_stddev_m <= 0.0f)
    {
        ekf->healthy = false;
        ++ekf->rejected_barometer_samples;
        return false;
    }

    const float predicted_measurement_m =
        ekf->state[AMBAR_EKF_ALTITUDE] + ekf->state[AMBAR_EKF_BARO_BIAS];
    const float innovation_m = barometer_altitude_agl_m - predicted_measurement_m;
    const float measurement_variance_m2 = barometer_stddev_m * barometer_stddev_m;

    /*
     * Barometer measurement model:
     *   z = altitude + barometer_bias + noise
     *
     * H is therefore [1, 0, 0, 1], which lets us write the scalar update without
     * allocating temporary 1x4 or 4x1 matrix objects.
     */
    const float innovation_variance =
        ekf->covariance[AMBAR_EKF_ALTITUDE][AMBAR_EKF_ALTITUDE]
      + ekf->covariance[AMBAR_EKF_BARO_BIAS][AMBAR_EKF_BARO_BIAS]
      + 2.0f * ekf->covariance[AMBAR_EKF_ALTITUDE][AMBAR_EKF_BARO_BIAS]
      + measurement_variance_m2;

    if (!ambar_is_finite(innovation_variance) || innovation_variance <= 1.0e-6f)
    {
        ekf->healthy = false;
        ++ekf->rejected_barometer_samples;
        return false;
    }

    const float innovation_sigma = sqrtf(innovation_variance);
    ekf->last_barometer_innovation_m = innovation_m;
    ekf->last_barometer_innovation_sigma = innovation_sigma;

    if (fabsf(innovation_m) > ekf->config.barometer_innovation_gate_sigma * innovation_sigma)
    {
        /*
         * A pressure spike should not kick the rocket into a false altitude.
         * Rejection leaves the IMU propagation alive until a sane barometer
         * sample returns.
         */
        ++ekf->rejected_barometer_samples;
        return false;
    }

    float gain[4];
    for (int row = 0; row < AMBAR_EKF_STATE_COUNT; ++row)
    {
        /*
         * Scalar Kalman gain for H=[1 0 0 1].  Every state gets a gain term based
         * on how strongly it covaries with altitude and barometer bias.
         */
        gain[row] =
            (ekf->covariance[row][AMBAR_EKF_ALTITUDE]
           + ekf->covariance[row][AMBAR_EKF_BARO_BIAS])
            / innovation_variance;
        ekf->state[row] += gain[row] * innovation_m;
    }

    float gain_measurement[4][4] = {0};
    for (int row = 0; row < AMBAR_EKF_STATE_COUNT; ++row)
    {
        gain_measurement[row][AMBAR_EKF_ALTITUDE] = gain[row];
        gain_measurement[row][AMBAR_EKF_BARO_BIAS] = gain[row];
    }

    float identity[4][4];
    float correction[4][4];
    float correction_transpose[4][4];
    float temp[4][4];
    float joseph_state[4][4];
    float joseph_noise[4][4] = {0};

    ambar_matrix_identity(identity);
    ambar_matrix_subtract(identity, gain_measurement, correction);
    ambar_matrix_transpose(correction, correction_transpose);

    for (int row = 0; row < AMBAR_EKF_STATE_COUNT; ++row)
    {
        for (int column = 0; column < AMBAR_EKF_STATE_COUNT; ++column)
        {
            joseph_noise[row][column] =
                gain[row] * measurement_variance_m2 * gain[column];
        }
    }

    /*
     * Joseph-form covariance update:
     *   P = (I - K H) P (I - K H)^T + K R K^T
     *
     * It costs a few more multiplies than the compact update but is less likely
     * to produce a negative covariance from floating-point roundoff.
     */
    ambar_matrix_multiply(correction, ekf->covariance, temp);
    ambar_matrix_multiply(temp, correction_transpose, joseph_state);
    ambar_matrix_add(joseph_state, joseph_noise, ekf->covariance);
    ambar_symmetrize_covariance(ekf);

    return true;
}

/* ===================== READ-ONLY OUTPUT SNAPSHOTS ===================== */

AmbarNavigationEstimate_t AmbarEkf_GetEstimate(const AmbarEkf_t *ekf)
{
    /*
     * Return a copy, not a pointer.  That keeps callers from accidentally
     * changing filter state while building telemetry or actuator decisions.
     */
    AmbarNavigationEstimate_t estimate;
    memset(&estimate, 0, sizeof(estimate));

    if (ekf == NULL)
    {
        return estimate;
    }

    estimate.altitude_agl_m = ekf->state[AMBAR_EKF_ALTITUDE];
    estimate.vertical_velocity_mps = ekf->state[AMBAR_EKF_VELOCITY];
    estimate.vertical_acceleration_mps2 = ekf->last_vertical_acceleration_mps2;
    estimate.predicted_apogee_m =
        ambar_ballistic_apogee(ekf->state[AMBAR_EKF_ALTITUDE],
                               ekf->state[AMBAR_EKF_VELOCITY]);
    estimate.altitude_variance_m2 = ekf->covariance[AMBAR_EKF_ALTITUDE][AMBAR_EKF_ALTITUDE];
    estimate.velocity_variance_m2ps2 = ekf->covariance[AMBAR_EKF_VELOCITY][AMBAR_EKF_VELOCITY];
    estimate.barometer_bias_m = ekf->state[AMBAR_EKF_BARO_BIAS];
    estimate.initialized = ekf->initialized;
    estimate.healthy = ekf->healthy;

    return estimate;
}

AmbarEstimatorHealth_t AmbarEkf_GetHealth(const AmbarEkf_t *ekf)
{
    /* Health snapshot is intentionally small enough to send over telemetry. */
    AmbarEstimatorHealth_t health;
    memset(&health, 0, sizeof(health));

    if (ekf == NULL)
    {
        return health;
    }

    health.initialized = ekf->initialized;
    health.healthy = ekf->healthy;
    health.rejected_imu_samples = ekf->rejected_imu_samples;
    health.rejected_barometer_samples = ekf->rejected_barometer_samples;
    health.last_barometer_innovation_m = ekf->last_barometer_innovation_m;
    health.last_barometer_innovation_sigma = ekf->last_barometer_innovation_sigma;

    return health;
}
