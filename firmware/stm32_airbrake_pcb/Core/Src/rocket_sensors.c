/*
 * AMBAR BOARD SENSOR ADAPTER - IMPLEMENTATION
 *
 * Purpose
 *   Owns the board-level LSM6DSV32X, BMP388, and LIS2MDL instances and converts
 *   their raw bus data into the pad-relative SI-unit channels used by the
 *   vertical estimator.  Raw counts remain available for bring-up and telemetry.
 *
 * Data flow
 *   Init starts the required IMU/barometer and optional magnetometer.  A pad
 *   capture or stored configuration establishes the stationary vertical
 *   acceleration and pressure baselines.  IMU reads select the configured
 *   rocket axis/sign and subtract the pad value; barometer reads compensate raw
 *   pressure and convert it to altitude above the pad.  AmbarApp feeds those
 *   channels into AmbarFlight/AmbarEkf as described in CODE_GUIDE.md [ARCH-2].
 *
 * Section map
 *   1. Board mapping, conversion constants, and driver handles
 *   2. Calibration/data-ready module state
 *   3. Private unit/orientation/pressure helpers
 *   4. Runtime calibration access
 *   5. Sensor initialization and stationary pad capture
 *   6. Raw/converted read API
 *   7. EXTI data-ready callbacks and consumers
 *
 * Safety and assumptions
 *   The pad reference must be captured upright and motionless, and the configured
 *   axis/sign must be bench-verified so upward acceleration is positive.  Scale
 *   factors assume the driver ranges shown below.  Pad capture uses short HAL
 *   delays and belongs in setup/reset flow, not a time-critical sensor tick.
 *   Magnetometer failure does not invalidate this vertical estimator.  There is
 *   no automatic orientation detection, sensor redundancy, or pressure-vent model.
 */

#include "rocket_sensors.h"

#include "ambar_features.h"
#include "main.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* ===================== BOARD MAPPING AND CONVERSION CONSTANTS ===================== */

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

#ifndef ROCKET_IMU_I2C_HANDLE
#define ROCKET_IMU_I2C_HANDLE   hi2c1
#endif

#ifndef ROCKET_BARO_I2C_HANDLE
#define ROCKET_BARO_I2C_HANDLE  hi2c2
#endif

#ifndef ROCKET_MAG_I2C_HANDLE
#define ROCKET_MAG_I2C_HANDLE   hi2c3
#endif

#define ROCKET_STANDARD_GRAVITY_MPS2       9.80665f
#define ROCKET_LSM6_ACCEL_32G_MPS2_PER_LSB (0.000976f * ROCKET_STANDARD_GRAVITY_MPS2)
#define ROCKET_LSM6_GYRO_4000DPS_PER_LSB   0.140f
#define ROCKET_ALTITUDE_EXPONENT           0.190294957f

/* ===================== BOARD-OWNED DRIVER HANDLES ===================== */

LSM6DSV32X_HandleTypeDef rocket_imu = {
    /* I2C1: primary acceleration source for the EKF propagation step. */
    .hi2c = &ROCKET_IMU_I2C_HANDLE,
    .addr_7bit = LSM6DSV32X_I2C_ADDR_LOW,
    .who_am_i = 0U
};

LIS2MDL_HandleTypeDef rocket_mag = {
    /* I2C3: health/telemetry only in the first vertical estimator build. */
    .hi2c = &ROCKET_MAG_I2C_HANDLE,
    .addr_7bit = LIS2MDL_I2C_ADDR,
    .who_am_i = 0U
};

BMP388_HandleTypeDef rocket_baro = {
    /* I2C2: pressure source for altitude AGL and EKF correction. */
    .hi2c = &ROCKET_BARO_I2C_HANDLE,
    .addr_7bit = BMP388_I2C_ADDR_LOW,
    .chip_id = 0U
};

/* ===================== CALIBRATION AND DATA-READY STATE ===================== */

/* Pad baselines share the zero reference used by both converted sensor paths. */
static float s_vertical_accel_pad_mps2 = 0.0f;
static float s_pad_pressure_pa = 0.0f;
static bool s_pad_reference_valid = false;

/* Runtime orientation may come from saved config; macros provide safe defaults. */
static int32_t s_vertical_axis_index = ROCKET_IMU_VERTICAL_ACCEL_INDEX;
static float s_vertical_axis_sign = ROCKET_IMU_VERTICAL_ACCEL_SIGN;

/* ISR-set flags are consumed by the cooperative scheduler; counters remain diagnostic. */
static volatile uint8_t s_imu_data_ready_seen = 0U;
static volatile uint8_t s_baro_data_ready_seen = 0U;
static uint32_t s_imu_data_ready_count = 0U;
static uint32_t s_baro_data_ready_count = 0U;
static uint8_t s_last_imu_status = 0U;
static uint8_t s_last_baro_status = 0U;
static uint8_t s_last_baro_error = 0U;

/* ===================== PRIVATE CONVERSION HELPERS ===================== */

static float rocket_accel_mps2_from_raw(int16_t raw)
{
    /* LSM6DSV32X is configured for +/-32 g, giving 0.976 mg/LSB. */
    return (float)raw * ROCKET_LSM6_ACCEL_32G_MPS2_PER_LSB;
}

static float rocket_gyro_dps_from_raw(int16_t raw)
{
    /* Gyro telemetry is not part of this vertical EKF, but it remains useful. */
    return (float)raw * ROCKET_LSM6_GYRO_4000DPS_PER_LSB;
}

static float rocket_signed_vertical_accel(const int16_t imu[LSM6DSV32X_DATA_COUNT])
{
    /*
     * The PCB orientation was not safe to hard-code from the schematic alone.
     * Change ROCKET_IMU_VERTICAL_ACCEL_INDEX and ROCKET_IMU_VERTICAL_ACCEL_SIGN
     * after the bench orientation check so positive acceleration points upward
     * along the rocket body.
     */
    int32_t axis = s_vertical_axis_index;
    if (axis < LSM6DSV32X_ACCEL_X || axis > LSM6DSV32X_ACCEL_Z)
    {
        axis = ROCKET_IMU_VERTICAL_ACCEL_INDEX;
    }

    return s_vertical_axis_sign * rocket_accel_mps2_from_raw(imu[axis]);
}

/* ===================== RUNTIME ORIENTATION AND PAD REFERENCE ===================== */

HAL_StatusTypeDef RocketSensors_ApplyOrientationConfig(int32_t axis_index, float axis_sign)
{
    /*
     * The first EKF build used compile-time axis/sign values.  This lets the
     * bench config choose the vertical accelerometer axis after the PCB is placed
     * in the rocket orientation, without recompiling the firmware.
     */
    if (axis_index < LSM6DSV32X_ACCEL_X || axis_index > LSM6DSV32X_ACCEL_Z)
    {
        return HAL_ERROR;
    }

    if (!isfinite(axis_sign) || fabsf(axis_sign) < 0.5f)
    {
        return HAL_ERROR;
    }

    s_vertical_axis_index = axis_index;
    s_vertical_axis_sign = (axis_sign >= 0.0f) ? 1.0f : -1.0f;
    return HAL_OK;
}

/* Restore or inject a pad baseline only after both SI values pass sanity checks. */
HAL_StatusTypeDef RocketSensors_SetPadReference(float vertical_accel_mps2, float pressure_pa)
{
    if (!isfinite(vertical_accel_mps2) || !isfinite(pressure_pa) || pressure_pa <= 0.0f)
    {
        s_pad_reference_valid = false;
        return HAL_ERROR;
    }

    s_vertical_accel_pad_mps2 = vertical_accel_mps2;
    s_pad_pressure_pa = pressure_pa;
    s_pad_reference_valid = true;
    return HAL_OK;
}

/* Return calibration, interrupt counts, and last sensor status without bus access. */
RocketSensorCalibrationStatus_t RocketSensors_GetCalibrationStatus(void)
{
    RocketSensorCalibrationStatus_t status;

    memset(&status, 0, sizeof(status));
    status.vertical_axis_index = s_vertical_axis_index;
    status.vertical_axis_sign = s_vertical_axis_sign;
    status.pad_vertical_accel_mps2 = s_vertical_accel_pad_mps2;
    status.pad_pressure_pa = s_pad_pressure_pa;
    status.imu_data_ready_count = s_imu_data_ready_count;
    status.barometer_data_ready_count = s_baro_data_ready_count;
    status.imu_status_reg = s_last_imu_status;
    status.barometer_status_reg = s_last_baro_status;
    status.barometer_error_reg = s_last_baro_error;
    status.pad_reference_valid = s_pad_reference_valid;

    return status;
}

/* ===================== CONVERTED-SAMPLE HELPERS ===================== */

static float rocket_altitude_from_pressure(float pressure_pa, float reference_pressure_pa)
{
    if (pressure_pa <= 0.0f || reference_pressure_pa <= 0.0f)
    {
        return 0.0f;
    }

    /*
     * The pad pressure captured during reset is the zero-altitude reference.
     * This gives altitude AGL for the barometer correction without requiring a
     * launch-site sea-level pressure setting.
     */
    return 44330.0f
         * (1.0f - powf(pressure_pa / reference_pressure_pa, ROCKET_ALTITUDE_EXPONENT));
}

static void rocket_fill_converted_imu(const RocketSensorRawData_t *raw,
                                      RocketSensorConvertedData_t *converted)
{
    /*
     * Keep all three axes in converted data even though the EKF uses one vertical
     * axis.  This makes the orientation check visible over telemetry.
     */
    converted->gyro_dps[0] = rocket_gyro_dps_from_raw(raw->imu[LSM6DSV32X_GYRO_X]);
    converted->gyro_dps[1] = rocket_gyro_dps_from_raw(raw->imu[LSM6DSV32X_GYRO_Y]);
    converted->gyro_dps[2] = rocket_gyro_dps_from_raw(raw->imu[LSM6DSV32X_GYRO_Z]);

    converted->accel_mps2[0] = rocket_accel_mps2_from_raw(raw->imu[LSM6DSV32X_ACCEL_X]);
    converted->accel_mps2[1] = rocket_accel_mps2_from_raw(raw->imu[LSM6DSV32X_ACCEL_Y]);
    converted->accel_mps2[2] = rocket_accel_mps2_from_raw(raw->imu[LSM6DSV32X_ACCEL_Z]);

    converted->vertical_acceleration_mps2 =
        rocket_signed_vertical_accel(raw->imu) - s_vertical_accel_pad_mps2;
    converted->imu_valid = true;
    converted->pad_reference_valid = s_pad_reference_valid;
}

/* ===================== INITIALIZATION AND STATIONARY PAD CAPTURE ===================== */

HAL_StatusTypeDef RocketSensors_Init(void)
{
    HAL_StatusTypeDef status;
    HAL_StatusTypeDef overall_status = HAL_OK;

    /*
     * The first estimator only needs the IMU and BMP388.  LIS2MDL is still read
     * for health/telemetry, but a magnetometer init fault should not prevent pad
     * reference capture or vertical EKF operation.
     */
    status = LSM6DSV32X_Init(&rocket_imu);
    if (status != HAL_OK) { overall_status = HAL_ERROR; }
    if (status == HAL_OK && LSM6DSV32X_RunBasicSelfTest(&rocket_imu) != HAL_OK)
    {
        overall_status = HAL_ERROR;
    }

#if AMBAR_FEATURE_MAGNETOMETER_TELEMETRY
    status = LIS2MDL_Init(&rocket_mag);
    (void)status;
#endif

    status = BMP388_Init(&rocket_baro);
    if (status != HAL_OK) { overall_status = HAL_ERROR; }

    return overall_status;
}

HAL_StatusTypeDef RocketSensors_ResetPadReference(uint16_t sample_count)
{
    /* Average independent successful IMU and pressure reads while stationary. */
    if (sample_count == 0U)
    {
        sample_count = 1U;
    }

    float accel_sum = 0.0f;
    float pressure_sum = 0.0f;
    uint16_t accel_count = 0U;
    uint16_t pressure_count = 0U;

    /*
     * Average a short stationary window on the pad.  The acceleration average is
     * subtracted from future vertical acceleration samples, and pressure average
     * becomes the altitude AGL reference.
     */
    for (uint16_t sample = 0U; sample < sample_count; ++sample)
    {
        int16_t imu[LSM6DSV32X_DATA_COUNT] = {0};
        float pressure_pa = 0.0f;
        float temperature_c = 0.0f;

        if (LSM6DSV32X_ReadData(&rocket_imu, imu) == HAL_OK)
        {
            accel_sum += rocket_signed_vertical_accel(imu);
            ++accel_count;
        }

        if (BMP388_ReadPressureTemperature(&rocket_baro, &pressure_pa, &temperature_c) == HAL_OK
            && pressure_pa > 0.0f)
        {
            pressure_sum += pressure_pa;
            ++pressure_count;
        }

        HAL_Delay(5U);
    }

    if (accel_count == 0U || pressure_count == 0U)
    {
        s_pad_reference_valid = false;
        return HAL_ERROR;
    }

    s_vertical_accel_pad_mps2 = accel_sum / (float)accel_count;
    s_pad_pressure_pa = pressure_sum / (float)pressure_count;
    s_pad_reference_valid = s_pad_pressure_pa > 0.0f;

    return s_pad_reference_valid ? HAL_OK : HAL_ERROR;
}

/* ===================== RAW AND CONVERTED SENSOR READS ===================== */

HAL_StatusTypeDef RocketSensors_ReadAll(
    RocketSensorRawData_t *data,
    HAL_StatusTypeDef *imu_status,
    HAL_StatusTypeDef *mag_status,
    HAL_StatusTypeDef *baro_status
)
{
    /*
     * Raw all-sensor read used mostly for bring-up.  The EKF scheduler uses the
     * smaller converted read functions so IMU and barometer can run at different
     * rates.
     */
    HAL_StatusTypeDef overall_status = HAL_OK;

    if (data == NULL || imu_status == NULL || mag_status == NULL || baro_status == NULL)
    {
        return HAL_ERROR;
    }

    *imu_status = LSM6DSV32X_ReadData(&rocket_imu, data->imu);
    if (*imu_status != HAL_OK)
    {
        overall_status = HAL_ERROR;
    }

    *mag_status = LIS2MDL_ReadData(&rocket_mag, data->mag);
    if (*mag_status != HAL_OK)
    {
        overall_status = HAL_ERROR;
    }

    *baro_status = BMP388_ReadRawData(&rocket_baro, data->baro);
    if (*baro_status != HAL_OK)
    {
        overall_status = HAL_ERROR;
    }

    return overall_status;
}

HAL_StatusTypeDef RocketSensors_ReadImuConverted(RocketSensorRawData_t *raw,
                                                 RocketSensorConvertedData_t *converted)
{
    /*
     * This path is called at the highest rate.  Keep it short: one IMU read,
     * unit conversion, and pad acceleration subtraction.
     */
    if (raw == NULL || converted == NULL)
    {
        return HAL_ERROR;
    }

    (void)LSM6DSV32X_ReadStatus(&rocket_imu, &s_last_imu_status);

    HAL_StatusTypeDef status = LSM6DSV32X_ReadData(&rocket_imu, raw->imu);
    if (status != HAL_OK)
    {
        converted->imu_valid = false;
        return status;
    }

    rocket_fill_converted_imu(raw, converted);
    return HAL_OK;
}

HAL_StatusTypeDef RocketSensors_ReadBarometerConverted(RocketSensorRawData_t *raw,
                                                       RocketSensorConvertedData_t *converted)
{
    /*
     * The BMP388 raw read is preserved in raw->baro before compensation so the
     * same sample can be used both for telemetry and EKF altitude correction.
     */
    if (raw == NULL || converted == NULL)
    {
        return HAL_ERROR;
    }

    (void)BMP388_ReadStatus(&rocket_baro, &s_last_baro_status);
    (void)BMP388_ReadError(&rocket_baro, &s_last_baro_error);

    HAL_StatusTypeDef status = BMP388_ReadRawData(&rocket_baro, raw->baro);
    if (status != HAL_OK)
    {
        converted->barometer_valid = false;
        return status;
    }

    status = BMP388_CompensateRawData(&rocket_baro,
                                      raw->baro[BMP388_RAW_PRESSURE],
                                      raw->baro[BMP388_RAW_TEMPERATURE],
                                      &converted->pressure_pa,
                                      &converted->temperature_c);
    if (status != HAL_OK || converted->pressure_pa <= 0.0f)
    {
        converted->barometer_valid = false;
        return (status == HAL_OK) ? HAL_ERROR : status;
    }

    converted->altitude_agl_m =
        rocket_altitude_from_pressure(converted->pressure_pa, s_pad_pressure_pa);
    converted->pad_pressure_pa = s_pad_pressure_pa;
    converted->barometer_valid = true;
    converted->pad_reference_valid = s_pad_reference_valid;

    return HAL_OK;
}

HAL_StatusTypeDef RocketSensors_ReadMagnetometerRaw(RocketSensorRawData_t *raw)
{
    /* Optional telemetry path; it never supplies a state to the vertical EKF. */
    if (raw == NULL)
    {
        return HAL_ERROR;
    }

#if AMBAR_FEATURE_MAGNETOMETER_TELEMETRY
    return LIS2MDL_ReadData(&rocket_mag, raw->mag);
#else
    memset(raw->mag, 0, sizeof(raw->mag));
    return HAL_ERROR;
#endif
}

HAL_StatusTypeDef RocketSensors_ReadAllConverted(RocketSensorRawData_t *raw,
                                                 RocketSensorConvertedData_t *converted)
{
    /* Diagnostic snapshot; scheduled flight code normally reads each rate separately. */
    if (raw == NULL || converted == NULL)
    {
        return HAL_ERROR;
    }

    memset(converted, 0, sizeof(*converted));

    HAL_StatusTypeDef imu_status = RocketSensors_ReadImuConverted(raw, converted);
    HAL_StatusTypeDef baro_status = RocketSensors_ReadBarometerConverted(raw, converted);
    HAL_StatusTypeDef mag_status = RocketSensors_ReadMagnetometerRaw(raw);

    converted->magnetometer_valid = mag_status == HAL_OK;

    /*
     * A magnetometer fault is visible in converted->magnetometer_valid, but it
     * does not make the vertical EKF sample invalid.  IMU and barometer remain
     * the required sensors for this first PCB estimator.
     */
    if (imu_status != HAL_OK || baro_status != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* ===================== EXTI DATA-READY ENTRY POINTS ===================== */

void RocketSensors_HandleExtiPin(uint16_t GPIO_Pin)
{
    /* Keep ISR work bounded: latch occurrence and count it; bus reads happen later. */
    if (GPIO_Pin == I2C1_INT_Pin)
    {
        s_imu_data_ready_seen = 1U;
        ++s_imu_data_ready_count;
    }
    else if (GPIO_Pin == BARO_INT_Pin)
    {
        s_baro_data_ready_seen = 1U;
        ++s_baro_data_ready_count;
    }
}

bool RocketSensors_ConsumeImuDataReady(void)
{
    /* Coalesces multiple interrupts into one pending scheduler hint. */
    if (s_imu_data_ready_seen == 0U)
    {
        return false;
    }

    s_imu_data_ready_seen = 0U;
    return true;
}

bool RocketSensors_ConsumeBarometerDataReady(void)
{
    /* Polling remains a fallback, so false is not itself a sensor fault. */
    if (s_baro_data_ready_seen == 0U)
    {
        return false;
    }

    s_baro_data_ready_seen = 0U;
    return true;
}
