/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 17:44:48 -04:00
 *
 * What this file does:
 *   This header describes the board-level sensor layer. It gathers IMU, barometer, and magnetometer drivers and exposes raw and converted readings for the flight app.
 *
 * Process flow:
 *   Sensors initialize, a pad reference is captured, then IMU and barometer readings convert into EKF-ready units. Raw readings remain available for telemetry and debugging.
 *
 * Main variables and what can be changed:
 *   ROCKET_IMU_VERTICAL_ACCEL_INDEX and ROCKET_IMU_VERTICAL_ACCEL_SIGN are the main orientation settings. They must be checked on the bench with the PCB in rocket orientation.
 *
 * Assumptions:
 *   The rocket is still during pad reset. The first EKF uses only vertical IMU acceleration and BMP388 altitude; LIS2MDL remains telemetry and health only.
 *
 * What is missing:
 *   No automatic orientation detection, sensor interrupt scheduling, stored calibration, or sensor redundancy is implemented.
 */

#ifndef ROCKET_SENSORS_H
#define ROCKET_SENSORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lsm6dsv32x.h"
#include "lis2mdl.h"
#include "bmp388.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * ===================== AMBAR EKF PCB INTEGRATION - UPDATED FILE =====================
 *
 * The flight estimator needs converted SI-unit samples, not just raw sensor words.
 * This header keeps the raw telemetry path intact and adds a converted-data path:
 *   - LSM6DSV32X accelerometer raw counts -> m/s^2 at the configured 32 g scale.
 *   - One configured IMU axis/sign -> rocket vertical acceleration.
 *   - Pad reset -> subtract stationary acceleration and pressure baseline.
 *   - BMP388 compensated pressure -> altitude above ground level.
 *
 * The vertical-axis constants are intentionally configurable.  Before flight, put
 * the PCB in the rocket orientation, run the pad reset, and verify stationary
 * vertical acceleration is close to zero in telemetry.
 */

#ifndef ROCKET_IMU_VERTICAL_ACCEL_INDEX
#define ROCKET_IMU_VERTICAL_ACCEL_INDEX LSM6DSV32X_ACCEL_Z
#endif

#ifndef ROCKET_IMU_VERTICAL_ACCEL_SIGN
#define ROCKET_IMU_VERTICAL_ACCEL_SIGN 1.0f
#endif

#define ROCKET_PAD_REFERENCE_DEFAULT_SAMPLES 32U

extern LSM6DSV32X_HandleTypeDef rocket_imu;
extern LIS2MDL_HandleTypeDef rocket_mag;
extern BMP388_HandleTypeDef rocket_baro;

typedef struct
{
    /*
     * Raw telemetry mirrors the physical bus reads.  These fields are useful for
     * checking byte order, sensor saturation, and wiring before trusting the EKF.
     */
    int16_t imu[LSM6DSV32X_DATA_COUNT];     // gyro_x/y/z, accel_x/y/z
    int16_t mag[LIS2MDL_DATA_COUNT];        // mag_x/y/z
    uint32_t baro[BMP388_DATA_COUNT];       // raw_pressure, raw_temperature
} RocketSensorRawData_t;

typedef struct
{
    /*
     * All values below are SI units or explicit engineering units.  These are the
     * values passed into the EKF and appended to telemetry after fixed-point
     * packing in radio_bridge.c.
     */
    float gyro_dps[3];
    float accel_mps2[3];
    float vertical_acceleration_mps2;
    float pressure_pa;
    float temperature_c;
    float altitude_agl_m;
    float pad_pressure_pa;
    bool imu_valid;
    bool barometer_valid;
    bool magnetometer_valid;
    bool pad_reference_valid;
} RocketSensorConvertedData_t;

typedef struct
{
    int32_t vertical_axis_index;
    float vertical_axis_sign;
    float pad_vertical_accel_mps2;
    float pad_pressure_pa;
    uint32_t imu_data_ready_count;
    uint32_t barometer_data_ready_count;
    uint8_t imu_status_reg;
    uint8_t barometer_status_reg;
    uint8_t barometer_error_reg;
    bool pad_reference_valid;
} RocketSensorCalibrationStatus_t;

HAL_StatusTypeDef RocketSensors_Init(void);

/* Runtime orientation/config hooks loaded from ambar_config. */
HAL_StatusTypeDef RocketSensors_ApplyOrientationConfig(int32_t axis_index, float axis_sign);
HAL_StatusTypeDef RocketSensors_SetPadReference(float vertical_accel_mps2, float pressure_pa);
RocketSensorCalibrationStatus_t RocketSensors_GetCalibrationStatus(void);

/*
 * Capture stationary pad acceleration and pressure.  Call this while the rocket
 * is upright and motionless; otherwise the vertical EKF will start with a bad
 * acceleration or altitude zero.
 */
HAL_StatusTypeDef RocketSensors_ResetPadReference(uint16_t sample_count);

/* Read all raw sensors once for telemetry. */
HAL_StatusTypeDef RocketSensors_ReadAll(
    RocketSensorRawData_t *data,
    HAL_StatusTypeDef *imu_status,
    HAL_StatusTypeDef *mag_status,
    HAL_StatusTypeDef *baro_status
);

HAL_StatusTypeDef RocketSensors_ReadImuConverted(RocketSensorRawData_t *raw,
                                                 RocketSensorConvertedData_t *converted);

/* Read/convert BMP388 pressure and altitude AGL using the stored pad baseline. */
HAL_StatusTypeDef RocketSensors_ReadBarometerConverted(RocketSensorRawData_t *raw,
                                                       RocketSensorConvertedData_t *converted);

/* Read magnetometer raw counts for health/telemetry; not used by vertical EKF. */
HAL_StatusTypeDef RocketSensors_ReadMagnetometerRaw(RocketSensorRawData_t *raw);

/* Convenience helper for diagnostics that want one full converted snapshot. */
HAL_StatusTypeDef RocketSensors_ReadAllConverted(RocketSensorRawData_t *raw,
                                                 RocketSensorConvertedData_t *converted);

/* EXTI-assisted data-ready flags.  Scheduler polling remains as fallback. */
void RocketSensors_HandleExtiPin(uint16_t GPIO_Pin);
bool RocketSensors_ConsumeImuDataReady(void);
bool RocketSensors_ConsumeBarometerDataReady(void);

#ifdef __cplusplus
}
#endif

#endif // ROCKET_SENSORS_H
