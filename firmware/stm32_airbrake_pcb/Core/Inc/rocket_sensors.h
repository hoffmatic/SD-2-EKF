/*
 * AMBAR BOARD SENSOR ADAPTER - PUBLIC INTERFACE
 *
 * Purpose and ownership
 *   Groups the three board sensor drivers and exposes both raw bus values and
 *   converted SI-unit samples.  This layer owns rocket-axis selection, stationary
 *   pad acceleration, pressure zero, and EXTI data-ready hints; the EKF never
 *   reads a sensor driver directly.
 *
 * Sensor flow
 *   Init starts the LSM6DSV32X IMU and BMP388 barometer required by the vertical
 *   estimate; LIS2MDL is optional telemetry.  ResetPadReference, or validated
 *   stored configuration, establishes the common pad zero.  Rate-specific read
 *   calls then supply upward acceleration and altitude AGL to AmbarApp.  See
 *   CODE_GUIDE.md [ARCH-2].
 *
 * Section map
 *   1. Compile-time orientation fallback and pad-capture default
 *   2. Board-owned driver handles
 *   3. Raw, converted, and calibration-status snapshots
 *   4. Initialization/calibration API
 *   5. Rate-specific and diagnostic read API
 *   6. EXTI data-ready API
 *
 * Safety and assumptions
 *   Bench-verify the axis/sign with the installed PCB orientation and capture the
 *   pad reference only while upright and motionless.  A valid converted sample
 *   does not by itself prove the pad reference is valid; inspect the accompanying
 *   flag.  Magnetometer health is reported but does not gate the vertical EKF.
 *   There is no automatic orientation detection, redundant sensor voting, or
 *   pressure-vent dynamics model.
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

/* ===================== ORIENTATION AND PAD-CAPTURE DEFAULTS ===================== */

/* Saved runtime config may override this fallback after boot. */
#ifndef ROCKET_IMU_VERTICAL_ACCEL_INDEX
#define ROCKET_IMU_VERTICAL_ACCEL_INDEX LSM6DSV32X_ACCEL_Z
#endif

#ifndef ROCKET_IMU_VERTICAL_ACCEL_SIGN
#define ROCKET_IMU_VERTICAL_ACCEL_SIGN 1.0f
#endif

#define ROCKET_PAD_REFERENCE_DEFAULT_SAMPLES 32U

/* ===================== BOARD-OWNED DRIVER HANDLES ===================== */

extern LSM6DSV32X_HandleTypeDef rocket_imu;
extern LIS2MDL_HandleTypeDef rocket_mag;
extern BMP388_HandleTypeDef rocket_baro;

/* ===================== SENSOR SNAPSHOTS ===================== */

typedef struct
{
    /*
     * Raw telemetry mirrors the physical bus reads.  These fields are useful for
     * checking byte order, sensor saturation, and wiring before trusting the EKF.
     */
    int16_t imu[LSM6DSV32X_DATA_COUNT];     /* gyro_x/y/z, accel_x/y/z */
    int16_t mag[LIS2MDL_DATA_COUNT];        /* mag_x/y/z */
    uint32_t baro[BMP388_DATA_COUNT];       /* raw_pressure, raw_temperature */
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
    /* Persistable orientation/pad values plus live interrupt/status diagnostics. */
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

/* ===================== INITIALIZATION AND CALIBRATION API ===================== */

/* Required IMU/barometer failures return HAL_ERROR; magnetometer is optional. */
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

/* ===================== SENSOR READ API ===================== */

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

/* ===================== EXTI DATA-READY API ===================== */

/* EXTI-assisted data-ready flags.  Scheduler polling remains as fallback. */
void RocketSensors_HandleExtiPin(uint16_t GPIO_Pin);
bool RocketSensors_ConsumeImuDataReady(void);
bool RocketSensors_ConsumeBarometerDataReady(void);

#ifdef __cplusplus
}
#endif

#endif // ROCKET_SENSORS_H
