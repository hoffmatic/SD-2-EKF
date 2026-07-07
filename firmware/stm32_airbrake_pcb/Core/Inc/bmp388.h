/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 17:44:48 -04:00
 *
 * What this file does:
 *   This header describes the BMP388 barometer driver. The barometer provides pressure and temperature, which become altitude above the launch pad for EKF correction.
 *
 * Process flow:
 *   Startup verifies chip ID, resets the sensor, reads factory calibration values, and starts normal pressure and temperature mode. Later reads return raw data or compensated pressure and temperature.
 *
 * Main variables and what can be changed:
 *   The I2C address can change between 0x76 and 0x77 if the hardware strap changes. Oversampling and filter settings are in bmp388.c.
 *
 * Assumptions:
 *   Factory calibration registers are valid and must be read before compensated pressure is trusted.
 *
 * What is missing:
 *   No interrupt-driven data-ready path, pressure venting correction, or reference-sensor validation is included yet.
 */

#ifndef BMP388_H
#define BMP388_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>

/*
 * ===================== AMBAR EKF PCB INTEGRATION - UPDATED FILE =====================
 *
 * This driver still exposes the original raw 24-bit pressure/temperature words, but
 * it now also stores the BMP388 factory calibration constants and can return fully
 * compensated pressure in pascals plus temperature in degrees C.  The EKF scheduler
 * uses the compensated pressure so the flight layer can compute altitude above the
 * pad instead of trying to estimate from raw ADC counts.
 */

// 7-bit I2C addresses. Use the unshifted value in the handle.
// SDO = GND -> 0x76. SDO = VDDIO -> 0x77.
#define BMP388_I2C_ADDR_LOW          0x76U
#define BMP388_I2C_ADDR_HIGH         0x77U

#define BMP388_CHIP_ID_REG           0x00U
#define BMP388_CHIP_ID_VALUE         0x50U
#define BMP388_ERR_REG               0x02U
#define BMP388_STATUS_REG            0x03U
#define BMP388_DATA_0                0x04U
#define BMP388_PWR_CTRL              0x1BU
#define BMP388_OSR                   0x1CU
#define BMP388_ODR                   0x1DU
#define BMP388_CONFIG                0x1FU
#define BMP388_CALIB_DATA_START      0x31U
#define BMP388_CALIB_DATA_LEN        21U
#define BMP388_CMD                   0x7EU

#define BMP388_CMD_SOFT_RESET        0xB6U

// Data order returned by BMP388_ReadRawData()
typedef enum
{
    BMP388_RAW_PRESSURE = 0,
    BMP388_RAW_TEMPERATURE = 1,
    BMP388_DATA_COUNT = 2
} BMP388_DataIndex_t;

typedef struct
{
    /*
     * Calibration constants are stored as doubles because the Bosch compensation
     * equations use very small scale factors.  The public API returns floats so
     * the rest of the firmware remains light and consistent with the EKF.
     */
    double par_t1;
    double par_t2;
    double par_t3;
    double par_p1;
    double par_p2;
    double par_p3;
    double par_p4;
    double par_p5;
    double par_p6;
    double par_p7;
    double par_p8;
    double par_p9;
    double par_p10;
    double par_p11;
    double t_lin;
    uint8_t loaded;
} BMP388_CalibrationData_t;

typedef struct
{
    /* HAL I2C bus handle.  On the Airbrake PCB this is I2C2. */
    I2C_HandleTypeDef *hi2c;

    /* Unshifted 7-bit address.  BMP388 supports 0x76 or 0x77 depending on SDO. */
    uint8_t addr_7bit;

    /* Cached chip ID read at init for debugger inspection. */
    uint8_t chip_id;

    /* Factory trim data required for pressure/temperature compensation. */
    BMP388_CalibrationData_t calibration;
} BMP388_HandleTypeDef;

/* Reset, verify chip ID, load calibration, and start normal pressure/temp mode. */
HAL_StatusTypeDef BMP388_Init(BMP388_HandleTypeDef *dev);
HAL_StatusTypeDef BMP388_ReadChipID(BMP388_HandleTypeDef *dev, uint8_t *chip_id);
HAL_StatusTypeDef BMP388_ReadStatus(BMP388_HandleTypeDef *dev, uint8_t *status);
HAL_StatusTypeDef BMP388_ReadError(BMP388_HandleTypeDef *dev, uint8_t *error);

// Fills data[2] as: raw_pressure_24bit, raw_temperature_24bit.
HAL_StatusTypeDef BMP388_ReadRawData(BMP388_HandleTypeDef *dev,
                                     uint32_t data[BMP388_DATA_COUNT]);

/* BEGIN AMBAR EKF PCB INTEGRATION - NEW COMPENSATED BAROMETER API */
/*
 * Converts BMP388 raw pressure/temperature samples into real-world units using
 * the calibration block read during BMP388_Init().  Pressure is returned in Pa;
 * temperature is returned in degrees C.  These are the values used for pad-zeroed
 * altitude before the EKF barometer correction step.
 */
HAL_StatusTypeDef BMP388_CompensateRawData(BMP388_HandleTypeDef *dev,
                                           uint32_t raw_pressure,
                                           uint32_t raw_temperature,
                                           float *pressure_pa,
                                           float *temperature_c);
HAL_StatusTypeDef BMP388_ReadPressureTemperature(BMP388_HandleTypeDef *dev,
                                                 float *pressure_pa,
                                                 float *temperature_c);
/* END AMBAR EKF PCB INTEGRATION - NEW COMPENSATED BAROMETER API */

#ifdef __cplusplus
}
#endif

#endif // BMP388_H
