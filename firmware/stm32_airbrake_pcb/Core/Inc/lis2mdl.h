/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 17:44:48 -04:00
 *
 * What this file does:
 *   This header describes the LIS2MDL magnetometer driver. The magnetometer is included for board health and telemetry, not for the first vertical EKF.
 *
 * Process flow:
 *   Startup verifies the I2C3 device and configures it for continuous readings. Telemetry can include raw X, Y, and Z magnetic readings.
 *
 * Main variables and what can be changed:
 *   The I2C address and register settings should only change if hardware or desired magnetometer mode changes.
 *
 * Assumptions:
 *   A magnetometer fault should be visible but should not stop the vertical IMU and barometer estimator.
 *
 * What is missing:
 *   No calibration, tilt-compensated heading, or attitude-estimator use is implemented.
 */

#ifndef LIS2MDL_H
#define LIS2MDL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>

/*
 * LIS2MDL magnetometer driver.
 *
 * The first vertical-only EKF does not use magnetic field data for estimation.
 * We still initialize and read the magnetometer so telemetry can show whether
 * the I2C3 device is alive and whether the PCB is seeing plausible field data.
 */

#define LIS2MDL_I2C_ADDR             0x1EU
#define LIS2MDL_WHO_AM_I_REG         0x4FU
#define LIS2MDL_WHO_AM_I_VALUE       0x40U

#define LIS2MDL_CFG_REG_A            0x60U
#define LIS2MDL_CFG_REG_B            0x61U
#define LIS2MDL_CFG_REG_C            0x62U
#define LIS2MDL_STATUS_REG           0x67U
#define LIS2MDL_OUTX_L_REG           0x68U

// Data order returned by LIS2MDL_ReadData()
typedef enum
{
    LIS2MDL_MAG_X = 0,
    LIS2MDL_MAG_Y = 1,
    LIS2MDL_MAG_Z = 2,
    LIS2MDL_DATA_COUNT = 3
} LIS2MDL_DataIndex_t;

typedef struct
{
    /* HAL I2C bus handle.  On this PCB the LIS2MDL is assigned to I2C3. */
    I2C_HandleTypeDef *hi2c;

    /* Unshifted 7-bit address used by the shared I2C helper macro. */
    uint8_t addr_7bit;

    /* Cached WHO_AM_I value for debugger/telemetry bring-up checks. */
    uint8_t who_am_i;
} LIS2MDL_HandleTypeDef;

/* Configure the magnetometer for continuous high-resolution measurements. */
HAL_StatusTypeDef LIS2MDL_Init(LIS2MDL_HandleTypeDef *dev);

/* Direct register checks used to separate wiring faults from data faults. */
HAL_StatusTypeDef LIS2MDL_ReadWhoAmI(LIS2MDL_HandleTypeDef *dev, uint8_t *who_am_i);
HAL_StatusTypeDef LIS2MDL_ReadStatus(LIS2MDL_HandleTypeDef *dev, uint8_t *status);

// Fills data[3] as: mag_x, mag_y, mag_z. Raw two's-complement magnetic readings.
HAL_StatusTypeDef LIS2MDL_ReadData(LIS2MDL_HandleTypeDef *dev, int16_t data[LIS2MDL_DATA_COUNT]);

#ifdef __cplusplus
}
#endif

#endif // LIS2MDL_H
