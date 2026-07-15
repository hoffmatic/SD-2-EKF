/**
 * @file lis2mdl.h
 * @brief LIS2MDL magnetometer health/telemetry driver.
 *
 * The driver verifies the I2C3 device, configures continuous high-resolution
 * conversion, and returns raw signed X/Y/Z counts.  Magnetic data is retained
 * for board health and future attitude work; it does not enter the current
 * vertical EKF.  A LIS2MDL failure therefore remains visible without stopping
 * IMU/barometer estimation.  No hard/soft-iron calibration or heading solution
 * is provided yet.  See CODE_GUIDE.md [ARCH-2].
 */

#ifndef LIS2MDL_H
#define LIS2MDL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>

/* Device identity and register map. */
#define LIS2MDL_I2C_ADDR             0x1EU
#define LIS2MDL_WHO_AM_I_REG         0x4FU
#define LIS2MDL_WHO_AM_I_VALUE       0x40U

#define LIS2MDL_CFG_REG_A            0x60U
#define LIS2MDL_CFG_REG_B            0x61U
#define LIS2MDL_CFG_REG_C            0x62U
#define LIS2MDL_STATUS_REG           0x67U
#define LIS2MDL_OUTX_L_REG           0x68U

/** Stable raw-count array order returned by LIS2MDL_ReadData(). */
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

/** @brief Identify and configure continuous high-resolution measurements. */
HAL_StatusTypeDef LIS2MDL_Init(LIS2MDL_HandleTypeDef *dev);

/** @brief Read and return WHO_AM_I for wiring/identity diagnosis. */
HAL_StatusTypeDef LIS2MDL_ReadWhoAmI(LIS2MDL_HandleTypeDef *dev, uint8_t *who_am_i);
/** @brief Read data-ready/overrun status bits. */
HAL_StatusTypeDef LIS2MDL_ReadStatus(LIS2MDL_HandleTypeDef *dev, uint8_t *status);

/** @brief Read raw two's-complement X/Y/Z magnetic counts. */
HAL_StatusTypeDef LIS2MDL_ReadData(
    LIS2MDL_HandleTypeDef *dev,
    int16_t data[LIS2MDL_DATA_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* LIS2MDL_H */
