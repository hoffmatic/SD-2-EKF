/**
 * @file lsm6dsv32x.h
 * @brief LSM6DSV32X IMU driver for raw gyro/accelerometer acquisition.
 *
 * The IMU is the high-rate propagation source for the vertical EKF.  This layer
 * only identifies/configures the device and returns chip-axis signed counts;
 * rocket_sensors.c owns scale conversion, board orientation, gravity/pad
 * reference, and vertical-axis selection.  The current polling implementation
 * does not use FIFO, DRDY interrupts, gyro integration, or a full attitude
 * estimator.  See CODE_GUIDE.md [ARCH-2].
 */

#ifndef LSM6DSV32X_H
#define LSM6DSV32X_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>

/* Seven-bit addresses stored unshifted; SA0 selects low or high. */
#define LSM6DSV32X_I2C_ADDR_LOW      0x6AU
#define LSM6DSV32X_I2C_ADDR_HIGH     0x6BU

#define LSM6DSV32X_WHO_AM_I_REG      0x0FU
#define LSM6DSV32X_WHO_AM_I_VALUE    0x70U

#define LSM6DSV32X_STATUS_REG        0x1EU
#define LSM6DSV32X_OUT_TEMP_L        0x20U
#define LSM6DSV32X_OUTX_L_G          0x22U
#define LSM6DSV32X_OUTX_L_A          0x28U

#define LSM6DSV32X_CTRL1             0x10U
#define LSM6DSV32X_CTRL2             0x11U
#define LSM6DSV32X_CTRL3             0x12U
#define LSM6DSV32X_CTRL6             0x15U
#define LSM6DSV32X_CTRL8             0x17U

/** Stable raw-count order returned by LSM6DSV32X_ReadData(). */
typedef enum
{
    LSM6DSV32X_GYRO_X = 0,
    LSM6DSV32X_GYRO_Y = 1,
    LSM6DSV32X_GYRO_Z = 2,
    LSM6DSV32X_ACCEL_X = 3,
    LSM6DSV32X_ACCEL_Y = 4,
    LSM6DSV32X_ACCEL_Z = 5,
    LSM6DSV32X_DATA_COUNT = 6
} LSM6DSV32X_DataIndex_t;

typedef struct
{
    /* HAL I2C bus handle.  On the Airbrake PCB this is I2C1. */
    I2C_HandleTypeDef *hi2c;

    /* Unshifted 7-bit I2C address.  The HAL helper shifts it when used. */
    uint8_t addr_7bit;

    /* Cached WHO_AM_I value read during init for quick debugger inspection. */
    uint8_t who_am_i;
} LSM6DSV32X_HandleTypeDef;

/** @brief Configure 120 Hz, +/-32 g acceleration, and +/-4000 dps gyro. */
HAL_StatusTypeDef LSM6DSV32X_Init(LSM6DSV32X_HandleTypeDef *dev);

/** @brief Read the identity register for bring-up/fault isolation. */
HAL_StatusTypeDef LSM6DSV32X_ReadWhoAmI(LSM6DSV32X_HandleTypeDef *dev, uint8_t *who_am_i);
/** @brief Read current data-ready/status bits. */
HAL_StatusTypeDef LSM6DSV32X_ReadStatus(LSM6DSV32X_HandleTypeDef *dev, uint8_t *status);
/** @brief Non-disruptive identity/status/nonzero-data health check. */
HAL_StatusTypeDef LSM6DSV32X_RunBasicSelfTest(LSM6DSV32X_HandleTypeDef *dev);

/** @brief Read raw gyro X/Y/Z followed by accelerometer X/Y/Z counts. */
HAL_StatusTypeDef LSM6DSV32X_ReadData(
    LSM6DSV32X_HandleTypeDef *dev,
    int16_t data[LSM6DSV32X_DATA_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSV32X_H */
