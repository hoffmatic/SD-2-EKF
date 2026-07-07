/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 17:44:48 -04:00
 *
 * What this file does:
 *   This header describes the LSM6DSV32X IMU driver. The IMU provides high-rate acceleration used to move the vertical EKF forward in time.
 *
 * Process flow:
 *   The driver initializes the IMU on I2C1, configures accel and gyro output, and reads raw gyro and accel counts. Unit conversion happens later in rocket_sensors.c.
 *
 * Main variables and what can be changed:
 *   The I2C address can change if SA0 is strapped differently. Full-scale ranges and output rates are configured in lsm6dsv32x.c.
 *
 * Assumptions:
 *   Board orientation is handled outside this driver. This driver reports sensor axes as the chip provides them.
 *
 * What is missing:
 *   No data-ready interrupt path, FIFO use, gyro integration, or full attitude estimator is implemented.
 */

#ifndef LSM6DSV32X_H
#define LSM6DSV32X_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>

/*
 * LSM6DSV32X IMU driver.
 *
 * This is the primary propagation sensor for the vertical EKF.  The driver
 * intentionally returns raw signed 16-bit gyro/accelerometer words so telemetry
 * can show the exact sensor output.  Unit conversion lives in rocket_sensors.c,
 * where the board orientation and pad-zero reference are handled.
 */

// 7-bit I2C addresses. Use the unshifted value in the handle.
// SA0 = 0 -> 0x6A. SA0 = 1 -> 0x6B.
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

// Data order returned by LSM6DSV32X_ReadData()
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

/* Configure the IMU for 120 Hz accel/gyro, +/-32 g accel, and +/-4000 dps gyro. */
HAL_StatusTypeDef LSM6DSV32X_Init(LSM6DSV32X_HandleTypeDef *dev);

/* Lightweight register checks used during bring-up and fault isolation. */
HAL_StatusTypeDef LSM6DSV32X_ReadWhoAmI(LSM6DSV32X_HandleTypeDef *dev, uint8_t *who_am_i);
HAL_StatusTypeDef LSM6DSV32X_ReadStatus(LSM6DSV32X_HandleTypeDef *dev, uint8_t *status);
HAL_StatusTypeDef LSM6DSV32X_RunBasicSelfTest(LSM6DSV32X_HandleTypeDef *dev);

// Fills data[6] as: gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z.
// Raw two's-complement values. Convert using the full-scale settings you selected.
HAL_StatusTypeDef LSM6DSV32X_ReadData(LSM6DSV32X_HandleTypeDef *dev, int16_t data[LSM6DSV32X_DATA_COUNT]);

#ifdef __cplusplus
}
#endif

#endif // LSM6DSV32X_H
