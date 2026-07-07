/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 17:44:48 -04:00
 *
 * What this file does:
 *   This file talks to the LSM6DSV32X IMU over I2C1. It configures the sensor and reads raw gyro and accelerometer data.
 *
 * Process flow:
 *   Startup confirms the IMU, checks WHO_AM_I, resets the chip, waits for reset to finish, sets data update behavior, then sets accel and gyro ranges and 120 Hz output rates.
 *
 * Main variables and what can be changed:
 *   CTRL1, CTRL2, CTRL6, and CTRL8 writes set rate and scale. Change them only if the desired IMU rate or sensor range changes.
 *
 * Assumptions:
 *   The EKF receives converted, pad-zeroed vertical acceleration from rocket_sensors.c, not directly from this driver.
 *
 * What is missing:
 *   No data-ready interrupt, FIFO, self-test, or gyro-based attitude estimate is implemented.
 */

#include "lsm6dsv32x.h"

/*
 * LSM6DSV32X low-level I2C driver.
 *
 * The EKF only consumes vertical acceleration, but keeping all six raw channels
 * available makes the radio packet useful for board orientation checks and bench
 * debugging.  This file deliberately avoids EKF math; it only speaks sensor
 * registers and returns raw counts.
 */

#define LSM6DSV32X_I2C_TIMEOUT_MS    100U
#define LSM6DSV32X_ADDR(dev)         ((uint16_t)((dev)->addr_7bit << 1))

static HAL_StatusTypeDef lsm6_write_u8(LSM6DSV32X_HandleTypeDef *dev, uint8_t reg, uint8_t value)
{
    /*
     * All register writes go through one helper so NULL checks and HAL address
     * shifting stay consistent.  HAL expects the 7-bit address shifted left.
     */
    if (dev == NULL || dev->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(dev->hi2c,
                             LSM6DSV32X_ADDR(dev),
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             LSM6DSV32X_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef lsm6_read_u8(
    LSM6DSV32X_HandleTypeDef *dev,
    uint8_t reg,
    uint8_t *value
)
{
    /* Single-byte register reads are used for WHO_AM_I, STATUS, and reset polling. */
    if (dev == NULL || dev->hi2c == NULL || value == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(dev->hi2c,
                            LSM6DSV32X_ADDR(dev),
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            value,
                            1U,
                            LSM6DSV32X_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef LSM6DSV32X_ReadWhoAmI(
    LSM6DSV32X_HandleTypeDef *dev,
    uint8_t *who_am_i
)
{
    if (who_am_i == NULL)
    {
        return HAL_ERROR;
    }

    return lsm6_read_u8(dev, LSM6DSV32X_WHO_AM_I_REG, who_am_i);
}

HAL_StatusTypeDef LSM6DSV32X_ReadStatus(
    LSM6DSV32X_HandleTypeDef *dev,
    uint8_t *status
)
{
    if (status == NULL)
    {
        return HAL_ERROR;
    }

    return lsm6_read_u8(dev, LSM6DSV32X_STATUS_REG, status);
}

static HAL_StatusTypeDef lsm6_read_bytes(LSM6DSV32X_HandleTypeDef *dev, uint8_t start_reg, uint8_t *buffer, uint16_t len)
{
    /*
     * Multi-byte reads rely on IF_INC being enabled in CTRL3 during init.  That
     * lets one I2C transaction pull gyro and accelerometer outputs in timestamp
     * order with less bus overhead.
     */
    if (dev == NULL || dev->hi2c == NULL || buffer == NULL || len == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(dev->hi2c,
                            LSM6DSV32X_ADDR(dev),
                            start_reg,
                            I2C_MEMADD_SIZE_8BIT,
                            buffer,
                            len,
                            LSM6DSV32X_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef LSM6DSV32X_Init(LSM6DSV32X_HandleTypeDef *dev)
{
    /*
     * Bring-up order:
     *   1. Confirm the device responds on I2C1.
     *   2. Verify WHO_AM_I so a wiring/address fault is caught immediately.
     *   3. Reset the sensor and wait for the reset bit to clear.
     *   4. Configure block-data update, auto-increment, scale, and ODR.
     */
    if (dev == NULL || dev->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    status = HAL_I2C_IsDeviceReady(dev->hi2c, LSM6DSV32X_ADDR(dev), 3U, LSM6DSV32X_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }

    status = LSM6DSV32X_ReadWhoAmI(dev, &dev->who_am_i);
    if (status != HAL_OK)
    {
        return status;
    }
    if (dev->who_am_i != LSM6DSV32X_WHO_AM_I_VALUE)
    {
        return HAL_ERROR;
    }

    // Software reset: CTRL3 bit0 = SW_RESET.
    status = lsm6_write_u8(dev, LSM6DSV32X_CTRL3, 0x01U);
    if (status != HAL_OK)
    {
        return status;
    }

    uint32_t reset_start = HAL_GetTick();
    uint8_t ctrl3 = 0x01U;

    do
    {
        /*
         * The reset bit is self-clearing.  Polling here prevents later register
         * writes from racing the sensor while it is still resetting.
         */
        HAL_Delay(1U);

        status = lsm6_read_u8(dev, LSM6DSV32X_CTRL3, &ctrl3);
        if (status != HAL_OK)
        {
            return status;
        }

        if ((ctrl3 & 0x01U) == 0U)
        {
            break;
        }

    } while ((HAL_GetTick() - reset_start) < 100U);

    if ((ctrl3 & 0x01U) != 0U)
    {
        return HAL_TIMEOUT;
    }

    // CTRL3: BDU=1, IF_INC=1. Reserved bits remain 0.
    status = lsm6_write_u8(dev, LSM6DSV32X_CTRL3, 0x44U);
    if (status != HAL_OK) { return status; }

    // CTRL8: bit2 must be 1. FS_XL[1:0] = 11 -> +/-32 g.
    status = lsm6_write_u8(dev, LSM6DSV32X_CTRL8, 0x07U);
    if (status != HAL_OK) { return status; }

    // CTRL6: FS_G[3:0] = 1100 -> +/-4000 dps. LPF bits left 0.
    status = lsm6_write_u8(dev, LSM6DSV32X_CTRL6, 0x0CU);
    if (status != HAL_OK) { return status; }

    // CTRL1: accel high-performance mode, ODR_XL = 120 Hz.
    status = lsm6_write_u8(dev, LSM6DSV32X_CTRL1, 0x06U);
    if (status != HAL_OK) { return status; }

    // CTRL2: gyro high-performance mode, ODR_G = 120 Hz.
    status = lsm6_write_u8(dev, LSM6DSV32X_CTRL2, 0x06U);
    if (status != HAL_OK) { return status; }

    HAL_Delay(50U);
    return HAL_OK;
}

HAL_StatusTypeDef LSM6DSV32X_ReadData(LSM6DSV32X_HandleTypeDef *dev, int16_t data[LSM6DSV32X_DATA_COUNT])
{
    /*
     * Read order begins at OUTX_L_G, so the 12 bytes are:
     * gyro X/Y/Z followed by accel X/Y/Z, little-endian two's complement.
     * rocket_sensors.c converts these counts into dps and m/s^2.
     */
    if (data == NULL)
    {
        return HAL_ERROR;
    }

    uint8_t raw[12];
    HAL_StatusTypeDef status = lsm6_read_bytes(dev, LSM6DSV32X_OUTX_L_G, raw, sizeof(raw));
    if (status != HAL_OK)
    {
        return status;
    }

    data[LSM6DSV32X_GYRO_X]  = (int16_t)((uint16_t)raw[1]  << 8 | raw[0]);
    data[LSM6DSV32X_GYRO_Y]  = (int16_t)((uint16_t)raw[3]  << 8 | raw[2]);
    data[LSM6DSV32X_GYRO_Z]  = (int16_t)((uint16_t)raw[5]  << 8 | raw[4]);
    data[LSM6DSV32X_ACCEL_X] = (int16_t)((uint16_t)raw[7]  << 8 | raw[6]);
    data[LSM6DSV32X_ACCEL_Y] = (int16_t)((uint16_t)raw[9]  << 8 | raw[8]);
    data[LSM6DSV32X_ACCEL_Z] = (int16_t)((uint16_t)raw[11] << 8 | raw[10]);

    return HAL_OK;
}

HAL_StatusTypeDef LSM6DSV32X_RunBasicSelfTest(LSM6DSV32X_HandleTypeDef *dev)
{
    /*
     * BEGIN AMBAR BENCH-GATED EXPANSION - BASIC IMU HEALTH CHECK
     *
     * This is not the factory electro-mechanical self-test sequence.  It is a
     * non-disruptive bench health check that confirms WHO_AM_I still matches,
     * STATUS is readable, and at least one accelerometer axis is producing a
     * nonzero count.  It avoids changing sensor modes during EKF bring-up.
     */
    uint8_t who = 0U;
    uint8_t status = 0U;
    int16_t data[LSM6DSV32X_DATA_COUNT] = {0};

    if (LSM6DSV32X_ReadWhoAmI(dev, &who) != HAL_OK || who != LSM6DSV32X_WHO_AM_I_VALUE)
    {
        return HAL_ERROR;
    }

    if (LSM6DSV32X_ReadStatus(dev, &status) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (LSM6DSV32X_ReadData(dev, data) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (data[LSM6DSV32X_ACCEL_X] == 0
        && data[LSM6DSV32X_ACCEL_Y] == 0
        && data[LSM6DSV32X_ACCEL_Z] == 0)
    {
        return HAL_ERROR;
    }

    (void)status;
    return HAL_OK;
    /* END AMBAR BENCH-GATED EXPANSION - BASIC IMU HEALTH CHECK */
}
