/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 17:44:48 -04:00
 *
 * What this file does:
 *   This file reads and configures the LIS2MDL magnetometer over I2C3 for health checks and telemetry.
 *
 * Process flow:
 *   The driver confirms the chip answers, checks WHO_AM_I, configures continuous conversion, then reads six output bytes for X, Y, and Z raw magnetic counts.
 *
 * Main variables and what can be changed:
 *   The configuration bytes in LIS2MDL_Init can be changed for data rate, filtering, or interrupt behavior after magnetometer testing.
 *
 * Assumptions:
 *   Raw counts are enough for first-board bring-up. The EKF does not depend on magnetic readings.
 *
 * What is missing:
 *   No hard-iron or soft-iron calibration, heading output, or flight-loop use exists yet.
 */

#include "lis2mdl.h"

/*
 * LIS2MDL low-level I2C driver.
 *
 * This driver is intentionally simple because the magnetometer is health and
 * telemetry only in the first EKF build.  The key requirement is to keep any
 * LIS2MDL failure visible without blocking the IMU/barometer vertical estimator.
 */

#define LIS2MDL_I2C_TIMEOUT_MS       100U
#define LIS2MDL_ADDR(dev)            ((uint16_t)((dev)->addr_7bit << 1))
#define LIS2MDL_AUTO_INC             0x80U

static HAL_StatusTypeDef lis2mdl_write_u8(LIS2MDL_HandleTypeDef *dev, uint8_t reg, uint8_t value)
{
    /* Shared byte-write helper keeps all LIS2MDL register access consistent. */
    if (dev == NULL || dev->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(dev->hi2c,
                             LIS2MDL_ADDR(dev),
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             LIS2MDL_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef lis2mdl_read_u8(LIS2MDL_HandleTypeDef *dev, uint8_t reg, uint8_t *value)
{
    /* Single-byte reads are enough for WHO_AM_I and STATUS diagnostics. */
    if (dev == NULL || dev->hi2c == NULL || value == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(dev->hi2c,
                            LIS2MDL_ADDR(dev),
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            value,
                            1U,
                            LIS2MDL_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef lis2mdl_read_bytes_auto_inc(LIS2MDL_HandleTypeDef *dev, uint8_t start_reg, uint8_t *buffer, uint16_t len)
{
    /*
     * LIS2MDL uses bit 7 of the sub-address to auto-increment.  Without that
     * bit, a six-byte read would repeatedly return the first output register.
     */
    if (dev == NULL || dev->hi2c == NULL || buffer == NULL || len == 0U)
    {
        return HAL_ERROR;
    }

    // For LIS2MDL, the MSB of the subaddress enables auto-increment during multiple-byte reads.
    return HAL_I2C_Mem_Read(dev->hi2c,
                            LIS2MDL_ADDR(dev),
                            (uint8_t)(start_reg | LIS2MDL_AUTO_INC),
                            I2C_MEMADD_SIZE_8BIT,
                            buffer,
                            len,
                            LIS2MDL_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef LIS2MDL_ReadWhoAmI(LIS2MDL_HandleTypeDef *dev, uint8_t *who_am_i)
{
    return lis2mdl_read_u8(dev, LIS2MDL_WHO_AM_I_REG, who_am_i);
}

HAL_StatusTypeDef LIS2MDL_ReadStatus(LIS2MDL_HandleTypeDef *dev, uint8_t *status)
{
    return lis2mdl_read_u8(dev, LIS2MDL_STATUS_REG, status);
}

HAL_StatusTypeDef LIS2MDL_Init(LIS2MDL_HandleTypeDef *dev)
{
    /*
     * Bring-up sequence:
     *   1. Confirm the device ACKs on I2C3.
     *   2. Confirm WHO_AM_I matches the LIS2MDL.
     *   3. Put the part into continuous conversion for periodic telemetry.
     */
    if (dev == NULL || dev->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    status = HAL_I2C_IsDeviceReady(dev->hi2c, LIS2MDL_ADDR(dev), 3U, LIS2MDL_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }

    status = LIS2MDL_ReadWhoAmI(dev, &dev->who_am_i);
    if (status != HAL_OK)
    {
        return status;
    }
    if (dev->who_am_i != LIS2MDL_WHO_AM_I_VALUE)
    {
        return HAL_ERROR;
    }

    // CFG_REG_A:
    // COMP_TEMP_EN=1, high-resolution mode, ODR=100 Hz, continuous-conversion mode.
    status = lis2mdl_write_u8(dev, LIS2MDL_CFG_REG_A, 0x8CU);
    if (status != HAL_OK) { return status; }

    // CFG_REG_B: enable offset cancellation. Keep LPF disabled for simpler raw response.
    status = lis2mdl_write_u8(dev, LIS2MDL_CFG_REG_B, 0x02U);
    if (status != HAL_OK) { return status; }

    // CFG_REG_C: BDU=1. Do not route DRDY to INT pin by default.
    status = lis2mdl_write_u8(dev, LIS2MDL_CFG_REG_C, 0x10U);
    if (status != HAL_OK) { return status; }

    HAL_Delay(20U);
    return HAL_OK;
}

HAL_StatusTypeDef LIS2MDL_ReadData(LIS2MDL_HandleTypeDef *dev, int16_t data[LIS2MDL_DATA_COUNT])
{
    /*
     * The output registers are little-endian X/Y/Z magnetic readings.  They are
     * left as raw counts so future calibration can be applied in one place.
     */
    if (data == NULL)
    {
        return HAL_ERROR;
    }

    uint8_t raw[6];
    HAL_StatusTypeDef status = lis2mdl_read_bytes_auto_inc(dev, LIS2MDL_OUTX_L_REG, raw, sizeof(raw));
    if (status != HAL_OK)
    {
        return status;
    }

    data[LIS2MDL_MAG_X] = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);
    data[LIS2MDL_MAG_Y] = (int16_t)((uint16_t)raw[3] << 8 | raw[2]);
    data[LIS2MDL_MAG_Z] = (int16_t)((uint16_t)raw[5] << 8 | raw[4]);

    return HAL_OK;
}
