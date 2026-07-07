/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 17:44:48 -04:00
 *
 * What this file does:
 *   This file talks to the BMP388 over I2C2 and turns raw pressure counts into pressure in pascals and temperature in degrees C.
 *
 * Process flow:
 *   The driver checks the sensor, resets it, reads calibration numbers, sets oversampling, filter, and output rate, then enables normal mode. Each sample is read raw and compensated before altitude code uses it.
 *
 * Main variables and what can be changed:
 *   BMP388_OSR, BMP388_ODR, and BMP388_CONFIG writes control sample smoothness and rate. Tune them after measuring noise and delay.
 *
 * Assumptions:
 *   The compensation formulas match the Bosch datasheet. I2C2 is wired to address 0x76 unless the PCB strap changes.
 *
 * What is missing:
 *   No data-ready interrupt support, pressure-port calibration, or long-term drift correction beyond the EKF bias state is included.
 */

#include "bmp388.h"

#include <stddef.h>
#include <string.h>

/*
 * ===================== AMBAR EKF PCB INTEGRATION - UPDATED FILE =====================
 *
 * The original BMP388 support was enough for a chip-ID check and raw telemetry.
 * The vertical EKF needs pressure in pascals, so this file now reads the Bosch
 * calibration registers at boot and applies the datasheet compensation math to
 * every pressure/temperature sample used by the flight estimator.
 */

#define BMP388_I2C_TIMEOUT_MS        100U
#define BMP388_ADDR(dev)             ((uint16_t)((dev)->addr_7bit << 1))

static HAL_StatusTypeDef bmp388_write_u8(BMP388_HandleTypeDef *dev,
                                         uint8_t reg,
                                         uint8_t value)
{
    /* All writes go through this helper so the I2C address and timeout match. */
    if (dev == NULL || dev->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(dev->hi2c,
                             BMP388_ADDR(dev),
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             BMP388_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef bmp388_read_u8(BMP388_HandleTypeDef *dev,
                                        uint8_t reg,
                                        uint8_t *value)
{
    if (dev == NULL || dev->hi2c == NULL || value == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(dev->hi2c,
                            BMP388_ADDR(dev),
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            value,
                            1U,
                            BMP388_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef bmp388_read_bytes(BMP388_HandleTypeDef *dev,
                                           uint8_t start_reg,
                                           uint8_t *buffer,
                                           uint16_t len)
{
    /*
     * Used for both calibration and pressure/temperature data.  The BMP388 data
     * registers auto-increment during multi-byte reads.
     */
    if (dev == NULL || dev->hi2c == NULL || buffer == NULL || len == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(dev->hi2c,
                            BMP388_ADDR(dev),
                            start_reg,
                            I2C_MEMADD_SIZE_8BIT,
                            buffer,
                            len,
                            BMP388_I2C_TIMEOUT_MS);
}

static uint16_t bmp388_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[1] << 8U | data[0]);
}

static int16_t bmp388_i16_le(const uint8_t *data)
{
    return (int16_t)bmp388_u16_le(data);
}

static HAL_StatusTypeDef bmp388_read_calibration(BMP388_HandleTypeDef *dev)
{
    if (dev == NULL)
    {
        return HAL_ERROR;
    }

    uint8_t raw[BMP388_CALIB_DATA_LEN] = {0};
    HAL_StatusTypeDef status =
        bmp388_read_bytes(dev, BMP388_CALIB_DATA_START, raw, sizeof(raw));
    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * BEGIN AMBAR EKF PCB INTEGRATION - NEW CALIBRATION READBACK
     *
     * The BMP388 stores 21 bytes of trim data.  The bytes are little-endian and
     * mixed signed/unsigned, so decode them explicitly before scaling them into
     * the coefficients used by the Bosch compensation equations.
     */
    const uint16_t raw_t1 = bmp388_u16_le(&raw[0]);
    const uint16_t raw_t2 = bmp388_u16_le(&raw[2]);
    const int8_t raw_t3 = (int8_t)raw[4];
    const int16_t raw_p1 = bmp388_i16_le(&raw[5]);
    const int16_t raw_p2 = bmp388_i16_le(&raw[7]);
    const int8_t raw_p3 = (int8_t)raw[9];
    const int8_t raw_p4 = (int8_t)raw[10];
    const uint16_t raw_p5 = bmp388_u16_le(&raw[11]);
    const uint16_t raw_p6 = bmp388_u16_le(&raw[13]);
    const int8_t raw_p7 = (int8_t)raw[15];
    const int8_t raw_p8 = (int8_t)raw[16];
    const int16_t raw_p9 = bmp388_i16_le(&raw[17]);
    const int8_t raw_p10 = (int8_t)raw[19];
    const int8_t raw_p11 = (int8_t)raw[20];

    BMP388_CalibrationData_t *cal = &dev->calibration;
    memset(cal, 0, sizeof(*cal));

    cal->par_t1 = (double)raw_t1 / 256.0;
    cal->par_t2 = (double)raw_t2 / 1073741824.0;
    cal->par_t3 = (double)raw_t3 / 281474976710656.0;
    cal->par_p1 = ((double)raw_p1 - 16384.0) / 1048576.0;
    cal->par_p2 = ((double)raw_p2 - 16384.0) / 536870912.0;
    cal->par_p3 = (double)raw_p3 / 4294967296.0;
    cal->par_p4 = (double)raw_p4 / 137438953472.0;
    cal->par_p5 = (double)raw_p5 * 8.0;
    cal->par_p6 = (double)raw_p6 / 64.0;
    cal->par_p7 = (double)raw_p7 / 256.0;
    cal->par_p8 = (double)raw_p8 / 32768.0;
    cal->par_p9 = (double)raw_p9 / 281474976710656.0;
    cal->par_p10 = (double)raw_p10 / 281474976710656.0;
    cal->par_p11 = (double)raw_p11 / 36893488147419103232.0;
    cal->loaded = 1U;
    /* END AMBAR EKF PCB INTEGRATION - NEW CALIBRATION READBACK */

    return HAL_OK;
}

HAL_StatusTypeDef BMP388_ReadChipID(BMP388_HandleTypeDef *dev, uint8_t *chip_id)
{
    return bmp388_read_u8(dev, BMP388_CHIP_ID_REG, chip_id);
}

HAL_StatusTypeDef BMP388_ReadStatus(BMP388_HandleTypeDef *dev, uint8_t *status)
{
    return bmp388_read_u8(dev, BMP388_STATUS_REG, status);
}

HAL_StatusTypeDef BMP388_ReadError(BMP388_HandleTypeDef *dev, uint8_t *error)
{
    return bmp388_read_u8(dev, BMP388_ERR_REG, error);
}

HAL_StatusTypeDef BMP388_Init(BMP388_HandleTypeDef *dev)
{
    /*
     * The EKF must never use uncompensated pressure counts, so init only succeeds
     * after chip ID verification and calibration load both pass.
     */
    if (dev == NULL || dev->hi2c == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    status = HAL_I2C_IsDeviceReady(dev->hi2c, BMP388_ADDR(dev), 3U, BMP388_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BMP388_ReadChipID(dev, &dev->chip_id);
    if (status != HAL_OK)
    {
        return status;
    }
    if (dev->chip_id != BMP388_CHIP_ID_VALUE)
    {
        return HAL_ERROR;
    }

    status = bmp388_write_u8(dev, BMP388_CMD, BMP388_CMD_SOFT_RESET);
    if (status != HAL_OK) { return status; }
    HAL_Delay(10U);

    status = BMP388_ReadChipID(dev, &dev->chip_id);
    if (status != HAL_OK) { return status; }
    if (dev->chip_id != BMP388_CHIP_ID_VALUE) { return HAL_ERROR; }

    /*
     * BEGIN AMBAR EKF PCB INTEGRATION - NEW BOOT-TIME CALIBRATION LOAD
     *
     * Calibration must be read after reset and before any compensated samples.
     * If this fails, the barometer is treated as unavailable and the EKF remains
     * inhibited instead of using unscaled pressure counts.
     */
    status = bmp388_read_calibration(dev);
    if (status != HAL_OK) { return status; }
    /* END AMBAR EKF PCB INTEGRATION - NEW BOOT-TIME CALIBRATION LOAD */

    // OSR: pressure x4, temperature x2 -> osr_p=010, osr_t=001.
    status = bmp388_write_u8(dev, BMP388_OSR, 0x0AU);
    if (status != HAL_OK) { return status; }

    // ODR: 50 Hz.
    status = bmp388_write_u8(dev, BMP388_ODR, 0x02U);
    if (status != HAL_OK) { return status; }

    // CONFIG: IIR coefficient 3. Set to 0x00 if you want no pressure smoothing.
    status = bmp388_write_u8(dev, BMP388_CONFIG, 0x04U);
    if (status != HAL_OK) { return status; }

    // PWR_CTRL: press_en=1, temp_en=1, normal mode=11.
    status = bmp388_write_u8(dev, BMP388_PWR_CTRL, 0x33U);
    if (status != HAL_OK) { return status; }

    HAL_Delay(20U);
    return HAL_OK;
}

HAL_StatusTypeDef BMP388_ReadRawData(BMP388_HandleTypeDef *dev,
                                     uint32_t data[BMP388_DATA_COUNT])
{
    /*
     * BMP388 pressure and temperature are 24-bit unsigned little-endian ADC
     * values.  Compensation happens in BMP388_CompensateRawData().
     */
    if (data == NULL)
    {
        return HAL_ERROR;
    }

    uint8_t raw[6];
    HAL_StatusTypeDef status = bmp388_read_bytes(dev, BMP388_DATA_0, raw, sizeof(raw));
    if (status != HAL_OK)
    {
        return status;
    }

    data[BMP388_RAW_PRESSURE] = ((uint32_t)raw[2] << 16) | ((uint32_t)raw[1] << 8) | raw[0];
    data[BMP388_RAW_TEMPERATURE] = ((uint32_t)raw[5] << 16) | ((uint32_t)raw[4] << 8) | raw[3];

    return HAL_OK;
}

HAL_StatusTypeDef BMP388_CompensateRawData(BMP388_HandleTypeDef *dev,
                                           uint32_t raw_pressure,
                                           uint32_t raw_temperature,
                                           float *pressure_pa,
                                           float *temperature_c)
{
    if (dev == NULL || pressure_pa == NULL || temperature_c == NULL)
    {
        return HAL_ERROR;
    }

    BMP388_CalibrationData_t *cal = &dev->calibration;
    if (cal->loaded == 0U)
    {
        return HAL_ERROR;
    }

    /*
     * BEGIN AMBAR EKF PCB INTEGRATION - NEW PRESSURE COMPENSATION
     *
     * This follows the Bosch BMP388 compensation flow:
     *   1. Build a linearized temperature value, t_lin.
     *   2. Use t_lin and the raw pressure count to compensate pressure.
     *   3. Return practical float values to the rest of the flight code.
     *
     * Keeping this math in one function makes it easy to compare against Bosch
     * reference code during bench validation.
     */
    const double uncomp_temp = (double)raw_temperature;
    const double partial_t1 = uncomp_temp - cal->par_t1;
    const double partial_t2 = partial_t1 * cal->par_t2;
    cal->t_lin = partial_t2 + (partial_t1 * partial_t1) * cal->par_t3;

    const double t = cal->t_lin;
    const double uncomp_press = (double)raw_pressure;

    double partial_data1 = cal->par_p6 * t;
    double partial_data2 = cal->par_p7 * t * t;
    double partial_data3 = cal->par_p8 * t * t * t;
    const double partial_out1 = cal->par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = cal->par_p2 * t;
    partial_data2 = cal->par_p3 * t * t;
    partial_data3 = cal->par_p4 * t * t * t;
    const double partial_out2 =
        uncomp_press * (cal->par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = uncomp_press * uncomp_press;
    partial_data2 = cal->par_p9 + cal->par_p10 * t;
    partial_data3 = partial_data1 * partial_data2;
    const double partial_data4 =
        partial_data3 + uncomp_press * uncomp_press * uncomp_press * cal->par_p11;

    *temperature_c = (float)t;
    *pressure_pa = (float)(partial_out1 + partial_out2 + partial_data4);

    /* END AMBAR EKF PCB INTEGRATION - NEW PRESSURE COMPENSATION */
    return HAL_OK;
}

HAL_StatusTypeDef BMP388_ReadPressureTemperature(BMP388_HandleTypeDef *dev,
                                                 float *pressure_pa,
                                                 float *temperature_c)
{
    uint32_t raw[BMP388_DATA_COUNT] = {0};
    HAL_StatusTypeDef status = BMP388_ReadRawData(dev, raw);
    if (status != HAL_OK)
    {
        return status;
    }

    return BMP388_CompensateRawData(dev,
                                    raw[BMP388_RAW_PRESSURE],
                                    raw[BMP388_RAW_TEMPERATURE],
                                    pressure_pa,
                                    temperature_c);
}
