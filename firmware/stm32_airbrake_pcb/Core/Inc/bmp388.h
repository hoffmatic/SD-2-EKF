/**
 * @file bmp388.h
 * @brief BMP388 pressure/temperature driver and factory compensation state.
 *
 * Startup verifies the chip, performs a soft reset, loads all 21 factory trim
 * bytes, configures 50 Hz pressure/temperature sampling, and enters normal
 * mode.  Runtime callers may retain raw 24-bit values for logs or request
 * compensated pascals/degrees Celsius for pad-relative altitude and the EKF.
 *
 * Calibration must be loaded before compensated samples are accepted.  The
 * current implementation is polling-based and assumes the PCB's pressure port
 * exposes ambient pressure without a measured lag correction.  See
 * CODE_GUIDE.md [ARCH-2].
 */

#ifndef BMP388_H
#define BMP388_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h5xx_hal.h"
#include <stdint.h>

/* Seven-bit addresses stored unshifted in BMP388_HandleTypeDef. */
#define BMP388_I2C_ADDR_LOW          0x76U
#define BMP388_I2C_ADDR_HIGH         0x77U

/* Register map and immutable device identity. */
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

/** Stable array order returned by BMP388_ReadRawData(). */
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

/** @brief Reset, identify, calibrate, configure, and start the barometer. */
HAL_StatusTypeDef BMP388_Init(BMP388_HandleTypeDef *dev);
/** @brief Read the device identity register without changing configuration. */
HAL_StatusTypeDef BMP388_ReadChipID(BMP388_HandleTypeDef *dev, uint8_t *chip_id);
/** @brief Read BMP388 data-ready/status bits. */
HAL_StatusTypeDef BMP388_ReadStatus(BMP388_HandleTypeDef *dev, uint8_t *status);
/** @brief Read BMP388 command/configuration error bits. */
HAL_StatusTypeDef BMP388_ReadError(BMP388_HandleTypeDef *dev, uint8_t *error);

/** @brief Read raw unsigned 24-bit pressure and temperature ADC values. */
HAL_StatusTypeDef BMP388_ReadRawData(BMP388_HandleTypeDef *dev,
                                     uint32_t data[BMP388_DATA_COUNT]);

/**
 * @brief Apply cached Bosch trim coefficients to one raw sample pair.
 * @return HAL_ERROR when calibration has not been loaded or an output is NULL.
 */
HAL_StatusTypeDef BMP388_CompensateRawData(BMP388_HandleTypeDef *dev,
                                           uint32_t raw_pressure,
                                           uint32_t raw_temperature,
                                           float *pressure_pa,
                                           float *temperature_c);

/** @brief Read and compensate one pressure/temperature sample in a single call. */
HAL_StatusTypeDef BMP388_ReadPressureTemperature(BMP388_HandleTypeDef *dev,
                                                 float *pressure_pa,
                                                 float *temperature_c);

#ifdef __cplusplus
}
#endif

#endif /* BMP388_H */
