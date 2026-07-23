/**
 * @file tmc5240.c
 * @brief TMC5240 SPI transport, board bring-up, and ramp-register operations.
 *
 * IMPLEMENTATION MAP
 * ------------------
 *  1. Convert requested current labels into bounded register fields.
 *  2. Transfer five-byte SPI datagrams and handle pipelined register reads.
 *  3. Keep outputs disabled while checking identity and loading a profile.
 *  4. Expose position/status helpers to the actuator safety state machine.
 *
 * The permission to move is deliberately outside this file.  See
 * CODE_GUIDE.md [ARCH-5] and ambar_actuator.c for homing, travel clamps,
 * phase gates, progress monitoring, and fault behavior.
 */
#include "tmc5240.h"
#include "ambar_features.h"

extern SPI_HandleTypeDef hspi2;

/* -------------------------------------------------------------------------- */
/* Current-field conversion (temporary, calibration still required)            */
/* -------------------------------------------------------------------------- */

static uint8_t tmc5240_current_ma_to_cs(uint16_t current_ma)
{
    /*
     * The real current scale depends on sense-resistor and driver configuration
     * details that must be measured on the PCB.  This conservative placeholder
     * maps roughly 0..3100 mA into the TMC 0..31 current field so bench firmware
     * has a bounded register value before final calibration.
     */
    uint32_t cs = ((uint32_t)current_ma + 50U) / 100U;
    /* The presentation profile's 3000 placeholder maps to the prototype IRUN=31. */
    if (current_ma >= 3000U)
    {
        cs = 31U;
    }
    if (cs > 31U)
    {
        cs = 31U;
    }
    return (uint8_t)cs;
}

/* -------------------------------------------------------------------------- */
/* SPI framing and raw register access                                         */
/* -------------------------------------------------------------------------- */

static void TMC5240_CsLow(void)
{
    /* Active-low chip select for the TMC5240 on SPI2. */
    HAL_GPIO_WritePin(MOTOR_CS_GPIO_Port, MOTOR_CS_Pin, GPIO_PIN_RESET);
}

static void TMC5240_CsHigh(void)
{
    /* Release the driver from the current SPI transaction. */
    HAL_GPIO_WritePin(MOTOR_CS_GPIO_Port, MOTOR_CS_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef TMC5240_Transfer(uint8_t address, uint32_t tx_data, uint32_t *rx_data)
{
    /*
     * The TMC5240 protocol is full-duplex.  Even a read clocks out reply bytes
     * during a later transaction, which is why TMC5240_ReadRegister() performs
     * two transfers below.
     */
    uint8_t tx[5];
    uint8_t rx[5] = {0};

    /* TMC5240 SPI datagrams are 5 bytes: register address plus 32-bit data. */
    tx[0] = address;
    tx[1] = (uint8_t)(tx_data >> 24);
    tx[2] = (uint8_t)(tx_data >> 16);
    tx[3] = (uint8_t)(tx_data >> 8);
    tx[4] = (uint8_t)(tx_data);

    TMC5240_CsLow();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 5, 100);
    TMC5240_CsHigh();

    if (rx_data != 0)
    {
        *rx_data = ((uint32_t)rx[1] << 24) |
                   ((uint32_t)rx[2] << 16) |
                   ((uint32_t)rx[3] << 8) |
                   ((uint32_t)rx[4]);
    }

    return status;
}

HAL_StatusTypeDef TMC5240_WriteRegister(uint8_t address, uint32_t value)
{
    return TMC5240_Transfer(address | 0x80, value, 0);
}

HAL_StatusTypeDef TMC5240_ReadRegister(uint8_t address, uint32_t *value)
{
    HAL_StatusTypeDef status;

    /* Reads are pipelined: request the register, then clock out the reply. */
    status = TMC5240_Transfer(address & 0x7F, 0, 0);
    if (status != HAL_OK)
    {
        return status;
    }

    return TMC5240_Transfer(address & 0x7F, 0, value);
}

HAL_StatusTypeDef TMC5240_ReadVersion(uint8_t *version)
{
    uint32_t ioin = 0;
    HAL_StatusTypeDef status = TMC5240_ReadRegister(TMC5240_REG_IOIN, &ioin);

    if (status == HAL_OK && version != 0)
    {
        /* VERSION is the top byte of IOIN and should be 0x40 for TMC5240. */
        *version = (uint8_t)(ioin >> 24);
    }

    return status;
}

/* -------------------------------------------------------------------------- */
/* Direct enable/sleep controls and identity checks                            */
/* -------------------------------------------------------------------------- */

void TMC5240_SetDriverEnabled(uint8_t enabled)
{
    /* DRV_ENN is active-low: reset enables outputs, set disables outputs. */
    HAL_GPIO_WritePin(MOTOR_DRV_ENN_GPIO_Port,
                      MOTOR_DRV_ENN_Pin,
                      enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void TMC5240_SetAwake(uint8_t awake)
{
    /* SLEEPN is active-low: set keeps the TMC5240 awake. */
    HAL_GPIO_WritePin(MOTOR_SLEEPN_GPIO_Port,
                      MOTOR_SLEEPN_Pin,
                      awake ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

HAL_StatusTypeDef TMC5240_BringupTest(uint8_t *version)
{
    uint8_t first_version = 0U;
    uint8_t second_version = 0U;
    HAL_StatusTypeDef status;

    /* First power-up test: keep motor outputs disabled, wake chip, read ID. */
    TMC5240_SetDriverEnabled(0);
    TMC5240_SetAwake(1);
    HAL_Delay(10);

    status = TMC5240_ReadVersion(&first_version);
    if (status != HAL_OK)
    {
        return status;
    }
    status = TMC5240_ReadVersion(&second_version);
    if (status != HAL_OK || first_version != second_version)
    {
        return HAL_ERROR;
    }
    if (version != NULL)
    {
        *version = second_version;
    }
    return HAL_OK;
}

HAL_StatusTypeDef TMC5240_Init(uint8_t *version)
{
    uint8_t detected_version = 0;
    HAL_StatusTypeDef status = TMC5240_BringupTest(&detected_version);

    /* Return the version to main.c so it is visible while debugging. */
    if (version != 0)
    {
        *version = detected_version;
    }

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * The datasheet defines 0x40. The assembled presentation PCB consistently
     * reports 0x41 and its earlier prototype exercised the same register subset.
     * Keep that board-specific allowance out of normal motion-inhibited builds.
     */
    if (detected_version != TMC5240_EXPECTED_VERSION
#if AMBAR_ACTUATOR_USE_PROTOTYPE_PROFILE
        && detected_version != TMC5240_PRESENTATION_BOARD_VERSION
#endif
        )
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* -------------------------------------------------------------------------- */
/* Electrical and ramp-generator profile                                       */
/* -------------------------------------------------------------------------- */

HAL_StatusTypeDef TMC5240_ConfigureSafeDefaults(void)
{
    /*
     * Keep outputs disabled while writing conservative position-mode defaults.
     * The actuator state machine decides later whether DRV_ENN may be released.
     */
    HAL_StatusTypeDef status;

    TMC5240_SetDriverEnabled(0U);
    TMC5240_SetAwake(1U);

    status = TMC5240_WriteRegister(TMC5240_REG_GSTAT, 0x0000001FU);
    if (status != HAL_OK) { return status; }

    /* Match the known-moving prototype's SpreadCycle electrical setup. */
    status = TMC5240_WriteRegister(TMC5240_REG_GCONF, 0x00000000U);
    if (status != HAL_OK) { return status; }

    status = TMC5240_WriteRegister(TMC5240_REG_DRV_CONF, 0x00000000U);
    if (status != HAL_OK) { return status; }

    status = TMC5240_WriteRegister(TMC5240_REG_TPOWERDOWN, 10U);
    if (status != HAL_OK) { return status; }

    /* TOFF=5 enables the chopper; INTPOL=1 and TBL=2 match the prototype. */
    status = TMC5240_WriteRegister(TMC5240_REG_CHOPCONF, 0x10010005U);
    if (status != HAL_OK) { return status; }

    status = TMC5240_WriteRegister(TMC5240_REG_RAMPMODE, TMC5240_RAMPMODE_POSITION);
    if (status != HAL_OK) { return status; }

#if AMBAR_ACTUATOR_USE_PROTOTYPE_PROFILE
    /*
     * Register-equivalent settings from the earlier motor prototype that moved
     * successfully: IHOLD=16, IRUN=31, GLOBALSCALER=0 (the chip's full-scale
     * special value).  The *_ma API remains an explicitly uncalibrated mapping;
     * these numbers must not be interpreted as measured motor current.
     */
    status = TMC5240_SetCurrentLimits(1600U, 3000U);
    if (status != HAL_OK) { return status; }

    status = TMC5240_SetGlobalScaler(0U);
    if (status != HAL_OK) { return status; }

    return TMC5240_SetMotionLimits(200000U, 20000U);
#else
    status = TMC5240_SetCurrentLimits(100U, 200U);
    if (status != HAL_OK) { return status; }

    status = TMC5240_SetGlobalScaler(32U);
    if (status != HAL_OK) { return status; }

    return TMC5240_SetMotionLimits(1000U, 1000U);
#endif
}

HAL_StatusTypeDef TMC5240_SetCurrentLimits(uint16_t hold_current_ma,
                                           uint16_t run_current_ma)
{
    const uint8_t ihold = tmc5240_current_ma_to_cs(hold_current_ma);
    const uint8_t irun = tmc5240_current_ma_to_cs(run_current_ma);
    const uint32_t ihold_irun =
        ((uint32_t)1U << 24) | ((uint32_t)4U << 16)
        | ((uint32_t)irun << 8) | (uint32_t)ihold;

    return TMC5240_WriteRegister(TMC5240_REG_IHOLD_IRUN, ihold_irun);
}

HAL_StatusTypeDef TMC5240_SetGlobalScaler(uint8_t scaler)
{
    /* Hardware accepts 0 as full scale, or values in the range 32..255. */
    if (scaler != 0U && scaler < 32U)
    {
        return HAL_ERROR;
    }

    return TMC5240_WriteRegister(TMC5240_REG_GLOBALSCALER, (uint32_t)scaler);
}

HAL_StatusTypeDef TMC5240_SetMotionLimits(uint32_t max_velocity_steps_per_s,
                                          uint32_t max_accel_steps_per_s2)
{
    /*
     * TMC5240 motion units are internal controller units, not literal steps/s in
     * every configuration.  The naming is kept human-friendly for config files;
     * real conversion must be calibrated before flight.
     */
    HAL_StatusTypeDef status;
    uint32_t velocity = max_velocity_steps_per_s;
    uint32_t accel = max_accel_steps_per_s2;

    if (velocity == 0U) { velocity = 1U; }
    if (accel == 0U) { accel = 1U; }

    status = TMC5240_WriteRegister(TMC5240_REG_VSTART, 1U);
    if (status != HAL_OK) { return status; }

    status = TMC5240_WriteRegister(TMC5240_REG_A1, accel);
    if (status != HAL_OK) { return status; }

    status = TMC5240_WriteRegister(TMC5240_REG_V1, velocity / 2U + 1U);
    if (status != HAL_OK) { return status; }

    status = TMC5240_WriteRegister(TMC5240_REG_AMAX, accel);
    if (status != HAL_OK) { return status; }

    status = TMC5240_WriteRegister(TMC5240_REG_VMAX, velocity);
    if (status != HAL_OK) { return status; }

    status = TMC5240_WriteRegister(TMC5240_REG_DMAX, accel);
    if (status != HAL_OK) { return status; }

    status = TMC5240_WriteRegister(TMC5240_REG_D1, accel);
    if (status != HAL_OK) { return status; }

    status = TMC5240_WriteRegister(TMC5240_REG_VSTOP, 10U);
    if (status != HAL_OK) { return status; }

    return TMC5240_WriteRegister(TMC5240_REG_TZEROWAIT, 10U);
}

/* -------------------------------------------------------------------------- */
/* Position commands and runtime diagnostics                                   */
/* -------------------------------------------------------------------------- */

HAL_StatusTypeDef TMC5240_SetActualPosition(int32_t position_steps)
{
    return TMC5240_WriteRegister(TMC5240_REG_XACTUAL, (uint32_t)position_steps);
}

HAL_StatusTypeDef TMC5240_SetTargetPosition(int32_t target_steps)
{
    return TMC5240_WriteRegister(TMC5240_REG_XTARGET, (uint32_t)target_steps);
}

HAL_StatusTypeDef TMC5240_ReadActualPosition(int32_t *position_steps)
{
    uint32_t raw = 0U;
    HAL_StatusTypeDef status;

    if (position_steps == 0)
    {
        return HAL_ERROR;
    }

    status = TMC5240_ReadRegister(TMC5240_REG_XACTUAL, &raw);
    if (status == HAL_OK)
    {
        *position_steps = (int32_t)raw;
    }

    return status;
}

HAL_StatusTypeDef TMC5240_ReadTargetPosition(int32_t *position_steps)
{
    uint32_t raw = 0U;
    HAL_StatusTypeDef status;

    if (position_steps == 0)
    {
        return HAL_ERROR;
    }

    status = TMC5240_ReadRegister(TMC5240_REG_XTARGET, &raw);
    if (status == HAL_OK)
    {
        *position_steps = (int32_t)raw;
    }

    return status;
}

HAL_StatusTypeDef TMC5240_ReadDriverStatus(uint32_t *driver_status)
{
    if (driver_status == 0)
    {
        return HAL_ERROR;
    }

    return TMC5240_ReadRegister(TMC5240_REG_DRV_STATUS, driver_status);
}

uint8_t TMC5240_DriverStatusHasHardFault(uint32_t driver_status)
{
    return (driver_status & TMC5240_DRV_STATUS_HARD_FAULT_MASK) != 0U ? 1U : 0U;
}

HAL_StatusTypeDef TMC5240_Stop(void)
{
    /*
     * Assert active-low DRV_ENN before any SPI transaction so a sick bus cannot
     * delay removal of motor energy.  Setting target to the current ramp-generator
     * position then prevents a new move if the driver is enabled again later.
     */
    int32_t actual = 0;
    HAL_StatusTypeDef status;

    TMC5240_SetDriverEnabled(0U);
    status = TMC5240_ReadActualPosition(&actual);

    if (status == HAL_OK)
    {
        status = TMC5240_SetTargetPosition(actual);
    }
    return status;
}

uint8_t TMC5240_ReadDiagPins(void)
{
    uint8_t pins = 0U;

    if (HAL_GPIO_ReadPin(MOTOR_DIAG0_GPIO_Port, MOTOR_DIAG0_Pin) == GPIO_PIN_SET)
    {
        pins |= 0x01U;
    }
    if (HAL_GPIO_ReadPin(MOTOR_DIAG1_GPIO_Port, MOTOR_DIAG1_Pin) == GPIO_PIN_SET)
    {
        pins |= 0x02U;
    }

    return pins;
}
