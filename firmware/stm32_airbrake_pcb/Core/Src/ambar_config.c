/**
 * @file ambar_config.c
 * @brief Implement safe defaults, record validation, and redundant flash I/O.
 *
 * Sections: CRC helpers -> compiled defaults -> value validation -> record I/O
 * -> public status.  Flash operations are blocking and must be invoked only by
 * the application maintenance path while motion is off.  See CODE_GUIDE.md
 * [ARCH-7].
 */

#include "ambar_config.h"

#include <math.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Module status and record integrity helpers                                 */
/* -------------------------------------------------------------------------- */

/** Cumulative state exported to telemetry; defaults are true at reset. */
static uint32_t s_config_status_flags = AMBAR_CONFIG_STATUS_DEFAULTS_USED;

uint32_t AmbarConfig_Crc32(const uint8_t *data, size_t length)
{
    /*
     * Standard CRC-32 used for config and log records.  It is small and portable,
     * and it catches accidental flash corruption during bench work.
     */
    uint32_t crc = 0xFFFFFFFFUL;

    if (data == NULL)
    {
        return 0UL;
    }

    for (size_t i = 0U; i < length; ++i)
    {
        crc ^= (uint32_t)data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 1UL) != 0UL)
            {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc ^ 0xFFFFFFFFUL;
}

static uint32_t ambar_config_record_crc(const AmbarConfig_t *config)
{
    /* The stored crc32 word is excluded by evaluating a copy with crc32=0. */
    AmbarConfig_t copy;

    if (config == NULL)
    {
        return 0UL;
    }

    copy = *config;
    copy.crc32 = 0UL;
    return AmbarConfig_Crc32((const uint8_t *)&copy, sizeof(copy));
}

void AmbarConfig_LoadDefaults(AmbarConfig_t *config)
{
    /* Defaults are always constructed before any untrusted flash read. */
    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->magic = AMBAR_CONFIG_MAGIC;
    config->version = AMBAR_CONFIG_VERSION;
    config->struct_size = (uint32_t)sizeof(*config);
    config->sequence = 1UL;
    config->imu_vertical_axis_index = (int32_t)ROCKET_IMU_VERTICAL_ACCEL_INDEX;
    config->imu_vertical_axis_sign = ROCKET_IMU_VERTICAL_ACCEL_SIGN;
    config->has_stored_pad_reference = 0UL;
    config->stored_pad_vertical_accel_mps2 = 0.0f;
    config->stored_pad_pressure_pa = 0.0f;
    config->require_arm_command = 1UL;
    config->flight = AmbarFlight_DefaultConfig();
    config->actuator = AmbarActuator_DefaultConfig();
    config->crc32 = ambar_config_record_crc(config);

    s_config_status_flags = AMBAR_CONFIG_STATUS_DEFAULTS_USED;
}

static bool ambar_float_in_range(float value, float lower, float upper)
{
    /* Reject NaN/Inf explicitly; ordinary comparisons alone do not explain it. */
    return isfinite(value) && value >= lower && value <= upper;
}

bool AmbarConfig_Validate(const AmbarConfig_t *config)
{
    /* Validation is intentionally exhaustive because accepted fields reach the
     * estimator, phase machine, and motor driver without another schema check.
     */
    if (config == NULL)
    {
        return false;
    }

    if (config->magic != AMBAR_CONFIG_MAGIC
        || config->version != AMBAR_CONFIG_VERSION
        || config->struct_size != sizeof(*config))
    {
        return false;
    }

    if (config->crc32 != ambar_config_record_crc(config))
    {
        s_config_status_flags |= AMBAR_CONFIG_STATUS_CRC_ERROR;
        return false;
    }

    if (config->imu_vertical_axis_index < LSM6DSV32X_ACCEL_X
        || config->imu_vertical_axis_index > LSM6DSV32X_ACCEL_Z
        || !ambar_float_in_range(config->imu_vertical_axis_sign, -1.0f, 1.0f)
        || fabsf(config->imu_vertical_axis_sign) < 0.5f)
    {
        s_config_status_flags |= AMBAR_CONFIG_STATUS_VALUE_ERROR;
        return false;
    }

    if (!ambar_float_in_range(config->flight.controller.target_apogee_m, 10.0f, 10000.0f)
        || !ambar_float_in_range(config->flight.controller.apogee_tolerance_m, 0.1f, 1000.0f)
        || !ambar_float_in_range(config->flight.controller.maximum_deploy_fraction, 0.0f, 1.0f)
        || !ambar_float_in_range(config->flight.apogee.vehicle_mass_kg, 0.1f, 100.0f)
        || !ambar_float_in_range(config->flight.apogee.drag_area_m2, 0.0f, 1.0f)
        || !ambar_float_in_range(config->flight.apogee.air_density_kgpm3, 0.1f, 2.0f)
        || !ambar_float_in_range(config->flight.apogee.actuator_effectiveness, 0.1f, 5.0f))
    {
        s_config_status_flags |= AMBAR_CONFIG_STATUS_VALUE_ERROR;
        return false;
    }

    if (config->actuator.max_extension_steps <= 0
        || config->actuator.max_extension_steps > 1000000
        || config->actuator.max_velocity_steps_per_s == 0U
        || config->actuator.max_accel_steps_per_s2 == 0U
        || config->actuator.run_current_ma == 0U
        || config->actuator.run_current_ma > 3000U
        || config->actuator.hold_current_ma > 3000U)
    {
        s_config_status_flags |= AMBAR_CONFIG_STATUS_VALUE_ERROR;
        return false;
    }

    return true;
}

static HAL_StatusTypeDef ambar_config_read_record(uint32_t address, AmbarConfig_t *config)
{
    /* Read one complete candidate, then validate identity/CRC/value as a unit. */
    HAL_StatusTypeDef status;

    if (config == NULL)
    {
        return HAL_ERROR;
    }

    status = W25Q64_Read(address, (uint8_t *)config, sizeof(*config));
    if (status != HAL_OK)
    {
        return status;
    }

    return AmbarConfig_Validate(config) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef AmbarConfig_LoadFromFlash(AmbarConfig_t *config)
{
    /* Prefer the newest valid generation; one corrupt sector is survivable. */
    HAL_StatusTypeDef status;

    if (config == NULL)
    {
        return HAL_ERROR;
    }

    status = ambar_config_read_record(AMBAR_CONFIG_PRIMARY_ADDR, config);
    if (status != HAL_OK)
    {
        status = ambar_config_read_record(AMBAR_CONFIG_BACKUP_ADDR, config);
    }

    if (status == HAL_OK)
    {
        s_config_status_flags &= ~AMBAR_CONFIG_STATUS_DEFAULTS_USED;
        s_config_status_flags |= AMBAR_CONFIG_STATUS_FLASH_LOAD_OK;
    }
    else
    {
        AmbarConfig_LoadDefaults(config);
    }

    return status;
}

static HAL_StatusTypeDef ambar_config_write_record(uint32_t address, const AmbarConfig_t *config)
{
    /* Each record owns one sector so erase/program operations cannot overlap. */
    HAL_StatusTypeDef status;

    status = W25Q64_EraseSector(address);
    if (status != HAL_OK)
    {
        return status;
    }

    return W25Q64_Program(address, (const uint8_t *)config, sizeof(*config));
}

HAL_StatusTypeDef AmbarConfig_SaveToFlash(const AmbarConfig_t *config)
{
    /* Save is all-or-error from the caller's perspective; status flags retain
     * enough detail for telemetry and bench diagnosis.
     */
    AmbarConfig_t writable;
    HAL_StatusTypeDef status;

    if (config == NULL)
    {
        return HAL_ERROR;
    }

    writable = *config;
    writable.magic = AMBAR_CONFIG_MAGIC;
    writable.version = AMBAR_CONFIG_VERSION;
    writable.struct_size = (uint32_t)sizeof(writable);
    writable.sequence++;
    writable.crc32 = ambar_config_record_crc(&writable);

    if (!AmbarConfig_Validate(&writable))
    {
        return HAL_ERROR;
    }

    status = ambar_config_write_record(AMBAR_CONFIG_PRIMARY_ADDR, &writable);
    if (status == HAL_OK)
    {
        status = ambar_config_write_record(AMBAR_CONFIG_BACKUP_ADDR, &writable);
    }

    if (status == HAL_OK)
    {
        s_config_status_flags |= AMBAR_CONFIG_STATUS_FLASH_SAVE_OK;
    }

    return status;
}

void AmbarConfig_ApplyToFlightConfig(const AmbarConfig_t *config,
                                     AmbarFlightConfig_t *flight_config)
{
    if (config == NULL || flight_config == NULL)
    {
        return;
    }

    *flight_config = config->flight;
}

uint32_t AmbarConfig_GetStatusFlags(void)
{
    return s_config_status_flags;
}
