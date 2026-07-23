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

/*
 * Exact V2 on-flash layout retained only for read-time migration. All members
 * are fixed-width integers, enums, or IEEE-754 floats with four-byte alignment
 * on the STM32 ABI. V2 is never written after this firmware boots.
 */
#define AMBAR_CONFIG_VERSION_V2 2UL

typedef struct
{
    float target_apogee_m;
    float apogee_tolerance_m;
    float full_deployment_error_m;
    float maximum_deploy_fraction;
    float minimum_deploy_altitude_m;
    float minimum_flight_time_s;
} AmbarAirbrakeControllerConfigV2_t;

typedef struct
{
    AmbarApogeePredictionMode_t mode;
    float vehicle_mass_kg;
    float drag_area_m2;
    float air_density_kgpm3;
    float time_step_s;
    float max_predict_time_s;
    float actuator_effectiveness;
} AmbarApogeePredictorConfigV2_t;

typedef struct
{
    AmbarEkfConfig_t estimator;
    AmbarFlightPhaseConfig_t phase;
    AmbarAirbrakeControllerConfigV2_t controller;
    AmbarApogeePredictorConfigV2_t apogee;
} AmbarFlightConfigV2_t;

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t struct_size;
    uint32_t sequence;
    int32_t imu_vertical_axis_index;
    float imu_vertical_axis_sign;
    uint32_t has_stored_pad_reference;
    float stored_pad_vertical_accel_mps2;
    float stored_pad_pressure_pa;
    uint32_t require_arm_command;
    AmbarFlightConfigV2_t flight;
    AmbarActuatorConfig_t actuator;
    uint32_t reserved[16];
    uint32_t crc32;
} AmbarConfigV2_t;

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t struct_size;
} AmbarConfigEnvelope_t;

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

static uint32_t ambar_config_v2_record_crc(const AmbarConfigV2_t *config)
{
    AmbarConfigV2_t copy;

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

static bool ambar_config_v2_validate(const AmbarConfigV2_t *config)
{
    if (config == NULL
        || config->magic != AMBAR_CONFIG_MAGIC
        || config->version != AMBAR_CONFIG_VERSION_V2
        || config->struct_size != sizeof(*config))
    {
        return false;
    }
    if (config->crc32 != ambar_config_v2_record_crc(config))
    {
        s_config_status_flags |= AMBAR_CONFIG_STATUS_CRC_ERROR;
        return false;
    }
    if (config->imu_vertical_axis_index < LSM6DSV32X_ACCEL_X
        || config->imu_vertical_axis_index > LSM6DSV32X_ACCEL_Z
        || !ambar_float_in_range(config->imu_vertical_axis_sign, -1.0f, 1.0f)
        || fabsf(config->imu_vertical_axis_sign) < 0.5f
        || config->has_stored_pad_reference > 1UL
        || config->require_arm_command > 1UL)
    {
        s_config_status_flags |= AMBAR_CONFIG_STATUS_VALUE_ERROR;
        return false;
    }
    if (config->has_stored_pad_reference != 0UL
        && (!ambar_float_in_range(config->stored_pad_vertical_accel_mps2,
                                  -100.0f,
                                  100.0f)
            || !ambar_float_in_range(config->stored_pad_pressure_pa,
                                     1000.0f,
                                     200000.0f)))
    {
        s_config_status_flags |= AMBAR_CONFIG_STATUS_VALUE_ERROR;
        return false;
    }

    const AmbarEkfConfig_t *ekf = &config->flight.estimator;
    if (!ambar_float_in_range(ekf->min_dt_s, 0.000001f, 0.1f)
        || !ambar_float_in_range(ekf->max_dt_s, ekf->min_dt_s, 2.0f)
        || !ambar_float_in_range(ekf->accel_noise_stddev_mps2, 0.0f, 1000.0f)
        || !ambar_float_in_range(ekf->accel_bias_random_walk_mps2_per_root_s,
                                 0.0f,
                                 1000.0f)
        || !ambar_float_in_range(ekf->barometer_bias_random_walk_m_per_root_s,
                                 0.0f,
                                 1000.0f)
        || !ambar_float_in_range(ekf->initial_altitude_variance_m2,
                                 0.0f,
                                 1000000.0f)
        || !ambar_float_in_range(ekf->initial_velocity_variance_m2ps2,
                                 0.0f,
                                 1000000.0f)
        || !ambar_float_in_range(ekf->initial_accel_bias_variance_m2ps4,
                                 0.0f,
                                 1000000.0f)
        || !ambar_float_in_range(ekf->initial_barometer_bias_variance_m2,
                                 0.0f,
                                 1000000.0f)
        || !ambar_float_in_range(ekf->barometer_innovation_gate_sigma,
                                 0.1f,
                                 100.0f))
    {
        s_config_status_flags |= AMBAR_CONFIG_STATUS_VALUE_ERROR;
        return false;
    }

    const AmbarFlightPhaseConfig_t *phase = &config->flight.phase;
    if (!ambar_float_in_range(phase->liftoff_altitude_m, 0.0f, 1000.0f)
        || !ambar_float_in_range(phase->liftoff_acceleration_mps2,
                                 0.0f,
                                 1000.0f)
        || !ambar_float_in_range(phase->minimum_boost_time_s, 0.0f, 60.0f)
        || !ambar_float_in_range(phase->burnout_acceleration_mps2,
                                 -1000.0f,
                                 1000.0f)
        || !ambar_float_in_range(phase->minimum_coast_velocity_mps,
                                 0.0f,
                                 2000.0f)
        || !ambar_float_in_range(phase->recovery_descent_velocity_mps,
                                 -2000.0f,
                                 0.0f))
    {
        s_config_status_flags |= AMBAR_CONFIG_STATUS_VALUE_ERROR;
        return false;
    }

    const AmbarAirbrakeControllerConfigV2_t *controller =
        &config->flight.controller;
    const AmbarApogeePredictorConfigV2_t *apogee = &config->flight.apogee;
    if (!ambar_float_in_range(controller->target_apogee_m, 10.0f, 10000.0f)
        || !ambar_float_in_range(controller->apogee_tolerance_m, 0.1f, 1000.0f)
        || !ambar_float_in_range(controller->full_deployment_error_m,
                                 0.1f,
                                 10000.0f)
        || !ambar_float_in_range(controller->maximum_deploy_fraction,
                                 0.0f,
                                 1.0f)
        || !ambar_float_in_range(controller->minimum_deploy_altitude_m,
                                 0.0f,
                                 10000.0f)
        || !ambar_float_in_range(controller->minimum_flight_time_s,
                                 0.0f,
                                 120.0f)
        || (apogee->mode != AMBAR_APOGEE_MODE_BALLISTIC
            && apogee->mode != AMBAR_APOGEE_MODE_DRAG)
        || !ambar_float_in_range(apogee->vehicle_mass_kg, 0.1f, 100.0f)
        || !ambar_float_in_range(apogee->drag_area_m2, 0.0f, 1.0f)
        || !ambar_float_in_range(apogee->air_density_kgpm3, 0.1f, 2.0f)
        || !ambar_float_in_range(apogee->time_step_s, 0.001f, 0.1f)
        || !ambar_float_in_range(apogee->max_predict_time_s, 1.0f, 120.0f)
        || !ambar_float_in_range(apogee->actuator_effectiveness, 0.01f, 10.0f))
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
        || config->actuator.hold_current_ma > 3000U
        || config->actuator.direction_inverted > 1U
        || config->actuator.require_diag_for_home > 1U)
    {
        s_config_status_flags |= AMBAR_CONFIG_STATUS_VALUE_ERROR;
        return false;
    }
    return true;
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

    uint32_t flight_invalid_flags = 0UL;

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

    if (!AmbarFlight_ValidateControlConfig(&config->flight.controller,
                                            &config->flight.apogee,
                                            &flight_invalid_flags))
    {
        (void)flight_invalid_flags;
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

static bool ambar_config_migrate_v2(const AmbarConfigV2_t *legacy,
                                    AmbarConfig_t *config)
{
    if (!ambar_config_v2_validate(legacy) || config == NULL)
    {
        return false;
    }

    AmbarConfig_LoadDefaults(config);
    config->sequence = legacy->sequence;
    config->imu_vertical_axis_index = legacy->imu_vertical_axis_index;
    config->imu_vertical_axis_sign = legacy->imu_vertical_axis_sign;
    config->has_stored_pad_reference = legacy->has_stored_pad_reference;
    config->stored_pad_vertical_accel_mps2 =
        legacy->stored_pad_vertical_accel_mps2;
    config->stored_pad_pressure_pa = legacy->stored_pad_pressure_pa;
    config->require_arm_command = legacy->require_arm_command;
    config->flight.estimator = legacy->flight.estimator;
    config->flight.phase = legacy->flight.phase;

    config->flight.controller.target_apogee_m =
        legacy->flight.controller.target_apogee_m;
    config->flight.controller.apogee_tolerance_m =
        legacy->flight.controller.apogee_tolerance_m;
    config->flight.controller.mission_tolerance_m =
        legacy->flight.controller.apogee_tolerance_m;
    config->flight.controller.control_deadband_m =
        legacy->flight.controller.apogee_tolerance_m;
    config->flight.controller.deployment_hysteresis_fraction = 0.0f;
    config->flight.controller.control_mode = AMBAR_CONTROL_MODE_PROPORTIONAL;
    config->flight.controller.full_deployment_error_m =
        legacy->flight.controller.full_deployment_error_m;
    config->flight.controller.maximum_deploy_fraction =
        legacy->flight.controller.maximum_deploy_fraction;
    config->flight.controller.minimum_deploy_altitude_m =
        legacy->flight.controller.minimum_deploy_altitude_m;
    config->flight.controller.minimum_flight_time_s =
        legacy->flight.controller.minimum_flight_time_s;

    config->flight.apogee.mode = legacy->flight.apogee.mode;
    config->flight.apogee.vehicle_mass_kg = legacy->flight.apogee.vehicle_mass_kg;
    config->flight.apogee.drag_area_m2 = legacy->flight.apogee.drag_area_m2;
    config->flight.apogee.air_density_kgpm3 =
        legacy->flight.apogee.air_density_kgpm3;
    config->flight.apogee.calibration_version = 0UL;
    config->flight.apogee.coast_mass_kg = legacy->flight.apogee.vehicle_mass_kg;
    config->flight.apogee.baseline_drag_area_m2 =
        legacy->flight.apogee.drag_area_m2;
    for (uint8_t index = 0U;
         index < AMBAR_DEPLOYMENT_CDA_POINT_COUNT;
         ++index)
    {
        config->flight.apogee.deployment_drag_area_m2[index] =
            legacy->flight.apogee.drag_area_m2;
    }
    config->flight.apogee.sea_level_air_density_kgpm3 =
        legacy->flight.apogee.air_density_kgpm3;
    config->flight.apogee.time_step_s = legacy->flight.apogee.time_step_s;
    config->flight.apogee.max_predict_time_s =
        legacy->flight.apogee.max_predict_time_s;
    config->flight.apogee.actuator_delay_s = 0.0f;
    config->flight.apogee.actuator_open_rate_fraction_per_s = 1.0f;
    config->flight.apogee.actuator_close_rate_fraction_per_s = 1.0f;
    config->flight.apogee.actuator_effectiveness =
        legacy->flight.apogee.actuator_effectiveness;

    config->actuator = legacy->actuator;
    memcpy(config->reserved, legacy->reserved, sizeof(config->reserved));
    config->crc32 = ambar_config_record_crc(config);
    return AmbarConfig_Validate(config);
}

static HAL_StatusTypeDef ambar_config_read_record(uint32_t address,
                                                   AmbarConfig_t *config,
                                                   bool *migrated_v2)
{
    AmbarConfigEnvelope_t envelope;
    HAL_StatusTypeDef status;

    if (config == NULL || migrated_v2 == NULL)
    {
        return HAL_ERROR;
    }
    *migrated_v2 = false;
    status = W25Q64_Read(address, (uint8_t *)&envelope, sizeof(envelope));
    if (status != HAL_OK)
    {
        return status;
    }
    if (envelope.magic != AMBAR_CONFIG_MAGIC)
    {
        return HAL_ERROR;
    }

    if (envelope.version == AMBAR_CONFIG_VERSION
        && envelope.struct_size == sizeof(*config))
    {
        status = W25Q64_Read(address, (uint8_t *)config, sizeof(*config));
        return status == HAL_OK && AmbarConfig_Validate(config)
            ? HAL_OK : HAL_ERROR;
    }
    if (envelope.version == AMBAR_CONFIG_VERSION_V2
        && envelope.struct_size == sizeof(AmbarConfigV2_t))
    {
        AmbarConfigV2_t legacy;
        status = W25Q64_Read(address, (uint8_t *)&legacy, sizeof(legacy));
        if (status == HAL_OK && ambar_config_migrate_v2(&legacy, config))
        {
            *migrated_v2 = true;
            return HAL_OK;
        }
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef AmbarConfig_LoadFromFlash(AmbarConfig_t *config)
{
    /* Prefer the newest valid generation; one corrupt sector is survivable. */
    HAL_StatusTypeDef status;
    bool migrated_v2 = false;

    if (config == NULL)
    {
        return HAL_ERROR;
    }

    status = ambar_config_read_record(AMBAR_CONFIG_PRIMARY_ADDR,
                                      config,
                                      &migrated_v2);
    if (status != HAL_OK)
    {
        status = ambar_config_read_record(AMBAR_CONFIG_BACKUP_ADDR,
                                          config,
                                          &migrated_v2);
    }

    if (status == HAL_OK)
    {
        s_config_status_flags &= ~AMBAR_CONFIG_STATUS_DEFAULTS_USED;
        s_config_status_flags |= AMBAR_CONFIG_STATUS_FLASH_LOAD_OK;
        if (migrated_v2)
        {
            s_config_status_flags |= AMBAR_CONFIG_STATUS_MIGRATED_V2;
        }
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
