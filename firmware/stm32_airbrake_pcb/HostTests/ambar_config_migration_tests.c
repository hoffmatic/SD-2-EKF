/* Host regression tests for read-only V2 -> V3 configuration migration. */

#include "ambar_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define AMBAR_CONFIG_VERSION_V2_TEST 2UL
#define TEST_FLASH_BYTES (2U * W25Q64_SECTOR_BYTES)

typedef struct
{
    float target_apogee_m;
    float apogee_tolerance_m;
    float full_deployment_error_m;
    float maximum_deploy_fraction;
    float minimum_deploy_altitude_m;
    float minimum_flight_time_s;
} AmbarAirbrakeControllerConfigV2Test_t;

typedef struct
{
    AmbarApogeePredictionMode_t mode;
    float vehicle_mass_kg;
    float drag_area_m2;
    float air_density_kgpm3;
    float time_step_s;
    float max_predict_time_s;
    float actuator_effectiveness;
} AmbarApogeePredictorConfigV2Test_t;

typedef struct
{
    AmbarEkfConfig_t estimator;
    AmbarFlightPhaseConfig_t phase;
    AmbarAirbrakeControllerConfigV2Test_t controller;
    AmbarApogeePredictorConfigV2Test_t apogee;
} AmbarFlightConfigV2Test_t;

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
    AmbarFlightConfigV2Test_t flight;
    AmbarActuatorConfig_t actuator;
    uint32_t reserved[16];
    uint32_t crc32;
} AmbarConfigV2Test_t;

static uint8_t s_flash[TEST_FLASH_BYTES];
static uint32_t s_program_calls;
static uint32_t s_erase_calls;

#define CHECK(condition)                                                        \
    do                                                                          \
    {                                                                           \
        if (!(condition))                                                       \
        {                                                                       \
            (void)printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            return false;                                                       \
        }                                                                       \
    } while (0)

/* -------------------- Production dependency fakes -------------------- */

AmbarFlightConfig_t AmbarFlight_DefaultConfig(void)
{
    AmbarFlightConfig_t config;
    (void)memset(&config, 0, sizeof(config));
    config.controller.target_apogee_m = 914.4f;
    config.controller.apogee_tolerance_m = 30.48f;
    config.controller.mission_tolerance_m = 30.48f;
    config.controller.control_deadband_m = 30.48f;
    config.controller.predictive_update_period_s = 0.05f;
    config.controller.control_mode = AMBAR_CONTROL_MODE_PROPORTIONAL;
    config.controller.full_deployment_error_m = 76.2f;
    config.controller.maximum_deploy_fraction = 1.0f;
    config.apogee.mode = AMBAR_APOGEE_MODE_DRAG;
    config.apogee.vehicle_mass_kg = 5.0f;
    config.apogee.drag_area_m2 = 0.012f;
    config.apogee.air_density_kgpm3 = 1.225f;
    config.apogee.coast_mass_kg = 5.0f;
    config.apogee.baseline_drag_area_m2 = 0.012f;
    for (uint8_t index = 0U;
         index < AMBAR_DEPLOYMENT_CDA_POINT_COUNT;
         ++index)
    {
        config.apogee.deployment_drag_area_m2[index] = 0.012f;
    }
    config.apogee.sea_level_air_density_kgpm3 = 1.225f;
    config.apogee.density_scale_height_m = 8500.0f;
    config.apogee.time_step_s = 0.02f;
    config.apogee.max_predict_time_s = 30.0f;
    config.apogee.actuator_open_rate_fraction_per_s = 1.0f;
    config.apogee.actuator_close_rate_fraction_per_s = 1.0f;
    config.apogee.actuator_effectiveness = 1.0f;
    return config;
}

bool AmbarFlight_ValidateControlConfig(
    const AmbarAirbrakeControllerConfig_t *controller,
    const AmbarApogeePredictorConfig_t *apogee,
    uint32_t *invalid_flags)
{
    if (invalid_flags != NULL)
    {
        *invalid_flags = 0UL;
    }
    return controller != NULL && apogee != NULL;
}

AmbarActuatorConfig_t AmbarActuator_DefaultConfig(void)
{
    const AmbarActuatorConfig_t config = {
        .home_position_steps = 0,
        .max_extension_steps = 10000,
        .max_velocity_steps_per_s = 5000U,
        .max_accel_steps_per_s2 = 10000U,
        .hold_current_ma = 250U,
        .run_current_ma = 500U,
        .direction_inverted = 0U,
        .require_diag_for_home = 0U
    };
    return config;
}

HAL_StatusTypeDef W25Q64_Read(uint32_t address,
                              uint8_t *data,
                              uint32_t length)
{
    if (data == NULL || address > TEST_FLASH_BYTES
        || length > TEST_FLASH_BYTES - address)
    {
        return HAL_ERROR;
    }
    (void)memcpy(data, &s_flash[address], length);
    return HAL_OK;
}

HAL_StatusTypeDef W25Q64_Program(uint32_t address,
                                 const uint8_t *data,
                                 uint32_t length)
{
    ++s_program_calls;
    if (data == NULL || address > TEST_FLASH_BYTES
        || length > TEST_FLASH_BYTES - address)
    {
        return HAL_ERROR;
    }
    (void)memcpy(&s_flash[address], data, length);
    return HAL_OK;
}

HAL_StatusTypeDef W25Q64_EraseSector(uint32_t address)
{
    ++s_erase_calls;
    if (address >= TEST_FLASH_BYTES)
    {
        return HAL_ERROR;
    }
    const uint32_t start = address - (address % W25Q64_SECTOR_BYTES);
    (void)memset(&s_flash[start], 0xFF, W25Q64_SECTOR_BYTES);
    return HAL_OK;
}

/* -------------------- V2 fixture and migration checks -------------------- */

static AmbarConfigV2Test_t valid_v2_record(void)
{
    AmbarConfigV2Test_t record;
    (void)memset(&record, 0, sizeof(record));
    record.magic = AMBAR_CONFIG_MAGIC;
    record.version = AMBAR_CONFIG_VERSION_V2_TEST;
    record.struct_size = (uint32_t)sizeof(record);
    record.sequence = 42UL;
    record.imu_vertical_axis_index = LSM6DSV32X_ACCEL_Z;
    record.imu_vertical_axis_sign = -1.0f;
    record.has_stored_pad_reference = 1UL;
    record.stored_pad_vertical_accel_mps2 = 9.75f;
    record.stored_pad_pressure_pa = 100123.0f;
    record.require_arm_command = 1UL;

    record.flight.estimator.min_dt_s = 0.001f;
    record.flight.estimator.max_dt_s = 0.1f;
    record.flight.estimator.accel_noise_stddev_mps2 = 1.0f;
    record.flight.estimator.accel_bias_random_walk_mps2_per_root_s = 0.1f;
    record.flight.estimator.barometer_bias_random_walk_m_per_root_s = 0.1f;
    record.flight.estimator.initial_altitude_variance_m2 = 10.0f;
    record.flight.estimator.initial_velocity_variance_m2ps2 = 10.0f;
    record.flight.estimator.initial_accel_bias_variance_m2ps4 = 10.0f;
    record.flight.estimator.initial_barometer_bias_variance_m2 = 10.0f;
    record.flight.estimator.barometer_innovation_gate_sigma = 5.0f;
    record.flight.phase.liftoff_altitude_m = 3.0f;
    record.flight.phase.liftoff_acceleration_mps2 = 15.0f;
    record.flight.phase.minimum_boost_time_s = 0.5f;
    record.flight.phase.burnout_acceleration_mps2 = -2.0f;
    record.flight.phase.minimum_coast_velocity_mps = 20.0f;
    record.flight.phase.recovery_descent_velocity_mps = -5.0f;
    record.flight.controller.target_apogee_m = 920.0f;
    record.flight.controller.apogee_tolerance_m = 31.0f;
    record.flight.controller.full_deployment_error_m = 80.0f;
    record.flight.controller.maximum_deploy_fraction = 0.75f;
    record.flight.controller.minimum_deploy_altitude_m = 62.0f;
    record.flight.controller.minimum_flight_time_s = 1.5f;
    record.flight.apogee.mode = AMBAR_APOGEE_MODE_DRAG;
    record.flight.apogee.vehicle_mass_kg = 5.25f;
    record.flight.apogee.drag_area_m2 = 0.014f;
    record.flight.apogee.air_density_kgpm3 = 1.18f;
    record.flight.apogee.time_step_s = 0.02f;
    record.flight.apogee.max_predict_time_s = 30.0f;
    record.flight.apogee.actuator_effectiveness = 0.9f;
    record.actuator.home_position_steps = -123;
    record.actuator.max_extension_steps = 12000;
    record.actuator.max_velocity_steps_per_s = 4321U;
    record.actuator.max_accel_steps_per_s2 = 8765U;
    record.actuator.hold_current_ma = 300U;
    record.actuator.run_current_ma = 700U;
    record.actuator.direction_inverted = 1U;
    record.actuator.require_diag_for_home = 1U;
    record.reserved[3] = 0x12345678UL;
    record.crc32 = 0UL;
    record.crc32 = AmbarConfig_Crc32((const uint8_t *)&record, sizeof(record));
    return record;
}

static void reset_flash(void)
{
    (void)memset(s_flash, 0xFF, sizeof(s_flash));
    s_program_calls = 0UL;
    s_erase_calls = 0UL;
}

static bool test_valid_v2_migrates_in_ram_without_flash_write(void)
{
    const AmbarConfigV2Test_t legacy = valid_v2_record();
    AmbarConfig_t migrated;
    reset_flash();
    (void)memcpy(&s_flash[AMBAR_CONFIG_PRIMARY_ADDR], &legacy, sizeof(legacy));

    CHECK(AmbarConfig_LoadFromFlash(&migrated) == HAL_OK);
    CHECK(migrated.magic == AMBAR_CONFIG_MAGIC);
    CHECK(migrated.version == AMBAR_CONFIG_VERSION);
    CHECK(migrated.struct_size == sizeof(migrated));
    CHECK(migrated.sequence == legacy.sequence);
    CHECK(migrated.imu_vertical_axis_index == legacy.imu_vertical_axis_index);
    CHECK(migrated.imu_vertical_axis_sign == legacy.imu_vertical_axis_sign);
    CHECK(migrated.flight.controller.target_apogee_m
          == legacy.flight.controller.target_apogee_m);
    CHECK(migrated.flight.controller.mission_tolerance_m
          == legacy.flight.controller.apogee_tolerance_m);
    CHECK(migrated.flight.controller.control_deadband_m
          == legacy.flight.controller.apogee_tolerance_m);
    CHECK(migrated.flight.controller.control_mode
          == AMBAR_CONTROL_MODE_PROPORTIONAL);
    CHECK(migrated.flight.controller.deployment_hysteresis_fraction == 0.0f);
    CHECK(migrated.flight.apogee.calibration_version == 0UL);
    CHECK(migrated.flight.apogee.coast_mass_kg
          == legacy.flight.apogee.vehicle_mass_kg);
    CHECK(migrated.flight.apogee.baseline_drag_area_m2
          == legacy.flight.apogee.drag_area_m2);
    for (uint8_t index = 0U;
         index < AMBAR_DEPLOYMENT_CDA_POINT_COUNT;
         ++index)
    {
        CHECK(migrated.flight.apogee.deployment_drag_area_m2[index]
              == legacy.flight.apogee.drag_area_m2);
    }
    CHECK(migrated.actuator.home_position_steps
          == legacy.actuator.home_position_steps);
    CHECK(migrated.actuator.max_extension_steps
          == legacy.actuator.max_extension_steps);
    CHECK(migrated.actuator.direction_inverted
          == legacy.actuator.direction_inverted);
    CHECK(migrated.reserved[3] == legacy.reserved[3]);
    CHECK(AmbarConfig_Validate(&migrated));
    CHECK((AmbarConfig_GetStatusFlags() & AMBAR_CONFIG_STATUS_FLASH_LOAD_OK) != 0U);
    CHECK((AmbarConfig_GetStatusFlags() & AMBAR_CONFIG_STATUS_MIGRATED_V2) != 0U);
    CHECK((AmbarConfig_GetStatusFlags() & AMBAR_CONFIG_STATUS_DEFAULTS_USED) == 0U);
    CHECK(s_program_calls == 0UL);
    CHECK(s_erase_calls == 0UL);
    CHECK(((const AmbarConfigV2Test_t *)&s_flash[AMBAR_CONFIG_PRIMARY_ADDR])->version
          == AMBAR_CONFIG_VERSION_V2_TEST);
    return true;
}

static bool test_bad_v2_crc_falls_back_to_defaults_without_flash_write(void)
{
    AmbarConfigV2Test_t legacy = valid_v2_record();
    AmbarConfig_t loaded;
    reset_flash();
    legacy.flight.controller.target_apogee_m += 1.0f;
    (void)memcpy(&s_flash[AMBAR_CONFIG_PRIMARY_ADDR], &legacy, sizeof(legacy));

    CHECK(AmbarConfig_LoadFromFlash(&loaded) == HAL_ERROR);
    CHECK(loaded.version == AMBAR_CONFIG_VERSION);
    CHECK((AmbarConfig_GetStatusFlags() & AMBAR_CONFIG_STATUS_DEFAULTS_USED) != 0U);
    CHECK((AmbarConfig_GetStatusFlags() & AMBAR_CONFIG_STATUS_MIGRATED_V2) == 0U);
    CHECK(s_program_calls == 0UL);
    CHECK(s_erase_calls == 0UL);
    return true;
}

int main(void)
{
    unsigned passed = 0U;
    const unsigned total = 2U;
    passed += test_valid_v2_migrates_in_ram_without_flash_write() ? 1U : 0U;
    passed += test_bad_v2_crc_falls_back_to_defaults_without_flash_write()
        ? 1U : 0U;
    (void)printf("AMBAR config migration tests: %u/%u passed\n", passed, total);
    return passed == total ? 0 : 1;
}
