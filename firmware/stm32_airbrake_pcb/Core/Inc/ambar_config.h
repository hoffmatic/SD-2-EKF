/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 18:31:46 -04:00
 *
 * What this file does:
 *   This header defines the saved bench configuration for the Airbrake PCB. It holds EKF tuning, drag model values, sensor orientation, and actuator safety limits.
 *
 * Process flow:
 *   The app loads defaults, tries to read a valid flash copy, validates the values, then applies them to sensors, flight logic, and the actuator. Save writes a CRC-protected record back to flash.
 *
 * Main variables and what can be changed:
 *   Most fields in AmbarConfig_t are tuning knobs. Target apogee, vertical axis/sign, drag values, actuator travel, current, velocity, and acceleration are expected to change during bench testing.
 *
 * Assumptions:
 *   Invalid or missing flash config must never prevent safe startup. The firmware falls back to conservative defaults and keeps actuator motion inhibited.
 *
 * What is missing:
 *   There is no ground-station editor yet, no config schema migration beyond version checking, and no automatic measurement of actuator travel/current.
 */

#ifndef AMBAR_CONFIG_H
#define AMBAR_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ambar_actuator.h"
#include "ambar_flight.h"
#include "rocket_sensors.h"
#include "w25q64.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AMBAR_CONFIG_MAGIC   0x414D4243UL
#define AMBAR_CONFIG_VERSION 1UL

#define AMBAR_CONFIG_PRIMARY_ADDR 0x00000000UL
#define AMBAR_CONFIG_BACKUP_ADDR  0x00001000UL

typedef enum
{
    AMBAR_CONFIG_STATUS_DEFAULTS_USED = 1U << 0U,
    AMBAR_CONFIG_STATUS_FLASH_LOAD_OK = 1U << 1U,
    AMBAR_CONFIG_STATUS_FLASH_SAVE_OK = 1U << 2U,
    AMBAR_CONFIG_STATUS_CRC_ERROR = 1U << 3U,
    AMBAR_CONFIG_STATUS_VALUE_ERROR = 1U << 4U
} AmbarConfigStatusFlags_t;

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
    AmbarFlightConfig_t flight;
    AmbarActuatorConfig_t actuator;

    uint32_t reserved[16];
    uint32_t crc32;
} AmbarConfig_t;

void AmbarConfig_LoadDefaults(AmbarConfig_t *config);
bool AmbarConfig_Validate(const AmbarConfig_t *config);
HAL_StatusTypeDef AmbarConfig_LoadFromFlash(AmbarConfig_t *config);
HAL_StatusTypeDef AmbarConfig_SaveToFlash(const AmbarConfig_t *config);
void AmbarConfig_ApplyToFlightConfig(const AmbarConfig_t *config,
                                     AmbarFlightConfig_t *flight_config);
uint32_t AmbarConfig_GetStatusFlags(void);
uint32_t AmbarConfig_Crc32(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* AMBAR_CONFIG_H */
