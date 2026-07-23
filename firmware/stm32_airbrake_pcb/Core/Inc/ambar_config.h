/**
 * @file ambar_config.h
 * @brief Versioned, validated configuration stored redundantly in W25Q64 flash.
 *
 * OVERVIEW / FLOW
 * ---------------
 * AmbarApp first fills safe defaults, then this module reads the primary flash
 * record and falls back to a backup record.  A record is accepted only when its
 * magic, version, structure size, CRC, and every bounded value are valid.  The
 * accepted settings are applied separately to sensors, flight logic, and the
 * actuator; bad flash therefore cannot create partially initialized tuning.
 *
 * LAYOUT / OWNERSHIP
 * ------------------
 * The first two 4 KiB flash sectors hold redundant records.  sequence chooses
 * the newest valid generation, reserved[] allows a compatible future schema,
 * and crc32 protects the entire structure with the crc field zeroed.  See
 * CODE_GUIDE.md [ARCH-2], [ARCH-4], [ARCH-5], and [ARCH-7].
 *
 * SAFETY
 * ------
 * Stored actuator values remain untrusted until AmbarConfig_Validate() and
 * AmbarActuator_ApplyConfig() both accept them.  Presentation builds overwrite
 * stored actuator geometry with the reviewed build profile at runtime.
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

/** ASCII-like on-flash identity for an AMBAR configuration record. */
#define AMBAR_CONFIG_MAGIC   0x414D4243UL

/** Increment when the binary structure layout changes incompatibly. */
/* V3 expands AmbarFlightConfig_t with the calibrated variable-control model. */
#define AMBAR_CONFIG_VERSION 3UL

/** Redundant record sectors; logging begins later and never overlaps them. */
#define AMBAR_CONFIG_PRIMARY_ADDR 0x00000000UL
#define AMBAR_CONFIG_BACKUP_ADDR  0x00001000UL

typedef enum
{
    /** Startup is currently using compiled defaults. */
    AMBAR_CONFIG_STATUS_DEFAULTS_USED = 1U << 0U,
    /** At least one stored record passed identity, CRC, and value checks. */
    AMBAR_CONFIG_STATUS_FLASH_LOAD_OK = 1U << 1U,
    /** The most recent redundant save completed and verified. */
    AMBAR_CONFIG_STATUS_FLASH_SAVE_OK = 1U << 2U,
    /** A candidate record failed its CRC check. */
    AMBAR_CONFIG_STATUS_CRC_ERROR = 1U << 3U,
    /** A candidate record contained an out-of-range field. */
    AMBAR_CONFIG_STATUS_VALUE_ERROR = 1U << 4U,
    /** A valid V2 record was mapped into V3 RAM without writing flash. */
    AMBAR_CONFIG_STATUS_MIGRATED_V2 = 1U << 5U
} AmbarConfigStatusFlags_t;

typedef struct
{
    /* Record envelope used before any nested field is trusted. */
    uint32_t magic;
    uint32_t version;
    uint32_t struct_size;
    uint32_t sequence;

    int32_t imu_vertical_axis_index; /**< Accelerometer axis: 0=X, 1=Y, 2=Z. */
    float imu_vertical_axis_sign;    /**< +1 or -1 after board orientation. */

    /* Optional measured pad reference retained across resets. */
    uint32_t has_stored_pad_reference;
    float stored_pad_vertical_accel_mps2;
    float stored_pad_pressure_pa;

    uint32_t require_arm_command; /**< Nonzero keeps deployment inhibited until ARM. */
    AmbarFlightConfig_t flight;
    AmbarActuatorConfig_t actuator;

    /* Reserved words are written as zero until a later schema defines them. */
    uint32_t reserved[16];
    uint32_t crc32;
} AmbarConfig_t;

/** @brief Fill a complete, safe configuration without reading flash. */
void AmbarConfig_LoadDefaults(AmbarConfig_t *config);

/** @brief Check record identity and every sensor/flight/actuator value range. */
bool AmbarConfig_Validate(const AmbarConfig_t *config);

/** @brief Load the newest valid primary/backup record, preserving defaults on failure. */
HAL_StatusTypeDef AmbarConfig_LoadFromFlash(AmbarConfig_t *config);

/** @brief Validate, sequence, erase, write, and verify both redundant records. */
HAL_StatusTypeDef AmbarConfig_SaveToFlash(const AmbarConfig_t *config);

/** @brief Copy only the nested flight settings into the flight module format. */
void AmbarConfig_ApplyToFlightConfig(const AmbarConfig_t *config,
                                     AmbarFlightConfig_t *flight_config);

/** @brief Return cumulative load/save/default/validation status bits. */
uint32_t AmbarConfig_GetStatusFlags(void);

/** @brief Portable CRC-32 helper shared by config and log records. */
uint32_t AmbarConfig_Crc32(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* AMBAR_CONFIG_H */
