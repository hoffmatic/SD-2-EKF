/**
 * @file ambar_log.h
 * @brief Fixed-slot CRC-protected snapshot logger in external W25Q64 flash.
 *
 * OVERVIEW / FLOW
 * ---------------
 * AmbarLog_Init() scans the reserved flash range for the first reusable slot.
 * START_LOG enables appends; the application provides one self-contained
 * AmbarLogSnapshot_t at the configured cadence; STOP_LOG pauses writes; and
 * ERASE_LOG clears the entire ring.  Records wrap rather than growing into
 * configuration sectors.
 *
 * STORAGE MODEL
 * -------------
 * Records occupy aligned 128-byte slots so no header/payload crosses a W25Q64
 * page or sector boundary.  Each record has sequence, timestamp, payload CRC,
 * and header CRC.  Flash calls are blocking, so log maintenance must not run in
 * a timing-critical motion/USB interval.  See CODE_GUIDE.md [ARCH-1], [ARCH-7].
 */

#ifndef AMBAR_LOG_H
#define AMBAR_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ambar_flight.h"
#include "rocket_sensors.h"

#include <stdbool.h>
#include <stdint.h>

/** First logging byte; lower sectors are reserved for configuration records. */
#define AMBAR_LOG_START_ADDR   0x00010000UL

/** One-megabyte circular region reserved for fixed-size log slots. */
#define AMBAR_LOG_LENGTH_BYTES (1024UL * 1024UL)

typedef enum
{
    /** External flash answered and the log region could be scanned. */
    AMBAR_LOG_STATUS_FLASH_OK = 1U << 0U,
    /** Appends are currently enabled. */
    AMBAR_LOG_STATUS_ACTIVE = 1U << 1U,
    /** The write cursor reached the end and wrapped to an older generation. */
    AMBAR_LOG_STATUS_FULL_OR_WRAPPED = 1U << 2U,
    /** Most recent append completed successfully. */
    AMBAR_LOG_STATUS_LAST_WRITE_OK = 1U << 3U,
    /** Most recent append or erase failed. */
    AMBAR_LOG_STATUS_LAST_WRITE_FAILED = 1U << 4U
} AmbarLogStatusFlags_t;

typedef struct
{
    /* Capture time and raw sensor evidence. Raw arrays preserve device counts. */
    uint32_t timestamp_ms;
    int16_t imu[LSM6DSV32X_DATA_COUNT];
    int16_t mag[LIS2MDL_DATA_COUNT];
    uint32_t baro[BMP388_DATA_COUNT];

    /* Converted estimator/predictor/controller snapshot in engineering units. */
    float altitude_m;
    float velocity_mps;
    float acceleration_mps2;
    float predicted_apogee_m;
    float ballistic_apogee_m;
    float drag_apogee_m;
    float deploy_fraction;
    float pressure_pa;
    float temperature_c;

    /* Decision/status fields explain why the actuator did or did not move. */
    uint32_t phase;
    uint32_t flight_inhibit_flags;
    uint32_t actuator_state;
    uint32_t actuator_inhibit_flags;
    uint32_t command_action;
    uint32_t command_ack;
    int32_t actuator_target_steps;
    int32_t actuator_actual_steps;
} AmbarLogSnapshot_t;

/** @brief Scan or disable the logger according to external-flash health. */
HAL_StatusTypeDef AmbarLog_Init(uint8_t flash_ok);
/** @brief Enable subsequent snapshot appends without erasing existing data. */
HAL_StatusTypeDef AmbarLog_Start(void);
/** @brief Pause appends; existing records remain intact. */
HAL_StatusTypeDef AmbarLog_Stop(void);
/** @brief Erase the complete reserved log range and reset the write cursor. */
HAL_StatusTypeDef AmbarLog_Erase(void);
/** @brief Serialize one CRC-protected snapshot into the current ring slot. */
HAL_StatusTypeDef AmbarLog_AppendSnapshot(const AmbarLogSnapshot_t *snapshot);
/** @brief Return cumulative logger state/error flags for telemetry. */
uint32_t AmbarLog_GetStatusFlags(void);
/** @brief Return the flash address that will receive the next record. */
uint32_t AmbarLog_GetNextAddress(void);
/** @brief Report whether snapshot appends are currently enabled. */
bool AmbarLog_IsActive(void);

#ifdef __cplusplus
}
#endif

#endif /* AMBAR_LOG_H */
