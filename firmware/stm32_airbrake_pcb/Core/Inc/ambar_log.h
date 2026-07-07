/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 18:31:46 -04:00
 *
 * What this file does:
 *   This header describes the flash-backed bench and flight logger. It records compact snapshots of sensors, EKF output, actuator state, and command status.
 *
 * Process flow:
 *   The app initializes the logger after flash is checked. START_LOG enables appends, each telemetry cycle can append one snapshot, STOP_LOG pauses writes, and ERASE_LOG clears the log area.
 *
 * Main variables and what can be changed:
 *   AMBAR_LOG_START_ADDR and AMBAR_LOG_LENGTH_BYTES choose where records live in W25Q64 flash. The snapshot struct can be extended when telemetry needs more recorded fields.
 *
 * Assumptions:
 *   Config sectors use the beginning of flash, so log records start at 0x00010000. Logging is low-rate and can use blocking sector erase/page program calls.
 *
 * What is missing:
 *   There is no ground-side log download command, no compression, and no high-rate IMU FIFO logging yet.
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

#define AMBAR_LOG_START_ADDR   0x00010000UL
#define AMBAR_LOG_LENGTH_BYTES (1024UL * 1024UL)

typedef enum
{
    AMBAR_LOG_STATUS_FLASH_OK = 1U << 0U,
    AMBAR_LOG_STATUS_ACTIVE = 1U << 1U,
    AMBAR_LOG_STATUS_FULL_OR_WRAPPED = 1U << 2U,
    AMBAR_LOG_STATUS_LAST_WRITE_OK = 1U << 3U,
    AMBAR_LOG_STATUS_LAST_WRITE_FAILED = 1U << 4U
} AmbarLogStatusFlags_t;

typedef struct
{
    uint32_t timestamp_ms;
    int16_t imu[LSM6DSV32X_DATA_COUNT];
    int16_t mag[LIS2MDL_DATA_COUNT];
    uint32_t baro[BMP388_DATA_COUNT];

    float altitude_m;
    float velocity_mps;
    float acceleration_mps2;
    float predicted_apogee_m;
    float ballistic_apogee_m;
    float drag_apogee_m;
    float deploy_fraction;
    float pressure_pa;
    float temperature_c;

    uint32_t phase;
    uint32_t flight_inhibit_flags;
    uint32_t actuator_state;
    uint32_t actuator_inhibit_flags;
    uint32_t command_action;
    uint32_t command_ack;
    int32_t actuator_target_steps;
    int32_t actuator_actual_steps;
} AmbarLogSnapshot_t;

HAL_StatusTypeDef AmbarLog_Init(uint8_t flash_ok);
HAL_StatusTypeDef AmbarLog_Start(void);
HAL_StatusTypeDef AmbarLog_Stop(void);
HAL_StatusTypeDef AmbarLog_Erase(void);
HAL_StatusTypeDef AmbarLog_AppendSnapshot(const AmbarLogSnapshot_t *snapshot);
uint32_t AmbarLog_GetStatusFlags(void);
uint32_t AmbarLog_GetNextAddress(void);
bool AmbarLog_IsActive(void);

#ifdef __cplusplus
}
#endif

#endif /* AMBAR_LOG_H */
