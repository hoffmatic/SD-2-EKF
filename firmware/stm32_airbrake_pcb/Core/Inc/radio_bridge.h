/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 17:44:48 -04:00
 *
 * What this file does:
 *   This header describes the radio bridge between flight firmware and the SX1280 radio driver. It packages raw sensor data and EKF output for telemetry.
 *
 * Process flow:
 *   The app initializes the bridge, calls its task often, and calls the payload builder during telemetry cycles. Raw fields stay first, and EKF estimate, command, and health fields are appended.
 *
 * Main variables and what can be changed:
 *   Packet layout is mostly in radio_bridge.c. Function signatures should stay stable unless the ground receiver is updated at the same time.
 *
 * Assumptions:
 *   The ground station understands tagged packet fields and little-endian fixed-point integers.
 *
 * What is missing:
 *   There is no full command parser, application-level CRC beyond LoRa CRC, or packet version negotiation.
 */

#ifndef RADIO_BRIDGE_H
#define RADIO_BRIDGE_H

#include "main.h"
#include "ambar_flight.h"
#include "ambar_command.h"
#include <stdint.h>
#include <string.h>

/*
 * ===================== AMBAR EKF PCB INTEGRATION - UPDATED FILE =====================
 *
 * PayloadPipeline() is kept for existing code.  PayloadPipelineWithFlight() adds
 * EKF estimate, command, health, and actuator-inhibit sections while preserving
 * the original raw sensor fields.
 */

HAL_StatusTypeDef RadioBridge_Init(void);

/* Poll RX/heartbeat service; call often from the cooperative scheduler. */
void RadioBridge_Task(void);

/* Send a short ASCII message directly over SX1280 for boot/debug checks. */
HAL_StatusTypeDef RadioBridge_SendText(const char *text);

typedef struct
{
    uint32_t config_flags;
    uint32_t flash_log_flags;
    uint32_t command_action;
    uint32_t command_ack;
    uint32_t calibration_flags;
    int32_t actuator_state;
    int32_t actuator_target_steps;
    int32_t actuator_actual_steps;
    int32_t tmc_driver_status;
    int32_t tmc_diag_pins;
    int32_t ballistic_apogee_cm;
    int32_t drag_apogee_cm;
    int32_t drag_area_u_m2;
    int32_t actuator_effectiveness_milli;
} AmbarTelemetryExtra_t;

bool RadioBridge_TakeCommand(AmbarCommandResult_t *result);
const AmbarCommandResult_t *RadioBridge_GetLastCommand(void);

/*
 * Legacy packet builder kept so older code can still transmit raw IMU/baro/mag
 * and a small calculated section without EKF fields.
 */
HAL_StatusTypeDef PayloadPipeline(uint16_t *IMU,
                                  uint32_t *Baro,
								  uint16_t *Magnet,
								  uint16_t *Calc,
                                  const char *Message[3],
                                  uint8_t deployment_state);

/*
 * EKF-aware packet builder.  Raw sensor fields stay first; signed fixed-point
 * estimate/command/health fields are appended as tagged sections.
 */
HAL_StatusTypeDef PayloadPipelineWithFlight(uint16_t *IMU,
                                            uint32_t *Baro,
                                            uint16_t *Magnet,
                                            uint16_t *Calc,
                                            const char *Message[3],
                                            uint8_t deployment_state,
                                            const AmbarFlightOutput_t *Flight,
                                            uint32_t actuator_inhibit_flags,
                                            const AmbarTelemetryExtra_t *Extra);

#endif
