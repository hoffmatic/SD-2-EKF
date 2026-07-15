/**
 * @file radio_bridge.h
 * @brief Application-level telemetry and command bridge for the SX1280 radio.
 *
 * OVERVIEW
 * --------
 * The bridge packages raw sensors, flight estimates, deployment decisions,
 * actuator state, and optional text into tagged binary telemetry.  In the
 * opposite direction it receives radio packets, asks ambar_command to parse
 * them, and presents one pending command to the application at a time.
 *
 * HOW IT WORKS
 * ------------
 * RadioBridge_Init() starts LoRa and sends a boot marker.  The cooperative
 * scheduler calls RadioBridge_Task() frequently so transmit completion,
 * receive events, commands, and optional heartbeats are serviced without
 * blocking sensor/EKF work.  Telemetry producers call
 * PayloadPipelineWithFlight() to append compatible tagged sections.  See
 * CODE_GUIDE.md [ARCH-6] for the complete board-to-ground path.
 *
 * COMPATIBILITY RULE
 * ------------------
 * Packet tags, integer widths, byte order, counts, and fixed-point scales form
 * a shared wire contract with the receiver.  Change them only with a matching
 * ground-station decoder update.  The SX1280 supplies its radio CRC; this layer
 * does not currently add packet negotiation or an application checksum.
 */

#ifndef RADIO_BRIDGE_H
#define RADIO_BRIDGE_H

#include "main.h"
#include "ambar_flight.h"
#include "ambar_command.h"
#include <stdint.h>
#include <string.h>

/** Initialize LoRa and transmit the STM32 boot marker without enabling motion. */
HAL_StatusTypeDef RadioBridge_Init(void);

/** Poll asynchronous TX, RX/command, and heartbeat work; call frequently. */
void RadioBridge_Task(void);

/** Send one bounded ASCII diagnostic message directly over the SX1280. */
HAL_StatusTypeDef RadioBridge_SendText(const char *text);

/** Extra system fields appended after the core raw and flight sections. */
typedef struct
{
    /** Runtime configuration validity/source flags; see ambar_config.h. */
    uint32_t config_flags;
    /** Flash and persistent-log health flags; see ambar_log.h. */
    uint32_t flash_log_flags;
    /** Most recently parsed command action and its application acknowledgement. */
    uint32_t command_action;
    uint32_t command_ack;
    /** Active sensor/mechanism calibration flags. */
    uint32_t calibration_flags;
    /** Actuator state plus commanded and internal-ramp positions. */
    int32_t actuator_state;
    int32_t actuator_target_steps;
    int32_t actuator_actual_steps;
    /** Raw TMC status register and sampled DIAG pins for ground diagnosis. */
    int32_t tmc_driver_status;
    int32_t tmc_diag_pins;
    /** Alternate apogee predictions and drag-model inputs in fixed-point units. */
    int32_t ballistic_apogee_cm;
    int32_t drag_apogee_cm;
    int32_t drag_area_u_m2;
    int32_t actuator_effectiveness_milli;
} AmbarTelemetryExtra_t;

/** Copy and clear the single pending parsed radio command, if one exists. */
bool RadioBridge_TakeCommand(AmbarCommandResult_t *result);
/** Inspect the latest parsed command or parse failure without consuming it. */
const AmbarCommandResult_t *RadioBridge_GetLastCommand(void);

/**
 * Build/transmit the legacy raw-sensor packet.  This wrapper omits all newer
 * flight and actuator sections so older call sites retain source compatibility.
 */
HAL_StatusTypeDef PayloadPipeline(uint16_t *IMU,
                                  uint32_t *Baro,
								  uint16_t *Magnet,
								  uint16_t *Calc,
                                  const char *Message[3],
                                  uint8_t deployment_state);

/**
 * Build and start transmitting the complete tagged telemetry packet.  Raw
 * fields stay first; flight and Extra fields are appended when non-null.
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
