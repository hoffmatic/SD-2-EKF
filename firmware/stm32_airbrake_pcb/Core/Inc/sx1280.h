/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 17:44:48 -04:00
 *
 * What this file does:
 *   This header describes the SX1280 radio driver interface. The radio sends and receives bytes; radio_bridge.c decides what those bytes mean.
 *
 * Process flow:
 *   The radio initializes into LoRa mode, rests in receive mode, temporarily transmits telemetry, then returns to receive mode.
 *
 * Main variables and what can be changed:
 *   SX1280_MAX_PAYLOAD_LEN controls packet buffer size and must match radio_bridge.c payload limits. Radio tuning values are in sx1280.c.
 *
 * Assumptions:
 *   The SX1280 is wired to SPI1 and the LILYGO ground side uses matching LoRa settings.
 *
 * What is missing:
 *   No advanced link management, telemetry retries, command authentication, or packet version negotiation is implemented.
 */

#ifndef SX1280_H
#define SX1280_H

#include "main.h"
#include <stdint.h>

/*
 * SX1280 radio driver interface.
 *
 * radio_bridge.c builds application packets; this module only configures the
 * SX1280 LoRa modem and moves bytes in and out of the radio FIFO over SPI1.
 * Keeping packet formatting separate from the radio driver makes telemetry
 * changes safer and easier to test.
 */

#define SX1280_MAX_PAYLOAD_LEN 200

/* Reset and configure the SX1280 for the bench LoRa link used by the LILYGO. */
HAL_StatusTypeDef SX1280_InitLoRa(void);

/* Return the radio to continuous receive after startup or after a transmit. */
HAL_StatusTypeDef SX1280_StartRxContinuous(void);

/* Blocking transmit with a caller-provided timeout in milliseconds. */
HAL_StatusTypeDef SX1280_Transmit(const uint8_t *data, uint8_t len, uint32_t timeout_ms);

/* Nonblocking receive poll: HAL_OK means a packet was copied into data/len. */
HAL_StatusTypeDef SX1280_ReadPacketIfAvailable(uint8_t *data, uint8_t *len);

#endif
