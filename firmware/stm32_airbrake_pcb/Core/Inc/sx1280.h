/*
 * AMBAR SX1280 LORA MODEM DRIVER - PUBLIC INTERFACE
 *
 * Purpose and ownership
 *   Exposes byte-oriented initialization, receive, and transmit operations for
 *   the flight computer's SX1280.  This layer owns modem/FIFO/IRQ state;
 *   radio_bridge owns the version-1 packet and command meaning.  The board-specific
 *   SPI, BUSY, reset, and chip-select details remain behind sx1280_port.
 *
 * Call flow
 *   InitLoRa() resets and configures the matched air/ground link, then leaves the
 *   modem in continuous receive.  ReadPacketIfAvailable() polls complete packets.
 *   Periodic telemetry uses StartTransmit()/ServiceTransmit() so the cooperative
 *   flight loop can keep running; completion or recovery returns to receive.
 *   See CODE_GUIDE.md [ARCH-6].
 *
 * Section map
 *   1. Shared payload bound
 *   2. Initialization and receive-state control
 *   3. Cooperative and blocking transmit API
 *   4. Nonblocking receive API
 *
 * Safety and assumptions
 *   The modulation, frequency, sync word, preamble, CRC, and payload limit must
 *   match the LILYGO ground station.  HAL_OK reports a completed driver operation,
 *   not authenticated command content.  Start/Service should be preferred in the
 *   time-sensitive main loop; the blocking wrapper can delay other cooperative
 *   tasks.  There is no retry/ACK policy, link encryption, or command authorization
 *   in this byte-moving driver.
 */

#ifndef SX1280_H
#define SX1280_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* ===================== SHARED PAYLOAD BOUND ===================== */

/* Must cover the largest radio_bridge packet without exceeding FIFO buffers. */
#define SX1280_MAX_PAYLOAD_LEN 200

/* ===================== INITIALIZATION AND RECEIVE STATE ===================== */

/* Reset and configure the SX1280 for the bench LoRa link used by the LILYGO. */
HAL_StatusTypeDef SX1280_InitLoRa(void);

/* Return the radio to continuous receive after startup or after a transmit. */
HAL_StatusTypeDef SX1280_StartRxContinuous(void);

/* ===================== TRANSMIT API ===================== */

/* Blocking compatibility wrapper; it services TX until completion or timeout. */
HAL_StatusTypeDef SX1280_Transmit(const uint8_t *data, uint8_t len, uint32_t timeout_ms);

/*
 * Start and service a transmit without blocking the cooperative flight loop.
 * StartTransmit copies the packet into the radio FIFO before it returns, so the
 * caller may reuse its buffer immediately.  ServiceTransmit returns HAL_BUSY
 * while the packet is on air and restores continuous RX after TX_DONE.
 */
HAL_StatusTypeDef SX1280_StartTransmit(const uint8_t *data,
                                       uint8_t len,
                                       uint32_t timeout_ms);
HAL_StatusTypeDef SX1280_ServiceTransmit(void);

/* True only while a StartTransmit operation still owns the modem. */
bool SX1280_IsTransmitBusy(void);

/* ===================== RECEIVE API ===================== */

/* HAL_OK copies one packet; HAL_BUSY means no packet; other status means fault. */
HAL_StatusTypeDef SX1280_ReadPacketIfAvailable(uint8_t *data, uint8_t *len);

#endif
