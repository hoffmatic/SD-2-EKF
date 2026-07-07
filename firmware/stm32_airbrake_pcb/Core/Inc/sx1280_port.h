/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 17:44:48 -04:00
 *
 * What this file does:
 *   This header separates SX1280 board wiring from the radio command driver. It names functions for chip select, reset, BUSY, and SPI.
 *
 * Process flow:
 *   sx1280.c asks this port layer to select the chip, reset it, wait until it is ready, and clock bytes over SPI1.
 *
 * Main variables and what can be changed:
 *   There are no tuning values here. Pin names come from main.h and should change through CubeMX if PCB routing changes.
 *
 * Assumptions:
 *   The SX1280 uses SPI1 and the LORA pins defined in main.h.
 *
 * What is missing:
 *   No DMA SPI transfers or separate timeout tuning for different radio command types is implemented.
 */

#ifndef SX1280_PORT_H
#define SX1280_PORT_H

#include "main.h"
#include <stdint.h>

/*
 * SX1280 board-port layer.
 *
 * The generic radio code calls these functions instead of touching pins and SPI
 * directly.  That keeps all Airbrake PCB pin assignments in one small file:
 * SPI1 for bytes, LORA_CS for chip select, LORA_NRESET for reset, and LORA_BUSY
 * for command pacing.
 */

/* Put the radio control pins into their idle levels after Cube GPIO init. */
HAL_StatusTypeDef SX1280_PortInit(void);

/* Wait until BUSY is low before sending the next command over SPI. */
HAL_StatusTypeDef SX1280_PortWaitBusyLow(uint32_t timeout_ms);

/* One full-duplex SPI1 transfer.  The SX1280 command layer frames the bytes. */
HAL_StatusTypeDef SX1280_PortSpiTxRx(const uint8_t *tx, uint8_t *rx, uint16_t len);

/* Direct control of the active-low SX1280 chip-select pin. */
void SX1280_PortCsLow(void);
void SX1280_PortCsHigh(void);

/* Hardware reset pulse for recovering the radio after timeout/fault cases. */
void SX1280_PortReset(void);

#endif
