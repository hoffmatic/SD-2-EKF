/*
 * AMBAR SX1280 BOARD PORT - PUBLIC INTERFACE
 *
 * Purpose and ownership
 *   Isolates the generic SX1280 command driver from Airbrake PCB wiring and HAL
 *   handles.  It is the only SX1280 layer that knows about SPI1 and the generated
 *   LORA_CS, LORA_NRESET, and LORA_BUSY symbols from main.h.
 *
 * Transaction flow
 *   sx1280.c waits for BUSY low, asserts chip select, clocks one full-duplex SPI
 *   command, releases chip select, and waits for BUSY again.  PortReset supplies
 *   the hardware reset timing used at startup and TX-failure recovery.  This is
 *   the board-binding edge of CODE_GUIDE.md [ARCH-6].
 *
 * Section map
 *   1. Port initialization and reset/chip-select control
 *   2. BUSY pacing and SPI transfer API
 *
 * Safety and assumptions
 *   Cube-generated GPIO and SPI1 initialization must complete before these calls.
 *   BUSY waits and SPI transfers are bounded but polling/blocking; callers must
 *   keep timeouts compatible with the cooperative scheduler.  Pin-routing changes
 *   belong in CubeMX/main.h and this port, not in the modem command driver.  DMA
 *   and asynchronous SPI completion are not implemented.
 */

#ifndef SX1280_PORT_H
#define SX1280_PORT_H

#include "main.h"
#include <stdint.h>

/* ===================== BOARD PORT API ===================== */

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
