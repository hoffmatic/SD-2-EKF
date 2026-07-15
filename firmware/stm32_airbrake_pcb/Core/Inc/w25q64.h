/**
 * @file w25q64.h
 * @brief Blocking SPI3 driver for the 8 MiB external configuration/log flash.
 *
 * The driver verifies the JEDEC identity, exposes bounded reads, splits programs
 * at 256-byte page boundaries, aligns erases to 4 KiB sectors, and waits for the
 * BUSY bit after every write operation.  ambar_config.c and ambar_log.c own the
 * on-flash formats; this layer only performs address-based storage operations.
 *
 * Calls are synchronous and sector erase can take seconds.  The application
 * must keep them out of critical USB/sensor/motion intervals.  See
 * CODE_GUIDE.md [ARCH-1], [ARCH-7].
 */

#ifndef W25Q64_H
#define W25Q64_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

/* Geometry of the fitted W25Q64JV-class device. */
#define W25Q64_TOTAL_BYTES   (8UL * 1024UL * 1024UL)
#define W25Q64_SECTOR_BYTES  4096UL
#define W25Q64_PAGE_BYTES    256UL

/* Three-byte JEDEC identity expected from the assembled PCB. */
#define W25Q64_EXPECTED_MANUFACTURER_ID 0xEFU
#define W25Q64_EXPECTED_MEMORY_TYPE     0x40U
#define W25Q64_EXPECTED_CAPACITY        0x17U

typedef struct
{
    /** Winbond manufacturer byte. */
    uint8_t manufacturer_id;
    /** Device-family/type byte. */
    uint8_t memory_type;
    /** Density byte (0x17 represents 64 Mbit). */
    uint8_t capacity;
} W25Q64_JedecId_t;

/** @brief Release chip select, verify JEDEC identity, and wait until ready. */
HAL_StatusTypeDef W25Q64_Init(W25Q64_JedecId_t *jedec_id);
/** @brief Read the raw three-byte JEDEC identity. */
HAL_StatusTypeDef W25Q64_ReadJedecId(W25Q64_JedecId_t *jedec_id);
/** @brief Read a bounded byte range without modifying flash. */
HAL_StatusTypeDef W25Q64_Read(uint32_t address, uint8_t *data, uint32_t length);
/** @brief Page-split a bounded program operation; caller must erase as needed. */
HAL_StatusTypeDef W25Q64_Program(uint32_t address, const uint8_t *data, uint32_t length);
/** @brief Erase the 4 KiB sector containing @p address. */
HAL_StatusTypeDef W25Q64_EraseSector(uint32_t address);
/** @brief Poll status register 1 until BUSY clears or timeout expires. */
HAL_StatusTypeDef W25Q64_WaitReady(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* W25Q64_H */
