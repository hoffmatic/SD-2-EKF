/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 18:31:46 -04:00
 *
 * What this file does:
 *   This header describes the W25Q64 external flash driver. The flash chip stores bench configuration and flight or bench log records.
 *
 * Process flow:
 *   The app initializes the chip, reads the JEDEC ID, erases 4 KB sectors before rewriting them, writes data in 256-byte pages, and reads records back by address.
 *
 * Main variables and what can be changed:
 *   W25Q64_TOTAL_BYTES, W25Q64_SECTOR_BYTES, and W25Q64_PAGE_BYTES describe the installed W25Q64JVSSIQ. Do not change them unless the flash part changes.
 *
 * Assumptions:
 *   The flash is wired to SPI3 with FLASH_CS as the active-low chip select. The driver uses blocking HAL transfers because logging happens at low rate.
 *
 * What is missing:
 *   No wear leveling beyond the simple log ring, DMA transfers, filesystem, or bad-sector tracking is implemented.
 */

#ifndef W25Q64_H
#define W25Q64_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

#define W25Q64_TOTAL_BYTES   (8UL * 1024UL * 1024UL)
#define W25Q64_SECTOR_BYTES  4096UL
#define W25Q64_PAGE_BYTES    256UL

#define W25Q64_EXPECTED_MANUFACTURER_ID 0xEFU
#define W25Q64_EXPECTED_MEMORY_TYPE     0x40U
#define W25Q64_EXPECTED_CAPACITY        0x17U

typedef struct
{
    uint8_t manufacturer_id;
    uint8_t memory_type;
    uint8_t capacity;
} W25Q64_JedecId_t;

HAL_StatusTypeDef W25Q64_Init(W25Q64_JedecId_t *jedec_id);
HAL_StatusTypeDef W25Q64_ReadJedecId(W25Q64_JedecId_t *jedec_id);
HAL_StatusTypeDef W25Q64_Read(uint32_t address, uint8_t *data, uint32_t length);
HAL_StatusTypeDef W25Q64_Program(uint32_t address, const uint8_t *data, uint32_t length);
HAL_StatusTypeDef W25Q64_EraseSector(uint32_t address);
HAL_StatusTypeDef W25Q64_WaitReady(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* W25Q64_H */
