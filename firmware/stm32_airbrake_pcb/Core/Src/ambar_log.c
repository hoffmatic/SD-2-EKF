/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 18:31:46 -04:00
 *
 * What this file does:
 *   This file stores compact logger records in W25Q64 flash. It acts like a simple ring buffer for bench and first-flight data.
 *
 * Process flow:
 *   Init scans the log area for the first empty record slot. Start turns logging on. Append writes a CRC-protected header plus snapshot payload. When the end is reached, the write pointer wraps to the beginning and erases sectors as needed.
 *
 * Main variables and what can be changed:
 *   The log start address and length are in ambar_log.h. AMBAR_LOG_RECORD_MAGIC identifies valid records. The snapshot payload decides what data is saved.
 *
 * Assumptions:
 *   Logging is low-rate, flash writes can block for a short time, and erased flash reads back as 0xFF.
 *
 * What is missing:
 *   There is no wear-level database, no partial-record recovery beyond CRC, and no radio command to stream saved records back yet.
 */

#include "ambar_log.h"

#include "ambar_config.h"
#include "w25q64.h"

#include <string.h>

#define AMBAR_LOG_RECORD_MAGIC 0x414D4C47UL
#define AMBAR_LOG_END_ADDR     (AMBAR_LOG_START_ADDR + AMBAR_LOG_LENGTH_BYTES)

typedef struct
{
    uint32_t magic;
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint32_t payload_size;
    uint32_t payload_crc32;
    uint32_t header_crc32;
} AmbarLogRecordHeader_t;

static uint32_t s_log_status_flags = 0U;
static uint32_t s_log_next_address = AMBAR_LOG_START_ADDR;
static uint32_t s_log_sequence = 0U;
static bool s_log_active = false;

static uint32_t ambar_log_record_size(void)
{
    return (uint32_t)(sizeof(AmbarLogRecordHeader_t) + sizeof(AmbarLogSnapshot_t));
}

static bool ambar_log_address_in_range(uint32_t address)
{
    return address >= AMBAR_LOG_START_ADDR
        && address + ambar_log_record_size() <= AMBAR_LOG_END_ADDR;
}

static uint32_t ambar_log_header_crc(const AmbarLogRecordHeader_t *header)
{
    AmbarLogRecordHeader_t copy;

    if (header == 0)
    {
        return 0U;
    }

    copy = *header;
    copy.header_crc32 = 0U;
    return AmbarConfig_Crc32((const uint8_t *)&copy, sizeof(copy));
}

HAL_StatusTypeDef AmbarLog_Init(uint8_t flash_ok)
{
    s_log_status_flags = 0U;
    s_log_next_address = AMBAR_LOG_START_ADDR;
    s_log_sequence = 0U;
    s_log_active = false;

    if (flash_ok == 0U)
    {
        return HAL_ERROR;
    }

    s_log_status_flags |= AMBAR_LOG_STATUS_FLASH_OK;

    for (uint32_t address = AMBAR_LOG_START_ADDR;
         address + sizeof(AmbarLogRecordHeader_t) <= AMBAR_LOG_END_ADDR;
         address += ambar_log_record_size())
    {
        AmbarLogRecordHeader_t header;
        HAL_StatusTypeDef status =
            W25Q64_Read(address, (uint8_t *)&header, sizeof(header));

        if (status != HAL_OK)
        {
            return status;
        }

        if (header.magic == 0xFFFFFFFFUL)
        {
            s_log_next_address = address;
            return HAL_OK;
        }

        if (header.magic == AMBAR_LOG_RECORD_MAGIC
            && header.payload_size == sizeof(AmbarLogSnapshot_t)
            && header.header_crc32 == ambar_log_header_crc(&header))
        {
            s_log_sequence = header.sequence + 1U;
        }
    }

    s_log_next_address = AMBAR_LOG_START_ADDR;
    s_log_status_flags |= AMBAR_LOG_STATUS_FULL_OR_WRAPPED;
    return HAL_OK;
}

HAL_StatusTypeDef AmbarLog_Start(void)
{
    if ((s_log_status_flags & AMBAR_LOG_STATUS_FLASH_OK) == 0U)
    {
        return HAL_ERROR;
    }

    s_log_active = true;
    s_log_status_flags |= AMBAR_LOG_STATUS_ACTIVE;
    return HAL_OK;
}

HAL_StatusTypeDef AmbarLog_Stop(void)
{
    s_log_active = false;
    s_log_status_flags &= ~AMBAR_LOG_STATUS_ACTIVE;
    return HAL_OK;
}

HAL_StatusTypeDef AmbarLog_Erase(void)
{
    if ((s_log_status_flags & AMBAR_LOG_STATUS_FLASH_OK) == 0U)
    {
        return HAL_ERROR;
    }

    s_log_active = false;
    s_log_status_flags &= ~AMBAR_LOG_STATUS_ACTIVE;

    for (uint32_t address = AMBAR_LOG_START_ADDR;
         address < AMBAR_LOG_END_ADDR;
         address += W25Q64_SECTOR_BYTES)
    {
        HAL_StatusTypeDef status = W25Q64_EraseSector(address);
        if (status != HAL_OK)
        {
            return status;
        }
    }

    s_log_next_address = AMBAR_LOG_START_ADDR;
    s_log_sequence = 0U;
    s_log_status_flags &= ~AMBAR_LOG_STATUS_FULL_OR_WRAPPED;
    return HAL_OK;
}

HAL_StatusTypeDef AmbarLog_AppendSnapshot(const AmbarLogSnapshot_t *snapshot)
{
    AmbarLogRecordHeader_t header;
    HAL_StatusTypeDef status;

    if (!s_log_active || snapshot == 0)
    {
        return HAL_ERROR;
    }

    if (!ambar_log_address_in_range(s_log_next_address))
    {
        s_log_next_address = AMBAR_LOG_START_ADDR;
        s_log_status_flags |= AMBAR_LOG_STATUS_FULL_OR_WRAPPED;
    }

    if ((s_log_next_address % W25Q64_SECTOR_BYTES) == 0U)
    {
        status = W25Q64_EraseSector(s_log_next_address);
        if (status != HAL_OK)
        {
            s_log_status_flags |= AMBAR_LOG_STATUS_LAST_WRITE_FAILED;
            return status;
        }
    }

    memset(&header, 0, sizeof(header));
    header.magic = AMBAR_LOG_RECORD_MAGIC;
    header.sequence = s_log_sequence++;
    header.timestamp_ms = snapshot->timestamp_ms;
    header.payload_size = (uint32_t)sizeof(*snapshot);
    header.payload_crc32 = AmbarConfig_Crc32((const uint8_t *)snapshot, sizeof(*snapshot));
    header.header_crc32 = ambar_log_header_crc(&header);

    status = W25Q64_Program(s_log_next_address, (const uint8_t *)&header, sizeof(header));
    if (status == HAL_OK)
    {
        status = W25Q64_Program(s_log_next_address + sizeof(header),
                                (const uint8_t *)snapshot,
                                sizeof(*snapshot));
    }

    if (status == HAL_OK)
    {
        s_log_next_address += ambar_log_record_size();
        s_log_status_flags |= AMBAR_LOG_STATUS_LAST_WRITE_OK;
        s_log_status_flags &= ~AMBAR_LOG_STATUS_LAST_WRITE_FAILED;
    }
    else
    {
        s_log_status_flags |= AMBAR_LOG_STATUS_LAST_WRITE_FAILED;
    }

    return status;
}

uint32_t AmbarLog_GetStatusFlags(void)
{
    return s_log_status_flags;
}

uint32_t AmbarLog_GetNextAddress(void)
{
    return s_log_next_address;
}

bool AmbarLog_IsActive(void)
{
    return s_log_active;
}
