/**
 * @file ambar_log.c
 * @brief Implement the aligned W25Q64 snapshot ring described in ambar_log.h.
 *
 * Sections: on-flash format -> geometry checks -> scan/erase helpers -> public
 * lifecycle -> append/accessors.  The implementation favors recoverability and
 * simple offline decoding over maximum write throughput.  See CODE_GUIDE.md
 * [ARCH-7].
 */

#include "ambar_log.h"

#include "ambar_config.h"
#include "w25q64.h"

#include <string.h>

/* -------------------------------------------------------------------------- */
/* On-flash record format and geometry                                        */
/* -------------------------------------------------------------------------- */

/** Identity at the front of every committed AMBAR log record. */
#define AMBAR_LOG_RECORD_MAGIC 0x414D4C47UL
/** First byte after the reserved circular log region. */
#define AMBAR_LOG_END_ADDR     (AMBAR_LOG_START_ADDR + AMBAR_LOG_LENGTH_BYTES)
/** Aligned allocation unit: two slots/page and 32 slots/sector. */
#define AMBAR_LOG_SLOT_BYTES   128UL

typedef struct
{
    /* Header CRC covers this structure with header_crc32 cleared. */
    uint32_t magic;
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint32_t payload_size;
    uint32_t payload_crc32;
    uint32_t header_crc32;
} AmbarLogRecordHeader_t;

/* Two slots fit each flash page and 32 fit each sector, so records never cross
 * an erase/program boundary. The serialized header+payload currently uses 124 B.
 */
_Static_assert((sizeof(AmbarLogRecordHeader_t) + sizeof(AmbarLogSnapshot_t))
               <= AMBAR_LOG_SLOT_BYTES,
               "AMBAR log record no longer fits its flash slot");
_Static_assert((W25Q64_PAGE_BYTES % AMBAR_LOG_SLOT_BYTES) == 0U,
               "AMBAR log slot must divide a flash page");
_Static_assert((W25Q64_SECTOR_BYTES % AMBAR_LOG_SLOT_BYTES) == 0U,
               "AMBAR log slot must divide a flash sector");

/* -------------------------------------------------------------------------- */
/* Module-owned ring state                                                    */
/* -------------------------------------------------------------------------- */

static uint32_t s_log_status_flags = 0U; /**< Exported health/activity bits. */
static uint32_t s_log_next_address = AMBAR_LOG_START_ADDR; /**< Next slot. */
static uint32_t s_log_sequence = 0U; /**< Monotonic record generation. */
static bool s_log_active = false; /**< START_LOG/STOP_LOG latch. */

/** Return the allocation stride independently of the serialized byte count. */
static uint32_t ambar_log_record_size(void)
{
    return AMBAR_LOG_SLOT_BYTES;
}

/** Ensure a candidate slot stays completely inside the reserved log region. */
static bool ambar_log_address_in_range(uint32_t address)
{
    return address >= AMBAR_LOG_START_ADDR
        && address + ambar_log_record_size() <= AMBAR_LOG_END_ADDR;
}

/** Erase a sector only when it is not already blank, limiting erase wear. */
static HAL_StatusTypeDef ambar_log_prepare_sector(uint32_t address)
{
    static uint8_t verify_buffer[W25Q64_PAGE_BYTES];
    const uint32_t sector_start = address - (address % W25Q64_SECTOR_BYTES);

    /* Fresh log space is already 0xFF. Avoid a long erase unless this sector
     * actually contains an older log generation or a partial record.
     */
    for (uint32_t offset = 0U; offset < W25Q64_SECTOR_BYTES; offset += sizeof(verify_buffer))
    {
        HAL_StatusTypeDef status =
            W25Q64_Read(sector_start + offset, verify_buffer, sizeof(verify_buffer));
        if (status != HAL_OK)
        {
            return status;
        }

        for (uint32_t i = 0U; i < sizeof(verify_buffer); ++i)
        {
            if (verify_buffer[i] != 0xFFU)
            {
                return W25Q64_EraseSector(sector_start);
            }
        }
    }

    return HAL_OK;
}

/** Calculate the header CRC without recursively including header_crc32. */
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

/* -------------------------------------------------------------------------- */
/* Public logger lifecycle                                                    */
/* -------------------------------------------------------------------------- */

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
            continue;
        }

        /* An interrupted or legacy 124-byte-stride record is not safe to
         * overwrite in place. Resume at the next aligned sector instead.
         */
        s_log_next_address =
            (address - (address % W25Q64_SECTOR_BYTES)) + W25Q64_SECTOR_BYTES;
        if (!ambar_log_address_in_range(s_log_next_address))
        {
            s_log_next_address = AMBAR_LOG_START_ADDR;
        }
        s_log_status_flags |= AMBAR_LOG_STATUS_FULL_OR_WRAPPED;
        return HAL_OK;
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
    /* Payload and header are programmed separately but accepted only when both
     * CRCs decode correctly during a later scan.
     */
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
        status = ambar_log_prepare_sector(s_log_next_address);
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
