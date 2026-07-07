/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 18:31:46 -04:00
 *
 * What this file does:
 *   This file talks to the W25Q64JV flash chip over SPI3. It gives the rest of the firmware safe read, sector erase, and page program functions.
 *
 * Process flow:
 *   Every write first enables writes, sends either a page program or sector erase command, then waits until the busy bit clears. Reads stream bytes from an address without changing flash contents.
 *
 * Main variables and what can be changed:
 *   Command bytes match the W25Q64JV datasheet. Timeout values can be adjusted after bench testing, but page and sector sizes are fixed by the chip.
 *
 * Assumptions:
 *   The chip select pin idles high, SPI3 mode matches the flash device, and callers erase sectors before programming bytes from 1 to 0.
 *
 * What is missing:
 *   The driver does not implement quad-SPI, suspend/resume, unique ID reads, power-down modes, or a filesystem.
 */

#include "w25q64.h"

#include <stddef.h>

extern SPI_HandleTypeDef hspi3;

#define W25Q64_CMD_WRITE_ENABLE  0x06U
#define W25Q64_CMD_READ_STATUS1  0x05U
#define W25Q64_CMD_READ_DATA     0x03U
#define W25Q64_CMD_PAGE_PROGRAM  0x02U
#define W25Q64_CMD_SECTOR_ERASE  0x20U
#define W25Q64_CMD_JEDEC_ID      0x9FU

#define W25Q64_STATUS_BUSY       0x01U
#define W25Q64_SPI_TIMEOUT_MS    100U
#define W25Q64_ERASE_TIMEOUT_MS  5000U
#define W25Q64_PROGRAM_TIMEOUT_MS 1000U

static void w25q64_cs_low(void)
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
}

static void w25q64_cs_high(void)
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

static void w25q64_address_bytes(uint32_t address, uint8_t out[3])
{
    out[0] = (uint8_t)((address >> 16) & 0xFFU);
    out[1] = (uint8_t)((address >> 8) & 0xFFU);
    out[2] = (uint8_t)(address & 0xFFU);
}

static HAL_StatusTypeDef w25q64_write_enable(void)
{
    uint8_t cmd = W25Q64_CMD_WRITE_ENABLE;
    HAL_StatusTypeDef status;

    w25q64_cs_low();
    status = HAL_SPI_Transmit(&hspi3, &cmd, 1U, W25Q64_SPI_TIMEOUT_MS);
    w25q64_cs_high();

    return status;
}

static HAL_StatusTypeDef w25q64_read_status1(uint8_t *status1)
{
    uint8_t cmd = W25Q64_CMD_READ_STATUS1;
    HAL_StatusTypeDef status;

    if (status1 == NULL)
    {
        return HAL_ERROR;
    }

    w25q64_cs_low();
    status = HAL_SPI_Transmit(&hspi3, &cmd, 1U, W25Q64_SPI_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        status = HAL_SPI_Receive(&hspi3, status1, 1U, W25Q64_SPI_TIMEOUT_MS);
    }
    w25q64_cs_high();

    return status;
}

HAL_StatusTypeDef W25Q64_WaitReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t status1 = 0U;

    do
    {
        HAL_StatusTypeDef status = w25q64_read_status1(&status1);
        if (status != HAL_OK)
        {
            return status;
        }

        if ((status1 & W25Q64_STATUS_BUSY) == 0U)
        {
            return HAL_OK;
        }
    } while ((HAL_GetTick() - start) < timeout_ms);

    return HAL_TIMEOUT;
}

HAL_StatusTypeDef W25Q64_ReadJedecId(W25Q64_JedecId_t *jedec_id)
{
    uint8_t cmd = W25Q64_CMD_JEDEC_ID;
    uint8_t raw[3] = {0};
    HAL_StatusTypeDef status;

    if (jedec_id == NULL)
    {
        return HAL_ERROR;
    }

    w25q64_cs_low();
    status = HAL_SPI_Transmit(&hspi3, &cmd, 1U, W25Q64_SPI_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        status = HAL_SPI_Receive(&hspi3, raw, sizeof(raw), W25Q64_SPI_TIMEOUT_MS);
    }
    w25q64_cs_high();

    if (status == HAL_OK)
    {
        jedec_id->manufacturer_id = raw[0];
        jedec_id->memory_type = raw[1];
        jedec_id->capacity = raw[2];
    }

    return status;
}

HAL_StatusTypeDef W25Q64_Init(W25Q64_JedecId_t *jedec_id)
{
    W25Q64_JedecId_t local_id;
    HAL_StatusTypeDef status;

    w25q64_cs_high();
    status = W25Q64_ReadJedecId(&local_id);
    if (status != HAL_OK)
    {
        return status;
    }

    if (jedec_id != NULL)
    {
        *jedec_id = local_id;
    }

    if (local_id.manufacturer_id != W25Q64_EXPECTED_MANUFACTURER_ID
        || local_id.memory_type != W25Q64_EXPECTED_MEMORY_TYPE
        || local_id.capacity != W25Q64_EXPECTED_CAPACITY)
    {
        return HAL_ERROR;
    }

    return W25Q64_WaitReady(W25Q64_PROGRAM_TIMEOUT_MS);
}

HAL_StatusTypeDef W25Q64_Read(uint32_t address, uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0U || address >= W25Q64_TOTAL_BYTES
        || length > (W25Q64_TOTAL_BYTES - address))
    {
        return HAL_ERROR;
    }

    while (length > 0U)
    {
        uint8_t cmd[4];
        uint32_t chunk = (length > 60000UL) ? 60000UL : length;
        HAL_StatusTypeDef status;

        cmd[0] = W25Q64_CMD_READ_DATA;
        w25q64_address_bytes(address, &cmd[1]);

        w25q64_cs_low();
        status = HAL_SPI_Transmit(&hspi3, cmd, sizeof(cmd), W25Q64_SPI_TIMEOUT_MS);
        if (status == HAL_OK)
        {
            status = HAL_SPI_Receive(&hspi3, data, (uint16_t)chunk, W25Q64_SPI_TIMEOUT_MS);
        }
        w25q64_cs_high();

        if (status != HAL_OK)
        {
            return status;
        }

        address += chunk;
        data += chunk;
        length -= chunk;
    }

    return HAL_OK;
}

HAL_StatusTypeDef W25Q64_Program(uint32_t address, const uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0U || address >= W25Q64_TOTAL_BYTES
        || length > (W25Q64_TOTAL_BYTES - address))
    {
        return HAL_ERROR;
    }

    while (length > 0U)
    {
        uint8_t cmd[4];
        uint32_t page_space = W25Q64_PAGE_BYTES - (address % W25Q64_PAGE_BYTES);
        uint32_t chunk = (length < page_space) ? length : page_space;
        HAL_StatusTypeDef status;

        if (chunk > W25Q64_PAGE_BYTES)
        {
            chunk = W25Q64_PAGE_BYTES;
        }

        status = w25q64_write_enable();
        if (status != HAL_OK)
        {
            return status;
        }

        cmd[0] = W25Q64_CMD_PAGE_PROGRAM;
        w25q64_address_bytes(address, &cmd[1]);

        w25q64_cs_low();
        status = HAL_SPI_Transmit(&hspi3, cmd, sizeof(cmd), W25Q64_SPI_TIMEOUT_MS);
        if (status == HAL_OK)
        {
            status = HAL_SPI_Transmit(&hspi3, (uint8_t *)data, (uint16_t)chunk, W25Q64_SPI_TIMEOUT_MS);
        }
        w25q64_cs_high();

        if (status != HAL_OK)
        {
            return status;
        }

        status = W25Q64_WaitReady(W25Q64_PROGRAM_TIMEOUT_MS);
        if (status != HAL_OK)
        {
            return status;
        }

        address += chunk;
        data += chunk;
        length -= chunk;
    }

    return HAL_OK;
}

HAL_StatusTypeDef W25Q64_EraseSector(uint32_t address)
{
    uint8_t cmd[4];
    HAL_StatusTypeDef status;

    if (address >= W25Q64_TOTAL_BYTES)
    {
        return HAL_ERROR;
    }

    address -= address % W25Q64_SECTOR_BYTES;

    status = w25q64_write_enable();
    if (status != HAL_OK)
    {
        return status;
    }

    cmd[0] = W25Q64_CMD_SECTOR_ERASE;
    w25q64_address_bytes(address, &cmd[1]);

    w25q64_cs_low();
    status = HAL_SPI_Transmit(&hspi3, cmd, sizeof(cmd), W25Q64_SPI_TIMEOUT_MS);
    w25q64_cs_high();

    if (status != HAL_OK)
    {
        return status;
    }

    return W25Q64_WaitReady(W25Q64_ERASE_TIMEOUT_MS);
}
