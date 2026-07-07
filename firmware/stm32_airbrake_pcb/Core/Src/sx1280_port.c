/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 17:44:48 -04:00
 *
 * What this file does:
 *   This file is the Airbrake PCB-specific SX1280 port layer. It controls chip select, reset, BUSY, and SPI1 transfers.
 *
 * Process flow:
 *   The radio driver sets CS low, sends bytes through this port, sets CS high, and waits for BUSY to clear. Reset pulses are used at startup and after timeouts.
 *
 * Main variables and what can be changed:
 *   Timeout values inside wait and SPI calls can be tuned if radio transactions need more time. Pin assignments come from main.h.
 *
 * Assumptions:
 *   GPIO and SPI1 have already been initialized by CubeMX startup code.
 *
 * What is missing:
 *   No DMA transfer support or separate logging when BUSY timeout occurs is implemented.
 */

#include "sx1280_port.h"

/*
 * SX1280 Airbrake PCB port layer.
 *
 * This file contains the only direct pin/SPI knowledge needed by sx1280.c.  If
 * the PCB routing changes, update this port layer and main.h pin names instead
 * of touching the radio protocol driver.
 */

extern SPI_HandleTypeDef hspi1;

HAL_StatusTypeDef SX1280_PortInit(void)
{
    /*
     * Idle state: CS high means the radio is not selected, and NRESET high keeps
     * it out of reset.  GPIO mode/alternate-function setup is generated in main.c.
     */
    SX1280_PortCsHigh();
    HAL_GPIO_WritePin(LORA_NRESET_GPIO_Port, LORA_NRESET_Pin, GPIO_PIN_SET);
    return HAL_OK;
}

void SX1280_PortCsLow(void)
{
    HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_RESET);
}

void SX1280_PortCsHigh(void)
{
    HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_SET);
}

void SX1280_PortReset(void)
{
    /*
     * Reset timing is intentionally conservative for bench reliability.  The
     * driver calls this at startup and after radio timeouts to return to a known
     * state before reconfiguring LoRa mode.
     */
    SX1280_PortCsHigh();

    HAL_GPIO_WritePin(LORA_NRESET_GPIO_Port, LORA_NRESET_Pin, GPIO_PIN_SET);
    HAL_Delay(5);

    HAL_GPIO_WritePin(LORA_NRESET_GPIO_Port, LORA_NRESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);

    HAL_GPIO_WritePin(LORA_NRESET_GPIO_Port, LORA_NRESET_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
}

HAL_StatusTypeDef SX1280_PortWaitBusyLow(uint32_t timeout_ms)
{
    /*
     * BUSY high means the SX1280 is processing the previous command.  Starting
     * another SPI transaction too early can corrupt the radio command stream.
     */
    uint32_t start = HAL_GetTick();

    while (HAL_GPIO_ReadPin(LORA_BUSY_GPIO_Port, LORA_BUSY_Pin) == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef SX1280_PortSpiTxRx(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    /* The radio command layer owns CS and BUSY timing; this function only clocks bytes. */
    return HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, len, 100);
}
