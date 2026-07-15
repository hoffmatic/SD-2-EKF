/*
 * AMBAR SX1280 BOARD PORT - IMPLEMENTATION
 *
 * Purpose
 *   Binds the generic SX1280 modem driver to the Airbrake PCB's Cube-generated
 *   SPI1 handle and LORA control pins.  All direct GPIO/SPI HAL calls required by
 *   sx1280.c are intentionally concentrated here.
 *
 * Hardware flow
 *   PortInit establishes inactive control levels.  A modem transaction waits for
 *   BUSY low, drives active-low CS around PortSpiTxRx, then waits for BUSY low
 *   again.  PortReset supplies a conservative reset pulse and settling delay for
 *   startup or recovery.  See CODE_GUIDE.md [ARCH-6].
 *
 * Section map
 *   1. Cube-generated peripheral binding
 *   2. Idle levels, chip select, and hardware reset
 *   3. BUSY polling and blocking SPI transfer
 *
 * Safety and assumptions
 *   MX_GPIO_Init() and MX_SPI1_Init() must run first.  Reset and SPI operations
 *   block for bounded intervals, so normal packet flow should use the higher-level
 *   cooperative transmit service.  CS is always returned high by the command
 *   layer even when SPI reports an error.  Timeouts are reported to sx1280.c;
 *   this port performs no radio reconfiguration or retry policy.  Pin changes
 *   should be regenerated through CubeMX and reflected here, not scattered above.
 */

#include "sx1280_port.h"

/* ===================== CUBE-GENERATED PERIPHERAL BINDING ===================== */

extern SPI_HandleTypeDef hspi1;

/* ===================== IDLE LEVELS, CHIP SELECT, AND RESET ===================== */

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
    /* Active-low CS ownership stays with sx1280_transfer(). */
    HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_RESET);
}

void SX1280_PortCsHigh(void)
{
    /* The radio must be deselected between every command transaction. */
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

/* ===================== BUSY PACING AND SPI TRANSFER ===================== */

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
    /* Fixed HAL timeout bounds a failed SPI1 transaction; caller owns CS/BUSY. */
    return HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, len, 100);
}
