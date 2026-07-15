/*
 * AMBAR SX1280 LORA MODEM DRIVER - IMPLEMENTATION
 *
 * Purpose
 *   Configures the flight-side SX1280 LoRa modem and moves opaque packet bytes
 *   between radio_bridge and the radio FIFO.  It owns command framing, modem
 *   parameters, IRQ polling/clearing, continuous-RX restoration, and the
 *   cooperative transmit state.  Packet schemas and command meaning stay in
 *   radio_bridge; SPI/GPIO wiring stays in sx1280_port.
 *
 * Data/control flow
 *   InitLoRa resets the radio, selects LoRa packet mode, applies the matched
 *   air/ground profile, routes IRQs to DIO1, and enters continuous receive.
 *   StartTransmit stages one packet and starts TX; ServiceTransmit polls IRQs
 *   across later main-loop passes and restores RX on completion.  RX polling
 *   validates radio IRQ/length state before copying FIFO bytes to the bridge.
 *   This is the modem portion of CODE_GUIDE.md [ARCH-6].
 *
 * Section map
 *   1. Datasheet opcodes, registers, and IRQ masks
 *   2. Ground-station-matched LoRa profile and TX runtime state
 *   3. SPI command/register/FIFO helpers
 *   4. Modem configuration helpers
 *   5. IRQ and receive-status helpers
 *   6. Continuous RX and full initialization
 *   7. Cooperative/blocking TX and failure recovery
 *   8. Nonblocking receive poll
 *
 * Safety and assumptions
 *   Frequency, SF, bandwidth, coding rate, sync word, header mode, preamble, IQ,
 *   and CRC must remain synchronized with the LILYGO sketch.  Every transaction
 *   uses bounded BUSY/SPI waits, and TX failure performs full reinitialization so
 *   the modem cannot silently remain out of RX.  The blocking TX wrapper can
 *   delay the cooperative scheduler.  Receive callers must provide storage for
 *   SX1280_MAX_PAYLOAD_LEN bytes.  This driver provides neither authentication,
 *   retransmission/ACK policy, adaptive data rate, nor link encryption.
 */

#include "sx1280.h"
#include "sx1280_port.h"
#include <string.h>

/* ===================== DATASHEET COMMAND, REGISTER, AND IRQ MAP ===================== */

/* SX1280 command opcodes */
#define SX1280_CMD_GET_STATUS            0xC0
#define SX1280_CMD_WRITE_REGISTER        0x18
#define SX1280_CMD_READ_REGISTER         0x19
#define SX1280_CMD_WRITE_BUFFER          0x1A
#define SX1280_CMD_READ_BUFFER           0x1B
#define SX1280_CMD_SET_STANDBY           0x80
#define SX1280_CMD_SET_RX                0x82
#define SX1280_CMD_SET_TX                0x83
#define SX1280_CMD_SET_PACKET_TYPE       0x8A
#define SX1280_CMD_SET_RF_FREQUENCY      0x86
#define SX1280_CMD_SET_TX_PARAMS         0x8E
#define SX1280_CMD_SET_BUFFER_BASE_ADDR  0x8F
#define SX1280_CMD_SET_MOD_PARAMS        0x8B
#define SX1280_CMD_SET_PACKET_PARAMS     0x8C
#define SX1280_CMD_SET_DIO_IRQ_PARAMS    0x8D
#define SX1280_CMD_GET_RX_BUFFER_STATUS  0x17
#define SX1280_CMD_GET_IRQ_STATUS        0x15
#define SX1280_CMD_CLEAR_IRQ_STATUS      0x97

#define SX1280_REG_LORA_SYNC_WORD_MSB    0x0944

/* Packet type */
#define SX1280_PACKET_TYPE_LORA          0x01

/* IRQ bits */
#define SX1280_IRQ_TX_DONE               0x0001
#define SX1280_IRQ_RX_DONE               0x0002
#define SX1280_IRQ_HEADER_VALID          0x0010
#define SX1280_IRQ_HEADER_ERROR          0x0020
#define SX1280_IRQ_CRC_ERROR             0x0040
#define SX1280_IRQ_RX_TX_TIMEOUT         0x4000

#define SX1280_IRQ_ALL                   0xFFFF

/* ===================== MATCHED AIR/GROUND LORA PROFILE ===================== */

/* Flight and LILYGO ground station both operate at 2445 MHz. */
#define SX1280_RF_FREQUENCY_HZ           2445000000UL

/* LoRa settings matching the LILYGO sketch */
#define SX1280_LORA_SF7                  0x70
#define SX1280_LORA_BW_203_KHZ           0x34
#define SX1280_LORA_CR_4_5               0x01

#define SX1280_LORA_HEADER_EXPLICIT      0x00
#define SX1280_LORA_CRC_ON               0x20
#define SX1280_LORA_IQ_NORMAL            0x40
#define SX1280_LORA_PREAMBLE_12_SYMBOLS  0x16
#define SX1280_LORA_SYNC_WORD_PRIVATE    0x12
#define SX1280_LORA_SYNC_WORD_CONTROL    0x44

#define SX1280_STDBY_RC                  0x00

/* ===================== COOPERATIVE TRANSMIT STATE ===================== */

/* A single in-flight packet owns the modem until ServiceTransmit resolves it. */
static bool sx1280_tx_active = false;
static uint32_t sx1280_tx_started_ms = 0U;
static uint32_t sx1280_tx_timeout_ms = 0U;

/* ===================== SPI COMMAND, REGISTER, AND FIFO HELPERS ===================== */

static HAL_StatusTypeDef sx1280_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    /*
     * Every SX1280 command follows the same safe transaction shape:
     * wait BUSY low, assert CS, clock bytes, release CS, then wait BUSY low again.
     */
    HAL_StatusTypeDef status;

    status = SX1280_PortWaitBusyLow(100);
    if (status != HAL_OK)
    {
        return status;
    }

    SX1280_PortCsLow();
    status = SX1280_PortSpiTxRx(tx, rx, len);
    SX1280_PortCsHigh();

    if (status != HAL_OK)
    {
        return status;
    }

    return SX1280_PortWaitBusyLow(100);
}

static HAL_StatusTypeDef sx1280_command(uint8_t opcode, const uint8_t *args, uint8_t arg_len)
{
    /*
     * Small command helper for commands whose payload fits in this stack buffer.
     * FIFO reads/writes use dedicated helpers below because packets can be larger.
     */
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};

    if (arg_len > 15)
    {
        return HAL_ERROR;
    }

    tx[0] = opcode;

    if ((args != 0) && (arg_len > 0))
    {
        memcpy(&tx[1], args, arg_len);
    }

    return sx1280_transfer(tx, rx, (uint16_t)(arg_len + 1));
}

static HAL_StatusTypeDef sx1280_write_register(uint16_t address, uint8_t value)
{
    /* Register addresses are sent big-endian as required by the SX1280 command. */
    uint8_t tx[4];
    uint8_t rx[4] = {0};

    tx[0] = SX1280_CMD_WRITE_REGISTER;
    tx[1] = (uint8_t)(address >> 8);
    tx[2] = (uint8_t)(address & 0xFF);
    tx[3] = value;

    return sx1280_transfer(tx, rx, sizeof(tx));
}

static HAL_StatusTypeDef sx1280_write_buffer(uint8_t offset, const uint8_t *data, uint8_t len)
{
    /*
     * The radio FIFO starts at offset 0 for both TX and RX in this design.  Static
     * buffers avoid large stack use inside the telemetry path.
     */
    static uint8_t tx[2 + SX1280_MAX_PAYLOAD_LEN];
    static uint8_t rx[2 + SX1280_MAX_PAYLOAD_LEN];

    if (len > SX1280_MAX_PAYLOAD_LEN)
    {
        return HAL_ERROR;
    }

    tx[0] = SX1280_CMD_WRITE_BUFFER;
    tx[1] = offset;
    memcpy(&tx[2], data, len);

    return sx1280_transfer(tx, rx, (uint16_t)(len + 2));
}

static HAL_StatusTypeDef sx1280_read_buffer(uint8_t offset, uint8_t *data, uint8_t len)
{
    /*
     * READ_BUFFER returns a status/dummy byte before payload data, so rx[3] is
     * the first actual packet byte.
     */
    static uint8_t tx[3 + SX1280_MAX_PAYLOAD_LEN];
    static uint8_t rx[3 + SX1280_MAX_PAYLOAD_LEN];
    HAL_StatusTypeDef status;

    if (len > SX1280_MAX_PAYLOAD_LEN)
    {
        return HAL_ERROR;
    }

    tx[0] = SX1280_CMD_READ_BUFFER;
    tx[1] = offset;
    tx[2] = 0x00;

    status = sx1280_transfer(tx, rx, (uint16_t)(len + 3));
    if (status != HAL_OK)
    {
        return status;
    }

    memcpy(data, &rx[3], len);
    return HAL_OK;
}

/* ===================== MODEM CONFIGURATION HELPERS ===================== */

static HAL_StatusTypeDef sx1280_set_standby(void)
{
    uint8_t args[1] = {SX1280_STDBY_RC};
    return sx1280_command(SX1280_CMD_SET_STANDBY, args, sizeof(args));
}

static HAL_StatusTypeDef sx1280_set_packet_type_lora(void)
{
    uint8_t args[1] = {SX1280_PACKET_TYPE_LORA};
    return sx1280_command(SX1280_CMD_SET_PACKET_TYPE, args, sizeof(args));
}

static HAL_StatusTypeDef sx1280_set_rf_frequency(uint32_t freq_hz)
{
    /*
     * SX1280 RF frequency register:
     * rfFrequency = frequency_Hz * 2^18 / 52 MHz
     */
    uint32_t reg = (uint32_t)((((uint64_t)freq_hz << 18) + 26000000ULL) / 52000000ULL);

    uint8_t args[3];
    args[0] = (uint8_t)(reg >> 16);
    args[1] = (uint8_t)(reg >> 8);
    args[2] = (uint8_t)(reg);

    return sx1280_command(SX1280_CMD_SET_RF_FREQUENCY, args, sizeof(args));
}

static HAL_StatusTypeDef sx1280_set_buffer_base_address(void)
{
    uint8_t args[2] = {
        0x00,   /* TX base address */
        0x00    /* RX base address */
    };

    return sx1280_command(SX1280_CMD_SET_BUFFER_BASE_ADDR, args, sizeof(args));
}

static HAL_StatusTypeDef sx1280_set_modulation_params(void)
{
    /*
     * These values must match the LILYGO ground-side sketch.  If the link stops
     * receiving, check spreading factor, bandwidth, coding rate, and sync word
     * before debugging packet content.
     */
    uint8_t args[3] = {
        SX1280_LORA_SF7,
        SX1280_LORA_BW_203_KHZ,
        SX1280_LORA_CR_4_5
    };

    return sx1280_command(SX1280_CMD_SET_MOD_PARAMS, args, sizeof(args));
}

static HAL_StatusTypeDef sx1280_set_lora_sync_word(uint8_t sync_word)
{
    /*
     * SX1280 stores LoRa sync word nibbles spread across two registers with
     * control nibbles preserved.  This helper keeps that packing in one place.
     */
    uint8_t sync_msb =
        (uint8_t)((sync_word & 0xF0) | ((SX1280_LORA_SYNC_WORD_CONTROL & 0xF0) >> 4));
    uint8_t sync_lsb =
        (uint8_t)(((sync_word & 0x0F) << 4) | (SX1280_LORA_SYNC_WORD_CONTROL & 0x0F));
    HAL_StatusTypeDef status;

    status = sx1280_write_register(SX1280_REG_LORA_SYNC_WORD_MSB, sync_msb);
    if (status != HAL_OK) return status;

    return sx1280_write_register(SX1280_REG_LORA_SYNC_WORD_MSB + 1U, sync_lsb);
}

static HAL_StatusTypeDef sx1280_set_packet_params(uint8_t payload_len)
{
    uint8_t args[7] = {
        /* SX1280 packs preamble as exponent/mantissa: 6 * 2^1 = 12. */
        SX1280_LORA_PREAMBLE_12_SYMBOLS,
        SX1280_LORA_HEADER_EXPLICIT,  /* explicit header */
        payload_len,                  /* payload length */
        SX1280_LORA_CRC_ON,           /* CRC enabled */
        SX1280_LORA_IQ_NORMAL,        /* normal IQ */
        0x00,
        0x00
    };

    return sx1280_command(SX1280_CMD_SET_PACKET_PARAMS, args, sizeof(args));
}

static HAL_StatusTypeDef sx1280_set_tx_params(void)
{
    /*
     * power = 18 gives about 0 dBm using Pout = -18 + power.
     * Use a conservative value for bench testing.
     * rampTime = 0xE0 is 20 us.
     */
    uint8_t args[2] = {
        18,
        0xE0
    };

    return sx1280_command(SX1280_CMD_SET_TX_PARAMS, args, sizeof(args));
}

static HAL_StatusTypeDef sx1280_set_dio_irq_params(void)
{
    /*
     * Only DIO1 is wired into the current firmware path.  All enabled radio IRQs
     * are routed there so radio_bridge.c can wake up on TX/RX/error events.
     */
    uint16_t irq_mask =
        SX1280_IRQ_TX_DONE |
        SX1280_IRQ_RX_DONE |
        SX1280_IRQ_HEADER_ERROR |
        SX1280_IRQ_CRC_ERROR |
        SX1280_IRQ_RX_TX_TIMEOUT;

    uint8_t args[8];

    args[0] = (uint8_t)(irq_mask >> 8);
    args[1] = (uint8_t)(irq_mask);

    /* Route all enabled IRQs to DIO1 */
    args[2] = (uint8_t)(irq_mask >> 8);
    args[3] = (uint8_t)(irq_mask);

    /* DIO2 disabled */
    args[4] = 0x00;
    args[5] = 0x00;

    /* DIO3 disabled */
    args[6] = 0x00;
    args[7] = 0x00;

    return sx1280_command(SX1280_CMD_SET_DIO_IRQ_PARAMS, args, sizeof(args));
}

/* ===================== IRQ AND RECEIVE-STATUS HELPERS ===================== */

static HAL_StatusTypeDef sx1280_clear_irq(uint16_t irq_mask)
{
    uint8_t args[2];

    args[0] = (uint8_t)(irq_mask >> 8);
    args[1] = (uint8_t)(irq_mask);

    return sx1280_command(SX1280_CMD_CLEAR_IRQ_STATUS, args, sizeof(args));
}

static HAL_StatusTypeDef sx1280_get_irq_status(uint16_t *irq_status)
{
    /*
     * The IRQ status command returns two useful status bytes after the opcode and
     * dummy byte.  The bit definitions above are copied from the SX1280 datasheet.
     */
    uint8_t tx[4] = {
        SX1280_CMD_GET_IRQ_STATUS,
        0x00,
        0x00,
        0x00
    };

    uint8_t rx[4] = {0};
    HAL_StatusTypeDef status;

    status = sx1280_transfer(tx, rx, sizeof(tx));
    if (status != HAL_OK)
    {
        return status;
    }

    *irq_status = ((uint16_t)rx[2] << 8) | rx[3];
    return HAL_OK;
}

static HAL_StatusTypeDef sx1280_get_rx_buffer_status(uint8_t *payload_len, uint8_t *rx_start_pointer)
{
    /* Return the radio-reported packet length and FIFO start address as a pair. */
    uint8_t tx[4] = {
        SX1280_CMD_GET_RX_BUFFER_STATUS,
        0x00,
        0x00,
        0x00
    };

    uint8_t rx[4] = {0};
    HAL_StatusTypeDef status;

    status = sx1280_transfer(tx, rx, sizeof(tx));
    if (status != HAL_OK)
    {
        return status;
    }

    *payload_len = rx[2];
    *rx_start_pointer = rx[3];

    return HAL_OK;
}

/* ===================== CONTINUOUS RECEIVE AND INITIALIZATION ===================== */

HAL_StatusTypeDef SX1280_StartRxContinuous(void)
{
    /*
     * Continuous RX is the resting radio state.  Telemetry transmit temporarily
     * leaves RX, then SX1280_Transmit() calls this again before returning.
     */
    HAL_StatusTypeDef status;

    status = sx1280_set_standby();
    if (status != HAL_OK) return status;

    status = sx1280_set_packet_params(SX1280_MAX_PAYLOAD_LEN);
    if (status != HAL_OK) return status;

    status = sx1280_clear_irq(SX1280_IRQ_ALL);
    if (status != HAL_OK) return status;

    /*
     * SetRx(periodBase, periodBaseCount)
     * periodBase = 0x03 = 4 ms
     * periodBaseCount = 0xFFFF = continuous RX
     */
    uint8_t args[3] = {
        0x03,
        0xFF,
        0xFF
    };

    return sx1280_command(SX1280_CMD_SET_RX, args, sizeof(args));
}

HAL_StatusTypeDef SX1280_InitLoRa(void)
{
    /*
     * Full radio initialization after reset.  Any failure here is reported up to
     * main.c/ambar_app.c so telemetry can show the radio fault instead of hiding it.
     */
    HAL_StatusTypeDef status;

    /* A full reset always cancels any in-progress software TX state. */
    sx1280_tx_active = false;
    sx1280_tx_started_ms = 0U;
    sx1280_tx_timeout_ms = 0U;

    status = SX1280_PortInit();
    if (status != HAL_OK) return status;

    SX1280_PortReset();

    status = SX1280_PortWaitBusyLow(500);
    if (status != HAL_OK) return status;

    status = sx1280_set_standby();
    if (status != HAL_OK) return status;

    status = sx1280_set_packet_type_lora();
    if (status != HAL_OK) return status;

    status = sx1280_set_rf_frequency(SX1280_RF_FREQUENCY_HZ);
    if (status != HAL_OK) return status;

    status = sx1280_set_buffer_base_address();
    if (status != HAL_OK) return status;

    status = sx1280_set_modulation_params();
    if (status != HAL_OK) return status;

    status = sx1280_set_lora_sync_word(SX1280_LORA_SYNC_WORD_PRIVATE);
    if (status != HAL_OK) return status;

    /*
     * Semtech-recommended LoRa register settings commonly used by SX1280 drivers:
     * 0x0925 = 0x37 for SF7/SF8 operation.
     * 0x093C = 0x01 for frequency error correction mode.
     */
    status = sx1280_write_register(0x0925, 0x37);
    if (status != HAL_OK) return status;

    status = sx1280_write_register(0x093C, 0x01);
    if (status != HAL_OK) return status;

    status = sx1280_set_tx_params();
    if (status != HAL_OK) return status;

    status = sx1280_set_dio_irq_params();
    if (status != HAL_OK) return status;

    status = sx1280_clear_irq(SX1280_IRQ_ALL);
    if (status != HAL_OK) return status;

    return SX1280_StartRxContinuous();
}

/* ===================== TRANSMIT STATE MACHINE AND FAILURE RECOVERY ===================== */

static HAL_StatusTypeDef sx1280_recover_after_tx_failure(HAL_StatusTypeDef failure)
{
    /*
     * TX setup and completion both leave continuous RX temporarily.  Always
     * perform a full known-good reinitialization before reporting a failed TX;
     * otherwise the bridge can look idle while the modem remains in standby.
     */
    sx1280_tx_active = false;
    sx1280_tx_started_ms = 0U;
    sx1280_tx_timeout_ms = 0U;

    return SX1280_InitLoRa() == HAL_OK ? failure : HAL_ERROR;
}

bool SX1280_IsTransmitBusy(void)
{
    /* Read-only ownership query used by radio_bridge scheduling. */
    return sx1280_tx_active;
}

HAL_StatusTypeDef SX1280_StartTransmit(const uint8_t *data,
                                       uint8_t len,
                                       uint32_t timeout_ms)
{
    /* Validate/stage the complete packet before publishing the active TX state. */
    HAL_StatusTypeDef status;

    if ((data == 0) || (len == 0) || (len > SX1280_MAX_PAYLOAD_LEN) ||
        (timeout_ms == 0U))
    {
        return HAL_ERROR;
    }

    if (sx1280_tx_active)
    {
        return HAL_BUSY;
    }

    status = sx1280_set_standby();
    if (status != HAL_OK) return sx1280_recover_after_tx_failure(status);

    status = sx1280_set_packet_params(len);
    if (status != HAL_OK) return sx1280_recover_after_tx_failure(status);

    status = sx1280_clear_irq(SX1280_IRQ_ALL);
    if (status != HAL_OK) return sx1280_recover_after_tx_failure(status);

    status = sx1280_write_buffer(0x00, data, len);
    if (status != HAL_OK) return sx1280_recover_after_tx_failure(status);

    /*
     * SetTx with no timeout:
     * periodBase = 0x00, periodBaseCount = 0x0000
     */
    uint8_t tx_args[3] = {
        0x00,
        0x00,
        0x00
    };

    status = sx1280_command(SX1280_CMD_SET_TX, tx_args, sizeof(tx_args));
    if (status != HAL_OK) return sx1280_recover_after_tx_failure(status);

    sx1280_tx_started_ms = HAL_GetTick();
    sx1280_tx_timeout_ms = timeout_ms;
    sx1280_tx_active = true;

    return HAL_OK;
}

HAL_StatusTypeDef SX1280_ServiceTransmit(void)
{
    /* Resolve TX_DONE, recover on radio/software timeout, or yield HAL_BUSY. */
    HAL_StatusTypeDef status;
    uint16_t irq_status = 0U;

    if (!sx1280_tx_active)
    {
        return HAL_OK;
    }

    status = sx1280_get_irq_status(&irq_status);

    if ((status == HAL_OK) && ((irq_status & SX1280_IRQ_TX_DONE) != 0U))
    {
        status = sx1280_clear_irq(irq_status);
        if (status != HAL_OK)
        {
            return sx1280_recover_after_tx_failure(status);
        }

        status = SX1280_StartRxContinuous();
        if (status != HAL_OK)
        {
            return sx1280_recover_after_tx_failure(status);
        }

        sx1280_tx_active = false;
        sx1280_tx_started_ms = 0U;
        sx1280_tx_timeout_ms = 0U;
        return HAL_OK;
    }

    if (((status == HAL_OK) &&
         ((irq_status & SX1280_IRQ_RX_TX_TIMEOUT) != 0U)) ||
        ((uint32_t)(HAL_GetTick() - sx1280_tx_started_ms) >= sx1280_tx_timeout_ms))
    {
        /* Reinitialize into continuous RX so one failed packet cannot wedge radio. */
        (void)sx1280_clear_irq(SX1280_IRQ_ALL);
        return sx1280_recover_after_tx_failure(HAL_TIMEOUT);
    }

    /* A transient SPI status read failure is retried until the TX timeout. */
    return HAL_BUSY;
}

HAL_StatusTypeDef SX1280_Transmit(const uint8_t *data, uint8_t len, uint32_t timeout_ms)
{
    /*
     * Retain a blocking wrapper for short boot and command-response packets.
     * Periodic full telemetry uses Start/Service so sensor scheduling continues.
     */
    HAL_StatusTypeDef status = SX1280_StartTransmit(data, len, timeout_ms);

    if (status != HAL_OK)
    {
        return status;
    }

    do
    {
        status = SX1280_ServiceTransmit();
    }
    while (status == HAL_BUSY);

    return status;
}

/* ===================== NONBLOCKING RECEIVE POLL ===================== */

HAL_StatusTypeDef SX1280_ReadPacketIfAvailable(uint8_t *data, uint8_t *len)
{
    /*
     * Nonblocking receive poll:
     *   HAL_OK   -> packet copied to data and len.
     *   HAL_BUSY -> no packet waiting.
     *   HAL_ERROR-> CRC/header/timeout or invalid length.
     */
    HAL_StatusTypeDef status;
    uint16_t irq_status;
    uint8_t payload_len;
    uint8_t start_pointer;

    if ((data == 0) || (len == 0))
    {
        return HAL_ERROR;
    }

    if (sx1280_tx_active)
    {
        return HAL_BUSY;
    }

    status = sx1280_get_irq_status(&irq_status);
    if (status != HAL_OK)
    {
        return status;
    }

    if (irq_status == 0)
    {
        return HAL_BUSY;
    }

    if ((irq_status & (SX1280_IRQ_HEADER_ERROR | SX1280_IRQ_CRC_ERROR | SX1280_IRQ_RX_TX_TIMEOUT)) != 0)
    {
        sx1280_clear_irq(irq_status);
        SX1280_StartRxContinuous();
        return HAL_ERROR;
    }

    if ((irq_status & SX1280_IRQ_RX_DONE) == 0)
    {
        sx1280_clear_irq(irq_status);
        return HAL_BUSY;
    }

    status = sx1280_get_rx_buffer_status(&payload_len, &start_pointer);
    if (status != HAL_OK)
    {
        sx1280_clear_irq(irq_status);
        SX1280_StartRxContinuous();
        return status;
    }

    if (payload_len > SX1280_MAX_PAYLOAD_LEN)
    {
        sx1280_clear_irq(irq_status);
        SX1280_StartRxContinuous();
        return HAL_ERROR;
    }

    status = sx1280_read_buffer(start_pointer, data, payload_len);

    sx1280_clear_irq(irq_status);
    SX1280_StartRxContinuous();

    if (status != HAL_OK)
    {
        return status;
    }

    *len = payload_len;
    return HAL_OK;
}
