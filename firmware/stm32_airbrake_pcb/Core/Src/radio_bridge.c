/*
 * PROJECT FILE OVERVIEW
 * Comment made: 2026-07-07 17:44:48 -04:00
 *
 * What this file does:
 *   This file builds telemetry packets and services radio receive and heartbeat work. It keeps raw sensor fields while adding EKF estimate, command, and health information.
 *
 * Process flow:
 *   Startup initializes the SX1280 and sends a boot message. Telemetry writes a header, raw sensor sections, calculated fields, EKF fields, and text messages into one packet. The task also handles incoming packets and heartbeats.
 *
 * Main variables and what can be changed:
 *   TAG values define packet sections and must match the receiver. Payload_FloatToS32 scaling controls fixed-point precision. Heartbeat timing can be reduced if needed.
 *
 * Assumptions:
 *   Packet length stays under SX1280_MAX_PAYLOAD_LEN, and the receiver knows tag meanings and little-endian integer format.
 *
 * What is missing:
 *   No robust command protocol, telemetry acknowledgements, or flash logging of radio failures is implemented.
 */

#include "radio_bridge.h"
#include "ambar_app.h"
#include "ambar_actuator.h"
#include "rocket_sensors.h"
#include "sx1280.h"
#include <limits.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

/*
 * ===================== AMBAR EKF PCB INTEGRATION - UPDATED FILE =====================
 *
 * The original packet structure is preserved and expanded with three new tagged
 * fixed-point sections:
 *   0x60 = EKF estimate: altitude, velocity, acceleration, apogee, baro bias.
 *   0x61 = command: deploy fraction, predicted/target apogee, inhibit flags.
 *   0x62 = health: phase, healthy flag, rejected counts, last baro innovation.
 */

static volatile uint8_t radio_dio1_seen = 0;
/*
 * Set by HAL_GPIO_EXTI_Callback() when SX1280 DIO1 fires.  RadioBridge_Task()
 * also checks the pin level directly so bring-up still works if EXTI routing is
 * not fully configured yet.
 */
static uint32_t last_heartbeat_ms = 0;
/* Heartbeat counter proves STM32-to-LILYGO TX is alive even without RX traffic. */
static uint32_t heartbeat_counter = 0;
static volatile uint8_t command_pending = 0U;
static AmbarCommandResult_t pending_command;
static AmbarCommandResult_t last_command;

#define PAYLOAD_MAGIC       0xA5
#define PAYLOAD_VERSION     0x01
#define PAYLOAD_TYPE_DATA   0x01

#define TAG_IMU             0x10
#define TAG_BARO            0x20
#define TAG_MAGNET          0x30
#define TAG_CALC            0x40

#define TAG_STATUS_MSG      0x50
#define TAG_COMMAND_MSG     0x51
#define TAG_ERROR_MSG       0x52
#define TAG_FLIGHT_ESTIMATE 0x60
#define TAG_FLIGHT_COMMAND  0x61
#define TAG_FLIGHT_HEALTH   0x62
#define TAG_SYSTEM_STATUS   0x63
#define TAG_ACTUATOR_STATUS 0x64
#define TAG_APOGEE_DETAIL   0x65

#define IMU_COUNT           6
#define BARO_COUNT          2
#define MAGNET_COUNT        3
#define CALC_COUNT        	4
#define MESSAGE_COUNT       3
#define FLIGHT_ESTIMATE_COUNT 5
#define FLIGHT_COMMAND_COUNT  5
#define FLIGHT_HEALTH_COUNT   5
#define SYSTEM_STATUS_COUNT   5
#define ACTUATOR_STATUS_COUNT 5
#define APOGEE_DETAIL_COUNT   4

#define MAX_MESSAGE_LEN     48

static uint16_t payload_sequence = 0;

// The following functions add to the buffer and shift

static uint8_t Payload_CanAdd(uint8_t index, uint8_t needed)
{
    return (index <= SX1280_MAX_PAYLOAD_LEN)
        && (needed <= (uint8_t)(SX1280_MAX_PAYLOAD_LEN - index));
}

static uint8_t Payload_AddU8(uint8_t *buffer, uint8_t index, uint8_t value)
{
    /* Append one byte and return the next free payload index. */
    buffer[index++] = value;
    return index;
}

static uint8_t Payload_AddU16(uint8_t *buffer, uint8_t index, uint16_t value)
{
    /* Telemetry integers are little-endian so the receiver can decode cheaply. */
    buffer[index++] = (uint8_t)(value & 0xFF);
    buffer[index++] = (uint8_t)((value >> 8) & 0xFF);
    return index;
}

static uint8_t Payload_AddU32(uint8_t *buffer, uint8_t index, uint32_t value)
{
    buffer[index++] = (uint8_t)(value & 0xFF);
    buffer[index++] = (uint8_t)((value >> 8) & 0xFF);
    buffer[index++] = (uint8_t)((value >> 16) & 0xFF);
    buffer[index++] = (uint8_t)((value >> 24) & 0xFF);
    return index;
}

static uint8_t Payload_AddU16Array(uint8_t *buffer,
                                   uint8_t index,
                                   uint8_t tag,
                                   uint16_t *data,
                                   uint8_t count)
{
    /*
     * Tagged array layout:
     *   byte 0 = tag
     *   byte 1 = element count
     *   remaining bytes = little-endian values
     */
    if ((data == 0) || (count == 0))
    {
        return index;
    }

    if (!Payload_CanAdd(index, (uint8_t)(2U + (uint8_t)(2U * count))))
    {
        return index;
    }

    buffer[index++] = tag;
    buffer[index++] = count;

    for (uint8_t i = 0; i < count; i++)
    {
        index = Payload_AddU16(buffer, index, data[i]);
    }

    return index;
}

static uint8_t Payload_AddU32Array(uint8_t *buffer,
                                   uint8_t index,
                                   uint8_t tag,
                                   uint32_t *data,
                                   uint8_t count)
{
    if ((data == 0) || (count == 0))
    {
        return index;
    }

    if (!Payload_CanAdd(index, (uint8_t)(2U + (uint8_t)(4U * count))))
    {
        return index;
    }

    buffer[index++] = tag;
    buffer[index++] = count;

    for (uint8_t i = 0; i < count; i++)
    {
        index = Payload_AddU32(buffer, index, data[i]);
    }

    return index;
}

static uint8_t Payload_AddS32Array(uint8_t *buffer,
                                   uint8_t index,
                                   uint8_t tag,
                                   const int32_t *data,
                                   uint8_t count)
{
    /*
     * BEGIN AMBAR EKF PCB INTEGRATION - SIGNED FIXED-POINT PACKING
     *
     * EKF values can be negative, especially velocity, acceleration, bias, and
     * innovation.  They are packed little-endian as signed 32-bit two's-complement
     * fields, using the same byte writer as uint32_t after casting.
     */
    if ((data == 0) || (count == 0))
    {
        return index;
    }

    if (!Payload_CanAdd(index, (uint8_t)(2U + (uint8_t)(4U * count))))
    {
        return index;
    }

    buffer[index++] = tag;
    buffer[index++] = count;

    for (uint8_t i = 0; i < count; i++)
    {
        index = Payload_AddU32(buffer, index, (uint32_t)data[i]);
    }

    /* END AMBAR EKF PCB INTEGRATION - SIGNED FIXED-POINT PACKING */
    return index;
}

static int32_t Payload_FloatToS32(float value, float scale)
{
    /*
     * Convert float telemetry to deterministic fixed-point integers.  Returning
     * zero for NAN/INF keeps malformed estimator values from poisoning the radio
     * packet while the health fields still report the fault.
     */
    if (!isfinite(value))
    {
        return 0;
    }

    const float scaled = value * scale;
    if (scaled > (float)INT32_MAX)
    {
        return INT32_MAX;
    }

    if (scaled < (float)INT32_MIN)
    {
        return INT32_MIN;
    }

    return (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static uint8_t Payload_AddString(uint8_t *buffer,
                                 uint8_t index,
                                 uint8_t tag,
                                 const char *message)
{
    /*
     * Text fields are optional and length-prefixed.  They are intentionally kept
     * short so status messages cannot crowd out the fixed telemetry fields.
     */
    if (message == 0)
    {
        return index;
    }

    size_t len = strlen(message);

    if (len == 0)
    {
        return index;
    }

    /*
     * Need space for:
     * 1 byte tag
     * 1 byte length
     * N bytes message
     */
    if (index + 2 >= SX1280_MAX_PAYLOAD_LEN)
    {
        return index;
    }

    size_t remaining = SX1280_MAX_PAYLOAD_LEN - index - 2;

    /*
     * Because length is stored as one byte, one string field can hold
     * at most 255 bytes. In practice it is also limited by remaining
     * SX1280 payload space.
     */
    if (len > remaining)
    {
        len = remaining;
    }

    if (len > 255)
    {
        len = 255;
    }

    buffer[index++] = tag;
    buffer[index++] = (uint8_t)len;

    memcpy(&buffer[index], message, len);
    index += (uint8_t)len;

    return index;
}
// builds the payload

HAL_StatusTypeDef PayloadPipelineWithFlight(uint16_t *IMU,
                                            uint32_t *Baro,
                                            uint16_t *Magnet,
                                            uint16_t *Calc,
                                            const char *Message[3],
                                            uint8_t deployment_state,
                                            const AmbarFlightOutput_t *Flight,
                                            uint32_t actuator_inhibit_flags,
                                            const AmbarTelemetryExtra_t *Extra)
{
    /*
     * BEGIN AMBAR EKF PCB INTEGRATION - FLIGHT PAYLOAD ENTRY POINT
     *
     * The old PayloadPipeline() wrapper still calls this function with Flight=0.
     * When a flight output is supplied, the new tagged sections are appended
     * after the original raw/calculated fields.
     */
    static uint8_t txBuffer[SX1280_MAX_PAYLOAD_LEN];
    /*
     * txBuffer is static to avoid placing a 200-byte packet buffer on the stack
     * during periodic telemetry.
     */
    uint8_t index = 0;
    index = Payload_AddU8(txBuffer, index, PAYLOAD_TYPE_DATA);

    uint32_t current_time_ms = HAL_GetTick();

    if (IMU == 0 || Baro == 0 || Magnet == 0)
    {
        /* Raw sections are mandatory because the ground station expects them. */
        return HAL_ERROR;
    }

    /*
     * Simple application-level header.
     * The SX1280 will handle the actual radio packet structure.
     */
    index = Payload_AddU8(txBuffer, index, PAYLOAD_MAGIC);
    index = Payload_AddU8(txBuffer, index, PAYLOAD_VERSION);
    index = Payload_AddU8(txBuffer, index, PAYLOAD_TYPE_DATA);
    index = Payload_AddU16(txBuffer, index, payload_sequence++);
    /*
	 * Bytes 1-4: current time in milliseconds.
	 */
	index = Payload_AddU32(txBuffer, index, current_time_ms);

	/*
	 * Byte 5: deployment percentage.
	 * 0 = fully retracted
	 * 100 = fully deployed
	 */
	index = Payload_AddU8(txBuffer, index, deployment_state);

    /*
     * Sensor data sections.
     */
    index = Payload_AddU16Array(txBuffer, index, TAG_IMU, IMU, IMU_COUNT);
    index = Payload_AddU32Array(txBuffer, index, TAG_BARO, Baro, BARO_COUNT);
    index = Payload_AddU16Array(txBuffer, index, TAG_MAGNET, Magnet, MAGNET_COUNT);

    /*
     * Calculated values.
     */
    index = Payload_AddU16Array(txBuffer, index, TAG_CALC, Calc, CALC_COUNT);

    if (Flight != 0)
    {
        /*
         * BEGIN AMBAR EKF PCB INTEGRATION - EKF TELEMETRY FIELDS
         *
         * Scaling:
         *   meters, m/s, m/s^2, and bias use x100 -> centimeters-style precision.
         *   deploy_fraction uses x1000 -> 0.001 resolution.
         *   flags and enum values are copied directly.
         */
        const int32_t estimate_fields[FLIGHT_ESTIMATE_COUNT] = {
            Payload_FloatToS32(Flight->estimate.altitude_agl_m, 100.0f),
            Payload_FloatToS32(Flight->estimate.vertical_velocity_mps, 100.0f),
            Payload_FloatToS32(Flight->estimate.vertical_acceleration_mps2, 100.0f),
            Payload_FloatToS32(Flight->estimate.predicted_apogee_m, 100.0f),
            Payload_FloatToS32(Flight->estimate.barometer_bias_m, 100.0f)
        };
        const int32_t command_fields[FLIGHT_COMMAND_COUNT] = {
            Payload_FloatToS32(Flight->airbrake_command.deploy_fraction, 1000.0f),
            Payload_FloatToS32(Flight->airbrake_command.predicted_apogee_m, 100.0f),
            Payload_FloatToS32(Flight->airbrake_command.target_apogee_m, 100.0f),
            (int32_t)Flight->airbrake_command.inhibit_flags,
            (int32_t)actuator_inhibit_flags
        };
        const int32_t health_fields[FLIGHT_HEALTH_COUNT] = {
            (int32_t)Flight->phase,
            Flight->estimate.healthy ? 1 : 0,
            (int32_t)Flight->health.rejected_imu_samples,
            (int32_t)Flight->health.rejected_barometer_samples,
            Payload_FloatToS32(Flight->health.last_barometer_innovation_m, 100.0f)
        };

        index = Payload_AddS32Array(txBuffer,
                                    index,
                                    TAG_FLIGHT_ESTIMATE,
                                    estimate_fields,
                                    FLIGHT_ESTIMATE_COUNT);
        index = Payload_AddS32Array(txBuffer,
                                    index,
                                    TAG_FLIGHT_COMMAND,
                                    command_fields,
                                    FLIGHT_COMMAND_COUNT);
        index = Payload_AddS32Array(txBuffer,
                                    index,
                                    TAG_FLIGHT_HEALTH,
                                    health_fields,
                                    FLIGHT_HEALTH_COUNT);
        /* END AMBAR EKF PCB INTEGRATION - EKF TELEMETRY FIELDS */
    }

    if (Extra != 0)
    {
        /*
         * BEGIN AMBAR BENCH-GATED EXPANSION - COMPATIBLE STATUS ADD-ONS
         *
         * These tagged sections are appended after the existing fields.  Older
         * receivers can ignore unknown tags, while newer bench tools can decode
         * config, flash/log, actuator, TMC, and drag-predictor details.
         */
        const int32_t system_fields[SYSTEM_STATUS_COUNT] = {
            (int32_t)Extra->config_flags,
            (int32_t)Extra->flash_log_flags,
            (int32_t)Extra->command_action,
            (int32_t)Extra->command_ack,
            (int32_t)Extra->calibration_flags
        };
        const int32_t actuator_fields[ACTUATOR_STATUS_COUNT] = {
            Extra->actuator_state,
            Extra->actuator_target_steps,
            Extra->actuator_actual_steps,
            Extra->tmc_driver_status,
            Extra->tmc_diag_pins
        };
        const int32_t apogee_fields[APOGEE_DETAIL_COUNT] = {
            Extra->ballistic_apogee_cm,
            Extra->drag_apogee_cm,
            Extra->drag_area_u_m2,
            Extra->actuator_effectiveness_milli
        };

        index = Payload_AddS32Array(txBuffer,
                                    index,
                                    TAG_SYSTEM_STATUS,
                                    system_fields,
                                    SYSTEM_STATUS_COUNT);
        index = Payload_AddS32Array(txBuffer,
                                    index,
                                    TAG_ACTUATOR_STATUS,
                                    actuator_fields,
                                    ACTUATOR_STATUS_COUNT);
        index = Payload_AddS32Array(txBuffer,
                                    index,
                                    TAG_APOGEE_DETAIL,
                                    apogee_fields,
                                    APOGEE_DETAIL_COUNT);
        /* END AMBAR BENCH-GATED EXPANSION - COMPATIBLE STATUS ADD-ONS */
    }

    /*
     * Optional messages.
     * Message[0] = status
     * Message[1] = command response
     * Message[2] = error
     */
    if (Message != 0)
    {
        index = Payload_AddString(txBuffer, index, TAG_STATUS_MSG,  Message[0]);
        index = Payload_AddString(txBuffer, index, TAG_COMMAND_MSG, Message[1]);
        index = Payload_AddString(txBuffer, index, TAG_ERROR_MSG,   Message[2]);
    }

    if (index > SX1280_MAX_PAYLOAD_LEN)
    {
        return HAL_ERROR;
    }

    /* END AMBAR EKF PCB INTEGRATION - FLIGHT PAYLOAD ENTRY POINT */
    return SX1280_Transmit(txBuffer, index, 1000);
}

HAL_StatusTypeDef PayloadPipeline(uint16_t *IMU,
                                  uint32_t *Baro,
                                  uint16_t *Magnet,
                                  uint16_t *Calc,
                                  const char *Message[3],
                                  uint8_t deployment_state)
{
    return PayloadPipelineWithFlight(IMU,
                                     Baro,
                                     Magnet,
                                     Calc,
                                     Message,
                                     deployment_state,
                                     0,
                                     0U,
                                     0);
}

/*
 * If you already have HAL_GPIO_EXTI_Callback() somewhere else,
 * merge this logic into the existing callback instead of defining it twice.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /*
     * Cube/HAL calls this weak callback from the EXTI interrupt path.  We only
     * latch a flag here; the actual SPI read happens later in RadioBridge_Task()
     * so the interrupt callback stays very short.
     */
    if (GPIO_Pin == LORA_DIO1_Pin)
    {
        radio_dio1_seen = 1;
    }

    RocketSensors_HandleExtiPin(GPIO_Pin);
    AmbarApp_HandleExtiPin(GPIO_Pin);
}

HAL_StatusTypeDef RadioBridge_Init(void)
{
    HAL_StatusTypeDef status;

    status = SX1280_InitLoRa();
    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Send a boot message so the LILYGO can confirm STM32 -> LILYGO works.
     */
    const char boot_msg[] = "STM32_BOOT";
    SX1280_Transmit((const uint8_t *)boot_msg, (uint8_t)strlen(boot_msg), 1000);

    last_heartbeat_ms = HAL_GetTick();

    return HAL_OK;
}

HAL_StatusTypeDef RadioBridge_SendText(const char *text)
{
    size_t len;

    if (text == 0)
    {
        return HAL_ERROR;
    }

    len = strlen(text);

    if ((len == 0) || (len > SX1280_MAX_PAYLOAD_LEN))
    {
        return HAL_ERROR;
    }

    return SX1280_Transmit((const uint8_t *)text, (uint8_t)len, 1000);
}

bool RadioBridge_TakeCommand(AmbarCommandResult_t *result)
{
    if (result == 0 || command_pending == 0U)
    {
        return false;
    }

    *result = pending_command;
    command_pending = 0U;
    return true;
}

const AmbarCommandResult_t *RadioBridge_GetLastCommand(void)
{
    return &last_command;
}

void RadioBridge_Task(void)
{
    /*
     * This task is intentionally nonblocking.  It handles incoming packets if
     * present and emits an occasional heartbeat, then returns to the EKF scheduler.
     */
    uint8_t payload[SX1280_MAX_PAYLOAD_LEN + 1];
    uint8_t payload_len = 0;

    /*
     * This handles both cases:
     * 1. DIO1 interrupt was generated.
     * 2. NVIC/EXTI is not working yet, but DIO1 is physically high.
     */
    if ((radio_dio1_seen != 0) ||
        (HAL_GPIO_ReadPin(LORA_DIO1_GPIO_Port, LORA_DIO1_Pin) == GPIO_PIN_SET))
    {
        radio_dio1_seen = 0;

        if (SX1280_ReadPacketIfAvailable(payload, &payload_len) == HAL_OK)
        {
            payload[payload_len] = '\0';

            /*
             * BEGIN AMBAR BENCH-GATED EXPANSION - COMMAND PARSE
             *
             * The bridge parses the packet and stores the result for the app.
             * The app then performs the safety-checked action and sends a short
             * response.  Parser errors are still reported immediately.
             */
            AmbarCommandResult_t parsed;
            if (AmbarCommand_ProcessPacket(payload, payload_len, &parsed) == HAL_OK)
            {
                pending_command = parsed;
                last_command = parsed;
                command_pending = 1U;
            }
            else
            {
                last_command = parsed;
                (void)RadioBridge_SendText(parsed.response);
            }
            /* END AMBAR BENCH-GATED EXPANSION - COMMAND PARSE */
        }
    }

    /*
     * Optional heartbeat proves STM32 -> LILYGO transmit works even before
     * the LILYGO sends anything.
     */
    if ((HAL_GetTick() - last_heartbeat_ms) >= 5000)
    {
        last_heartbeat_ms = HAL_GetTick();

        char msg[48];
        int n = snprintf(msg, sizeof(msg), "STM32_HEARTBEAT_%lu", heartbeat_counter++);

        if (n > 0)
        {
            SX1280_Transmit((const uint8_t *)msg, (uint8_t)n, 1000);
        }
    }
}
