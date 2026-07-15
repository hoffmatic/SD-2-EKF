/*
 * AMBAR USB PROTOCOL V2 - CODEC IMPLEMENTATION
 *
 * Purpose
 *   Implements the transport-independent wire codec shared by STM32 USB CDC
 *   and host tools: CRC-16/CCITT-FALSE, COBS stream framing, fixed header
 *   validation, explicit little-endian payload serializers, and stable text
 *   labels for diagnostics.  This module performs no I/O and executes no command.
 *
 * Data flow
 *   A producer serializes a typed payload, then EncodeFrame prepends the header,
 *   appends CRC, COBS-encodes, and terminates with zero.  The USB transport
 *   removes the delimiter and calls DecodeFrame; only validated header/payload
 *   bytes are returned to AmbarApp.  See CODE_GUIDE.md [ARCH-3] and [ARCH-6].
 *
 * Section map
 *   1. CRC and COBS byte transforms
 *   2. Complete frame encode/decode
 *   3. Typed payload serializers/deserializers
 *   4. Operator-facing code-to-text helpers
 *
 * Safety and assumptions
 *   Wire values are little-endian and structures are never copied directly to
 *   the link.  All encoders require caller-provided bounded storage; zero means
 *   an invalid argument or insufficient capacity.  Decoding validates bounds,
 *   magic, version, and CRC, but command authorization and physical-motion gates
 *   intentionally live above this codec.  The SX1280 radio/TLV format remains a
 *   separate version-1 protocol.
 */

#include "rocket_protocol.h"

#include <string.h>

/* ===================== CRC AND COBS BYTE TRANSFORMS ===================== */

uint16_t RocketProtocol_Crc16(const uint8_t *data, size_t length)
{
    /* CCITT-FALSE parameters match the host implementation and 0x29B1 check vector. */
    uint16_t crc = 0xFFFFu;

    if (data == NULL && length != 0u)
    {
        return 0u;
    }

    for (size_t i = 0u; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0u; bit < 8u; ++bit)
        {
            crc = (crc & 0x8000u) != 0u
                ? (uint16_t)((crc << 1) ^ 0x1021u)
                : (uint16_t)(crc << 1);
        }
    }

    return crc;
}

size_t RocketProtocol_CobsEncode(const uint8_t *input,
                                size_t input_length,
                                uint8_t *output,
                                size_t output_capacity)
{
    /* Remove embedded zeroes so one zero byte can delimit frames on CDC. */
    size_t read_index = 0u;
    size_t write_index = 1u;
    size_t code_index = 0u;
    uint8_t code = 1u;

    if ((input == NULL && input_length != 0u) || output == NULL || output_capacity == 0u)
    {
        return 0u;
    }

    while (read_index < input_length)
    {
        if (input[read_index] == 0u)
        {
            if (code_index >= output_capacity)
            {
                return 0u;
            }
            output[code_index] = code;
            code = 1u;
            code_index = write_index;
            ++write_index;
            ++read_index;
        }
        else
        {
            if (write_index >= output_capacity)
            {
                return 0u;
            }
            output[write_index++] = input[read_index++];
            ++code;

            if (code == 0xFFu)
            {
                if (code_index >= output_capacity)
                {
                    return 0u;
                }
                output[code_index] = code;
                code = 1u;
                code_index = write_index;
                ++write_index;
            }
        }
    }

    if (code_index >= output_capacity)
    {
        return 0u;
    }
    output[code_index] = code;
    return write_index;
}

size_t RocketProtocol_CobsDecode(const uint8_t *input,
                                size_t input_length,
                                uint8_t *output,
                                size_t output_capacity)
{
    /* Reject malformed code runs before copying beyond either input or output. */
    size_t read_index = 0u;
    size_t write_index = 0u;

    if (input == NULL || input_length == 0u || output == NULL)
    {
        return 0u;
    }

    while (read_index < input_length)
    {
        const uint8_t code = input[read_index++];
        if (code == 0u)
        {
            return 0u;
        }

        const size_t copy_length = (size_t)code - 1u;
        if (copy_length > input_length - read_index
            || copy_length > output_capacity - write_index)
        {
            return 0u;
        }

        if (copy_length != 0u)
        {
            memcpy(output + write_index, input + read_index, copy_length);
            read_index += copy_length;
            write_index += copy_length;
        }

        if (code != 0xFFu && read_index < input_length)
        {
            if (write_index >= output_capacity)
            {
                return 0u;
            }
            output[write_index++] = 0u;
        }
    }

    return write_index;
}

/* ===================== COMPLETE FRAME CODEC ===================== */

size_t RocketProtocol_EncodeFrame(uint8_t *frame,
                                 size_t frame_capacity,
                                 uint8_t type,
                                 uint16_t sequence,
                                 uint32_t time_ms,
                                 const uint8_t *payload,
                                 size_t payload_length)
{
    /* Build in bounded raw storage, then append the stream delimiter after COBS. */
    uint8_t raw[ROCKET_PROTOCOL_MAX_PACKET];

    if (frame == NULL
        || frame_capacity < 2u
        || payload_length > ROCKET_PROTOCOL_MAX_PAYLOAD
        || (payload == NULL && payload_length != 0u))
    {
        return 0u;
    }

    raw[0] = ROCKET_PROTOCOL_MAGIC;
    raw[1] = ROCKET_PROTOCOL_VERSION;
    raw[2] = type;
    RocketProtocol_WriteU16(raw + 3, sequence);
    RocketProtocol_WriteU32(raw + 5, time_ms);
    if (payload_length != 0u)
    {
        memcpy(raw + ROCKET_PROTOCOL_HEADER_SIZE, payload, payload_length);
    }

    const size_t covered_length = ROCKET_PROTOCOL_HEADER_SIZE + payload_length;
    const uint16_t crc = RocketProtocol_Crc16(raw, covered_length);
    RocketProtocol_WriteU16(raw + covered_length, crc);

    const size_t encoded_length = RocketProtocol_CobsEncode(
        raw,
        covered_length + ROCKET_PROTOCOL_CRC_SIZE,
        frame,
        frame_capacity - 1u);
    if (encoded_length == 0u || encoded_length >= frame_capacity)
    {
        return 0u;
    }

    frame[encoded_length] = ROCKET_PROTOCOL_FRAME_DELIMITER;
    return encoded_length + 1u;
}

RocketProtocolResult RocketProtocol_DecodeFrame(const uint8_t *encoded_frame,
                                                size_t encoded_length,
                                                RocketDecodedPacket *packet)
{
    /* Input excludes the trailing delimiter; output is written only after validation. */
    uint8_t raw[ROCKET_PROTOCOL_MAX_PACKET];

    if (encoded_frame == NULL || packet == NULL)
    {
        return ROCKET_PROTOCOL_BAD_ARGUMENT;
    }
    if (encoded_length == 0u || encoded_length >= ROCKET_PROTOCOL_MAX_FRAME)
    {
        return ROCKET_PROTOCOL_BAD_LENGTH;
    }

    const size_t raw_length = RocketProtocol_CobsDecode(
        encoded_frame,
        encoded_length,
        raw,
        sizeof(raw));
    if (raw_length == 0u)
    {
        return ROCKET_PROTOCOL_BAD_COBS;
    }
    if (raw_length < ROCKET_PROTOCOL_HEADER_SIZE + ROCKET_PROTOCOL_CRC_SIZE
        || raw_length > ROCKET_PROTOCOL_MAX_PACKET)
    {
        return ROCKET_PROTOCOL_BAD_LENGTH;
    }
    if (raw[0] != ROCKET_PROTOCOL_MAGIC)
    {
        return ROCKET_PROTOCOL_BAD_MAGIC;
    }
    if (raw[1] != ROCKET_PROTOCOL_VERSION)
    {
        return ROCKET_PROTOCOL_BAD_VERSION;
    }

    const size_t covered_length = raw_length - ROCKET_PROTOCOL_CRC_SIZE;
    const uint16_t received_crc = RocketProtocol_ReadU16(raw + covered_length);
    if (RocketProtocol_Crc16(raw, covered_length) != received_crc)
    {
        return ROCKET_PROTOCOL_BAD_CRC;
    }

    const size_t payload_length = covered_length - ROCKET_PROTOCOL_HEADER_SIZE;
    packet->header.type = raw[2];
    packet->header.sequence = RocketProtocol_ReadU16(raw + 3);
    packet->header.time_ms = RocketProtocol_ReadU32(raw + 5);
    packet->payload_length = (uint8_t)payload_length;
    if (payload_length != 0u)
    {
        memcpy(packet->payload, raw + ROCKET_PROTOCOL_HEADER_SIZE, payload_length);
    }

    return ROCKET_PROTOCOL_OK;
}

/* ===================== TYPED PAYLOAD CODECS ===================== */

size_t RocketProtocol_EncodeTelemetryPayload(uint8_t *out,
                                             size_t capacity,
                                             const RocketTelemetryPayload *p)
{
    /* Serialize field by field so compiler padding never becomes wire format. */
    if (out == NULL || p == NULL || capacity < ROCKET_TELEMETRY_PAYLOAD_SIZE)
    {
        return 0u;
    }

    RocketProtocol_WriteU16(out + 0, p->flags);
    out[2] = p->state;
    out[3] = p->status_code;
    RocketProtocol_WriteI16(out + 4, p->altitude_dm);
    RocketProtocol_WriteI16(out + 6, p->velocity_cms);
    RocketProtocol_WriteI16(out + 8, p->acceleration_cms2);
    RocketProtocol_WriteU16(out + 10, p->predicted_apogee_dm);
    RocketProtocol_WriteU16(out + 12, p->target_apogee_dm);
    RocketProtocol_WriteI16(out + 14, p->roll_ddeg);
    RocketProtocol_WriteI16(out + 16, p->pitch_ddeg);
    RocketProtocol_WriteI16(out + 18, p->yaw_ddeg);
    out[20] = p->deployment_percent;
    out[21] = p->sensor_health;
    RocketProtocol_WriteU16(out + 22, p->failed_reads);
    out[24] = p->message_code;
    out[25] = p->reserved;
    return ROCKET_TELEMETRY_PAYLOAD_SIZE;
}

size_t RocketProtocol_EncodeEventPayload(uint8_t *out,
                                         size_t capacity,
                                         const RocketEventPayload *p)
{
    if (out == NULL || p == NULL || capacity < ROCKET_EVENT_PAYLOAD_SIZE)
    {
        return 0u;
    }
    RocketProtocol_WriteU16(out + 0, p->changed_flags);
    RocketProtocol_WriteU16(out + 2, p->current_flags);
    out[4] = p->previous_state;
    out[5] = p->current_state;
    out[6] = p->status_code;
    out[7] = p->message_code;
    RocketProtocol_WriteU16(out + 8, p->detail);
    return ROCKET_EVENT_PAYLOAD_SIZE;
}

size_t RocketProtocol_EncodeCommandPayload(uint8_t *out,
                                           size_t capacity,
                                           const RocketCommandPayload *p)
{
    /* The command prefix carries an explicit bounded data length. */
    if (out == NULL || p == NULL
        || p->payload_length > ROCKET_COMMAND_DATA_MAX
        || capacity < ROCKET_COMMAND_PREFIX_SIZE + p->payload_length)
    {
        return 0u;
    }
    out[0] = p->command;
    out[1] = p->payload_length;
    if (p->payload_length != 0u)
    {
        memcpy(out + ROCKET_COMMAND_PREFIX_SIZE, p->payload, p->payload_length);
    }
    return ROCKET_COMMAND_PREFIX_SIZE + p->payload_length;
}

int RocketProtocol_DecodeCommandPayload(const uint8_t *data,
                                        size_t length,
                                        RocketCommandPayload *p)
{
    /* Exact-length matching rejects both truncated and trailing command bytes. */
    if (data == NULL || p == NULL || length < ROCKET_COMMAND_PREFIX_SIZE)
    {
        return 0;
    }
    const uint8_t payload_length = data[1];
    if (payload_length > ROCKET_COMMAND_DATA_MAX
        || length != ROCKET_COMMAND_PREFIX_SIZE + payload_length)
    {
        return 0;
    }
    p->command = data[0];
    p->payload_length = payload_length;
    if (payload_length != 0u)
    {
        memcpy(p->payload, data + ROCKET_COMMAND_PREFIX_SIZE, payload_length);
    }
    return 1;
}

size_t RocketProtocol_EncodeAckPayload(uint8_t *out,
                                       size_t capacity,
                                       const RocketAckPayload *p)
{
    if (out == NULL || p == NULL || capacity < ROCKET_ACK_PAYLOAD_SIZE)
    {
        return 0u;
    }
    RocketProtocol_WriteU16(out + 0, p->command_sequence);
    out[2] = p->command;
    out[3] = p->result;
    RocketProtocol_WriteU16(out + 4, p->detail);
    return ROCKET_ACK_PAYLOAD_SIZE;
}

size_t RocketProtocol_EncodeSimulationPayload(uint8_t *out,
                                              size_t capacity,
                                              const RocketSimulationPayload *p)
{
    /* Millimetre-based integers make replay deterministic across host/MCU floats. */
    if (out == NULL || p == NULL || capacity < ROCKET_SIMULATION_PAYLOAD_SIZE)
    {
        return 0u;
    }
    RocketProtocol_WriteU16(out + 0, p->flags);
    RocketProtocol_WriteI32(out + 2, p->altitude_mm);
    RocketProtocol_WriteI32(out + 6, p->acceleration_mmps2);
    RocketProtocol_WriteI32(out + 10, p->velocity_mmps);
    RocketProtocol_WriteU16(out + 14, p->barometer_stddev_cm);
    return ROCKET_SIMULATION_PAYLOAD_SIZE;
}

int RocketProtocol_DecodeSimulationPayload(const uint8_t *data,
                                           size_t length,
                                           RocketSimulationPayload *p)
{
    /* Simulation payloads are fixed-size; presence/meaning is carried by flags. */
    if (data == NULL || p == NULL || length != ROCKET_SIMULATION_PAYLOAD_SIZE)
    {
        return 0;
    }
    p->flags = RocketProtocol_ReadU16(data + 0);
    p->altitude_mm = RocketProtocol_ReadI32(data + 2);
    p->acceleration_mmps2 = RocketProtocol_ReadI32(data + 6);
    p->velocity_mmps = RocketProtocol_ReadI32(data + 10);
    p->barometer_stddev_cm = RocketProtocol_ReadU16(data + 14);
    return 1;
}

size_t RocketProtocol_EncodeActuatorStatusPayload(
    uint8_t *out,
    size_t capacity,
    const RocketActuatorStatusPayload *p)
{
    if (out == NULL || p == NULL || capacity < ROCKET_ACTUATOR_STATUS_PAYLOAD_SIZE)
    {
        return 0u;
    }
    RocketProtocol_WriteU32(out + 0, p->actuator_inhibit_flags);
    RocketProtocol_WriteU32(out + 4, p->flight_inhibit_flags);
    RocketProtocol_WriteI32(out + 8, p->target_steps);
    RocketProtocol_WriteI32(out + 12, p->actual_steps);
    RocketProtocol_WriteU32(out + 16, p->driver_status);
    out[20] = p->machine_state;
    out[21] = p->flags;
    RocketProtocol_WriteU16(out + 22, p->reserved);
    return ROCKET_ACTUATOR_STATUS_PAYLOAD_SIZE;
}

size_t RocketProtocol_EncodeHeartbeatPayload(uint8_t *out,
                                             size_t capacity,
                                             const RocketHeartbeatPayload *p)
{
    if (out == NULL || p == NULL || capacity < ROCKET_HEARTBEAT_PAYLOAD_SIZE)
    {
        return 0u;
    }
    RocketProtocol_WriteU32(out + 0, p->feature_flags);
    RocketProtocol_WriteU16(out + 4, p->receive_errors);
    RocketProtocol_WriteU16(out + 6, p->transmit_drops);
    return ROCKET_HEARTBEAT_PAYLOAD_SIZE;
}

/* ===================== OPERATOR-FACING CODE LABELS ===================== */

const char *RocketProtocol_StatusText(uint8_t code)
{
    /* Stable ASCII labels keep logs readable without changing numeric wire values. */
    switch (code)
    {
    case ROCKET_STATUS_OK: return "OK";
    case ROCKET_STATUS_SENSOR_INIT_FAILED: return "SENSOR_INIT_FAILED";
    case ROCKET_STATUS_INVALID_CONFIG: return "INVALID_CONFIG";
    case ROCKET_STATUS_BAD_ARGUMENT: return "BAD_ARGUMENT";
    case ROCKET_STATUS_UNSUPPORTED_MODE: return "UNSUPPORTED_MODE";
    case ROCKET_STATUS_SENSOR_READ_FAILED: return "SENSOR_READ_FAILED";
    case ROCKET_STATUS_BARO_STALE: return "BARO_STALE";
    case ROCKET_STATUS_ESTIMATOR_INVALID: return "ESTIMATOR_INVALID";
    case ROCKET_STATUS_CONTROLLER_INVALID: return "CONTROLLER_INVALID";
    case ROCKET_STATUS_RADIO_ERROR: return "RADIO_ERROR";
    case ROCKET_STATUS_USB_ERROR: return "USB_ERROR";
    case ROCKET_STATUS_SIMULATION_STALE: return "SIMULATION_STALE";
    default: return "UNKNOWN_STATUS";
    }
}

const char *RocketProtocol_MessageText(uint8_t code)
{
    switch (code)
    {
    case ROCKET_MSG_NONE: return "NONE";
    case ROCKET_MSG_BOOT: return "BOOT";
    case ROCKET_MSG_RADIO_READY: return "RADIO_READY";
    case ROCKET_MSG_CONFIG_APPLIED: return "CONFIG_APPLIED";
    case ROCKET_MSG_MODE_CHANGED: return "MODE_CHANGED";
    case ROCKET_MSG_LAUNCH_DETECTED: return "LAUNCH_DETECTED";
    case ROCKET_MSG_BURNOUT_DETECTED: return "BURNOUT_DETECTED";
    case ROCKET_MSG_APOGEE_REACHED: return "APOGEE_REACHED";
    case ROCKET_MSG_LANDING_DETECTED: return "LANDING_DETECTED";
    case ROCKET_MSG_AIRBRAKE_DEPLOYED: return "AIRBRAKE_DEPLOYED";
    case ROCKET_MSG_AIRBRAKE_RETRACTED: return "AIRBRAKE_RETRACTED";
    case ROCKET_MSG_CONTROLLER_ENABLED: return "CONTROLLER_ENABLED";
    case ROCKET_MSG_CONTROLLER_DISABLED: return "CONTROLLER_DISABLED";
    case ROCKET_MSG_SNAPSHOT: return "SNAPSHOT";
    case ROCKET_MSG_HEARTBEAT: return "HEARTBEAT";
    case ROCKET_MSG_SIMULATION_STARTED: return "SIMULATION_STARTED";
    case ROCKET_MSG_SIMULATION_STOPPED: return "SIMULATION_STOPPED";
    case ROCKET_MSG_SIMULATION_STALE: return "SIMULATION_STALE";
    default: return "UNKNOWN_MESSAGE";
    }
}

const char *RocketProtocol_CommandText(uint8_t code)
{
    switch (code)
    {
    case ROCKET_CMD_NOP: return "NOP";
    case ROCKET_CMD_PING: return "PING";
    case ROCKET_CMD_REQUEST_SNAPSHOT: return "REQUEST_SNAPSHOT";
    case ROCKET_CMD_SET_TARGET_APOGEE: return "SET_TARGET_APOGEE";
    case ROCKET_CMD_SET_CONTROLLER: return "SET_ARMED";
    case ROCKET_CMD_SET_MODE: return "SET_MODE";
    case ROCKET_CMD_RETURN_STANDARD: return "RETURN_STANDARD";
    case ROCKET_CMD_MANUAL_AIRBRAKE: return "MANUAL_AIRBRAKE";
    case ROCKET_CMD_ESTOP: return "ESTOP";
    case ROCKET_CMD_HOME: return "HOME";
    case ROCKET_CMD_RETRACT: return "RETRACT";
    case ROCKET_CMD_PAD_RESET: return "PAD_RESET";
    case ROCKET_CMD_SAVE_CONFIG: return "SAVE_CONFIG";
    case ROCKET_CMD_BENCH_MOVE_STEPS: return "BENCH_MOVE_STEPS";
    case ROCKET_CMD_SIM_START: return "SIM_START";
    case ROCKET_CMD_SIM_STOP: return "SIM_STOP";
    case ROCKET_CMD_START_LOG: return "START_LOG";
    case ROCKET_CMD_STOP_LOG: return "STOP_LOG";
    case ROCKET_CMD_ERASE_LOG: return "ERASE_LOG";
    default: return "UNKNOWN_COMMAND";
    }
}
