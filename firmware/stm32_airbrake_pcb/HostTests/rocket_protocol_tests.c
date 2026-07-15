/**
 * @file rocket_protocol_tests.c
 * @brief Cross-language regression vectors for the production C wire codec.
 *
 * The test compiles rocket_protocol.c on the host and checks CRC, COBS framing,
 * typed payloads, malformed-input rejection, and stable golden bytes.  These
 * cases protect the [ARCH-3]/[ARCH-6] contract shared by STM32 and Python; they
 * do not exercise USBX or a real CDC device.
 */
#include "rocket_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void test_crc_vector(void)
{
    static const uint8_t vector[] = "123456789";
    assert(RocketProtocol_Crc16(vector, sizeof(vector) - 1u) == 0x29B1u);
}

static void test_cobs_round_trip(void)
{
    const uint8_t source[] = {0u, 1u, 2u, 0u, 3u, 0u, 0u, 4u};
    uint8_t encoded[32];
    uint8_t decoded[32];
    const size_t encoded_length = RocketProtocol_CobsEncode(source,
                                                             sizeof(source),
                                                             encoded,
                                                             sizeof(encoded));
    assert(encoded_length != 0u);
    for (size_t i = 0u; i < encoded_length; ++i)
    {
        assert(encoded[i] != 0u);
    }
    const size_t decoded_length = RocketProtocol_CobsDecode(encoded,
                                                             encoded_length,
                                                             decoded,
                                                             sizeof(decoded));
    assert(decoded_length == sizeof(source));
    assert(memcmp(source, decoded, sizeof(source)) == 0);
}

static void test_telemetry_golden_payload(void)
{
    RocketTelemetryPayload payload;
    uint8_t bytes[ROCKET_TELEMETRY_PAYLOAD_SIZE];
    memset(&payload, 0, sizeof(payload));
    payload.flags = 0x1234u;
    payload.state = 0x05u;
    payload.status_code = 0x06u;
    payload.altitude_dm = -2;
    payload.velocity_cms = 0x1234;
    payload.acceleration_cms2 = -32768;
    payload.predicted_apogee_dm = 0xBEEFu;
    payload.target_apogee_dm = 0xCAFEu;
    payload.roll_ddeg = 1;
    payload.pitch_ddeg = -1;
    payload.yaw_ddeg = 0x2222;
    payload.deployment_percent = 99u;
    payload.sensor_health = 0xE4u;
    payload.failed_reads = 0xABCDu;
    payload.message_code = 0x0Du;
    payload.reserved = 0x55u;

    assert(RocketProtocol_EncodeTelemetryPayload(bytes, sizeof(bytes), &payload)
           == ROCKET_TELEMETRY_PAYLOAD_SIZE);
    const uint8_t expected[] = {
        0x34, 0x12, 0x05, 0x06, 0xFE, 0xFF, 0x34, 0x12, 0x00, 0x80,
        0xEF, 0xBE, 0xFE, 0xCA, 0x01, 0x00, 0xFF, 0xFF, 0x22, 0x22,
        0x63, 0xE4, 0xCD, 0xAB, 0x0D, 0x55
    };
    assert(sizeof(expected) == sizeof(bytes));
    assert(memcmp(bytes, expected, sizeof(bytes)) == 0);
}

static void test_frame_round_trip_and_crc_rejection(void)
{
    RocketCommandPayload command;
    uint8_t command_bytes[ROCKET_COMMAND_PREFIX_SIZE + ROCKET_COMMAND_DATA_MAX];
    uint8_t frame[ROCKET_PROTOCOL_MAX_FRAME];
    RocketDecodedPacket decoded;

    memset(&command, 0, sizeof(command));
    command.command = ROCKET_CMD_SET_TARGET_APOGEE;
    command.payload_length = 2u;
    RocketProtocol_WriteU16(command.payload, 3048u);
    const size_t command_length = RocketProtocol_EncodeCommandPayload(command_bytes,
                                                                       sizeof(command_bytes),
                                                                       &command);
    assert(command_length == 4u);

    const size_t frame_length = RocketProtocol_EncodeFrame(frame,
                                                            sizeof(frame),
                                                            ROCKET_PKT_COMMAND,
                                                            0x1234u,
                                                            0x89ABCDEFu,
                                                            command_bytes,
                                                            command_length);
    const uint8_t golden_frame[] = {
        0x10, 0xA5, 0x02, 0x10, 0x34, 0x12, 0xEF, 0xCD,
        0xAB, 0x89, 0x10, 0x02, 0xE8, 0x0B, 0x9F, 0xF6, 0x00
    };
    assert(frame_length > 1u && frame_length <= ROCKET_PROTOCOL_MAX_FRAME);
    assert(frame_length == sizeof(golden_frame));
    assert(memcmp(frame, golden_frame, sizeof(golden_frame)) == 0);
    assert(frame[frame_length - 1u] == ROCKET_PROTOCOL_FRAME_DELIMITER);
    assert(RocketProtocol_DecodeFrame(frame, frame_length - 1u, &decoded)
           == ROCKET_PROTOCOL_OK);
    assert(decoded.header.type == ROCKET_PKT_COMMAND);
    assert(decoded.header.sequence == 0x1234u);
    assert(decoded.header.time_ms == 0x89ABCDEFu);
    assert(decoded.payload_length == command_length);
    assert(memcmp(decoded.payload, command_bytes, command_length) == 0);

    uint8_t raw[ROCKET_PROTOCOL_MAX_PACKET];
    uint8_t damaged[ROCKET_PROTOCOL_MAX_FRAME];
    const size_t raw_length = RocketProtocol_CobsDecode(frame,
                                                         frame_length - 1u,
                                                         raw,
                                                         sizeof(raw));
    assert(raw_length > ROCKET_PROTOCOL_HEADER_SIZE + ROCKET_PROTOCOL_CRC_SIZE);
    raw[ROCKET_PROTOCOL_HEADER_SIZE] ^= 0x01u;
    const size_t damaged_length = RocketProtocol_CobsEncode(raw,
                                                             raw_length,
                                                             damaged,
                                                             sizeof(damaged));
    assert(damaged_length != 0u);
    assert(RocketProtocol_DecodeFrame(damaged, damaged_length, &decoded)
           == ROCKET_PROTOCOL_BAD_CRC);
}

static void test_maximum_frame(void)
{
    uint8_t payload[ROCKET_PROTOCOL_MAX_PAYLOAD];
    uint8_t frame[ROCKET_PROTOCOL_MAX_FRAME];
    RocketDecodedPacket decoded;

    for (size_t i = 0u; i < sizeof(payload); ++i)
    {
        payload[i] = (uint8_t)(i + 1u);
    }
    const size_t length = RocketProtocol_EncodeFrame(frame,
                                                      sizeof(frame),
                                                      ROCKET_PKT_EVENT,
                                                      0xFFFFu,
                                                      0xFFFFFFFFu,
                                                      payload,
                                                      sizeof(payload));
    assert(length != 0u && length <= sizeof(frame));
    assert(RocketProtocol_DecodeFrame(frame, length - 1u, &decoded)
           == ROCKET_PROTOCOL_OK);
    assert(decoded.payload_length == sizeof(payload));
    assert(memcmp(decoded.payload, payload, sizeof(payload)) == 0);
}

static void test_simulation_payload(void)
{
    RocketSimulationPayload source = {
        .flags = ROCKET_SIM_FLAG_ALTITUDE_VALID
            | ROCKET_SIM_FLAG_ACCELERATION_VALID
            | ROCKET_SIM_FLAG_VELOCITY_VALID,
        .altitude_mm = 123456,
        .acceleration_mmps2 = -9810,
        .velocity_mmps = 42000,
        .barometer_stddev_cm = 150u
    };
    RocketSimulationPayload decoded;
    uint8_t bytes[ROCKET_SIMULATION_PAYLOAD_SIZE];
    assert(RocketProtocol_EncodeSimulationPayload(bytes, sizeof(bytes), &source)
           == sizeof(bytes));
    assert(RocketProtocol_DecodeSimulationPayload(bytes, sizeof(bytes), &decoded));
    assert(decoded.flags == source.flags);
    assert(decoded.altitude_mm == source.altitude_mm);
    assert(decoded.acceleration_mmps2 == source.acceleration_mmps2);
    assert(decoded.velocity_mmps == source.velocity_mmps);
    assert(decoded.barometer_stddev_cm == source.barometer_stddev_cm);
}

int main(void)
{
    test_crc_vector();
    test_cobs_round_trip();
    test_telemetry_golden_payload();
    test_frame_round_trip_and_crc_rejection();
    test_maximum_frame();
    test_simulation_payload();
    puts("rocket_protocol_tests: PASS");
    return 0;
}
