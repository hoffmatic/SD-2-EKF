/*
 * AMBAR USB PROTOCOL V2 - SHARED WIRE CONTRACT
 *
 * Purpose and ownership
 *   Defines the binary protocol shared by STM32 direct USB, Python replay tools,
 *   and GUI implementations.  This is a wire schema, not an in-memory ABI:
 *   every multi-byte value is little-endian and every payload is serialized
 *   field by field by rocket_protocol.c.
 *
 * Frame flow
 *   One USB CDC frame is:
 *
 *     COBS(header || payload || crc16_le) || 0x00
 *
 *   The nine-byte header is magic, version, packet type, sequence, and device or
 *   host time_ms.  CRC-16/CCITT-FALSE covers header plus payload; the check
 *   vector "123456789" is 0x29B1.  See CODE_GUIDE.md [ARCH-3] and [ARCH-6].
 *
 * Section map
 *   1. Frame limits and fixed payload sizes
 *   2. Packet, command, ACK, status, and message codes
 *   3. Telemetry, sensor, simulation, and actuator flags
 *   4. Decoded headers and typed payload models
 *   5. Little-endian helpers and sensor-health packing
 *   6. Frame/payload codec and diagnostic-text API
 *
 * Safety and compatibility
 *   Decoding proves framing and integrity, not command authorization.  AmbarApp
 *   and AmbarActuator must still validate mode, arming, HOME, health, and motion
 *   gates.  Keep numeric values, field offsets, scales, and payload sizes stable
 *   within protocol version 2; incompatible changes require a version change and
 *   synchronized host updates.  The SX1280/Arduino TLV protocol remains version
 *   1 and is intentionally separate.
 */

#ifndef ROCKET_PROTOCOL_H
#define ROCKET_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== FRAME LIMITS AND PAYLOAD SIZES ===================== */

/* Raw packets are bounded to one full-speed USB endpoint-sized transfer. */
#define ROCKET_PROTOCOL_MAGIC              0xA5u
#define ROCKET_PROTOCOL_VERSION            0x02u
#define ROCKET_PROTOCOL_HEADER_SIZE        9u
#define ROCKET_PROTOCOL_CRC_SIZE           2u
#define ROCKET_PROTOCOL_MAX_PACKET         64u
#define ROCKET_PROTOCOL_MAX_PAYLOAD        \
    (ROCKET_PROTOCOL_MAX_PACKET - ROCKET_PROTOCOL_HEADER_SIZE - ROCKET_PROTOCOL_CRC_SIZE)
#define ROCKET_PROTOCOL_MAX_FRAME          66u
#define ROCKET_PROTOCOL_FRAME_DELIMITER    0x00u

/* Fixed schema sizes are checked before any payload field is read or written. */
#define ROCKET_TELEMETRY_PAYLOAD_SIZE      26u
#define ROCKET_EVENT_PAYLOAD_SIZE          10u
#define ROCKET_COMMAND_PREFIX_SIZE         2u
#define ROCKET_COMMAND_DATA_MAX            8u
#define ROCKET_ACK_PAYLOAD_SIZE            6u
#define ROCKET_SIMULATION_PAYLOAD_SIZE     16u
#define ROCKET_ACTUATOR_STATUS_PAYLOAD_SIZE 24u
#define ROCKET_HEARTBEAT_PAYLOAD_SIZE       8u

/* ===================== PACKET AND CONTROL VOCABULARY ===================== */

typedef enum
{
    ROCKET_PKT_TELEMETRY      = 0x01,
    ROCKET_PKT_EVENT          = 0x02,
    ROCKET_PKT_ACTUATOR_STATUS = 0x03,
    ROCKET_PKT_SIMULATION     = 0x20,
    ROCKET_PKT_COMMAND        = 0x10,
    ROCKET_PKT_ACK            = 0x11,
    ROCKET_PKT_HEARTBEAT      = 0x12
} RocketPacketType;

typedef enum
{
    ROCKET_CMD_NOP               = 0x00,
    ROCKET_CMD_PING              = 0x01,
    ROCKET_CMD_REQUEST_SNAPSHOT  = 0x02,
    ROCKET_CMD_SET_TARGET_APOGEE = 0x10, /* uint16: decimetres */
    ROCKET_CMD_SET_CONTROLLER    = 0x11, /* uint8: 0=disarm, 1=arm */
    ROCKET_CMD_SET_ARMED         = 0x11, /* clearer alias for new GUI code */
    ROCKET_CMD_SET_MODE          = 0x12, /* reserved legacy behavior command */
    ROCKET_CMD_RETURN_STANDARD   = 0x13, /* reserved legacy behavior command */
    ROCKET_CMD_MANUAL_AIRBRAKE   = 0x14, /* reserved until direction-safe */
    ROCKET_CMD_ESTOP             = 0x15,
    ROCKET_CMD_HOME              = 0x16,
    ROCKET_CMD_RETRACT           = 0x17,
    ROCKET_CMD_PAD_RESET         = 0x18,
    ROCKET_CMD_SAVE_CONFIG       = 0x19,
    ROCKET_CMD_BENCH_MOVE_STEPS  = 0x1A, /* int32: absolute motor steps */
    ROCKET_CMD_SIM_START         = 0x20,
    ROCKET_CMD_SIM_STOP          = 0x21,
    ROCKET_CMD_START_LOG         = 0x30,
    ROCKET_CMD_STOP_LOG          = 0x31,
    ROCKET_CMD_ERASE_LOG         = 0x32
} RocketCommandCode;

typedef enum
{
    ROCKET_ACK_OK              = 0x00,
    ROCKET_ACK_BAD_LENGTH      = 0x01,
    ROCKET_ACK_BAD_VALUE       = 0x02,
    ROCKET_ACK_UNSUPPORTED     = 0x03,
    ROCKET_ACK_BUSY            = 0x04,
    ROCKET_ACK_EXECUTION_ERROR = 0x05,
    ROCKET_ACK_BAD_CRC         = 0x06
} RocketAckCode;

typedef enum
{
    ROCKET_STATUS_OK                 = 0x00,
    ROCKET_STATUS_SENSOR_INIT_FAILED = 0x01,
    ROCKET_STATUS_INVALID_CONFIG     = 0x02,
    ROCKET_STATUS_BAD_ARGUMENT       = 0x03,
    ROCKET_STATUS_UNSUPPORTED_MODE   = 0x04,
    ROCKET_STATUS_SENSOR_READ_FAILED = 0x05,
    ROCKET_STATUS_BARO_STALE         = 0x06,
    ROCKET_STATUS_ESTIMATOR_INVALID  = 0x07,
    ROCKET_STATUS_CONTROLLER_INVALID = 0x08,
    ROCKET_STATUS_RADIO_ERROR        = 0x09,
    ROCKET_STATUS_USB_ERROR          = 0x0A,
    ROCKET_STATUS_SIMULATION_STALE   = 0x0B,
    ROCKET_STATUS_UNKNOWN            = 0xFF
} RocketStatusCode;

typedef enum
{
    ROCKET_MSG_NONE                = 0x00,
    ROCKET_MSG_BOOT                = 0x01,
    ROCKET_MSG_RADIO_READY         = 0x02,
    ROCKET_MSG_CONFIG_APPLIED      = 0x03,
    ROCKET_MSG_MODE_CHANGED        = 0x04,
    ROCKET_MSG_LAUNCH_DETECTED     = 0x05,
    ROCKET_MSG_BURNOUT_DETECTED    = 0x06,
    ROCKET_MSG_APOGEE_REACHED      = 0x07,
    ROCKET_MSG_LANDING_DETECTED    = 0x08,
    ROCKET_MSG_AIRBRAKE_DEPLOYED   = 0x09,
    ROCKET_MSG_AIRBRAKE_RETRACTED  = 0x0A,
    ROCKET_MSG_CONTROLLER_ENABLED  = 0x0B,
    ROCKET_MSG_CONTROLLER_DISABLED = 0x0C,
    ROCKET_MSG_SNAPSHOT            = 0x0D,
    ROCKET_MSG_HEARTBEAT           = 0x0E,
    ROCKET_MSG_SIMULATION_STARTED  = 0x0F,
    ROCKET_MSG_SIMULATION_STOPPED  = 0x10,
    ROCKET_MSG_SIMULATION_STALE    = 0x11
} RocketMessageCode;

/* ===================== TELEMETRY AND INPUT FLAGS ===================== */

enum
{
    ROCKET_FLAG_INITIALIZED          = 1u << 0,
    ROCKET_FLAG_BARO_VALID           = 1u << 1,
    ROCKET_FLAG_FUSION_VALID         = 1u << 2,
    ROCKET_FLAG_EKF_VALID            = 1u << 3,
    ROCKET_FLAG_CONTROLLER_ENABLED   = 1u << 4,
    ROCKET_FLAG_ARMED                = 1u << 4,
    ROCKET_FLAG_CONTROLLER_ACTIVE    = 1u << 5,
    ROCKET_FLAG_APOGEE_REACHED       = 1u << 6,
    ROCKET_FLAG_MODE_CHANGED         = 1u << 7,
    ROCKET_FLAG_BARO_CORRECTION_USED = 1u << 8,
    ROCKET_FLAG_FUSION_STARTUP       = 1u << 9,
    ROCKET_FLAG_ACCEL_IGNORED        = 1u << 10,
    ROCKET_FLAG_MAG_IGNORED          = 1u << 11,
    ROCKET_FLAG_PIPELINE_COMPLETE    = 1u << 12,
    ROCKET_FLAG_PIPELINE_PASS        = 1u << 13,
    ROCKET_FLAG_SENSOR_FAULT         = 1u << 14,
    ROCKET_FLAG_SIMULATION_ACTIVE    = 1u << 15
};

/* Four two-bit sensor-health values are packed into one telemetry byte. */
typedef enum
{
    ROCKET_SENSOR_UNKNOWN = 0,
    ROCKET_SENSOR_OK      = 1,
    ROCKET_SENSOR_STALE   = 2,
    ROCKET_SENSOR_FAULT   = 3
} RocketSensorHealth;

#define ROCKET_SENSOR_IMU_SHIFT      0u
#define ROCKET_SENSOR_BARO_SHIFT     2u
#define ROCKET_SENSOR_MAG_SHIFT      4u
#define ROCKET_SENSOR_ACTUATOR_SHIFT 6u
#define ROCKET_SENSOR_FIELD_MASK     0x03u

/* Simulation flags state which numeric channels are authoritative this sample. */
enum
{
    ROCKET_SIM_FLAG_ALTITUDE_VALID     = 1u << 0,
    ROCKET_SIM_FLAG_ACCELERATION_VALID = 1u << 1,
    ROCKET_SIM_FLAG_VELOCITY_VALID     = 1u << 2,
    ROCKET_SIM_FLAG_END_OF_STREAM      = 1u << 3
};

/* Actuator status flags describe readiness/state; they do not authorize motion. */
enum
{
    ROCKET_ACTUATOR_FLAG_BUILD_ENABLED = 1u << 0,
    ROCKET_ACTUATOR_FLAG_BENCH_ENABLED = 1u << 1,
    ROCKET_ACTUATOR_FLAG_HOMED         = 1u << 2,
    ROCKET_ACTUATOR_FLAG_DRIVER_OK     = 1u << 3,
    ROCKET_ACTUATOR_FLAG_DRIVER_ENABLED = 1u << 4,
    ROCKET_ACTUATOR_FLAG_ESTOP         = 1u << 5,
    ROCKET_ACTUATOR_FLAG_CONFIG_VALID  = 1u << 6,
    ROCKET_ACTUATOR_FLAG_MANUAL_PENDING = 1u << 7
};

/* ===================== FRAME RESULTS AND WIRE DATA MODELS ===================== */

typedef enum
{
    ROCKET_PROTOCOL_OK = 0,
    ROCKET_PROTOCOL_BAD_ARGUMENT,
    ROCKET_PROTOCOL_BAD_LENGTH,
    ROCKET_PROTOCOL_BAD_COBS,
    ROCKET_PROTOCOL_BAD_MAGIC,
    ROCKET_PROTOCOL_BAD_VERSION,
    ROCKET_PROTOCOL_BAD_CRC
} RocketProtocolResult;

typedef struct
{
    /* Parsed header fields; these are not laid out as the nine raw wire bytes. */
    uint8_t type;
    uint16_t sequence;
    uint32_t time_ms;
} RocketPacketHeader;

/* Telemetry wire size: 26 bytes.  Never transmit this struct with memcpy. */
typedef struct
{
    uint16_t flags;
    uint8_t state;             /* upper nibble reserved; lower nibble flight phase */
    uint8_t status_code;
    int16_t altitude_dm;       /* 0.1 m */
    int16_t velocity_cms;      /* 0.01 m/s */
    int16_t acceleration_cms2; /* 0.01 m/s^2 */
    uint16_t predicted_apogee_dm;
    uint16_t target_apogee_dm;
    int16_t roll_ddeg;         /* 0.1 degree; zero until attitude exists */
    int16_t pitch_ddeg;
    int16_t yaw_ddeg;
    uint8_t deployment_percent;
    uint8_t sensor_health;
    uint16_t failed_reads;
    uint8_t message_code;
    uint8_t reserved;
} RocketTelemetryPayload;

/* Event wire size: 10 bytes; reports state/flag transitions and one detail value. */
typedef struct
{
    uint16_t changed_flags;
    uint16_t current_flags;
    uint8_t previous_state;
    uint8_t current_state;
    uint8_t status_code;
    uint8_t message_code;
    uint16_t detail;
} RocketEventPayload;

/* Variable command body: two-byte command/length prefix plus at most eight bytes. */
typedef struct
{
    uint8_t command;
    uint8_t payload_length;
    uint8_t payload[ROCKET_COMMAND_DATA_MAX];
} RocketCommandPayload;

/* ACK wire size: 6 bytes; correlates result to command sequence and code. */
typedef struct
{
    uint16_t command_sequence;
    uint8_t command;
    uint8_t result;
    uint16_t detail;
} RocketAckPayload;

/* Simulation wire size: 16 bytes; fixed-point SI channels for deterministic replay. */
typedef struct
{
    uint16_t flags;
    int32_t altitude_mm;
    int32_t acceleration_mmps2;
    int32_t velocity_mmps;
    uint16_t barometer_stddev_cm;
} RocketSimulationPayload;

/* Actuator status wire size: 24 bytes; requested/internal positions are step counts. */
typedef struct
{
    uint32_t actuator_inhibit_flags;
    uint32_t flight_inhibit_flags;
    int32_t target_steps;
    int32_t actual_steps;
    uint32_t driver_status;
    uint8_t machine_state;
    uint8_t flags;
    uint16_t reserved;
} RocketActuatorStatusPayload;

/* Heartbeat wire size: 8 bytes; build identity plus transport error summary. */
typedef struct
{
    uint32_t feature_flags;
    uint16_t receive_errors;
    uint16_t transmit_drops;
} RocketHeartbeatPayload;

/* Maximum validated frame contents returned by RocketProtocol_DecodeFrame(). */
typedef struct
{
    RocketPacketHeader header;
    uint8_t payload_length;
    uint8_t payload[ROCKET_PROTOCOL_MAX_PAYLOAD];
} RocketDecodedPacket;

/* ===================== LITTLE-ENDIAN FIELD HELPERS ===================== */

/* These helpers are the only supported conversion between integers and wire bytes. */
static inline void RocketProtocol_WriteU16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static inline void RocketProtocol_WriteI16(uint8_t *p, int16_t v)
{
    RocketProtocol_WriteU16(p, (uint16_t)v);
}

static inline void RocketProtocol_WriteU32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static inline void RocketProtocol_WriteI32(uint8_t *p, int32_t v)
{
    RocketProtocol_WriteU32(p, (uint32_t)v);
}

static inline uint16_t RocketProtocol_ReadU16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline int16_t RocketProtocol_ReadI16(const uint8_t *p)
{
    return (int16_t)RocketProtocol_ReadU16(p);
}

static inline uint32_t RocketProtocol_ReadU32(const uint8_t *p)
{
    return (uint32_t)p[0]
        | ((uint32_t)p[1] << 8)
        | ((uint32_t)p[2] << 16)
        | ((uint32_t)p[3] << 24);
}

static inline int32_t RocketProtocol_ReadI32(const uint8_t *p)
{
    return (int32_t)RocketProtocol_ReadU32(p);
}

static inline uint8_t RocketProtocol_PackSensorHealth(uint8_t imu,
                                                       uint8_t barometer,
                                                       uint8_t magnetometer,
                                                       uint8_t actuator)
{
    return (uint8_t)(((imu & ROCKET_SENSOR_FIELD_MASK) << ROCKET_SENSOR_IMU_SHIFT)
        | ((barometer & ROCKET_SENSOR_FIELD_MASK) << ROCKET_SENSOR_BARO_SHIFT)
        | ((magnetometer & ROCKET_SENSOR_FIELD_MASK) << ROCKET_SENSOR_MAG_SHIFT)
        | ((actuator & ROCKET_SENSOR_FIELD_MASK) << ROCKET_SENSOR_ACTUATOR_SHIFT));
}

/* ===================== FRAME AND PAYLOAD CODEC API ===================== */

/* Integrity and COBS helpers return zero for invalid arguments/capacity failures. */
uint16_t RocketProtocol_Crc16(const uint8_t *data, size_t length);

size_t RocketProtocol_CobsEncode(const uint8_t *input,
                                size_t input_length,
                                uint8_t *output,
                                size_t output_capacity);

size_t RocketProtocol_CobsDecode(const uint8_t *input,
                                size_t input_length,
                                uint8_t *output,
                                size_t output_capacity);

size_t RocketProtocol_EncodeFrame(uint8_t *frame,
                                 size_t frame_capacity,
                                 uint8_t type,
                                 uint16_t sequence,
                                 uint32_t time_ms,
                                 const uint8_t *payload,
                                 size_t payload_length);

RocketProtocolResult RocketProtocol_DecodeFrame(const uint8_t *encoded_frame,
                                                size_t encoded_length,
                                                RocketDecodedPacket *packet);

/* Typed encoders return bytes written; typed decoders return 1 on exact success. */
size_t RocketProtocol_EncodeTelemetryPayload(uint8_t *out,
                                             size_t capacity,
                                             const RocketTelemetryPayload *payload);
size_t RocketProtocol_EncodeEventPayload(uint8_t *out,
                                         size_t capacity,
                                         const RocketEventPayload *payload);
size_t RocketProtocol_EncodeCommandPayload(uint8_t *out,
                                           size_t capacity,
                                           const RocketCommandPayload *payload);
int RocketProtocol_DecodeCommandPayload(const uint8_t *data,
                                        size_t length,
                                        RocketCommandPayload *payload);
size_t RocketProtocol_EncodeAckPayload(uint8_t *out,
                                       size_t capacity,
                                       const RocketAckPayload *payload);
size_t RocketProtocol_EncodeSimulationPayload(uint8_t *out,
                                              size_t capacity,
                                              const RocketSimulationPayload *payload);
int RocketProtocol_DecodeSimulationPayload(const uint8_t *data,
                                           size_t length,
                                           RocketSimulationPayload *payload);
size_t RocketProtocol_EncodeActuatorStatusPayload(
    uint8_t *out,
    size_t capacity,
    const RocketActuatorStatusPayload *payload);
size_t RocketProtocol_EncodeHeartbeatPayload(uint8_t *out,
                                             size_t capacity,
                                             const RocketHeartbeatPayload *payload);

/* ===================== DIAGNOSTIC TEXT ===================== */

/* Stable labels for logs/UI; unknown values return an explicit UNKNOWN string. */
const char *RocketProtocol_StatusText(uint8_t code);
const char *RocketProtocol_MessageText(uint8_t code);
const char *RocketProtocol_CommandText(uint8_t code);

#ifdef __cplusplus
}
#endif

#endif /* ROCKET_PROTOCOL_H */
