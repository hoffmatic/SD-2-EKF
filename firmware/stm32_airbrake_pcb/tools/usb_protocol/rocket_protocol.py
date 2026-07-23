"""Encode and decode the AMBAR USB protocol-v2 wire format.

Overview
========
This module is the host-side reference implementation of
``Core/Inc/rocket_protocol.h``.  It is intentionally independent of pyserial:
callers provide and consume byte strings, while this file owns packet layout,
COBS framing, CRC validation, stream resynchronization, and engineering-unit
conversion.

How data moves
==============
Command and simulation helpers pack typed payloads, :func:`encode_frame` adds
the versioned header and CRC, and COBS removes zero bytes before the final
``0x00`` serial delimiter.  Receive code feeds arbitrary chunks into
:class:`StreamDecoder`; complete validated packets are then interpreted by the
payload-specific decoder functions near the end of the file.

Safety and compatibility
========================
Packet sizes and payload layouts must remain byte-for-byte compatible with the
STM32 definition.  This codec does not authorize motor motion; firmware gates
and the explicit host workflows remain responsible for that decision.  See
``CODE_GUIDE.md`` [ARCH-3], [ARCH-6], and [ARCH-8].
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
import time
from typing import Iterable
import zlib


# ---------------------------------------------------------------------------
# Wire-format limits and protocol identifiers
# ---------------------------------------------------------------------------
# These values mirror rocket_protocol.h. Changing one side without the other
# causes frames to be rejected before any command reaches the application.
MAGIC = 0xA5
VERSION = 0x02
HEADER_SIZE = 9
CRC_SIZE = 2
MAX_PACKET = 64
MAX_PAYLOAD = MAX_PACKET - HEADER_SIZE - CRC_SIZE
MAX_FRAME = 66

PKT_TELEMETRY = 0x01
PKT_EVENT = 0x02
PKT_ACTUATOR_STATUS = 0x03
PKT_VARIABLE_HIL_STATE = 0x04
PKT_VARIABLE_HIL_CONFIG = 0x05
PKT_COMMAND = 0x10
PKT_ACK = 0x11
PKT_HEARTBEAT = 0x12
PKT_SIMULATION = 0x20
PKT_VARIABLE_HIL_CONFIG_UPLOAD = 0x21

CMD_NOP = 0x00
CMD_PING = 0x01
CMD_REQUEST_SNAPSHOT = 0x02
CMD_SET_TARGET_APOGEE = 0x10
CMD_SET_ARMED = 0x11
CMD_ESTOP = 0x15
CMD_HOME = 0x16
CMD_RETRACT = 0x17
CMD_PAD_RESET = 0x18
CMD_SAVE_CONFIG = 0x19
CMD_BENCH_MOVE_STEPS = 0x1A
CMD_SIM_START = 0x20
CMD_SIM_STOP = 0x21
CMD_HIL_SET_OVERRIDE = 0x22
CMD_VARIABLE_HIL_GET_CONFIG = 0x23
CMD_VARIABLE_HIL_CONFIG_UPLOAD = 0x24
CMD_RECOVER_KNOWN_FULL_RETRACT = 0x25
CMD_START_LOG = 0x30
CMD_STOP_LOG = 0x31
CMD_ERASE_LOG = 0x32

ACK_OK = 0x00
ACK_BAD_LENGTH = 0x01
ACK_BAD_VALUE = 0x02
ACK_UNSUPPORTED = 0x03
ACK_BUSY = 0x04
ACK_EXECUTION_ERROR = 0x05
ACK_BAD_CRC = 0x06

SIM_ALTITUDE_VALID = 1 << 0
SIM_ACCELERATION_VALID = 1 << 1
SIM_VELOCITY_VALID = 1 << 2
SIM_END_OF_STREAM = 1 << 3

HIL_OVERRIDE_OFF = 0
HIL_OVERRIDE_FORCE_FULL = 1
HIL_OVERRIDE_FORCE_HOME = 2

# Required payload for the CONTINUOUS_HIL-only reset-zero recovery command.
RECOVER_KNOWN_FULL_MAGIC = b"FULL"

# Protocol-v2 keeps the original reserved-bit positions so packet size and
# older captures remain compatible. In the switch-free build these bits
# describe software geometry derived from the TMC5240 ramp state; they are not
# physical switch or encoder evidence.
ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE = 1 << 0
ACTUATOR_STATUS_SOFTWARE_FULL_ACTIVE = 1 << 1
ACTUATOR_STATUS_GEOMETRY_PLAUSIBLE = 1 << 2
ACTUATOR_STATUS_HIL_OVERRIDE_ACTIVE = 1 << 3
ACTUATOR_STATUS_HIL_OVERRIDE_SHIFT = 4
ACTUATOR_STATUS_HIL_OVERRIDE_MASK = 0x0030
ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE = 1 << 6
ACTUATOR_STATUS_STROKE_SEQUENCE_VERIFIED = 1 << 7
ACTUATOR_STATUS_VARIABLE_HIL_PROFILE = 1 << 8

# RocketActuatorStatusPayload.flags (distinct from the reserved status bits).
ACTUATOR_FLAG_DRIVER_ENABLED = 1 << 4

VARIABLE_HIL_STATE_PAYLOAD_SIZE = 44
VARIABLE_HIL_CONFIG_PAYLOAD_SIZE = 52
VARIABLE_HIL_CONFIG_VERSION = 1
VARIABLE_HIL_CDA_POINT_COUNT = 5
FRACTION_U16_FULL_SCALE = 65535
FEATURE_ACTUATOR = 1 << 3
FEATURE_USB_PROTOCOL = 1 << 10
FEATURE_SIMULATION = 1 << 11
FEATURE_CONTINUOUS_HIL = 1 << 14
FEATURE_VARIABLE_HIL = 1 << 16
VARIABLE_HIL_FEEDBACK_UNKNOWN = 0
VARIABLE_HIL_FEEDBACK_TMC5240_XACTUAL = 1

VARIABLE_HIL_FLAG_DRIVER_OK = 1 << 0
VARIABLE_HIL_FLAG_DRIVER_ENABLED = 1 << 1
VARIABLE_HIL_FLAG_CONFIG_VALID = 1 << 2
VARIABLE_HIL_FLAG_SIM_ACTIVE = 1 << 3
VARIABLE_HIL_FLAG_SIM_FRESH = 1 << 4
VARIABLE_HIL_FLAG_ARMED = 1 << 5
VARIABLE_HIL_FLAG_SOFTWARE_HOME = 1 << 6
VARIABLE_HIL_FLAG_TARGET_REACHABLE = 1 << 7

# Compatibility aliases for existing protocol-v2 consumers. New code and UI
# text should use the software-geometry names above.
ACTUATOR_STATUS_HOME_ACTIVE = ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE
ACTUATOR_STATUS_FULL_ACTIVE = ACTUATOR_STATUS_SOFTWARE_FULL_ACTIVE
ACTUATOR_STATUS_LIMITS_PLAUSIBLE = ACTUATOR_STATUS_GEOMETRY_PLAUSIBLE
ACTUATOR_STATUS_ENDPOINT_SEQUENCE_VERIFIED = (
    ACTUATOR_STATUS_STROKE_SEQUENCE_VERIFIED
)


class ProtocolError(ValueError):
    """Raised when bytes cannot represent one valid protocol-v2 packet."""

    pass


@dataclass(frozen=True)
class Packet:
    """One validated packet with the raw payload left for a typed decoder.

    ``time_ms`` is the sender's diagnostic timestamp.  It is not used as the
    flight estimator timestamp and must not be confused with host receive time.
    """

    packet_type: int
    sequence: int
    time_ms: int
    payload: bytes


@dataclass(frozen=True)
class VariableHilConfig:
    """Canonical atomic controller configuration shared with VARIABLE_HIL."""

    calibration_version: int
    control_mode: int
    predictor_mode: int
    target_apogee_m: float
    mission_tolerance_m: float
    control_deadband_m: float
    full_deployment_error_m: float
    minimum_deploy_altitude_m: float
    minimum_flight_time_s: float
    predictive_update_period_s: float
    coast_mass_kg: float
    maximum_deploy_fraction: float
    deployment_hysteresis_fraction: float
    deployment_cda_m2: tuple[float, float, float, float, float]
    sea_level_air_density_kgpm3: float
    density_scale_height_m: float
    launch_site_elevation_m: float
    actuator_delay_s: float
    actuator_open_rate_fraction_per_s: float
    actuator_close_rate_fraction_per_s: float


def crc16_ccitt_false(data: bytes) -> int:
    """Return the CRC-16/CCITT-FALSE value used by firmware frames."""

    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    """COBS-encode raw bytes so zero can remain the serial frame delimiter."""

    output = bytearray(b"\x00")
    code_index = 0
    code = 1
    for value in data:
        if value == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(value)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1
    output[code_index] = code
    return bytes(output)


def cobs_decode(data: bytes) -> bytes:
    """Decode one delimiter-free COBS frame or raise :class:`ProtocolError`."""

    if not data:
        raise ProtocolError("empty COBS frame")
    output = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        index += 1
        if code == 0:
            raise ProtocolError("zero byte inside COBS frame")
        end = index + code - 1
        if end > len(data):
            raise ProtocolError("truncated COBS block")
        output.extend(data[index:end])
        index = end
        if code != 0xFF and index < len(data):
            output.append(0)
    return bytes(output)


def encode_frame(
    packet_type: int,
    sequence: int,
    time_ms: int,
    payload: bytes = b"",
) -> bytes:
    """Build one complete COBS frame, including its trailing zero delimiter."""

    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError(f"payload exceeds {MAX_PAYLOAD} bytes")
    header = struct.pack("<BBBHI", MAGIC, VERSION, packet_type, sequence & 0xFFFF, time_ms & 0xFFFFFFFF)
    raw = header + payload
    raw += struct.pack("<H", crc16_ccitt_false(raw))
    frame = cobs_encode(raw) + b"\x00"
    if len(frame) > MAX_FRAME:
        raise ProtocolError("encoded frame exceeds transport maximum")
    return frame


def decode_frame(encoded_without_delimiter: bytes) -> Packet:
    """Validate and decode one COBS frame that does not include ``0x00``."""

    raw = cobs_decode(encoded_without_delimiter)
    if not HEADER_SIZE + CRC_SIZE <= len(raw) <= MAX_PACKET:
        raise ProtocolError("invalid raw packet length")
    magic, version, packet_type, sequence, time_ms = struct.unpack_from("<BBBHI", raw)
    if magic != MAGIC:
        raise ProtocolError("bad magic")
    if version != VERSION:
        raise ProtocolError(f"unsupported version {version}")
    received_crc = struct.unpack_from("<H", raw, len(raw) - CRC_SIZE)[0]
    if crc16_ccitt_false(raw[:-CRC_SIZE]) != received_crc:
        raise ProtocolError("bad CRC")
    return Packet(packet_type, sequence, time_ms, raw[HEADER_SIZE:-CRC_SIZE])


class StreamDecoder:
    """Incremental decoder tolerant of arbitrary serial read boundaries.

    Malformed or oversized input is counted and discarded through the next
    delimiter so a damaged frame cannot desynchronize every later packet.
    """

    def __init__(self) -> None:
        self._encoded = bytearray()
        self._drop_until_delimiter = False
        self.errors = 0

    def feed(self, chunk: bytes | bytearray | memoryview) -> list[Packet]:
        """Consume a byte chunk and return every complete validated packet."""

        packets: list[Packet] = []
        for value in bytes(chunk):
            if value == 0:
                if self._drop_until_delimiter:
                    self._drop_until_delimiter = False
                    self._encoded.clear()
                elif self._encoded:
                    try:
                        packets.append(decode_frame(bytes(self._encoded)))
                    except ProtocolError:
                        self.errors += 1
                    self._encoded.clear()
                continue
            if self._drop_until_delimiter:
                continue
            if len(self._encoded) >= MAX_FRAME - 1:
                self.errors += 1
                self._encoded.clear()
                self._drop_until_delimiter = True
                continue
            self._encoded.append(value)
        return packets


# ---------------------------------------------------------------------------
# Typed transmit helpers
# ---------------------------------------------------------------------------


def command_frame(
    sequence: int,
    command: int,
    data: bytes = b"",
    time_ms: int | None = None,
) -> bytes:
    """Encode one bounded command payload for the STM32 command dispatcher."""

    if len(data) > 8:
        raise ProtocolError("command data exceeds 8 bytes")
    if time_ms is None:
        time_ms = int(time.monotonic() * 1000)
    return encode_frame(PKT_COMMAND, sequence, time_ms, bytes((command, len(data))) + data)


def simulation_frame(
    sequence: int,
    *,
    altitude_m: float,
    acceleration_mps2: float,
    velocity_mps: float | None = None,
    barometer_stddev_m: float = 1.5,
    end_of_stream: bool = False,
    time_ms: int | None = None,
) -> bytes:
    """Encode one synthetic vertical-state sample in fixed-point wire units.

    Altitude, acceleration, and optional velocity are converted to millimetre
    based integers; barometer uncertainty is carried in centimetres.
    """

    flags = SIM_ALTITUDE_VALID | SIM_ACCELERATION_VALID
    velocity_mmps = 0
    if velocity_mps is not None:
        flags |= SIM_VELOCITY_VALID
        velocity_mmps = round(velocity_mps * 1000)
    if end_of_stream:
        flags |= SIM_END_OF_STREAM
    payload = struct.pack(
        "<HiiiH",
        flags,
        round(altitude_m * 1000),
        round(acceleration_mps2 * 1000),
        velocity_mmps,
        round(barometer_stddev_m * 100),
    )
    if time_ms is None:
        time_ms = int(time.monotonic() * 1000)
    return encode_frame(PKT_SIMULATION, sequence, time_ms, payload)


def hil_override_frame(
    sequence: int,
    mode: int,
    *,
    time_ms: int | None = None,
) -> bytes:
    """Encode the fixed one-byte continuous-HIL override command.

    Firmware remains the authority for arming, freshness, software geometry,
    driver health, and build-profile gating. This helper only prevents
    malformed or future-reserved override values from reaching the wire.
    """

    if mode not in (
        HIL_OVERRIDE_OFF,
        HIL_OVERRIDE_FORCE_FULL,
        HIL_OVERRIDE_FORCE_HOME,
    ):
        raise ProtocolError(f"unsupported HIL override mode {mode}")
    return command_frame(
        sequence,
        CMD_HIL_SET_OVERRIDE,
        bytes((mode,)),
        time_ms=time_ms,
    )


def recover_known_full_retract_frame(
    sequence: int,
    *,
    time_ms: int | None = None,
) -> bytes:
    """Encode the explicit CONTINUOUS_HIL known-FULL recovery command.

    The firmware remains responsible for profile, disarmed/SIM_STOP, driver-off,
    health, unhomed, and reset-zero ramp-state gates.
    """

    return command_frame(
        sequence,
        CMD_RECOVER_KNOWN_FULL_RETRACT,
        RECOVER_KNOWN_FULL_MAGIC,
        time_ms=time_ms,
    )


def _scaled_int(name: str, value: float, scale: float, lower: int, upper: int) -> int:
    scaled = round(value * scale)
    if not lower <= scaled <= upper:
        raise ProtocolError(
            f"{name}={value!r} is outside encodable range "
            f"[{lower / scale}, {upper / scale}]"
        )
    return scaled


def encode_variable_hil_config_payload(config: VariableHilConfig) -> bytes:
    """Encode one versioned, self-CRC'd atomic VARIABLE_HIL configuration."""

    if len(config.deployment_cda_m2) != VARIABLE_HIL_CDA_POINT_COUNT:
        raise ProtocolError("deployment CdA curve must contain five points")
    fields = (
        VARIABLE_HIL_CONFIG_VERSION,
        config.control_mode,
        config.predictor_mode,
        VARIABLE_HIL_CDA_POINT_COUNT,
        config.calibration_version,
        _scaled_int("target_apogee_m", config.target_apogee_m, 10.0, 0, 0xFFFF),
        _scaled_int("mission_tolerance_m", config.mission_tolerance_m, 10.0, 0, 0xFFFF),
        _scaled_int("control_deadband_m", config.control_deadband_m, 100.0, 0, 0xFFFF),
        _scaled_int("full_deployment_error_m", config.full_deployment_error_m, 10.0, 0, 0xFFFF),
        _scaled_int("minimum_deploy_altitude_m", config.minimum_deploy_altitude_m, 10.0, 0, 0xFFFF),
        _scaled_int("minimum_flight_time_s", config.minimum_flight_time_s, 100.0, 0, 0xFFFF),
        _scaled_int("predictive_update_period_s", config.predictive_update_period_s, 1000.0, 0, 0xFFFF),
        _scaled_int("coast_mass_kg", config.coast_mass_kg, 1000.0, 0, 0xFFFF),
        _scaled_int("maximum_deploy_fraction", config.maximum_deploy_fraction, 255.0, 0, 0xFF),
        _scaled_int("deployment_hysteresis_fraction", config.deployment_hysteresis_fraction, 1000.0, 0, 0xFF),
        *(
            _scaled_int(f"deployment_cda_m2[{index}]", value, 1_000_000.0, 0, 0xFFFF)
            for index, value in enumerate(config.deployment_cda_m2)
        ),
        _scaled_int("sea_level_air_density_kgpm3", config.sea_level_air_density_kgpm3, 10_000.0, 0, 0xFFFF),
        _scaled_int("density_scale_height_m", config.density_scale_height_m, 1.0, 0, 0xFFFF),
        _scaled_int("launch_site_elevation_m", config.launch_site_elevation_m, 10.0, -0x8000, 0x7FFF),
        _scaled_int("actuator_delay_s", config.actuator_delay_s, 1000.0, 0, 0xFFFF),
        _scaled_int("actuator_open_rate_fraction_per_s", config.actuator_open_rate_fraction_per_s, 1000.0, 0, 0xFFFF),
        _scaled_int("actuator_close_rate_fraction_per_s", config.actuator_close_rate_fraction_per_s, 1000.0, 0, 0xFFFF),
    )
    body = struct.pack("<BBBBI8HBB5HHHhHHH", *fields)
    if len(body) != VARIABLE_HIL_CONFIG_PAYLOAD_SIZE - 4:
        raise AssertionError("VARIABLE_HIL config body layout drifted")
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def variable_hil_config_upload_frame(
    sequence: int,
    config: VariableHilConfig,
    *,
    time_ms: int | None = None,
) -> bytes:
    """Upload an atomic config; firmware accepts it only in its energy-off state."""

    if time_ms is None:
        time_ms = int(time.monotonic() * 1000)
    return encode_frame(
        PKT_VARIABLE_HIL_CONFIG_UPLOAD,
        sequence,
        time_ms,
        encode_variable_hil_config_payload(config),
    )


# ---------------------------------------------------------------------------
# Typed receive-payload decoders
# ---------------------------------------------------------------------------


def decode_ack(payload: bytes) -> dict[str, int]:
    """Decode a command acknowledgement and its original command sequence."""

    if len(payload) != 6:
        raise ProtocolError("ACK payload must be 6 bytes")
    command_sequence, command, result, detail = struct.unpack("<HBBH", payload)
    return {
        "command_sequence": command_sequence,
        "command": command,
        "result": result,
        "detail": detail,
    }


def decode_event(payload: bytes) -> dict[str, int]:
    """Decode a state/flag transition event emitted by the application."""

    if len(payload) != 10:
        raise ProtocolError("event payload must be 10 bytes")
    values = struct.unpack("<HHBBBBH", payload)
    return {
        "changed_flags": values[0],
        "current_flags": values[1],
        "previous_state": values[2],
        "current_state": values[3],
        "status_code": values[4],
        "message_code": values[5],
        "detail": values[6],
    }


def decode_telemetry(payload: bytes) -> dict[str, int | float]:
    """Decode the compact flight snapshot and restore engineering units."""

    if len(payload) != 26:
        raise ProtocolError("telemetry payload must be 26 bytes")
    fields = struct.unpack("<HBBhhhHHhhhBBHBB", payload)
    return {
        "flags": fields[0],
        "state": fields[1],
        "status_code": fields[2],
        "altitude_m": fields[3] * 0.1,
        "velocity_mps": fields[4] * 0.01,
        "acceleration_mps2": fields[5] * 0.01,
        "predicted_apogee_m": fields[6] * 0.1,
        "target_apogee_m": fields[7] * 0.1,
        "roll_deg": fields[8] * 0.1,
        "pitch_deg": fields[9] * 0.1,
        "yaw_deg": fields[10] * 0.1,
        "deployment_percent": fields[11],
        "sensor_health": fields[12],
        "failed_reads": fields[13],
        "message_code": fields[14],
        "reserved": fields[15],
    }


def decode_actuator_status(payload: bytes) -> dict[str, int | bool]:
    """Decode actuator intent, ramp position, inhibit flags, and driver state."""

    if len(payload) != 24:
        raise ProtocolError("actuator status payload must be 24 bytes")
    values = struct.unpack("<IIiiIBBH", payload)
    reserved = values[7]
    return {
        "actuator_inhibit_flags": values[0],
        "flight_inhibit_flags": values[1],
        "target_steps": values[2],
        "actual_steps": values[3],
        "driver_status": values[4],
        "machine_state": values[5],
        "flags": values[6],
        "driver_enabled": bool(values[6] & ACTUATOR_FLAG_DRIVER_ENABLED),
        "reserved": reserved,
        "software_home_active": bool(
            reserved & ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE
        ),
        "software_full_active": bool(
            reserved & ACTUATOR_STATUS_SOFTWARE_FULL_ACTIVE
        ),
        "geometry_plausible": bool(
            reserved & ACTUATOR_STATUS_GEOMETRY_PLAUSIBLE
        ),
        "hil_override_active": bool(
            reserved & ACTUATOR_STATUS_HIL_OVERRIDE_ACTIVE
        ),
        "hil_override_mode": (
            reserved & ACTUATOR_STATUS_HIL_OVERRIDE_MASK
        )
        >> ACTUATOR_STATUS_HIL_OVERRIDE_SHIFT,
        "continuous_hil_profile": bool(
            reserved & ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE
        ),
        "stroke_sequence_verified": bool(
            reserved & ACTUATOR_STATUS_STROKE_SEQUENCE_VERIFIED
        ),
        "variable_hil_profile": bool(
            reserved & ACTUATOR_STATUS_VARIABLE_HIL_PROFILE
        ),
        # Legacy dictionary keys remain available for protocol-v2 callers.
        "home_active": bool(
            reserved & ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE
        ),
        "full_active": bool(
            reserved & ACTUATOR_STATUS_SOFTWARE_FULL_ACTIVE
        ),
        "limits_plausible": bool(
            reserved & ACTUATOR_STATUS_GEOMETRY_PLAUSIBLE
        ),
        "endpoint_sequence_verified": bool(
            reserved & ACTUATOR_STATUS_STROKE_SEQUENCE_VERIFIED
        ),
    }


def decode_variable_hil_state(payload: bytes) -> dict[str, int | float | bool]:
    """Decode one causal state reply correlated to a simulation input sequence."""

    if len(payload) != VARIABLE_HIL_STATE_PAYLOAD_SIZE:
        raise ProtocolError("VARIABLE_HIL state payload must be 44 bytes")
    values = struct.unpack("<HHHHiiIIIIiiBBBB", payload)
    flags = values[14]
    return {
        "simulation_sequence": values[0],
        "controller_requested_fraction": values[1] / FRACTION_U16_FULL_SCALE,
        "actuator_target_fraction": values[2] / FRACTION_U16_FULL_SCALE,
        "xactual_fraction": values[3] / FRACTION_U16_FULL_SCALE,
        "target_steps": values[4],
        "actual_steps": values[5],
        "flight_inhibit_flags": values[6],
        "actuator_inhibit_flags": values[7],
        "driver_status": values[8],
        "config_crc32": values[9],
        "closed_predicted_apogee_m": values[10] * 0.1,
        "full_predicted_apogee_m": values[11] * 0.1,
        "phase": values[12],
        "machine_state": values[13],
        "state_flags": flags,
        "feedback_source": values[15],
        "driver_ok": bool(flags & VARIABLE_HIL_FLAG_DRIVER_OK),
        "driver_enabled": bool(flags & VARIABLE_HIL_FLAG_DRIVER_ENABLED),
        "config_valid": bool(flags & VARIABLE_HIL_FLAG_CONFIG_VALID),
        "simulation_active": bool(flags & VARIABLE_HIL_FLAG_SIM_ACTIVE),
        "simulation_fresh": bool(flags & VARIABLE_HIL_FLAG_SIM_FRESH),
        "armed": bool(flags & VARIABLE_HIL_FLAG_ARMED),
        "software_home": bool(flags & VARIABLE_HIL_FLAG_SOFTWARE_HOME),
        "target_reachable": bool(flags & VARIABLE_HIL_FLAG_TARGET_REACHABLE),
    }


def decode_variable_hil_config(payload: bytes) -> dict[str, int | float | tuple[float, ...]]:
    """Decode and CRC-check the complete board config readback."""

    if len(payload) != VARIABLE_HIL_CONFIG_PAYLOAD_SIZE:
        raise ProtocolError("VARIABLE_HIL config payload must be 52 bytes")
    received_crc = struct.unpack_from("<I", payload, 48)[0]
    calculated_crc = zlib.crc32(payload[:48]) & 0xFFFFFFFF
    if received_crc != calculated_crc:
        raise ProtocolError(
            f"VARIABLE_HIL config CRC mismatch: received 0x{received_crc:08X}, "
            f"calculated 0x{calculated_crc:08X}"
        )
    values = struct.unpack("<BBBBI8HBB5HHHhHHHI", payload)
    if values[0] != VARIABLE_HIL_CONFIG_VERSION:
        raise ProtocolError(f"unsupported VARIABLE_HIL config version {values[0]}")
    if values[3] != VARIABLE_HIL_CDA_POINT_COUNT:
        raise ProtocolError(f"unsupported CdA point count {values[3]}")
    return {
        "schema_version": values[0],
        "control_mode": values[1],
        "predictor_mode": values[2],
        "cda_point_count": values[3],
        "calibration_version": values[4],
        "target_apogee_m": values[5] * 0.1,
        "mission_tolerance_m": values[6] * 0.1,
        "control_deadband_m": values[7] * 0.01,
        "full_deployment_error_m": values[8] * 0.1,
        "minimum_deploy_altitude_m": values[9] * 0.1,
        "minimum_flight_time_s": values[10] * 0.01,
        "predictive_update_period_s": values[11] * 0.001,
        "coast_mass_kg": values[12] * 0.001,
        "maximum_deploy_fraction": values[13] / 255.0,
        "deployment_hysteresis_fraction": values[14] * 0.001,
        "deployment_cda_m2": tuple(value * 1e-6 for value in values[15:20]),
        "sea_level_air_density_kgpm3": values[20] * 1e-4,
        "density_scale_height_m": float(values[21]),
        "launch_site_elevation_m": values[22] * 0.1,
        "actuator_delay_s": values[23] * 0.001,
        "actuator_open_rate_fraction_per_s": values[24] * 0.001,
        "actuator_close_rate_fraction_per_s": values[25] * 0.001,
        "config_crc32": values[26],
    }


def decode_heartbeat(payload: bytes) -> dict[str, int]:
    """Decode build feature bits and cumulative USB error/drop counters."""

    if len(payload) != 8:
        raise ProtocolError("heartbeat payload must be 8 bytes")
    feature_flags, receive_errors, transmit_drops = struct.unpack("<IHH", payload)
    return {
        "feature_flags": feature_flags,
        "receive_errors": receive_errors,
        "transmit_drops": transmit_drops,
    }


def decode_chunks(chunks: Iterable[bytes]) -> list[Packet]:
    """Convenience helper used by tests to decode an iterable of read chunks."""

    decoder = StreamDecoder()
    packets: list[Packet] = []
    for chunk in chunks:
        packets.extend(decoder.feed(chunk))
    return packets
