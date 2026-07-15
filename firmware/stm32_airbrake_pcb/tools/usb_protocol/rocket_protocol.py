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
PKT_COMMAND = 0x10
PKT_ACK = 0x11
PKT_HEARTBEAT = 0x12
PKT_SIMULATION = 0x20

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


def decode_actuator_status(payload: bytes) -> dict[str, int]:
    """Decode actuator intent, ramp position, inhibit flags, and driver state."""

    if len(payload) != 24:
        raise ProtocolError("actuator status payload must be 24 bytes")
    values = struct.unpack("<IIiiIBBH", payload)
    return {
        "actuator_inhibit_flags": values[0],
        "flight_inhibit_flags": values[1],
        "target_steps": values[2],
        "actual_steps": values[3],
        "driver_status": values[4],
        "machine_state": values[5],
        "flags": values[6],
        "reserved": values[7],
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
