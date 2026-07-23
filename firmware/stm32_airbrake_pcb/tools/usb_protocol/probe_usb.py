"""Perform a read-only communication check on the STM32 USB-C interface.

The probe enumerates serial devices, selects AMBAR VID:PID ``0483:5740`` unless
the operator supplies a port, sends only ``PING`` and ``REQUEST_SNAPSHOT``, and
prints decoded replies for a bounded interval.  It does not HOME, ARM, start a
simulation, or issue any actuator command.

Use this before replay to separate cable/driver/protocol problems from motor or
flight-control problems.  See ``CODE_GUIDE.md`` [ARCH-3], [ARCH-6], [ARCH-8].
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import sys
import time
from typing import Any, Callable

from rocket_protocol import (
    ACK_OK,
    CMD_PING,
    CMD_REQUEST_SNAPSHOT,
    PKT_ACK,
    PKT_ACTUATOR_STATUS,
    PKT_EVENT,
    PKT_HEARTBEAT,
    PKT_TELEMETRY,
    StreamDecoder,
    command_frame,
    decode_ack,
    decode_actuator_status,
    decode_event,
    decode_heartbeat,
    decode_telemetry,
)


class ReadOnlyProbeError(RuntimeError):
    """Raised when the bounded safe probe cannot prove a fresh board response."""


@dataclass(frozen=True)
class ReadOnlyProbeResult:
    """Fresh evidence received after issuing only PING and REQUEST_SNAPSHOT."""

    heartbeat: dict[str, int]
    actuator: dict[str, int | bool]
    acknowledgements: dict[int, dict[str, int]]
    packet_count: int


def require_serial():
    """Load pyserial lazily so dry codec/tests do not require that dependency."""

    try:
        import serial  # type: ignore
        from serial.tools import list_ports  # type: ignore
    except ImportError as exc:
        raise SystemExit("pyserial is required: py -m pip install pyserial") from exc
    return serial, list_ports


def print_ports(list_ports) -> list:
    """Print discoverable ports and return the same materialized list."""

    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
    for port in ports:
        print(
            f"{port.device:8}  {port.description}  "
            f"VID:PID={port.vid or 0:04X}:{port.pid or 0:04X}  {port.hwid}"
        )
    return ports


def choose_port(ports, requested: str | None) -> str:
    """Select an explicit port or require exactly one AMBAR USB match."""

    if requested:
        return requested
    matches = []
    for port in ports:
        text = f"{port.description} {port.product or ''}".upper()
        if "AMBAR" in text or (port.vid == 0x0483 and port.pid == 0x5740):
            matches.append(port)
    if len(matches) == 1:
        return str(matches[0].device)
    if len(matches) > 1:
        devices = ", ".join(str(port.device) for port in matches)
        raise SystemExit(
            "multiple AMBAR USB devices were found "
            f"({devices}); pass --port COMx explicitly"
        )
    raise SystemExit("AMBAR USB device not found; pass --port COMx after checking --list")


def collect_read_only_status(
    port: Any,
    *,
    seconds: float = 3.0,
    sequence_start: int = 1,
    clock: Callable[[], float] = time.perf_counter,
    packet_callback: Callable[[Any], None] | None = None,
) -> ReadOnlyProbeResult:
    """Collect fresh heartbeat/actuator evidence using only two safe commands.

    The input buffer is discarded before either command is written, so every
    accepted packet was received after this probe began. Matching successful
    acknowledgements prove that the newly flashed application decoded both
    commands; heartbeat and actuator status then identify its runtime profile.
    """

    if seconds <= 0.0:
        raise ValueError("probe duration must be positive")

    ping_sequence = sequence_start & 0xFFFF
    snapshot_sequence = (sequence_start + 1) & 0xFFFF
    expected_commands = {
        ping_sequence: CMD_PING,
        snapshot_sequence: CMD_REQUEST_SNAPSHOT,
    }
    decoder = StreamDecoder()
    acknowledgements: dict[int, dict[str, int]] = {}
    heartbeat: dict[str, int] | None = None
    actuator: dict[str, int | bool] | None = None
    packet_count = 0

    port.reset_input_buffer()
    port.write(command_frame(ping_sequence, CMD_PING))
    port.write(command_frame(snapshot_sequence, CMD_REQUEST_SNAPSHOT))

    deadline = clock() + seconds
    while clock() < deadline:
        chunk = port.read(256)
        for packet in decoder.feed(chunk):
            packet_count += 1
            if packet_callback is not None:
                packet_callback(packet)
            if packet.packet_type == PKT_ACK:
                ack = decode_ack(packet.payload)
                expected = expected_commands.get(int(ack["command_sequence"]))
                if expected is not None:
                    if int(ack["command"]) != expected:
                        raise ReadOnlyProbeError(
                            "matching ACK sequence reported the wrong command"
                        )
                    if int(ack["result"]) != ACK_OK:
                        raise ReadOnlyProbeError(
                            f"read-only command 0x{expected:02X} was rejected "
                            f"with ACK result 0x{int(ack['result']):02X}"
                        )
                    acknowledgements[int(ack["command_sequence"])] = ack
            elif packet.packet_type == PKT_ACTUATOR_STATUS:
                actuator = decode_actuator_status(packet.payload)
            elif packet.packet_type == PKT_HEARTBEAT:
                heartbeat = decode_heartbeat(packet.payload)

        if (
            heartbeat is not None
            and actuator is not None
            and all(sequence in acknowledgements for sequence in expected_commands)
        ):
            break

    if decoder.errors:
        raise ReadOnlyProbeError(
            f"decoder rejected {decoder.errors} malformed frame(s)"
        )

    missing: list[str] = []
    if ping_sequence not in acknowledgements:
        missing.append("PING ACK")
    if snapshot_sequence not in acknowledgements:
        missing.append("REQUEST_SNAPSHOT ACK")
    if heartbeat is None:
        missing.append("fresh heartbeat")
    if actuator is None:
        missing.append("fresh actuator status")
    if missing:
        raise ReadOnlyProbeError(
            "timed out waiting for " + ", ".join(missing)
        )

    return ReadOnlyProbeResult(
        heartbeat=heartbeat,
        actuator=actuator,
        acknowledgements=acknowledgements,
        packet_count=packet_count,
    )


def print_decoded_packet(packet: Any) -> None:
    """Print one typed packet received by the read-only probe."""

    if packet.packet_type == PKT_ACK:
        print("ACK", decode_ack(packet.payload))
    elif packet.packet_type == PKT_TELEMETRY:
        print("TELEMETRY", decode_telemetry(packet.payload))
    elif packet.packet_type == PKT_ACTUATOR_STATUS:
        print("ACTUATOR", decode_actuator_status(packet.payload))
    elif packet.packet_type == PKT_EVENT:
        print("EVENT", decode_event(packet.payload))
    elif packet.packet_type == PKT_HEARTBEAT:
        print("HEARTBEAT", decode_heartbeat(packet.payload))
    else:
        print(
            f"PACKET type=0x{packet.packet_type:02X} "
            f"seq={packet.sequence} payload={packet.payload.hex()}"
        )


def main() -> int:
    """Run port enumeration or the bounded PING/snapshot probe."""

    parser = argparse.ArgumentParser()
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    parser.add_argument("--port", help="COM port, for example COM8")
    parser.add_argument("--seconds", type=float, default=3.0, help="listen duration")
    args = parser.parse_args()

    serial, list_ports = require_serial()
    ports = print_ports(list_ports)
    if args.list:
        return 0

    port_name = choose_port(ports, args.port)
    print(f"Opening {port_name}; sending PING and REQUEST_SNAPSHOT...")
    with serial.Serial(port_name, 115200, timeout=0.05, write_timeout=1.0) as port:
        try:
            collect_read_only_status(
                port,
                seconds=args.seconds,
                packet_callback=print_decoded_packet,
            )
        except ReadOnlyProbeError as error:
            print(f"Read-only probe failed: {error}", file=sys.stderr)
            return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
