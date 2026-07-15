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
import sys
import time

from rocket_protocol import (
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
    """Select an explicit port or the first AMBAR USB CDC VID/PID match."""

    if requested:
        return requested
    for port in ports:
        text = f"{port.description} {port.product or ''}".upper()
        if "AMBAR" in text or (port.vid == 0x0483 and port.pid == 0x5740):
            return port.device
    raise SystemExit("AMBAR USB device not found; pass --port COMx after checking --list")


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
    decoder = StreamDecoder()
    print(f"Opening {port_name}; sending PING and REQUEST_SNAPSHOT...")
    with serial.Serial(port_name, 115200, timeout=0.05, write_timeout=1.0) as port:
        port.reset_input_buffer()
        port.write(command_frame(1, CMD_PING))
        port.write(command_frame(2, CMD_REQUEST_SNAPSHOT))
        # perf_counter is high resolution on Windows; monotonic() in the bundled
        # runtime advances in coarse 15.625 ms steps.
        deadline = time.perf_counter() + args.seconds
        while time.perf_counter() < deadline:
            chunk = port.read(256)
            for packet in decoder.feed(chunk):
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
    if decoder.errors:
        print(f"Decoder rejected {decoder.errors} malformed frame(s).", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
