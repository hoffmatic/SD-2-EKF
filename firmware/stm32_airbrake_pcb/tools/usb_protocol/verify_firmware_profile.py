"""Verify the flashed AMBAR build profile without enabling motor energy.

This verifier deliberately shares the bounded probe used for USB diagnostics.
It sends only protocol-v2 PING and REQUEST_SNAPSHOT, then validates fresh
heartbeat and actuator-status identity. It never arms, starts simulation,
sets a HIL override, homes, retracts, or sends a bench-motion command.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any

from probe_usb import (
    ReadOnlyProbeError,
    ReadOnlyProbeResult,
    choose_port,
    collect_read_only_status,
    require_serial,
)


FEATURE_ACTUATOR = 1 << 3
FEATURE_USB_PROTOCOL = 1 << 10
FEATURE_SIMULATION = 1 << 11
FEATURE_CONTINUOUS_HIL = 1 << 14
FEATURE_VARIABLE_HIL = 1 << 16


class ProfileVerificationError(RuntimeError):
    """Raised when fresh runtime identity does not match the flashed profile."""


def validate_runtime_profile(
    profile: str,
    heartbeat: dict[str, Any],
    actuator: dict[str, Any],
) -> dict[str, Any]:
    """Validate the exact runtime identity required for one build profile."""

    if profile not in {"Normal", "ContinuousHil", "VariableHil"}:
        raise ValueError(f"unsupported profile {profile!r}")

    features = int(heartbeat["feature_flags"])
    actuator_hil_profile = bool(actuator["continuous_hil_profile"])
    actuator_variable_profile = bool(actuator.get("variable_hil_profile", False))

    if profile == "Normal":
        forbidden = FEATURE_SIMULATION | FEATURE_CONTINUOUS_HIL | FEATURE_VARIABLE_HIL
        reported_forbidden = features & forbidden
        if reported_forbidden:
            raise ProfileVerificationError(
                "NORMAL reported simulation/continuous-HIL feature bits "
                f"0x{reported_forbidden:08X} in feature word 0x{features:08X}"
            )
        if actuator_hil_profile:
            raise ProfileVerificationError(
                "NORMAL actuator status reported the continuous-HIL profile bit"
            )
        if actuator_variable_profile:
            raise ProfileVerificationError(
                "NORMAL actuator status reported the variable-HIL profile bit"
            )
    elif profile == "ContinuousHil":
        required = (
            FEATURE_USB_PROTOCOL
            | FEATURE_SIMULATION
            | FEATURE_ACTUATOR
            | FEATURE_CONTINUOUS_HIL
        )
        missing = required & ~features
        if missing:
            raise ProfileVerificationError(
                "CONTINUOUS_HIL is missing required USB/simulation/actuator/HIL "
                f"feature bits 0x{missing:08X} from feature word 0x{features:08X}"
            )
        if not actuator_hil_profile:
            raise ProfileVerificationError(
                "CONTINUOUS_HIL actuator status did not report its HIL-profile bit"
            )
        if features & FEATURE_VARIABLE_HIL or actuator_variable_profile:
            raise ProfileVerificationError(
                "CONTINUOUS_HIL reported VARIABLE_HIL identity"
            )
    else:
        required = (
            FEATURE_USB_PROTOCOL
            | FEATURE_SIMULATION
            | FEATURE_ACTUATOR
            | FEATURE_VARIABLE_HIL
        )
        missing = required & ~features
        if missing:
            raise ProfileVerificationError(
                "VARIABLE_HIL is missing required USB/simulation/actuator/profile "
                f"feature bits 0x{missing:08X} from feature word 0x{features:08X}"
            )
        if features & FEATURE_CONTINUOUS_HIL or actuator_hil_profile:
            raise ProfileVerificationError(
                "VARIABLE_HIL reported forced CONTINUOUS_HIL identity"
            )
        if not actuator_variable_profile:
            raise ProfileVerificationError(
                "VARIABLE_HIL actuator status did not report its profile bit"
            )

    return {
        "profile": profile,
        "feature_flags": features,
        "continuous_hil_profile": actuator_hil_profile,
        "variable_hil_profile": actuator_variable_profile,
        "receive_errors": int(heartbeat.get("receive_errors", 0)),
        "transmit_drops": int(heartbeat.get("transmit_drops", 0)),
    }


def _available_port(list_ports: Any, requested: str | None) -> str | None:
    ports = list(list_ports.comports())
    if requested:
        requested_upper = requested.upper()
        for port in ports:
            if str(port.device).upper() == requested_upper:
                return str(port.device)
        return None
    try:
        return choose_port(ports, None)
    except SystemExit as error:
        if "multiple AMBAR USB devices" in str(error):
            raise ProfileVerificationError(str(error)) from error
        return None


def verify_connected_profile(
    *,
    serial_module: Any,
    list_ports: Any,
    profile: str,
    requested_port: str | None,
    reconnect_seconds: float,
    listen_seconds: float,
) -> tuple[str, ReadOnlyProbeResult, dict[str, Any]]:
    """Wait for USB enumeration, run the safe probe, and validate the profile."""

    if reconnect_seconds <= 0.0:
        raise ValueError("USB reconnect timeout must be positive")
    deadline = time.perf_counter() + reconnect_seconds
    last_connection_error: Exception | None = None
    serial_exception = getattr(serial_module, "SerialException", OSError)

    while time.perf_counter() < deadline:
        port_name = _available_port(list_ports, requested_port)
        if port_name is None:
            time.sleep(0.25)
            continue
        remaining_seconds = deadline - time.perf_counter()
        if remaining_seconds <= 0.0:
            break
        try:
            with serial_module.Serial(
                port_name,
                115200,
                timeout=0.05,
                write_timeout=1.0,
            ) as port:
                sequence_start = int(time.perf_counter() * 1000.0) & 0xFFFF
                result = collect_read_only_status(
                    port,
                    seconds=min(listen_seconds, remaining_seconds),
                    sequence_start=sequence_start,
                )
        except (OSError, serial_exception, ReadOnlyProbeError) as error:
            last_connection_error = error
            time.sleep(0.25)
            continue

        summary = validate_runtime_profile(
            profile,
            result.heartbeat,
            result.actuator,
        )
        return port_name, result, summary

    detail = (
        f": {last_connection_error}"
        if last_connection_error is not None
        else ""
    )
    raise ProfileVerificationError(
        f"AMBAR USB did not become available within {reconnect_seconds:.1f} s{detail}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read-only verification of a flashed AMBAR firmware profile"
    )
    parser.add_argument(
        "--profile",
        choices=("Normal", "ContinuousHil", "VariableHil"),
        required=True,
    )
    parser.add_argument("--port", help="explicit AMBAR USB COM port")
    parser.add_argument(
        "--reconnect-seconds",
        type=float,
        default=20.0,
        help="bounded wait for USB CDC to re-enumerate after reset",
    )
    parser.add_argument(
        "--listen-seconds",
        type=float,
        default=3.0,
        help="bounded wait for fresh ACK, heartbeat, and actuator packets",
    )
    args = parser.parse_args()

    try:
        serial, list_ports = require_serial()
        port_name, result, summary = verify_connected_profile(
            serial_module=serial,
            list_ports=list_ports,
            profile=args.profile,
            requested_port=args.port,
            reconnect_seconds=args.reconnect_seconds,
            listen_seconds=args.listen_seconds,
        )
    except (ReadOnlyProbeError, ProfileVerificationError, ValueError) as error:
        print(f"Runtime profile verification FAILED: {error}", file=sys.stderr)
        return 2

    print("Runtime profile verification PASS")
    print(f"  Port:            {port_name}")
    print(f"  Profile:         {summary['profile']}")
    print(f"  Feature flags:   0x{summary['feature_flags']:08X}")
    print(
        "  Actuator HIL ID: "
        f"{'set' if summary['continuous_hil_profile'] else 'clear'}"
    )
    print(
        "  Variable HIL ID: "
        f"{'set' if summary['variable_hil_profile'] else 'clear'}"
    )
    print(f"  Fresh packets:   {result.packet_count}")
    print(
        "  USB counters:    "
        f"rx_errors={summary['receive_errors']} "
        f"tx_drops={summary['transmit_drops']}"
    )
    print("  Commands sent:   PING, REQUEST_SNAPSHOT only")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
