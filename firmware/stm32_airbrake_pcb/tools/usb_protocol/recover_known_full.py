"""Perform one guarded recovery stroke from an operator-confirmed FULL position.

This utility exists only for the lost-ramp-reference case: the mechanism is
physically fully extended, while a rebooted TMC5240 reports XACTUAL/XTARGET zero
and the firmware has no software HOME.  The matching CONTINUOUS_HIL-only
firmware command first assigns the configured FULL coordinate with the driver
disabled, then uses the normal bounded RETRACT path to return to HOME.

XACTUAL is not an encoder.  The operator confirmation, clear mechanism, and
independent power cutoff remain mandatory.
"""

from __future__ import annotations

import argparse
import sys
import time

from rocket_protocol import (
    CMD_PING,
    CMD_RECOVER_KNOWN_FULL_RETRACT,
    CMD_REQUEST_SNAPSHOT,
    CMD_SET_ARMED,
    CMD_SIM_STOP,
    RECOVER_KNOWN_FULL_MAGIC,
    command_frame,
)
from replay_openrocket import (
    ACTUATOR_FLAG_DRIVER_ENABLED,
    ACTUATOR_FLAG_ESTOP,
    ACTUATOR_FLAG_HOMED,
    ACTUATOR_FLAG_MANUAL_PENDING,
    ACTUATOR_POSITION_TOLERANCE_STEPS,
    ACTUATOR_STATE_ESTOP,
    ACTUATOR_STATE_FAULT,
    ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE,
    ACTUATOR_STATUS_GEOMETRY_PLAUSIBLE,
    ACTUATOR_STATUS_SOFTWARE_FULL_ACTIVE,
    ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE,
    PacketMonitor,
    ReplayError,
    SequenceCounter,
    _choose_port,
    _collect_preflight_status,
    _enforce_actuator_motion,
    _require_serial,
    _send_command,
    _wait_for_ack,
    _wait_for_actuator,
)


PRESENTATION_FULL_TRAVEL_STEPS = 200 * 256 * 3
RECOVERY_TIMEOUT_S = 12.0


def _require_lost_reference_at_known_full(status: dict[str, object]) -> None:
    """Reject every state except the reset-zero/no-HOME recovery condition."""

    flags = int(status.get("flags", 0))
    reserved = int(status.get("reserved", 0))
    target = int(status.get("target_steps", 0))
    actual = int(status.get("actual_steps", 0))

    if not reserved & ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE:
        raise ReplayError("known-FULL recovery requires CONTINUOUS_HIL firmware")
    if flags & ACTUATOR_FLAG_HOMED:
        raise ReplayError(
            "software HOME still exists; use the normal RETRACT command instead"
        )
    if reserved & (
        ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE
        | ACTUATOR_STATUS_SOFTWARE_FULL_ACTIVE
    ):
        raise ReplayError("software endpoint state is not the lost-reference condition")
    if abs(target) > ACTUATOR_POSITION_TOLERANCE_STEPS or abs(actual) > ACTUATOR_POSITION_TOLERANCE_STEPS:
        raise ReplayError(
            "recovery requires reset-zero XACTUAL/XTARGET; "
            f"target={target} actual={actual}"
        )


def _recovery_complete(status: dict[str, object]) -> bool:
    flags = int(status.get("flags", 0))
    reserved = int(status.get("reserved", 0))
    return (
        flags & (ACTUATOR_FLAG_DRIVER_ENABLED | ACTUATOR_FLAG_MANUAL_PENDING) == 0
        and flags & ACTUATOR_FLAG_HOMED != 0
        and reserved & ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE != 0
        and reserved & ACTUATOR_STATUS_GEOMETRY_PLAUSIBLE != 0
        and reserved & ACTUATOR_STATUS_SOFTWARE_HOME_ACTIVE != 0
        and reserved & ACTUATOR_STATUS_SOFTWARE_FULL_ACTIVE == 0
        and abs(int(status.get("target_steps", 0)))
        <= ACTUATOR_POSITION_TOLERANCE_STEPS
        and abs(int(status.get("actual_steps", 0)))
        <= ACTUATOR_POSITION_TOLERANCE_STEPS
    )


def recover_known_full(port_name: str | None) -> None:
    serial, list_ports = _require_serial()
    selected_port = _choose_port(list_ports, port_name)
    sequence = SequenceCounter()
    monitor = PacketMonitor()
    cleanup_required = True

    print(f"Opening {selected_port}; KNOWN-FULL RECOVERY STROKE ENABLED.")
    print(
        f"The command will assign XACTUAL={PRESENTATION_FULL_TRAVEL_STEPS} with the driver off, then "
        "command one bounded return to zero."
    )

    with serial.Serial(selected_port, 115200, timeout=0.0, write_timeout=1.0) as port:
        port.reset_input_buffer()
        try:
            ping = _send_command(port, sequence, CMD_PING)
            _wait_for_ack(port, monitor, ping, CMD_PING)
            snapshot = _send_command(port, sequence, CMD_REQUEST_SNAPSHOT)
            _wait_for_ack(port, monitor, snapshot, CMD_REQUEST_SNAPSHOT)
            _collect_preflight_status(port, monitor)

            before_stop = monitor.actuator_count
            disarm = _send_command(port, sequence, CMD_SET_ARMED, b"\x00")
            _wait_for_ack(port, monitor, disarm, CMD_SET_ARMED)
            sim_stop = _send_command(port, sequence, CMD_SIM_STOP)
            _wait_for_ack(port, monitor, sim_stop, CMD_SIM_STOP)
            stopped = _wait_for_actuator(
                port,
                monitor,
                lambda _status: True,
                after_count=before_stop,
                description="report stopped lost-reference state",
                timeout_s=1.5,
            )
            _enforce_actuator_motion(monitor, require_geometry=False)
            _require_lost_reference_at_known_full(stopped)

            before_recovery = monitor.actuator_count
            command_sequence = _send_command(
                port,
                sequence,
                CMD_RECOVER_KNOWN_FULL_RETRACT,
                RECOVER_KNOWN_FULL_MAGIC,
            )
            _wait_for_ack(
                port,
                monitor,
                command_sequence,
                CMD_RECOVER_KNOWN_FULL_RETRACT,
            )

            deadline = monitor.clock.now() + RECOVERY_TIMEOUT_S
            maximum_observed_position = 0
            last_report_count = before_recovery
            final_status: dict[str, object] | None = None
            while monitor.clock.now() < deadline:
                monitor.poll(port)
                if monitor.actuator_count > last_report_count and monitor.actuator is not None:
                    last_report_count = monitor.actuator_count
                    status = monitor.actuator
                    flags = int(status.get("flags", 0))
                    machine_state = int(status.get("machine_state", 0))
                    if flags & ACTUATOR_FLAG_ESTOP or machine_state in (
                        ACTUATOR_STATE_FAULT,
                        ACTUATOR_STATE_ESTOP,
                    ):
                        raise ReplayError(f"actuator faulted during recovery: {status}")
                    if not int(status.get("reserved", 0)) & ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE:
                        raise ReplayError("CONTINUOUS_HIL identity disappeared during recovery")
                    maximum_observed_position = max(
                        maximum_observed_position,
                        abs(int(status.get("actual_steps", 0))),
                    )
                    if _recovery_complete(status):
                        final_status = status
                        break
                monitor.clock.sleep(0.005)

            if final_status is None:
                raise ReplayError("timeout waiting for the full recovery stroke to reach HOME")
            if maximum_observed_position < PRESENTATION_FULL_TRAVEL_STEPS // 2:
                raise ReplayError(
                    "HOME was reported without observing the commanded full-stroke coordinate"
                )

            _enforce_actuator_motion(monitor)
            print(
                "RECOVERY COMPLETE: software HOME, XACTUAL=0, target=0, driver off; "
                f"maximum observed XACTUAL={maximum_observed_position}."
            )
            cleanup_required = False
        finally:
            if cleanup_required:
                try:
                    port.write(command_frame(sequence.take(), CMD_SET_ARMED, b"\x00"))
                    port.write(command_frame(sequence.take(), CMD_SIM_STOP))
                    time.sleep(0.1)
                except Exception:
                    pass


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Retract once from an operator-confirmed physical FULL position"
    )
    parser.add_argument("--port", help="COM port; omit to auto-detect 0483:5740")
    parser.add_argument("--allow-actuator-motion", action="store_true")
    parser.add_argument("--confirm-known-fully-extended", action="store_true")
    args = parser.parse_args()

    try:
        if not args.allow_actuator_motion or not args.confirm_known_fully_extended:
            raise ReplayError(
                "recovery requires --allow-actuator-motion and "
                "--confirm-known-fully-extended"
            )
        recover_known_full(args.port)
        return 0
    except (ReplayError, OSError, ValueError) as exc:
        print(f"RECOVERY FAILED: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
