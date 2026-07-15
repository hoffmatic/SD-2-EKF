"""Run a guarded, bounded actuator motion checkout over STM32 USB.

The checkout is deliberately separate from the flight replay.  It first proves
USB/feature/driver readiness while energy is off, cancels any old session,
declares the operator-confirmed current position HOME, moves to a bounded
percentage of the three-rotation presentation range, and retracts to HOME.  Any
error sends DISARM and SIM_STOP; it never performs a blind cleanup retract.

Two CLI acknowledgements are mandatory for every move, and travel above ten
percent requires a third explicit acknowledgement.  ``XACTUAL`` is internal
ramp position rather than encoder feedback, so visual observation and a
physical cutoff remain necessary.  See ``CODE_GUIDE.md`` [ARCH-5], [ARCH-8].
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

from rocket_protocol import (
    CMD_BENCH_MOVE_STEPS,
    CMD_HOME,
    CMD_PING,
    CMD_REQUEST_SNAPSHOT,
    CMD_RETRACT,
    CMD_SET_ARMED,
    CMD_SIM_STOP,
    command_frame,
)
from replay_openrocket import (
    ACTUATOR_FLAG_DRIVER_ENABLED,
    ACTUATOR_FLAG_HOMED,
    ACTUATOR_FLAG_MANUAL_PENDING,
    ACTUATOR_POSITION_TOLERANCE_STEPS,
    FEATURE_PRESENTATION_MOTION,
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
FEATURE_DIRECTION_INVERTED = 1 << 15


def build_argument_parser() -> argparse.ArgumentParser:
    """Define the three independent operator acknowledgements for movement."""

    parser = argparse.ArgumentParser(
        description="Move a small percentage, stop, and retract the presentation actuator"
    )
    parser.add_argument("--port", help="COM port; omit to auto-detect 0483:5740")
    parser.add_argument("--percent", type=float, default=2.0)
    parser.add_argument("--allow-actuator-motion", action="store_true")
    parser.add_argument("--home-at-current-position", action="store_true")
    parser.add_argument(
        "--allow-more-than-10-percent",
        action="store_true",
        help="separate acknowledgement required for checkout travel above 10 percent",
    )
    return parser


def run_checkout(*, port_name: str | None, percent: float) -> None:
    """Execute HOME -> bounded move -> retract with continuous status checks."""

    serial, list_ports = _require_serial()
    selected_port = _choose_port(list_ports, port_name)
    sequence = SequenceCounter()
    monitor = PacketMonitor()
    cleanup_required = True

    print(f"Opening {selected_port}; LOW-TRAVEL MOTOR CHECKOUT ENABLED.")
    print("The current physical position will be declared fully retracted HOME.")

    with serial.Serial(selected_port, 115200, timeout=0.0, write_timeout=1.0) as port:
        port.reset_input_buffer()
        try:
            # Phase 1: prove protocol and capture fresh, energy-off status.
            ping = _send_command(port, sequence, CMD_PING)
            _wait_for_ack(port, monitor, ping, CMD_PING)
            snapshot = _send_command(port, sequence, CMD_REQUEST_SNAPSHOT)
            _wait_for_ack(port, monitor, snapshot, CMD_REQUEST_SNAPSHOT)
            _collect_preflight_status(port, monitor)

            count_before_stop = monitor.actuator_count
            disarm = _send_command(port, sequence, CMD_SET_ARMED, b"\x00")
            _wait_for_ack(port, monitor, disarm, CMD_SET_ARMED)
            sim_stop = _send_command(port, sequence, CMD_SIM_STOP)
            _wait_for_ack(port, monitor, sim_stop, CMD_SIM_STOP)
            _wait_for_actuator(
                port,
                monitor,
                lambda _status: True,
                after_count=count_before_stop,
                description="report its stopped preflight state",
                timeout_s=1.5,
            )
            _enforce_actuator_motion(monitor)

            # Phase 2: establish the only position reference available on this
            # presentation mechanism. The operator has already confirmed that
            # the physical mechanism is fully retracted.
            count_before_home = monitor.actuator_count
            home = _send_command(port, sequence, CMD_HOME)
            _wait_for_ack(port, monitor, home, CMD_HOME)
            _wait_for_actuator(
                port,
                monitor,
                lambda status: (int(status["flags"]) & ACTUATOR_FLAG_HOMED) != 0,
                after_count=count_before_home,
                description="confirm HOME",
                timeout_s=1.5,
            )

            # Phase 3: convert percentage into the firmware's signed geometry.
            assert monitor.heartbeat is not None
            sign = -1 if (
                int(monitor.heartbeat["feature_flags"]) & FEATURE_DIRECTION_INVERTED
            ) else 1
            target_steps = sign * round(PRESENTATION_FULL_TRAVEL_STEPS * percent / 100.0)
            count_before_move = monitor.actuator_count
            move = _send_command(
                port,
                sequence,
                CMD_BENCH_MOVE_STEPS,
                struct.pack("<i", target_steps),
            )
            _wait_for_ack(port, monitor, move, CMD_BENCH_MOVE_STEPS)
            reached = _wait_for_actuator(
                port,
                monitor,
                lambda status: (
                    int(status["flags"])
                    & (ACTUATOR_FLAG_DRIVER_ENABLED | ACTUATOR_FLAG_MANUAL_PENDING)
                ) == 0
                and abs(int(status["actual_steps"]) - target_steps)
                <= ACTUATOR_POSITION_TOLERANCE_STEPS,
                after_count=count_before_move,
                description=f"reach the {percent:.2f}% checkout target and stop",
                timeout_s=8.5,
            )
            print(
                f"Checkout target reached: requested={target_steps} "
                f"actual={reached['actual_steps']}."
            )

            # Phase 4: return to HOME and require the driver-off status bits.
            count_before_retract = monitor.actuator_count
            retract = _send_command(port, sequence, CMD_RETRACT)
            _wait_for_ack(port, monitor, retract, CMD_RETRACT)
            retracted = _wait_for_actuator(
                port,
                monitor,
                lambda status: (
                    int(status["flags"])
                    & (ACTUATOR_FLAG_DRIVER_ENABLED | ACTUATOR_FLAG_MANUAL_PENDING)
                ) == 0
                and abs(int(status["target_steps"]) - int(status["actual_steps"]))
                <= ACTUATOR_POSITION_TOLERANCE_STEPS,
                after_count=count_before_retract,
                description="retract to HOME and stop",
                timeout_s=8.5,
            )
            print(
                "Retract complete and driver off: "
                f"target={retracted['target_steps']} actual={retracted['actual_steps']}"
            )
            cleanup_required = False
        finally:
            if cleanup_required:
                # Failure cleanup removes authorization/energy. It intentionally
                # does not assume that an unknown mechanism can retract safely.
                try:
                    port.write(command_frame(sequence.take(), CMD_SET_ARMED, b"\x00"))
                    port.write(command_frame(sequence.take(), CMD_SIM_STOP))
                    time.sleep(0.1)
                except Exception:
                    pass


def main() -> int:
    """Validate explicit acknowledgements before opening any serial port."""

    args = build_argument_parser().parse_args()
    try:
        if not args.allow_actuator_motion or not args.home_at_current_position:
            raise ReplayError(
                "checkout requires --allow-actuator-motion and --home-at-current-position"
            )
        if not 0.1 <= args.percent <= 100.0:
            raise ReplayError("--percent must be within 0.1..100")
        if args.percent > 10.0 and not args.allow_more_than_10_percent:
            raise ReplayError(
                "checkout above 10% requires --allow-more-than-10-percent"
            )
        run_checkout(port_name=args.port, percent=args.percent)
        return 0
    except (ReplayError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Interrupted; requested DISARM and SIM_STOP.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
