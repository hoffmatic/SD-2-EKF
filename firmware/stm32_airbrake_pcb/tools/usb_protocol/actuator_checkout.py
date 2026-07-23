"""Run a guarded, bounded actuator motion checkout over STM32 USB.

The checkout is deliberately separate from the flight replay.  It first proves
USB/feature/driver readiness while energy is off, cancels any old session,
declares the operator-confirmed fully closed current position as software HOME,
moves to a bounded percentage of the three-rotation range, and returns to zero.
Optional host-side staging makes the movement observable as a sequence of small,
completed absolute moves without changing the firmware motion limits. Any error
sends DISARM and SIM_STOP; it never performs a blind cleanup retract.

Two CLI acknowledgements are mandatory for every move, and travel above ten
percent requires a third explicit acknowledgement.  ``XACTUAL`` is internal
ramp position rather than encoder or endstop feedback, so visual observation
and a physical cutoff remain necessary. See ``CODE_GUIDE.md`` [ARCH-5],
[ARCH-8].
"""

from __future__ import annotations

import argparse
import math
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
MAX_CHECKOUT_STAGES = 100
MAX_INTER_STAGE_DWELL_S = 10.0
MAX_OUTBOUND_HOLD_S = 60.0
MONITORED_WAIT_POLL_S = 0.02


def build_argument_parser() -> argparse.ArgumentParser:
    """Define the three independent operator acknowledgements for movement."""

    parser = argparse.ArgumentParser(
        description="Move a small percentage, stop, and retract the presentation actuator"
    )
    parser.add_argument("--port", help="COM port; omit to auto-detect 0483:5740")
    parser.add_argument("--percent", type=float, default=2.0)
    parser.add_argument("--allow-actuator-motion", action="store_true")
    parser.add_argument(
        "--home-at-current-position",
        action="store_true",
        help=(
            "confirm the mechanism is manually fully closed and authorize "
            "software HOME by setting the current TMC ramp position to zero"
        ),
    )
    parser.add_argument(
        "--allow-more-than-10-percent",
        action="store_true",
        help="separate acknowledgement required for checkout travel above 10 percent",
    )
    parser.add_argument(
        "--stage-percent",
        type=float,
        help=(
            "maximum percentage points of full travel per completed host-side "
            "stage; mutually exclusive with --stage-count"
        ),
    )
    parser.add_argument(
        "--stage-count",
        type=int,
        help=(
            "number of equal completed host-side stages to the outbound target; "
            "mutually exclusive with --stage-percent"
        ),
    )
    parser.add_argument(
        "--inter-stage-dwell-s",
        type=float,
        default=0.0,
        help="pause between completed stages (zero or 0.1..10 seconds)",
    )
    parser.add_argument(
        "--outbound-hold-s",
        type=float,
        default=0.0,
        help=(
            "hold energy-off at the outbound target before returning "
            "(zero or 0.1..60 seconds)"
        ),
    )
    return parser


def _outbound_stage_targets(
    target_steps: int,
    *,
    stage_percent: float | None,
    stage_count: int | None,
) -> list[int]:
    """Return monotonic absolute targets ending exactly at ``target_steps``."""

    if target_steps == 0:
        raise ReplayError("checkout target rounded to zero steps")
    if stage_percent is None and stage_count is None:
        return [target_steps]

    direction = -1 if target_steps < 0 else 1
    magnitude = abs(target_steps)
    magnitudes: list[int]

    if stage_count is not None:
        magnitudes = [round(magnitude * index / stage_count)
                      for index in range(1, stage_count + 1)]
    else:
        assert stage_percent is not None
        increment = round(PRESENTATION_FULL_TRAVEL_STEPS * stage_percent / 100.0)
        magnitudes = list(range(increment, magnitude, increment))
        # Do not create a final stage whose requested change is already within
        # the firmware's completion tolerance; merge that remainder instead.
        if magnitudes and magnitude - magnitudes[-1] <= ACTUATOR_POSITION_TOLERANCE_STEPS:
            magnitudes.pop()
        magnitudes.append(magnitude)

    if any(current <= previous for previous, current in zip([0, *magnitudes], magnitudes)):
        raise ReplayError("checkout stages must increase by at least one motor count")
    return [direction * value for value in magnitudes]


def _command_staged_target(
    port,
    monitor: PacketMonitor,
    sequence: SequenceCounter,
    *,
    target_steps: int,
    description: str,
) -> dict[str, int]:
    """Command one absolute target and prove it stopped safely at that target."""

    # Drain anything that arrived after the previous completed target. Never
    # issue the next target from a stale apparently-safe actuator report.
    monitor.poll(port)
    _enforce_actuator_motion(monitor)
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
        and abs(int(status["target_steps"]) - target_steps)
        <= ACTUATOR_POSITION_TOLERANCE_STEPS
        and abs(int(status["actual_steps"]) - target_steps)
        <= ACTUATOR_POSITION_TOLERANCE_STEPS,
        after_count=count_before_move,
        description=description,
        timeout_s=8.5,
    )
    _enforce_actuator_motion(monitor)
    return reached


def _monitored_energy_off_wait(
    port,
    monitor: PacketMonitor,
    *,
    duration_s: float,
    description: str,
    expected_steps: int,
) -> None:
    """Drain telemetry and enforce safe stopped geometry throughout a pause."""

    if duration_s <= 0.0:
        return
    deadline = monitor.clock.now() + duration_s
    while True:
        monitor.poll(port)
        try:
            _enforce_actuator_motion(monitor)
        except ReplayError as exc:
            raise ReplayError(f"unsafe actuator state while {description}: {exc}") from exc
        assert monitor.actuator is not None
        if (
            abs(int(monitor.actuator["target_steps"]) - expected_steps)
            > ACTUATOR_POSITION_TOLERANCE_STEPS
            or abs(int(monitor.actuator["actual_steps"]) - expected_steps)
            > ACTUATOR_POSITION_TOLERANCE_STEPS
        ):
            raise ReplayError(
                f"actuator left the stopped target while {description}: "
                f"expected={expected_steps} status={monitor.actuator}"
            )
        remaining_s = deadline - monitor.clock.now()
        if remaining_s <= 0.0:
            return
        monitor.clock.sleep(min(MONITORED_WAIT_POLL_S, remaining_s))


def run_checkout(
    *,
    port_name: str | None,
    percent: float,
    stage_percent: float | None = None,
    stage_count: int | None = None,
    inter_stage_dwell_s: float = 0.0,
    outbound_hold_s: float = 0.0,
) -> None:
    """Execute HOME -> bounded move -> retract with continuous status checks."""

    serial, list_ports = _require_serial()
    selected_port = _choose_port(list_ports, port_name)
    sequence = SequenceCounter()
    monitor = PacketMonitor()
    cleanup_required = True

    print(f"Opening {selected_port}; LOW-TRAVEL MOTOR CHECKOUT ENABLED.")
    print(
        "CMD_HOME will declare the manually fully closed current position as "
        "XACTUAL=0. It will not seek a switch."
    )

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
            # An unhomed image cannot report plausible software geometry yet.
            # All build, driver, fault, and energy-off gates still apply here;
            # geometry becomes mandatory immediately after CMD_HOME.
            _enforce_actuator_motion(monitor, require_geometry=False)

            # Phase 2: the explicit CLI acknowledgement asserts that the
            # operator manually placed the mechanism fully closed. CMD_HOME
            # records that current TMC ramp position as software zero.
            count_before_home = monitor.actuator_count
            home = _send_command(port, sequence, CMD_HOME)
            _wait_for_ack(port, monitor, home, CMD_HOME)
            _wait_for_actuator(
                port,
                monitor,
                lambda status: (
                    (int(status["flags"]) & ACTUATOR_FLAG_HOMED) != 0
                    and bool(
                        status.get(
                            "software_home_active",
                            status.get("home_active"),
                        )
                    )
                    and not bool(
                        status.get(
                            "software_full_active",
                            status.get("full_active"),
                        )
                    )
                    and abs(int(status.get("actual_steps", 0)))
                    <= ACTUATOR_POSITION_TOLERANCE_STEPS
                ),
                after_count=count_before_home,
                description="confirm software HOME at XACTUAL zero",
                timeout_s=1.5,
            )
            _enforce_actuator_motion(monitor)

            # Phase 3: convert percentage into the firmware's signed geometry.
            assert monitor.heartbeat is not None
            sign = -1 if (
                int(monitor.heartbeat["feature_flags"]) & FEATURE_DIRECTION_INVERTED
            ) else 1
            target_steps = sign * round(PRESENTATION_FULL_TRAVEL_STEPS * percent / 100.0)
            outbound_targets = _outbound_stage_targets(
                target_steps,
                stage_percent=stage_percent,
                stage_count=stage_count,
            )
            reached: dict[str, int] | None = None
            for index, staged_target in enumerate(outbound_targets, start=1):
                reached = _command_staged_target(
                    port,
                    monitor,
                    sequence,
                    target_steps=staged_target,
                    description=(
                        f"reach outbound stage {index}/{len(outbound_targets)} "
                        "and stop"
                    ),
                )
                print(
                    f"Outbound stage {index}/{len(outbound_targets)} reached: "
                    f"requested={staged_target} actual={reached['actual_steps']}."
                )
                if index < len(outbound_targets) and inter_stage_dwell_s > 0.0:
                    _monitored_energy_off_wait(
                        port,
                        monitor,
                        duration_s=inter_stage_dwell_s,
                        description="waiting between outbound stages",
                        expected_steps=staged_target,
                    )

            assert reached is not None
            print(
                f"Checkout target reached: requested={target_steps} "
                f"actual={reached['actual_steps']}."
            )
            if outbound_hold_s > 0.0:
                print(f"Holding energy-off at the outbound target for {outbound_hold_s:.2f}s.")
                _monitored_energy_off_wait(
                    port,
                    monitor,
                    duration_s=outbound_hold_s,
                    description="holding at the outbound target",
                    expected_steps=target_steps,
                )

            # Phase 4: mirror all intermediate outbound stages on the return.
            # The final short leg still uses the explicit RETRACT command so the
            # established HOME completion path and final-zero checks are kept.
            return_targets = list(reversed(outbound_targets[:-1]))
            for index, staged_target in enumerate(return_targets, start=1):
                returned = _command_staged_target(
                    port,
                    monitor,
                    sequence,
                    target_steps=staged_target,
                    description=(
                        f"reach return stage {index}/{len(return_targets)} and stop"
                    ),
                )
                print(
                    f"Return stage {index}/{len(return_targets)} reached: "
                    f"requested={staged_target} actual={returned['actual_steps']}."
                )
                if inter_stage_dwell_s > 0.0:
                    _monitored_energy_off_wait(
                        port,
                        monitor,
                        duration_s=inter_stage_dwell_s,
                        description="waiting between return stages",
                        expected_steps=staged_target,
                    )

            # Finish at software zero and require driver-off status.
            monitor.poll(port)
            _enforce_actuator_motion(monitor)
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
                and abs(int(status["target_steps"]))
                <= ACTUATOR_POSITION_TOLERANCE_STEPS
                and abs(int(status["actual_steps"]))
                <= ACTUATOR_POSITION_TOLERANCE_STEPS,
                after_count=count_before_retract,
                description="return to XACTUAL zero and stop",
                timeout_s=8.5,
            )
            _enforce_actuator_motion(monitor)
            print(
                "Software-zero return complete and driver off: "
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
                "checkout requires --allow-actuator-motion and "
                "--home-at-current-position after manually fully closing the "
                "mechanism"
            )
        if not 0.1 <= args.percent <= 100.0:
            raise ReplayError("--percent must be within 0.1..100")
        if args.percent > 10.0 and not args.allow_more_than_10_percent:
            raise ReplayError(
                "checkout above 10% requires --allow-more-than-10-percent"
            )
        if args.stage_percent is not None and args.stage_count is not None:
            raise ReplayError("use only one of --stage-percent or --stage-count")
        if args.stage_percent is not None:
            if not 0.1 <= args.stage_percent <= args.percent:
                raise ReplayError("--stage-percent must be within 0.1..--percent")
            required_stages = math.ceil(args.percent / args.stage_percent)
            if required_stages > MAX_CHECKOUT_STAGES:
                raise ReplayError(
                    f"staged checkout is limited to {MAX_CHECKOUT_STAGES} stages"
                )
        if args.stage_count is not None:
            if not 1 <= args.stage_count <= MAX_CHECKOUT_STAGES:
                raise ReplayError(
                    f"--stage-count must be within 1..{MAX_CHECKOUT_STAGES}"
                )
            target_magnitude = round(
                PRESENTATION_FULL_TRAVEL_STEPS * args.percent / 100.0
            )
            maximum_meaningful_stages = max(
                1,
                target_magnitude // (ACTUATOR_POSITION_TOLERANCE_STEPS + 1),
            )
            if args.stage_count > maximum_meaningful_stages:
                raise ReplayError(
                    "--stage-count creates increments within the position tolerance"
                )
        if not (
            args.inter_stage_dwell_s == 0.0
            or 0.1 <= args.inter_stage_dwell_s <= MAX_INTER_STAGE_DWELL_S
        ):
            raise ReplayError(
                "--inter-stage-dwell-s must be zero or within "
                f"0.1..{MAX_INTER_STAGE_DWELL_S:g}"
            )
        if (
            args.inter_stage_dwell_s > 0.0
            and args.stage_percent is None
            and args.stage_count is None
        ):
            raise ReplayError(
                "--inter-stage-dwell-s requires --stage-percent or --stage-count"
            )
        if not (
            args.outbound_hold_s == 0.0
            or 0.1 <= args.outbound_hold_s <= MAX_OUTBOUND_HOLD_S
        ):
            raise ReplayError(
                "--outbound-hold-s must be zero or within "
                f"0.1..{MAX_OUTBOUND_HOLD_S:g}"
            )
        run_checkout(
            port_name=args.port,
            percent=args.percent,
            stage_percent=args.stage_percent,
            stage_count=args.stage_count,
            inter_stage_dwell_s=args.inter_stage_dwell_s,
            outbound_hold_s=args.outbound_hold_s,
        )
        return 0
    except (ReplayError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Interrupted; requested DISARM and SIM_STOP.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
