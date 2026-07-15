"""Deterministic host tests for the guarded low-travel actuator checkout.

Fake serial, telemetry, actuator state, and time let the tests cover preflight
gates, movement tracking, timeout/fault cleanup, and final driver-off behavior
without energizing hardware.  The production checkout script remains the COM
owner described by [ARCH-5] and [ARCH-8].
"""

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
import io
import struct
import sys
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "usb_protocol"))

import actuator_checkout as checkout  # noqa: E402
import replay_openrocket as replay  # noqa: E402
import rocket_protocol as protocol  # noqa: E402


# ---------------------------------------------------------------------------
# Deterministic clock and serial-board fakes
# ---------------------------------------------------------------------------


class ActuatorCheckoutTests(unittest.TestCase):
    class _FakeClock:
        def __init__(self) -> None:
            self.now_s = 0.0

        def monotonic(self) -> float:
            return self.now_s

        def sleep(self, seconds: float) -> None:
            # Keep timeout-path tests bounded without changing successful waits.
            self.now_s += max(1.0, seconds)

    class _FakeCheckoutPort:
        def __init__(
            self,
            monitor: replay.PacketMonitor,
            *,
            direction_inverted: bool = False,
            fault_on_move: bool = False,
        ) -> None:
            self.monitor = monitor
            self.fault_on_move = fault_on_move
            self.decoder = protocol.StreamDecoder()
            self.commands: list[int] = []
            self.move_targets: list[int] = []
            self.current_flags = (
                replay.ACTUATOR_FLAG_BUILD_ENABLED
                | replay.ACTUATOR_FLAG_BENCH_ENABLED
                | replay.ACTUATOR_FLAG_DRIVER_OK
                | replay.ACTUATOR_FLAG_CONFIG_VALID
            )
            features = (
                replay.FEATURE_USB_PROTOCOL
                | replay.FEATURE_SIMULATION
                | replay.FEATURE_ACTUATOR
                | replay.FEATURE_BENCH_COMMANDS
                | replay.FEATURE_PRESENTATION_MOTION
            )
            if direction_inverted:
                features |= checkout.FEATURE_DIRECTION_INVERTED
            self.monitor.heartbeat = {"feature_flags": features}
            self._publish_actuator(machine_state=1)

        def __enter__(self) -> "ActuatorCheckoutTests._FakeCheckoutPort":
            return self

        def __exit__(self, exc_type, exc, traceback) -> None:
            return None

        def reset_input_buffer(self) -> None:
            return None

        def read(self, _size: int) -> bytes:
            return b""

        def _publish_actuator(
            self,
            *,
            machine_state: int,
            target_steps: int = 0,
            actual_steps: int = 0,
        ) -> None:
            self.monitor.actuator = {
                "flags": self.current_flags,
                "machine_state": machine_state,
                "target_steps": target_steps,
                "actual_steps": actual_steps,
            }
            self.monitor.actuator_count += 1

        def write(self, data: bytes) -> int:
            packets = self.decoder.feed(bytes(data))
            if len(packets) != 1:
                raise AssertionError("each fake serial write must contain one frame")
            packet = packets[0]
            if packet.packet_type != protocol.PKT_COMMAND:
                raise AssertionError("checkout should send command frames only")

            command = packet.payload[0]
            command_data = packet.payload[2:]
            self.commands.append(command)
            self.monitor.acks[packet.sequence] = {
                "command_sequence": packet.sequence,
                "command": command,
                "result": protocol.ACK_OK,
                "detail": 0,
            }

            if command == protocol.CMD_SET_ARMED and command_data == b"\x00":
                self.current_flags &= ~(
                    replay.ACTUATOR_FLAG_DRIVER_ENABLED
                    | replay.ACTUATOR_FLAG_MANUAL_PENDING
                )
                self._publish_actuator(machine_state=1)
            elif command == protocol.CMD_HOME:
                self.current_flags |= replay.ACTUATOR_FLAG_HOMED
                self._publish_actuator(machine_state=3)
            elif command == protocol.CMD_BENCH_MOVE_STEPS:
                target_steps = struct.unpack("<i", command_data)[0]
                self.move_targets.append(target_steps)
                if self.fault_on_move:
                    self.current_flags |= replay.ACTUATOR_FLAG_ESTOP
                    self._publish_actuator(
                        machine_state=replay.ACTUATOR_STATE_ESTOP,
                        target_steps=target_steps,
                    )
                else:
                    self.current_flags &= ~(
                        replay.ACTUATOR_FLAG_DRIVER_ENABLED
                        | replay.ACTUATOR_FLAG_MANUAL_PENDING
                    )
                    self._publish_actuator(
                        machine_state=3,
                        target_steps=target_steps,
                        actual_steps=target_steps,
                    )
            elif command == protocol.CMD_RETRACT:
                self.current_flags &= ~(
                    replay.ACTUATOR_FLAG_DRIVER_ENABLED
                    | replay.ACTUATOR_FLAG_MANUAL_PENDING
                )
                self._publish_actuator(machine_state=3)

            return len(data)

    # -----------------------------------------------------------------------
    # Production invocation fixtures
    # -----------------------------------------------------------------------

    @staticmethod
    def _invoke_main(*arguments: str) -> tuple[int, mock.Mock, str]:
        error = io.StringIO()
        with (
            mock.patch.object(sys, "argv", ["actuator_checkout.py", *arguments]),
            mock.patch.object(checkout, "run_checkout") as run_checkout,
            redirect_stdout(io.StringIO()),
            redirect_stderr(error),
        ):
            result = checkout.main()
        return result, run_checkout, error.getvalue()

    def _run_fake_checkout(self, port: _FakeCheckoutPort, *, percent: float = 2.0) -> None:
        clock = self._FakeClock()

        class FakeSerialModule:
            @staticmethod
            def Serial(*_args, **_kwargs):
                return port

        with (
            mock.patch.object(
                checkout,
                "_require_serial",
                return_value=(FakeSerialModule, object()),
            ),
            mock.patch.object(checkout, "PacketMonitor", return_value=port.monitor),
            mock.patch.object(checkout.time, "monotonic", side_effect=clock.monotonic),
            mock.patch.object(checkout.time, "sleep", side_effect=clock.sleep),
            redirect_stdout(io.StringIO()),
        ):
            checkout.run_checkout(port_name="COM_TEST", percent=percent)

    # -----------------------------------------------------------------------
    # Operator-gate, success-path, and fault-cleanup regressions
    # -----------------------------------------------------------------------

    def test_cli_requires_both_motion_acknowledgements(self) -> None:
        incomplete_arguments = (
            (),
            ("--allow-actuator-motion",),
            ("--home-at-current-position",),
        )
        for arguments in incomplete_arguments:
            with self.subTest(arguments=arguments):
                result, run_checkout, error = self._invoke_main(*arguments)
                self.assertEqual(result, 2)
                run_checkout.assert_not_called()
                self.assertIn("requires --allow-actuator-motion", error)

        result, run_checkout, error = self._invoke_main(
            "--allow-actuator-motion",
            "--home-at-current-position",
        )
        self.assertEqual(result, 0)
        run_checkout.assert_called_once_with(port_name=None, percent=2.0)
        self.assertEqual(error, "")

    def test_cli_requires_separate_acknowledgement_above_ten_percent(self) -> None:
        required = ("--allow-actuator-motion", "--home-at-current-position")

        result, run_checkout, _error = self._invoke_main(
            *required,
            "--percent",
            "10.01",
        )
        self.assertEqual(result, 2)
        run_checkout.assert_not_called()

        result, run_checkout, error = self._invoke_main(
            *required,
            "--percent",
            "10.01",
            "--allow-more-than-10-percent",
        )
        self.assertEqual(result, 0)
        run_checkout.assert_called_once_with(port_name=None, percent=10.01)
        self.assertEqual(error, "")

    def test_direction_bit_controls_sign_and_success_command_order(self) -> None:
        expected_commands = [
            protocol.CMD_PING,
            protocol.CMD_REQUEST_SNAPSHOT,
            protocol.CMD_SET_ARMED,
            protocol.CMD_SIM_STOP,
            protocol.CMD_HOME,
            protocol.CMD_BENCH_MOVE_STEPS,
            protocol.CMD_RETRACT,
        ]
        expected_magnitude = round(
            checkout.PRESENTATION_FULL_TRAVEL_STEPS * 2.0 / 100.0
        )

        for inverted, expected_target in (
            (False, expected_magnitude),
            (True, -expected_magnitude),
        ):
            with self.subTest(direction_inverted=inverted):
                monitor = replay.PacketMonitor()
                port = self._FakeCheckoutPort(
                    monitor,
                    direction_inverted=inverted,
                )
                self._run_fake_checkout(port)
                self.assertEqual(port.move_targets, [expected_target])
                self.assertEqual(port.commands, expected_commands)

    def test_fault_cleanup_disarms_and_stops_without_blind_retract(self) -> None:
        monitor = replay.PacketMonitor()
        port = self._FakeCheckoutPort(monitor, fault_on_move=True)

        with self.assertRaises(replay.ReplayError):
            self._run_fake_checkout(port)

        self.assertEqual(
            port.commands,
            [
                protocol.CMD_PING,
                protocol.CMD_REQUEST_SNAPSHOT,
                protocol.CMD_SET_ARMED,
                protocol.CMD_SIM_STOP,
                protocol.CMD_HOME,
                protocol.CMD_BENCH_MOVE_STEPS,
                protocol.CMD_SET_ARMED,
                protocol.CMD_SIM_STOP,
            ],
        )
        self.assertEqual(
            port.commands[-2:],
            [protocol.CMD_SET_ARMED, protocol.CMD_SIM_STOP],
        )
        self.assertNotIn(protocol.CMD_RETRACT, port.commands)


if __name__ == "__main__":
    unittest.main()
