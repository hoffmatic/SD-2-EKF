"""Host regression tests for OpenRocket replay and presentation evidence.

Coverage follows the production boundaries in [ARCH-3], [ARCH-6], and [ARCH-8]:
CSV normalization, guarded motion preflight, absolute USB scheduling, safe fault
cleanup, persistent run verdicts, and the localhost-only GUI mirror. Hardware
motion is represented by a deterministic fake board; these tests prove command
and evidence behavior without claiming encoder-verified physical travel.

The fake board publishes the firmware's 50 ms telemetry and 100 ms actuator
cadences on an injected clock. Tests may add deterministic host oversleep without
weakening production freshness or deadline thresholds.
"""

from __future__ import annotations

from contextlib import redirect_stdout
from pathlib import Path
import io
import json
import struct
import sys
import tempfile
import time
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools" / "usb_protocol"))

import replay_openrocket as replay  # noqa: E402
import replay_reporting as reporting  # noqa: E402
import rocket_protocol as protocol  # noqa: E402


REAL_OPENROCKET_CSV = ROOT / "SimulationData" / "Source" / "26-07-10_Openrocket_Run.csv"

FEATURE_ACTUATOR = 1 << 3
FEATURE_BENCH_COMMANDS = 1 << 4
FEATURE_USB_PROTOCOL = 1 << 10
FEATURE_SIMULATION = 1 << 11
FEATURE_PRESENTATION_MOTION = 1 << 14

ACTUATOR_FLAG_BUILD_ENABLED = 1 << 0
ACTUATOR_FLAG_BENCH_ENABLED = 1 << 1
ACTUATOR_FLAG_HOMED = 1 << 2
ACTUATOR_FLAG_DRIVER_OK = 1 << 3
ACTUATOR_FLAG_DRIVER_ENABLED = 1 << 4
ACTUATOR_FLAG_ESTOP = 1 << 5
ACTUATOR_FLAG_CONFIG_VALID = 1 << 6
ACTUATOR_FLAG_MANUAL_PENDING = 1 << 7


# ---------------------------------------------------------------------------
# Dataset parsing, normalization, metadata, and supplied-profile tests
# ---------------------------------------------------------------------------


class OpenRocketReplayTests(unittest.TestCase):
    def _load_text(self, text: str) -> replay.OpenRocketDataset:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        path = Path(temporary.name) / "openrocket.csv"
        path.write_text(text, encoding="utf-8")
        return replay.load_openrocket_csv(path)

    def test_parser_accepts_commented_header_and_collects_events(self) -> None:
        dataset = self._load_text(
            "\n".join(
                (
                    "# Example OpenRocket export",
                    "# Time (s),Altitude (m),Vertical velocity (m/s),Vertical acceleration (m/s^2)",
                    "# Event LIFTOFF occurred at t=0.25 seconds",
                    "0,100,0,0",
                    "0.25,101,8,32",
                    "# Event APOGEE occurred at t=1.5 seconds",
                    "1.5,120,0,-9.8",
                )
            )
            + "\n"
        )

        self.assertEqual(dataset.row_count, 3)
        self.assertEqual(dataset.headers[0], "Time (s)")
        self.assertEqual(dataset.vertical_velocity_index, 2)
        self.assertEqual(dataset.vertical_acceleration_index, 3)
        self.assertEqual(dataset.event_times("liftoff"), (0.25,))
        self.assertEqual(dataset.event_times("APOGEE"), (1.5,))
        self.assertEqual(dataset.values(dataset.altitude_index), (100.0, 101.0, 120.0))

    def test_nonmonotonic_timestamps_are_rejected(self) -> None:
        with self.assertRaisesRegex(replay.ReplayError, "strictly increasing"):
            self._load_text(
                "\n".join(
                    (
                        "Time (s),Altitude (m),Total velocity (m/s),Total acceleration (m/s^2)",
                        "0,0,0,0",
                        "1,10,10,2",
                        "0.5,12,8,2",
                    )
                )
                + "\n"
            )

    def test_non_si_units_are_rejected(self) -> None:
        with self.assertRaisesRegex(replay.ReplayError, "unsupported or missing units"):
            self._load_text(
                "\n".join(
                    (
                        "Time (s),Altitude (ft),Vertical velocity (ft/s),"
                        "Vertical acceleration (ft/s^2)",
                        "0,0,0,0",
                        "1,10,10,1",
                    )
                )
                + "\n"
            )

    def test_total_channels_are_converted_to_signed_vertical_kinematics(self) -> None:
        lines = [
            "# Time (s),Altitude (m),Total velocity (m/s),Total acceleration (m/s^2)",
            "# Event LIFTOFF occurred at t=0.1 seconds",
        ]
        for index in range(17):
            time_s = index * 0.25
            altitude_m = 4.0 - (time_s - 2.0) ** 2
            speed_mps = abs(4.0 - 2.0 * time_s)
            lines.append(f"{time_s},{altitude_m},{speed_mps},2")

        dataset = self._load_text("\n".join(lines) + "\n")
        profile = replay.build_replay_profile(
            dataset,
            rate_hz=20.0,
            prepad_s=0.5,
            stop_s=3.5,
            derivative_points=9,
        )

        by_source_time = {
            round(sample.source_time_s, 6): sample
            for sample in profile.samples
            if sample.replay_time_s >= profile.prepad_s
        }
        ascending = by_source_time[1.0]
        descending = by_source_time[3.0]

        self.assertTrue(profile.uses_derived_vertical)
        self.assertIn("signed Total acceleration", profile.vertical_source)
        self.assertGreater(ascending.vertical_velocity_mps, 0.0)
        self.assertLess(descending.vertical_velocity_mps, 0.0)
        self.assertAlmostEqual(ascending.vertical_acceleration_mps2, -2.0, places=6)
        self.assertAlmostEqual(descending.vertical_acceleration_mps2, -2.0, places=6)
        self.assertTrue(any("No APOGEE" in warning for warning in profile.warnings))

    def test_prototype_counts_map_three_rotations_to_153600_counts(self) -> None:
        full_rotations, full_counts = replay.prototype_counts(
            100.0,
            rotations=3.0,
            full_steps_per_revolution=200,
            microsteps=256,
            gear_ratio=1.0,
        )
        half_rotations, half_counts = replay.prototype_counts(
            50.0,
            rotations=3.0,
            full_steps_per_revolution=200,
            microsteps=256,
            gear_ratio=1.0,
        )

        self.assertEqual((full_rotations, full_counts), (3.0, 153600))
        self.assertEqual((half_rotations, half_counts), (1.5, 76800))

    def test_motion_profile_is_locked_to_firmware_travel(self) -> None:
        replay.validate_presentation_motion_profile(
            rotations=3.0,
            full_steps_per_revolution=200,
            microsteps=256,
            gear_ratio=1.0,
        )
        with self.assertRaisesRegex(replay.ReplayError, "153600 counts"):
            replay.validate_presentation_motion_profile(
                rotations=2.0,
                full_steps_per_revolution=200,
                microsteps=256,
                gear_ratio=1.0,
            )

    def test_export_replay_csv_has_stable_gui_columns_and_prepad(self) -> None:
        dataset = self._load_text(
            "\n".join(
                (
                    "Time (s),Altitude (m),Vertical velocity (m/s),Vertical acceleration (m/s^2)",
                    "0,0,0,0",
                    "0.5,1,2,4",
                    "1.0,2,0,-4",
                )
            )
            + "\n"
        )
        profile = replay.build_replay_profile(
            dataset,
            rate_hz=20.0,
            prepad_s=0.5,
            stop_s=1.0,
        )
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        destination = Path(temporary.name) / "nested" / "normalized.csv"

        exported, metadata_path = replay.export_replay_csv(
            destination,
            dataset,
            profile,
            barometer_stddev_m=1.5,
            target_apogee_m=914.4,
            brake_count=4,
            force_per_brake_n=55.0,
            rotations=3.0,
            full_steps_per_revolution=200,
            microsteps=256,
            gear_ratio=1.0,
        )
        rows = exported.read_text(encoding="utf-8").splitlines()
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))

        self.assertEqual(
            rows[0],
            "simulation_time_s,source_time_s,altitude_m_agl,"
            "vertical_velocity_mps_reference,"
            "vertical_acceleration_mps2_gravity_removed,"
            "barometer_stddev_m,is_prepad",
        )
        self.assertTrue(rows[1].endswith(",1"))
        self.assertTrue(rows[11].endswith(",0"))
        self.assertEqual(len(rows), len(profile.samples) + 1)
        self.assertEqual(metadata["schema"], "ambar.openrocket_replay.v1")
        self.assertFalse(Path(metadata["source"]["path"]).is_absolute())
        self.assertTrue(metadata["replay"]["vertical_channels_derived"] is False)
        self.assertEqual(
            metadata["mechanical_display_assumptions"]["prototype_equivalent_full_counts"],
            153600,
        )

    def test_sequence_counter_uses_seed_and_wraps(self) -> None:
        sequence = replay.SequenceCounter(seed=0xFFFF)
        self.assertEqual(sequence.take(), 0xFFFF)
        self.assertEqual(sequence.take(), 0)

    def test_live_auto_detection_rejects_multiple_ambar_boards(self) -> None:
        class Port:
            def __init__(self, device: str) -> None:
                self.device = device
                self.vid = replay.AMBAR_USB_VID
                self.pid = replay.AMBAR_USB_PID

        class ListPorts:
            @staticmethod
            def comports():
                return [Port("COM4"), Port("COM7")]

        with self.assertRaisesRegex(replay.ReplayError, "multiple AMBAR"):
            replay._choose_port(ListPorts, None)

    @unittest.skipUnless(
        REAL_OPENROCKET_CSV.is_file(),
        "the project copy of the supplied OpenRocket CSV is missing",
    )
    def test_supplied_openrocket_profile(self) -> None:
        dataset = replay.load_openrocket_csv(REAL_OPENROCKET_CSV)
        profile = replay.build_replay_profile(dataset)
        times = dataset.values(dataset.time_index)
        altitudes = dataset.values(dataset.altitude_index)
        peak_index = max(range(dataset.row_count), key=altitudes.__getitem__)

        self.assertEqual(dataset.row_count, 618)
        self.assertAlmostEqual(times[-1], 104.9163, places=4)
        self.assertAlmostEqual(altitudes[peak_index], 1031.2646, places=4)
        self.assertAlmostEqual(times[peak_index], 13.9185, places=4)
        self.assertEqual(dataset.event_times("LIFTOFF"), (0.071,))
        self.assertEqual(dataset.event_times("APOGEE"), (13.918,))
        self.assertAlmostEqual(profile.source_max_gap_s, 0.5, places=6)
        self.assertTrue(profile.uses_derived_vertical)
        self.assertAlmostEqual(profile.source_stop_s, 15.918, places=3)

        source_samples = {
            round(sample.source_time_s, 2): sample
            for sample in profile.samples
            if sample.replay_time_s >= profile.prepad_s
        }
        self.assertGreater(source_samples[1.0].vertical_velocity_mps, 0.0)
        self.assertLess(source_samples[14.5].vertical_velocity_mps, 0.0)
        self.assertLess(source_samples[13.92].vertical_acceleration_mps2, 0.0)
        self.assertTrue(any("TUMBLE" in warning for warning in profile.warnings))
        self.assertTrue(any("open-loop" in warning for warning in profile.warnings))


@unittest.skipUnless(
    hasattr(replay, "_enforce_actuator_motion")
    and hasattr(replay, "_actuator_ready_for_flight"),
    "planned actuator-motion replay interface has not landed yet",
)
class ActuatorMotionReplayTests(unittest.TestCase):
    # -----------------------------------------------------------------------
    # Deterministic high-resolution clock, UDP sink, and fake serial board
    # -----------------------------------------------------------------------

    class _FakeClock:
        def __init__(
            self,
            *,
            oversleep_at_s: float | None = None,
            oversleep_s: float = 0.0,
        ) -> None:
            self.now_s = 0.0
            self.oversleep_at_s = oversleep_at_s
            self.oversleep_s = oversleep_s
            self.oversleep_applied = False

        def now(self) -> float:
            return self.now_s

        def sleep(self, seconds: float) -> None:
            increment = max(0.0, seconds)
            if (
                not self.oversleep_applied
                and self.oversleep_at_s is not None
                and self.now_s + increment >= self.oversleep_at_s
            ):
                increment += self.oversleep_s
                self.oversleep_applied = True
            self.now_s += increment

    class _FakeUdpSocket:
        def __init__(self) -> None:
            self.sent: list[tuple[bytes, tuple[str, int]]] = []
            self.closed = False

        def sendto(self, payload: bytes, target: tuple[str, int]) -> int:
            self.sent.append((bytes(payload), target))
            return len(payload)

        def close(self) -> None:
            self.closed = True

    class _FakeMotionPort:
        def __init__(
            self,
            monitor: replay.PacketMonitor,
            *,
            fault_on_simulation_sample: int | None = None,
            prehome_geometry_plausible: bool = True,
        ) -> None:
            self.monitor = monitor
            self.fault_on_simulation_sample = fault_on_simulation_sample
            self.prehome_geometry_plausible = prehome_geometry_plausible
            self.decoder = protocol.StreamDecoder()
            self.clock: ActuatorMotionReplayTests._FakeClock | None = None
            self.commands: list[int] = []
            self.simulation_sample_count = 0
            self.simulation_active = False
            self.next_telemetry_s = 0.0
            self.next_actuator_s = 0.0
            self.base_flags = (
                ACTUATOR_FLAG_BUILD_ENABLED
                | ACTUATOR_FLAG_BENCH_ENABLED
                | ACTUATOR_FLAG_DRIVER_OK
                | ACTUATOR_FLAG_CONFIG_VALID
            )
            self.current_flags = self.base_flags
            self.target_steps = 0
            self.actual_steps = 0
            self.monitor.heartbeat = {
                "feature_flags": (
                    FEATURE_USB_PROTOCOL
                    | FEATURE_SIMULATION
                    | FEATURE_ACTUATOR
                    | FEATURE_BENCH_COMMANDS
                    | FEATURE_PRESENTATION_MOTION
                ),
                "receive_errors": 0,
                "transmit_drops": 0,
            }
            self.monitor.initial_heartbeat = dict(self.monitor.heartbeat)
            self._publish_actuator(machine_state=1)

        def __enter__(self) -> "ActuatorMotionReplayTests._FakeMotionPort":
            return self

        def __exit__(self, exc_type, exc, traceback) -> None:
            return None

        def reset_input_buffer(self) -> None:
            return None

        def read(self, _size: int) -> bytes:
            self._emit_periodic_status()
            return b""

        def _now(self) -> float:
            return 0.0 if self.clock is None else self.clock.now()

        def _emit_periodic_status(self) -> None:
            if not self.simulation_active:
                return
            now = self._now()
            while now + 1e-12 >= self.next_telemetry_s:
                self._publish_simulation_telemetry()
                self.next_telemetry_s += 0.050
            while now + 1e-12 >= self.next_actuator_s:
                assert self.monitor.actuator is not None
                self._publish_actuator(
                    machine_state=int(self.monitor.actuator["machine_state"]),
                    target_steps=int(self.monitor.actuator["target_steps"]),
                    actual_steps=int(self.monitor.actuator["actual_steps"]),
                )
                self.next_actuator_s += 0.100

        def _publish_actuator(
            self,
            *,
            machine_state: int,
            target_steps: int = 0,
            actual_steps: int = 0,
        ) -> None:
            self.target_steps = target_steps
            self.actual_steps = actual_steps
            reserved = protocol.ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE
            if (
                self.prehome_geometry_plausible
                or self.current_flags & ACTUATOR_FLAG_HOMED
            ):
                reserved |= protocol.ACTUATOR_STATUS_LIMITS_PLAUSIBLE
            if (
                self.current_flags & ACTUATOR_FLAG_HOMED
                and target_steps == 0
                and actual_steps == 0
            ):
                reserved |= protocol.ACTUATOR_STATUS_HOME_ACTIVE
            self.monitor.actuator = {
                "flags": self.current_flags,
                "machine_state": machine_state,
                "target_steps": target_steps,
                "actual_steps": actual_steps,
                "driver_status": 0,
                "reserved": reserved,
                "home_active": bool(
                    reserved & protocol.ACTUATOR_STATUS_HOME_ACTIVE
                ),
                "full_active": False,
                "limits_plausible": True,
            }
            self.monitor.actuator_count += 1
            self.monitor.actuator_received_s = self._now()
            self.monitor.max_abs_actuator_target_steps = max(
                self.monitor.max_abs_actuator_target_steps,
                abs(target_steps),
            )
            self.monitor.max_abs_actuator_actual_steps = max(
                self.monitor.max_abs_actuator_actual_steps,
                abs(actual_steps),
            )
            self.monitor.max_actuator_tracking_error_steps = max(
                self.monitor.max_actuator_tracking_error_steps,
                abs(target_steps - actual_steps),
            )

        def _publish_simulation_telemetry(self) -> None:
            if self.simulation_sample_count < 6:
                phase = 0
                deployment = 0
            elif self.simulation_sample_count < 11:
                phase = 1
                deployment = 0
            elif self.simulation_sample_count < 16:
                phase = 2
                deployment = 0
            elif self.simulation_sample_count < 22:
                phase = 3
                deployment = 50
            else:
                phase = 4
                deployment = 0
            self.monitor.telemetry = {
                "flags": replay.TELEMETRY_FLAG_SIMULATION_ACTIVE,
                "state": phase,
                "altitude_m": 0.0,
                "velocity_mps": 0.0,
                "predicted_apogee_m": 0.0,
                "deployment_percent": deployment,
            }
            self.monitor.telemetry_count += 1
            self.monitor.telemetry_received_s = self._now()
            if (
                not self.monitor.phase_order
                or self.monitor.phase_order[-1] != phase
            ):
                self.monitor.phase_order.append(phase)
            self.monitor.max_deployment_percent = max(
                self.monitor.max_deployment_percent,
                deployment,
            )
            self.monitor.max_altitude_m = max(self.monitor.max_altitude_m, 0.0)
            self.monitor.max_predicted_apogee_m = max(
                self.monitor.max_predicted_apogee_m,
                0.0,
            )

        def write(self, data: bytes) -> int:
            packets = self.decoder.feed(bytes(data))
            if len(packets) != 1:
                raise AssertionError("each fake serial write must contain exactly one frame")
            packet = packets[0]

            if packet.packet_type == protocol.PKT_COMMAND:
                command = packet.payload[0]
                self.commands.append(command)
                self.monitor.acks[packet.sequence] = {
                    "command_sequence": packet.sequence,
                    "command": command,
                    "result": protocol.ACK_OK,
                    "detail": 0,
                }

                if command == protocol.CMD_SET_ARMED and packet.payload[2:] == b"\x00":
                    self.current_flags &= ~(
                        ACTUATOR_FLAG_DRIVER_ENABLED | ACTUATOR_FLAG_MANUAL_PENDING
                    )
                    self._publish_actuator(
                        machine_state=1,
                        target_steps=self.target_steps,
                        actual_steps=self.actual_steps,
                    )
                elif command == protocol.CMD_HOME:
                    self.current_flags |= ACTUATOR_FLAG_HOMED
                    self._publish_actuator(machine_state=3)
                elif command == protocol.CMD_SIM_START:
                    self.simulation_active = True
                    self.next_telemetry_s = self._now()
                    self.next_actuator_s = self._now()
                    self._emit_periodic_status()
                elif command == protocol.CMD_SIM_STOP:
                    self.simulation_active = False
                elif command == protocol.CMD_RETRACT:
                    self.current_flags &= ~(
                        ACTUATOR_FLAG_DRIVER_ENABLED | ACTUATOR_FLAG_MANUAL_PENDING
                    )
                    self._publish_actuator(machine_state=3)

            elif packet.packet_type == protocol.PKT_SIMULATION:
                self.simulation_sample_count += 1
                if self.simulation_sample_count == 16:
                    self.current_flags |= ACTUATOR_FLAG_DRIVER_ENABLED
                    self._publish_actuator(
                        machine_state=4,
                        target_steps=76800,
                        actual_steps=76800,
                    )
                elif self.simulation_sample_count == 22:
                    self.current_flags &= ~ACTUATOR_FLAG_DRIVER_ENABLED
                    self._publish_actuator(machine_state=3)
                if self.simulation_sample_count == self.fault_on_simulation_sample:
                    self.current_flags |= ACTUATOR_FLAG_ESTOP
                    self._publish_actuator(machine_state=replay.ACTUATOR_STATE_ESTOP)

            return len(data)

    @staticmethod
    def _tiny_profile() -> replay.ReplayProfile:
        samples = tuple(
            replay.ReplaySample(
                index * 0.020,
                max(0.0, index * 0.020 - 0.5),
                float(index),
                1.0,
                0.0,
            )
            for index in range(61)
        )
        return replay.ReplayProfile(
            samples=samples,
            rate_hz=50.0,
            prepad_s=0.5,
            source_stop_s=0.7,
            vertical_source="synthetic integration test",
            uses_derived_vertical=False,
            source_max_gap_s=0.1,
            warnings=(),
        )

    def _run_fake_motion_replay(
        self,
        port: _FakeMotionPort,
        *,
        clock: _FakeClock | None = None,
        run_bundle_dir: Path | None = None,
        gui_udp_port: int | None = None,
    ) -> dict:
        if clock is None:
            clock = self._FakeClock()
        port.clock = clock

        class FakeSerialModule:
            @staticmethod
            def Serial(*_args, **_kwargs):
                return port

        def configured_monitor(**kwargs):
            port.monitor.clock = kwargs["clock"]
            port.monitor.observer = kwargs["observer"]
            return port.monitor

        with (
            mock.patch.object(
                replay,
                "_require_serial",
                return_value=(FakeSerialModule, object()),
            ),
            mock.patch.object(replay, "PacketMonitor", side_effect=configured_monitor),
            mock.patch.object(replay.secrets, "randbelow", return_value=0),
            redirect_stdout(io.StringIO()),
        ):
            return replay.run_live_replay(
                self._tiny_profile(),
                port_name="COM_TEST",
                barometer_stddev_m=1.5,
                arm_after_s=0.1,
                target_apogee_m=914.4,
                no_arm=False,
                rotations=3.0,
                full_steps_per_revolution=200,
                microsteps=256,
                gear_ratio=1.0,
                allow_actuator_motion=True,
                home_at_current_position=True,
                clock=clock,
                run_bundle_dir=run_bundle_dir,
                gui_udp_port=gui_udp_port,
            )

    @staticmethod
    def _monitor(
        *,
        features: int,
        actuator_flags: int,
        reserved: int = (
            protocol.ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE
            | protocol.ACTUATOR_STATUS_LIMITS_PLAUSIBLE
        ),
    ) -> replay.PacketMonitor:
        monitor = replay.PacketMonitor()
        monitor.heartbeat = {"feature_flags": features}
        monitor.actuator = {
            "flags": actuator_flags,
            "machine_state": 1,
            "driver_status": 0,
            "reserved": reserved,
            "target_steps": 0,
            "actual_steps": 0,
        }
        return monitor

    @staticmethod
    def _motion_features() -> int:
        return (
            FEATURE_USB_PROTOCOL
            | FEATURE_SIMULATION
            | FEATURE_ACTUATOR
            | FEATURE_BENCH_COMMANDS
            | FEATURE_PRESENTATION_MOTION
        )

    @staticmethod
    def _prehome_actuator_flags() -> int:
        return (
            ACTUATOR_FLAG_BUILD_ENABLED
            | ACTUATOR_FLAG_BENCH_ENABLED
            | ACTUATOR_FLAG_DRIVER_OK
            | ACTUATOR_FLAG_CONFIG_VALID
        )

    # -----------------------------------------------------------------------
    # Motion preflight, freshness, command-order, and cleanup regressions
    # -----------------------------------------------------------------------

    def test_motion_preflight_accepts_safe_unhomed_actuator(self) -> None:
        monitor = self._monitor(
            features=self._motion_features(),
            actuator_flags=self._prehome_actuator_flags(),
        )

        self.assertIsNone(replay._enforce_actuator_motion(monitor))

    def test_motion_preflight_requires_every_build_feature(self) -> None:
        for missing in (
            FEATURE_USB_PROTOCOL,
            FEATURE_SIMULATION,
            FEATURE_ACTUATOR,
            FEATURE_BENCH_COMMANDS,
            FEATURE_PRESENTATION_MOTION,
        ):
            with self.subTest(missing=missing):
                monitor = self._monitor(
                    features=self._motion_features() & ~missing,
                    actuator_flags=self._prehome_actuator_flags(),
                )
                with self.assertRaises(replay.ReplayError):
                    replay._enforce_actuator_motion(monitor)

    def test_motion_preflight_requires_driver_and_configuration_status(self) -> None:
        for missing in (
            ACTUATOR_FLAG_BUILD_ENABLED,
            ACTUATOR_FLAG_BENCH_ENABLED,
            ACTUATOR_FLAG_DRIVER_OK,
            ACTUATOR_FLAG_CONFIG_VALID,
        ):
            with self.subTest(missing=missing):
                monitor = self._monitor(
                    features=self._motion_features(),
                    actuator_flags=self._prehome_actuator_flags() & ~missing,
                )
                with self.assertRaises(replay.ReplayError):
                    replay._enforce_actuator_motion(monitor)

    def test_motion_preflight_rejects_unsafe_runtime_states(self) -> None:
        for unsafe in (
            ACTUATOR_FLAG_ESTOP,
            ACTUATOR_FLAG_DRIVER_ENABLED,
            ACTUATOR_FLAG_MANUAL_PENDING,
        ):
            with self.subTest(unsafe=unsafe):
                monitor = self._monitor(
                    features=self._motion_features(),
                    actuator_flags=self._prehome_actuator_flags() | unsafe,
                )
                with self.assertRaises(replay.ReplayError):
                    replay._enforce_actuator_motion(monitor)

    def test_flight_readiness_additionally_requires_homed(self) -> None:
        unhomed = self._monitor(
            features=self._motion_features(),
            actuator_flags=self._prehome_actuator_flags(),
        )
        homed = self._monitor(
            features=self._motion_features(),
            actuator_flags=self._prehome_actuator_flags() | ACTUATOR_FLAG_HOMED,
            reserved=(
                protocol.ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE
                | protocol.ACTUATOR_STATUS_LIMITS_PLAUSIBLE
                | protocol.ACTUATOR_STATUS_HOME_ACTIVE
            ),
        )

        self.assertFalse(replay._actuator_ready_for_flight(unhomed))
        self.assertTrue(replay._actuator_ready_for_flight(homed))

    def test_no_motion_replay_accepts_hil_profile_only_when_override_is_off(self) -> None:
        monitor = self._monitor(
            features=self._motion_features(),
            actuator_flags=self._prehome_actuator_flags(),
        )
        self.assertIsNone(replay._enforce_no_motion(monitor))

        assert monitor.actuator is not None
        monitor.actuator["reserved"] |= (
            protocol.ACTUATOR_STATUS_HIL_OVERRIDE_ACTIVE
        )
        with self.assertRaisesRegex(replay.ReplayError, "override is active"):
            replay._enforce_no_motion(monitor)

    def test_continuous_hil_preflight_requires_profile_and_plausible_geometry(self) -> None:
        monitor = replay.PacketMonitor()
        monitor.heartbeat = {
            "feature_flags": (
                FEATURE_USB_PROTOCOL
                | FEATURE_SIMULATION
                | FEATURE_ACTUATOR
                | replay.FEATURE_CONTINUOUS_HIL
            )
        }
        monitor.actuator = {
            "flags": (
                ACTUATOR_FLAG_BUILD_ENABLED
                | ACTUATOR_FLAG_DRIVER_OK
                | ACTUATOR_FLAG_CONFIG_VALID
            ),
            "machine_state": 1,
            "driver_status": 0,
            "reserved": (
                protocol.ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE
                | protocol.ACTUATOR_STATUS_LIMITS_PLAUSIBLE
                | protocol.ACTUATOR_STATUS_HOME_ACTIVE
            ),
            "target_steps": 0,
            "actual_steps": 0,
        }
        self.assertIsNone(replay._enforce_continuous_hil(monitor))

        monitor.actuator["reserved"] = (
            protocol.ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE
        )
        with self.assertRaisesRegex(replay.ReplayError, "geometry is not plausible"):
            replay._enforce_continuous_hil(monitor)
        self.assertIsNone(
            replay._enforce_continuous_hil(
                monitor,
                require_geometry=False,
            )
        )

        monitor.actuator["reserved"] = (
            protocol.ACTUATOR_STATUS_CONTINUOUS_HIL_PROFILE
            | protocol.ACTUATOR_STATUS_LIMITS_PLAUSIBLE
        )
        monitor.actuator["driver_status"] = replay.TMC_DRV_STATUS_OTPW
        with self.assertRaisesRegex(replay.ReplayError, "short/thermal"):
            replay._enforce_continuous_hil(monitor)

    def test_supervisor_preflight_homes_only_after_static_gates(self) -> None:
        clock = self._FakeClock()
        monitor = replay.PacketMonitor(clock=clock)
        port = self._FakeMotionPort(
            monitor,
            prehome_geometry_plausible=False,
        )
        port.clock = clock

        class FakeSerialModule:
            @staticmethod
            def Serial(*_args, **_kwargs):
                return port

        def configured_monitor(**kwargs):
            port.monitor.clock = kwargs["clock"]
            return port.monitor

        with (
            mock.patch.object(
                replay,
                "_require_serial",
                return_value=(FakeSerialModule, object()),
            ),
            mock.patch.object(replay, "PacketMonitor", side_effect=configured_monitor),
            mock.patch.object(replay, "_choose_port", return_value="COM_TEST"),
            mock.patch.object(replay.secrets, "randbelow", return_value=0),
        ):
            result = replay.run_continuous_hil_preflight(
                port_name="COM_TEST",
                accept_current_position_home=True,
                clock=clock,
            )

        self.assertEqual(result["selected_port"], "COM_TEST")
        self.assertTrue(result["home_verified"])
        self.assertTrue(result["motor_energy_off"])
        self.assertEqual(
            [stage["stage"] for stage in result["stages"]],
            [
                "static_gates_passed",
                "energy_off_confirmed",
                "software_home_declared",
                "final_energy_off",
            ],
        )
        self.assertEqual(
            port.commands,
            [
                protocol.CMD_PING,
                protocol.CMD_REQUEST_SNAPSHOT,
                protocol.CMD_SET_ARMED,
                protocol.CMD_SIM_STOP,
                protocol.CMD_HOME,
                protocol.CMD_SET_ARMED,
                protocol.CMD_SIM_STOP,
            ],
        )

    def test_supervisor_preflight_requires_ack_before_serial_access(self) -> None:
        with mock.patch.object(replay, "_require_serial") as require_serial:
            with self.assertRaisesRegex(
                replay.ReplayError,
                "manually fully closed",
            ):
                replay.run_continuous_hil_preflight(
                    port_name="COM_TEST",
                    accept_current_position_home=False,
                )
        require_serial.assert_not_called()

    def test_supervisor_preflight_wrong_profile_never_homes(self) -> None:
        clock = self._FakeClock()
        monitor = replay.PacketMonitor(clock=clock)
        port = self._FakeMotionPort(monitor)
        port.clock = clock
        assert monitor.heartbeat is not None
        monitor.heartbeat["feature_flags"] &= ~FEATURE_PRESENTATION_MOTION

        class FakeSerialModule:
            @staticmethod
            def Serial(*_args, **_kwargs):
                return port

        def configured_monitor(**kwargs):
            port.monitor.clock = kwargs["clock"]
            return port.monitor

        with (
            mock.patch.object(
                replay,
                "_require_serial",
                return_value=(FakeSerialModule, object()),
            ),
            mock.patch.object(replay, "PacketMonitor", side_effect=configured_monitor),
            mock.patch.object(replay, "_choose_port", return_value="COM_TEST"),
            mock.patch.object(replay.secrets, "randbelow", return_value=0),
        ):
            with self.assertRaisesRegex(replay.ReplayError, "missing"):
                replay.run_continuous_hil_preflight(
                    port_name="COM_TEST",
                    accept_current_position_home=True,
                    clock=clock,
                )

        self.assertNotIn(protocol.CMD_HOME, port.commands)

    def test_hil_stroke_tracker_requires_software_geometry_and_xactual(self) -> None:
        tracker = replay.HilStrokeTracker(
            burn_time_s=1.0,
            post_burn_margin_s=0.1,
            full_hold_s=0.5,
            endpoint_timeout_s=2.0,
        )
        telemetry = {"state": 2, "deployment_percent": 25}
        self.assertEqual(
            tracker.observe(
                source_time_s=1.2,
                telemetry=telemetry,
                actuator=None,
                host_now_s=10.0,
            ),
            protocol.HIL_OVERRIDE_FORCE_FULL,
        )
        full = {
            "reserved": (
                protocol.ACTUATOR_STATUS_FULL_ACTIVE
                | protocol.ACTUATOR_STATUS_LIMITS_PLAUSIBLE
            ),
            "target_steps": 153600,
            "actual_steps": 153600,
        }
        self.assertIsNone(
            tracker.observe(
                source_time_s=1.3,
                telemetry=telemetry,
                actuator=full,
                host_now_s=10.4,
            )
        )
        self.assertEqual(tracker.state, "AT_FULL")
        self.assertEqual(
            tracker.observe(
                source_time_s=2.0,
                telemetry={"state": 4, "deployment_percent": 0},
                actuator=full,
                host_now_s=10.5,
            ),
            protocol.HIL_OVERRIDE_FORCE_HOME,
        )
        home = {
            "reserved": (
                protocol.ACTUATOR_STATUS_HOME_ACTIVE
                | protocol.ACTUATOR_STATUS_LIMITS_PLAUSIBLE
                | protocol.ACTUATOR_STATUS_ENDPOINT_SEQUENCE_VERIFIED
            ),
            "target_steps": 0,
            "actual_steps": 0,
        }
        self.assertIsNone(
            tracker.observe(
                source_time_s=2.1,
                telemetry={"state": 4, "deployment_percent": 0},
                actuator=home,
                host_now_s=10.9,
            )
        )
        self.assertTrue(tracker.complete)
        self.assertAlmostEqual(tracker.metrics()["open_time_s"], 0.4)
        self.assertAlmostEqual(tracker.metrics()["close_time_s"], 0.4)

    def test_hil_stroke_tracker_faults_on_missing_full_endpoint(self) -> None:
        tracker = replay.HilStrokeTracker(
            burn_time_s=0.0,
            endpoint_timeout_s=0.5,
        )
        tracker.observe(
            source_time_s=0.2,
            telemetry={"state": 2, "deployment_percent": 0},
            actuator=None,
            host_now_s=1.0,
        )
        with self.assertRaisesRegex(replay.ReplayError, "software FULL/XACTUAL"):
            tracker.observe(
                source_time_s=0.8,
                telemetry={"state": 2, "deployment_percent": 0},
                actuator={
                    "reserved": protocol.ACTUATOR_STATUS_LIMITS_PLAUSIBLE,
                    "target_steps": 153600,
                    "actual_steps": 153600,
                },
                host_now_s=1.6,
            )

    def test_injected_clock_controls_actuator_and_telemetry_freshness(self) -> None:
        clock = self._FakeClock()
        monitor = replay.PacketMonitor(clock=clock)
        monitor.actuator = {
            "flags": self._prehome_actuator_flags() | ACTUATOR_FLAG_HOMED,
            "machine_state": 3,
            "target_steps": 0,
            "actual_steps": 0,
        }
        monitor.telemetry = {"state": 0}
        monitor.actuator_received_s = 0.0
        monitor.telemetry_received_s = 0.0

        clock.now_s = 0.351
        with self.assertRaisesRegex(replay.ReplayError, "actuator status became stale"):
            replay._assert_motion_runtime_healthy(monitor)

        monitor.actuator_received_s = clock.now()
        with self.assertRaisesRegex(replay.ReplayError, "flight telemetry became stale"):
            replay._assert_motion_runtime_healthy(monitor)

    def test_parser_recognizes_motion_options_and_defaults_to_no_motion(self) -> None:
        defaults = replay.build_argument_parser().parse_args(["flight.csv"])
        enabled = replay.build_argument_parser().parse_args(
            [
                "flight.csv",
                "--allow-actuator-motion",
                "--home-at-current-position",
                "--run-bundle",
                "evidence",
                "--gui-udp-port",
                "52100",
            ]
        )

        self.assertFalse(defaults.allow_actuator_motion)
        self.assertFalse(defaults.home_at_current_position)
        self.assertIsNone(defaults.run_bundle)
        self.assertIsNone(defaults.gui_udp_port)
        self.assertTrue(enabled.allow_actuator_motion)
        self.assertTrue(enabled.home_at_current_position)
        self.assertEqual(enabled.run_bundle, Path("evidence"))
        self.assertEqual(enabled.gui_udp_port, 52100)

    def test_motion_live_replay_command_order_and_fault_cleanup(self) -> None:
        expected_success = [
            protocol.CMD_PING,
            protocol.CMD_REQUEST_SNAPSHOT,
            protocol.CMD_SET_ARMED,
            protocol.CMD_SIM_STOP,
            protocol.CMD_HOME,
            protocol.CMD_SET_TARGET_APOGEE,
            protocol.CMD_SIM_START,
            protocol.CMD_SET_ARMED,
            protocol.CMD_SET_ARMED,
            protocol.CMD_SIM_STOP,
            protocol.CMD_RETRACT,
        ]
        expected_fault = expected_success[:8] + [
            protocol.CMD_SET_ARMED,
            protocol.CMD_SIM_STOP,
        ]

        with self.subTest("successful replay retracts after stopping"):
            monitor = replay.PacketMonitor()
            port = self._FakeMotionPort(monitor)
            self._run_fake_motion_replay(port)
            self.assertEqual(port.commands, expected_success)

        with self.subTest("runtime fault performs energy-off cleanup without retract"):
            monitor = replay.PacketMonitor()
            port = self._FakeMotionPort(monitor, fault_on_simulation_sample=18)
            with self.assertRaises(replay.ReplayError):
                self._run_fake_motion_replay(port)
            self.assertEqual(port.commands, expected_fault)
            self.assertEqual(
                port.commands[-2:],
                [protocol.CMD_SET_ARMED, protocol.CMD_SIM_STOP],
            )
            self.assertNotIn(protocol.CMD_RETRACT, port.commands)

    # -----------------------------------------------------------------------
    # Persistent evidence, timing acceptance, and localhost GUI regressions
    # -----------------------------------------------------------------------

    def test_success_bundle_and_localhost_gui_are_complete(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        bundle = Path(temporary.name) / "presentation-run"
        fake_socket = self._FakeUdpSocket()
        monitor = replay.PacketMonitor()
        port = self._FakeMotionPort(monitor)

        with mock.patch.object(
            reporting.socket,
            "socket",
            return_value=fake_socket,
        ):
            verdict = self._run_fake_motion_replay(
                port,
                run_bundle_dir=bundle,
                gui_udp_port=52100,
            )

        manifest = json.loads((bundle / "manifest.json").read_text(encoding="utf-8"))
        stored_verdict = json.loads(
            (bundle / "verdict.json").read_text(encoding="utf-8")
        )
        records = [
            json.loads(line)
            for line in (bundle / "packets.jsonl")
            .read_text(encoding="utf-8")
            .splitlines()
        ]
        gui_records = [json.loads(payload) for payload, _target in fake_socket.sent]

        self.assertEqual(verdict["status"], "PASS")
        self.assertEqual(stored_verdict, verdict)
        self.assertEqual(manifest["status"], "PASS")
        self.assertEqual(
            manifest["clock"]["host_scheduler"],
            "time.perf_counter",
        )
        self.assertEqual(
            manifest["connection"]["gui_transport"],
            "udp://127.0.0.1:52100",
        )
        self.assertTrue(all(check["passed"] for check in verdict["checks"].values()))
        self.assertEqual(verdict["metrics"]["max_abs_actuator_actual_steps"], 76800)
        self.assertEqual(verdict["final_actuator"]["actual_steps"], 0)
        self.assertTrue(any(record["event"] == "packet_tx" for record in records))
        self.assertTrue(any(record["event"] == "sample_sent" for record in records))
        self.assertEqual(records[-1]["event"], "run_verdict")
        self.assertTrue(fake_socket.closed)
        self.assertTrue(all(payload.endswith(b"\n") for payload, _ in fake_socket.sent))
        self.assertTrue(
            all(target == ("127.0.0.1", 52100) for _payload, target in fake_socket.sent)
        )
        self.assertTrue(any(record["event"] == "run_verdict" for record in gui_records))

    def test_injected_clock_skips_late_sample_without_catchup_burst(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        bundle = Path(temporary.name) / "jitter-run"
        clock = self._FakeClock(oversleep_at_s=0.200, oversleep_s=0.015)
        monitor = replay.PacketMonitor()
        port = self._FakeMotionPort(monitor)

        verdict = self._run_fake_motion_replay(
            port,
            clock=clock,
            run_bundle_dir=bundle,
        )
        records = [
            json.loads(line)
            for line in (bundle / "packets.jsonl")
            .read_text(encoding="utf-8")
            .splitlines()
        ]
        sent_times = [
            float(record["host_elapsed_s"])
            for record in records
            if record["event"] == "sample_sent"
        ]

        self.assertEqual(verdict["status"], "PASS")
        self.assertEqual(verdict["metrics"]["skipped_host_samples"], 1)
        self.assertEqual(verdict["metrics"]["sent_host_samples"], 60)
        self.assertLessEqual(
            verdict["metrics"]["host_skip_ratio"],
            replay.MAX_HOST_SKIP_RATIO,
        )
        self.assertGreater(verdict["metrics"]["max_schedule_lag_s"], 0.010)
        self.assertLess(verdict["metrics"]["max_schedule_lag_s"], 0.100)
        self.assertEqual(
            sum(record["event"] == "sample_skipped" for record in records),
            1,
        )
        self.assertTrue(
            all(after - before >= 0.005 for before, after in zip(sent_times, sent_times[1:]))
        )

    def test_continuous_hil_rebases_transient_lag_without_skip_or_burst(
        self,
    ) -> None:
        clock = self._FakeClock(oversleep_at_s=0.200, oversleep_s=0.060)
        start_s = clock.now()
        sent_times: list[float] = []
        results: list[replay.ReplayDeadlineResult] = []

        for sample_index in range(21):
            result = replay._wait_for_replay_deadline(
                clock,
                lambda: None,
                start_s=start_s,
                replay_time_s=sample_index * 0.020,
                rate_hz=50.0,
                continuous_hil=True,
            )
            results.append(result)
            start_s += result.rebase_s
            if not result.skip:
                sent_times.append(clock.now())

        rebased = [
            (index, result)
            for index, result in enumerate(results)
            if result.rebase_s > 0.0
        ]
        self.assertEqual(len(sent_times), 21)
        self.assertTrue(all(not result.skip for result in results))
        self.assertEqual(len(rebased), 1)
        rebase_index, rebase_result = rebased[0]
        self.assertGreater(rebase_result.lag_s, 0.050)
        self.assertLess(rebase_result.lag_s, replay.MAX_HOST_SCHEDULE_LAG_S)
        self.assertAlmostEqual(
            sent_times[rebase_index + 1] - sent_times[rebase_index],
            0.020,
            places=9,
        )

    def test_deadline_failure_finalizes_bundle_after_safe_cleanup(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        bundle = Path(temporary.name) / "deadline-failure"
        clock = self._FakeClock(oversleep_at_s=0.200, oversleep_s=0.120)
        monitor = replay.PacketMonitor()
        port = self._FakeMotionPort(monitor)

        with self.assertRaisesRegex(replay.ReplayError, "missed a replay deadline"):
            self._run_fake_motion_replay(
                port,
                clock=clock,
                run_bundle_dir=bundle,
            )

        verdict = json.loads((bundle / "verdict.json").read_text(encoding="utf-8"))
        records = [
            json.loads(line)
            for line in (bundle / "packets.jsonl")
            .read_text(encoding="utf-8")
            .splitlines()
        ]
        transmitted_commands = [
            record["decoded"]["command"]
            for record in records
            if record["event"] == "packet_tx"
            and record["packet_type"] == protocol.PKT_COMMAND
        ]

        self.assertEqual(verdict["status"], "FAIL")
        self.assertFalse(verdict["checks"]["run_completed"]["passed"])
        self.assertFalse(verdict["checks"]["host_deadline"]["passed"])
        self.assertGreater(
            verdict["checks"]["host_deadline"]["actual"],
            replay.MAX_HOST_SCHEDULE_LAG_S,
        )
        self.assertTrue(verdict["checks"]["final_driver_off"]["passed"])
        self.assertTrue(
            verdict["checks"]["fault_cleanup_energy_off"]["passed"]
        )
        self.assertTrue(
            verdict["safety_cleanup"]["motor_energy_off_verified"]
        )
        self.assertIsNotNone(verdict["final_actuator"])
        self.assertEqual(verdict["failure"]["type"], "ReplayDeadlineError")
        self.assertTrue(any(record["event"] == "safety_cleanup" for record in records))
        self.assertTrue(
            any(record["event"] == "safety_cleanup_result" for record in records)
        )
        self.assertEqual(
            transmitted_commands[-2:],
            [protocol.CMD_SET_ARMED, protocol.CMD_SIM_STOP],
        )
        self.assertNotIn(protocol.CMD_RETRACT, transmitted_commands)
        self.assertEqual(records[-1]["event"], "run_verdict")

    def test_continuous_hil_fault_cleanup_stops_in_place_and_records_fresh_state(
        self,
    ) -> None:
        clock = self._FakeClock()
        monitor = replay.PacketMonitor(clock=clock)
        port = self._FakeMotionPort(monitor)
        port.clock = clock
        port.current_flags |= (
            ACTUATOR_FLAG_HOMED | ACTUATOR_FLAG_DRIVER_ENABLED
        )
        port._publish_actuator(
            machine_state=8,
            target_steps=60321,
            actual_steps=60284,
        )
        assert monitor.actuator is not None
        monitor.actuator["reserved"] |= (
            protocol.ACTUATOR_STATUS_HIL_OVERRIDE_ACTIVE
            | (
                protocol.HIL_OVERRIDE_FORCE_HOME
                << protocol.ACTUATOR_STATUS_HIL_OVERRIDE_SHIFT
            )
        )
        records: list[dict] = []
        observer = reporting.ReplayRunObserver(
            clock=clock,
            event_sink=lambda record: records.append(dict(record)),
        )
        observer.start({"test": True})
        monitor.observer = observer

        cleanup = replay._perform_safety_cleanup(
            port,
            monitor,
            replay.SequenceCounter(seed=1),
            observer,
            clock,
            continuous_hil=True,
        )
        observer.finalize({"status": "FAIL", "passed": False})

        self.assertEqual(
            port.commands,
            [
                protocol.CMD_HIL_SET_OVERRIDE,
                protocol.CMD_SET_ARMED,
                protocol.CMD_SIM_STOP,
            ],
        )
        self.assertNotIn(protocol.CMD_RETRACT, port.commands)
        self.assertEqual(
            cleanup["policy"],
            "stop_in_place_no_automatic_home_after_fault",
        )
        self.assertTrue(cleanup["fresh_actuator_status_received"])
        self.assertTrue(cleanup["motor_energy_off_verified"])
        self.assertEqual(
            cleanup["pre_cleanup_actuator"]["actual_steps"],
            60284,
        )
        self.assertEqual(
            cleanup["post_cleanup_actuator"]["actual_steps"],
            60284,
        )
        self.assertEqual(
            cleanup["post_cleanup_actuator"]["target_steps"],
            60321,
        )
        self.assertEqual(
            cleanup["post_cleanup_actuator"]["flags"]
            & ACTUATOR_FLAG_DRIVER_ENABLED,
            0,
        )
        self.assertEqual(
            cleanup["post_cleanup_actuator"]["reserved"]
            & protocol.ACTUATOR_STATUS_HIL_OVERRIDE_ACTIVE,
            0,
        )
        self.assertTrue(
            any(record["event"] == "safety_cleanup_result" for record in records)
        )

    def test_verdict_rejects_decoder_and_heartbeat_counter_increases(self) -> None:
        clock = self._FakeClock()
        monitor = replay.PacketMonitor(clock=clock)
        monitor.initial_heartbeat = {
            "feature_flags": self._motion_features(),
            "receive_errors": 2,
            "transmit_drops": 3,
        }
        monitor.heartbeat = {
            "feature_flags": self._motion_features(),
            "receive_errors": 3,
            "transmit_drops": 3,
        }
        monitor.phase_order = list(replay.PRESENTATION_PHASE_ORDER)
        monitor.max_deployment_percent = 50
        monitor.actuator = {
            "flags": self._prehome_actuator_flags() | ACTUATOR_FLAG_HOMED,
            "machine_state": 3,
            "target_steps": 0,
            "actual_steps": 0,
        }
        monitor.decoder.errors = 1

        verdict = replay._build_live_verdict(
            monitor,
            completed=True,
            failure=None,
            no_arm=False,
            allow_actuator_motion=True,
            max_schedule_lag_s=0.001,
            skipped_host_samples=0,
            sent_host_samples=10,
        )

        self.assertEqual(verdict["status"], "FAIL")
        self.assertFalse(verdict["checks"]["decoder_errors"]["passed"])
        self.assertFalse(
            verdict["checks"]["heartbeat_error_drop_deltas"]["passed"]
        )
        self.assertEqual(verdict["transport"]["receive_error_delta"], 1)

    def test_skip_ratio_accepts_two_percent_and_rejects_excess_or_zero(self) -> None:
        def ready_monitor() -> replay.PacketMonitor:
            monitor = replay.PacketMonitor(clock=self._FakeClock())
            monitor.initial_heartbeat = {
                "feature_flags": self._motion_features(),
                "receive_errors": 0,
                "transmit_drops": 0,
            }
            monitor.heartbeat = dict(monitor.initial_heartbeat)
            monitor.phase_order = list(replay.PRESENTATION_PHASE_ORDER)
            monitor.max_deployment_percent = 50
            monitor.actuator = {
                "flags": self._prehome_actuator_flags() | ACTUATOR_FLAG_HOMED,
                "machine_state": 3,
                "target_steps": 0,
                "actual_steps": 0,
            }
            return monitor

        cases = (
            (98, 2, True, 0.02),
            (97, 3, False, 0.03),
            (0, 0, False, None),
        )
        for sent, skipped, expected_pass, expected_ratio in cases:
            with self.subTest(sent=sent, skipped=skipped):
                verdict = replay._build_live_verdict(
                    ready_monitor(),
                    completed=True,
                    failure=None,
                    no_arm=False,
                    allow_actuator_motion=True,
                    max_schedule_lag_s=0.001,
                    skipped_host_samples=skipped,
                    sent_host_samples=sent,
                )
                check = verdict["checks"]["host_skip_ratio"]
                self.assertEqual(check["passed"], expected_pass)
                self.assertEqual(check["actual"], expected_ratio)
                self.assertEqual(verdict["passed"], expected_pass)

        continuous = replay._build_live_verdict(
            ready_monitor(),
            completed=True,
            failure=None,
            no_arm=False,
            allow_actuator_motion=True,
            max_schedule_lag_s=0.001,
            skipped_host_samples=1,
            sent_host_samples=99,
            continuous_hil=True,
        )
        self.assertFalse(continuous["checks"]["host_skip_ratio"]["passed"])
        self.assertEqual(
            continuous["checks"]["host_skip_ratio"]["expected"],
            "0 skipped samples",
        )

    def test_packet_monitor_mirrors_decoded_telemetry_to_gui(self) -> None:
        fake_socket = self._FakeUdpSocket()
        clock = self._FakeClock()
        telemetry_payload = struct.pack(
            "<HBBhhhHHhhhBBHBB",
            replay.TELEMETRY_FLAG_SIMULATION_ACTIVE,
            3,
            0,
            100,
            200,
            -10,
            1000,
            900,
            0,
            0,
            0,
            50,
            0,
            0,
            0,
            0,
        )
        frame = protocol.encode_frame(
            protocol.PKT_TELEMETRY,
            7,
            1234,
            telemetry_payload,
        )

        class OneReadPort:
            def __init__(self, payload: bytes) -> None:
                self.payload = payload

            def read(self, _size: int) -> bytes:
                payload, self.payload = self.payload, b""
                return payload

        with mock.patch.object(
            reporting.socket,
            "socket",
            return_value=fake_socket,
        ):
            observer = reporting.ReplayRunObserver(clock=clock, gui_udp_port=52101)
            observer.start({"test": True})
            monitor = replay.PacketMonitor(clock=clock, observer=observer)
            monitor.poll(OneReadPort(frame))
            observer.finalize({"status": "PASS", "passed": True})

        gui_records = [json.loads(payload) for payload, _target in fake_socket.sent]
        telemetry_records = [
            record
            for record in gui_records
            if record["event"] == "packet_rx"
            and record["packet_type"] == protocol.PKT_TELEMETRY
        ]
        self.assertEqual(len(telemetry_records), 1)
        self.assertEqual(telemetry_records[0]["packet_time_ms"], 1234)
        self.assertEqual(
            telemetry_records[0]["packet_time_domain"],
            "stm32_device",
        )
        self.assertEqual(telemetry_records[0]["decoded"]["deployment_percent"], 50)

    def test_slow_evidence_sink_never_blocks_replay_emits_and_drains_fifo(
        self,
    ) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        bundle = Path(temporary.name) / "async-evidence"

        class SlowSink:
            def __init__(self) -> None:
                self.records: list[dict[str, object]] = []
                self.flushed = False
                self.closed = False

            def __call__(self, record) -> None:
                time.sleep(0.020)
                self.records.append(dict(record))

            def flush(self) -> None:
                self.flushed = True

            def close(self) -> None:
                self.closed = True

        sink = SlowSink()
        observer = reporting.ReplayRunObserver(
            clock=reporting.PerfCounterClock(),
            bundle_dir=bundle,
            event_sink=sink,
            flush_interval_s=60.0,
        )
        observer.start({"test": True})
        mutable = {"value": 1}
        started = time.perf_counter()
        for index in range(40):
            observer.emit("sample", sample_index=index, mutable=mutable)
        emit_elapsed = time.perf_counter() - started
        mutable["value"] = 2
        observer.finalize({"status": "PASS", "passed": True})

        records = [
            json.loads(line)
            for line in (bundle / "packets.jsonl")
            .read_text(encoding="utf-8")
            .splitlines()
        ]
        self.assertLess(emit_elapsed, 0.150)
        self.assertEqual(
            [record["event_index"] for record in records],
            list(range(42)),
        )
        self.assertEqual(
            [record["event_index"] for record in sink.records],
            list(range(42)),
        )
        self.assertEqual(records[1]["mutable"]["value"], 1)
        self.assertEqual(records[-1]["event"], "run_verdict")
        self.assertTrue(sink.flushed)
        self.assertTrue(sink.closed)

    def test_async_evidence_failure_propagates_and_marks_verdict_failed(
        self,
    ) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        bundle = Path(temporary.name) / "failed-evidence"

        class FailingSink:
            def __call__(self, record) -> None:
                if record["event"] == "sample":
                    raise OSError("simulated SQLite write failure")

        observer = reporting.ReplayRunObserver(
            clock=reporting.PerfCounterClock(),
            bundle_dir=bundle,
            event_sink=FailingSink(),
        )
        observer.start({"test": True})
        observer.emit("sample", sample_index=0)
        with self.assertRaisesRegex(
            reporting.ReplayEvidenceWriterError,
            "simulated SQLite write failure",
        ):
            observer.finalize({"status": "PASS", "passed": True})

        verdict = json.loads(
            (bundle / "verdict.json").read_text(encoding="utf-8")
        )
        records = [
            json.loads(line)
            for line in (bundle / "packets.jsonl")
            .read_text(encoding="utf-8")
            .splitlines()
        ]
        self.assertEqual(verdict["status"], "FAIL")
        self.assertFalse(verdict["passed"])
        self.assertEqual(
            verdict["failure"]["type"],
            "ReplayEvidenceWriterError",
        )
        self.assertFalse(verdict["checks"]["evidence_writer"]["passed"])
        self.assertEqual(
            [record["event_index"] for record in records],
            sorted(record["event_index"] for record in records),
        )


if __name__ == "__main__":
    unittest.main()
