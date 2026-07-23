"""Deterministic causal timing and fail-safe tests for VARIABLE_HIL."""

from __future__ import annotations

import copy
from dataclasses import replace
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(
    0,
    str(
        ROOT
        / "firmware"
        / "stm32_airbrake_pcb"
        / "tools"
        / "usb_protocol"
    ),
)

import rocket_protocol as protocol  # noqa: E402
import run_variable_hil as runner  # noqa: E402
import variable_hil_store as storage  # noqa: E402


def deep_merge(base: dict, overlay: dict) -> dict:
    result = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


class VariableHilRunnerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.store = storage.VariableHilStore.create(
            Path(self.temporary.name),
            "timing-test",
            tick_hz=50.0,
            settings={"hardware": False},
            config_crc32=0x12345678,
        )
        self.addCleanup(self._close)
        self.case_id = "timing-test-case-00000000"
        self.hardware_id = "timing-test-hw-00000000"
        self.store.begin_run(
            simulation_case_id=self.case_id,
            hardware_run_id=self.hardware_id,
            case_index=0,
            input_config={"test": True},
            target_apogee_m=914.4,
            mode="nominal",
        )

    def _close(self) -> None:
        try:
            self.store.close()
        except Exception:
            pass

    @staticmethod
    def sample(time_s: float) -> runner.SensorSample:
        return runner.SensorSample(
            simulation_time_s=time_s,
            altitude_m=800.0,
            velocity_mps=50.0,
            acceleration_mps2=-9.0,
            truth_altitude_m=801.0,
            truth_velocity_mps=51.0,
            truth_acceleration_mps2=-8.5,
            barometer_stddev_m=1.5,
        )

    @staticmethod
    def serial_state(sequence: int, *, xactual_fraction: float = 0.25) -> dict:
        return {
            "simulation_sequence": sequence,
            "controller_requested_fraction": 0.30,
            "actuator_target_fraction": 0.30,
            "xactual_fraction": xactual_fraction,
            "target_steps": 46080,
            "actual_steps": round(153600 * xactual_fraction),
            "flight_inhibit_flags": 0,
            "actuator_inhibit_flags": 0,
            "driver_status": 0,
            "config_crc32": 0x12345678,
            "closed_predicted_apogee_m": 920.0,
            "full_predicted_apogee_m": 850.0,
            "target_reachable": True,
            "phase": 3,
            "machine_state": 3,
            "state_flags": 0,
            "feedback_source": runner.TMC_XACTUAL_SOURCE,
            "driver_ok": True,
            "config_valid": True,
            "simulation_active": True,
            "simulation_fresh": True,
            "armed": True,
        }

    @staticmethod
    def make_serial_exchange_transport():
        clock = runner.FakeClock()
        transport = object.__new__(runner.SerialVariableHilTransport)
        transport.clock = clock
        transport.protocol = protocol
        transport.prepared = True
        transport.stopped = False
        transport.states = []
        transport.expired_simulation_sequence = None
        transport.telemetry = None
        transport.target_apogee_m = 914.4
        writes = []
        transport._write = lambda frame: writes.append((clock.now(), frame))
        return clock, transport, writes

    def make_stepper(
        self,
        *,
        response_delays=(),
        sequence_offsets=None,
        disconnect_ticks=(),
        feedback_age_overrides=None,
        xactual_fractions=(),
        transport_crc=0x12345678,
        expected_crc=0x12345678,
    ):
        clock = runner.FakeClock()
        transport = runner.FakeVariableHilTransport(
            clock,
            config_crc32=transport_crc,
            response_delays_s=response_delays,
            sequence_offsets=sequence_offsets,
            disconnect_ticks=disconnect_ticks,
            feedback_age_overrides_s=feedback_age_overrides,
            xactual_fractions=xactual_fractions,
        )
        stepper = runner.CausalVariableHilStepper(
            transport,
            self.store,
            self.hardware_id,
            clock=clock,
            expected_config_crc32=expected_crc,
        )
        return clock, transport, stepper

    def test_confirmed_xactual_is_tagged_for_next_physics_interval(self) -> None:
        _, _, stepper = self.make_stepper(xactual_fractions=(0.25, 0.50))
        first = stepper.tick(self.sample(0.0))
        second = stepper.tick(self.sample(0.02))
        self.assertEqual(first.physics_applied_fraction, 0.25)
        self.assertEqual(second.physics_applied_fraction, 0.50)
        rows = list(
            self.store.connection.execute(
                "SELECT tick_index,actuator_xactual_fraction,physics_applied_fraction,"
                "physics_applies_to_tick_index,forced_hil_deployment_fraction "
                "FROM samples ORDER BY tick_index"
            )
        )
        self.assertEqual(rows[0]["physics_applies_to_tick_index"], 1)
        self.assertEqual(rows[1]["physics_applies_to_tick_index"], 2)
        self.assertTrue(all(row["forced_hil_deployment_fraction"] is None for row in rows))

    def test_one_missed_reply_uses_last_fraction_and_fails_timing_only(self) -> None:
        _, transport, stepper = self.make_stepper(
            response_delays=(0.0, None, 0.0),
            xactual_fractions=(0.30,),
        )
        first = stepper.tick(self.sample(0.0))
        missed = stepper.tick(self.sample(0.02))
        recovered = stepper.tick(self.sample(0.04))
        self.assertEqual(first.physics_applied_fraction, 0.30)
        self.assertEqual(missed.physics_applied_fraction, 0.30)
        self.assertTrue(missed.late_tick)
        self.assertFalse(recovered.late_tick)
        self.assertFalse(stepper.performance_timing_passed)
        self.assertEqual(stepper.late_tick_count, 1)
        self.assertEqual(transport.sim_stop_count, 0)
        self.assertEqual(transport.exchange_sequences, [0, 1, 2])

    def test_missed_reply_rebases_and_next_exchange_gets_full_window(self) -> None:
        clock, transport, stepper = self.make_stepper(
            response_delays=(0.0, None, 0.010),
            xactual_fractions=(0.30, 0.40, 0.50),
        )
        stepper.tick(self.sample(0.0))
        missed = stepper.tick(self.sample(0.02))
        # Reproduce the RocketPy/store overhead measured after the live miss.
        clock.advance(0.0119)
        recovered = stepper.tick(self.sample(0.04))

        self.assertTrue(missed.late_tick)
        self.assertFalse(recovered.late_tick)
        self.assertEqual(recovered.physics_applied_fraction, 0.50)
        self.assertEqual(stepper.consecutive_misses, 0)
        self.assertEqual(transport.sim_stop_count, 0)
        self.assertTrue(
            all(
                abs(budget - runner.TICK_PERIOD_S) < 1e-12
                for budget in transport.exchange_deadline_budgets_s
            )
        )
        self.assertTrue(
            all(
                later - earlier >= runner.TICK_PERIOD_S
                for earlier, later in zip(
                    transport.exchange_sent_at_s,
                    transport.exchange_sent_at_s[1:],
                )
            )
        )

    def test_no_catchup_burst_after_host_clock_jump(self) -> None:
        clock, transport, stepper = self.make_stepper(xactual_fractions=(0.1, 0.2))
        stepper.tick(self.sample(0.0))
        clock.advance(0.05)
        missed = stepper.tick(self.sample(0.02))
        recovered = stepper.tick(self.sample(0.04))
        self.assertTrue(missed.late_tick)
        self.assertFalse(recovered.late_tick)
        # The overdue tick sent nothing; no compressed sequence burst occurred.
        self.assertEqual(transport.exchange_sequences, [0, 1])

    def test_two_consecutive_misses_stop_disarm_and_fault(self) -> None:
        _, transport, stepper = self.make_stepper(response_delays=(None, None))
        first = stepper.tick(self.sample(0.0))
        self.assertTrue(first.late_tick)
        with self.assertRaisesRegex(runner.VariableHilSafetyFault, "two consecutive"):
            stepper.tick(self.sample(0.02))
        self.assertEqual(transport.sim_stop_count, 1)
        self.assertEqual(transport.disarm_count, 1)
        self.assertEqual(stepper.last_confirmed_fraction, 0.0)

    def test_feedback_older_than_100ms_faults_and_cleans_up(self) -> None:
        _, transport, stepper = self.make_stepper(
            feedback_age_overrides={0: 0.101}
        )
        with self.assertRaisesRegex(runner.VariableHilSafetyFault, "exceeds 100 ms"):
            stepper.tick(self.sample(0.0))
        self.assertEqual((transport.sim_stop_count, transport.disarm_count), (1, 1))

    def test_usb_loss_faults_and_cleans_up(self) -> None:
        _, transport, stepper = self.make_stepper(disconnect_ticks=(0,))
        with self.assertRaisesRegex(runner.VariableHilSafetyFault, "disconnect"):
            stepper.tick(self.sample(0.0))
        self.assertEqual((transport.sim_stop_count, transport.disarm_count), (1, 1))

    def test_protocol_decoder_fault_stops_and_disarms(self) -> None:
        _, transport, stepper = self.make_stepper()

        def corrupt_reply(sequence, sample, *, deadline_s):
            del sequence, sample, deadline_s
            raise runner.VariableHilSafetyFault("USB stream decoder rejected packet")

        transport.exchange = corrupt_reply
        with self.assertRaisesRegex(runner.VariableHilSafetyFault, "decoder"):
            stepper.tick(self.sample(0.0))
        self.assertEqual((transport.sim_stop_count, transport.disarm_count), (1, 1))

    def test_sequence_mismatch_faults_and_cleans_up(self) -> None:
        _, transport, stepper = self.make_stepper(sequence_offsets={0: 1})
        with self.assertRaisesRegex(runner.VariableHilSafetyFault, "sequence mismatch"):
            stepper.tick(self.sample(0.0))
        self.assertEqual((transport.sim_stop_count, transport.disarm_count), (1, 1))

    def test_header_payload_sequence_mismatch_faults_and_cleans_up(self) -> None:
        _, transport, stepper = self.make_stepper()
        exchange = transport.exchange

        def wrong_header(sequence, sample, *, deadline_s):
            feedback = exchange(sequence, sample, deadline_s=deadline_s)
            assert feedback is not None
            return replace(
                feedback,
                packet_sequence=(feedback.packet_sequence + 1) & 0xFFFF,
            )

        transport.exchange = wrong_header
        with self.assertRaisesRegex(runner.VariableHilSafetyFault, "correlation mismatch"):
            stepper.tick(self.sample(0.0))
        self.assertEqual((transport.sim_stop_count, transport.disarm_count), (1, 1))

    def test_driver_not_ok_faults_and_cleans_up(self) -> None:
        _, transport, stepper = self.make_stepper()
        exchange = transport.exchange

        def driver_fault(sequence, sample, *, deadline_s):
            feedback = exchange(sequence, sample, deadline_s=deadline_s)
            assert feedback is not None
            return replace(feedback, driver_ok=False)

        transport.exchange = driver_fault
        with self.assertRaisesRegex(runner.VariableHilSafetyFault, "driver_ok"):
            stepper.tick(self.sample(0.0))
        self.assertEqual((transport.sim_stop_count, transport.disarm_count), (1, 1))

    def test_config_crc_mismatch_faults_and_cleans_up(self) -> None:
        _, transport, stepper = self.make_stepper(
            transport_crc=0x87654321,
            expected_crc=0x12345678,
        )
        with self.assertRaisesRegex(runner.VariableHilSafetyFault, "config CRC32 mismatch"):
            stepper.tick(self.sample(0.0))
        self.assertEqual((transport.sim_stop_count, transport.disarm_count), (1, 1))

    def test_rocketpy_callback_applies_reply_after_current_sensor_sample(self) -> None:
        _, _, stepper = self.make_stepper(xactual_fractions=(0.2, 0.4))

        class IdentitySensor:
            values = {"barometer_measurement_std_dev_m": 1.5}

            @staticmethod
            def altitude(value):
                return value

            @staticmethod
            def acceleration(value):
                return value

        controller = runner.VariableHilRocketPyController(
            stepper,
            sensor_model=IdentitySensor(),
            environment_elevation_m=0.0,
            acceleration_projector=lambda acceleration, axis, pad: acceleration[2],
            body_axis=lambda state: (0.0, 0.0, 1.0),
        )
        brakes = SimpleNamespace(deployment_level=0.0)
        state = [0.0, 0.0, 100.0, 0.0, 0.0, 20.0, 1.0, 0.0, 0.0, 0.0]
        controller(0.0, 50.0, state, [], [], brakes, [], None)
        self.assertEqual(brakes.deployment_level, 0.2)
        state[2] = 100.4
        controller(0.02, 50.0, state, [], [], brakes, [], None)
        self.assertEqual(brakes.deployment_level, 0.4)
        self.assertEqual(controller.log[0]["simulation_time_s"], 0.0)
        self.assertEqual(controller.log[1]["simulation_time_s"], 0.02)

    def test_hardware_gate_rejects_partial_or_implicit_opt_in(self) -> None:
        missing = ROOT / "does-not-exist-variable-config.json"
        with self.assertRaisesRegex(runner.VariableHilError, "COM/motion options"):
            runner.validate_hardware_opt_in(
                hardware=False,
                allow_actuator_motion=True,
                accept_current_position_home=False,
                port_name=None,
                variable_config_path=None,
            )
        with self.assertRaisesRegex(runner.VariableHilError, "missing"):
            runner.validate_hardware_opt_in(
                hardware=True,
                allow_actuator_motion=False,
                accept_current_position_home=False,
                port_name=None,
                variable_config_path=missing,
            )

    def test_actuator_driver_flag_does_not_confuse_build_with_motor_energy(self) -> None:
        self.assertFalse(
            runner.actuator_status_driver_enabled({"flags": 0x01})
        )
        self.assertTrue(
            runner.actuator_status_driver_enabled(
                {"flags": runner.ACTUATOR_FLAG_DRIVER_ENABLED}
            )
        )
        self.assertFalse(
            runner.actuator_status_driver_enabled(
                {"flags": 0x01 | 0x02 | 0x04 | 0x08}
            )
        )

    def test_serial_prepare_primes_fresh_input_before_arm(self) -> None:
        transport = object.__new__(runner.SerialVariableHilTransport)
        calls = []
        transport.protocol = protocol
        transport.prepared = False
        transport._require_preflight = lambda: calls.append("preflight")

        def upload():
            calls.append("upload")
            return {"config_crc32": 0x12345678}

        transport._upload_and_verify_config = upload

        def command(command, data=b"", timeout_s=1.0):
            del timeout_s
            calls.append((command, bytes(data)))
            return len(calls)

        transport._command = command
        transport._prime_fresh_simulation_input_before_arm = (
            lambda *, expected_config_crc32: calls.append(
                ("prime", expected_config_crc32)
            )
        )

        decoded = transport.prepare()
        self.assertEqual(decoded["config_crc32"], 0x12345678)
        self.assertEqual(
            calls,
            [
                "preflight",
                "upload",
                (protocol.CMD_SIM_START, b""),
                ("prime", 0x12345678),
                (protocol.CMD_SET_ARMED, b"\x01"),
            ],
        )
        self.assertTrue(transport.prepared)

    def test_serial_exchange_discards_one_late_previous_reply_and_recovers(self) -> None:
        clock, transport, writes = self.make_serial_exchange_transport()
        transport._poll = lambda: clock.advance(runner.TICK_PERIOD_S)

        missed = transport.exchange(
            103,
            self.sample(2.06),
            deadline_s=clock.now() + runner.TICK_PERIOD_S,
        )
        self.assertIsNone(missed)
        self.assertEqual(transport.expired_simulation_sequence, 103)

        queued = False

        def queue_late_then_current() -> None:
            nonlocal queued
            if queued:
                return
            queued = True
            transport.states.extend(
                [
                    (103, self.serial_state(103, xactual_fraction=0.25), clock.now()),
                    (104, self.serial_state(104, xactual_fraction=0.40), clock.now()),
                ]
            )

        transport._poll = queue_late_then_current
        recovered = transport.exchange(
            104,
            self.sample(2.08),
            deadline_s=clock.now() + runner.TICK_PERIOD_S,
        )

        self.assertIsNotNone(recovered)
        assert recovered is not None
        self.assertEqual(recovered.packet_sequence, 104)
        self.assertEqual(recovered.simulation_sequence, 104)
        self.assertEqual(recovered.actuator_xactual_fraction, 0.40)
        self.assertIsNone(transport.expired_simulation_sequence)
        self.assertEqual(transport.states, [])
        self.assertEqual(len(writes), 2)
        self.assertGreaterEqual(writes[1][0] - writes[0][0], runner.TICK_PERIOD_S)

    def test_serial_exchange_late_previous_only_remains_a_miss(self) -> None:
        clock, transport, writes = self.make_serial_exchange_transport()
        transport.expired_simulation_sequence = 103
        queued = False

        def queue_only_late_previous() -> None:
            nonlocal queued
            if not queued:
                queued = True
                transport.states.append(
                    (103, self.serial_state(103), clock.now())
                )

        transport._poll = queue_only_late_previous
        feedback = transport.exchange(
            104,
            self.sample(2.08),
            deadline_s=clock.now() + runner.TICK_PERIOD_S,
        )

        self.assertIsNone(feedback)
        self.assertEqual(transport.expired_simulation_sequence, 104)
        self.assertEqual(len(writes), 1)
        self.assertGreaterEqual(clock.now(), runner.TICK_PERIOD_S)

    def test_serial_exchange_does_not_filter_other_sequence_mismatch(self) -> None:
        clock, transport, _ = self.make_serial_exchange_transport()
        transport.expired_simulation_sequence = 103
        queued = False

        def queue_unrelated_sequence() -> None:
            nonlocal queued
            if not queued:
                queued = True
                transport.states.append(
                    (102, self.serial_state(102), clock.now())
                )

        transport._poll = queue_unrelated_sequence
        feedback = transport.exchange(
            104,
            self.sample(2.08),
            deadline_s=clock.now() + runner.TICK_PERIOD_S,
        )
        self.assertIsNotNone(feedback)
        assert feedback is not None

        stepper = object.__new__(runner.CausalVariableHilStepper)
        stepper.maximum_feedback_age_s = runner.MAX_FEEDBACK_AGE_S
        stepper.maximum_feedback_age_observed_s = 0.0
        stepper.latched_config_crc32 = 0x12345678
        with self.assertRaisesRegex(runner.VariableHilSafetyFault, "sequence mismatch"):
            stepper._validate_feedback(
                feedback,
                expected_sequence=104,
                sent_at_s=clock.now(),
            )

    def test_serial_exchange_does_not_filter_malformed_late_reply(self) -> None:
        clock, transport, _ = self.make_serial_exchange_transport()
        transport.expired_simulation_sequence = 103
        queued = False

        def queue_header_payload_mismatch() -> None:
            nonlocal queued
            if not queued:
                queued = True
                transport.states.append(
                    (103, self.serial_state(102), clock.now())
                )

        transport._poll = queue_header_payload_mismatch
        feedback = transport.exchange(
            104,
            self.sample(2.08),
            deadline_s=clock.now() + runner.TICK_PERIOD_S,
        )
        self.assertIsNotNone(feedback)
        assert feedback is not None

        stepper = object.__new__(runner.CausalVariableHilStepper)
        stepper.maximum_feedback_age_s = runner.MAX_FEEDBACK_AGE_S
        stepper.maximum_feedback_age_observed_s = 0.0
        stepper.latched_config_crc32 = 0x12345678
        with self.assertRaisesRegex(runner.VariableHilSafetyFault, "correlation mismatch"):
            stepper._validate_feedback(
                feedback,
                expected_sequence=104,
                sent_at_s=clock.now(),
            )

    def test_prearm_prime_requires_correlated_fresh_home_zero_state(self) -> None:
        transport = object.__new__(runner.SerialVariableHilTransport)
        transport.protocol = protocol
        transport.clock = runner.FakeClock()
        transport.command_sequence = 42
        frames = []
        transport._write = frames.append
        transport._poll = lambda: None
        transport.states = [
            (
                42,
                {
                    "simulation_sequence": 42,
                    "config_valid": True,
                    "simulation_active": True,
                    "simulation_fresh": True,
                    "software_home": True,
                    "driver_ok": True,
                    "armed": False,
                    "driver_enabled": False,
                    "config_crc32": 0x12345678,
                    "target_steps": 0,
                    "actual_steps": 0,
                },
                0.0,
            )
        ]

        transport._prime_fresh_simulation_input_before_arm(
            expected_config_crc32=0x12345678
        )
        self.assertEqual(transport.command_sequence, 43)
        self.assertEqual(transport.states, [])
        self.assertEqual(len(frames), 1)

    def test_fake_main_and_rejected_hardware_gate_never_import_serial(self) -> None:
        results_root = Path(self.temporary.name) / "main-results"
        with mock.patch.object(runner.importlib, "import_module") as import_module:
            result = runner.main(
                [
                    "--session-id",
                    "safe-fake-main",
                    "--results-root",
                    str(results_root),
                    "--cycles",
                    "1",
                    "--max-time-s",
                    "0.02",
                    "--dwell-s",
                    "0",
                ]
            )
            self.assertEqual(result, 2)
            import_module.assert_not_called()

        with mock.patch.object(runner.importlib, "import_module") as import_module:
            with self.assertRaises(SystemExit):
                runner.main(["--hardware", "--port", "COM99"])
            import_module.assert_not_called()

    def test_m5_overlay_builds_versioned_atomic_config(self) -> None:
        base = json.loads(
            (ROOT / "sim" / "rocketpy" / "ambar_reference_config.json").read_text(
                encoding="utf-8"
            )
        )
        overlay = json.loads(
            (ROOT / "sim" / "rocketpy" / "variable_hil_m5_config.json").read_text(
                encoding="utf-8"
            )
        )
        merged = deep_merge(base, overlay)
        config, payload, decoded = runner.build_variable_hil_config(merged, protocol)
        self.assertEqual(decoded["calibration_version"], 20260719)
        self.assertEqual(decoded["control_mode"], 1)
        self.assertEqual(decoded["predictor_mode"], 1)
        self.assertEqual(len(decoded["deployment_cda_m2"]), 5)
        self.assertNotEqual(decoded["config_crc32"], 0)
        self.assertEqual(payload, protocol.encode_variable_hil_config_payload(config))

    def test_case_plan_repeats_low_nominal_high_without_randomness(self) -> None:
        self.assertEqual(
            runner.build_case_names(7, "low,nominal,high"),
            ["low", "nominal", "high", "low", "nominal", "high", "low"],
        )


if __name__ == "__main__":
    unittest.main()
