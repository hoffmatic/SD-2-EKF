"""Regression tests for the additive VARIABLE_HIL evidence store."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "scripts"))

import variable_hil_store as storage  # noqa: E402


class VariableHilStoreTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.store = storage.VariableHilStore.create(
            self.root,
            "variable-test",
            tick_hz=50.0,
            settings={"hardware": False},
            config_crc32=0x12345678,
        )
        self.addCleanup(self._close)

    def _close(self) -> None:
        try:
            self.store.close()
        except Exception:
            pass

    def _begin(self, index: int = 0) -> tuple[str, str]:
        case_id = f"variable-test-case-{index:08d}"
        hardware_id = f"variable-test-hw-{index:08d}"
        self.store.begin_run(
            simulation_case_id=case_id,
            hardware_run_id=hardware_id,
            case_index=index,
            input_config={"case": index},
            target_apogee_m=914.4,
            mode="nominal",
        )
        return case_id, hardware_id

    @staticmethod
    def _sample() -> dict[str, object]:
        return {
            "tick_index": 0,
            "scheduled_time_s": 0.0,
            "host_elapsed_s": 0.001,
            "simulation_time_s": 0.0,
            "schedule_lag_s": 0.0,
            "late_tick": 0,
            "late_tick_count": 0,
            "consecutive_misses": 0,
            "sensor_altitude_m": 10.0,
            "sensor_velocity_mps": 20.0,
            "sensor_acceleration_mps2": -1.0,
            "truth_altitude_m": 10.5,
            "truth_velocity_mps": 20.5,
            "truth_acceleration_mps2": -0.9,
            "controller_request_fraction": 0.6,
            "actuator_target_fraction": 0.5,
            "actuator_target_steps": 76800,
            "actuator_xactual_fraction": 0.4,
            "actuator_xactual_steps": 61440,
            "physics_applied_fraction": 0.4,
            "forced_hil_deployment_fraction": None,
            "feedback_source": "TMC5240_XACTUAL",
            "feedback_source_code": 1,
            "feedback_age_s": 0.004,
            "feedback_age_ms": 4.0,
            "board_sequence_sent": 7,
            "board_sequence_confirmed": 7,
            "config_crc32": 0x12345678,
            "flight_inhibit_flags": 0,
            "actuator_inhibit_flags": 0,
            "driver_status": 0,
            "phase": 3,
            "machine_state": 8,
            "state_flags": 0xBD,
            "hardware_ok": 1,
            "performance_ok": 1,
            "fault_active": 0,
        }

    def test_four_channels_are_distinct_and_forced_field_is_null(self) -> None:
        _, hardware_id = self._begin()
        self.store.record_sample(hardware_id, self._sample())
        row = self.store.connection.execute(
            "SELECT * FROM samples WHERE hardware_run_id=?", (hardware_id,)
        ).fetchone()
        self.assertEqual(row["controller_request_fraction"], 0.6)
        self.assertEqual(row["actuator_target_fraction"], 0.5)
        self.assertEqual(row["actuator_xactual_fraction"], 0.4)
        self.assertEqual(row["physics_applied_fraction"], 0.4)
        self.assertEqual(row["physics_applies_to_tick_index"], 1)
        self.assertIsNone(row["forced_hil_deployment_fraction"])
        self.assertEqual(row["feedback_age_ms"], 4.0)

    def test_live_store_does_not_force_disk_sync_or_checkpoint_in_control_loop(self) -> None:
        self.assertEqual(
            self.store.connection.execute("PRAGMA synchronous").fetchone()[0], 1
        )
        self.assertEqual(
            self.store.connection.execute("PRAGMA wal_autocheckpoint").fetchone()[0],
            0,
        )

    def test_nonnull_forced_fraction_is_rejected(self) -> None:
        _, hardware_id = self._begin()
        sample = self._sample()
        sample["forced_hil_deployment_fraction"] = 1.0
        with self.assertRaisesRegex(ValueError, "cannot persist"):
            self.store.record_sample(hardware_id, sample)

    def test_run_finalization_leaves_session_open_until_campaign_gate(self) -> None:
        case_id, hardware_id = self._begin()
        self.store.finalize_run(
            simulation_case_id=case_id,
            hardware_run_id=hardware_id,
            status="SIMULATED",
            hardware_verdict={
                "status": "NOT_APPLICABLE",
                "passed": None,
                "tracking_pass": False,
                "cleanup": {"success": True},
            },
            performance_verdict={
                "status": "PASS",
                "passed": True,
                "target_tolerance_m": 30.48,
            },
            truth_apogee_m=914.4,
            target_apogee_m=914.4,
        )
        self.assertEqual(self.store.session["status"], "RUNNING")
        result = self.store.finalize_session(
            expected_runs=1,
            hardware_required=False,
            stop_reason="one-run complete",
        )
        self.assertEqual(result["hardware"]["status"], "NOT_APPLICABLE")
        self.assertTrue(result["performance"]["passed"])
        self.assertEqual(self.store.session["status"], "COMPLETE")

    def test_25_cycle_campaign_computes_independent_acceptance_gates(self) -> None:
        for index in range(25):
            case_id, hardware_id = self._begin(index)
            # Exactly one target miss at +150 ft: 24/25 in band, p95 remains
            # below 100 ft, and worst remains below 200 ft.
            error_m = 45.72 if index == 24 else 0.0
            self.store.finalize_run(
                simulation_case_id=case_id,
                hardware_run_id=hardware_id,
                status="PASS",
                hardware_verdict={
                    "status": "PASS",
                    "passed": True,
                    "tracking_pass": True,
                    "protocol_failure": False,
                    "cleanup": {"success": True},
                },
                performance_verdict={
                    "status": "PASS" if index != 24 else "FAIL",
                    "passed": index != 24,
                    "target_tolerance_m": 30.48,
                },
                truth_apogee_m=914.4 + error_m,
                target_apogee_m=914.4,
            )
        result = self.store.finalize_session(
            expected_runs=25,
            hardware_required=True,
            stop_reason="qualification complete",
        )
        self.assertTrue(result["hardware"]["passed"])
        self.assertTrue(result["performance"]["passed"])
        self.assertEqual(result["performance"]["in_band_runs"], 24)
        self.assertLessEqual(result["performance"]["p95_absolute_error_ft"], 100.0)
        self.assertLessEqual(result["performance"]["worst_absolute_error_ft"], 200.0)
        session = self.store.session
        self.assertEqual(session["hardware_safety_verdict"], "PASS")
        self.assertEqual(session["performance_verdict"], "PASS")

    def test_cleanup_failure_fails_only_hardware_campaign_gate(self) -> None:
        case_id, hardware_id = self._begin()
        self.store.finalize_run(
            simulation_case_id=case_id,
            hardware_run_id=hardware_id,
            status="FAULTED",
            hardware_verdict={
                "status": "FAIL",
                "passed": False,
                "tracking_pass": False,
                "cleanup": {"success": False},
            },
            performance_verdict={
                "status": "PASS",
                "passed": True,
                "target_tolerance_m": 30.48,
            },
            truth_apogee_m=914.4,
            target_apogee_m=914.4,
        )
        result = self.store.finalize_session(
            expected_runs=1,
            hardware_required=True,
            stop_reason="cleanup fault",
        )
        self.assertFalse(result["hardware"]["passed"])
        self.assertTrue(result["performance"]["passed"])
        self.assertEqual(self.store.session["status"], "FAULTED")

    def test_dashboard_table_contract_is_present(self) -> None:
        tables = {
            row[0]
            for row in self.store.connection.execute(
                "SELECT name FROM sqlite_master WHERE type='table'"
            )
        }
        self.assertTrue(
            {"sessions", "simulation_cases", "hardware_runs", "samples"}
            <= tables
        )
        columns = {
            row[1]
            for row in self.store.connection.execute("PRAGMA table_info(samples)")
        }
        for name in (
            "controller_request_fraction",
            "actuator_target_fraction",
            "actuator_feedback_deployment_fraction",
            "physics_applied_fraction",
            "forced_hil_deployment_fraction",
            "feedback_age_ms",
            "config_crc32",
        ):
            self.assertIn(name, columns)


if __name__ == "__main__":
    unittest.main()
