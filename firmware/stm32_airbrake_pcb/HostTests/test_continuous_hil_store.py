"""Regression tests for deterministic continuous-HIL planning and storage."""

from __future__ import annotations

import json
from pathlib import Path
import sqlite3
import sys
import tempfile
from types import SimpleNamespace
import unittest


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

import continuous_hil_store as storage  # noqa: E402
import replay_reporting as reporting  # noqa: E402
import run_continuous_hil as supervisor  # noqa: E402


class ContinuousHilStoreTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.store = storage.ContinuousHilStore.create(
            self.root,
            "test-session",
            master_seed=1234,
            batch_size=50,
            baseline_interval=10,
            dwell_s=30.0,
            settings={"serial_port": "COM_TEST"},
        )
        self.addCleanup(self._close_store)

    def _close_store(self) -> None:
        try:
            self.store.close()
        except Exception:
            pass

    def _plan(self) -> str:
        simulation_run_id = "test-session-sim-00000000"
        self.store.add_simulation_plan(
            {
                "simulation_run_id": simulation_run_id,
                "cycle_index": 0,
                "random_index": 0,
                "batch_index": 0,
                "batch_position": 0,
                "mode": "monte_carlo",
                "run_seed": 55,
                "sampled_inputs": {"mass": 3.0},
            }
        )
        return simulation_run_id

    def _write_complete_simulation(self, simulation_run_id: str) -> Path:
        sim_dir = self.store.session_dir / "simulation_runs" / simulation_run_id
        sim_dir.mkdir(parents=True, exist_ok=True)
        profile = sim_dir / "rocketpy_profile.csv"
        timeseries = sim_dir / "sil_timeseries.csv.gz"
        resolved = sim_dir / "resolved_config.json"
        summary = sim_dir / "sil_summary.json"
        profile.write_text(
            "Time (s),Altitude (m),Vertical velocity (m/s),"
            "Vertical acceleration (m/s^2)\n0,0,0,0\n",
            encoding="utf-8",
        )
        timeseries.write_bytes(b"timeseries")
        resolved.write_text(
            json.dumps({"motor": {"burn_time_s": 1.64}}),
            encoding="utf-8",
        )
        summary.write_text(
            json.dumps(
                {
                    "run_id": simulation_run_id,
                    "run_index": 0,
                    "sample_index": 0,
                    "mode": "monte_carlo",
                    "run_seed": 55,
                    "status": "COMPLETE",
                    "config_sha256": "ABC",
                    "run_pass": "true",
                }
            ),
            encoding="utf-8",
        )
        self.store.begin_simulation(simulation_run_id)
        self.store.complete_simulation(
            simulation_run_id,
            resolved_config={"motor": {"burn_time_s": 1.64}},
            resolved_config_sha256="ABC",
            sil_row=json.loads(summary.read_text(encoding="utf-8")),
            profile_path=profile,
            sil_timeseries_path=timeseries,
        )
        return sim_dir

    @staticmethod
    def _sample_event(
        session_id: str,
        simulation_run_id: str,
        hardware_run_id: str,
        event_index: int = 1,
    ) -> dict[str, object]:
        return {
            "schema": "ambar.live_replay_event.v1",
            "session_id": session_id,
            "hardware_run_id": hardware_run_id,
            "simulation_run_id": simulation_run_id,
            "event_index": event_index,
            "event": "sample_observed",
            "host_elapsed_s": 0.5,
            "sample_index": 1,
            "replay_time_s": 0.5,
            "source_time_s": 0.0,
            "truth_altitude_m": 1.0,
            "truth_velocity_mps": 2.0,
            "truth_acceleration_mps2": 3.0,
            "sil": {
                "estimated_altitude_m": 0.9,
                "estimated_velocity_mps": 1.9,
                "predicted_apogee_m": 100.0,
                "command_fraction": 0.25,
                "actual_deployment_fraction": 0.2,
            },
            "stm32_altitude_m": 0.8,
            "stm32_velocity_mps": 1.8,
            "stm32_predicted_apogee_m": 99.0,
            "raw_controller_deployment_fraction": 0.2,
            "forced_hil_deployment_fraction": 1.0,
            "hil_override_mode": 1,
            "motor_target_steps": 153600,
            "motor_actual_steps": 153500,
            "motor_tracking_error_steps": 100,
            "home_active": False,
            "full_active": True,
            "limits_plausible": True,
            "endpoint_sequence_verified": False,
            "stm32_phase": 3,
            "driver_status": 0,
        }

    def test_database_uses_wal_and_exports_finalized_run(self) -> None:
        self.assertEqual(
            self.store.connection.execute("PRAGMA journal_mode").fetchone()[0],
            "wal",
        )
        simulation_run_id = self._plan()
        self._write_complete_simulation(simulation_run_id)

        hardware_run_id = "test-session-hw-00000000-a01"
        replay_dir = self.store.session_dir / "runs" / hardware_run_id / "replay"
        replay_dir.mkdir(parents=True)
        self.store.begin_hardware_run(
            simulation_run_id,
            hardware_run_id,
            replay_dir,
        )
        self.store.record_event(
            self._sample_event(
                self.store.session_id,
                simulation_run_id,
                hardware_run_id,
                event_index=0,
            )
        )
        verdict = {
            "status": "PASS",
            "passed": True,
            "metrics": {
                "open_time_s": 1.2,
                "close_time_s": 1.3,
                "max_actuator_tracking_error_steps": 100,
                "full_status": {"actual_steps": 153600},
                "home_status": {"actual_steps": 0},
            },
        }
        self.store.finalize_hardware_run(
            hardware_run_id,
            status="PASS",
            verdict=verdict,
        )

        sample = self.store.connection.execute(
            "SELECT * FROM samples WHERE hardware_run_id=?",
            (hardware_run_id,),
        ).fetchone()
        self.assertEqual(sample["sil_raw_controller_fraction"], 0.25)
        self.assertEqual(sample["sil_actual_deployment_fraction"], 0.2)
        self.assertEqual(sample["motor_actual_steps"], 153500)
        self.assertTrue((self.store.session_dir / "runs.csv").is_file())
        self.assertTrue((self.store.session_dir / "parameters.csv").is_file())
        self.assertTrue(
            (
                self.store.session_dir
                / "runs"
                / hardware_run_id
                / "telemetry.csv.gz"
            ).is_file()
        )

    def test_open_adds_virtual_deployment_column_to_legacy_v1_database(
        self,
    ) -> None:
        session_dir = self.store.session_dir
        self.store.close()
        legacy = sqlite3.connect(
            session_dir / "campaign.sqlite3",
            isolation_level=None,
        )
        try:
            legacy.execute(
                "ALTER TABLE samples DROP COLUMN "
                "sil_actual_deployment_fraction"
            )
        finally:
            legacy.close()

        self.store = storage.ContinuousHilStore(session_dir)
        columns = {
            str(row[1])
            for row in self.store.connection.execute(
                'PRAGMA table_info("samples")'
            )
        }
        self.assertIn("sil_actual_deployment_fraction", columns)

    def test_sample_metadata_preserves_virtual_actuator_state(self) -> None:
        profile = SimpleNamespace(
            samples=(SimpleNamespace(source_time_s=1.0),)
        )
        metadata = supervisor.build_sample_metadata(
            profile,
            [
                {
                    "time_s": 1.0,
                    "command_fraction": 0.5,
                    "delayed_desired_deployment_fraction": 0.4,
                    "actual_deployment_fraction": 0.3,
                }
            ],
        )
        self.assertEqual(
            metadata,
            [
                {
                    "time_s": 1.0,
                    "command_fraction": 0.5,
                    "delayed_desired_deployment_fraction": 0.4,
                    "actual_deployment_fraction": 0.3,
                }
            ],
        )

    def test_async_event_sink_uses_separate_connection_and_drains_to_sqlite(
        self,
    ) -> None:
        simulation_run_id = self._plan()
        hardware_run_id = "test-session-hw-00000000-a01"
        replay_dir = self.store.session_dir / "runs" / hardware_run_id / "replay"
        replay_dir.mkdir(parents=True)
        self.store.begin_hardware_run(
            simulation_run_id,
            hardware_run_id,
            replay_dir,
        )
        sink = self.store.create_event_sink(
            minimum_free_bytes=0,
            commit_interval_s=60.0,
        )
        observer = reporting.ReplayRunObserver(
            clock=reporting.PerfCounterClock(),
            bundle_dir=replay_dir,
            event_sink=sink,
            event_log_name="events.jsonl",
            record_context={
                "session_id": self.store.session_id,
                "simulation_run_id": simulation_run_id,
                "hardware_run_id": hardware_run_id,
            },
        )
        observer.start({"test": True})
        observer.emit(
            "sample_observed",
            **{
                key: value
                for key, value in self._sample_event(
                    self.store.session_id,
                    simulation_run_id,
                    hardware_run_id,
                ).items()
                if key
                not in {
                    "schema",
                    "session_id",
                    "simulation_run_id",
                    "hardware_run_id",
                    "event_index",
                    "event",
                    "host_elapsed_s",
                }
            },
        )
        observer.finalize({"status": "PASS", "passed": True})

        events = self.store.connection.execute(
            """
            SELECT event_index, event FROM events
            WHERE hardware_run_id=? ORDER BY event_index
            """,
            (hardware_run_id,),
        ).fetchall()
        samples = self.store.connection.execute(
            "SELECT COUNT(*) FROM samples WHERE hardware_run_id=?",
            (hardware_run_id,),
        ).fetchone()[0]
        self.assertEqual(
            [(row["event_index"], row["event"]) for row in events],
            [
                (0, "run_started"),
                (1, "sample_observed"),
                (2, "run_verdict"),
            ],
        )
        self.assertEqual(samples, 1)
        self.assertTrue(
            (
                self.store.session_dir
                / "simulation_runs"
                / simulation_run_id
                / storage.SIMULATION_MANIFEST
            ).is_file()
        )
        self.assertTrue(
            (
                self.store.session_dir
                / "runs"
                / hardware_run_id
                / storage.HARDWARE_MANIFEST
            ).is_file()
        )

    def test_recovery_truncates_partial_json_and_aborts_active_attempt(self) -> None:
        simulation_run_id = self._plan()
        hardware_run_id = "test-session-hw-00000000-a01"
        replay_dir = self.store.session_dir / "runs" / hardware_run_id / "replay"
        replay_dir.mkdir(parents=True)
        self.store.begin_hardware_run(
            simulation_run_id,
            hardware_run_id,
            replay_dir,
        )
        complete = {
            "schema": "ambar.live_replay_event.v1",
            "session_id": self.store.session_id,
            "simulation_run_id": simulation_run_id,
            "hardware_run_id": hardware_run_id,
            "event_index": 0,
            "event": "run_started",
            "host_elapsed_s": 0.0,
        }
        event_log = replay_dir / "events.jsonl"
        event_log.write_text(
            json.dumps(complete)
            + "\n"
            + '{"event_index":1,not-valid-json}\n',
            encoding="utf-8",
        )

        recovered = self.store.recover_interrupted()
        self.assertEqual(recovered, [hardware_run_id])
        self.assertEqual(
            event_log.read_text(encoding="utf-8"),
            json.dumps(complete) + "\n",
        )
        hardware = self.store.connection.execute(
            "SELECT status FROM hardware_runs WHERE hardware_run_id=?",
            (hardware_run_id,),
        ).fetchone()
        self.assertEqual(hardware["status"], "ABORTED_HOST_CRASH")
        self.assertEqual(
            self.store.connection.execute(
                "SELECT COUNT(*) FROM events WHERE hardware_run_id=?",
                (hardware_run_id,),
            ).fetchone()[0],
            1,
        )

    def test_planning_helpers_are_deterministic_and_insert_baselines(self) -> None:
        self.assertEqual(
            supervisor.derive_batch_seed(7, 2),
            supervisor.derive_batch_seed(7, 2),
        )
        self.assertNotEqual(
            supervisor.derive_batch_seed(7, 2),
            supervisor.derive_batch_seed(7, 3),
        )
        kinds = supervisor.interleave_run_kinds(50, 10)
        self.assertEqual(kinds.count("monte_carlo"), 50)
        self.assertEqual(kinds.count("baseline"), 5)
        self.assertEqual(kinds[10], "baseline")
        self.assertEqual(kinds[-1], "baseline")

    def test_explicit_rebuild_recovers_finalized_indexes_without_database(self) -> None:
        simulation_run_id = self._plan()
        self._write_complete_simulation(simulation_run_id)
        hardware_run_id = "test-session-hw-00000000-a01"
        hardware_dir = self.store.session_dir / "runs" / hardware_run_id
        replay_dir = hardware_dir / "replay"
        replay_dir.mkdir(parents=True)
        self.store.begin_hardware_run(
            simulation_run_id,
            hardware_run_id,
            replay_dir,
        )
        verdict = {
            "status": "PASS",
            "passed": True,
            "failure": None,
            "metrics": {
                "open_time_s": 1.2,
                "close_time_s": 1.3,
                "max_actuator_tracking_error_steps": 100,
                "full_status": {"actual_steps": 153600},
                "home_status": {"actual_steps": 0},
            },
        }
        records = [
            {
                "schema": "ambar.live_replay_event.v1",
                "session_id": self.store.session_id,
                "simulation_run_id": simulation_run_id,
                "hardware_run_id": hardware_run_id,
                "event_index": 0,
                "event": "run_started",
                "host_elapsed_s": 0.0,
            },
            self._sample_event(
                self.store.session_id,
                simulation_run_id,
                hardware_run_id,
            ),
            {
                "schema": "ambar.live_replay_event.v1",
                "session_id": self.store.session_id,
                "simulation_run_id": simulation_run_id,
                "hardware_run_id": hardware_run_id,
                "event_index": 2,
                "event": "run_verdict",
                "host_elapsed_s": 1.0,
                "verdict": verdict,
            },
        ]
        (replay_dir / "manifest.json").write_text(
            json.dumps(
                {
                    "schema": "ambar.live_replay_manifest.v1",
                    "status": "PASS",
                    "started_utc": "2026-07-16T12:00:00Z",
                    "finished_utc": "2026-07-16T12:01:00Z",
                    "run_metadata": {
                        "session_id": self.store.session_id,
                        "simulation_run_id": simulation_run_id,
                        "hardware_run_id": hardware_run_id,
                        "cycle_index": 0,
                        "mode": "monte_carlo",
                        "run_seed": 55,
                    },
                }
            ),
            encoding="utf-8",
        )
        (replay_dir / "events.jsonl").write_text(
            "".join(json.dumps(record) + "\n" for record in records),
            encoding="utf-8",
        )
        (replay_dir / "verdict.json").write_text(
            json.dumps(verdict),
            encoding="utf-8",
        )
        for record in records:
            self.store.record_event(record)
        self.store.finalize_hardware_run(
            hardware_run_id,
            status="PASS",
            verdict=verdict,
        )

        session_dir = self.store.session_dir
        self.store.close()
        for path in (
            session_dir / "campaign.sqlite3",
            Path(str(session_dir / "campaign.sqlite3") + "-wal"),
            Path(str(session_dir / "campaign.sqlite3") + "-shm"),
        ):
            path.unlink(missing_ok=True)
        rebuilt = storage.ContinuousHilStore.rebuild_database(session_dir)
        self.store = rebuilt

        self.assertEqual(self.store.session["status"], "STOPPED")
        self.assertEqual(
            self.store.connection.execute(
                "SELECT status FROM simulation_runs"
            ).fetchone()[0],
            "SIL_COMPLETE",
        )
        self.assertEqual(
            self.store.connection.execute(
                "SELECT status FROM hardware_runs"
            ).fetchone()[0],
            "PASS",
        )
        self.assertEqual(
            self.store.connection.execute(
                "SELECT COUNT(*) FROM events"
            ).fetchone()[0],
            3,
        )
        self.assertEqual(
            self.store.connection.execute(
                "SELECT COUNT(*) FROM samples"
            ).fetchone()[0],
            1,
        )
        report = json.loads(
            (session_dir / "last_database_rebuild.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertFalse(report["safety"]["home_inferred_after_restart"])
        self.assertEqual(report["counts"]["hardware_runs"], 1)

    def test_rebuild_archives_corrupt_database_and_preserves_planned_case(self) -> None:
        simulation_run_id = self._plan()
        session_dir = self.store.session_dir
        self.store.close()
        database = session_dir / "campaign.sqlite3"
        database.write_bytes(b"not a sqlite database")
        with self.assertRaises(sqlite3.DatabaseError):
            storage.ContinuousHilStore(session_dir)

        rebuilt = storage.ContinuousHilStore.rebuild_database(session_dir)
        self.store = rebuilt
        plan = self.store.connection.execute(
            "SELECT * FROM simulation_runs WHERE simulation_run_id=?",
            (simulation_run_id,),
        ).fetchone()
        self.assertEqual(plan["status"], "PLANNED")
        self.assertEqual(json.loads(plan["sampled_inputs_json"]), {"mass": 3.0})
        archived = list(
            (session_dir / "database_recovery").glob(
                "*/campaign.sqlite3"
            )
        )
        self.assertEqual(len(archived), 1)
        self.assertEqual(archived[0].read_bytes(), b"not a sqlite database")

    def test_rebuild_marks_unfinalized_attempt_aborted_and_truncates_tail(self) -> None:
        simulation_run_id = self._plan()
        hardware_run_id = "test-session-hw-00000000-a01"
        replay_dir = (
            self.store.session_dir
            / "runs"
            / hardware_run_id
            / "replay"
        )
        replay_dir.mkdir(parents=True)
        self.store.begin_hardware_run(
            simulation_run_id,
            hardware_run_id,
            replay_dir,
        )
        complete = {
            "schema": "ambar.live_replay_event.v1",
            "session_id": self.store.session_id,
            "simulation_run_id": simulation_run_id,
            "hardware_run_id": hardware_run_id,
            "event_index": 0,
            "event": "run_started",
            "host_elapsed_s": 0.0,
        }
        event_log = replay_dir / "events.jsonl"
        event_log.write_text(
            json.dumps(complete) + "\n" + '{"event_index":1',
            encoding="utf-8",
        )
        session_dir = self.store.session_dir
        self.store.close()
        for path in (
            session_dir / "campaign.sqlite3",
            Path(str(session_dir / "campaign.sqlite3") + "-wal"),
            Path(str(session_dir / "campaign.sqlite3") + "-shm"),
        ):
            path.unlink(missing_ok=True)

        rebuilt = storage.ContinuousHilStore.rebuild_database(session_dir)
        self.store = rebuilt
        hardware = self.store.connection.execute(
            "SELECT * FROM hardware_runs WHERE hardware_run_id=?",
            (hardware_run_id,),
        ).fetchone()
        self.assertEqual(hardware["status"], "ABORTED_HOST_CRASH")
        self.assertEqual(hardware["failure_type"], "InterruptedSession")
        self.assertTrue(event_log.read_bytes().endswith(b"\n"))
        self.assertEqual(
            self.store.connection.execute(
                "SELECT COUNT(*) FROM events"
            ).fetchone()[0],
            1,
        )

    def test_failed_attempt_uses_same_finalize_and_mirror_path(self) -> None:
        simulation_run_id = self._plan()
        hardware_run_id = "test-session-hw-00000000-a01"
        hardware_dir = self.store.session_dir / "runs" / hardware_run_id
        replay_dir = hardware_dir / "replay"
        replay_dir.mkdir(parents=True)
        self.store.begin_hardware_run(
            simulation_run_id,
            hardware_run_id,
            replay_dir,
        )
        verdict = {
            "status": "FAIL",
            "passed": False,
            "failure": {"type": "ReplayError", "message": "FULL timeout"},
            "metrics": {},
        }
        (replay_dir / "verdict.json").write_text(
            json.dumps(verdict),
            encoding="utf-8",
        )
        mirror = self.root / "mirror"
        supervisor._finalize_hardware_attempt(
            self.store,
            hardware_run_id=hardware_run_id,
            hardware_dir=hardware_dir,
            mirror_root=mirror,
            status="FAIL",
            verdict=verdict,
            failure=RuntimeError("FULL timeout"),
        )
        self.assertTrue(
            (mirror / "runs" / hardware_run_id / "replay" / "verdict.json").is_file()
        )
        self.assertTrue((mirror / "runs.csv").is_file())
        status = self.store.connection.execute(
            "SELECT status FROM hardware_runs WHERE hardware_run_id=?",
            (hardware_run_id,),
        ).fetchone()[0]
        self.assertEqual(status, "FAIL")

    def test_latest_mirror_drops_managed_artifacts_from_prior_session(self) -> None:
        mirror = self.root / "mirror"
        stale_run = mirror / "runs" / "old-session-hw-1"
        stale_report = mirror / "reports" / "latest.html"
        stale_run.mkdir(parents=True)
        stale_report.parent.mkdir(parents=True)
        (stale_run / "verdict.json").write_text("{}", encoding="utf-8")
        stale_report.write_text("old", encoding="utf-8")
        (mirror / "session.json").write_text(
            json.dumps({"session_id": "old-session"}),
            encoding="utf-8",
        )
        (mirror / "keep-user-note.txt").write_text("keep", encoding="utf-8")

        supervisor._mirror_session_portables(self.store, mirror)

        self.assertFalse((mirror / "runs").exists())
        self.assertFalse((mirror / "reports").exists())
        self.assertTrue((mirror / "keep-user-note.txt").is_file())
        mirrored_session = json.loads(
            (mirror / "session.json").read_text(encoding="utf-8")
        )
        self.assertEqual(mirrored_session["session_id"], self.store.session_id)

    def test_supervisor_exposes_explicit_rebuild_option(self) -> None:
        args = supervisor.build_argument_parser().parse_args(
            [
                "--resume",
                "test-session",
                "--rebuild-database",
                "--accept-current-position-home",
            ]
        )
        self.assertTrue(args.rebuild_database)
        self.assertTrue(args.accept_current_position_home)

    def test_supervisor_home_acknowledgement_defaults_fail_closed(self) -> None:
        args = supervisor.build_argument_parser().parse_args([])
        self.assertFalse(args.accept_current_position_home)


if __name__ == "__main__":
    unittest.main()
