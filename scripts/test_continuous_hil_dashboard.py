"""Focused tests for the read-only AMBAR continuous-HIL dashboard."""

from __future__ import annotations

import json
from pathlib import Path
import sqlite3
import sys
import tempfile
import unittest


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from continuous_hil_dashboard import (  # noqa: E402
    DashboardRepository,
    DatabaseLocator,
    DEFAULT_UI_ROOT,
    EventBus,
    write_snapshot,
    write_snapshot_set,
)


SCHEMA = """
CREATE TABLE sessions(
    session_id TEXT PRIMARY KEY,
    status TEXT,
    started_utc TEXT,
    finished_utc TEXT,
    master_seed INTEGER,
    batch_size INTEGER,
    baseline_interval INTEGER,
    dwell_s REAL,
    stop_reason TEXT
);
CREATE TABLE simulation_runs(
    simulation_run_id TEXT PRIMARY KEY,
    session_id TEXT,
    cycle_index INTEGER,
    mode TEXT,
    run_seed INTEGER,
    status TEXT,
    started_utc TEXT
);
CREATE TABLE hardware_runs(
    hardware_run_id TEXT PRIMARY KEY,
    session_id TEXT,
    simulation_run_id TEXT,
    status TEXT,
    started_utc TEXT,
    finished_utc TEXT,
    open_time_s REAL,
    close_time_s REAL,
    max_tracking_error_steps INTEGER,
    failure_type TEXT,
    failure_message TEXT
);
CREATE TABLE events(
    session_id TEXT,
    hardware_run_id TEXT,
    event_index INTEGER,
    event TEXT,
    host_elapsed_s REAL,
    record_json TEXT,
    PRIMARY KEY(hardware_run_id,event_index)
);
CREATE TABLE samples(
    hardware_run_id TEXT,
    event_index INTEGER,
    sample_index INTEGER,
    replay_time_s REAL,
    truth_altitude_m REAL,
    truth_velocity_mps REAL,
    stm32_altitude_m REAL,
    stm32_velocity_mps REAL,
    stm32_predicted_apogee_m REAL,
    target_apogee_m REAL,
    raw_controller_deployment_fraction REAL,
    forced_hil_deployment_fraction REAL,
    motor_target_steps INTEGER,
    motor_actual_steps INTEGER,
    home_active INTEGER,
    full_active INTEGER,
    limits_plausible INTEGER,
    stm32_phase INTEGER,
    schedule_lag_s REAL,
    skipped_host_samples INTEGER,
    PRIMARY KEY(hardware_run_id,event_index)
);
"""


class DashboardFixture(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.database = self.root / "campaign.sqlite3"
        connection = sqlite3.connect(self.database)
        connection.executescript(SCHEMA)
        connection.execute(
            "INSERT INTO sessions VALUES(?,?,?,?,?,?,?,?,?)",
            (
                "session-test",
                "RUNNING",
                "2026-07-16T12:00:00Z",
                None,
                123,
                50,
                10,
                30.0,
                None,
            ),
        )
        connection.executemany(
            "INSERT INTO simulation_runs VALUES(?,?,?,?,?,?,?)",
            [
                (
                    "sim-1",
                    "session-test",
                    1,
                    "baseline",
                    1,
                    "COMPLETE",
                    "2026-07-16T12:00:01Z",
                ),
                (
                    "sim-2",
                    "session-test",
                    2,
                    "monte_carlo",
                    2,
                    "RUNNING",
                    "2026-07-16T12:01:01Z",
                ),
            ],
        )
        connection.executemany(
            "INSERT INTO hardware_runs VALUES(?,?,?,?,?,?,?,?,?,?,?)",
            [
                (
                    "hw-1",
                    "session-test",
                    "sim-1",
                    "PASS",
                    "2026-07-16T12:00:02Z",
                    "2026-07-16T12:00:20Z",
                    2.0,
                    2.2,
                    512,
                    None,
                    None,
                ),
                (
                    "hw-2",
                    "session-test",
                    "sim-2",
                    "RUNNING",
                    "2026-07-16T12:01:02Z",
                    None,
                    None,
                    None,
                    None,
                    None,
                    None,
                ),
            ],
        )
        for index in range(12):
            connection.execute(
                "INSERT INTO samples VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
                (
                    "hw-2",
                    index,
                    index,
                    index * 0.05,
                    100.0 + index,
                    20.0 + index,
                    99.0 + index,
                    19.0 + index,
                    1000.0,
                    914.4,
                    0.25,
                    1.0,
                    153600,
                    51200 * min(3, index / 4),
                    int(index == 0),
                    int(index >= 10),
                    1,
                    3,
                    0.002,
                    index // 5,
                ),
            )
            record = {
                "schema": "ambar.live_replay_event.v1",
                "hardware_run_id": "hw-2",
                "event_index": index,
                "event": "sample_observed",
            }
            connection.execute(
                "INSERT INTO events VALUES(?,?,?,?,?,?)",
                (
                    "session-test",
                    "hw-2",
                    index,
                    "sample_observed",
                    index * 0.05,
                    json.dumps(record),
                ),
            )
        connection.commit()
        connection.close()
        self.repository = DashboardRepository(
            DatabaseLocator(self.database, self.root)
        )

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def test_state_projects_current_run_and_distinct_channels(self) -> None:
        state = self.repository.state()
        self.assertTrue(state["database"]["available"])
        self.assertEqual(state["session"]["completed_runs"], 1)
        self.assertEqual(state["session"]["passed_runs"], 1)
        self.assertEqual(state["current_run"]["hardware_run_id"], "hw-2")
        self.assertEqual(state["current_run"]["sample_count"], 12)
        sample = state["current_run"]["samples"][-1]
        self.assertAlmostEqual(sample["truth_altitude_ft"], 111 * 3.280839895)
        self.assertEqual(sample["raw_controller_percent"], 25.0)
        self.assertEqual(sample["applied_hil_percent"], 100.0)
        self.assertEqual(sample["target_rotations"], 3.0)
        self.assertEqual(sample["phase"], "AirbrakeActive")
        self.assertAlmostEqual(sample["packet_lag_ms"], 2.0)
        self.assertEqual(sample["skipped_host_samples"], 2)

    def test_variable_hil_sample_keeps_all_causal_channels_distinct(self) -> None:
        sample = DashboardRepository._normalize_sample(
            {
                "sample_index": 7,
                "replay_time_s": 0.14,
                "controller_request_fraction": 0.62,
                "actuator_target_fraction": 0.60,
                "actuator_xactual_fraction": 0.43,
                "physics_applied_fraction": 0.41,
                "feedback_age_ms": 18.5,
                "feedback_source": "TMC5240_XACTUAL",
                "reachable": 1,
                "predicted_closed_apogee_m": 960.0,
                "predicted_full_apogee_m": 860.0,
                "config_crc32": 0x1234ABCD,
                "simulation_input_sequence": 77,
            }
        )
        self.assertEqual(sample["controller_request_percent"], 62.0)
        self.assertEqual(sample["actuator_target_percent"], 60.0)
        self.assertEqual(sample["actuator_feedback_percent"], 43.0)
        self.assertEqual(sample["physics_applied_percent"], 41.0)
        self.assertEqual(sample["feedback_age_ms"], 18.5)
        self.assertEqual(sample["feedback_source"], "TMC5240_XACTUAL")
        self.assertTrue(sample["target_reachable"])
        self.assertEqual(sample["configuration_crc"], 0x1234ABCD)
        self.assertEqual(sample["correlated_sequence"], 77)

    def test_host_crash_abort_counts_as_failed_run(self) -> None:
        connection = sqlite3.connect(self.database)
        try:
            connection.execute(
                "INSERT INTO simulation_runs VALUES(?,?,?,?,?,?,?)",
                (
                    "sim-crash",
                    "session-test",
                    3,
                    "monte_carlo",
                    3,
                    "ABORTED",
                    "2026-07-16T12:02:01Z",
                ),
            )
            connection.execute(
                "INSERT INTO hardware_runs VALUES(?,?,?,?,?,?,?,?,?,?,?)",
                (
                    "hw-crash",
                    "session-test",
                    "sim-crash",
                    "ABORTED_HOST_CRASH",
                    "2026-07-16T12:02:02Z",
                    "2026-07-16T12:02:03Z",
                    None,
                    None,
                    None,
                    "HOST_CRASH",
                    "Recovered interrupted attempt",
                ),
            )
            connection.commit()
        finally:
            connection.close()

        state = self.repository.state()
        self.assertEqual(state["session"]["failed_runs"], 1)

    def test_event_backfill_uses_hardware_run_and_event_index(self) -> None:
        events = self.repository.event_backfill("hw-2", 8)
        self.assertEqual([event["event_index"] for event in events], [9, 10, 11])

    def test_snapshot_is_self_contained_and_carries_measurement_boundary(self) -> None:
        output = self.root / "reports" / "latest.html"
        write_snapshot(output, self.repository.state(), DEFAULT_UI_ROOT)
        html = output.read_text(encoding="utf-8")
        self.assertIn("window.AMBAR_SNAPSHOT_STATE=", html)
        self.assertIn("session-test", html)
        self.assertIn("not an independent encoder", html)
        self.assertIn("Packet lag", html)
        self.assertIn("Skipped samples", html)
        self.assertIn('"skipped_host_samples":2', html)
        self.assertNotIn('href="styles.css"', html)
        self.assertNotIn('src="app.js"', html)

    def test_checkpoint_snapshot_is_mirrored_with_latest_report(self) -> None:
        reports = self.root / "reports"
        mirror = self.root / "mirror"
        write_snapshot_set(
            reports,
            self.repository.state(),
            DEFAULT_UI_ROOT,
            checkpoint_completed=10,
            mirror_root=mirror,
        )
        self.assertTrue((reports / "latest.html").is_file())
        self.assertTrue((reports / "checkpoint-000010.html").is_file())
        self.assertTrue((mirror / "reports" / "latest.html").is_file())
        self.assertTrue((mirror / "reports" / "checkpoint-000010.html").is_file())

    def test_event_bus_reports_bounded_buffer_gap(self) -> None:
        bus = EventBus(maximum=2)
        bus.publish({"event": "one"})
        bus.publish({"event": "two"})
        bus.publish({"event": "three"})
        events, gap = bus.after(0)
        self.assertTrue(gap)
        self.assertEqual([sequence for sequence, _record in events], [2, 3])


if __name__ == "__main__":
    unittest.main()
