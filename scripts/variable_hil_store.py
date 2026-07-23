"""Durable evidence store for causal AMBAR VARIABLE_HIL runs.

This module is additive: it does not migrate or reinterpret the forced-stroke
continuous-HIL database.  VARIABLE_HIL sessions use the familiar dashboard
table names while keeping controller request, motor target, TMC5240 XACTUAL,
and the deployment applied to the *next* RocketPy interval as four separate
channels.

``forced_hil_deployment_fraction`` is deliberately constrained to NULL.  A
VARIABLE_HIL run is XACTUAL-coupled and must never be presented as the older
forced-full/forced-home qualification stroke.
"""

from __future__ import annotations

import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sqlite3
import time
from typing import Any, Mapping, Sequence
import zlib


SCHEMA_VERSION = 1
WORKFLOW = "VARIABLE_HIL"
COUPLING_MODE = "TMC_RAMP_STATE_COUPLED"
DEFAULT_TICK_HZ = 50.0


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def default_results_root() -> Path:
    local = os.environ.get("LOCALAPPDATA")
    if local:
        return Path(local) / "AMBAR" / "VariableHilRuns"
    return Path.home() / "AppData" / "Local" / "AMBAR" / "VariableHilRuns"


def canonical_config_crc32(config: Mapping[str, Any]) -> int:
    """Return an IEEE CRC32 over deterministic compact UTF-8 JSON.

    This CRC identifies a host-side case configuration.  The firmware's
    canonical control-config CRC is reported independently by every board
    feedback packet and becomes authoritative for hardware correlation.
    """

    payload = json.dumps(
        config,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return zlib.crc32(payload) & 0xFFFFFFFF


def _json(value: Any) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )


def _write_json_atomic(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _write_csv_atomic(
    path: Path,
    fieldnames: Sequence[str],
    rows: Sequence[Mapping[str, Any]],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    os.replace(temporary, path)


class VariableHilStore:
    """SQLite/WAL store with a dashboard-readable VARIABLE_HIL projection."""

    def __init__(self, session_dir: Path | str) -> None:
        self.session_dir = Path(session_dir).resolve()
        self.database_path = self.session_dir / "campaign.sqlite3"
        if not self.database_path.is_file():
            raise FileNotFoundError(self.database_path)
        self.connection = sqlite3.connect(
            self.database_path,
            isolation_level=None,
            timeout=10.0,
        )
        self.connection.row_factory = sqlite3.Row
        self.connection.execute("PRAGMA foreign_keys=ON")
        self.connection.execute("PRAGMA busy_timeout=10000")
        # The 50 Hz control loop writes several correlated rows per tick.  FULL
        # synchronous autocommit can pause that loop for a storage flush, while
        # an automatic WAL checkpoint can add another unbounded pause.  NORMAL
        # keeps WAL transactions crash-consistent without forcing the motor
        # control thread to wait for every physical disk flush; checkpointing is
        # left until the run has safely stopped.
        self.connection.execute("PRAGMA synchronous=NORMAL")
        self.connection.execute("PRAGMA wal_autocheckpoint=0")
        self._event_index = int(
            self.connection.execute(
                "SELECT COALESCE(MAX(event_index), -1) + 1 FROM events"
            ).fetchone()[0]
        )
        self._last_commit_s = time.monotonic()

    @classmethod
    def create(
        cls,
        results_root: Path | str,
        session_id: str,
        *,
        tick_hz: float = DEFAULT_TICK_HZ,
        settings: Mapping[str, Any] | None = None,
        config_crc32: int | None = None,
    ) -> "VariableHilStore":
        if not session_id or any(ch in session_id for ch in "\\/:"):
            raise ValueError("session ID must be a nonempty filesystem-safe leaf")
        if tick_hz <= 0.0:
            raise ValueError("tick rate must be positive")
        if config_crc32 is not None and not 1 <= int(config_crc32) <= 0xFFFFFFFF:
            raise ValueError("config CRC32 must be a nonzero uint32 when supplied")

        session_dir = Path(results_root).resolve() / session_id
        session_dir.mkdir(parents=True, exist_ok=False)
        database_path = session_dir / "campaign.sqlite3"
        connection = sqlite3.connect(database_path, isolation_level=None)
        try:
            connection.execute("PRAGMA journal_mode=WAL")
            connection.execute("PRAGMA synchronous=FULL")
            connection.execute("PRAGMA foreign_keys=ON")
            cls._create_schema(connection)
            now = utc_now()
            connection.execute(
                """
                INSERT INTO sessions(
                    session_id, schema_version, workflow, coupling_mode, status,
                    started_utc, updated_utc, tick_hz, config_crc32,
                    settings_json
                ) VALUES(?,?,?,?,?,?,?,?,?,?)
                """,
                (
                    session_id,
                    SCHEMA_VERSION,
                    WORKFLOW,
                    COUPLING_MODE,
                    "CREATED",
                    now,
                    now,
                    float(tick_hz),
                    config_crc32,
                    _json(dict(settings or {})),
                ),
            )
        finally:
            connection.close()
        store = cls(session_dir)
        store._write_session_manifest()
        return store

    @staticmethod
    def _create_schema(connection: sqlite3.Connection) -> None:
        connection.executescript(
            """
            CREATE TABLE sessions(
                session_id TEXT PRIMARY KEY,
                schema_version INTEGER NOT NULL,
                workflow TEXT NOT NULL,
                coupling_mode TEXT NOT NULL,
                status TEXT NOT NULL,
                started_utc TEXT NOT NULL,
                finished_utc TEXT,
                updated_utc TEXT NOT NULL,
                tick_hz REAL NOT NULL,
                config_crc32 INTEGER,
                settings_json TEXT NOT NULL,
                stop_reason TEXT,
                hardware_safety_verdict TEXT,
                performance_verdict TEXT,
                hardware_verdict_json TEXT,
                performance_verdict_json TEXT
            );

            CREATE TABLE simulation_cases(
                simulation_case_id TEXT PRIMARY KEY,
                session_id TEXT NOT NULL REFERENCES sessions(session_id),
                case_index INTEGER NOT NULL,
                mode TEXT NOT NULL,
                workflow TEXT NOT NULL,
                coupling_mode TEXT NOT NULL,
                status TEXT NOT NULL,
                started_utc TEXT NOT NULL,
                finished_utc TEXT,
                host_config_crc32 INTEGER NOT NULL,
                config_crc32 INTEGER,
                input_config_json TEXT NOT NULL,
                performance_verdict TEXT,
                performance_verdict_json TEXT,
                truth_apogee_m REAL,
                target_apogee_m REAL,
                target_band_pass INTEGER,
                run_pass INTEGER
            );

            CREATE TABLE hardware_runs(
                hardware_run_id TEXT PRIMARY KEY,
                session_id TEXT NOT NULL REFERENCES sessions(session_id),
                simulation_case_id TEXT NOT NULL
                    REFERENCES simulation_cases(simulation_case_id),
                status TEXT NOT NULL,
                workflow TEXT NOT NULL,
                coupling_mode TEXT NOT NULL,
                started_utc TEXT NOT NULL,
                finished_utc TEXT,
                failure_type TEXT,
                failure_message TEXT,
                tick_count INTEGER NOT NULL DEFAULT 0,
                late_tick_count INTEGER NOT NULL DEFAULT 0,
                max_consecutive_misses INTEGER NOT NULL DEFAULT 0,
                max_feedback_age_s REAL,
                max_feedback_age_ms REAL,
                final_target_steps INTEGER,
                final_xactual_steps INTEGER,
                max_tracking_error_steps INTEGER,
                feedback_source TEXT,
                config_crc32 INTEGER,
                hardware_safety_verdict TEXT,
                performance_verdict TEXT,
                hardware_verdict_json TEXT,
                performance_verdict_json TEXT
            );

            CREATE TABLE samples(
                hardware_run_id TEXT NOT NULL
                    REFERENCES hardware_runs(hardware_run_id),
                event_index INTEGER NOT NULL,
                tick_index INTEGER NOT NULL,
                sample_index INTEGER NOT NULL,
                scheduled_time_s REAL NOT NULL,
                host_elapsed_s REAL NOT NULL,
                replay_time_s REAL NOT NULL,
                simulation_time_s REAL NOT NULL,
                source_time_s REAL NOT NULL,
                schedule_lag_s REAL NOT NULL,
                packet_lag_ms REAL,
                late_tick INTEGER NOT NULL,
                consecutive_misses INTEGER NOT NULL,
                skipped_host_samples INTEGER NOT NULL,
                sensor_altitude_m REAL NOT NULL,
                sensor_velocity_mps REAL NOT NULL,
                sensor_acceleration_mps2 REAL NOT NULL,
                truth_altitude_m REAL NOT NULL,
                truth_velocity_mps REAL NOT NULL,
                truth_acceleration_mps2 REAL NOT NULL,
                estimated_altitude_m REAL,
                estimated_velocity_mps REAL,
                predicted_apogee_m REAL,
                closed_predicted_apogee_m REAL,
                full_predicted_apogee_m REAL,
                target_apogee_m REAL,
                target_reachable INTEGER,
                controller_request_fraction REAL,
                controller_requested_fraction REAL,
                raw_controller_deployment_fraction REAL,
                actuator_target_fraction REAL,
                actuator_target_deployment_fraction REAL,
                actuator_target_steps INTEGER,
                motor_target_steps INTEGER,
                actuator_xactual_fraction REAL,
                actuator_feedback_deployment_fraction REAL,
                actuator_xactual_steps INTEGER,
                motor_actual_steps INTEGER,
                physics_applied_fraction REAL NOT NULL,
                physics_applied_deployment_fraction REAL NOT NULL,
                physics_applies_to_tick_index INTEGER NOT NULL,
                forced_hil_deployment_fraction REAL
                    CHECK(forced_hil_deployment_fraction IS NULL),
                feedback_source TEXT,
                feedback_source_code INTEGER,
                feedback_age_s REAL,
                feedback_age_ms REAL,
                board_sequence_sent INTEGER,
                board_sequence_confirmed INTEGER,
                correlated_sequence INTEGER,
                config_crc32 INTEGER,
                flight_inhibit_flags INTEGER,
                actuator_inhibit_flags INTEGER,
                driver_status INTEGER,
                phase INTEGER,
                machine_state INTEGER,
                state_flags INTEGER,
                hardware_ok INTEGER NOT NULL,
                performance_ok INTEGER NOT NULL,
                fault_active INTEGER NOT NULL,
                PRIMARY KEY(hardware_run_id, tick_index)
            );

            CREATE TABLE events(
                session_id TEXT NOT NULL REFERENCES sessions(session_id),
                hardware_run_id TEXT,
                event_index INTEGER NOT NULL,
                event TEXT NOT NULL,
                host_elapsed_s REAL,
                record_json TEXT NOT NULL,
                PRIMARY KEY(session_id, event_index)
            );

            CREATE INDEX samples_time_idx
                ON samples(hardware_run_id, sample_index);
            CREATE INDEX events_run_idx
                ON events(hardware_run_id, event_index);
            """
        )

    @property
    def session_id(self) -> str:
        return self.session_dir.name

    @property
    def session(self) -> dict[str, Any]:
        row = self.connection.execute(
            "SELECT * FROM sessions WHERE session_id=?", (self.session_id,)
        ).fetchone()
        if row is None:
            raise RuntimeError("session row is missing")
        value = dict(row)
        value["settings"] = json.loads(value.pop("settings_json"))
        detail_names = {
            "hardware_verdict_json": "hardware_verdict_detail",
            "performance_verdict_json": "performance_verdict_detail",
        }
        for name, detail_name in detail_names.items():
            raw = value.get(name)
            value[detail_name] = json.loads(raw) if raw else None
        return value

    def _write_session_manifest(self) -> None:
        _write_json_atomic(self.session_dir / "session.json", self.session)

    def begin_run(
        self,
        *,
        simulation_case_id: str,
        hardware_run_id: str,
        case_index: int,
        input_config: Mapping[str, Any],
        target_apogee_m: float,
        mode: str,
    ) -> None:
        now = utc_now()
        host_crc = canonical_config_crc32(input_config)
        self.connection.execute("BEGIN IMMEDIATE")
        try:
            self.connection.execute(
                """
                INSERT INTO simulation_cases(
                    simulation_case_id,session_id,case_index,mode,workflow,
                    coupling_mode,status,started_utc,host_config_crc32,
                    input_config_json,target_apogee_m
                ) VALUES(?,?,?,?,?,?,?,?,?,?,?)
                """,
                (
                    simulation_case_id,
                    self.session_id,
                    int(case_index),
                    mode,
                    WORKFLOW,
                    COUPLING_MODE,
                    "RUNNING",
                    now,
                    host_crc,
                    _json(dict(input_config)),
                    float(target_apogee_m),
                ),
            )
            self.connection.execute(
                """
                INSERT INTO hardware_runs(
                    hardware_run_id,session_id,simulation_case_id,status,
                    workflow,coupling_mode,started_utc
                ) VALUES(?,?,?,?,?,?,?)
                """,
                (
                    hardware_run_id,
                    self.session_id,
                    simulation_case_id,
                    "RUNNING",
                    WORKFLOW,
                    COUPLING_MODE,
                    now,
                ),
            )
            self.connection.execute(
                "UPDATE sessions SET status='RUNNING',updated_utc=? WHERE session_id=?",
                (now, self.session_id),
            )
            self.connection.execute("COMMIT")
        except BaseException:
            self.connection.execute("ROLLBACK")
            raise
        self._write_session_manifest()

    def latch_board_config_crc32(self, config_crc32: int) -> None:
        value = int(config_crc32)
        if not 1 <= value <= 0xFFFFFFFF:
            raise ValueError("board config CRC32 must be a nonzero uint32")
        current = self.connection.execute(
            "SELECT config_crc32 FROM sessions WHERE session_id=?",
            (self.session_id,),
        ).fetchone()[0]
        if current is not None and int(current) != value:
            raise ValueError(
                f"board config CRC32 changed from 0x{int(current):08X} "
                f"to 0x{value:08X}"
            )
        self.connection.execute(
            "UPDATE sessions SET config_crc32=?,updated_utc=? WHERE session_id=?",
            (value, utc_now(), self.session_id),
        )

    def record_event(
        self,
        event: str,
        *,
        hardware_run_id: str | None,
        host_elapsed_s: float | None,
        **fields: Any,
    ) -> int:
        event_index = self._event_index
        self._event_index += 1
        record = {
            "schema": "ambar.variable_hil_event.v1",
            "session_id": self.session_id,
            "hardware_run_id": hardware_run_id,
            "event_index": event_index,
            "event": event,
            "host_elapsed_s": host_elapsed_s,
            **fields,
        }
        self.connection.execute(
            """
            INSERT INTO events(
                session_id,hardware_run_id,event_index,event,host_elapsed_s,
                record_json
            ) VALUES(?,?,?,?,?,?)
            """,
            (
                self.session_id,
                hardware_run_id,
                event_index,
                event,
                host_elapsed_s,
                _json(record),
            ),
        )
        return event_index

    @staticmethod
    def _fraction(value: Any, name: str, *, nullable: bool = True) -> float | None:
        if value is None:
            if nullable:
                return None
            raise ValueError(f"{name} is required")
        result = float(value)
        if not 0.0 <= result <= 1.0:
            raise ValueError(f"{name} must be within [0, 1]")
        return result

    def record_sample(self, hardware_run_id: str, sample: Mapping[str, Any]) -> None:
        if sample.get("forced_hil_deployment_fraction") is not None:
            raise ValueError(
                "VARIABLE_HIL cannot persist a forced-HIL deployment fraction"
            )
        controller = self._fraction(
            sample.get("controller_request_fraction"),
            "controller_request_fraction",
        )
        target_fraction = self._fraction(
            sample.get("actuator_target_fraction"),
            "actuator_target_fraction",
        )
        xactual_fraction = self._fraction(
            sample.get("actuator_xactual_fraction"),
            "actuator_xactual_fraction",
        )
        physics_fraction = self._fraction(
            sample.get("physics_applied_fraction"),
            "physics_applied_fraction",
            nullable=False,
        )
        config_crc = sample.get("config_crc32")
        if config_crc is not None and not 1 <= int(config_crc) <= 0xFFFFFFFF:
            raise ValueError("sample config CRC32 must be a nonzero uint32")
        for name in ("board_sequence_sent", "board_sequence_confirmed"):
            value = sample.get(name)
            if value is not None and not 0 <= int(value) <= 0xFFFF:
                raise ValueError(f"{name} must be a uint16")

        tick_index = int(sample["tick_index"])
        event_fields = dict(sample)
        event_fields.pop("host_elapsed_s", None)
        event_index = self.record_event(
            "variable_hil_sample",
            hardware_run_id=hardware_run_id,
            host_elapsed_s=float(sample["host_elapsed_s"]),
            **event_fields,
        )
        values = {
            **dict(sample),
            "hardware_run_id": hardware_run_id,
            "event_index": event_index,
            "sample_index": tick_index,
            "replay_time_s": float(sample["simulation_time_s"]),
            "source_time_s": float(sample["simulation_time_s"]),
            "controller_request_fraction": controller,
            "controller_requested_fraction": controller,
            "raw_controller_deployment_fraction": controller,
            "actuator_target_fraction": target_fraction,
            "actuator_target_deployment_fraction": target_fraction,
            "actuator_xactual_fraction": xactual_fraction,
            "actuator_feedback_deployment_fraction": xactual_fraction,
            "physics_applied_fraction": physics_fraction,
            "physics_applied_deployment_fraction": physics_fraction,
            "physics_applies_to_tick_index": tick_index + 1,
            "forced_hil_deployment_fraction": None,
            "correlated_sequence": sample.get("board_sequence_confirmed"),
            "packet_lag_ms": sample.get("feedback_age_ms"),
            "skipped_host_samples": int(sample.get("late_tick_count", 0)),
            "motor_target_steps": sample.get("actuator_target_steps"),
            "motor_actual_steps": sample.get("actuator_xactual_steps"),
        }
        columns = [
            str(row[1])
            for row in self.connection.execute("PRAGMA table_info(samples)")
        ]
        placeholders = ",".join("?" for _ in columns)
        self.connection.execute(
            f"INSERT INTO samples({','.join(columns)}) VALUES({placeholders})",
            tuple(values.get(name) for name in columns),
        )
        self.connection.execute(
            """
            UPDATE hardware_runs SET
                tick_count=tick_count+1,
                late_tick_count=late_tick_count+?,
                max_consecutive_misses=MAX(max_consecutive_misses,?),
                max_feedback_age_s=CASE
                    WHEN ? IS NULL THEN max_feedback_age_s
                    ELSE MAX(COALESCE(max_feedback_age_s,0),?) END,
                max_feedback_age_ms=CASE
                    WHEN ? IS NULL THEN max_feedback_age_ms
                    ELSE MAX(COALESCE(max_feedback_age_ms,0),?) END,
                final_target_steps=COALESCE(?,final_target_steps),
                final_xactual_steps=COALESCE(?,final_xactual_steps),
                max_tracking_error_steps=CASE
                    WHEN ? IS NULL OR ? IS NULL THEN max_tracking_error_steps
                    ELSE MAX(
                        COALESCE(max_tracking_error_steps,0),
                        ABS(?-?)
                    ) END,
                feedback_source=COALESCE(?,feedback_source),
                config_crc32=COALESCE(?,config_crc32)
            WHERE hardware_run_id=?
            """,
            (
                int(bool(sample.get("late_tick"))),
                int(sample.get("consecutive_misses", 0)),
                sample.get("feedback_age_s"),
                sample.get("feedback_age_s"),
                sample.get("feedback_age_ms"),
                sample.get("feedback_age_ms"),
                sample.get("actuator_target_steps"),
                sample.get("actuator_xactual_steps"),
                sample.get("actuator_target_steps"),
                sample.get("actuator_xactual_steps"),
                sample.get("actuator_target_steps"),
                sample.get("actuator_xactual_steps"),
                sample.get("feedback_source"),
                config_crc,
                hardware_run_id,
            ),
        )
        now = time.monotonic()
        if now - self._last_commit_s >= 0.25:
            self.connection.commit()
            self._last_commit_s = now

    def finalize_run(
        self,
        *,
        simulation_case_id: str,
        hardware_run_id: str,
        status: str,
        hardware_verdict: Mapping[str, Any],
        performance_verdict: Mapping[str, Any],
        truth_apogee_m: float | None,
        target_apogee_m: float,
        failure: BaseException | None = None,
        stop_reason: str | None = None,
    ) -> None:
        now = utc_now()
        hardware_text = str(
            hardware_verdict.get("status")
            or ("PASS" if bool(hardware_verdict.get("passed")) else "FAIL")
        ).upper()
        performance_text = str(
            performance_verdict.get("status")
            or ("PASS" if bool(performance_verdict.get("passed")) else "FAIL")
        ).upper()
        tolerance_m = performance_verdict.get("target_tolerance_m")
        target_band_pass = None
        if truth_apogee_m is not None and tolerance_m is not None:
            target_band_pass = int(
                abs(float(truth_apogee_m) - float(target_apogee_m))
                <= float(tolerance_m)
            )
        failure_type = type(failure).__name__ if failure is not None else None
        failure_message = str(failure) if failure is not None else None
        self.connection.execute("BEGIN IMMEDIATE")
        try:
            self.connection.execute(
                """
                UPDATE hardware_runs SET
                    status=?,finished_utc=?,failure_type=?,failure_message=?,
                    hardware_safety_verdict=?,performance_verdict=?,
                    hardware_verdict_json=?,performance_verdict_json=?
                WHERE hardware_run_id=?
                """,
                (
                    status,
                    now,
                    failure_type,
                    failure_message,
                    hardware_text,
                    performance_text,
                    _json(dict(hardware_verdict)),
                    _json(dict(performance_verdict)),
                    hardware_run_id,
                ),
            )
            self.connection.execute(
                """
                UPDATE simulation_cases SET
                    status='COMPLETE',finished_utc=?,config_crc32=(
                        SELECT config_crc32 FROM hardware_runs
                        WHERE hardware_run_id=?
                    ),performance_verdict=?,performance_verdict_json=?,
                    truth_apogee_m=?,target_apogee_m=?,target_band_pass=?,run_pass=?
                WHERE simulation_case_id=?
                """,
                (
                    now,
                    hardware_run_id,
                    performance_text,
                    _json(dict(performance_verdict)),
                    truth_apogee_m,
                    float(target_apogee_m),
                    target_band_pass,
                    int(bool(performance_verdict.get("passed"))),
                    simulation_case_id,
                ),
            )
            # A run verdict must not close or overwrite a multi-run session.
            # finalize_session() computes campaign-level gates after the last
            # bounded case (or immediately for explicit one-run mode).
            self.connection.execute(
                "UPDATE sessions SET updated_utc=?,stop_reason=? WHERE session_id=?",
                (now, stop_reason, self.session_id),
            )
            self.connection.execute("COMMIT")
        except BaseException:
            self.connection.execute("ROLLBACK")
            raise
        self.export_csv()
        self._write_session_manifest()

    @staticmethod
    def _percentile(values: Sequence[float], quantile: float) -> float | None:
        if not values:
            return None
        ordered = sorted(float(value) for value in values)
        if len(ordered) == 1:
            return ordered[0]
        position = (len(ordered) - 1) * float(quantile)
        lower = int(position)
        upper = min(len(ordered) - 1, lower + 1)
        weight = position - lower
        return ordered[lower] * (1.0 - weight) + ordered[upper] * weight

    def finalize_session(
        self,
        *,
        expected_runs: int,
        hardware_required: bool,
        stop_reason: str,
    ) -> dict[str, Any]:
        """Compute independent campaign safety and target-performance gates."""

        if expected_runs <= 0:
            raise ValueError("expected run count must be positive")
        hardware_rows = [
            dict(row)
            for row in self.connection.execute(
                "SELECT * FROM hardware_runs WHERE session_id=? ORDER BY rowid",
                (self.session_id,),
            )
        ]
        case_rows = [
            dict(row)
            for row in self.connection.execute(
                "SELECT * FROM simulation_cases WHERE session_id=? ORDER BY case_index",
                (self.session_id,),
            )
        ]
        completed = len(
            [row for row in hardware_rows if str(row["status"]).upper() != "RUNNING"]
        )
        hardware_passes = 0
        tracking_passes = 0
        protocol_failures = 0
        cleanup_failures = 0
        for row in hardware_rows:
            raw = row.get("hardware_verdict_json")
            verdict = json.loads(raw) if raw else {}
            if str(verdict.get("status", "")).upper() == "PASS" and bool(
                verdict.get("passed")
            ):
                hardware_passes += 1
            if bool(verdict.get("tracking_pass")):
                tracking_passes += 1
            if row.get("failure_type") or verdict.get("protocol_failure"):
                protocol_failures += 1
            cleanup = verdict.get("cleanup")
            if not isinstance(cleanup, Mapping) or not bool(cleanup.get("success")):
                cleanup_failures += 1

        errors_m = [
            abs(float(row["truth_apogee_m"]) - float(row["target_apogee_m"]))
            for row in case_rows
            if row.get("truth_apogee_m") is not None
            and row.get("target_apogee_m") is not None
        ]
        in_band = sum(int(bool(row.get("target_band_pass"))) for row in case_rows)
        p95_error_m = self._percentile(errors_m, 0.95)
        worst_error_m = max(errors_m) if errors_m else None
        required_in_band = max(1, expected_runs - 1)
        complete_gate = completed == expected_runs and len(errors_m) == expected_runs

        if hardware_required:
            hardware_passed = (
                complete_gate
                and hardware_passes == expected_runs
                and tracking_passes == expected_runs
                and protocol_failures == 0
                and cleanup_failures == 0
            )
            hardware_status = "PASS" if hardware_passed else "FAIL"
        else:
            hardware_passed = None
            hardware_status = "NOT_APPLICABLE"

        performance_passed = (
            complete_gate
            and in_band >= required_in_band
            and p95_error_m is not None
            and p95_error_m <= 30.48
            and worst_error_m is not None
            and worst_error_m <= 60.96
        )
        hardware_verdict = {
            "schema": "ambar.variable_hil_campaign_hardware_verdict.v1",
            "status": hardware_status,
            "passed": hardware_passed,
            "hardware_required": hardware_required,
            "expected_runs": expected_runs,
            "completed_runs": completed,
            "hardware_safety_passes": hardware_passes,
            "tracking_passes": tracking_passes,
            "protocol_failure_count": protocol_failures,
            "cleanup_failure_count": cleanup_failures,
            "required_hardware_safety_passes": expected_runs,
            "required_tracking_passes": expected_runs,
            "qualification_25_cycle": expected_runs == 25,
        }
        performance_verdict = {
            "schema": "ambar.variable_hil_campaign_performance_verdict.v1",
            "status": "PASS" if performance_passed else "FAIL",
            "passed": performance_passed,
            "expected_runs": expected_runs,
            "completed_runs": completed,
            "in_band_runs": in_band,
            "required_in_band_runs": required_in_band,
            "p95_absolute_error_m": p95_error_m,
            "p95_absolute_error_ft": (
                None if p95_error_m is None else p95_error_m * 3.280839895
            ),
            "p95_limit_ft": 100.0,
            "worst_absolute_error_m": worst_error_m,
            "worst_absolute_error_ft": (
                None if worst_error_m is None else worst_error_m * 3.280839895
            ),
            "worst_limit_ft": 200.0,
            "forced_hil_deployment_fraction": None,
            "qualification_25_cycle": expected_runs == 25,
        }
        now = utc_now()
        session_status = (
            "FAULTED" if hardware_required and not hardware_passed else "COMPLETE"
        )
        self.connection.execute(
            """
            UPDATE sessions SET status=?,finished_utc=?,updated_utc=?,stop_reason=?,
                hardware_safety_verdict=?,performance_verdict=?,
                hardware_verdict_json=?,performance_verdict_json=?
            WHERE session_id=?
            """,
            (
                session_status,
                now,
                now,
                stop_reason,
                hardware_status,
                performance_verdict["status"],
                _json(hardware_verdict),
                _json(performance_verdict),
                self.session_id,
            ),
        )
        self.export_csv()
        self._write_session_manifest()
        return {
            "hardware": hardware_verdict,
            "performance": performance_verdict,
        }

    def export_csv(self) -> None:
        for table, filename in (
            ("hardware_runs", "runs.csv"),
            ("simulation_cases", "cases.csv"),
            ("samples", "samples.csv"),
        ):
            rows = [dict(row) for row in self.connection.execute(f"SELECT * FROM {table}")]
            columns = [
                str(row[1])
                for row in self.connection.execute(f"PRAGMA table_info({table})")
            ]
            _write_csv_atomic(self.session_dir / filename, columns, rows)

    def close(self) -> None:
        if getattr(self, "connection", None) is not None:
            self.connection.commit()
            self.connection.close()
            self.connection = None  # type: ignore[assignment]

    def __enter__(self) -> "VariableHilStore":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()
