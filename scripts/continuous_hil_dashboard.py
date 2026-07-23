"""Read-only live dashboard server for AMBAR continuous RocketPy HIL tests.

The supervisor owns the STM32 COM port and the SQLite writer. This process only:

* reads ``campaign.sqlite3`` through SQLite's read-only URI mode,
* receives best-effort localhost UDP event copies,
* serves static dashboard assets and Server-Sent Events on localhost, and
* creates bounded, self-contained HTML snapshots.

No serial package is imported here, so a dashboard failure cannot contend for
or interfere with the safety-critical connection.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import deque
from contextlib import closing
from dataclasses import dataclass
from datetime import datetime, timezone
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import queue
import re
import socket
import sqlite3
import sys
import threading
import time
from typing import Any, Iterable, Mapping
from urllib.parse import parse_qs, urlparse


REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_UI_ROOT = REPO_ROOT / "ui" / "continuous_hil"
DEFAULT_RESULTS_ROOT = (
    Path(os.environ.get("LOCALAPPDATA", Path.home() / "AppData" / "Local"))
    / "AMBAR"
    / "TestRuns"
)
DEFAULT_MIRROR_ROOT = (
    Path.home()
    / "OneDrive"
    / "Desktop"
    / "AMBAR_Continuous_Test_Results"
    / "latest"
)
SCHEMA = "ambar.continuous_hil_dashboard.v1"
FEET_PER_METER = 3.280839895013123
STEPS_PER_ROTATION = 200.0 * 256.0
CAMPAIGN_TARGET_APOGEE_FT = 3000.0
CAMPAIGN_TRACE_POINTS = 180
CAMPAIGN_AVERAGE_POINTS = 121
TERMINAL_SESSION_STATES = {
    "COMPLETE",
    "COMPLETED",
    "FAILED",
    "FAULT",
    "FAULTED",
    "STOPPED",
    "ABORTED",
}
PASS_STATES = {"PASS", "PASSED", "SUCCESS"}
FINAL_APOGEE_STATES = PASS_STATES | {"COMPLETE", "COMPLETED"}
FAIL_STATES = {
    "FAIL",
    "FAILED",
    "ERROR",
    "FAULT",
    "ABORTED",
    "ABORTED_HOST_CRASH",
}
ACTIVE_RUN_STATES = {
    "RUNNING",
    "STARTED",
    "PREFLIGHT",
    "HOMING",
    "REPLAY",
    "RETRACTING",
}


def utc_now_text() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def parse_utc(value: Any) -> datetime | None:
    if not value:
        return None
    try:
        parsed = datetime.fromisoformat(str(value).replace("Z", "+00:00"))
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


def json_value(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, bool)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, bytes):
        return value.hex()
    return str(value)


def row_dict(row: sqlite3.Row | Mapping[str, Any] | None) -> dict[str, Any]:
    if row is None:
        return {}
    return {str(key): json_value(value) for key, value in dict(row).items()}


def pick(values: Mapping[str, Any], *names: str, default: Any = None) -> Any:
    for name in names:
        if name in values and values[name] is not None:
            return values[name]
    return default


def number(values: Mapping[str, Any], *names: str) -> float | None:
    value = pick(values, *names)
    if value is None or isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def integer(
    values: Mapping[str, Any],
    *names: str,
    default: int | None = None,
) -> int | None:
    value = number(values, *names)
    return default if value is None else int(value)


def boolean(values: Mapping[str, Any], *names: str) -> bool:
    value = pick(values, *names, default=False)
    if isinstance(value, str):
        return value.strip().upper() in {"1", "TRUE", "YES", "ON", "ACTIVE"}
    return bool(value)


def optional_boolean(values: Mapping[str, Any], *names: str) -> bool | None:
    """Return a boolean only when at least one source field is present."""

    if not any(name in values and values[name] is not None for name in names):
        return None
    return boolean(values, *names)


def percent_value(values: Mapping[str, Any], fraction_names: Iterable[str], percent_names: Iterable[str]) -> float | None:
    direct = number(values, *percent_names)
    if direct is not None:
        return direct
    fraction = number(values, *fraction_names)
    return None if fraction is None else fraction * 100.0


def feet_value(values: Mapping[str, Any], meter_names: Iterable[str], foot_names: Iterable[str]) -> float | None:
    direct = number(values, *foot_names)
    if direct is not None:
        return direct
    meters = number(values, *meter_names)
    return None if meters is None else meters * FEET_PER_METER


def rotations_value(values: Mapping[str, Any], rotation_names: Iterable[str], step_names: Iterable[str]) -> float | None:
    direct = number(values, *rotation_names)
    if direct is not None:
        return direct
    steps = number(values, *step_names)
    return None if steps is None else steps / STEPS_PER_ROTATION


def decode_json_object(value: Any) -> dict[str, Any]:
    if isinstance(value, dict):
        return value
    if not isinstance(value, str) or not value.strip():
        return {}
    try:
        decoded = json.loads(value)
    except json.JSONDecodeError:
        return {}
    return decoded if isinstance(decoded, dict) else {}


def downsample(rows: list[dict[str, Any]], maximum: int) -> list[dict[str, Any]]:
    if len(rows) <= maximum:
        return rows
    if maximum < 2:
        return rows[-maximum:]
    indices = {
        round(index * (len(rows) - 1) / (maximum - 1))
        for index in range(maximum)
    }
    return [rows[index] for index in sorted(indices)]


@dataclass
class DatabaseLocator:
    explicit_path: Path | None
    results_root: Path

    def locate(self) -> Path | None:
        if self.explicit_path is not None:
            return self.explicit_path.resolve() if self.explicit_path.exists() else None
        if not self.results_root.exists():
            return None
        candidates = list(self.results_root.glob("*/campaign.sqlite3"))
        if not candidates:
            return None
        return max(candidates, key=lambda path: path.stat().st_mtime_ns).resolve()


class DashboardRepository:
    """Schema-aware, read-only projection of the continuous-HIL evidence DB."""

    def __init__(self, locator: DatabaseLocator) -> None:
        self.locator = locator
        self._campaign_cache_database: Path | None = None
        self._campaign_trace_cache: dict[str, dict[str, Any]] = {}
        self._campaign_cache_lock = threading.RLock()

    def _connect(self, database: Path) -> sqlite3.Connection:
        connection = sqlite3.connect(
            database.resolve().as_uri() + "?mode=ro",
            uri=True,
            timeout=1.0,
        )
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA query_only=ON")
        connection.execute("PRAGMA busy_timeout=1000")
        return connection

    @staticmethod
    def _tables(connection: sqlite3.Connection) -> set[str]:
        return {
            str(row[0])
            for row in connection.execute(
                "SELECT name FROM sqlite_master WHERE type='table'"
            )
        }

    @staticmethod
    def _columns(connection: sqlite3.Connection, table: str) -> set[str]:
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", table):
            return set()
        return {
            str(row[1])
            for row in connection.execute(f'PRAGMA table_info("{table}")')
        }

    @staticmethod
    def _ordered_rows(
        connection: sqlite3.Connection,
        table: str,
        columns: set[str],
        *,
        session_id: str | None = None,
        limit: int | None = None,
        newest_first: bool = True,
    ) -> list[sqlite3.Row]:
        where = ""
        parameters: list[Any] = []
        if session_id is not None and "session_id" in columns:
            where = " WHERE session_id=?"
            parameters.append(session_id)
        order_column = next(
            (
                name
                for name in (
                    "started_utc",
                    "planned_utc",
                    "cycle_index",
                    "case_index",
                    "event_index",
                    "sample_index",
                )
                if name in columns
            ),
            "rowid",
        )
        direction = "DESC" if newest_first else "ASC"
        sql = f'SELECT * FROM "{table}"{where} ORDER BY "{order_column}" {direction}, rowid {direction}'
        if limit is not None:
            sql += " LIMIT ?"
            parameters.append(limit)
        return list(connection.execute(sql, parameters))

    def _empty_state(self, database: Path | None, error: str | None = None) -> dict[str, Any]:
        return {
            "schema": SCHEMA,
            "generated_utc": utc_now_text(),
            "database": {
                "available": False,
                "path": str(database) if database else None,
                "error": error,
            },
            "session": {
                "session_id": None,
                "status": "WAITING_FOR_DATA",
                "started_utc": None,
                "completed_runs": 0,
                "passed_runs": 0,
                "failed_runs": 0,
                "current_run": None,
                "latest_fault": None,
                "dwell_seconds": 0.0,
                "dwell_remaining_s": 0.0,
            },
            "current_run": None,
            "recent_runs": [],
            "trends": [],
            "campaign_altitude": {
                "target_apogee_ft": CAMPAIGN_TARGET_APOGEE_FT,
                "trace_count": 0,
                "final_apogee_count": 0,
                "mean_truth_apogee_ft": None,
                "minimum_truth_apogee_ft": None,
                "maximum_truth_apogee_ft": None,
                "mean_target_error_ft": None,
                "traces": [],
                "average_trace": [],
            },
            "warnings": [
                "Waiting for the supervisor to create and populate campaign.sqlite3."
            ],
        }

    def state(self) -> dict[str, Any]:
        database = self.locator.locate()
        if database is None:
            return self._empty_state(self.locator.explicit_path)
        try:
            with closing(self._connect(database)) as connection:
                tables = self._tables(connection)
                if "sessions" not in tables:
                    return self._empty_state(database, "sessions table is not available yet")
                session_columns = self._columns(connection, "sessions")
                session_rows = self._ordered_rows(
                    connection,
                    "sessions",
                    session_columns,
                    limit=1,
                )
                if not session_rows:
                    return self._empty_state(database, "session row is not available yet")
                session = row_dict(session_rows[0])
                session_id = str(pick(session, "session_id", default=""))
                hardware_rows: list[dict[str, Any]] = []
                hardware_columns: set[str] = set()
                if "hardware_runs" in tables:
                    hardware_columns = self._columns(connection, "hardware_runs")
                    hardware_rows = [
                        row_dict(row)
                        for row in self._ordered_rows(
                            connection,
                            "hardware_runs",
                            hardware_columns,
                            session_id=session_id,
                            limit=250,
                        )
                    ]

                simulation_by_id: dict[str, dict[str, Any]] = {}
                simulation_rows: list[dict[str, Any]] = []
                simulation_table = (
                    "simulation_runs"
                    if "simulation_runs" in tables
                    else "simulation_cases"
                    if "simulation_cases" in tables
                    else None
                )
                if simulation_table is not None:
                    simulation_columns = self._columns(connection, simulation_table)
                    for row in self._ordered_rows(
                        connection,
                        simulation_table,
                        simulation_columns,
                        session_id=session_id,
                        limit=300,
                    ):
                        value = row_dict(row)
                        sil_row = decode_json_object(value.get("sil_row_json"))
                        performance_row = decode_json_object(
                            value.get("performance_verdict_json")
                        )
                        sil_row.update(performance_row)
                        for name in (
                            "target_band_pass",
                            "run_pass",
                            "truth_apogee_m",
                            "target_apogee_m",
                        ):
                            if name in value and value[name] is not None:
                                sil_row[name] = value[name]
                        value["sil_row"] = sil_row
                        run_id = str(
                            pick(
                                value,
                                "simulation_run_id",
                                "simulation_case_id",
                                default="",
                            )
                        )
                        if run_id:
                            simulation_by_id[run_id] = value
                            simulation_rows.append(value)

                current_hardware = next(
                    (
                        row
                        for row in hardware_rows
                        if str(pick(row, "status", default="")).upper()
                        in ACTIVE_RUN_STATES
                    ),
                    hardware_rows[0] if hardware_rows else None,
                )
                current_run = self._current_run(
                    connection,
                    tables,
                    current_hardware,
                    simulation_by_id,
                )
                recent_runs = [
                    self._normalize_run(row, simulation_by_id)
                    for row in hardware_rows[:20]
                ]
                trends = [
                    self._normalize_trend(row, simulation_by_id)
                    for row in reversed(hardware_rows[:100])
                    if str(pick(row, "status", default="")).upper()
                    not in ACTIVE_RUN_STATES
                ]

                statuses = [
                    str(pick(row, "status", default="")).upper()
                    for row in hardware_rows
                ]
                status_counts: dict[str, int] = {}
                if "hardware_runs" in tables and "status" in hardware_columns:
                    where = (
                        " WHERE session_id=?"
                        if "session_id" in hardware_columns
                        else ""
                    )
                    parameters = (session_id,) if where else ()
                    for row in connection.execute(
                        f'SELECT status,COUNT(*) AS count FROM "hardware_runs"'
                        f"{where} GROUP BY status",
                        parameters,
                    ):
                        status_counts[str(row["status"]).upper()] = int(row["count"])
                if not status_counts:
                    for status in statuses:
                        status_counts[status] = status_counts.get(status, 0) + 1
                completed = sum(
                    count
                    for status, count in status_counts.items()
                    if status not in ACTIVE_RUN_STATES
                    and status not in {"", "PLANNED", "PENDING"}
                )
                passed = sum(
                    count
                    for status, count in status_counts.items()
                    if status in PASS_STATES
                )
                failed = sum(
                    count
                    for status, count in status_counts.items()
                    if status in FAIL_STATES
                )
                sil_evaluated_rows = [
                    row["sil_row"]
                    for row in simulation_rows
                    if isinstance(row.get("sil_row"), dict)
                    and "target_band_pass" in row["sil_row"]
                ]
                sil_target_band_passed = sum(
                    1
                    for row in sil_evaluated_rows
                    if boolean(row, "target_band_pass")
                )
                sil_overall_passed = sum(
                    1
                    for row in sil_evaluated_rows
                    if boolean(row, "run_pass")
                )
                latest_fault = self._latest_fault(
                    connection,
                    tables,
                    session_id,
                    hardware_rows,
                    session,
                )
                dwell_seconds = number(session, "dwell_s", "dwell_seconds") or 0.0
                dwell_remaining = self._dwell_remaining(
                    session,
                    hardware_rows,
                    dwell_seconds,
                )
                campaign_altitude = self._campaign_altitude(
                    connection,
                    tables,
                    database,
                    hardware_rows,
                    simulation_by_id,
                )
                trace_by_hardware_id = {
                    str(trace.get("hardware_run_id")): trace
                    for trace in campaign_altitude["traces"]
                }
                for run in recent_runs + trends:
                    trace = trace_by_hardware_id.get(
                        str(run.get("hardware_run_id"))
                    )
                    if trace is not None:
                        run["rocketpy_truth_apogee_ft"] = (
                            trace.get("truth_apogee_ft")
                            if trace.get("apogee_is_final")
                            else None
                        )
                        run["rocketpy_truth_peak_ft"] = trace.get(
                            "truth_peak_ft"
                        )
                        run["apogee_is_final"] = trace.get(
                            "apogee_is_final", False
                        )
                if current_run is not None:
                    trace = trace_by_hardware_id.get(
                        str(current_run.get("hardware_run_id"))
                    )
                    if trace is not None:
                        current_run["rocketpy_truth_apogee_ft"] = (
                            trace.get("truth_apogee_ft")
                            if trace.get("apogee_is_final")
                            else trace.get("truth_peak_ft")
                        )
                        current_run["apogee_is_final"] = trace.get(
                            "apogee_is_final", False
                        )

                session_state = {
                    "session_id": session_id or None,
                    "status": str(pick(session, "status", default="UNKNOWN")),
                    "started_utc": pick(session, "started_utc"),
                    "finished_utc": pick(session, "finished_utc"),
                    "master_seed": pick(session, "master_seed"),
                    "batch_size": pick(session, "batch_size"),
                    "baseline_interval": pick(session, "baseline_interval"),
                    "completed_runs": completed,
                    "passed_runs": passed,
                    "failed_runs": failed,
                    "sil_evaluated_runs": len(sil_evaluated_rows),
                    "sil_target_band_passed_runs": sil_target_band_passed,
                    "sil_overall_passed_runs": sil_overall_passed,
                    "current_run": (
                        pick(current_run or {}, "hardware_run_id")
                        if current_run is not None
                        else None
                    ),
                    "latest_fault": latest_fault,
                    "dwell_seconds": dwell_seconds,
                    "dwell_remaining_s": dwell_remaining,
                    "stop_reason": pick(session, "stop_reason"),
                    "coupling_mode": pick(
                        session,
                        "coupling_mode",
                        "session_mode",
                        "workflow",
                        "profile",
                        default="forced_replay",
                    ),
                    "configuration_crc": integer(
                        session,
                        "configuration_crc",
                        "config_crc",
                        "config_crc32",
                    ),
                }
                is_variable_hil = str(session_state["coupling_mode"]).upper() in {
                    "VARIABLE_HIL",
                    "CAUSAL_VARIABLE_HIL",
                    "TMC_RAMP_STATE_COUPLED",
                }
                warnings = (
                    [
                        "RocketPy physics is causally coupled to confirmed TMC XACTUAL ramp state; controller request, motor target, feedback, and physics-applied deployment remain distinct channels.",
                        "TMC XACTUAL is driver ramp-generator state, not independent shaft, linkage, or airbrake-position evidence. Results are TMC ramp-state-coupled HIL only.",
                    ]
                    if is_variable_hil
                    else [
                        "RocketPy/SIL physics, STM32 controller output, and forced HIL actuator motion are distinct evidence channels.",
                        "Software HOME/FULL are internal ramp-state evidence, not switches or endstops; TMC XACTUAL is not an independent encoder measurement.",
                    ]
                )
                return {
                    "schema": SCHEMA,
                    "generated_utc": utc_now_text(),
                    "database": {
                        "available": True,
                        "path": str(database),
                        "modified_utc": datetime.fromtimestamp(
                            database.stat().st_mtime, timezone.utc
                        )
                        .isoformat()
                        .replace("+00:00", "Z"),
                        "error": None,
                    },
                    "session": session_state,
                    "current_run": current_run,
                    "recent_runs": recent_runs,
                    "trends": trends,
                    "campaign_altitude": campaign_altitude,
                    "warnings": warnings,
                }
        except (OSError, sqlite3.Error) as error:
            return self._empty_state(database, str(error))

    @staticmethod
    def _first_column(columns: set[str], *candidates: str) -> str | None:
        return next((name for name in candidates if name in columns), None)

    @staticmethod
    def _interpolate_altitude(
        points: list[dict[str, float]], time_s: float
    ) -> float | None:
        if not points or time_s < points[0]["time_s"] or time_s > points[-1]["time_s"]:
            return None
        times = [point["time_s"] for point in points]
        index = bisect_left(times, time_s)
        if index == 0:
            return points[0]["altitude_ft"]
        if index >= len(points):
            return points[-1]["altitude_ft"]
        right = points[index]
        left = points[index - 1]
        duration = right["time_s"] - left["time_s"]
        if duration <= 0.0:
            return right["altitude_ft"]
        fraction = (time_s - left["time_s"]) / duration
        return left["altitude_ft"] + fraction * (
            right["altitude_ft"] - left["altitude_ft"]
        )

    @classmethod
    def _average_altitude_trace(
        cls, traces: list[dict[str, Any]]
    ) -> list[dict[str, Any]]:
        final_traces = [
            trace
            for trace in traces
            if trace.get("apogee_is_final")
            and len(trace.get("points", [])) >= 2
        ]
        if not final_traces:
            return []
        start_s = min(trace["points"][0]["time_s"] for trace in final_traces)
        end_s = max(trace["points"][-1]["time_s"] for trace in final_traces)
        if end_s <= start_s:
            return []
        average: list[dict[str, Any]] = []
        for index in range(CAMPAIGN_AVERAGE_POINTS):
            time_s = start_s + (end_s - start_s) * index / (
                CAMPAIGN_AVERAGE_POINTS - 1
            )
            values = [
                value
                for trace in final_traces
                if (
                    value := cls._interpolate_altitude(trace["points"], time_s)
                )
                is not None
            ]
            if values:
                average.append(
                    {
                        "time_s": time_s,
                        "altitude_ft": sum(values) / len(values),
                        "contributing_runs": len(values),
                    }
                )
        return average

    def _load_altitude_trace(
        self,
        connection: sqlite3.Connection,
        sample_columns: set[str],
        hardware: Mapping[str, Any],
        simulation_by_id: Mapping[str, dict[str, Any]],
    ) -> dict[str, Any] | None:
        hardware_id = str(pick(hardware, "hardware_run_id", default=""))
        if not hardware_id or "hardware_run_id" not in sample_columns:
            return None
        truth_column = self._first_column(
            sample_columns,
            "truth_altitude_m",
            "rocketpy_altitude_m",
            "sil_truth_altitude_m",
            "truth_altitude_ft",
            "rocketpy_altitude_ft",
        )
        if truth_column is None:
            return None
        order_column = self._first_column(
            sample_columns, "sample_index", "event_index"
        ) or "rowid"
        aggregate = connection.execute(
            f'SELECT COUNT(*) AS sample_count, MAX("{truth_column}") AS truth_peak '
            'FROM "samples" WHERE hardware_run_id=?',
            (hardware_id,),
        ).fetchone()
        sample_count = int(aggregate["sample_count"] or 0)
        if sample_count == 0 or aggregate["truth_peak"] is None:
            return None

        stride = max(1, math.ceil(sample_count / CAMPAIGN_TRACE_POINTS))
        rows = list(
            connection.execute(
                f'SELECT * FROM "samples" WHERE hardware_run_id=? AND ('
                f'(CAST("{order_column}" AS INTEGER) % ?)=0 OR '
                f'"{order_column}"=(SELECT MAX("{order_column}") FROM "samples" '
                'WHERE hardware_run_id=?) OR '
                f'"{truth_column}"=(SELECT MAX("{truth_column}") FROM "samples" '
                'WHERE hardware_run_id=?)) '
                f'ORDER BY "{order_column}" ASC',
                (hardware_id, stride, hardware_id, hardware_id),
            )
        )
        points: list[dict[str, float]] = []
        seen: set[tuple[float, float]] = set()
        for row in rows:
            sample = self._normalize_sample(row_dict(row))
            time_s = sample.get("time_s")
            altitude_ft = sample.get("truth_altitude_ft")
            if time_s is None or altitude_ft is None:
                continue
            point_key = (float(time_s), float(altitude_ft))
            if point_key in seen:
                continue
            seen.add(point_key)
            points.append(
                {"time_s": point_key[0], "altitude_ft": point_key[1]}
            )
        points.sort(key=lambda point: point["time_s"])

        status = str(pick(hardware, "status", default="")).upper()
        simulation_id = str(
            pick(hardware, "simulation_run_id", "simulation_case_id", default="")
        )
        simulation = simulation_by_id.get(simulation_id, {})
        sil_row = simulation.get("sil_row", {})
        peak = float(aggregate["truth_peak"])
        peak_ft = peak if truth_column.endswith("_ft") else peak * FEET_PER_METER
        apogee_is_final = status in FINAL_APOGEE_STATES
        return {
            "hardware_run_id": hardware_id,
            "simulation_run_id": simulation_id or None,
            "cycle_index": pick(simulation, "cycle_index", "case_index"),
            "mode": pick(simulation, "mode", "workflow", default="VARIABLE_HIL"),
            "status": pick(hardware, "status"),
            "target_band_pass": (
                boolean(sil_row, "target_band_pass")
                if "target_band_pass" in sil_row
                else None
            ),
            "sil_run_pass": (
                boolean(sil_row, "run_pass")
                if "run_pass" in sil_row
                else None
            ),
            "sample_count": sample_count,
            "truth_peak_ft": peak_ft,
            "truth_apogee_ft": peak_ft if apogee_is_final else None,
            "apogee_is_final": apogee_is_final,
            "points": points,
        }

    def _campaign_altitude(
        self,
        connection: sqlite3.Connection,
        tables: set[str],
        database: Path,
        hardware_rows: list[dict[str, Any]],
        simulation_by_id: Mapping[str, dict[str, Any]],
    ) -> dict[str, Any]:
        empty = {
            "target_apogee_ft": CAMPAIGN_TARGET_APOGEE_FT,
            "trace_count": 0,
            "final_apogee_count": 0,
            "mean_truth_apogee_ft": None,
            "minimum_truth_apogee_ft": None,
            "maximum_truth_apogee_ft": None,
            "mean_target_error_ft": None,
            "traces": [],
            "average_trace": [],
        }
        if "samples" not in tables:
            return empty
        sample_columns = self._columns(connection, "samples")
        if "hardware_run_id" not in sample_columns:
            return empty

        resolved_database = database.resolve()
        with self._campaign_cache_lock:
            if self._campaign_cache_database != resolved_database:
                self._campaign_cache_database = resolved_database
                self._campaign_trace_cache.clear()

        traces: list[dict[str, Any]] = []
        for hardware in reversed(hardware_rows):
            hardware_id = str(pick(hardware, "hardware_run_id", default=""))
            status = str(pick(hardware, "status", default="")).upper()
            trace: dict[str, Any] | None = None
            if status in FINAL_APOGEE_STATES:
                with self._campaign_cache_lock:
                    cached = self._campaign_trace_cache.get(hardware_id)
                    if cached is not None:
                        trace = dict(cached)
                        trace["points"] = list(cached["points"])
            if trace is None:
                trace = self._load_altitude_trace(
                    connection,
                    sample_columns,
                    hardware,
                    simulation_by_id,
                )
                if trace is not None and trace["apogee_is_final"]:
                    with self._campaign_cache_lock:
                        self._campaign_trace_cache[hardware_id] = {
                            **trace,
                            "points": list(trace["points"]),
                        }
            if trace is not None:
                traces.append(trace)

        traces.sort(
            key=lambda trace: (
                integer(trace, "cycle_index", default=0) or 0,
                str(trace.get("hardware_run_id", "")),
            )
        )
        final_apogees = [
            float(trace["truth_apogee_ft"])
            for trace in traces
            if trace.get("apogee_is_final")
            and trace.get("truth_apogee_ft") is not None
        ]
        mean_apogee = (
            sum(final_apogees) / len(final_apogees)
            if final_apogees
            else None
        )
        return {
            "target_apogee_ft": CAMPAIGN_TARGET_APOGEE_FT,
            "trace_count": len(traces),
            "final_apogee_count": len(final_apogees),
            "mean_truth_apogee_ft": mean_apogee,
            "minimum_truth_apogee_ft": min(final_apogees) if final_apogees else None,
            "maximum_truth_apogee_ft": max(final_apogees) if final_apogees else None,
            "mean_target_error_ft": (
                mean_apogee - CAMPAIGN_TARGET_APOGEE_FT
                if mean_apogee is not None
                else None
            ),
            "traces": traces,
            "average_trace": self._average_altitude_trace(traces),
        }

    def _current_run(
        self,
        connection: sqlite3.Connection,
        tables: set[str],
        hardware: dict[str, Any] | None,
        simulation_by_id: Mapping[str, dict[str, Any]],
    ) -> dict[str, Any] | None:
        if hardware is None:
            return None
        hardware_id = str(pick(hardware, "hardware_run_id", default=""))
        simulation_id = str(
            pick(hardware, "simulation_run_id", "simulation_case_id", default="")
        )
        simulation = simulation_by_id.get(simulation_id, {})
        samples: list[dict[str, Any]] = []
        total_sample_count = 0
        if hardware_id and "samples" in tables:
            columns = self._columns(connection, "samples")
            if "hardware_run_id" in columns:
                order_column = (
                    "sample_index"
                    if "sample_index" in columns
                    else "event_index"
                    if "event_index" in columns
                    else "rowid"
                )
                rows = list(
                    connection.execute(
                        f'SELECT * FROM "samples" WHERE hardware_run_id=? '
                        f'ORDER BY "{order_column}" DESC LIMIT 5000',
                        (hardware_id,),
                    )
                )
                total_sample_count = int(
                    connection.execute(
                        'SELECT COUNT(*) FROM "samples" WHERE hardware_run_id=?',
                        (hardware_id,),
                    ).fetchone()[0]
                )
                samples = [
                    self._normalize_sample(row_dict(row))
                    for row in reversed(rows)
                ]
                samples = downsample(samples, 1800)
                if (
                    samples
                    and all(
                        sample.get("sil_actual_deployment_percent") is None
                        for sample in samples
                    )
                    and "events" in tables
                ):
                    event_columns = self._columns(connection, "events")
                    required = {
                        "hardware_run_id",
                        "event_index",
                        "record_json",
                    }
                    if required.issubset(event_columns):
                        deployment_by_event: dict[int, float] = {}
                        event_rows = connection.execute(
                            'SELECT event_index,record_json FROM "events" '
                            'WHERE hardware_run_id=? ORDER BY event_index DESC '
                            'LIMIT 5000',
                            (hardware_id,),
                        )
                        for event_row in event_rows:
                            record = decode_json_object(event_row["record_json"])
                            sil = record.get("sil", {})
                            if not isinstance(sil, Mapping):
                                sil = {}
                            fraction = number(
                                sil,
                                "actual_deployment_fraction",
                                "actual_deploy_fraction",
                            )
                            if fraction is None:
                                fraction = number(
                                    record,
                                    "sil_actual_deployment_fraction",
                                    "rocketpy_virtual_deployment_fraction",
                                )
                            if fraction is not None:
                                deployment_by_event[int(event_row["event_index"])] = (
                                    fraction * 100.0
                                )
                        for sample in samples:
                            event_index = integer(sample, "event_index")
                            if event_index is not None:
                                sample["sil_actual_deployment_percent"] = (
                                    deployment_by_event.get(event_index)
                                )

        return {
            **self._normalize_run(hardware, simulation_by_id),
            "simulation_run_id": simulation_id or None,
            "mode": pick(simulation, "mode"),
            "cycle_index": pick(simulation, "cycle_index", "case_index"),
            "run_seed": pick(simulation, "run_seed"),
            "samples": samples,
            "sample_count": total_sample_count,
            "latest_event_index": (
                max(
                    (
                        int(sample["event_index"])
                        for sample in samples
                        if sample.get("event_index") is not None
                    ),
                    default=None,
                )
            ),
        }

    @staticmethod
    def _normalize_sample(values: Mapping[str, Any]) -> dict[str, Any]:
        time_s = number(
            values,
            "source_time_s",
            "replay_time_s",
            "host_elapsed_s",
            "time_s",
        )
        if time_s is None:
            device_ms = number(values, "device_time_ms", "stm32_time_ms")
            time_s = None if device_ms is None else device_ms / 1000.0
        fault_value = pick(
            values,
            "fault",
            "fault_active",
            "fault_flags",
            "driver_fault",
            default=False,
        )
        try:
            fault_active = bool(int(fault_value))
        except (TypeError, ValueError):
            fault_active = boolean({"value": fault_value}, "value")
        phase = pick(values, "phase", "flight_phase", "stm32_phase")
        phase_names = {
            0: "PadIdle",
            1: "Boost",
            2: "Coast",
            3: "AirbrakeActive",
            4: "Recovery",
            5: "Fault",
        }
        try:
            normalized_phase = phase_names.get(int(phase), str(phase))
        except (TypeError, ValueError):
            normalized_phase = None if phase is None else str(phase)
        schedule_lag_s = number(values, "schedule_lag_s")
        packet_lag_ms = number(values, "packet_lag_ms", "telemetry_lag_ms")
        if packet_lag_ms is None and schedule_lag_s is not None:
            packet_lag_ms = schedule_lag_s * 1000.0
        return {
            "event_index": pick(values, "event_index", "sample_index"),
            "time_s": time_s,
            "truth_altitude_ft": feet_value(
                values,
                ("truth_altitude_m", "rocketpy_altitude_m", "sil_truth_altitude_m"),
                ("truth_altitude_ft", "rocketpy_altitude_ft"),
            ),
            "stm32_altitude_ft": feet_value(
                values,
                ("stm32_altitude_m", "estimated_altitude_m", "ekf_altitude_m"),
                ("stm32_altitude_ft", "estimated_altitude_ft"),
            ),
            "truth_velocity_fps": feet_value(
                values,
                ("truth_velocity_mps", "truth_vertical_velocity_mps", "rocketpy_velocity_mps"),
                ("truth_velocity_fps", "truth_vertical_velocity_fps"),
            ),
            "stm32_velocity_fps": feet_value(
                values,
                ("stm32_velocity_mps", "estimated_velocity_mps", "ekf_velocity_mps"),
                ("stm32_velocity_fps", "estimated_velocity_fps"),
            ),
            "predicted_apogee_ft": feet_value(
                values,
                ("predicted_apogee_m", "stm32_predicted_apogee_m"),
                ("predicted_apogee_ft", "stm32_predicted_apogee_ft"),
            ),
            "target_apogee_ft": feet_value(
                values,
                ("target_apogee_m",),
                ("target_apogee_ft",),
            ),
            "raw_controller_percent": percent_value(
                values,
                (
                    "raw_controller_deploy_fraction",
                    "raw_controller_deployment_fraction",
                    "controller_deploy_fraction",
                    "raw_deploy_fraction",
                ),
                ("raw_controller_deploy_percent", "controller_deploy_percent"),
            ),
            "controller_request_percent": percent_value(
                values,
                (
                    "controller_requested_fraction",
                    "controller_request_fraction",
                    "raw_controller_deployment_fraction",
                    "raw_controller_deploy_fraction",
                    "controller_deploy_fraction",
                ),
                (
                    "controller_requested_percent",
                    "controller_request_percent",
                    "raw_controller_deploy_percent",
                    "controller_deploy_percent",
                ),
            ),
            "actuator_target_percent": percent_value(
                values,
                (
                    "actuator_target_deployment_fraction",
                    "actuator_target_fraction",
                    "motor_target_fraction",
                ),
                ("actuator_target_percent", "motor_target_percent"),
            ),
            "actuator_feedback_percent": percent_value(
                values,
                (
                    "actuator_feedback_deployment_fraction",
                    "xactual_deployment_fraction",
                    "actuator_feedback_fraction",
                    "actuator_xactual_fraction",
                ),
                (
                    "actuator_feedback_percent",
                    "xactual_deployment_percent",
                ),
            ),
            "physics_applied_percent": percent_value(
                values,
                (
                    "physics_applied_deployment_fraction",
                    "plant_applied_deployment_fraction",
                    "rocketpy_applied_deployment_fraction",
                    "physics_applied_fraction",
                ),
                (
                    "physics_applied_percent",
                    "plant_applied_percent",
                ),
            ),
            "sil_actual_deployment_percent": percent_value(
                values,
                (
                    "sil_actual_deployment_fraction",
                    "sil_actual_deploy_fraction",
                    "rocketpy_virtual_deployment_fraction",
                ),
                (
                    "sil_actual_deployment_percent",
                    "rocketpy_virtual_deployment_percent",
                ),
            ),
            "applied_hil_percent": percent_value(
                values,
                (
                    "applied_deploy_fraction",
                    "forced_deploy_fraction",
                    "forced_hil_deployment_fraction",
                    "hil_applied_deploy_fraction",
                ),
                ("applied_deploy_percent", "forced_deploy_percent"),
            ),
            "closed_predicted_apogee_ft": feet_value(
                values,
                (
                    "closed_predicted_apogee_m",
                    "predicted_closed_apogee_m",
                ),
                (
                    "closed_predicted_apogee_ft",
                    "predicted_closed_apogee_ft",
                ),
            ),
            "full_predicted_apogee_ft": feet_value(
                values,
                (
                    "full_predicted_apogee_m",
                    "predicted_full_apogee_m",
                ),
                (
                    "full_predicted_apogee_ft",
                    "predicted_full_apogee_ft",
                ),
            ),
            "target_reachable": optional_boolean(
                values,
                "target_reachable",
                "prediction_target_reachable",
                "reachable",
            ),
            "feedback_age_ms": number(
                values,
                "feedback_age_ms",
                "actuator_feedback_age_ms",
            ),
            "feedback_source": pick(
                values,
                "feedback_source",
                "actuator_feedback_source",
            ),
            "configuration_crc": integer(
                values,
                "configuration_crc",
                "config_crc",
                "config_crc32",
            ),
            "flight_inhibit_flags": integer(values, "flight_inhibit_flags"),
            "actuator_inhibit_flags": integer(values, "actuator_inhibit_flags"),
            "correlated_sequence": integer(
                values,
                "correlated_sequence",
                "simulation_input_sequence",
                "input_sequence",
            ),
            "target_rotations": rotations_value(
                values,
                ("target_rotations",),
                ("target_position_steps", "target_steps", "motor_target_steps"),
            ),
            "xactual_rotations": rotations_value(
                values,
                ("xactual_rotations", "actual_rotations"),
                (
                    "actual_position_steps",
                    "xactual_steps",
                    "actual_steps",
                    "motor_actual_steps",
                ),
            ),
            "software_home_active": boolean(
                values,
                "software_home_active",
                "home_active",
                "home_limit_active",
            ),
            "software_full_active": boolean(
                values,
                "software_full_active",
                "full_active",
                "full_limit_active",
            ),
            "geometry_plausible": boolean(
                values,
                "geometry_plausible",
                "limits_plausible",
                "limit_state_plausible",
            ),
            "phase": normalized_phase,
            "fault_active": fault_active,
            "packet_lag_ms": packet_lag_ms,
            "skipped_host_samples": integer(
                values,
                "skipped_host_samples",
                "skipped_samples",
                default=0,
            ),
        }

    @staticmethod
    def _normalize_run(
        hardware: Mapping[str, Any],
        simulation_by_id: Mapping[str, dict[str, Any]],
    ) -> dict[str, Any]:
        simulation_id = str(
            pick(hardware, "simulation_run_id", "simulation_case_id", default="")
        )
        simulation = simulation_by_id.get(simulation_id, {})
        sil_row = simulation.get("sil_row", {})
        tracking_steps = number(hardware, "max_tracking_error_steps")
        return {
            "hardware_run_id": pick(hardware, "hardware_run_id"),
            "simulation_run_id": simulation_id or None,
            "cycle_index": pick(simulation, "cycle_index", "case_index"),
            "mode": pick(simulation, "mode", "workflow", default="VARIABLE_HIL"),
            "status": pick(hardware, "status"),
            "target_band_pass": (
                boolean(sil_row, "target_band_pass")
                if "target_band_pass" in sil_row
                else None
            ),
            "sil_run_pass": (
                boolean(sil_row, "run_pass")
                if "run_pass" in sil_row
                else None
            ),
            "started_utc": pick(hardware, "started_utc"),
            "finished_utc": pick(hardware, "finished_utc"),
            "open_time_s": number(hardware, "open_time_s"),
            "close_time_s": number(hardware, "close_time_s"),
            "max_tracking_error_rotations": (
                None
                if tracking_steps is None
                else tracking_steps / STEPS_PER_ROTATION
            ),
            "full_reached_s": number(hardware, "full_reached_s"),
            "home_reached_s": number(hardware, "home_reached_s"),
            "failure_type": pick(hardware, "failure_type"),
            "failure_message": pick(hardware, "failure_message"),
            "hardware_safety_verdict": pick(
                hardware,
                "hardware_safety_verdict",
                "safety_verdict",
            ),
            "performance_verdict": pick(
                hardware,
                "performance_verdict",
                "target_performance_verdict",
            ),
            "coupling_mode": pick(
                hardware,
                "coupling_mode",
                "workflow",
                default=pick(simulation, "coupling_mode", "workflow", "mode"),
            ),
            "configuration_crc": integer(
                hardware,
                "configuration_crc",
                "config_crc",
                "config_crc32",
            ),
        }

    @staticmethod
    def _normalize_trend(
        hardware: Mapping[str, Any],
        simulation_by_id: Mapping[str, dict[str, Any]],
    ) -> dict[str, Any]:
        normalized = DashboardRepository._normalize_run(
            hardware, simulation_by_id
        )
        normalized["sequence"] = pick(
            simulation_by_id.get(
                str(
                    pick(
                        hardware,
                        "simulation_run_id",
                        "simulation_case_id",
                        default="",
                    )
                ),
                {},
            ),
            "cycle_index",
            "case_index",
        )
        return normalized

    @staticmethod
    def _dwell_remaining(
        session: Mapping[str, Any],
        hardware_rows: list[dict[str, Any]],
        dwell_seconds: float,
    ) -> float:
        if dwell_seconds <= 0.0 or not hardware_rows:
            return 0.0
        session_status = str(pick(session, "status", default="")).upper()
        if session_status in TERMINAL_SESSION_STATES:
            return 0.0
        finished = parse_utc(pick(hardware_rows[0], "finished_utc"))
        if finished is None:
            return 0.0
        elapsed = (datetime.now(timezone.utc) - finished).total_seconds()
        return round(max(0.0, dwell_seconds - elapsed), 1)

    def _latest_fault(
        self,
        connection: sqlite3.Connection,
        tables: set[str],
        session_id: str,
        hardware_rows: list[dict[str, Any]],
        session: Mapping[str, Any],
    ) -> str | None:
        session_status = str(pick(session, "status", default="")).upper()
        direct = pick(session, "stop_reason", "latest_fault")
        if direct and session_status in FAIL_STATES:
            return str(direct)
        for run in hardware_rows:
            run_status = str(pick(run, "status", default="")).upper()
            message = pick(run, "failure_message", "failure_type")
            if message and run_status in FAIL_STATES:
                return str(message)
        if "events" not in tables:
            return None
        columns = self._columns(connection, "events")
        if "record_json" not in columns:
            return None
        where = " WHERE session_id=?" if "session_id" in columns else ""
        parameters = (session_id,) if where else ()
        for row in connection.execute(
            f'SELECT record_json FROM "events"{where} ORDER BY rowid DESC LIMIT 50',
            parameters,
        ):
            record = decode_json_object(row[0])
            event_name = str(record.get("event", "")).lower()
            if "fault" in event_name or "failed" in event_name:
                return str(
                    pick(
                        record,
                        "message",
                        "reason",
                        "failure_message",
                        default=event_name,
                    )
                )
        return None

    def event_backfill(
        self,
        hardware_run_id: str,
        after_event_index: int,
        limit: int = 500,
    ) -> list[dict[str, Any]]:
        database = self.locator.locate()
        if database is None:
            return []
        with closing(self._connect(database)) as connection:
            tables = self._tables(connection)
            if "events" not in tables:
                return []
            columns = self._columns(connection, "events")
            required = {"hardware_run_id", "event_index", "record_json"}
            if not required.issubset(columns):
                return []
            rows = connection.execute(
                'SELECT hardware_run_id,event_index,record_json FROM "events" '
                "WHERE hardware_run_id=? AND event_index>? "
                "ORDER BY event_index ASC LIMIT ?",
                (hardware_run_id, after_event_index, min(max(limit, 1), 2000)),
            )
            result = []
            for row in rows:
                record = decode_json_object(row["record_json"])
                record.setdefault("hardware_run_id", row["hardware_run_id"])
                record.setdefault("event_index", row["event_index"])
                result.append(record)
            return result


class EventBus:
    """Bounded in-memory fan-out for best-effort UDP events."""

    def __init__(self, maximum: int = 3000) -> None:
        self._events: deque[tuple[int, dict[str, Any]]] = deque(maxlen=maximum)
        self._condition = threading.Condition()
        self._sequence = 0

    def publish(self, event: Mapping[str, Any]) -> int:
        with self._condition:
            self._sequence += 1
            record = dict(event)
            record["dashboard_sequence"] = self._sequence
            record["dashboard_received_utc"] = utc_now_text()
            self._events.append((self._sequence, record))
            self._condition.notify_all()
            return self._sequence

    def after(self, sequence: int) -> tuple[list[tuple[int, dict[str, Any]]], bool]:
        with self._condition:
            gap = bool(self._events and sequence < self._events[0][0] - 1)
            return [
                item for item in self._events if item[0] > sequence
            ], gap

    def wait(self, sequence: int, timeout: float) -> None:
        with self._condition:
            if self._sequence <= sequence:
                self._condition.wait(timeout)


class UdpEventReceiver(threading.Thread):
    def __init__(self, host: str, port: int, bus: EventBus) -> None:
        super().__init__(name="ambar-dashboard-udp", daemon=True)
        self.host = host
        self.port = port
        self.bus = bus
        self.stop_event = threading.Event()
        self.socket: socket.socket | None = None
        self.error: str | None = None

    def run(self) -> None:
        try:
            receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            receiver.bind((self.host, self.port))
            receiver.settimeout(0.5)
            self.socket = receiver
            while not self.stop_event.is_set():
                try:
                    payload, _address = receiver.recvfrom(65507)
                except socket.timeout:
                    continue
                except OSError:
                    if self.stop_event.is_set():
                        break
                    raise
                for line in payload.decode("utf-8", errors="replace").splitlines():
                    if not line.strip():
                        continue
                    try:
                        event = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if isinstance(event, dict):
                        self.bus.publish(event)
        except OSError as error:
            self.error = str(error)
        finally:
            if self.socket is not None:
                self.socket.close()
                self.socket = None

    def stop(self) -> None:
        self.stop_event.set()
        if self.socket is not None:
            self.socket.close()


def snapshot_html(state: Mapping[str, Any], ui_root: Path) -> str:
    index = (ui_root / "index.html").read_text(encoding="utf-8")
    styles = (ui_root / "styles.css").read_text(encoding="utf-8")
    script = (ui_root / "app.js").read_text(encoding="utf-8")
    payload = json.dumps(state, separators=(",", ":"), ensure_ascii=False).replace(
        "<", "\\u003c"
    )
    index = index.replace(
        '<link rel="stylesheet" href="styles.css">',
        f"<style>\n{styles}\n</style>",
    )
    index = index.replace(
        '<script src="app.js" defer></script>',
        f"<script>window.AMBAR_SNAPSHOT_STATE={payload};</script>",
    )
    index = index.replace(
        "</body>",
        f"<script>\n{script}\n</script>\n</body>",
    )
    index = index.replace(
        "</head>",
        (
            "<meta http-equiv=\"Content-Security-Policy\" "
            "content=\"default-src 'none'; img-src data:; style-src 'unsafe-inline'; "
            "script-src 'unsafe-inline'; connect-src 'none'; base-uri 'none'\">"
            "\n</head>"
        ),
    )
    return index


def write_snapshot(path: Path, state: Mapping[str, Any], ui_root: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(snapshot_html(state, ui_root), encoding="utf-8")
    temporary.replace(path)


def write_snapshot_set(
    reports: Path,
    state: Mapping[str, Any],
    ui_root: Path,
    *,
    checkpoint_completed: int | None = None,
    mirror_root: Path | None = None,
) -> None:
    """Write local and finalized-mirror copies of a portable dashboard snapshot."""

    names = ["latest.html"]
    if checkpoint_completed is not None:
        names.append(f"checkpoint-{checkpoint_completed:06d}.html")
    destinations = [reports]
    if mirror_root is not None:
        destinations.append(mirror_root / "reports")
    for destination in destinations:
        for name in names:
            write_snapshot(destination / name, state, ui_root)


class SnapshotMonitor(threading.Thread):
    def __init__(
        self,
        repository: DashboardRepository,
        ui_root: Path,
        checkpoint_runs: int,
        mirror_root: Path | None,
    ) -> None:
        super().__init__(name="ambar-dashboard-snapshots", daemon=True)
        self.repository = repository
        self.ui_root = ui_root
        self.checkpoint_runs = checkpoint_runs
        self.mirror_root = mirror_root
        self.stop_event = threading.Event()
        self.last_checkpoint = -1

    def run(self) -> None:
        while not self.stop_event.wait(2.0):
            state = self.repository.state()
            database_text = pick(state.get("database", {}), "path")
            if not database_text:
                continue
            completed = int(pick(state.get("session", {}), "completed_runs", default=0))
            status = str(pick(state.get("session", {}), "status", default="")).upper()
            due_checkpoint = (
                self.checkpoint_runs > 0
                and completed > 0
                and completed % self.checkpoint_runs == 0
                and completed != self.last_checkpoint
            )
            due_terminal = status in TERMINAL_SESSION_STATES and completed != self.last_checkpoint
            if not due_checkpoint and not due_terminal:
                continue
            reports = Path(str(database_text)).resolve().parent / "reports"
            try:
                write_snapshot_set(
                    reports,
                    state,
                    self.ui_root,
                    checkpoint_completed=completed if due_checkpoint else None,
                    mirror_root=self.mirror_root,
                )
                self.last_checkpoint = completed
            except OSError as error:
                print(f"[dashboard] snapshot write failed: {error}", file=sys.stderr)

    def stop(self) -> None:
        self.stop_event.set()


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        address: tuple[str, int],
        handler: type[SimpleHTTPRequestHandler],
        *,
        repository: DashboardRepository,
        bus: EventBus,
        udp_receiver: UdpEventReceiver,
        ui_root: Path,
    ) -> None:
        super().__init__(address, handler)
        self.repository = repository
        self.bus = bus
        self.udp_receiver = udp_receiver
        self.ui_root = ui_root


class DashboardHandler(SimpleHTTPRequestHandler):
    server: DashboardServer

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        ui_root = kwargs.pop("ui_root", DEFAULT_UI_ROOT)
        super().__init__(*args, directory=str(ui_root), **kwargs)

    def log_message(self, format_string: str, *args: Any) -> None:
        sys.stdout.write("[dashboard] " + format_string % args + "\n")

    def send_json(self, payload: Any, status: int = 200) -> None:
        body = json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode(
            "utf-8"
        )
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/health":
            database = self.server.repository.locator.locate()
            self.send_json(
                {
                    "ok": True,
                    "schema": SCHEMA,
                    "databaseAvailable": database is not None,
                    "udpPort": self.server.udp_receiver.port,
                    "udpError": self.server.udp_receiver.error,
                }
            )
            return
        if parsed.path == "/api/state":
            self.send_json(self.server.repository.state())
            return
        if parsed.path == "/api/events/backfill":
            query = parse_qs(parsed.query)
            hardware_id = query.get("hardware_run_id", [""])[0]
            try:
                after_index = int(query.get("after_event_index", ["-1"])[0])
            except ValueError:
                self.send_json({"error": "after_event_index must be an integer"}, 400)
                return
            if not hardware_id:
                self.send_json({"error": "hardware_run_id is required"}, 400)
                return
            self.send_json(
                {
                    "events": self.server.repository.event_backfill(
                        hardware_id, after_index
                    )
                }
            )
            return
        if parsed.path == "/api/events":
            self.send_event_stream()
            return
        if parsed.path == "/":
            self.path = "/index.html"
        super().do_GET()

    def end_headers(self) -> None:
        if not self.path.startswith("/api/"):
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
        super().end_headers()

    def send_event_stream(self) -> None:
        try:
            sequence = int(self.headers.get("Last-Event-ID", "0") or "0")
        except ValueError:
            sequence = 0
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        try:
            while True:
                events, gap = self.server.bus.after(sequence)
                if gap:
                    self.wfile.write(b"event: reset\ndata: {\"reason\":\"buffer_gap\"}\n\n")
                    self.wfile.flush()
                for event_sequence, record in events:
                    body = json.dumps(
                        record, separators=(",", ":"), ensure_ascii=False
                    ).encode("utf-8")
                    self.wfile.write(f"id: {event_sequence}\n".encode("ascii"))
                    self.wfile.write(b"event: update\n")
                    self.wfile.write(b"data: " + body + b"\n\n")
                    sequence = event_sequence
                if events:
                    self.wfile.flush()
                else:
                    self.wfile.write(b": heartbeat\n\n")
                    self.wfile.flush()
                    self.server.bus.wait(sequence, 10.0)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return


def create_server(
    host: str,
    port: int,
    udp_port: int,
    repository: DashboardRepository,
    ui_root: Path,
) -> tuple[DashboardServer, UdpEventReceiver]:
    bus = EventBus()
    receiver = UdpEventReceiver(host, udp_port, bus)

    class BoundHandler(DashboardHandler):
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, ui_root=ui_root, **kwargs)

    server = DashboardServer(
        (host, port),
        BoundHandler,
        repository=repository,
        bus=bus,
        udp_receiver=receiver,
        ui_root=ui_root,
    )
    return server, receiver


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Serve the read-only AMBAR continuous-HIL dashboard."
    )
    parser.add_argument("--db", type=Path, help="campaign.sqlite3 path; it may not exist yet")
    parser.add_argument("--results-root", type=Path, default=DEFAULT_RESULTS_ROOT)
    parser.add_argument("--mirror-root", type=Path, default=DEFAULT_MIRROR_ROOT)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=52101)
    parser.add_argument("--udp-port", type=int, default=52102)
    parser.add_argument("--ui-root", type=Path, default=DEFAULT_UI_ROOT)
    parser.add_argument("--checkpoint-runs", type=int, default=10)
    parser.add_argument("--snapshot", type=Path)
    parser.add_argument("--snapshot-latest", action="store_true")
    parser.add_argument("--snapshot-only", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.host not in {"127.0.0.1", "localhost"}:
        raise SystemExit("The AMBAR dashboard may bind only to localhost.")
    if not 1 <= args.port <= 65535 or not 1 <= args.udp_port <= 65535:
        raise SystemExit("HTTP and UDP ports must be within 1..65535.")
    ui_root = args.ui_root.resolve()
    for asset in ("index.html", "styles.css", "app.js"):
        if not (ui_root / asset).is_file():
            raise SystemExit(f"Dashboard asset is missing: {ui_root / asset}")

    explicit_db = args.db.resolve() if args.db is not None else None
    locator = DatabaseLocator(explicit_db, args.results_root.resolve())
    repository = DashboardRepository(locator)

    if args.snapshot_only or args.snapshot or args.snapshot_latest:
        state = repository.state()
        database_text = pick(state.get("database", {}), "path")
        if not database_text:
            if args.snapshot_only:
                print("No completed dashboard database is available for a snapshot.", file=sys.stderr)
                return 2
        else:
            if args.snapshot is not None:
                output = args.snapshot.resolve()
                write_snapshot(output, state, ui_root)
            else:
                reports = Path(str(database_text)).resolve().parent / "reports"
                output = reports / "latest.html"
                write_snapshot_set(
                    reports,
                    state,
                    ui_root,
                    mirror_root=args.mirror_root.resolve(),
                )
            print(f"AMBAR dashboard snapshot: {output}")
        if args.snapshot_only:
            return 0

    server, receiver = create_server(
        "127.0.0.1",
        args.port,
        args.udp_port,
        repository,
        ui_root,
    )
    snapshot_monitor = SnapshotMonitor(
        repository,
        ui_root,
        max(args.checkpoint_runs, 0),
        args.mirror_root.resolve(),
    )
    receiver.start()
    snapshot_monitor.start()
    print(
        f"AMBAR Continuous HIL Dashboard: http://127.0.0.1:{args.port} "
        f"(UDP 127.0.0.1:{args.udp_port})",
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        snapshot_monitor.stop()
        receiver.stop()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
