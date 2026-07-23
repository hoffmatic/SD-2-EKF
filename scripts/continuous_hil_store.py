"""Durable session storage for AMBAR continuous RocketPy/HIL testing.

SQLite in WAL mode is the live query source.  Per-run JSONL remains the
append-only recovery record, while CSV files are atomically regenerated
portable exports after each finalized hardware attempt.
"""

from __future__ import annotations

import csv
from datetime import datetime, timezone
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import sqlite3
import time
from typing import Any, Iterable, Mapping
import uuid


SCHEMA_VERSION = 1
MIN_FREE_BYTES = 5 * 1024 * 1024 * 1024
SIMULATION_MANIFEST = "simulation_run.json"
HARDWARE_MANIFEST = "hardware_run.json"
SIMULATION_ARTIFACTS = (
    "resolved_config.json",
    "rocketpy_profile.csv",
    "sil_timeseries.csv.gz",
    "sil_summary.json",
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def default_results_root() -> Path:
    local_app_data = os.environ.get("LOCALAPPDATA")
    if not local_app_data:
        local_app_data = str(Path.home() / "AppData" / "Local")
    return Path(local_app_data) / "AMBAR" / "TestRuns"


def _json(value: Any) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )


def _read_json_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON artifact must contain an object: {path}")
    return value


def _sha256_json(value: Any) -> str:
    return hashlib.sha256(_json(value).encode("utf-8")).hexdigest().upper()


def _parse_csv_scalar(value: str) -> Any:
    text = value.strip()
    if not text:
        return None
    lowered = text.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    try:
        integer = int(text, 10)
    except ValueError:
        pass
    else:
        return integer
    try:
        return float(text)
    except ValueError:
        return text


def _file_time_utc(path: Path, fallback: str) -> str:
    if not path.exists():
        return fallback
    return datetime.fromtimestamp(
        path.stat().st_mtime,
        timezone.utc,
    ).isoformat().replace("+00:00", "Z")


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
    fieldnames: list[str],
    rows: Iterable[Mapping[str, Any]],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=fieldnames,
            extrasaction="ignore",
        )
        writer.writeheader()
        writer.writerows(rows)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


class ContinuousHilStore:
    """Own one session database and its atomic portable exports."""

    def __init__(self, session_dir: str | Path) -> None:
        self.session_dir = Path(session_dir).resolve()
        self.db_path = self.session_dir / "campaign.sqlite3"
        if not self.db_path.is_file():
            raise FileNotFoundError(f"session database not found: {self.db_path}")
        self.connection = sqlite3.connect(
            self.db_path,
            timeout=10.0,
            isolation_level=None,
        )
        try:
            self.connection.row_factory = sqlite3.Row
            self._configure()
            quick_check = self.connection.execute(
                "PRAGMA quick_check"
            ).fetchone()
            if quick_check is None or str(quick_check[0]).lower() != "ok":
                raise sqlite3.DatabaseError(
                    f"session database failed quick_check: {quick_check}"
                )
            required_tables = {
                "sessions",
                "simulation_runs",
                "hardware_runs",
                "events",
                "samples",
            }
            actual_tables = {
                str(row[0])
                for row in self.connection.execute(
                    "SELECT name FROM sqlite_master WHERE type='table'"
                )
            }
            missing_tables = required_tables - actual_tables
            if missing_tables:
                raise sqlite3.DatabaseError(
                    "session database is missing tables: "
                    + ", ".join(sorted(missing_tables))
                )
            self._ensure_compatible_schema(self.connection)
        except BaseException:
            self.connection.close()
            raise
        self._last_event_commit_s = time.monotonic()

    @classmethod
    def create(
        cls,
        results_root: str | Path,
        session_id: str,
        *,
        master_seed: int,
        batch_size: int,
        baseline_interval: int,
        dwell_s: float,
        settings: Mapping[str, Any],
    ) -> "ContinuousHilStore":
        if not session_id or any(
            character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_"
            for character in session_id
        ):
            raise ValueError("session ID may contain only letters, digits, '-' and '_'")
        session_dir = Path(results_root).resolve() / session_id
        session_dir.mkdir(parents=True, exist_ok=True)
        db_path = session_dir / "campaign.sqlite3"
        if db_path.exists():
            raise FileExistsError(f"session already exists: {session_dir}")
        connection = sqlite3.connect(db_path, timeout=10.0, isolation_level=None)
        try:
            connection.row_factory = sqlite3.Row
            cls._configure_connection(connection)
            cls._create_schema(connection)
            now = utc_now()
            connection.execute(
                """
                INSERT INTO sessions(
                    session_id, schema_version, status, started_utc, updated_utc,
                    master_seed, batch_size, baseline_interval, dwell_s,
                    settings_json
                ) VALUES (?, ?, 'RUNNING', ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    session_id,
                    SCHEMA_VERSION,
                    now,
                    now,
                    master_seed,
                    batch_size,
                    baseline_interval,
                    dwell_s,
                    _json(dict(settings)),
                ),
            )
        finally:
            connection.close()
        store = cls(session_dir)
        store._write_session_json()
        return store

    @classmethod
    def rebuild_database(
        cls,
        session_dir: str | Path,
    ) -> "ContinuousHilStore":
        """Atomically rebuild a missing/corrupt database from durable evidence.

        This operation is intentionally explicit.  It never treats a run
        without a final verdict and complete JSONL record as passed, never
        infers HOME after a restart, and archives existing database/WAL files
        before installing the integrity-checked replacement.
        """

        session_dir = Path(session_dir).resolve()
        session_path = session_dir / "session.json"
        if not session_path.is_file():
            raise FileNotFoundError(
                f"cannot rebuild without session.json: {session_path}"
            )
        session_document = _read_json_object(session_path)
        if int(session_document.get("schema_version", -1)) != SCHEMA_VERSION:
            raise ValueError(
                "session.json schema version is not supported for rebuild"
            )
        session_id = str(session_document.get("session_id", ""))
        if not session_id:
            raise ValueError("session.json does not contain a session_id")
        settings = session_document.get("settings", {})
        if not isinstance(settings, Mapping):
            raise ValueError("session.json settings must be an object")
        started_utc = str(
            session_document.get("started_utc")
            or _file_time_utc(session_path, utc_now())
        )
        rebuilt_utc = utc_now()
        previous_status = str(
            session_document.get("status", "STOPPED")
        ).upper()
        rebuilt_status = (
            "STOPPED" if previous_status == "RUNNING" else previous_status
        )
        previous_reason = session_document.get("stop_reason")
        rebuilt_reason = (
            "database rebuilt from durable evidence; explicit resume required"
            if previous_status == "RUNNING"
            else previous_reason
        )
        master_seed = int(session_document["master_seed"])
        batch_size = int(session_document["batch_size"])
        baseline_interval = int(session_document["baseline_interval"])
        dwell_s = float(session_document["dwell_s"])
        if batch_size <= 0 or baseline_interval <= 0 or dwell_s < 0.0:
            raise ValueError("session.json contains invalid campaign settings")

        parameter_base_fields = {
            "cycle_index",
            "simulation_run_id",
            "mode",
            "run_seed",
            "batch_index",
            "batch_position",
            "random_index",
            "resolved_config_sha256",
        }
        simulations: dict[str, dict[str, Any]] = {}
        hardware_csv: dict[str, dict[str, Any]] = {}
        source_files: set[str] = {"session.json"}
        limitations: list[str] = []
        limitations.append(
            "Legacy planned cases that existed only in the lost database and "
            "have no simulation_run.json, parameters.csv row, or artifact "
            "directory cannot be enumerated. New sessions persist one "
            "simulation_run.json per plan to remove this legacy gap."
        )

        def batch_layout(cycle_index: int) -> tuple[int, int, str, int | None]:
            kinds: list[str] = []
            random_seen = 0
            for offset in range(batch_size):
                kinds.append("monte_carlo")
                random_seen += 1
                if (offset + 1) % baseline_interval == 0:
                    kinds.append("baseline")
            span = len(kinds)
            batch_index = cycle_index // span
            batch_position = cycle_index % span
            mode = kinds[batch_position]
            random_index = None
            if mode == "monte_carlo":
                random_before = sum(
                    kind == "monte_carlo"
                    for kind in kinds[:batch_position]
                )
                random_index = batch_index * batch_size + random_before
            return batch_index, batch_position, mode, random_index

        def derive_seed(mode: str, random_index: int | None) -> int:
            seed_index = (
                0x7FFFFFFF if mode == "baseline" else int(random_index or 0)
            )
            payload = f"AMBAR:{master_seed}:{seed_index}".encode("ascii")
            return int.from_bytes(hashlib.sha256(payload).digest()[:4], "big")

        def simulation_defaults(
            simulation_run_id: str,
            cycle_index: int,
        ) -> dict[str, Any]:
            batch_index, batch_position, mode, random_index = batch_layout(
                cycle_index
            )
            return {
                "simulation_run_id": simulation_run_id,
                "session_id": session_id,
                "cycle_index": cycle_index,
                "random_index": random_index,
                "batch_index": batch_index,
                "batch_position": batch_position,
                "mode": mode,
                "run_seed": derive_seed(mode, random_index),
                "status": "PLANNED",
                "planned_utc": started_utc,
                "started_utc": None,
                "finished_utc": None,
                "sampled_inputs": {},
                "resolved_config": None,
                "resolved_config_sha256": None,
                "sil_row": None,
                "profile_path": None,
                "sil_timeseries_path": None,
                "failure_reason": None,
            }

        def ensure_simulation(
            simulation_run_id: str,
            cycle_hint: Any = None,
        ) -> dict[str, Any]:
            existing = simulations.get(simulation_run_id)
            if existing is not None:
                return existing
            try:
                cycle_index = int(cycle_hint)
            except (TypeError, ValueError):
                match = re.search(r"-sim-(\d+)$", simulation_run_id)
                if match is None:
                    raise ValueError(
                        "cannot recover cycle index for simulation "
                        f"{simulation_run_id}"
                    )
                cycle_index = int(match.group(1))
            value = simulation_defaults(simulation_run_id, cycle_index)
            simulations[simulation_run_id] = value
            return value

        parameters_path = session_dir / "parameters.csv"
        if parameters_path.is_file():
            source_files.add("parameters.csv")
            with parameters_path.open(
                "r",
                encoding="utf-8-sig",
                newline="",
            ) as handle:
                for row in csv.DictReader(handle):
                    simulation_run_id = str(
                        row.get("simulation_run_id", "")
                    ).strip()
                    if not simulation_run_id:
                        continue
                    value = ensure_simulation(
                        simulation_run_id,
                        row.get("cycle_index"),
                    )
                    for key in (
                        "cycle_index",
                        "random_index",
                        "batch_index",
                        "batch_position",
                        "run_seed",
                    ):
                        parsed = _parse_csv_scalar(str(row.get(key, "")))
                        if parsed is not None:
                            value[key] = int(parsed)
                    if row.get("mode"):
                        value["mode"] = str(row["mode"])
                    if row.get("resolved_config_sha256"):
                        value["resolved_config_sha256"] = str(
                            row["resolved_config_sha256"]
                        )
                    value["sampled_inputs"] = {
                        key: parsed
                        for key, raw in row.items()
                        if key not in parameter_base_fields
                        and (parsed := _parse_csv_scalar(str(raw))) is not None
                    }

        runs_path = session_dir / "runs.csv"
        if runs_path.is_file():
            source_files.add("runs.csv")
            with runs_path.open(
                "r",
                encoding="utf-8-sig",
                newline="",
            ) as handle:
                for row in csv.DictReader(handle):
                    simulation_run_id = str(
                        row.get("simulation_run_id", "")
                    ).strip()
                    if simulation_run_id:
                        value = ensure_simulation(
                            simulation_run_id,
                            row.get("cycle_index"),
                        )
                        if row.get("mode"):
                            value["mode"] = str(row["mode"])
                        if row.get("run_seed"):
                            value["run_seed"] = int(row["run_seed"])
                        sil_row = {
                            key[4:]: _parse_csv_scalar(str(raw))
                            for key, raw in row.items()
                            if key.startswith("sil_") and str(raw).strip()
                        }
                        if sil_row:
                            value["sil_row"] = sil_row
                    hardware_run_id = str(
                        row.get("hardware_run_id", "")
                    ).strip()
                    if hardware_run_id:
                        hardware_csv[hardware_run_id] = dict(row)

        simulation_root = session_dir / "simulation_runs"
        if simulation_root.is_dir():
            for simulation_dir in sorted(
                path for path in simulation_root.iterdir() if path.is_dir()
            ):
                simulation_run_id = simulation_dir.name
                manifest_path = simulation_dir / SIMULATION_MANIFEST
                manifest: dict[str, Any] = {}
                if manifest_path.is_file():
                    manifest = _read_json_object(manifest_path)
                    source_files.add(
                        str(manifest_path.relative_to(session_dir))
                    )
                value = ensure_simulation(
                    simulation_run_id,
                    manifest.get("cycle_index"),
                )
                for key in (
                    "random_index",
                    "batch_index",
                    "batch_position",
                    "run_seed",
                ):
                    if manifest.get(key) is not None:
                        value[key] = int(manifest[key])
                for key in (
                    "mode",
                    "planned_utc",
                    "started_utc",
                    "finished_utc",
                    "failure_reason",
                    "resolved_config_sha256",
                ):
                    if manifest.get(key) is not None:
                        value[key] = manifest[key]
                if isinstance(manifest.get("sampled_inputs"), Mapping):
                    value["sampled_inputs"] = dict(
                        manifest["sampled_inputs"]
                    )
                if isinstance(manifest.get("resolved_config"), Mapping):
                    value["resolved_config"] = dict(
                        manifest["resolved_config"]
                    )
                if isinstance(manifest.get("sil_row"), Mapping):
                    value["sil_row"] = dict(manifest["sil_row"])
                if manifest.get("status") in ("ERROR", "PLANNED"):
                    value["status"] = str(manifest["status"])

                resolved_path = simulation_dir / "resolved_config.json"
                profile_path = simulation_dir / "rocketpy_profile.csv"
                timeseries_path = simulation_dir / "sil_timeseries.csv.gz"
                summary_path = simulation_dir / "sil_summary.json"
                complete = all(
                    (simulation_dir / name).is_file()
                    for name in SIMULATION_ARTIFACTS
                )
                if resolved_path.is_file():
                    value["resolved_config"] = _read_json_object(
                        resolved_path
                    )
                if summary_path.is_file():
                    value["sil_row"] = _read_json_object(summary_path)
                    summary = value["sil_row"]
                    for key, target in (
                        ("run_index", "cycle_index"),
                        ("sample_index", "random_index"),
                        ("mode", "mode"),
                        ("run_seed", "run_seed"),
                        ("config_sha256", "resolved_config_sha256"),
                    ):
                        if summary.get(key) is not None:
                            value[target] = summary[key]
                if complete:
                    value["status"] = "SIL_COMPLETE"
                    value["profile_path"] = str(profile_path.resolve())
                    value["sil_timeseries_path"] = str(
                        timeseries_path.resolve()
                    )
                    value["finished_utc"] = (
                        value.get("finished_utc")
                        or _file_time_utc(summary_path, rebuilt_utc)
                    )
                    if value["resolved_config_sha256"] is None:
                        value["resolved_config_sha256"] = _sha256_json(
                            value["resolved_config"]
                        )
                elif manifest.get("status") == "SIL_RUNNING":
                    value["status"] = "PLANNED"
                    value["failure_reason"] = None
                elif any(
                    (simulation_dir / name).exists()
                    for name in SIMULATION_ARTIFACTS
                ):
                    value["status"] = "ERROR"
                    value["failure_reason"] = (
                        "incomplete simulation evidence discovered during "
                        "database rebuild"
                    )

        hardware_records: list[dict[str, Any]] = []
        run_root = session_dir / "runs"
        hardware_directories = (
            sorted(path for path in run_root.iterdir() if path.is_dir())
            if run_root.is_dir()
            else []
        )
        discovered_hardware_ids: set[str] = set()
        for hardware_dir in hardware_directories:
            hardware_run_id = hardware_dir.name
            discovered_hardware_ids.add(hardware_run_id)
            replay_dir = hardware_dir / "replay"
            hardware_manifest_path = hardware_dir / HARDWARE_MANIFEST
            replay_manifest_path = replay_dir / "manifest.json"
            verdict_path = replay_dir / "verdict.json"
            hardware_manifest = (
                _read_json_object(hardware_manifest_path)
                if hardware_manifest_path.is_file()
                else {}
            )
            replay_manifest = (
                _read_json_object(replay_manifest_path)
                if replay_manifest_path.is_file()
                else {}
            )
            verdict = (
                _read_json_object(verdict_path)
                if verdict_path.is_file()
                else (
                    dict(hardware_manifest["verdict"])
                    if isinstance(
                        hardware_manifest.get("verdict"),
                        Mapping,
                    )
                    else None
                )
            )
            for path in (
                hardware_manifest_path,
                replay_manifest_path,
                verdict_path,
            ):
                if path.is_file():
                    source_files.add(str(path.relative_to(session_dir)))

            event_path = replay_dir / "events.jsonl"
            if not event_path.is_file():
                event_path = replay_dir / "packets.jsonl"
            records: list[dict[str, Any]] = []
            if event_path.is_file():
                source_files.add(str(event_path.relative_to(session_dir)))
                cls._truncate_to_complete_jsonl(event_path)
                previous_index = -1
                for line_number, line in enumerate(
                    event_path.read_text(encoding="utf-8").splitlines(),
                    1,
                ):
                    try:
                        record = json.loads(line)
                    except json.JSONDecodeError as error:
                        raise ValueError(
                            f"invalid complete JSONL record {event_path}:"
                            f"{line_number}"
                        ) from error
                    if not isinstance(record, dict):
                        raise ValueError(
                            f"JSONL record is not an object: {event_path}:"
                            f"{line_number}"
                        )
                    try:
                        event_index = int(record["event_index"])
                    except (KeyError, TypeError, ValueError) as error:
                        raise ValueError(
                            f"JSONL record lacks event_index: {event_path}:"
                            f"{line_number}"
                        ) from error
                    if event_index <= previous_index:
                        raise ValueError(
                            f"JSONL event_index is not strictly increasing: "
                            f"{event_path}:{line_number}"
                        )
                    previous_index = event_index
                    records.append(record)

            first_event = records[0] if records else {}
            run_metadata = replay_manifest.get("run_metadata", {})
            if not isinstance(run_metadata, Mapping):
                run_metadata = {}
            csv_row = hardware_csv.get(hardware_run_id, {})
            simulation_run_id = str(
                hardware_manifest.get("simulation_run_id")
                or run_metadata.get("simulation_run_id")
                or first_event.get("simulation_run_id")
                or csv_row.get("simulation_run_id")
                or ""
            )
            if not simulation_run_id:
                copied_summary = hardware_dir / "sil_summary.json"
                if copied_summary.is_file():
                    simulation_run_id = str(
                        _read_json_object(copied_summary).get("run_id", "")
                    )
            if not simulation_run_id:
                raise ValueError(
                    f"hardware evidence lacks simulation linkage: {hardware_dir}"
                )
            cycle_hint = (
                hardware_manifest.get("cycle_index")
                or run_metadata.get("cycle_index")
                or first_event.get("cycle_index")
                or csv_row.get("cycle_index")
            )
            simulation = ensure_simulation(
                simulation_run_id,
                cycle_hint,
            )
            for key in ("mode", "run_seed", "cycle_index"):
                candidate = (
                    run_metadata.get(key)
                    if run_metadata.get(key) is not None
                    else first_event.get(key)
                )
                if candidate is not None:
                    simulation[key] = candidate

            simulation_dir = simulation_root / simulation_run_id
            simulation_dir.mkdir(parents=True, exist_ok=True)
            for name in SIMULATION_ARTIFACTS:
                destination = simulation_dir / name
                source = hardware_dir / name
                if not destination.is_file() and source.is_file():
                    shutil.copy2(source, destination)
            if all(
                (simulation_dir / name).is_file()
                for name in SIMULATION_ARTIFACTS
            ):
                simulation["status"] = "SIL_COMPLETE"
                simulation["resolved_config"] = _read_json_object(
                    simulation_dir / "resolved_config.json"
                )
                simulation["sil_row"] = _read_json_object(
                    simulation_dir / "sil_summary.json"
                )
                simulation["resolved_config_sha256"] = (
                    simulation["sil_row"].get("config_sha256")
                    or _sha256_json(simulation["resolved_config"])
                )
                simulation["profile_path"] = str(
                    (simulation_dir / "rocketpy_profile.csv").resolve()
                )
                simulation["sil_timeseries_path"] = str(
                    (simulation_dir / "sil_timeseries.csv.gz").resolve()
                )

            attempt_match = re.search(r"-a(\d+)$", hardware_run_id)
            attempt_index = int(
                hardware_manifest.get("attempt_index")
                or csv_row.get("attempt_index")
                or (
                    attempt_match.group(1)
                    if attempt_match is not None
                    else 1
                )
            )
            has_run_verdict_event = any(
                record.get("event") == "run_verdict"
                for record in records
            )
            sample_count = sum(
                record.get("event") == "sample_observed"
                for record in records
            )
            failure_type: str | None = None
            failure_message: str | None = None
            if verdict is None:
                status = "ABORTED_HOST_CRASH"
                failure_type = "InterruptedSession"
                failure_message = (
                    "host stopped before a final verdict; HOME state was not "
                    "inferred during rebuild"
                )
            else:
                verdict_pass = (
                    str(verdict.get("status", "")).upper() == "PASS"
                    and bool(verdict.get("passed"))
                )
                status = "PASS" if verdict_pass else "FAIL"
                if verdict_pass and (
                    not has_run_verdict_event or sample_count == 0
                ):
                    status = "FAIL"
                    failure_type = "IncompleteEvidence"
                    failure_message = (
                        "PASS verdict lacked a complete verdict event or "
                        "sample evidence during database rebuild"
                    )
                failure = verdict.get("failure")
                if isinstance(failure, Mapping):
                    failure_type = failure_type or str(
                        failure.get("type") or "ReplayFailure"
                    )
                    failure_message = failure_message or str(
                        failure.get("message") or "replay failed"
                    )
            failure_type = (
                failure_type
                or hardware_manifest.get("failure_type")
                or csv_row.get("failure_type")
                or None
            )
            failure_message = (
                failure_message
                or hardware_manifest.get("failure_message")
                or csv_row.get("failure_message")
                or None
            )
            metrics = (
                dict(verdict.get("metrics", {}))
                if isinstance(verdict, Mapping)
                and isinstance(verdict.get("metrics"), Mapping)
                else {}
            )
            full_status = metrics.get("full_status")
            if not isinstance(full_status, Mapping):
                full_status = {}
            home_status = metrics.get("home_status")
            if not isinstance(home_status, Mapping):
                home_status = {}
            started = str(
                hardware_manifest.get("started_utc")
                or replay_manifest.get("started_utc")
                or csv_row.get("started_utc")
                or _file_time_utc(hardware_dir, started_utc)
            )
            finished = (
                hardware_manifest.get("finished_utc")
                or replay_manifest.get("finished_utc")
                or csv_row.get("finished_utc")
                or (
                    _file_time_utc(
                        verdict_path if verdict_path.is_file() else event_path,
                        rebuilt_utc,
                    )
                    if status != "RUNNING"
                    else None
                )
            )
            for record in records:
                record.setdefault("session_id", session_id)
                record.setdefault("simulation_run_id", simulation_run_id)
                record.setdefault("hardware_run_id", hardware_run_id)
            hardware_records.append(
                {
                    "hardware_run_id": hardware_run_id,
                    "session_id": session_id,
                    "simulation_run_id": simulation_run_id,
                    "attempt_index": attempt_index,
                    "status": status,
                    "started_utc": started,
                    "finished_utc": finished,
                    "bundle_path": str(replay_dir.resolve()),
                    "verdict": verdict,
                    "failure_type": failure_type,
                    "failure_message": failure_message,
                    "open_time_s": metrics.get("open_time_s"),
                    "close_time_s": metrics.get("close_time_s"),
                    "max_tracking_error_steps": metrics.get(
                        "max_actuator_tracking_error_steps"
                    ),
                    "full_actual_steps": full_status.get("actual_steps"),
                    "home_actual_steps": home_status.get("actual_steps"),
                    "events": records,
                }
            )

        for hardware_run_id, csv_row in sorted(hardware_csv.items()):
            if hardware_run_id in discovered_hardware_ids:
                continue
            simulation_run_id = str(
                csv_row.get("simulation_run_id", "")
            ).strip()
            if not simulation_run_id:
                continue
            ensure_simulation(
                simulation_run_id,
                csv_row.get("cycle_index"),
            )
            attempt_match = re.search(r"-a(\d+)$", hardware_run_id)
            hardware_records.append(
                {
                    "hardware_run_id": hardware_run_id,
                    "session_id": session_id,
                    "simulation_run_id": simulation_run_id,
                    "attempt_index": int(
                        csv_row.get("attempt_index")
                        or (
                            attempt_match.group(1)
                            if attempt_match is not None
                            else 1
                        )
                    ),
                    "status": "FAIL",
                    "started_utc": (
                        csv_row.get("started_utc") or started_utc
                    ),
                    "finished_utc": (
                        csv_row.get("finished_utc") or rebuilt_utc
                    ),
                    "bundle_path": str(
                        (run_root / hardware_run_id / "replay").resolve()
                    ),
                    "verdict": None,
                    "failure_type": "IncompleteEvidence",
                    "failure_message": (
                        "portable CSV referenced a hardware run whose evidence "
                        "directory is missing"
                    ),
                    "open_time_s": _parse_csv_scalar(
                        str(csv_row.get("open_time_s", ""))
                    ),
                    "close_time_s": _parse_csv_scalar(
                        str(csv_row.get("close_time_s", ""))
                    ),
                    "max_tracking_error_steps": _parse_csv_scalar(
                        str(csv_row.get("max_tracking_error_steps", ""))
                    ),
                    "full_actual_steps": None,
                    "home_actual_steps": None,
                    "events": [],
                }
            )

        for simulation_run_id, value in simulations.items():
            value["cycle_index"] = int(value["cycle_index"])
            value["batch_index"] = int(value["batch_index"])
            value["batch_position"] = int(value["batch_position"])
            value["mode"] = str(value["mode"])
            if value["mode"] == "baseline":
                value["random_index"] = None
            elif value["random_index"] is not None:
                value["random_index"] = int(value["random_index"])
            value["run_seed"] = int(value["run_seed"])
            if value["status"] == "PLANNED" and not value["sampled_inputs"]:
                value["status"] = "ERROR"
                value["failure_reason"] = (
                    "planned case metadata was not recoverable from a "
                    "simulation manifest or parameters.csv"
                )
                limitations.append(
                    f"{simulation_run_id}: missing sampled plan inputs"
                )
            if value["status"] == "SIL_COMPLETE":
                required = (
                    value["resolved_config"],
                    value["sil_row"],
                    value["profile_path"],
                    value["sil_timeseries_path"],
                )
                if any(item is None for item in required):
                    value["status"] = "ERROR"
                    value["failure_reason"] = (
                        "SIL_COMPLETE metadata referenced incomplete artifacts"
                    )

        temporary_db = session_dir / (
            f"campaign.sqlite3.rebuild-{os.getpid()}-{uuid.uuid4().hex}.tmp"
        )
        connection = sqlite3.connect(
            temporary_db,
            timeout=10.0,
            isolation_level=None,
        )
        try:
            connection.row_factory = sqlite3.Row
            connection.execute("PRAGMA journal_mode=DELETE")
            connection.execute("PRAGMA synchronous=FULL")
            connection.execute("PRAGMA foreign_keys=ON")
            cls._create_schema(connection)
            connection.execute(
                """
                INSERT INTO sessions(
                    session_id, schema_version, status, started_utc,
                    finished_utc, updated_utc, master_seed, batch_size,
                    baseline_interval, dwell_s, stop_reason, settings_json
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    session_id,
                    SCHEMA_VERSION,
                    rebuilt_status,
                    started_utc,
                    session_document.get("finished_utc"),
                    rebuilt_utc,
                    master_seed,
                    batch_size,
                    baseline_interval,
                    dwell_s,
                    rebuilt_reason,
                    _json(dict(settings)),
                ),
            )
            for value in sorted(
                simulations.values(),
                key=lambda item: int(item["cycle_index"]),
            ):
                connection.execute(
                    """
                    INSERT INTO simulation_runs(
                        simulation_run_id, session_id, cycle_index,
                        random_index, batch_index, batch_position, mode,
                        run_seed, status, planned_utc, started_utc, finished_utc,
                        sampled_inputs_json, resolved_config_json,
                        resolved_config_sha256, sil_row_json, profile_path,
                        sil_timeseries_path, failure_reason
                    ) VALUES (
                        ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?
                    )
                    """,
                    (
                        value["simulation_run_id"],
                        session_id,
                        int(value["cycle_index"]),
                        (
                            int(value["random_index"])
                            if value["random_index"] is not None
                            else None
                        ),
                        int(value["batch_index"]),
                        int(value["batch_position"]),
                        str(value["mode"]),
                        int(value["run_seed"]),
                        str(value["status"]),
                        str(value["planned_utc"]),
                        value["started_utc"],
                        value["finished_utc"],
                        _json(dict(value["sampled_inputs"])),
                        (
                            _json(dict(value["resolved_config"]))
                            if isinstance(
                                value["resolved_config"],
                                Mapping,
                            )
                            else None
                        ),
                        value["resolved_config_sha256"],
                        (
                            _json(dict(value["sil_row"]))
                            if isinstance(value["sil_row"], Mapping)
                            else None
                        ),
                        value["profile_path"],
                        value["sil_timeseries_path"],
                        value["failure_reason"],
                    ),
                )
            for value in sorted(
                hardware_records,
                key=lambda item: (
                    simulations[item["simulation_run_id"]]["cycle_index"],
                    item["attempt_index"],
                ),
            ):
                connection.execute(
                    """
                    INSERT INTO hardware_runs(
                        hardware_run_id, session_id, simulation_run_id,
                        attempt_index, status, started_utc, finished_utc,
                        bundle_path, verdict_json, failure_type,
                        failure_message, open_time_s, close_time_s,
                        max_tracking_error_steps, full_actual_steps,
                        home_actual_steps
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        value["hardware_run_id"],
                        session_id,
                        value["simulation_run_id"],
                        int(value["attempt_index"]),
                        value["status"],
                        value["started_utc"],
                        value["finished_utc"],
                        value["bundle_path"],
                        (
                            _json(dict(value["verdict"]))
                            if isinstance(value["verdict"], Mapping)
                            else None
                        ),
                        value["failure_type"],
                        value["failure_message"],
                        value["open_time_s"],
                        value["close_time_s"],
                        value["max_tracking_error_steps"],
                        value["full_actual_steps"],
                        value["home_actual_steps"],
                    ),
                )
                for record in value["events"]:
                    cls._insert_event(
                        connection,
                        session_id,
                        record,
                        hardware_run_id=value["hardware_run_id"],
                    )
            foreign_key_errors = list(
                connection.execute("PRAGMA foreign_key_check")
            )
            if foreign_key_errors:
                raise sqlite3.IntegrityError(
                    f"rebuilt database has foreign-key errors: "
                    f"{foreign_key_errors}"
                )
            integrity = connection.execute(
                "PRAGMA integrity_check"
            ).fetchone()
            if integrity is None or str(integrity[0]).lower() != "ok":
                raise sqlite3.DatabaseError(
                    f"rebuilt database failed integrity_check: {integrity}"
                )
            counts = {
                "simulation_runs": len(simulations),
                "hardware_runs": len(hardware_records),
                "events": int(
                    connection.execute(
                        "SELECT COUNT(*) FROM events"
                    ).fetchone()[0]
                ),
                "samples": int(
                    connection.execute(
                        "SELECT COUNT(*) FROM samples"
                    ).fetchone()[0]
                ),
            }
        except BaseException:
            connection.close()
            temporary_db.unlink(missing_ok=True)
            raise
        else:
            connection.close()
        with temporary_db.open("rb+") as handle:
            os.fsync(handle.fileno())

        recovery_token = (
            datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            + "-"
            + uuid.uuid4().hex[:8]
        )
        recovery_dir = session_dir / "database_recovery" / recovery_token
        recovery_dir.mkdir(parents=True, exist_ok=False)
        shutil.copy2(session_path, recovery_dir / "session.before.json")
        database_path = session_dir / "campaign.sqlite3"
        archived: list[tuple[Path, Path]] = []
        for existing in (
            database_path,
            Path(str(database_path) + "-wal"),
            Path(str(database_path) + "-shm"),
        ):
            if existing.exists():
                destination = recovery_dir / existing.name
                os.replace(existing, destination)
                archived.append((existing, destination))
        try:
            os.replace(temporary_db, database_path)
        except BaseException:
            for original, archived_path in reversed(archived):
                if archived_path.exists():
                    os.replace(archived_path, original)
            temporary_db.unlink(missing_ok=True)
            raise

        store = cls(session_dir)
        try:
            for simulation_run_id in simulations:
                store._write_simulation_manifest(simulation_run_id)
            for value in hardware_records:
                store._write_hardware_manifest(value["hardware_run_id"])
                store.export_telemetry(value["hardware_run_id"])
            store.export_csvs()
            store._write_session_json()
            report = {
                "schema": "ambar.continuous_hil_database_rebuild.v1",
                "rebuilt_utc": rebuilt_utc,
                "session_id": session_id,
                "previous_session_status": previous_status,
                "rebuilt_session_status": rebuilt_status,
                "database": str(database_path),
                "archived_database_files": [
                    str(destination.relative_to(session_dir))
                    for _original, destination in archived
                ],
                "source_files": sorted(source_files),
                "counts": counts,
                "limitations": sorted(set(limitations)),
                "safety": {
                    "unfinalized_hardware_runs": "ABORTED_HOST_CRASH",
                    "home_inferred_after_restart": False,
                    "pass_requires_verdict_event_and_samples": True,
                },
            }
            _write_json_atomic(
                recovery_dir / "rebuild_report.json",
                report,
            )
            _write_json_atomic(
                session_dir / "last_database_rebuild.json",
                report,
            )
        except BaseException:
            store.close()
            raise
        return store

    @staticmethod
    def resolve_resume(
        value: str | Path,
        results_root: str | Path,
    ) -> Path:
        candidate = Path(value)
        if candidate.is_dir():
            return candidate.resolve()
        return (Path(results_root).resolve() / str(value)).resolve()

    @staticmethod
    def _configure_connection(connection: sqlite3.Connection) -> None:
        connection.execute("PRAGMA journal_mode=WAL")
        connection.execute("PRAGMA synchronous=FULL")
        connection.execute("PRAGMA foreign_keys=ON")
        connection.execute("PRAGMA busy_timeout=10000")

    def _configure(self) -> None:
        self._configure_connection(self.connection)

    @staticmethod
    def _create_schema(connection: sqlite3.Connection) -> None:
        connection.executescript(
            """
            CREATE TABLE sessions(
                session_id TEXT PRIMARY KEY,
                schema_version INTEGER NOT NULL,
                status TEXT NOT NULL,
                started_utc TEXT NOT NULL,
                finished_utc TEXT,
                updated_utc TEXT NOT NULL,
                master_seed INTEGER NOT NULL,
                batch_size INTEGER NOT NULL,
                baseline_interval INTEGER NOT NULL,
                dwell_s REAL NOT NULL,
                stop_reason TEXT,
                settings_json TEXT NOT NULL
            );

            CREATE TABLE simulation_runs(
                simulation_run_id TEXT PRIMARY KEY,
                session_id TEXT NOT NULL REFERENCES sessions(session_id),
                cycle_index INTEGER NOT NULL,
                random_index INTEGER,
                batch_index INTEGER NOT NULL,
                batch_position INTEGER NOT NULL,
                mode TEXT NOT NULL,
                run_seed INTEGER NOT NULL,
                status TEXT NOT NULL,
                planned_utc TEXT NOT NULL,
                started_utc TEXT,
                finished_utc TEXT,
                sampled_inputs_json TEXT NOT NULL,
                resolved_config_json TEXT,
                resolved_config_sha256 TEXT,
                sil_row_json TEXT,
                profile_path TEXT,
                sil_timeseries_path TEXT,
                failure_reason TEXT,
                UNIQUE(session_id, cycle_index)
            );

            CREATE TABLE hardware_runs(
                hardware_run_id TEXT PRIMARY KEY,
                session_id TEXT NOT NULL REFERENCES sessions(session_id),
                simulation_run_id TEXT NOT NULL
                    REFERENCES simulation_runs(simulation_run_id),
                attempt_index INTEGER NOT NULL,
                status TEXT NOT NULL,
                started_utc TEXT NOT NULL,
                finished_utc TEXT,
                bundle_path TEXT NOT NULL,
                verdict_json TEXT,
                failure_type TEXT,
                failure_message TEXT,
                open_time_s REAL,
                close_time_s REAL,
                max_tracking_error_steps INTEGER,
                full_actual_steps INTEGER,
                home_actual_steps INTEGER,
                UNIQUE(simulation_run_id, attempt_index)
            );

            CREATE TABLE events(
                session_id TEXT NOT NULL REFERENCES sessions(session_id),
                hardware_run_id TEXT NOT NULL REFERENCES hardware_runs(hardware_run_id),
                event_index INTEGER NOT NULL,
                event TEXT NOT NULL,
                host_elapsed_s REAL,
                record_json TEXT NOT NULL,
                PRIMARY KEY(hardware_run_id, event_index)
            );

            CREATE TABLE samples(
                hardware_run_id TEXT NOT NULL REFERENCES hardware_runs(hardware_run_id),
                event_index INTEGER NOT NULL,
                sample_index INTEGER,
                replay_time_s REAL,
                source_time_s REAL,
                truth_altitude_m REAL,
                truth_velocity_mps REAL,
                truth_acceleration_mps2 REAL,
                sil_estimated_altitude_m REAL,
                sil_estimated_velocity_mps REAL,
                sil_predicted_apogee_m REAL,
                sil_raw_controller_fraction REAL,
                sil_actual_deployment_fraction REAL,
                stm32_altitude_m REAL,
                stm32_velocity_mps REAL,
                stm32_predicted_apogee_m REAL,
                target_apogee_m REAL,
                raw_controller_deployment_fraction REAL,
                forced_hil_deployment_fraction REAL,
                hil_override_mode INTEGER,
                motor_target_steps INTEGER,
                motor_actual_steps INTEGER,
                motor_tracking_error_steps INTEGER,
                home_active INTEGER,
                full_active INTEGER,
                limits_plausible INTEGER,
                endpoint_sequence_verified INTEGER,
                stm32_phase INTEGER,
                actuator_machine_state INTEGER,
                actuator_flags INTEGER,
                actuator_inhibit_flags INTEGER,
                driver_status INTEGER,
                schedule_lag_s REAL,
                skipped_host_samples INTEGER,
                PRIMARY KEY(hardware_run_id, event_index)
            );

            CREATE INDEX simulation_runs_status_idx
                ON simulation_runs(session_id, status, cycle_index);
            CREATE INDEX hardware_runs_status_idx
                ON hardware_runs(session_id, status, started_utc);
            CREATE INDEX samples_time_idx
                ON samples(hardware_run_id, replay_time_s);
            """
        )

    @staticmethod
    def _ensure_compatible_schema(connection: sqlite3.Connection) -> None:
        """Apply additive schema updates without invalidating v1 sessions."""

        sample_columns = {
            str(row[1])
            for row in connection.execute('PRAGMA table_info("samples")')
        }
        if "sil_actual_deployment_fraction" not in sample_columns:
            connection.execute(
                "ALTER TABLE samples "
                "ADD COLUMN sil_actual_deployment_fraction REAL"
            )

    @property
    def session(self) -> sqlite3.Row:
        row = self.connection.execute("SELECT * FROM sessions LIMIT 1").fetchone()
        if row is None:
            raise RuntimeError("session database has no sessions row")
        return row

    @property
    def session_id(self) -> str:
        return str(self.session["session_id"])

    def close(self) -> None:
        self.connection.commit()
        self.connection.close()

    def check_free_space(self, minimum_bytes: int = MIN_FREE_BYTES) -> int:
        free = shutil.disk_usage(self.session_dir).free
        if free < minimum_bytes:
            raise OSError(
                f"only {free / (1024 ** 3):.2f} GiB free; "
                f"{minimum_bytes / (1024 ** 3):.2f} GiB is required"
            )
        return free

    def create_event_sink(
        self,
        *,
        minimum_free_bytes: int = MIN_FREE_BYTES,
        commit_interval_s: float = 0.25,
    ) -> "ContinuousHilEventSink":
        """Create one run-scoped SQLite sink for the evidence writer thread.

        The sink owns a separate connection and batches WAL commits at a
        bounded cadence. The supervisor's control connection therefore remains
        on its creating thread while replay events are persisted asynchronously.
        """

        return ContinuousHilEventSink(
            self.db_path,
            self.session_dir,
            self.session_id,
            minimum_free_bytes=minimum_free_bytes,
            commit_interval_s=commit_interval_s,
        )

    def _write_session_json(self) -> None:
        row = dict(self.session)
        row["settings"] = json.loads(row.pop("settings_json"))
        row["database"] = self.db_path.name
        _write_json_atomic(self.session_dir / "session.json", row)

    def _simulation_manifest_path(self, simulation_run_id: str) -> Path:
        return (
            self.session_dir
            / "simulation_runs"
            / simulation_run_id
            / SIMULATION_MANIFEST
        )

    def _write_simulation_manifest(self, simulation_run_id: str) -> None:
        row = self.connection.execute(
            "SELECT * FROM simulation_runs WHERE simulation_run_id=?",
            (simulation_run_id,),
        ).fetchone()
        if row is None:
            raise KeyError(simulation_run_id)
        value = dict(row)
        value["sampled_inputs"] = json.loads(
            value.pop("sampled_inputs_json")
        )
        value["resolved_config"] = (
            json.loads(value.pop("resolved_config_json"))
            if value.get("resolved_config_json")
            else None
        )
        value.pop("resolved_config_json", None)
        value["sil_row"] = (
            json.loads(value.pop("sil_row_json"))
            if value.get("sil_row_json")
            else None
        )
        value.pop("sil_row_json", None)
        _write_json_atomic(
            self._simulation_manifest_path(simulation_run_id),
            {
                "schema": "ambar.continuous_hil_simulation_run.v1",
                **value,
            },
        )

    @staticmethod
    def _hardware_manifest_path(bundle_path: str | Path) -> Path:
        return Path(bundle_path).resolve().parent / HARDWARE_MANIFEST

    def _write_hardware_manifest(self, hardware_run_id: str) -> None:
        row = self.connection.execute(
            "SELECT * FROM hardware_runs WHERE hardware_run_id=?",
            (hardware_run_id,),
        ).fetchone()
        if row is None:
            raise KeyError(hardware_run_id)
        value = dict(row)
        value["verdict"] = (
            json.loads(value.pop("verdict_json"))
            if value.get("verdict_json")
            else None
        )
        value.pop("verdict_json", None)
        _write_json_atomic(
            self._hardware_manifest_path(value["bundle_path"]),
            {
                "schema": "ambar.continuous_hil_hardware_run.v1",
                **value,
            },
        )

    def update_session_status(self, status: str, reason: str | None = None) -> None:
        now = utc_now()
        finished = now if status != "RUNNING" else None
        self.connection.execute(
            """
            UPDATE sessions
            SET status=?, stop_reason=?, updated_utc=?, finished_utc=?
            """,
            (status, reason, now, finished),
        )
        self._write_session_json()

    def add_simulation_plan(self, plan: Mapping[str, Any]) -> None:
        planned_utc = utc_now()
        self.connection.execute(
            """
            INSERT INTO simulation_runs(
                simulation_run_id, session_id, cycle_index, random_index,
                batch_index, batch_position, mode, run_seed, status,
                planned_utc, sampled_inputs_json
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, 'PLANNED', ?, ?)
            """,
            (
                plan["simulation_run_id"],
                self.session_id,
                plan["cycle_index"],
                plan.get("random_index"),
                plan["batch_index"],
                plan["batch_position"],
                plan["mode"],
                plan["run_seed"],
                planned_utc,
                _json(dict(plan.get("sampled_inputs", {}))),
            ),
        )
        self._write_simulation_manifest(str(plan["simulation_run_id"]))

    def plan_count(self) -> int:
        return int(
            self.connection.execute(
                "SELECT COUNT(*) FROM simulation_runs"
            ).fetchone()[0]
        )

    def next_unfinished(self) -> sqlite3.Row | None:
        return self.connection.execute(
            """
            SELECT sr.*
            FROM simulation_runs sr
            WHERE sr.session_id=?
              AND sr.status IN ('PLANNED', 'SIL_COMPLETE')
              AND NOT EXISTS (
                  SELECT 1 FROM hardware_runs hr
                  WHERE hr.simulation_run_id=sr.simulation_run_id
                    AND hr.status='PASS'
              )
            ORDER BY sr.cycle_index
            LIMIT 1
            """,
            (self.session_id,),
        ).fetchone()

    def begin_simulation(self, simulation_run_id: str) -> None:
        self.connection.execute(
            """
            UPDATE simulation_runs
            SET status='SIL_RUNNING', started_utc=?, failure_reason=NULL
            WHERE simulation_run_id=?
            """,
            (utc_now(), simulation_run_id),
        )
        self._write_simulation_manifest(simulation_run_id)

    def complete_simulation(
        self,
        simulation_run_id: str,
        *,
        resolved_config: Mapping[str, Any],
        resolved_config_sha256: str,
        sil_row: Mapping[str, Any],
        profile_path: str | Path,
        sil_timeseries_path: str | Path,
    ) -> None:
        self.connection.execute(
            """
            UPDATE simulation_runs
            SET status='SIL_COMPLETE', finished_utc=?,
                resolved_config_json=?, resolved_config_sha256=?,
                sil_row_json=?, profile_path=?, sil_timeseries_path=?,
                failure_reason=NULL
            WHERE simulation_run_id=?
            """,
            (
                utc_now(),
                _json(dict(resolved_config)),
                resolved_config_sha256,
                _json(dict(sil_row)),
                str(profile_path),
                str(sil_timeseries_path),
                simulation_run_id,
            ),
        )
        self._write_simulation_manifest(simulation_run_id)

    def fail_simulation(self, simulation_run_id: str, reason: str) -> None:
        self.connection.execute(
            """
            UPDATE simulation_runs
            SET status='ERROR', finished_utc=?, failure_reason=?
            WHERE simulation_run_id=?
            """,
            (utc_now(), reason, simulation_run_id),
        )
        self._write_simulation_manifest(simulation_run_id)

    def begin_hardware_run(
        self,
        simulation_run_id: str,
        hardware_run_id: str,
        bundle_path: str | Path,
    ) -> int:
        attempt = int(
            self.connection.execute(
                """
                SELECT COALESCE(MAX(attempt_index), 0) + 1
                FROM hardware_runs WHERE simulation_run_id=?
                """,
                (simulation_run_id,),
            ).fetchone()[0]
        )
        self.connection.execute(
            """
            INSERT INTO hardware_runs(
                hardware_run_id, session_id, simulation_run_id, attempt_index,
                status, started_utc, bundle_path
            ) VALUES (?, ?, ?, ?, 'RUNNING', ?, ?)
            """,
            (
                hardware_run_id,
                self.session_id,
                simulation_run_id,
                attempt,
                utc_now(),
                str(bundle_path),
            ),
        )
        self._write_hardware_manifest(hardware_run_id)
        return attempt

    @staticmethod
    def _insert_event(
        connection: sqlite3.Connection,
        session_id: str,
        record: Mapping[str, Any],
        *,
        hardware_run_id: str | None = None,
    ) -> None:
        resolved_hardware_run_id = (
            hardware_run_id
            if hardware_run_id is not None
            else str(record["hardware_run_id"])
        )
        event_index = int(record["event_index"])
        connection.execute(
            """
            INSERT OR REPLACE INTO events(
                session_id, hardware_run_id, event_index, event,
                host_elapsed_s, record_json
            ) VALUES (?, ?, ?, ?, ?, ?)
            """,
            (
                session_id,
                resolved_hardware_run_id,
                event_index,
                str(record["event"]),
                record.get("host_elapsed_s"),
                _json(dict(record)),
            ),
        )
        if record.get("event") == "sample_observed":
            sil = record.get("sil")
            if not isinstance(sil, Mapping):
                sil = {}
            connection.execute(
                """
                INSERT OR REPLACE INTO samples(
                    hardware_run_id, event_index, sample_index, replay_time_s,
                    source_time_s, truth_altitude_m, truth_velocity_mps,
                    truth_acceleration_mps2, sil_estimated_altitude_m,
                    sil_estimated_velocity_mps, sil_predicted_apogee_m,
                    sil_raw_controller_fraction,
                    sil_actual_deployment_fraction, stm32_altitude_m,
                    stm32_velocity_mps, stm32_predicted_apogee_m,
                    target_apogee_m, raw_controller_deployment_fraction,
                    forced_hil_deployment_fraction, hil_override_mode,
                    motor_target_steps, motor_actual_steps,
                    motor_tracking_error_steps, home_active, full_active,
                    limits_plausible, endpoint_sequence_verified, stm32_phase,
                    actuator_machine_state, actuator_flags,
                    actuator_inhibit_flags, driver_status, schedule_lag_s,
                    skipped_host_samples
                ) VALUES (
                    ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?,
                    ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?
                )
                """,
                (
                    resolved_hardware_run_id,
                    event_index,
                    record.get("sample_index"),
                    record.get("replay_time_s"),
                    record.get("source_time_s"),
                    record.get("truth_altitude_m"),
                    record.get("truth_velocity_mps"),
                    record.get("truth_acceleration_mps2"),
                    sil.get("estimated_altitude_m"),
                    sil.get("estimated_velocity_mps"),
                    sil.get("predicted_apogee_m"),
                    sil.get("command_fraction"),
                    sil.get("actual_deployment_fraction"),
                    record.get("stm32_altitude_m"),
                    record.get("stm32_velocity_mps"),
                    record.get("stm32_predicted_apogee_m"),
                    record.get("target_apogee_m"),
                    record.get("raw_controller_deployment_fraction"),
                    record.get("forced_hil_deployment_fraction"),
                    record.get("hil_override_mode"),
                    record.get("motor_target_steps"),
                    record.get("motor_actual_steps"),
                    record.get("motor_tracking_error_steps"),
                    int(bool(record.get("home_active"))),
                    int(bool(record.get("full_active"))),
                    int(bool(record.get("limits_plausible"))),
                    int(bool(record.get("endpoint_sequence_verified"))),
                    record.get("stm32_phase"),
                    record.get("actuator_machine_state"),
                    record.get("actuator_flags"),
                    record.get("actuator_inhibit_flags"),
                    record.get("driver_status"),
                    record.get("schedule_lag_s"),
                    record.get("skipped_host_samples"),
                ),
            )

    def record_event(self, record: Mapping[str, Any]) -> None:
        self._insert_event(
            self.connection,
            self.session_id,
            record,
        )
        now = time.monotonic()
        if now - self._last_event_commit_s >= 0.25:
            self.connection.commit()
            self._last_event_commit_s = now

    def finalize_hardware_run(
        self,
        hardware_run_id: str,
        *,
        status: str,
        verdict: Mapping[str, Any] | None,
        failure: BaseException | None = None,
    ) -> None:
        metrics = dict(verdict.get("metrics", {})) if verdict else {}
        full_status = metrics.get("full_status") or {}
        home_status = metrics.get("home_status") or {}
        self.connection.execute(
            """
            UPDATE hardware_runs
            SET status=?, finished_utc=?, verdict_json=?,
                failure_type=?, failure_message=?, open_time_s=?, close_time_s=?,
                max_tracking_error_steps=?, full_actual_steps=?,
                home_actual_steps=?
            WHERE hardware_run_id=?
            """,
            (
                status,
                utc_now(),
                _json(dict(verdict)) if verdict is not None else None,
                type(failure).__name__ if failure is not None else None,
                str(failure) if failure is not None else None,
                metrics.get("open_time_s"),
                metrics.get("close_time_s"),
                metrics.get("max_actuator_tracking_error_steps"),
                full_status.get("actual_steps"),
                home_status.get("actual_steps"),
                hardware_run_id,
            ),
        )
        self._write_hardware_manifest(hardware_run_id)
        self.connection.commit()
        self.export_telemetry(hardware_run_id)
        self.export_csvs()

    def export_telemetry(self, hardware_run_id: str) -> Path:
        row = self.connection.execute(
            "SELECT bundle_path FROM hardware_runs WHERE hardware_run_id=?",
            (hardware_run_id,),
        ).fetchone()
        if row is None:
            raise KeyError(hardware_run_id)
        destination = Path(row["bundle_path"]).parent / "telemetry.csv.gz"
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(destination.name + ".tmp")
        samples = self.connection.execute(
            """
            SELECT * FROM samples
            WHERE hardware_run_id=?
            ORDER BY event_index
            """,
            (hardware_run_id,),
        ).fetchall()
        fieldnames = [entry[1] for entry in self.connection.execute(
            "PRAGMA table_info(samples)"
        )]
        with gzip.open(temporary, "wt", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(dict(sample) for sample in samples)
        os.replace(temporary, destination)
        return destination

    def export_csvs(self) -> None:
        rows = self.connection.execute(
            """
            SELECT sr.cycle_index, sr.simulation_run_id, sr.mode, sr.run_seed,
                   sr.status AS simulation_status, sr.sil_row_json,
                   hr.hardware_run_id, hr.attempt_index,
                   hr.status AS hardware_status, hr.started_utc,
                   hr.finished_utc, hr.open_time_s, hr.close_time_s,
                   hr.max_tracking_error_steps, hr.failure_type,
                   hr.failure_message
            FROM simulation_runs sr
            LEFT JOIN hardware_runs hr
              ON hr.simulation_run_id=sr.simulation_run_id
            ORDER BY sr.cycle_index, hr.attempt_index
            """
        ).fetchall()
        run_records: list[dict[str, Any]] = []
        for row in rows:
            record = dict(row)
            sil_row_text = record.pop("sil_row_json")
            if sil_row_text:
                for key, value in json.loads(sil_row_text).items():
                    record[f"sil_{key}"] = value
            run_records.append(record)
        run_fields = sorted(
            {key for record in run_records for key in record},
            key=lambda key: (
                key
                not in (
                    "cycle_index",
                    "simulation_run_id",
                    "hardware_run_id",
                    "mode",
                    "run_seed",
                    "simulation_status",
                    "hardware_status",
                ),
                key,
            ),
        )
        _write_csv_atomic(
            self.session_dir / "runs.csv",
            run_fields,
            run_records,
        )

        plans = self.connection.execute(
            """
            SELECT cycle_index, simulation_run_id, mode, run_seed,
                   batch_index, batch_position, random_index,
                   sampled_inputs_json, resolved_config_sha256
            FROM simulation_runs ORDER BY cycle_index
            """
        ).fetchall()
        parameter_records: list[dict[str, Any]] = []
        for row in plans:
            record = dict(row)
            sampled = json.loads(record.pop("sampled_inputs_json"))
            record.update(sampled)
            parameter_records.append(record)
        parameter_fields = sorted(
            {key for record in parameter_records for key in record},
            key=lambda key: (
                key
                not in (
                    "cycle_index",
                    "simulation_run_id",
                    "mode",
                    "run_seed",
                    "batch_index",
                    "batch_position",
                    "random_index",
                ),
                key,
            ),
        )
        _write_csv_atomic(
            self.session_dir / "parameters.csv",
            parameter_fields,
            parameter_records,
        )

    @staticmethod
    def _truncate_to_complete_jsonl(path: Path) -> None:
        """Keep only newline-terminated, decodable JSON records.

        A crash can leave either a partial byte tail or a newline-terminated
        record whose write did not complete coherently. Both are outside the
        durable evidence boundary and are truncated to the last valid record.
        Structurally valid but semantically inconsistent records remain visible
        for the stricter rebuild checks to reject.
        """

        if not path.is_file():
            return
        with path.open("rb+") as handle:
            last_valid_end = 0
            while True:
                line = handle.readline()
                if not line:
                    break
                if not line.endswith(b"\n"):
                    break
                try:
                    json.loads(line.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    break
                last_valid_end = handle.tell()
            handle.seek(0, os.SEEK_END)
            if handle.tell() != last_valid_end:
                handle.seek(last_valid_end)
                handle.truncate()

    def recover_interrupted(self) -> list[str]:
        """Abort active attempts and rebuild their ordered evidence indexes."""

        recovered: list[str] = []
        active = self.connection.execute(
            "SELECT * FROM hardware_runs WHERE status='RUNNING'"
        ).fetchall()
        for hardware in active:
            hardware_run_id = str(hardware["hardware_run_id"])
            event_log = Path(hardware["bundle_path"]) / "events.jsonl"
            self._truncate_to_complete_jsonl(event_log)
            self.connection.execute(
                """
                DELETE FROM samples WHERE hardware_run_id=?;
                """,
                (hardware_run_id,),
            )
            self.connection.execute(
                "DELETE FROM events WHERE hardware_run_id=?",
                (hardware_run_id,),
            )
            if event_log.is_file():
                for line in event_log.read_text(encoding="utf-8").splitlines():
                    try:
                        record = json.loads(line)
                    except json.JSONDecodeError:
                        break
                    self.record_event(record)
            self.connection.execute(
                """
                UPDATE hardware_runs
                SET status='ABORTED_HOST_CRASH', finished_utc=?,
                    failure_type='InterruptedSession',
                    failure_message='host stopped before a final verdict'
                WHERE hardware_run_id=?
                """,
                (utc_now(), hardware_run_id),
            )
            self._write_hardware_manifest(hardware_run_id)
            recovered.append(hardware_run_id)

        self.connection.execute(
            """
            UPDATE simulation_runs
            SET status=CASE
                WHEN profile_path IS NOT NULL AND sil_timeseries_path IS NOT NULL
                    THEN 'SIL_COMPLETE'
                ELSE 'PLANNED'
            END,
            failure_reason=CASE
                WHEN profile_path IS NOT NULL AND sil_timeseries_path IS NOT NULL
                    THEN failure_reason
                ELSE NULL
            END
            WHERE status='SIL_RUNNING'
            """
        )
        self.connection.commit()
        for row in self.connection.execute(
            """
            SELECT simulation_run_id FROM simulation_runs
            WHERE status IN ('PLANNED', 'SIL_COMPLETE')
            """
        ):
            self._write_simulation_manifest(str(row["simulation_run_id"]))
        self.export_csvs()
        return recovered


class ContinuousHilEventSink:
    """Run-scoped, ordered SQLite sink used only by the evidence worker.

    Its connection is opened lazily on the worker thread. Events and samples
    share one transaction, committed every ``commit_interval_s`` and once more
    during observer drain. A failed insert or free-space check rolls back the
    active batch and propagates to the replay observer.
    """

    def __init__(
        self,
        db_path: str | Path,
        session_dir: str | Path,
        session_id: str,
        *,
        minimum_free_bytes: int,
        commit_interval_s: float,
    ) -> None:
        if minimum_free_bytes < 0:
            raise ValueError("minimum free bytes must be nonnegative")
        if commit_interval_s <= 0.0:
            raise ValueError("event commit interval must be positive")
        self.db_path = Path(db_path).resolve()
        self.session_dir = Path(session_dir).resolve()
        self.session_id = session_id
        self.minimum_free_bytes = minimum_free_bytes
        self.commit_interval_s = commit_interval_s
        self.connection: sqlite3.Connection | None = None
        self._last_commit_s = time.monotonic()
        self._last_space_check_s = float("-inf")
        self._closed = False
        self._failed = False

    def _connect(self) -> sqlite3.Connection:
        if self._closed:
            raise RuntimeError("continuous-HIL event sink is closed")
        if self.connection is None:
            connection = sqlite3.connect(
                self.db_path,
                timeout=10.0,
                isolation_level=None,
            )
            try:
                ContinuousHilStore._configure_connection(connection)
            except BaseException:
                connection.close()
                raise
            self.connection = connection
            self._last_commit_s = time.monotonic()
        return self.connection

    def _check_free_space(self) -> None:
        if self.minimum_free_bytes == 0:
            return
        free = shutil.disk_usage(self.session_dir).free
        if free < self.minimum_free_bytes:
            raise OSError(
                f"only {free / (1024 ** 3):.2f} GiB free; "
                f"{self.minimum_free_bytes / (1024 ** 3):.2f} GiB is required"
            )

    def __call__(self, record: Mapping[str, Any]) -> None:
        if self._failed:
            raise RuntimeError("continuous-HIL event sink previously failed")
        connection = self._connect()
        now = time.monotonic()
        try:
            if not connection.in_transaction:
                connection.execute("BEGIN")
            ContinuousHilStore._insert_event(
                connection,
                self.session_id,
                record,
            )
            if now - self._last_space_check_s >= self.commit_interval_s:
                self._check_free_space()
                self._last_space_check_s = now
            if now - self._last_commit_s >= self.commit_interval_s:
                connection.commit()
                self._last_commit_s = now
        except BaseException:
            self._failed = True
            if connection.in_transaction:
                connection.rollback()
            raise

    def flush(self) -> None:
        if self.connection is not None and self.connection.in_transaction:
            self.connection.commit()
            self._last_commit_s = time.monotonic()

    def close(self) -> None:
        if self._closed:
            return
        try:
            if not self._failed:
                self.flush()
            elif self.connection is not None and self.connection.in_transaction:
                self.connection.rollback()
        finally:
            if self.connection is not None:
                self.connection.close()
                self.connection = None
            self._closed = True
