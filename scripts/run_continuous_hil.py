"""Run indefinite RocketPy-to-STM32 continuous HIL sessions.

This process is the sole serial-port owner.  It generates deterministic rolling
Latin-hypercube batches, persists SIL evidence, replays one case through the
CONTINUOUS_HIL firmware, forces a separately labelled 0 -> 153600 -> 0
TMC5240 ramp-state stroke, and stops the complete session on the first safety
or evidence fault. XACTUAL is not encoder or endstop evidence.
"""

from __future__ import annotations

import argparse
import bisect
import copy
import csv
from datetime import datetime, timezone
import gzip
import hashlib
import json
import os
from pathlib import Path
import secrets
import shutil
import sqlite3
import sys
import time
from typing import Any, Mapping, Sequence

from continuous_hil_store import (
    ContinuousHilStore,
    MIN_FREE_BYTES,
    default_results_root,
)


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "sim" / "rocketpy"
USB_DIR = (
    ROOT
    / "firmware"
    / "stm32_airbrake_pcb"
    / "tools"
    / "usb_protocol"
)
DEFAULT_STUDY_CONFIG = MODEL_DIR / "monte_carlo_config.json"
DEFAULT_BRIDGE = ROOT / "build" / "ambar_stm32_controller_bridge.exe"
DEFAULT_MIRROR_ROOT = (
    Path.home()
    / "OneDrive"
    / "Desktop"
    / "AMBAR_Continuous_Test_Results"
    / "latest"
)


def derive_batch_seed(master_seed: int, batch_index: int) -> int:
    """Derive an order-independent seed for one rolling LHS batch."""

    payload = f"AMBAR-CONTINUOUS-HIL:{master_seed}:{batch_index}".encode(
        "ascii"
    )
    return int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")


def interleave_run_kinds(
    batch_size: int,
    baseline_interval: int,
) -> list[str]:
    """Place one baseline after each configured number of randomized cases."""

    if batch_size <= 0:
        raise ValueError("batch size must be positive")
    if baseline_interval <= 0:
        raise ValueError("baseline interval must be positive")
    kinds: list[str] = []
    for random_offset in range(batch_size):
        kinds.append("monte_carlo")
        if (random_offset + 1) % baseline_interval == 0:
            kinds.append("baseline")
    return kinds


def _session_id_now() -> str:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{timestamp}-{secrets.token_hex(3)}"


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _write_timeseries_gzip(
    path: Path,
    rows: Sequence[Mapping[str, Any]],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    fieldnames = sorted({key for row in rows for key in row})
    with gzip.open(temporary, "wt", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=fieldnames,
            extrasaction="ignore",
        )
        writer.writeheader()
        writer.writerows(rows)
    os.replace(temporary, path)


def _load_runtime_modules():
    """Import RocketPy/serial-dependent project modules only for real runs."""

    for directory in (MODEL_DIR, USB_DIR):
        text = str(directory)
        if text not in sys.path:
            sys.path.insert(0, text)
    import run_monte_carlo as monte_carlo
    import run_rocketpy_sim as rocketpy_sim
    import replay_openrocket as replay

    return monte_carlo, rocketpy_sim, replay


def build_sample_metadata(
    profile,
    controller_log: Sequence[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    """Align the full SIL/controller trace to every 50 Hz replay sample."""

    if not controller_log:
        raise ValueError("controller log is empty")
    times = [float(row["time_s"]) for row in controller_log]
    selected: list[dict[str, Any]] = []
    keys = (
        "time_s",
        "measurement_source_time_s",
        "estimated_altitude_m",
        "estimated_velocity_mps",
        "predicted_apogee_m",
        "command_fraction",
        "desired_deployment_fraction",
        "delayed_desired_deployment_fraction",
        "actual_deployment_fraction",
        "phase",
        "health",
        "inhibit_flags",
    )
    for sample in profile.samples:
        index = bisect.bisect_left(times, float(sample.source_time_s))
        if index >= len(times):
            index = len(times) - 1
        elif index > 0 and (
            abs(times[index - 1] - float(sample.source_time_s))
            <= abs(times[index] - float(sample.source_time_s))
        ):
            index -= 1
        row = controller_log[index]
        selected.append({key: row.get(key) for key in keys if key in row})
    return selected


def _batch_index(store: ContinuousHilStore) -> int:
    value = store.connection.execute(
        "SELECT COALESCE(MAX(batch_index), -1) + 1 FROM simulation_runs"
    ).fetchone()[0]
    return int(value)


def _random_count(store: ContinuousHilStore) -> int:
    return int(
        store.connection.execute(
            "SELECT COUNT(*) FROM simulation_runs WHERE mode='monte_carlo'"
        ).fetchone()[0]
    )


def plan_next_batch(
    store: ContinuousHilStore,
    *,
    monte_carlo,
    base_config: Mapping[str, Any],
    study_config: Mapping[str, Any],
) -> int:
    """Persist a complete deterministic rolling batch before executing it."""

    session = store.session
    master_seed = int(session["master_seed"])
    batch_size = int(session["batch_size"])
    baseline_interval = int(session["baseline_interval"])
    batch_index = _batch_index(store)
    first_cycle = store.plan_count()
    first_random = _random_count(store)
    random_rows = monte_carlo.build_latin_hypercube(
        study_config["parameters"],
        batch_size,
        derive_batch_seed(master_seed, batch_index),
    )
    baseline = monte_carlo.baseline_inputs(
        dict(base_config),
        study_config["parameters"],
    )
    random_offset = 0
    for batch_position, kind in enumerate(
        interleave_run_kinds(batch_size, baseline_interval)
    ):
        cycle_index = first_cycle + batch_position
        if kind == "monte_carlo":
            sampled_inputs = random_rows[random_offset]
            random_index: int | None = first_random + random_offset
            run_seed = monte_carlo.derive_run_seed(
                master_seed,
                random_index,
            )
            random_offset += 1
        else:
            sampled_inputs = baseline
            random_index = None
            # Every checkpoint baseline is the same fully seeded case.
            run_seed = monte_carlo.derive_run_seed(
                master_seed,
                0x7FFFFFFF,
            )
        store.add_simulation_plan(
            {
                "simulation_run_id": (
                    f"{store.session_id}-sim-{cycle_index:08d}"
                ),
                "cycle_index": cycle_index,
                "random_index": random_index,
                "batch_index": batch_index,
                "batch_position": batch_position,
                "mode": kind,
                "run_seed": run_seed,
                "sampled_inputs": sampled_inputs,
            }
        )
    store.export_csvs()
    return len(interleave_run_kinds(batch_size, baseline_interval))


def _resolved_config(plan, *, monte_carlo, base_config, parameters):
    sampled = json.loads(plan["sampled_inputs_json"])
    return monte_carlo.apply_sampled_inputs(
        dict(base_config),
        parameters,
        sampled,
        int(plan["run_seed"]),
    )


def _hardware_attempt_index(
    store: ContinuousHilStore,
    simulation_run_id: str,
) -> int:
    return int(
        store.connection.execute(
            """
            SELECT COALESCE(MAX(attempt_index), 0) + 1
            FROM hardware_runs WHERE simulation_run_id=?
            """,
            (simulation_run_id,),
        ).fetchone()[0]
    )


def _copy_simulation_artifacts(source: Path, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    for name in (
        "hardware_preflight.json",
        "resolved_config.json",
        "rocketpy_profile.csv",
        "sil_timeseries.csv.gz",
        "sil_summary.json",
    ):
        shutil.copy2(source / name, destination / name)


def _mirror_session_portables(
    store: ContinuousHilStore,
    mirror_root: Path,
) -> None:
    """Mirror terminal/session exports without copying the live WAL database."""

    mirror_root.mkdir(parents=True, exist_ok=True)
    session_path = mirror_root / "session.json"
    mirrored_session_id: str | None = None
    if session_path.is_file():
        try:
            mirrored_session_id = str(
                json.loads(session_path.read_text(encoding="utf-8")).get(
                    "session_id",
                    "",
                )
            )
        except (OSError, json.JSONDecodeError, AttributeError):
            mirrored_session_id = None
    if mirrored_session_id != store.session_id:
        for managed_directory in ("runs", "reports"):
            managed_path = mirror_root / managed_directory
            if managed_path.is_dir():
                shutil.rmtree(managed_path)
        for managed_file in ("session.json", "runs.csv", "parameters.csv"):
            managed_path = mirror_root / managed_file
            if managed_path.is_file():
                managed_path.unlink()

    for name in ("session.json", "runs.csv", "parameters.csv"):
        source = store.session_dir / name
        if source.is_file():
            shutil.copy2(source, mirror_root / name)


def _mirror_finalized(
    store: ContinuousHilStore,
    hardware_dir: Path,
    mirror_root: Path,
) -> None:
    """Mirror one finalized PASS/FAIL run and the current portable indexes."""

    _mirror_session_portables(store, mirror_root)
    mirrored_run = mirror_root / "runs" / hardware_dir.name
    shutil.copytree(hardware_dir, mirrored_run, dirs_exist_ok=True)


def _finalize_hardware_attempt(
    store: ContinuousHilStore,
    *,
    hardware_run_id: str,
    hardware_dir: Path,
    mirror_root: Path,
    status: str,
    verdict: Mapping[str, Any] | None,
    failure: BaseException | None = None,
) -> None:
    """Finalize and mirror PASS or FAIL evidence through the same path."""

    store.finalize_hardware_run(
        hardware_run_id,
        status=status,
        verdict=verdict,
        failure=failure,
    )
    _mirror_finalized(store, hardware_dir, mirror_root)


def _run_one_cycle(
    store: ContinuousHilStore,
    plan,
    args: argparse.Namespace,
    *,
    monte_carlo,
    replay,
    base_config: Mapping[str, Any],
    study_config: Mapping[str, Any],
    controller_hash: str,
    validated_port: str,
    hardware_preflight: Mapping[str, Any],
    minimum_free_bytes: int,
) -> str:
    simulation_run_id = str(plan["simulation_run_id"])
    sim_dir = store.session_dir / "simulation_runs" / simulation_run_id
    sim_dir.mkdir(parents=True, exist_ok=True)
    _write_json(sim_dir / "hardware_preflight.json", hardware_preflight)

    if str(plan["status"]) == "PLANNED":
        store.begin_simulation(simulation_run_id)
        sampled_inputs = json.loads(plan["sampled_inputs_json"])
        config = _resolved_config(
            plan,
            monte_carlo=monte_carlo,
            base_config=base_config,
            parameters=study_config["parameters"],
        )
        artifact = monte_carlo.run_trial(
            campaign_id=store.session_id,
            run_id=simulation_run_id,
            run_index=int(plan["cycle_index"]),
            sample_index=(
                int(plan["random_index"])
                if plan["random_index"] is not None
                else -1
            ),
            mode=str(plan["mode"]),
            master_seed=int(store.session["master_seed"]),
            run_seed=int(plan["run_seed"]),
            config=config,
            sampled_inputs=sampled_inputs,
            bridge_path=args.bridge.resolve(),
            controller_hash=controller_hash,
            acceptance=study_config["acceptance"],
        )
        if artifact.row.get("status") == "ERROR" or not artifact.controller_log:
            reason = str(
                artifact.row.get("failure_reasons")
                or "RocketPy/SIL produced incomplete evidence"
            )
            store.fail_simulation(simulation_run_id, reason)
            raise RuntimeError(reason)

        resolved_path = sim_dir / "resolved_config.json"
        summary_path = sim_dir / "sil_summary.json"
        profile_path = sim_dir / "rocketpy_profile.csv"
        timeseries_path = sim_dir / "sil_timeseries.csv.gz"
        _write_json(resolved_path, artifact.resolved_config)
        _write_json(summary_path, artifact.row)
        _write_timeseries_gzip(timeseries_path, artifact.controller_log)
        monte_carlo.write_openrocket_profile(profile_path, artifact)
        store.complete_simulation(
            simulation_run_id,
            resolved_config=artifact.resolved_config,
            resolved_config_sha256=str(artifact.row["config_sha256"]),
            sil_row=artifact.row,
            profile_path=profile_path,
            sil_timeseries_path=timeseries_path,
        )
        controller_log = artifact.controller_log
        config = artifact.resolved_config
    else:
        config = json.loads(
            (sim_dir / "resolved_config.json").read_text(encoding="utf-8")
        )
        with gzip.open(
            sim_dir / "sil_timeseries.csv.gz",
            "rt",
            encoding="utf-8",
            newline="",
        ) as handle:
            controller_log = list(csv.DictReader(handle))

    dataset = replay.load_openrocket_csv(sim_dir / "rocketpy_profile.csv")
    profile = replay.build_replay_profile(
        dataset,
        rate_hz=50.0,
        prepad_s=1.0,
        full_flight=True,
    )
    metadata = build_sample_metadata(profile, controller_log)

    attempt = _hardware_attempt_index(store, simulation_run_id)
    hardware_run_id = (
        f"{store.session_id}-hw-{int(plan['cycle_index']):08d}-a{attempt:02d}"
    )
    hardware_dir = store.session_dir / "runs" / hardware_run_id
    _copy_simulation_artifacts(sim_dir, hardware_dir)
    replay_dir = hardware_dir / "replay"
    store.begin_hardware_run(
        simulation_run_id,
        hardware_run_id,
        replay_dir,
    )

    verdict: Mapping[str, Any] | None = None
    event_sink = store.create_event_sink(
        minimum_free_bytes=minimum_free_bytes,
        commit_interval_s=0.25,
    )

    try:
        verdict = replay.run_live_replay(
            profile,
            port_name=validated_port,
            barometer_stddev_m=float(
                config["sensor_model"]["barometer_measurement_std_dev_m"]
            ),
            arm_after_s=0.5,
            target_apogee_m=(
                float(config["requirements"]["target_apogee_ft"]) * 0.3048
            ),
            no_arm=False,
            rotations=3.0,
            full_steps_per_revolution=200,
            microsteps=256,
            gear_ratio=1.0,
            allow_actuator_motion=True,
            home_at_current_position=True,
            run_bundle_dir=replay_dir,
            gui_udp_port=args.gui_udp_port,
            run_metadata={
                "session_id": store.session_id,
                "simulation_run_id": simulation_run_id,
                "hardware_run_id": hardware_run_id,
                "cycle_index": int(plan["cycle_index"]),
                "mode": str(plan["mode"]),
                "run_seed": int(plan["run_seed"]),
                "hardware_preflight_artifact": "../hardware_preflight.json",
            },
            continuous_hil=True,
            burn_time_s=float(config["motor"]["burn_time_s"]),
            post_burn_margin_s=float(
                config["airbrakes"]["post_burn_enable_margin_s"]
            ),
            full_hold_s=8.0,
            endpoint_timeout_s=8.0,
            sample_metadata=metadata,
            event_sink=event_sink,
            event_log_name="events.jsonl",
        )
        _finalize_hardware_attempt(
            store,
            hardware_run_id=hardware_run_id,
            hardware_dir=hardware_dir,
            mirror_root=args.mirror_root.resolve(),
            status="PASS",
            verdict=verdict,
        )
    except BaseException as error:
        verdict_path = replay_dir / "verdict.json"
        if verdict_path.is_file():
            try:
                verdict = json.loads(verdict_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                verdict = None
        _finalize_hardware_attempt(
            store,
            hardware_run_id=hardware_run_id,
            hardware_dir=hardware_dir,
            mirror_root=args.mirror_root.resolve(),
            status="FAIL",
            verdict=verdict,
            failure=error,
        )
        raise

    return hardware_run_id


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run continuous RocketPy replay with a forced 3-rotation HIL stroke"
    )
    parser.add_argument(
        "--port",
        "--serial-port",
        dest="port",
        help="COM port; omit to auto-detect the AMBAR 0483:5740 USB device",
    )
    parser.add_argument("--results-root", type=Path, default=default_results_root())
    parser.add_argument("--session-id")
    parser.add_argument(
        "--resume",
        help="resume by session directory or ID (explicit operator action)",
    )
    parser.add_argument(
        "--rebuild-database",
        action="store_true",
        help=(
            "with --resume, explicitly rebuild a missing/corrupt SQLite "
            "database from durable session and per-run evidence"
        ),
    )
    parser.add_argument("--master-seed", type=int)
    parser.add_argument("--batch-size", type=int, default=50)
    parser.add_argument("--baseline-interval", type=int, default=10)
    parser.add_argument("--dwell-s", type=float, default=30.0)
    parser.add_argument("--gui-udp-port", type=int, default=52100)
    parser.add_argument(
        "--max-cycles",
        type=int,
        default=0,
        help="0 runs indefinitely; positive values are for qualification/debug",
    )
    parser.add_argument("--bridge", type=Path, default=DEFAULT_BRIDGE)
    parser.add_argument("--study-config", type=Path, default=DEFAULT_STUDY_CONFIG)
    parser.add_argument("--base-overrides", type=Path)
    parser.add_argument("--mirror-root", type=Path, default=DEFAULT_MIRROR_ROOT)
    parser.add_argument(
        "--min-free-gb",
        type=float,
        default=5.0,
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--accept-current-position-home",
        action="store_true",
        help=(
            "confirm the mechanism is manually fully closed and authorize one "
            "software HOME declaration (XACTUAL=0) for this process"
        ),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    if args.resume and args.session_id:
        resume_leaf = Path(args.resume).name
        if resume_leaf != args.session_id:
            raise SystemExit(
                "--session-id must match the resumed session directory"
            )
    if args.rebuild_database and not args.resume:
        raise SystemExit("--rebuild-database requires --resume")
    if not args.accept_current_position_home:
        raise SystemExit(
            "continuous HIL requires --accept-current-position-home after the "
            "operator manually places the mechanism fully closed; a restart "
            "or resume requires a fresh acknowledgement"
        )
    if args.batch_size <= 0 or args.baseline_interval <= 0:
        raise SystemExit("batch size and baseline interval must be positive")
    if args.dwell_s < 0.0:
        raise SystemExit("dwell time must be nonnegative")
    if args.max_cycles < 0:
        raise SystemExit("max cycles must be nonnegative")
    if not 1 <= args.gui_udp_port <= 65535:
        raise SystemExit("GUI UDP port must be within 1..65535")
    if not args.bridge.resolve().is_file():
        raise SystemExit(
            f"production STM32 controller bridge is missing: {args.bridge.resolve()}"
        )
    if not args.study_config.resolve().is_file():
        raise SystemExit(f"study config is missing: {args.study_config.resolve()}")

    monte_carlo, rocketpy_sim, replay = _load_runtime_modules()
    study_config = monte_carlo.load_study_config(args.study_config.resolve())
    base_config = rocketpy_sim.load_config(
        args.base_overrides.resolve() if args.base_overrides else None
    )
    controller_hash = monte_carlo.combined_controller_source_hash()
    minimum_free = int(args.min_free_gb * 1024 ** 3)
    if minimum_free < 0:
        raise SystemExit("minimum free space must be nonnegative")

    if args.resume:
        session_dir = ContinuousHilStore.resolve_resume(
            args.resume,
            args.results_root,
        )
        if args.rebuild_database:
            store = ContinuousHilStore.rebuild_database(session_dir)
            print(
                "Rebuilt campaign.sqlite3 from durable evidence; "
                "existing database files were archived."
            )
        else:
            try:
                store = ContinuousHilStore(session_dir)
            except (FileNotFoundError, sqlite3.DatabaseError) as error:
                raise SystemExit(
                    f"session database cannot be opened: {error}. "
                    "Review the evidence, then rerun the explicit resume with "
                    "--rebuild-database."
                ) from error
        store.recover_interrupted()
        if (
            args.master_seed is not None
            and args.master_seed != int(store.session["master_seed"])
        ):
            raise SystemExit("--master-seed does not match the resumed session")
        store.update_session_status("RUNNING", "explicit operator resume")
    else:
        master_seed = (
            args.master_seed
            if args.master_seed is not None
            else secrets.randbits(63)
        )
        session_id = args.session_id or _session_id_now()
        store = ContinuousHilStore.create(
            args.results_root,
            session_id,
            master_seed=master_seed,
            batch_size=args.batch_size,
            baseline_interval=args.baseline_interval,
            dwell_s=args.dwell_s,
            settings={
                "serial_port": args.port,
                "results_root": str(args.results_root.resolve()),
                "bridge": str(args.bridge.resolve()),
                "study_config": str(args.study_config.resolve()),
                "gui_udp_port": args.gui_udp_port,
                "mirror_root": str(args.mirror_root.resolve()),
                "minimum_free_bytes": minimum_free or MIN_FREE_BYTES,
                "operator_confirmed_manually_fully_closed": True,
                "software_home_declaration_required_each_process_start": True,
                "position_evidence_boundary": (
                    "TMC target/XACTUAL only; no encoder or endstop proof"
                ),
            },
        )

    _mirror_session_portables(store, args.mirror_root.resolve())
    completed = 0
    try:
        print(f"AMBAR continuous-HIL session: {store.session_id}")
        print(f"Results: {store.session_dir}")
        store.check_free_space(minimum_free or MIN_FREE_BYTES)
        hardware_preflight = replay.run_continuous_hil_preflight(
            port_name=args.port,
            accept_current_position_home=True,
        )
        validated_port = str(hardware_preflight["selected_port"])
        print(
            "Software HOME declared once from the operator-confirmed fully "
            "closed position. Later cycles will verify, never re-zero."
        )
        while args.max_cycles == 0 or completed < args.max_cycles:
            store.check_free_space(minimum_free or MIN_FREE_BYTES)
            plan = store.next_unfinished()
            if plan is None:
                plan_next_batch(
                    store,
                    monte_carlo=monte_carlo,
                    base_config=base_config,
                    study_config=study_config,
                )
                plan = store.next_unfinished()
            if plan is None:
                raise RuntimeError("batch planning produced no runnable case")

            store.check_free_space(minimum_free or MIN_FREE_BYTES)
            hardware_run_id = _run_one_cycle(
                store,
                plan,
                args,
                monte_carlo=monte_carlo,
                replay=replay,
                base_config=base_config,
                study_config=study_config,
                controller_hash=controller_hash,
                validated_port=validated_port,
                hardware_preflight=hardware_preflight,
                minimum_free_bytes=minimum_free or MIN_FREE_BYTES,
            )
            completed += 1
            print(
                f"Completed {hardware_run_id}; dwelling "
                f"{float(store.session['dwell_s']):.1f} s fully retracted."
            )
            dwell_deadline = time.monotonic() + float(store.session["dwell_s"])
            while time.monotonic() < dwell_deadline:
                store.check_free_space(minimum_free or MIN_FREE_BYTES)
                time.sleep(min(1.0, dwell_deadline - time.monotonic()))

        store.update_session_status(
            "COMPLETE",
            f"bounded max-cycles limit reached ({args.max_cycles})",
        )
        _mirror_session_portables(store, args.mirror_root.resolve())
        return 0
    except KeyboardInterrupt:
        store.update_session_status("STOPPED", "operator interrupt")
        _mirror_session_portables(store, args.mirror_root.resolve())
        print("Continuous HIL stopped by operator; firmware cleanup removed motor energy.")
        return 130
    except BaseException as error:
        try:
            store.update_session_status(
                "FAULTED",
                f"{type(error).__name__}: {error}",
            )
            _mirror_session_portables(store, args.mirror_root.resolve())
        finally:
            print(f"CONTINUOUS HIL FAULT: {type(error).__name__}: {error}")
        return 1
    finally:
        store.close()


if __name__ == "__main__":
    raise SystemExit(main())
