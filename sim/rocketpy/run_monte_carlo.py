"""Run reproducible AMBAR baseline and Monte Carlo RocketPy campaigns.

Overall purpose
---------------
The single-run RocketPy adapter is the closed-loop physics workbench.  This
module turns it into a repeatable campaign:

1. repeat the unchanged baseline to prove deterministic execution;
2. build all randomized cases up front with seeded Latin-hypercube sampling;
3. run a paired passive and STM32-controlled flight for every case;
4. write one CSV row per attempted run, including failures;
5. retain representative time histories for later USB/HIL replay.

The controller is not a reimplementation.  ``--bridge`` must point to the host
executable built from the production ``ambar_ekf.c`` and ``ambar_flight.c``
sources.  RocketPy owns truth physics, provisional sensors, and the virtual
actuator.  The sampled plant values are deliberately kept separate from the
controller's fixed predictor defaults so the study does not give the firmware
perfect knowledge of the randomized vehicle.

Interpretation boundary
-----------------------
The default ranges are screening assumptions, not measured probability
distributions.  A 50-run result is useful for finding sensitivity and unsafe
software behavior; it is not a certified reliability or flight-readiness
probability.  The campaign never opens a serial port or moves the physical
motor.  Representative profiles can be replayed through the STM32 later with
motion inhibited, followed by a small number of supervised mechanism tests.
"""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import importlib.metadata
import json
import math
import os
import platform
import random
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Sequence

from run_rocketpy_sim import (
    FEET_TO_METERS,
    METERS_TO_FEET,
    derive_phase_transitions,
    flight_apogee_agl_m,
    load_config,
    run_closed_loop,
    run_passive,
    validate_controller_log,
)


ROOT = Path(__file__).resolve().parents[2]
MODEL_DIR = Path(__file__).resolve().parent
DEFAULT_STUDY_CONFIG = MODEL_DIR / "monte_carlo_config.json"
DEFAULT_OUTPUT_ROOT = ROOT / "build" / "monte-carlo"
STM32_FLIGHT_SOURCE = (
    ROOT / "firmware" / "stm32_airbrake_pcb" / "Core" / "Src" / "ambar_flight.c"
)
STM32_EKF_SOURCE = (
    ROOT / "firmware" / "stm32_airbrake_pcb" / "Core" / "Src" / "ambar_ekf.c"
)
STM32_FLIGHT_HEADER = (
    ROOT / "firmware" / "stm32_airbrake_pcb" / "Core" / "Inc" / "ambar_flight.h"
)
STM32_EKF_HEADER = (
    ROOT / "firmware" / "stm32_airbrake_pcb" / "Core" / "Inc" / "ambar_ekf.h"
)
STM32_FEATURES_HEADER = (
    ROOT / "firmware" / "stm32_airbrake_pcb" / "Core" / "Inc" / "ambar_features.h"
)
STM32_BRIDGE_SOURCE = ROOT / "sim" / "stm32_controller_bridge.c"
ROCKETPY_RUNNER_SOURCE = MODEL_DIR / "run_rocketpy_sim.py"
SIMULATION_REQUIREMENTS = ROOT / "requirements-simulation.txt"


# Compressed phase transitions may skip a one-update Coast state when the
# production controller enters AirbrakeActive immediately after burnout.  Every
# legal route is still monotonic; Recovery and Fault remain terminal.
LEGAL_PHASE_TRANSITIONS: dict[str, set[str]] = {
    "PadIdle": {"Boost", "Fault"},
    "Boost": {"Coast", "AirbrakeActive", "Fault"},
    "Coast": {"AirbrakeActive", "Recovery", "Fault"},
    "AirbrakeActive": {"Recovery", "Fault"},
    "Recovery": set(),
    "Fault": set(),
}


BASE_RUN_FIELDS = [
    "campaign_id",
    "run_id",
    "run_index",
    "sample_index",
    "mode",
    "master_seed",
    "run_seed",
    "status",
    "failure_reasons",
    "runtime_s",
    "config_sha256",
    "controller_source_sha256",
    "controller_log_sha256",
    "target_apogee_ft",
    "target_tolerance_ft",
    "equivalent_full_airbrake_cda_m2",
    "passive_apogee_ft",
    "controlled_apogee_ft",
    "target_error_ft",
    "absolute_target_error_ft",
    "apogee_reduction_ft",
    "passive_reference_difference_percent",
    "maximum_mach",
    "minimum_rail_exit_velocity_fps",
    "first_command_time_s",
    "first_deployment_time_s",
    "minimum_allowed_command_time_s",
    "peak_command_fraction",
    "peak_deployment_fraction",
    "final_deployment_fraction",
    "maximum_altitude_error_m",
    "altitude_rmse_m",
    "maximum_velocity_error_mps",
    "velocity_rmse_mps",
    "prediction_rmse_m",
    "barometer_sample_count",
    "controller_sample_count",
    "early_command_samples",
    "descending_command_samples",
    "excessive_descent_command_samples",
    "maximum_commanded_descent_velocity_mps",
    "maximum_deployment_below_recovery_velocity_fraction",
    "deployment_at_recovery_fraction",
    "retraction_time_after_recovery_s",
    "unhealthy_samples",
    "phase_sequence",
    "illegal_phase_transitions",
    "log_integrity_error_count",
    "passive_reference_pass",
    "commands_bounded",
    "deployment_after_burn",
    "command_phases_valid",
    "recovery_observed",
    "retraction_observed",
    "timely_retraction",
    "controller_healthy",
    "effectiveness_pass",
    "target_band_pass",
    "flight_envelope_pass",
    "safety_pass",
    "performance_pass",
    "run_pass",
]


@dataclass
class TrialArtifact:
    """One completed or failed campaign case plus its optional detailed log."""

    row: dict[str, Any]
    sampled_inputs: dict[str, float]
    resolved_config: dict[str, Any]
    controller_log: list[dict[str, Any]]


def canonical_json_bytes(value: Any) -> bytes:
    """Serialize a value deterministically for evidence hashes."""

    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    """Return an uppercase SHA-256 digest used in manifests and CSV rows."""

    return hashlib.sha256(value).hexdigest().upper()


def sha256_file(path: Path) -> str:
    """Hash one source artifact without loading unrelated repository files."""

    return sha256_bytes(path.read_bytes())


def combined_controller_source_hash() -> str:
    """Identify every source/header compiled into the controller bridge."""

    digest = hashlib.sha256()
    for path in (
        STM32_EKF_SOURCE,
        STM32_FLIGHT_SOURCE,
        STM32_EKF_HEADER,
        STM32_FLIGHT_HEADER,
        STM32_FEATURES_HEADER,
        STM32_BRIDGE_SOURCE,
    ):
        digest.update(path.relative_to(ROOT).as_posix().encode("utf-8"))
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest().upper()


def nested_get(tree: dict[str, Any], dotted_path: str) -> Any:
    """Read a dotted dictionary path from the resolved RocketPy config."""

    value: Any = tree
    for token in dotted_path.split("."):
        value = value[token]
    return value


def nested_set(tree: dict[str, Any], dotted_path: str, value: Any) -> None:
    """Set a dotted dictionary path without silently creating misspelled keys."""

    tokens = dotted_path.split(".")
    target: Any = tree
    for token in tokens[:-1]:
        if token not in target or not isinstance(target[token], dict):
            raise ValueError(f"unknown configuration path: {dotted_path}")
        target = target[token]
    if tokens[-1] not in target:
        raise ValueError(f"unknown configuration path: {dotted_path}")
    target[tokens[-1]] = value


def load_study_config(path: Path) -> dict[str, Any]:
    """Load and validate the uncertainty/acceptance contract."""

    values = json.loads(path.read_text(encoding="utf-8"))
    if values.get("schema_version") != 1:
        raise ValueError("Monte Carlo config schema_version must be 1")
    if values.get("sampling_method") != "latin_hypercube":
        raise ValueError("only latin_hypercube sampling is supported")
    parameters = values.get("parameters")
    if not isinstance(parameters, list) or not parameters:
        raise ValueError("Monte Carlo config must define at least one parameter")

    acceptance = values.get("acceptance")
    if not isinstance(acceptance, dict):
        raise ValueError("Monte Carlo config must define acceptance rules")
    rate_keys = (
        "required_safety_pass_rate",
        "required_effectiveness_pass_rate",
        "required_target_hit_rate",
        "required_envelope_pass_rate",
    )
    numeric_keys = (
        "passive_reference_tolerance_percent",
        "minimum_effective_apogee_reduction_ft",
        "maximum_commanded_descent_velocity_mps",
        "maximum_recovery_retraction_time_s",
        "maximum_p95_absolute_target_error_ft",
        "maximum_worst_absolute_target_error_ft",
    )
    for key in (*rate_keys, *numeric_keys):
        try:
            numeric = float(acceptance[key])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"acceptance.{key} must be numeric") from error
        if not math.isfinite(numeric):
            raise ValueError(f"acceptance.{key} must be finite")
        if key in rate_keys and not 0.0 <= numeric <= 1.0:
            raise ValueError(f"acceptance.{key} must be between 0 and 1")

    names: set[str] = set()
    used_paths: set[str] = set()
    for parameter in parameters:
        name = parameter.get("name")
        if (
            not isinstance(name, str)
            or not name
            or name in names
            or name in BASE_RUN_FIELDS
        ):
            raise ValueError(f"invalid or duplicate parameter name: {name!r}")
        names.add(name)
        paths = parameter.get("paths")
        if not isinstance(paths, list) or not paths:
            raise ValueError(f"{name} must define at least one config path")
        for dotted_path in paths:
            if not isinstance(dotted_path, str) or not dotted_path:
                raise ValueError(f"{name} contains an invalid config path")
            if dotted_path.startswith(("controller.", "requirements.")):
                raise ValueError(
                    f"{name} attempts to randomize fixed controller/mission path: "
                    f"{dotted_path}"
                )
            if dotted_path in used_paths:
                raise ValueError(f"config path is sampled more than once: {dotted_path}")
            used_paths.add(dotted_path)
        distribution = parameter.get("distribution")
        minimum = float(parameter.get("minimum"))
        maximum = float(parameter.get("maximum"))
        if not math.isfinite(minimum) or not math.isfinite(maximum) or minimum >= maximum:
            raise ValueError(f"{name} has invalid minimum/maximum bounds")
        if distribution == "triangular":
            mode = float(parameter.get("mode"))
            if not minimum <= mode <= maximum:
                raise ValueError(f"{name} triangular mode is outside its bounds")
        elif distribution != "uniform":
            raise ValueError(f"{name} uses unsupported distribution {distribution!r}")
    return values


def distribution_quantile(parameter: dict[str, Any], quantile: float) -> float:
    """Map a unit quantile into an explicit uniform or triangular assumption."""

    q = min(max(float(quantile), 0.0), 1.0)
    low = float(parameter["minimum"])
    high = float(parameter["maximum"])
    if parameter["distribution"] == "uniform":
        return low + q * (high - low)

    mode = float(parameter["mode"])
    split = (mode - low) / (high - low)
    if q < split:
        return low + math.sqrt(q * (high - low) * (mode - low))
    return high - math.sqrt((1.0 - q) * (high - low) * (high - mode))


def derive_run_seed(master_seed: int, run_index: int) -> int:
    """Derive an order-independent 32-bit seed for one numbered case."""

    payload = f"AMBAR:{master_seed}:{run_index}".encode("ascii")
    return int.from_bytes(hashlib.sha256(payload).digest()[:4], "big")


def build_latin_hypercube(
    parameters: Sequence[dict[str, Any]],
    run_count: int,
    master_seed: int,
) -> list[dict[str, float]]:
    """Create all stratified cases before running the first simulation.

    Every parameter uses each of the ``run_count`` strata exactly once.  The
    saved table therefore remains reviewable even if a later simulation fails.
    """

    if run_count <= 0:
        return []
    rng = random.Random(master_seed)
    rows: list[dict[str, float]] = [dict() for _ in range(run_count)]
    for parameter in parameters:
        quantiles = [(stratum + rng.random()) / run_count for stratum in range(run_count)]
        rng.shuffle(quantiles)
        for row, quantile in zip(rows, quantiles):
            row[parameter["name"]] = distribution_quantile(parameter, quantile)
    return rows


def apply_sampled_inputs(
    base_config: dict[str, Any],
    parameters: Sequence[dict[str, Any]],
    sampled: dict[str, float],
    run_seed: int,
) -> dict[str, Any]:
    """Copy the plant config, apply truth dispersions, and set the sensor seed."""

    config = copy.deepcopy(base_config)
    for parameter in parameters:
        value = sampled[parameter["name"]]
        for dotted_path in parameter["paths"]:
            nested_set(config, dotted_path, value)
    config["sensor_model"]["random_seed"] = run_seed
    config["model_status"] += "; seeded Monte Carlo truth dispersion applied"
    return config


def baseline_inputs(
    base_config: dict[str, Any], parameters: Sequence[dict[str, Any]]
) -> dict[str, float]:
    """Expose baseline values in the same columns as randomized inputs."""

    return {
        parameter["name"]: float(nested_get(base_config, parameter["paths"][0]))
        for parameter in parameters
    }


def illegal_phase_transitions(
    transitions: Sequence[dict[str, Any]],
) -> list[str]:
    """Reject phase regressions and every transition outside the flight policy."""

    illegal: list[str] = []
    names = [str(entry["phase"]) for entry in transitions]
    for current, following in zip(names, names[1:]):
        if following not in LEGAL_PHASE_TRANSITIONS.get(current, set()):
            illegal.append(f"{current}->{following}")
    return illegal


def root_mean_square(values: Iterable[float]) -> float:
    """Return RMS with a stable zero value for an empty diagnostic series."""

    samples = list(values)
    if not samples:
        return 0.0
    return math.sqrt(sum(value * value for value in samples) / len(samples))


def bool_csv(value: bool) -> str:
    """Use explicit lowercase booleans that round-trip cleanly through CSV."""

    return "true" if value else "false"


def evaluate_completed_trial(
    *,
    campaign_id: str,
    run_id: str,
    run_index: int,
    sample_index: int,
    mode: str,
    master_seed: int,
    run_seed: int,
    config: dict[str, Any],
    sampled_inputs: dict[str, float],
    controller_hash: str,
    passive: Any,
    controlled: Any,
    controller_log: list[dict[str, Any]],
    acceptance: dict[str, Any],
    runtime_s: float,
) -> dict[str, Any]:
    """Compute per-run metrics while keeping safety and performance separate."""

    requirements = config["requirements"]
    target_ft = float(requirements["target_apogee_ft"])
    tolerance_ft = float(requirements["target_tolerance_ft"])
    passive_apogee_agl_m = flight_apogee_agl_m(passive, config)
    controlled_apogee_agl_m = flight_apogee_agl_m(controlled, config)
    passive_ft = passive_apogee_agl_m * METERS_TO_FEET
    controlled_ft = controlled_apogee_agl_m * METERS_TO_FEET
    target_error_ft = controlled_ft - target_ft
    reduction_ft = passive_ft - controlled_ft
    reference_ft = float(requirements["m5_openrocket_passive_apogee_ft"])
    reference_difference_percent = 100.0 * (passive_ft - reference_ft) / reference_ft

    first_command = next(
        (entry for entry in controller_log if entry["command_fraction"] > 0.001),
        None,
    )
    first_deployment = next(
        (
            entry
            for entry in controller_log
            if entry["actual_deployment_fraction"] > 0.001
        ),
        None,
    )
    peak_command = max(
        (entry["command_fraction"] for entry in controller_log), default=0.0
    )
    peak_deployment = max(
        (entry["actual_deployment_fraction"] for entry in controller_log),
        default=0.0,
    )
    final_deployment = (
        float(controller_log[-1]["actual_deployment_fraction"])
        if controller_log
        else 0.0
    )
    minimum_command_time_s = (
        float(config["motor"]["burn_time_s"])
        + float(config["airbrakes"]["post_burn_enable_margin_s"])
    )
    early_command_samples = sum(
        1
        for entry in controller_log
        if entry["command_fraction"] > 0.001
        and entry["time_s"] + 1.0e-9 < minimum_command_time_s
    )
    descending_command_samples = sum(
        1
        for entry in controller_log
        if entry["command_fraction"] > 0.001
        and entry["truth_velocity_mps"] <= 0.0
    )
    maximum_commanded_descent_velocity_mps = float(
        acceptance["maximum_commanded_descent_velocity_mps"]
    )
    excessive_descent_command_samples = sum(
        1
        for entry in controller_log
        if entry["command_fraction"] > 0.001
        and entry["truth_velocity_mps"]
        <= maximum_commanded_descent_velocity_mps
    )
    maximum_deployment_below_recovery_velocity = max(
        (
            float(entry["actual_deployment_fraction"])
            for entry in controller_log
            if entry["truth_velocity_mps"]
            <= maximum_commanded_descent_velocity_mps
        ),
        default=0.0,
    )
    unhealthy_samples = sum(1 for entry in controller_log if not entry["healthy"])
    commands_bounded = bool(controller_log) and all(
        0.0 <= entry["command_fraction"] <= 1.0
        and 0.0 <= entry["actual_deployment_fraction"] <= 1.0
        for entry in controller_log
    )
    controller_healthy = bool(controller_log) and unhealthy_samples == 0
    deployment_after_burn = first_command is None or early_command_samples == 0
    command_phases_valid = all(
        entry["phase"] in {"Coast", "AirbrakeActive"}
        for entry in controller_log
        if entry["command_fraction"] > 0.001
    )

    transitions = derive_phase_transitions(controller_log)
    phase_names = [str(entry["phase"]) for entry in transitions]
    illegal = illegal_phase_transitions(transitions)
    recovery_observed = "Recovery" in phase_names
    retraction_observed = final_deployment <= 0.02
    recovery_entry = next(
        (entry for entry in controller_log if entry["phase"] == "Recovery"),
        None,
    )
    deployment_at_recovery = (
        float(recovery_entry["actual_deployment_fraction"])
        if recovery_entry is not None
        else math.nan
    )
    retracted_entry = (
        next(
            (
                entry
                for entry in controller_log
                if entry["phase"] == "Recovery"
                and entry["actual_deployment_fraction"] <= 0.02
            ),
            None,
        )
        if recovery_entry is not None
        else None
    )
    retraction_time_after_recovery_s = (
        float(retracted_entry["time_s"]) - float(recovery_entry["time_s"])
        if retracted_entry is not None and recovery_entry is not None
        else math.nan
    )
    timely_retraction = (
        retraction_observed
        and math.isfinite(retraction_time_after_recovery_s)
        and retraction_time_after_recovery_s
        <= float(acceptance["maximum_recovery_retraction_time_s"])
    )
    integrity_errors = validate_controller_log(controller_log)
    barometer_sample_count = sum(
        1 for entry in controller_log if entry["barometer_sample"]
    )

    altitude_errors = [
        float(entry["estimated_altitude_m"]) - float(entry["truth_altitude_m"])
        for entry in controller_log
    ]
    velocity_errors = [
        float(entry["estimated_velocity_mps"]) - float(entry["truth_velocity_mps"])
        for entry in controller_log
    ]
    prediction_errors = [
        float(entry["predicted_apogee_m"]) - controlled_apogee_agl_m
        for entry in controller_log
        if entry["phase"] in {"Coast", "AirbrakeActive"}
        and entry["truth_velocity_mps"] > 0.0
    ]

    maximum_mach = max(float(passive.max_mach_number), float(controlled.max_mach_number))
    minimum_rail_exit_fps = min(
        float(passive.out_of_rail_velocity) * METERS_TO_FEET,
        float(controlled.out_of_rail_velocity) * METERS_TO_FEET,
    )
    passive_reference_pass = abs(reference_difference_percent) <= float(
        acceptance["passive_reference_tolerance_percent"]
    )
    target_pass = abs(target_error_ft) <= tolerance_ft
    envelope_pass = (
        maximum_mach <= float(requirements["maximum_mach"])
        and minimum_rail_exit_fps
        >= float(requirements["minimum_rail_exit_velocity_fps"])
    )
    needs_braking = passive_ft > target_ft + tolerance_ft
    effectiveness_pass = controlled_ft <= passive_ft + 1.0
    if needs_braking:
        effectiveness_pass = (
            effectiveness_pass
            and peak_deployment > 0.05
            and reduction_ft
            >= float(acceptance["minimum_effective_apogee_reduction_ft"])
        )

    safety_pass = (
        commands_bounded
        and controller_healthy
        and deployment_after_burn
        and command_phases_valid
        and excessive_descent_command_samples == 0
        and not illegal
        and not integrity_errors
        and barometer_sample_count > 0
        and recovery_observed
        and retraction_observed
        and timely_retraction
        and controlled_ft <= passive_ft + 1.0
    )
    performance_pass = target_pass and envelope_pass and effectiveness_pass
    run_pass = safety_pass and performance_pass

    failure_reasons: list[str] = []
    checks = {
        "commands_out_of_bounds": commands_bounded,
        "controller_unhealthy": controller_healthy,
        "command_before_burnout_margin": deployment_after_burn,
        "command_outside_coast_airbrake_phase": command_phases_valid,
        "command_past_recovery_descent_limit": (
            excessive_descent_command_samples == 0
        ),
        "illegal_phase_transition": not illegal,
        "time_history_integrity": not integrity_errors,
        "barometer_samples_present": barometer_sample_count > 0,
        "recovery_observed": recovery_observed,
        "final_retraction": retraction_observed,
        "timely_retraction_after_recovery": timely_retraction,
        "controlled_not_above_passive": controlled_ft <= passive_ft + 1.0,
        "airbrake_effective": effectiveness_pass,
        "target_band": target_pass,
        "flight_envelope": envelope_pass,
    }
    failure_reasons.extend(name for name, passed in checks.items() if not passed)

    frontal_area_m2 = math.pi * float(config["rocket"]["radius_m"]) ** 2
    row: dict[str, Any] = {
        "campaign_id": campaign_id,
        "run_id": run_id,
        "run_index": run_index,
        "sample_index": sample_index,
        "mode": mode,
        "master_seed": master_seed,
        "run_seed": run_seed,
        "status": "PASS" if run_pass else "FAIL",
        "failure_reasons": ";".join(failure_reasons),
        "runtime_s": runtime_s,
        "config_sha256": sha256_bytes(canonical_json_bytes(config)),
        "controller_source_sha256": controller_hash,
        "controller_log_sha256": sha256_bytes(canonical_json_bytes(controller_log)),
        "target_apogee_ft": target_ft,
        "target_tolerance_ft": tolerance_ft,
        "equivalent_full_airbrake_cda_m2": (
            frontal_area_m2
            * float(config["airbrakes"]["drag_coefficient_at_full_deployment"])
        ),
        "passive_apogee_ft": passive_ft,
        "controlled_apogee_ft": controlled_ft,
        "target_error_ft": target_error_ft,
        "absolute_target_error_ft": abs(target_error_ft),
        "apogee_reduction_ft": reduction_ft,
        "passive_reference_difference_percent": reference_difference_percent,
        "maximum_mach": maximum_mach,
        "minimum_rail_exit_velocity_fps": minimum_rail_exit_fps,
        "first_command_time_s": first_command["time_s"] if first_command else "",
        "first_deployment_time_s": (
            first_deployment["time_s"] if first_deployment else ""
        ),
        "minimum_allowed_command_time_s": minimum_command_time_s,
        "peak_command_fraction": peak_command,
        "peak_deployment_fraction": peak_deployment,
        "final_deployment_fraction": final_deployment,
        "maximum_altitude_error_m": max(map(abs, altitude_errors), default=0.0),
        "altitude_rmse_m": root_mean_square(altitude_errors),
        "maximum_velocity_error_mps": max(map(abs, velocity_errors), default=0.0),
        "velocity_rmse_mps": root_mean_square(velocity_errors),
        "prediction_rmse_m": root_mean_square(prediction_errors),
        "barometer_sample_count": barometer_sample_count,
        "controller_sample_count": len(controller_log),
        "early_command_samples": early_command_samples,
        "descending_command_samples": descending_command_samples,
        "excessive_descent_command_samples": excessive_descent_command_samples,
        "maximum_commanded_descent_velocity_mps": (
            maximum_commanded_descent_velocity_mps
        ),
        "maximum_deployment_below_recovery_velocity_fraction": (
            maximum_deployment_below_recovery_velocity
        ),
        "deployment_at_recovery_fraction": deployment_at_recovery,
        "retraction_time_after_recovery_s": retraction_time_after_recovery_s,
        "unhealthy_samples": unhealthy_samples,
        "phase_sequence": "->".join(phase_names),
        "illegal_phase_transitions": ";".join(illegal),
        "log_integrity_error_count": len(integrity_errors),
        "passive_reference_pass": bool_csv(passive_reference_pass),
        "commands_bounded": bool_csv(commands_bounded),
        "deployment_after_burn": bool_csv(deployment_after_burn),
        "command_phases_valid": bool_csv(command_phases_valid),
        "recovery_observed": bool_csv(recovery_observed),
        "retraction_observed": bool_csv(retraction_observed),
        "timely_retraction": bool_csv(timely_retraction),
        "controller_healthy": bool_csv(controller_healthy),
        "effectiveness_pass": bool_csv(effectiveness_pass),
        "target_band_pass": bool_csv(target_pass),
        "flight_envelope_pass": bool_csv(envelope_pass),
        "safety_pass": bool_csv(safety_pass),
        "performance_pass": bool_csv(performance_pass),
        "run_pass": bool_csv(run_pass),
    }
    row.update(sampled_inputs)
    return row


def failed_trial_row(
    *,
    campaign_id: str,
    run_id: str,
    run_index: int,
    sample_index: int,
    mode: str,
    master_seed: int,
    run_seed: int,
    sampled_inputs: dict[str, float],
    config: dict[str, Any],
    controller_hash: str,
    error: BaseException,
    runtime_s: float,
) -> dict[str, Any]:
    """Keep execution failures in the denominator and preserve their inputs."""

    row: dict[str, Any] = {field: "" for field in BASE_RUN_FIELDS}
    row.update(
        {
            "campaign_id": campaign_id,
            "run_id": run_id,
            "run_index": run_index,
            "sample_index": sample_index,
            "mode": mode,
            "master_seed": master_seed,
            "run_seed": run_seed,
            "status": "ERROR",
            "failure_reasons": f"{type(error).__name__}: {error}",
            "runtime_s": runtime_s,
            "config_sha256": sha256_bytes(canonical_json_bytes(config)),
            "controller_source_sha256": controller_hash,
            "target_apogee_ft": config["requirements"]["target_apogee_ft"],
            "target_tolerance_ft": config["requirements"]["target_tolerance_ft"],
            "safety_pass": "false",
            "effectiveness_pass": "false",
            "target_band_pass": "false",
            "flight_envelope_pass": "false",
            "performance_pass": "false",
            "run_pass": "false",
        }
    )
    row.update(sampled_inputs)
    return row


def run_trial(
    *,
    campaign_id: str,
    run_id: str,
    run_index: int,
    sample_index: int,
    mode: str,
    master_seed: int,
    run_seed: int,
    config: dict[str, Any],
    sampled_inputs: dict[str, float],
    bridge_path: Path,
    controller_hash: str,
    acceptance: dict[str, Any],
) -> TrialArtifact:
    """Execute one paired passive/controlled RocketPy scenario."""

    started = time.perf_counter()
    try:
        passive = run_passive(config)
        observation_end_time_s = (
            float(passive.apogee_time)
            + float(config["airbrakes"]["post_apogee_observation_s"])
        )
        controlled, log = run_closed_loop(
            config,
            bridge_path,
            observation_end_time_s,
        )
        runtime_s = time.perf_counter() - started
        row = evaluate_completed_trial(
            campaign_id=campaign_id,
            run_id=run_id,
            run_index=run_index,
            sample_index=sample_index,
            mode=mode,
            master_seed=master_seed,
            run_seed=run_seed,
            config=config,
            sampled_inputs=sampled_inputs,
            controller_hash=controller_hash,
            passive=passive,
            controlled=controlled,
            controller_log=log,
            acceptance=acceptance,
            runtime_s=runtime_s,
        )
        return TrialArtifact(row, sampled_inputs, config, log)
    except Exception as error:  # Keep a failed attempt visible in runs.csv.
        runtime_s = time.perf_counter() - started
        return TrialArtifact(
            failed_trial_row(
                campaign_id=campaign_id,
                run_id=run_id,
                run_index=run_index,
                sample_index=sample_index,
                mode=mode,
                master_seed=master_seed,
                run_seed=run_seed,
                sampled_inputs=sampled_inputs,
                config=config,
                controller_hash=controller_hash,
                error=error,
                runtime_s=runtime_s,
            ),
            sampled_inputs,
            config,
            [],
        )


def percentile(values: Sequence[float], quantile: float) -> float:
    """Linear percentile used by both CSV and JSON aggregate outputs."""

    ordered = sorted(float(value) for value in values)
    if not ordered:
        return math.nan
    position = min(max(quantile, 0.0), 1.0) * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def wilson_interval(successes: int, trials: int) -> tuple[float, float]:
    """Return the two-sided 95% Wilson interval for a pass proportion."""

    if trials <= 0:
        return math.nan, math.nan
    z = 1.959963984540054
    proportion = successes / trials
    denominator = 1.0 + z * z / trials
    center = (proportion + z * z / (2.0 * trials)) / denominator
    margin = (
        z
        * math.sqrt(
            proportion * (1.0 - proportion) / trials
            + z * z / (4.0 * trials * trials)
        )
        / denominator
    )
    return max(0.0, center - margin), min(1.0, center + margin)


def rank_values(values: Sequence[float]) -> list[float]:
    """Assign average ranks so Spearman sensitivity needs no SciPy dependency."""

    indexed = sorted(enumerate(values), key=lambda item: item[1])
    ranks = [0.0] * len(indexed)
    position = 0
    while position < len(indexed):
        end = position + 1
        while end < len(indexed) and indexed[end][1] == indexed[position][1]:
            end += 1
        average_rank = (position + end - 1) / 2.0 + 1.0
        for sorted_position in range(position, end):
            ranks[indexed[sorted_position][0]] = average_rank
        position = end
    return ranks


def pearson_correlation(left: Sequence[float], right: Sequence[float]) -> float:
    """Calculate correlation with an explicit zero-variance guard."""

    if len(left) != len(right) or len(left) < 2:
        return math.nan
    left_mean = statistics.fmean(left)
    right_mean = statistics.fmean(right)
    numerator = sum(
        (a - left_mean) * (b - right_mean) for a, b in zip(left, right)
    )
    left_scale = math.sqrt(sum((value - left_mean) ** 2 for value in left))
    right_scale = math.sqrt(sum((value - right_mean) ** 2 for value in right))
    if left_scale == 0.0 or right_scale == 0.0:
        return math.nan
    return numerator / (left_scale * right_scale)


def summarize_campaign(
    artifacts: Sequence[TrialArtifact],
    acceptance: dict[str, Any],
    random_run_count: int,
) -> dict[str, Any]:
    """Aggregate campaign results without dropping errors from pass-rate bases."""

    baseline = [artifact.row for artifact in artifacts if artifact.row["mode"] == "baseline"]
    randomized = [
        artifact.row for artifact in artifacts if artifact.row["mode"] == "monte_carlo"
    ]
    completed_random = [row for row in randomized if row["status"] != "ERROR"]

    # Repeatability is evidence only when at least two identical baseline cases
    # were actually attempted.  A missing or single baseline must not be
    # reported as a vacuous success in the campaign summary.
    baseline_reproducible = len(baseline) >= 2 and all(
        row.get("status") != "ERROR" for row in baseline
    )
    if baseline_reproducible:
        comparison_fields = (
            "passive_apogee_ft",
            "controlled_apogee_ft",
            "target_error_ft",
            "apogee_reduction_ft",
            "phase_sequence",
            "controller_log_sha256",
            "status",
        )
        first = baseline[0]
        baseline_reproducible = all(
            all(row[field] == first[field] for field in comparison_fields)
            for row in baseline[1:]
        )
    baseline_model_reference_pass = bool(baseline) and all(
        row.get("passive_reference_pass") == "true" for row in baseline
    )

    total = len(randomized)
    safety_successes = sum(row["safety_pass"] == "true" for row in randomized)
    effectiveness_successes = sum(
        row.get("effectiveness_pass") == "true" for row in randomized
    )
    target_successes = sum(row["target_band_pass"] == "true" for row in randomized)
    envelope_successes = sum(
        row["flight_envelope_pass"] == "true" for row in randomized
    )
    overall_successes = sum(row["run_pass"] == "true" for row in randomized)
    target_interval = wilson_interval(target_successes, total)

    controlled_apogees = [float(row["controlled_apogee_ft"]) for row in completed_random]
    target_errors = [float(row["target_error_ft"]) for row in completed_random]
    absolute_errors = [float(row["absolute_target_error_ft"]) for row in completed_random]
    reductions = [float(row["apogee_reduction_ft"]) for row in completed_random]

    def describe(values: Sequence[float]) -> dict[str, float | None]:
        if not values:
            return {
                "minimum": None,
                "p05": None,
                "median": None,
                "mean": None,
                "p95": None,
                "maximum": None,
                "sample_stddev": None,
            }
        return {
            "minimum": min(values),
            "p05": percentile(values, 0.05),
            "median": percentile(values, 0.50),
            "mean": statistics.fmean(values),
            "p95": percentile(values, 0.95),
            "maximum": max(values),
            "sample_stddev": statistics.stdev(values) if len(values) > 1 else 0.0,
        }

    safety_rate = safety_successes / total if total else math.nan
    effectiveness_rate = effectiveness_successes / total if total else math.nan
    target_rate = target_successes / total if total else math.nan
    envelope_rate = envelope_successes / total if total else math.nan
    p95_abs = percentile(absolute_errors, 0.95) if absolute_errors else math.nan
    worst_abs = max(absolute_errors) if absolute_errors else math.nan
    campaign_pass = (
        baseline_reproducible
        and baseline_model_reference_pass
        and total == random_run_count
        and safety_rate >= float(acceptance["required_safety_pass_rate"])
        and effectiveness_rate
        >= float(acceptance["required_effectiveness_pass_rate"])
        and target_rate >= float(acceptance["required_target_hit_rate"])
        and envelope_rate >= float(acceptance["required_envelope_pass_rate"])
        and p95_abs <= float(acceptance["maximum_p95_absolute_target_error_ft"])
        and worst_abs <= float(acceptance["maximum_worst_absolute_target_error_ft"])
    )

    worst_rows = sorted(
        completed_random,
        key=lambda row: float(row["absolute_target_error_ft"]),
        reverse=True,
    )[:5]
    zero_safety_failure_upper_bound = (
        1.0 - 0.05 ** (1.0 / total)
        if total > 0 and safety_successes == total
        else None
    )
    zero_overall_failure_upper_bound = (
        1.0 - 0.05 ** (1.0 / total)
        if total > 0 and overall_successes == total
        else None
    )
    return {
        "campaign_pass": campaign_pass,
        "baseline_reproducible": baseline_reproducible,
        "baseline_model_reference_pass": baseline_model_reference_pass,
        "baseline_runs": len(baseline),
        "random_runs_attempted": total,
        "random_runs_completed": len(completed_random),
        "random_run_errors": total - len(completed_random),
        "safety_pass_count": safety_successes,
        "safety_pass_rate": safety_rate,
        "effectiveness_pass_count": effectiveness_successes,
        "effectiveness_pass_rate": effectiveness_rate,
        "target_hit_count": target_successes,
        "target_hit_rate": target_rate,
        "target_hit_wilson_95_low": target_interval[0],
        "target_hit_wilson_95_high": target_interval[1],
        "envelope_pass_count": envelope_successes,
        "envelope_pass_rate": envelope_rate,
        "overall_pass_count": overall_successes,
        "overall_pass_rate": overall_successes / total if total else math.nan,
        "zero_safety_failure_one_sided_95_upper_rate": (
            zero_safety_failure_upper_bound
        ),
        "zero_overall_failure_one_sided_95_upper_rate": (
            zero_overall_failure_upper_bound
        ),
        "controlled_apogee_ft": describe(controlled_apogees),
        "target_error_ft": describe(target_errors),
        "absolute_target_error_ft": describe(absolute_errors),
        "apogee_reduction_ft": describe(reductions),
        "worst_run_ids": [row["run_id"] for row in worst_rows],
        "acceptance": acceptance,
    }


def choose_representative_artifacts(
    artifacts: Sequence[TrialArtifact],
) -> list[TrialArtifact]:
    """Keep baseline, median, worst, and first safety failure without duplicates."""

    completed = [artifact for artifact in artifacts if artifact.controller_log]
    if not completed:
        return []
    selected: list[TrialArtifact] = []

    baseline = next(
        (artifact for artifact in completed if artifact.row["mode"] == "baseline"),
        None,
    )
    if baseline is not None:
        selected.append(baseline)

    randomized = [
        artifact for artifact in completed if artifact.row["mode"] == "monte_carlo"
    ]
    if randomized:
        ordered = sorted(
            randomized, key=lambda artifact: float(artifact.row["target_error_ft"])
        )
        selected.append(ordered[len(ordered) // 2])
        selected.append(
            max(
                randomized,
                key=lambda artifact: float(artifact.row["absolute_target_error_ft"]),
            )
        )
        failure = next(
            (
                artifact
                for artifact in randomized
                if artifact.row["safety_pass"] != "true"
            ),
            None,
        )
        if failure is not None:
            selected.append(failure)

    unique: dict[str, TrialArtifact] = {}
    for artifact in selected:
        unique[str(artifact.row["run_id"])] = artifact
    return list(unique.values())


def write_csv(path: Path, fieldnames: Sequence[str], rows: Iterable[dict[str, Any]]) -> None:
    """Atomically write stable UTF-8 CSV with explicit headers."""

    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    os.replace(temporary, path)


def write_json(path: Path, value: Any) -> None:
    """Atomically write strict JSON so interruptions cannot leave half a file."""

    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, allow_nan=False),
        encoding="utf-8",
    )
    os.replace(temporary, path)


def write_openrocket_profile(path: Path, artifact: TrialArtifact) -> None:
    """Export a representative truth trace accepted by replay_openrocket.py."""

    log = artifact.controller_log
    if not log:
        return
    transitions = derive_phase_transitions(log)
    phase_events = {
        "Boost": "CONTROLLER_BOOST",
        "Coast": "CONTROLLER_COAST",
        "AirbrakeActive": "CONTROLLER_AIRBRAKE_ACTIVE",
        "Recovery": "CONTROLLER_RECOVERY",
        "Fault": "CONTROLLER_FAULT",
    }
    apogee_entry = max(log, key=lambda entry: float(entry["truth_altitude_m"]))
    liftoff_entry = next(
        (
            entry
            for entry in log
            if float(entry["truth_altitude_m"]) > 0.5
            and float(entry["truth_speed_mps"]) > 1.0
        ),
        log[0],
    )

    with path.open("w", encoding="utf-8", newline="") as handle:
        handle.write(
            f"# AMBAR campaign representative run {artifact.row['run_id']}\n"
        )
        handle.write(
            "# Generated from provisional RocketPy truth; use for regression/HIL, not independent flight validation.\n"
        )
        handle.write(
            f"# Event LIFTOFF occurred at t={float(liftoff_entry['time_s']):.6f} seconds\n"
        )
        handle.write(
            "# Event BURNOUT occurred at "
            f"t={float(artifact.resolved_config['motor']['burn_time_s']):.6f} seconds\n"
        )
        for transition in transitions:
            event = phase_events.get(str(transition["phase"]))
            if event:
                handle.write(
                    f"# Event {event} occurred at t={float(transition['time_s']):.6f} seconds\n"
                )
        handle.write(
            f"# Event APOGEE occurred at t={float(apogee_entry['time_s']):.6f} seconds\n"
        )
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(
            [
                "Time (s)",
                "Altitude (m)",
                "Vertical velocity (m/s)",
                "Vertical acceleration (m/s^2)",
            ]
        )
        for entry in log:
            writer.writerow(
                [
                    f"{float(entry['time_s']):.6f}",
                    f"{float(entry['truth_altitude_m']):.6f}",
                    f"{float(entry['truth_velocity_mps']):.6f}",
                    f"{float(entry['truth_acceleration_mps2']):.6f}",
                ]
            )


def write_campaign_outputs(
    *,
    output_dir: Path,
    campaign_id: str,
    artifacts: Sequence[TrialArtifact],
    parameters: Sequence[dict[str, Any]],
    study_config: dict[str, Any],
    summary: dict[str, Any],
    manifest: dict[str, Any],
) -> None:
    """Write the complete evidence bundle only after every attempt is recorded."""

    parameter_names = [str(parameter["name"]) for parameter in parameters]
    run_fields = BASE_RUN_FIELDS + parameter_names
    write_csv(output_dir / "runs.csv", run_fields, (artifact.row for artifact in artifacts))

    parameter_rows = []
    for artifact in artifacts:
        row = {
            "campaign_id": campaign_id,
            "run_id": artifact.row["run_id"],
            "run_index": artifact.row["run_index"],
            "sample_index": artifact.row["sample_index"],
            "mode": artifact.row["mode"],
            "master_seed": artifact.row["master_seed"],
            "run_seed": artifact.row["run_seed"],
            "resolved_config_sha256": artifact.row["config_sha256"],
        }
        row.update(artifact.sampled_inputs)
        parameter_rows.append(row)
    write_csv(
        output_dir / "parameters.csv",
        [
            "campaign_id",
            "run_id",
            "run_index",
            "sample_index",
            "mode",
            "master_seed",
            "run_seed",
            "resolved_config_sha256",
        ]
        + parameter_names,
        parameter_rows,
    )

    aggregate_rows: list[dict[str, Any]] = []

    def add_metric(name: str, value: Any, units: str = "", note: str = "") -> None:
        aggregate_rows.append(
            {"metric": name, "value": value, "units": units, "note": note}
        )

    for name in (
        "campaign_pass",
        "baseline_reproducible",
        "baseline_model_reference_pass",
        "baseline_runs",
        "random_runs_attempted",
        "random_runs_completed",
        "random_run_errors",
        "safety_pass_count",
        "safety_pass_rate",
        "effectiveness_pass_count",
        "effectiveness_pass_rate",
        "target_hit_count",
        "target_hit_rate",
        "target_hit_wilson_95_low",
        "target_hit_wilson_95_high",
        "envelope_pass_count",
        "envelope_pass_rate",
        "overall_pass_count",
        "overall_pass_rate",
        "zero_safety_failure_one_sided_95_upper_rate",
        "zero_overall_failure_one_sided_95_upper_rate",
    ):
        add_metric(name, summary[name])
    for family in (
        "controlled_apogee_ft",
        "target_error_ft",
        "absolute_target_error_ft",
        "apogee_reduction_ft",
    ):
        for statistic_name, value in summary[family].items():
            add_metric(f"{family}.{statistic_name}", value, "ft")
    add_metric("worst_run_ids", ";".join(summary["worst_run_ids"]))
    write_csv(
        output_dir / "aggregate_metrics.csv",
        ["metric", "value", "units", "note"],
        aggregate_rows,
    )

    completed_random = [
        artifact
        for artifact in artifacts
        if artifact.row["mode"] == "monte_carlo"
        and artifact.row["status"] != "ERROR"
    ]
    sensitivity_rows: list[dict[str, Any]] = []
    if len(completed_random) >= 3:
        error_ranks = rank_values(
            [float(artifact.row["target_error_ft"]) for artifact in completed_random]
        )
        absolute_error_ranks = rank_values(
            [
                float(artifact.row["absolute_target_error_ft"])
                for artifact in completed_random
            ]
        )
        for parameter in parameters:
            input_ranks = rank_values(
                [
                    float(artifact.sampled_inputs[parameter["name"]])
                    for artifact in completed_random
                ]
            )
            sensitivity_rows.append(
                {
                    "parameter": parameter["name"],
                    "units": parameter.get("units", ""),
                    "status": parameter.get("status", ""),
                    "spearman_vs_signed_target_error": pearson_correlation(
                        input_ranks, error_ranks
                    ),
                    "spearman_vs_absolute_target_error": pearson_correlation(
                        input_ranks, absolute_error_ranks
                    ),
                }
            )
        sensitivity_rows.sort(
            key=lambda row: abs(float(row["spearman_vs_signed_target_error"])),
            reverse=True,
        )
    write_csv(
        output_dir / "sensitivity.csv",
        [
            "parameter",
            "units",
            "status",
            "spearman_vs_signed_target_error",
            "spearman_vs_absolute_target_error",
        ],
        sensitivity_rows,
    )

    representatives = choose_representative_artifacts(artifacts)
    time_series_fields = [
        "run_id",
        "mode",
        "run_seed",
        "time_s",
        "truth_altitude_m",
        "truth_velocity_mps",
        "truth_speed_mps",
        "truth_acceleration_mps2",
        "truth_sensor_axis_acceleration_mps2",
        "measured_acceleration_mps2",
        "measured_altitude_m",
        "barometer_sample",
        "measurement_source_time_s",
        "estimated_altitude_m",
        "estimated_velocity_mps",
        "predicted_apogee_m",
        "command_fraction",
        "desired_deployment_fraction",
        "delayed_desired_deployment_fraction",
        "actual_deployment_fraction",
        "inhibit",
        "inhibit_flags",
        "phase",
        "healthy",
    ]
    time_series_rows: list[dict[str, Any]] = []
    profiles_dir = output_dir / "representative_profiles"
    profiles_dir.mkdir()
    for artifact in representatives:
        for entry in artifact.controller_log:
            row = {
                "run_id": artifact.row["run_id"],
                "mode": artifact.row["mode"],
                "run_seed": artifact.row["run_seed"],
            }
            row.update(entry)
            time_series_rows.append(row)
        write_openrocket_profile(
            profiles_dir / f"{artifact.row['run_id']}.csv", artifact
        )
    write_csv(
        output_dir / "representative_timeseries.csv",
        time_series_fields,
        time_series_rows,
    )

    write_json(output_dir / "summary.json", summary)
    write_json(output_dir / "manifest.json", manifest)
    write_json(output_dir / "resolved_study_config.json", study_config)
    (output_dir / "README.txt").write_text(
        "AMBAR MONTE CARLO CAMPAIGN\n\n"
        "runs.csv: one row for every attempted baseline/randomized run.\n"
        "parameters.csv: exact seed and sampled truth inputs for reproduction.\n"
        "aggregate_metrics.csv: campaign pass rates and distribution statistics.\n"
        "sensitivity.csv: exploratory Spearman rank sensitivity.\n"
        "representative_timeseries.csv: baseline, median, worst, and first safety-failure traces.\n"
        "representative_profiles/: explicit vertical profiles accepted by replay_openrocket.py.\n"
        "summary.json and manifest.json: machine-readable acceptance and provenance.\n\n"
        "LIMITATION: default uncertainty bounds and vehicle aerodynamics are provisional.\n"
        "This campaign is production-controller SIL, not physical actuator or flight validation.\n",
        encoding="utf-8",
    )


def git_context() -> dict[str, Any]:
    """Record the checkout identity without requiring a clean worktree."""

    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip()
        dirty = bool(
            subprocess.check_output(
                ["git", "status", "--porcelain"], cwd=ROOT, text=True
            ).strip()
        )
        return {"commit": commit, "dirty": dirty}
    except (OSError, subprocess.CalledProcessError):
        return {"commit": None, "dirty": None}


def snapshot_material_inputs(
    output_dir: Path,
    paths: Sequence[Path],
) -> list[dict[str, str]]:
    """Copy and hash every material campaign input before the first trial.

    The live checkout may be dirty or change later.  These snapshots, including
    the freshly built executable, preserve the exact artifacts that were used
    even when a campaign is interrupted before aggregate output is produced.
    """

    snapshot_root = output_dir / "input_snapshot"
    snapshot_root.mkdir()
    records: list[dict[str, str]] = []
    seen: set[Path] = set()
    for original in paths:
        source = original.resolve()
        if source in seen:
            continue
        seen.add(source)
        if not source.is_file():
            raise FileNotFoundError(f"material campaign input not found: {source}")
        try:
            relative = source.relative_to(ROOT)
        except ValueError:
            relative = Path("external") / (
                f"{sha256_file(source)[:12]}-{source.name}"
            )
        destination = snapshot_root / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        # Read once, then hash and write those exact bytes.  This avoids a
        # second live-source read (and OneDrive's occasional immediate post-copy
        # file lock) while guaranteeing the hash names the snapshot payload.
        payload = source.read_bytes()
        snapshot_hash = sha256_bytes(payload)
        destination.write_bytes(payload)
        records.append(
            {
                "source": str(source),
                "snapshot": str(destination.relative_to(output_dir)),
                "sha256": snapshot_hash,
            }
        )

    try:
        diff = subprocess.check_output(
            ["git", "diff", "--binary", "HEAD"], cwd=ROOT
        )
    except (OSError, subprocess.CalledProcessError):
        diff = b""
    if diff:
        diff_path = snapshot_root / "git-working-tree.diff"
        diff_path.write_bytes(diff)
        records.append(
            {
                "source": "git diff --binary HEAD",
                "snapshot": str(diff_path.relative_to(output_dir)),
                "sha256": sha256_bytes(diff),
            }
        )
    return records


def checkpoint_runs(
    output_dir: Path,
    artifacts: Sequence[TrialArtifact],
    parameter_names: Sequence[str],
) -> None:
    """Persist every completed/error attempt atomically during a campaign."""

    write_csv(
        output_dir / "runs.csv",
        BASE_RUN_FIELDS + list(parameter_names),
        (artifact.row for artifact in artifacts),
    )


def main() -> int:
    """Parse the campaign contract, run every case, and emit one evidence bundle."""

    parser = argparse.ArgumentParser(
        description="Run baseline and seeded RocketPy/STM32-C Monte Carlo campaigns."
    )
    parser.add_argument("--bridge", type=Path, required=True)
    parser.add_argument("--base-overrides", type=Path)
    parser.add_argument("--study-config", type=Path, default=DEFAULT_STUDY_CONFIG)
    parser.add_argument("--baseline-runs", type=int)
    parser.add_argument("--random-runs", type=int)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()

    study_config = load_study_config(args.study_config.resolve())
    baseline_run_count = (
        args.baseline_runs
        if args.baseline_runs is not None
        else int(study_config["default_baseline_runs"])
    )
    random_run_count = (
        args.random_runs
        if args.random_runs is not None
        else int(study_config["default_random_runs"])
    )
    master_seed = (
        args.seed
        if args.seed is not None
        else int(study_config["default_master_seed"])
    )
    if baseline_run_count < 0:
        parser.error("baseline run count must be nonnegative")
    if random_run_count <= 0:
        parser.error("at least one randomized run is required")
    bridge_path = args.bridge.resolve()
    if not bridge_path.is_file():
        parser.error(f"controller bridge not found: {bridge_path}")

    base_config = load_config(args.base_overrides.resolve() if args.base_overrides else None)
    parameters = study_config["parameters"]
    # Validate every path before creating the campaign directory or running an
    # expensive case.  The same check prevents a typo from being silently absent
    # from the resulting uncertainty study.
    for parameter in parameters:
        for path in parameter["paths"]:
            leaf = nested_get(base_config, path)
            if (
                isinstance(leaf, bool)
                or not isinstance(leaf, (int, float))
                or not math.isfinite(float(leaf))
            ):
                parser.error(f"sampled configuration path is not a finite number: {path}")

    started_utc = datetime.now(timezone.utc)
    campaign_id = (
        f"{started_utc.strftime('%Y%m%dT%H%M%SZ')}-seed-{master_seed}"
    )
    output_dir = (
        args.output_dir.resolve()
        if args.output_dir
        else (DEFAULT_OUTPUT_ROOT / campaign_id).resolve()
    )
    if output_dir.exists() and any(output_dir.iterdir()):
        parser.error(f"output directory is not empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

    controller_hash = combined_controller_source_hash()
    random_samples = build_latin_hypercube(parameters, random_run_count, master_seed)
    nominal_inputs = baseline_inputs(base_config, parameters)
    parameter_names = [str(parameter["name"]) for parameter in parameters]

    # Resolve the complete campaign before trial one.  The controller section in
    # each resolved config remains fixed; only paths explicitly listed in the
    # study contract are randomized plant/sensor/virtual-actuator truth.
    trial_plans: list[dict[str, Any]] = []
    for baseline_index in range(baseline_run_count):
        trial_plans.append(
            {
                "run_id": f"baseline-{baseline_index + 1:03d}",
                "run_index": len(trial_plans),
                "sample_index": baseline_index,
                "mode": "baseline",
                "run_seed": int(base_config["sensor_model"]["random_seed"]),
                "config": copy.deepcopy(base_config),
                "sampled_inputs": copy.deepcopy(nominal_inputs),
            }
        )
    for random_index, sampled in enumerate(random_samples):
        run_seed = derive_run_seed(master_seed, random_index)
        trial_plans.append(
            {
                "run_id": f"monte-carlo-{random_index + 1:03d}",
                "run_index": len(trial_plans),
                "sample_index": random_index,
                "mode": "monte_carlo",
                "run_seed": run_seed,
                "config": apply_sampled_inputs(
                    base_config, parameters, sampled, run_seed
                ),
                "sampled_inputs": sampled,
            }
        )

    parameter_rows: list[dict[str, Any]] = []
    for plan in trial_plans:
        parameter_row = {
            "campaign_id": campaign_id,
            "run_id": plan["run_id"],
            "run_index": plan["run_index"],
            "sample_index": plan["sample_index"],
            "mode": plan["mode"],
            "master_seed": master_seed,
            "run_seed": plan["run_seed"],
            "resolved_config_sha256": sha256_bytes(
                canonical_json_bytes(plan["config"])
            ),
        }
        parameter_row.update(plan["sampled_inputs"])
        parameter_rows.append(parameter_row)

    parameter_fields = [
        "campaign_id",
        "run_id",
        "run_index",
        "sample_index",
        "mode",
        "master_seed",
        "run_seed",
        "resolved_config_sha256",
        *parameter_names,
    ]
    write_csv(output_dir / "parameters.csv", parameter_fields, parameter_rows)
    checkpoint_runs(output_dir, [], parameter_names)
    write_json(output_dir / "resolved_base_config.json", base_config)
    write_json(output_dir / "resolved_study_config.json", study_config)

    thrust_curve_source = (
        MODEL_DIR / str(base_config["motor"]["thrust_curve"])
    ).resolve()
    bridge_build_metadata = bridge_path.with_suffix(".build.txt")
    material_paths = [
        MODEL_DIR / "ambar_reference_config.json",
        args.study_config.resolve(),
        bridge_path,
        bridge_build_metadata,
        Path(__file__),
        ROCKETPY_RUNNER_SOURCE,
        thrust_curve_source,
        SIMULATION_REQUIREMENTS,
        STM32_EKF_SOURCE,
        STM32_FLIGHT_SOURCE,
        STM32_EKF_HEADER,
        STM32_FLIGHT_HEADER,
        STM32_FEATURES_HEADER,
        STM32_BRIDGE_SOURCE,
        ROOT / "scripts" / "build_stm32_controller_bridge.ps1",
        ROOT / "scripts" / "run_monte_carlo.ps1",
    ]
    if args.base_overrides:
        material_paths.append(args.base_overrides.resolve())
    input_snapshots = snapshot_material_inputs(output_dir, material_paths)
    bridge_snapshot_record = next(
        record
        for record in input_snapshots
        if Path(record["source"]) == bridge_path
    )
    campaign_bridge_path = (
        output_dir / bridge_snapshot_record["snapshot"]
    ).resolve()

    manifest = {
        "schema_version": 1,
        "campaign_id": campaign_id,
        "status": "running",
        "study_status": study_config["study_status"],
        "started_utc": started_utc.isoformat(),
        "finished_utc": None,
        "duration_s": None,
        "command": [sys.executable, *sys.argv],
        "host": {
            "platform": platform.platform(),
            "python": sys.version,
            "rocketpy": importlib.metadata.version("rocketpy"),
            "processor_count": os.cpu_count(),
        },
        "git": git_context(),
        "inputs": {
            "base_config": str(MODEL_DIR / "ambar_reference_config.json"),
            "base_config_sha256": sha256_file(
                MODEL_DIR / "ambar_reference_config.json"
            ),
            "base_overrides": str(args.base_overrides.resolve())
            if args.base_overrides
            else None,
            "base_overrides_sha256": sha256_file(args.base_overrides.resolve())
            if args.base_overrides
            else None,
            "resolved_base_config_sha256": sha256_bytes(
                canonical_json_bytes(base_config)
            ),
            "study_config": str(args.study_config.resolve()),
            "study_config_sha256": sha256_file(args.study_config.resolve()),
            "controller_bridge": str(bridge_path),
            "controller_bridge_sha256": sha256_file(bridge_path),
            "executed_controller_bridge_snapshot": str(campaign_bridge_path),
            "controller_bridge_build_metadata": str(bridge_build_metadata),
            "controller_bridge_build_metadata_sha256": sha256_file(
                bridge_build_metadata
            ),
            "controller_source_sha256": controller_hash,
            "thrust_curve": str(thrust_curve_source),
            "thrust_curve_sha256": sha256_file(thrust_curve_source),
            "simulation_requirements_sha256": sha256_file(
                SIMULATION_REQUIREMENTS
            ),
            "stm32_ekf_source_sha256": sha256_file(STM32_EKF_SOURCE),
            "stm32_flight_source_sha256": sha256_file(STM32_FLIGHT_SOURCE),
            "snapshots": input_snapshots,
        },
        "campaign": {
            "sampling_method": study_config["sampling_method"],
            "master_seed": master_seed,
            "baseline_runs": baseline_run_count,
            "random_runs": random_run_count,
            "parameter_order": [parameter["name"] for parameter in parameters],
            "acceptance": study_config["acceptance"],
        },
        "outputs": [
            "runs.csv",
            "parameters.csv",
            "aggregate_metrics.csv",
            "sensitivity.csv",
            "representative_timeseries.csv",
            "representative_profiles/",
            "summary.json",
            "resolved_study_config.json",
            "resolved_base_config.json",
            "input_snapshot/",
            "README.txt",
        ],
        "limitations": [
            "Mass, CG, inertia, rocket drag, airbrake drag, sensor error, and actuator bounds are provisional.",
            "RocketPy physics and production STM32 estimator/flight modules are used for every case; this is not independent model validation.",
            "The bridge uses the fixed host controller configuration recorded in resolved_base_config.json; saved configuration on a physical board may differ.",
            "The campaign does not execute STM32 USB scheduling, sensor drivers, TMC5240 hardware, or physical airbrakes.",
            "Fifty runs are an exploratory screen and weak evidence for rare-event probability.",
        ],
    }
    write_json(output_dir / "manifest.json", manifest)

    print("AMBAR production-controller Monte Carlo campaign")
    print(f"Campaign: {campaign_id}")
    print(f"Baseline repeats: {baseline_run_count}")
    print(f"Randomized Latin-hypercube runs: {random_run_count}")
    print(f"Master seed: {master_seed}")
    print("Physical motor: NOT USED")
    print(f"Planned inputs saved before run 1: {output_dir / 'parameters.csv'}")

    artifacts: list[TrialArtifact] = []
    campaign_clock = time.perf_counter()
    total_runs = len(trial_plans)
    for position, plan in enumerate(trial_plans, start=1):
        seed_note = (
            f" seed={plan['run_seed']}" if plan["mode"] == "monte_carlo" else ""
        )
        print(
            f"[{position}/{total_runs}] {plan['run_id']}{seed_note}",
            flush=True,
        )
        artifact = run_trial(
            campaign_id=campaign_id,
            run_id=plan["run_id"],
            run_index=plan["run_index"],
            sample_index=plan["sample_index"],
            mode=plan["mode"],
            master_seed=master_seed,
            run_seed=plan["run_seed"],
            config=plan["config"],
            sampled_inputs=plan["sampled_inputs"],
            bridge_path=campaign_bridge_path,
            controller_hash=controller_hash,
            acceptance=study_config["acceptance"],
        )
        artifacts.append(artifact)
        checkpoint_runs(output_dir, artifacts, parameter_names)
        elapsed_s = time.perf_counter() - campaign_clock
        eta_s = elapsed_s / position * (total_runs - position)
        print(
            f"    {artifact.row['status']} | elapsed {elapsed_s:.0f} s | ETA {eta_s:.0f} s",
            flush=True,
        )

    summary = summarize_campaign(
        artifacts, study_config["acceptance"], random_run_count
    )
    finished_utc = datetime.now(timezone.utc)
    manifest["status"] = "completed"
    manifest["finished_utc"] = finished_utc.isoformat()
    manifest["duration_s"] = (finished_utc - started_utc).total_seconds()
    manifest["completed_run_rows"] = len(artifacts)
    manifest["campaign_pass"] = summary["campaign_pass"]
    write_campaign_outputs(
        output_dir=output_dir,
        campaign_id=campaign_id,
        artifacts=artifacts,
        parameters=parameters,
        study_config=study_config,
        summary=summary,
        manifest=manifest,
    )

    print("\nCAMPAIGN SUMMARY")
    print(f"Baseline reproducible: {summary['baseline_reproducible']}")
    print(
        "Baseline model/reference agreement: "
        f"{summary['baseline_model_reference_pass']}"
    )
    print(
        f"Safety passes: {summary['safety_pass_count']}/{summary['random_runs_attempted']}"
    )
    print(
        "Effective-control passes: "
        f"{summary['effectiveness_pass_count']}/{summary['random_runs_attempted']}"
    )
    print(
        f"Target-band hits: {summary['target_hit_count']}/{summary['random_runs_attempted']}"
    )
    print(
        f"Envelope passes: {summary['envelope_pass_count']}/{summary['random_runs_attempted']}"
    )
    print(f"Campaign result: {'PASS' if summary['campaign_pass'] else 'FAIL'}")
    print(f"Results: {output_dir}")
    print(
        "Interpretation: screening evidence only; default distributions and "
        "aerodynamics are not measurement-backed."
    )
    return 0 if summary["campaign_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
