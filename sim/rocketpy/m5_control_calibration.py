"""Deterministic gates for the provisional M5 VARIABLE_HIL controller profile.

This tool keeps three questions separate:

1. Does the unchanged RocketPy mass/motor plant reproduce the documented
   3379 ft passive reference after fitting only the rocket drag coefficient?
2. At the earliest safe coast state, does the provisional airbrake CdA bracket
   the required authority thresholds in at least 96 percent of cases?
3. Does the production C fixed-deployment predictor meet its error gates against
   a finer-step independent implementation, and how does proportional control
   compare with the eight-iteration predictive fraction solve?

It is deterministic and never opens a COM port, flashes a board, or moves a
motor.  Default counts are 200 tuning plus 500 held-out cases; ``--quick`` is
intended only for fast developer smoke tests.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import random
import statistics
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from run_rocketpy_sim import (
    ControllerBridge,
    FEET_TO_METERS,
    METERS_TO_FEET,
    flight_apogee_agl_m,
    load_config,
    run_fixed_deployment,
    run_passive,
)


MODEL_DIR = Path(__file__).resolve().parent
ROOT = MODEL_DIR.parents[1]
VARIABLE_CONFIG_PATH = MODEL_DIR / "variable_hil_m5_config.json"
DEFAULT_RESULT_PATH = ROOT / "build" / "m5-control-calibration-last-run.json"

AUTHORITY_REQUIRED_RATE = 0.96
RETRACTED_MINIMUM_FT = 3100.0
FULL_MAXIMUM_FT = 2900.0
CALIBRATION_RMSE_LIMIT_FT = 50.0
CALIBRATION_FIRST_ACTION_BIAS_LIMIT_FT = 25.0
CALIBRATION_P95_LIMIT_FT = 75.0


@dataclass(frozen=True)
class CoastCase:
    altitude_m: float
    velocity_mps: float
    deployment_fraction: float
    first_action: bool
    passive_apogee_ft: float = 3379.0
    mass_scale: float = 1.0
    baseline_cda_scale: float = 1.0
    incremental_cda_scale: float = 1.0
    density_scale: float = 1.0
    delay_scale: float = 1.0
    open_rate_scale: float = 1.0
    close_rate_scale: float = 1.0
    reference_apogee_m: float | None = None


@dataclass(frozen=True)
class CalibrationMetrics:
    case_count: int
    rmse_ft: float
    first_action_bias_ft: float
    p95_absolute_error_ft: float
    maximum_absolute_error_ft: float
    passed: bool


def percentile(values: list[float], quantile: float) -> float:
    """Return a deterministic linearly interpolated percentile."""
    if not values:
        return math.nan
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def cda_for_fraction(predictor: dict[str, Any], fraction: float) -> float:
    points = predictor["deployment_drag_area_m2"]
    bounded = max(0.0, min(1.0, fraction))
    scaled = bounded * (len(points) - 1)
    lower = min(len(points) - 2, int(scaled))
    local = scaled - lower
    return float(points[lower]) + local * (float(points[lower + 1]) - float(points[lower]))


def reference_predict_apogee(
    predictor: dict[str, Any],
    altitude_m: float,
    velocity_mps: float,
    current_fraction: float,
    target_fraction: float,
) -> float:
    """Fine-step reference independent of the C loop discretization."""
    if velocity_mps <= 0.0:
        return altitude_m
    dt_s = min(0.002, float(predictor["time_step_s"]) / 10.0)
    max_time_s = float(predictor["max_predict_time_s"])
    deployment = max(0.0, min(1.0, current_fraction))
    target = max(0.0, min(1.0, target_fraction))
    altitude = altitude_m
    velocity = velocity_mps
    elapsed = 0.0

    while elapsed < max_time_s and velocity > 0.0:
        if elapsed + 1.0e-12 >= float(predictor["actuator_delay_s"]):
            rate = (
                float(predictor["actuator_open_rate_fraction_per_s"])
                if target >= deployment
                else float(predictor["actuator_close_rate_fraction_per_s"])
            )
            maximum_change = rate * dt_s
            deployment += max(
                -maximum_change,
                min(maximum_change, target - deployment),
            )
        absolute_altitude = float(predictor["launch_site_elevation_m"]) + altitude
        density = float(predictor["sea_level_air_density_kgpm3"]) * math.exp(
            -absolute_altitude / float(predictor["density_scale_height_m"])
        )
        drag_k = (
            0.5
            * density
            * cda_for_fraction(predictor, deployment)
            / float(predictor["coast_mass_kg"])
        )
        acceleration = -9.80665 - drag_k * velocity * velocity
        next_velocity = velocity + acceleration * dt_s
        if next_velocity <= 0.0:
            stop_time = max(0.0, min(dt_s, velocity / -acceleration))
            altitude += velocity * stop_time + 0.5 * acceleration * stop_time**2
            break
        altitude += velocity * dt_s + 0.5 * acceleration * dt_s**2
        velocity = next_velocity
        elapsed += dt_s
    return max(altitude_m, altitude)


def make_bridge(config: dict[str, Any], bridge_path: Path) -> ControllerBridge:
    controller = config["controller"]
    return ControllerBridge(
        bridge_path,
        float(controller["minimum_boost_time_s"]),
        float(controller["target_apogee_ft"]) * FEET_TO_METERS,
        float(controller["target_tolerance_ft"]) * FEET_TO_METERS,
        controller_config=controller,
        predictor_config=config["predictor"],
    )


def solve_velocity_for_apogee(
    predictor: dict[str, Any], altitude_m: float, target_apogee_m: float
) -> float:
    lower_velocity = 1.0
    upper_velocity = 300.0
    for _ in range(40):
        middle = 0.5 * (lower_velocity + upper_velocity)
        prediction = reference_predict_apogee(
            predictor, altitude_m, middle, 0.0, 0.0
        )
        if prediction < target_apogee_m:
            lower_velocity = middle
        else:
            upper_velocity = middle
    return 0.5 * (lower_velocity + upper_velocity)


def build_authority_cases(
    predictor: dict[str, Any], case_count: int, seed: int
) -> list[CoastCase]:
    randomizer = random.Random(seed)
    cases: list[CoastCase] = []
    passive_reference_ft = float(predictor["passive_reference_apogee_ft"])
    for _ in range(case_count):
        # Earliest-safe coast states span plausible burnout altitudes while the
        # no-action apogee spans +/-100 ft around the documented passive result.
        altitude_m = randomizer.uniform(150.0, 300.0)
        passive_apogee_ft = passive_reference_ft + randomizer.uniform(-100.0, 100.0)
        provisional_case = CoastCase(
            altitude_m,
            0.0,
            0.0,
            True,
            passive_apogee_ft=passive_apogee_ft,
            mass_scale=randomizer.uniform(0.95, 1.05),
            baseline_cda_scale=randomizer.uniform(0.95, 1.05),
            incremental_cda_scale=randomizer.uniform(0.85, 1.15),
            density_scale=randomizer.uniform(0.95, 1.05),
            delay_scale=randomizer.uniform(0.5, 2.0),
            open_rate_scale=randomizer.uniform(0.9, 1.1),
            close_rate_scale=randomizer.uniform(0.9, 1.1),
        )
        case_predictor = predictor_for_authority_case(predictor, provisional_case)
        velocity_mps = solve_velocity_for_apogee(
            case_predictor, altitude_m, passive_apogee_ft * FEET_TO_METERS
        )
        cases.append(
            CoastCase(
                **{
                    **asdict(provisional_case),
                    "velocity_mps": velocity_mps,
                }
            )
        )
    return cases


def predictor_for_authority_case(
    predictor: dict[str, Any], case: CoastCase
) -> dict[str, Any]:
    """Apply plant-only screening scales without changing controller inputs."""
    varied = copy.deepcopy(predictor)
    baseline = float(predictor["baseline_drag_area_m2"])
    varied_baseline = baseline * case.baseline_cda_scale
    varied["coast_mass_kg"] = float(predictor["coast_mass_kg"]) * case.mass_scale
    varied["baseline_drag_area_m2"] = varied_baseline
    varied["deployment_drag_area_m2"] = [
        varied_baseline
        + (float(point) - baseline) * case.incremental_cda_scale
        for point in predictor["deployment_drag_area_m2"]
    ]
    varied["sea_level_air_density_kgpm3"] = (
        float(predictor["sea_level_air_density_kgpm3"]) * case.density_scale
    )
    varied["actuator_delay_s"] = (
        float(predictor["actuator_delay_s"]) * case.delay_scale
    )
    varied["actuator_open_rate_fraction_per_s"] = (
        float(predictor["actuator_open_rate_fraction_per_s"])
        * case.open_rate_scale
    )
    varied["actuator_close_rate_fraction_per_s"] = (
        float(predictor["actuator_close_rate_fraction_per_s"])
        * case.close_rate_scale
    )
    return varied


def run_authority_sweep(
    bridge: ControllerBridge,
    cases: list[CoastCase],
    predictor: dict[str, Any] | None = None,
) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    passes = 0
    for case in cases:
        c_retracted_ft = bridge.predict(
            case.altitude_m, case.velocity_mps, 0.0, 0.0
        ) * METERS_TO_FEET
        c_earliest_safe_full_ft = bridge.predict(
            case.altitude_m, case.velocity_mps, 0.0, 1.0
        ) * METERS_TO_FEET
        if predictor is None:
            retracted_ft = c_retracted_ft
            earliest_safe_full_ft = c_earliest_safe_full_ft
        else:
            case_predictor = predictor_for_authority_case(predictor, case)
            retracted_ft = reference_predict_apogee(
                case_predictor,
                case.altitude_m,
                case.velocity_mps,
                0.0,
                0.0,
            ) * METERS_TO_FEET
            earliest_safe_full_ft = reference_predict_apogee(
                case_predictor,
                case.altitude_m,
                case.velocity_mps,
                0.0,
                1.0,
            ) * METERS_TO_FEET
        passed = (
            retracted_ft > RETRACTED_MINIMUM_FT
            and earliest_safe_full_ft < FULL_MAXIMUM_FT
        )
        passes += int(passed)
        rows.append(
            {
                "altitude_m": case.altitude_m,
                "velocity_mps": case.velocity_mps,
                "passive_apogee_ft": case.passive_apogee_ft,
                "retracted_apogee_ft": retracted_ft,
                "earliest_safe_full_apogee_ft": earliest_safe_full_ft,
                "controller_retracted_prediction_ft": c_retracted_ft,
                "controller_full_prediction_ft": c_earliest_safe_full_ft,
                "mass_scale": case.mass_scale,
                "baseline_cda_scale": case.baseline_cda_scale,
                "incremental_cda_scale": case.incremental_cda_scale,
                "density_scale": case.density_scale,
                "delay_scale": case.delay_scale,
                "open_rate_scale": case.open_rate_scale,
                "close_rate_scale": case.close_rate_scale,
                "passed": passed,
            }
        )
    pass_rate = passes / len(cases) if cases else 0.0
    return {
        "evaluation_scope": "plant-uncertainty earliest-safe coast sweep anchored to the 3379 ft passive reference; controller predictions are retained separately",
        "case_count": len(cases),
        "pass_count": passes,
        "pass_rate": pass_rate,
        "required_pass_rate": AUTHORITY_REQUIRED_RATE,
        "retracted_threshold_ft": RETRACTED_MINIMUM_FT,
        "earliest_safe_full_threshold_ft": FULL_MAXIMUM_FT,
        "minimum_retracted_apogee_ft": min(
            row["retracted_apogee_ft"] for row in rows
        ) if rows else math.nan,
        "maximum_full_apogee_ft": max(
            row["earliest_safe_full_apogee_ft"] for row in rows
        ) if rows else math.nan,
        "passed": pass_rate >= AUTHORITY_REQUIRED_RATE,
        "cases": rows,
    }


def build_calibration_cases(case_count: int, seed: int) -> list[CoastCase]:
    randomizer = random.Random(seed)
    stations = (0.0, 0.25, 0.5, 0.75, 1.0)
    cases: list[CoastCase] = []
    for index in range(case_count):
        first_action = index % 4 == 0
        altitude_m = (
            randomizer.uniform(60.96, 220.0)
            if first_action
            else randomizer.uniform(60.96, 700.0)
        )
        velocity_mps = (
            randomizer.uniform(90.0, 190.0)
            if first_action
            else randomizer.uniform(20.0, 190.0)
        )
        cases.append(
            CoastCase(
                altitude_m,
                velocity_mps,
                stations[index % len(stations)],
                first_action,
            )
        )
    return cases


def build_rocketpy_fixed_deployment_cases(
    config: dict[str, Any], case_count: int, seed: int
) -> list[CoastCase]:
    """Collect disjoint coast states from five fixed-station RocketPy flights."""
    pooled: list[CoastCase] = []
    minimum_altitude_m = float(config["controller"].get(
        "minimum_deploy_altitude_m", 200.0 * FEET_TO_METERS
    ))
    minimum_time_s = float(config["controller"]["minimum_boost_time_s"])
    for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
        flight, log = run_fixed_deployment(config, fraction)
        apogee_m = flight_apogee_agl_m(flight, config)
        eligible = [
            row
            for row in log
            if row["time_s"] >= minimum_time_s
            and row["altitude_m"] >= minimum_altitude_m
            and row["vertical_velocity_mps"] > 5.0
            and abs(row["deployment_fraction"] - fraction) <= 0.002
        ]
        if not eligible:
            raise RuntimeError(
                f"fixed-deployment RocketPy flight produced no settled {fraction:.2f} samples"
            )
        first_time_s = eligible[0]["time_s"]
        pooled.extend(
            CoastCase(
                altitude_m=row["altitude_m"],
                velocity_mps=row["vertical_velocity_mps"],
                deployment_fraction=fraction,
                first_action=row["time_s"] <= first_time_s + 0.50,
                reference_apogee_m=apogee_m,
            )
            for row in eligible
        )

    if len(pooled) < case_count:
        raise RuntimeError(
            f"only {len(pooled)} fixed-deployment RocketPy states are available; "
            f"{case_count} requested"
        )
    randomizer = random.Random(seed)
    first_action_cases = [case for case in pooled if case.first_action]
    later_cases = [case for case in pooled if not case.first_action]
    randomizer.shuffle(first_action_cases)
    randomizer.shuffle(later_cases)
    first_action_count = min(
        len(first_action_cases), max(1, (case_count + 5) // 6)
    )
    later_count = case_count - first_action_count
    if len(later_cases) < later_count:
        raise RuntimeError("fixed-deployment RocketPy pool lacks later coast states")

    selected_first = first_action_cases[:first_action_count]
    selected_later = later_cases[:later_count]
    selected: list[CoastCase] = []
    while selected_first or selected_later:
        for _ in range(5):
            if selected_later:
                selected.append(selected_later.pop())
        if selected_first:
            selected.append(selected_first.pop())
    return selected[:case_count]


def calibrate_fixed_deployment(
    bridge: ControllerBridge,
    predictor: dict[str, Any],
    cases: list[CoastCase],
) -> CalibrationMetrics:
    errors_ft: list[float] = []
    first_action_errors_ft: list[float] = []
    for case in cases:
        c_prediction_m = bridge.predict(
            case.altitude_m,
            case.velocity_mps,
            case.deployment_fraction,
            case.deployment_fraction,
        )
        reference_m = (
            case.reference_apogee_m
            if case.reference_apogee_m is not None
            else reference_predict_apogee(
                predictor,
                case.altitude_m,
                case.velocity_mps,
                case.deployment_fraction,
                case.deployment_fraction,
            )
        )
        error_ft = (c_prediction_m - reference_m) * METERS_TO_FEET
        errors_ft.append(error_ft)
        if case.first_action:
            first_action_errors_ft.append(error_ft)

    rmse_ft = math.sqrt(statistics.fmean(error * error for error in errors_ft))
    first_action_bias_ft = statistics.fmean(first_action_errors_ft)
    p95_ft = percentile([abs(error) for error in errors_ft], 0.95)
    maximum_ft = max(abs(error) for error in errors_ft)
    passed = (
        rmse_ft <= CALIBRATION_RMSE_LIMIT_FT
        and abs(first_action_bias_ft) <= CALIBRATION_FIRST_ACTION_BIAS_LIMIT_FT
        and p95_ft <= CALIBRATION_P95_LIMIT_FT
    )
    return CalibrationMetrics(
        len(cases), rmse_ft, first_action_bias_ft, p95_ft, maximum_ft, passed
    )


def solve_predictive_fraction(
    bridge: ControllerBridge,
    case: CoastCase,
    target_apogee_m: float,
) -> tuple[float, bool, bool]:
    closed = bridge.predict(case.altitude_m, case.velocity_mps, 0.0, 0.0)
    full = bridge.predict(case.altitude_m, case.velocity_mps, 0.0, 1.0)
    if not math.isfinite(closed) or not math.isfinite(full) or closed < full:
        return 0.0, False, False
    if target_apogee_m >= closed:
        return 0.0, True, True
    if target_apogee_m <= full:
        return 1.0, True, True
    lower = 0.0
    upper = 1.0
    for _ in range(8):
        middle = 0.5 * (lower + upper)
        prediction = bridge.predict(
            case.altitude_m, case.velocity_mps, 0.0, middle
        )
        if prediction > target_apogee_m:
            lower = middle
        else:
            upper = middle
    return 0.5 * (lower + upper), True, False


def evaluate_controllers(
    bridge: ControllerBridge,
    config: dict[str, Any],
    cases: list[CoastCase],
) -> dict[str, Any]:
    predictor = config["predictor"]
    controller = config["controller"]
    target_m = float(controller["target_apogee_ft"]) * FEET_TO_METERS
    full_error_m = 250.0 * FEET_TO_METERS
    proportional_errors_ft: list[float] = []
    predictive_errors_ft: list[float] = []
    predictive_fallbacks = 0
    predictive_saturations = 0
    mission_tolerance_ft = float(controller["mission_tolerance_ft"])

    for case in cases:
        case_predictor = predictor_for_authority_case(predictor, case)
        closed_m = bridge.predict(case.altitude_m, case.velocity_mps, 0.0, 0.0)
        proportional_fraction = max(
            0.0, min(1.0, (closed_m - target_m) / full_error_m)
        )
        proportional_apogee_m = reference_predict_apogee(
            case_predictor,
            case.altitude_m,
            case.velocity_mps,
            0.0,
            proportional_fraction,
        )
        proportional_errors_ft.append(
            (proportional_apogee_m - target_m) * METERS_TO_FEET
        )

        predictive_fraction, valid, saturated = solve_predictive_fraction(
            bridge, case, target_m
        )
        if not valid:
            predictive_fallbacks += 1
            predictive_fraction = proportional_fraction
        predictive_saturations += int(saturated)
        predictive_apogee_m = reference_predict_apogee(
            case_predictor,
            case.altitude_m,
            case.velocity_mps,
            0.0,
            predictive_fraction,
        )
        predictive_errors_ft.append(
            (predictive_apogee_m - target_m) * METERS_TO_FEET
        )

    def summarize(errors: list[float]) -> dict[str, Any]:
        hit_rate = sum(
            abs(value) <= mission_tolerance_ft for value in errors
        ) / len(errors)
        p95_absolute_error_ft = percentile(
            [abs(value) for value in errors], 0.95
        )
        maximum_absolute_error_ft = max(abs(value) for value in errors)
        return {
            "mean_error_ft": statistics.fmean(errors),
            "mean_absolute_error_ft": statistics.fmean(abs(value) for value in errors),
            "mission_band_hit_rate": hit_rate,
            "p95_absolute_error_ft": p95_absolute_error_ft,
            "maximum_absolute_error_ft": maximum_absolute_error_ft,
            "provisional_robustness_gate_passed": (
                hit_rate >= 0.96
                and p95_absolute_error_ft <= 100.0
                and maximum_absolute_error_ft <= 200.0
            ),
        }

    return {
        "evaluation_scope": "one-shot earliest-safe coast controller comparison; use RocketPy Monte Carlo for closed-loop acceptance",
        "case_count": len(cases),
        "solver_period_s": float(controller["predictive_update_period_s"]),
        "bisection_iterations": 8,
        "provisional_robustness_gates": {
            "required_mission_band_hit_rate": 0.96,
            "maximum_p95_absolute_error_ft": 100.0,
            "maximum_worst_absolute_error_ft": 200.0,
        },
        "proportional": summarize(proportional_errors_ft),
        "predictive": summarize(predictive_errors_ft),
        "predictive_fallback_count": predictive_fallbacks,
        "predictive_authority_saturation_count": predictive_saturations,
    }


def fit_passive_drag(
    config: dict[str, Any], target_ft: float, iterations: int = 12
) -> dict[str, float]:
    """Fit only equal power-on/off Cd; never change plant mass or motor."""
    lower_cd = 0.45
    upper_cd = 0.60
    candidate_cd = lower_cd
    apogee_ft = math.nan
    best_cd = candidate_cd
    best_apogee_ft = math.nan
    best_absolute_residual_ft = math.inf
    for _ in range(iterations):
        candidate_cd = 0.5 * (lower_cd + upper_cd)
        candidate = copy.deepcopy(config)
        candidate["rocket"]["power_on_drag_coefficient"] = candidate_cd
        candidate["rocket"]["power_off_drag_coefficient"] = candidate_cd
        apogee_ft = flight_apogee_agl_m(
            run_passive(candidate), candidate
        ) * METERS_TO_FEET
        absolute_residual_ft = abs(apogee_ft - target_ft)
        if absolute_residual_ft < best_absolute_residual_ft:
            best_absolute_residual_ft = absolute_residual_ft
            best_cd = candidate_cd
            best_apogee_ft = apogee_ft
        if apogee_ft > target_ft:
            lower_cd = candidate_cd
        else:
            upper_cd = candidate_cd
    return {
        "fitted_drag_coefficient": best_cd,
        "apogee_ft": best_apogee_ft,
        "target_ft": target_ft,
        "residual_ft": best_apogee_ft - target_ft,
        "iterations": iterations,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=VARIABLE_CONFIG_PATH)
    parser.add_argument("--tuning-cases", type=int, default=200)
    parser.add_argument("--held-out-cases", type=int, default=500)
    parser.add_argument("--authority-cases", type=int, default=50)
    parser.add_argument("--seed", type=int, default=20260719)
    parser.add_argument("--output", type=Path, default=DEFAULT_RESULT_PATH)
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--refit-passive-drag", action="store_true")
    parser.add_argument("--skip-passive-verification", action="store_true")
    parser.add_argument(
        "--synthetic-calibration",
        action="store_true",
        help="Use fine-step model consistency instead of five RocketPy fixed-station flights.",
    )
    args = parser.parse_args()

    if args.quick:
        args.tuning_cases = min(args.tuning_cases, 12)
        args.held_out_cases = min(args.held_out_cases, 20)
        args.authority_cases = min(args.authority_cases, 25)
    if min(args.tuning_cases, args.held_out_cases, args.authority_cases) <= 0:
        raise SystemExit("case counts must be positive")
    if not args.bridge.exists():
        raise SystemExit(f"bridge is missing: {args.bridge}")

    config = load_config(args.config)
    passive: dict[str, Any]
    if args.refit_passive_drag:
        passive = fit_passive_drag(
            config, float(config["predictor"]["passive_reference_apogee_ft"])
        )
    elif args.skip_passive_verification:
        passive = {"skipped": True}
    else:
        apogee_ft = flight_apogee_agl_m(
            run_passive(config), config
        ) * METERS_TO_FEET
        target_ft = float(config["predictor"]["passive_reference_apogee_ft"])
        passive = {
            "configured_drag_coefficient": config["rocket"]["power_off_drag_coefficient"],
            "apogee_ft": apogee_ft,
            "target_ft": target_ft,
            "residual_ft": apogee_ft - target_ft,
            "passed": abs(apogee_ft - target_ft) <= 1.0,
        }

    result: dict[str, Any] = {
        "profile": str(args.config.resolve()),
        "seed": args.seed,
        "passive_drag_fit_verification": passive,
        "configuration": {
            "sampling_rate_hz": config["airbrakes"]["sampling_rate_hz"],
            "plant_dry_mass_kg": config["rocket"]["dry_mass_kg"],
            "motor": config["motor"]["name"],
            "calibration_version": config["predictor"]["calibration_version"],
        },
    }

    bridge = make_bridge(config, args.bridge)
    try:
        authority_cases = build_authority_cases(
            config["predictor"], args.authority_cases, args.seed
        )
        authority = run_authority_sweep(
            bridge, authority_cases, config["predictor"]
        )
        result["authority_sweep"] = authority

        # Hard stop: calibration/controller scores are meaningless if the
        # provisional plant cannot bracket the required apogee authority.
        if not authority["passed"]:
            result["status"] = "INSUFFICIENT_CONTROL_AUTHORITY"
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
            print(json.dumps(result, indent=2))
            return 3

        if args.synthetic_calibration:
            tuning_cases = build_calibration_cases(args.tuning_cases, args.seed + 1)
            held_out_cases = build_calibration_cases(args.held_out_cases, args.seed + 2)
            calibration_truth = "fine_step_vertical_model_consistency"
        else:
            fixed_cases = build_rocketpy_fixed_deployment_cases(
                config,
                args.tuning_cases + args.held_out_cases,
                args.seed + 1,
            )
            tuning_cases = fixed_cases[: args.tuning_cases]
            held_out_cases = fixed_cases[args.tuning_cases :]
            calibration_truth = "five_fixed_deployment_rocketpy_flights"
        tuning = calibrate_fixed_deployment(
            bridge, config["predictor"], tuning_cases
        )
        held_out = calibrate_fixed_deployment(
            bridge, config["predictor"], held_out_cases
        )
        result["fixed_deployment_calibration"] = {
            "truth_source": calibration_truth,
            "gates": {
                "rmse_limit_ft": CALIBRATION_RMSE_LIMIT_FT,
                "first_action_bias_limit_ft": CALIBRATION_FIRST_ACTION_BIAS_LIMIT_FT,
                "p95_absolute_error_limit_ft": CALIBRATION_P95_LIMIT_FT,
            },
            "tuning": asdict(tuning),
            "held_out": asdict(held_out),
        }
        result["controller_evaluation"] = evaluate_controllers(
            bridge, config, authority_cases
        )
        passive_pass = passive.get("passed", True)
        result["status"] = (
            "PASS"
            if passive_pass and tuning.passed and held_out.passed
            else "FAIL"
        )
    finally:
        bridge.close()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
