"""RocketPy physics simulation connected to the AMBAR C++ flight computer.

RocketPy owns the motor, changing mass, atmosphere, aerodynamics, and 6-DOF
trajectory. A persistent line-oriented bridge feeds virtual vertical IMU and
barometer measurements into the same C++ estimator/controller used by the
desktop sandboxes. The returned deployment command drives RocketPy airbrakes.

Architecture connections:
- ambar_reference_config.json supplies the versioned vehicle/model inputs.
- j420r.eng supplies the motor thrust curve.
- sim/controller_bridge.cpp exposes the shared C++ flight logic as a process.
- scripts/run_rocketpy_sim.ps1 builds the bridge and launches this module.
- build/rocketpy-last-run.json is the detailed machine-readable output.

Use this file for trajectory/controller coupling and apogee studies. Use the
smaller C++ sandboxes for quick logic and fault-regression checks.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import subprocess
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from rocketpy import Environment, Flight, Rocket, SolidMotor


ROOT = Path(__file__).resolve().parents[2]
MODEL_DIR = Path(__file__).resolve().parent
CONFIG_PATH = MODEL_DIR / "ambar_reference_config.json"
RESULT_PATH = ROOT / "build" / "rocketpy-last-run.json"
METERS_TO_FEET = 3.280839895
FEET_TO_METERS = 1.0 / METERS_TO_FEET
MPS_TO_FPS = METERS_TO_FEET


@dataclass
class BridgeState:
    deploy_fraction: float = 0.0
    inhibit: bool = True
    inhibit_flags: int = 0
    altitude_m: float = 0.0
    velocity_mps: float = 0.0
    predicted_apogee_m: float = 0.0
    healthy: bool = True
    phase: str = "PadIdle"


class SensorModel:
    """Deterministic provisional sensor errors applied before the C++ bridge."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.values = config["sensor_model"]
        self.random = random.Random(self.values["random_seed"])

    @staticmethod
    def _quantize(value: float, resolution: float) -> float:
        return round(value / resolution) * resolution if resolution > 0 else value

    def acceleration(self, truth_mps2: float) -> float:
        value = (
            truth_mps2
            + self.values["accelerometer_bias_mps2"]
            + self.random.gauss(0.0, self.values["accelerometer_noise_std_dev_mps2"])
        )
        measurement_range = self.values["accelerometer_range_mps2"]
        value = max(-measurement_range, min(measurement_range, value))
        return self._quantize(value, self.values["accelerometer_resolution_mps2"])

    def altitude(self, truth_m: float) -> float:
        value = (
            truth_m
            + self.values["barometer_bias_m"]
            + self.random.gauss(0.0, self.values["barometer_noise_std_dev_m"])
        )
        return self._quantize(value, self.values["barometer_resolution_m"])


class ControllerBridge:
    """Own the persistent C++ child process and its line protocol."""

    def __init__(
        self,
        executable: Path,
        minimum_boost_time_s: float,
        target_apogee_m: float,
        target_tolerance_m: float,
    ) -> None:
        self.process = subprocess.Popen(
            [
                str(executable),
                "--minimum-boost-time",
                f"{minimum_boost_time_s:.6f}",
                "--target-apogee-m",
                f"{target_apogee_m:.6f}",
                "--target-tolerance-m",
                f"{target_tolerance_m:.6f}",
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            bufsize=1,
        )
        self.state = self._exchange("RESET 0")

    def _exchange(self, command: str) -> BridgeState:
        if self.process.stdin is None or self.process.stdout is None:
            raise RuntimeError("Controller bridge pipes are unavailable.")
        self.process.stdin.write(command + "\n")
        self.process.stdin.flush()
        response = self.process.stdout.readline().strip()
        if not response.startswith("STATE "):
            raise RuntimeError(f"Controller bridge returned: {response or '<no output>'}")
        values = response.split(maxsplit=8)
        if len(values) != 9:
            raise RuntimeError(f"Controller bridge state is malformed: {response}")
        self.state = BridgeState(
            deploy_fraction=float(values[1]),
            inhibit=values[2] == "1",
            inhibit_flags=int(values[3]),
            altitude_m=float(values[4]),
            velocity_mps=float(values[5]),
            predicted_apogee_m=float(values[6]),
            healthy=values[7] == "1",
            phase=values[8],
        )
        return self.state

    def step(
        self,
        timestamp_s: float,
        acceleration_mps2: float,
        altitude_m: float,
        altitude_std_dev_m: float,
        use_barometer: bool,
    ) -> BridgeState:
        return self._exchange(
            "STEP "
            f"{timestamp_s:.9f} {acceleration_mps2:.9f} {altitude_m:.9f} "
            f"{altitude_std_dev_m:.9f} {1 if use_barometer else 0}"
        )

    def close(self) -> None:
        if self.process.poll() is None and self.process.stdin is not None:
            try:
                self.process.stdin.write("QUIT\n")
                self.process.stdin.flush()
                self.process.wait(timeout=2)
            except (BrokenPipeError, subprocess.TimeoutExpired):
                self.process.kill()
        if self.process.returncode not in (None, 0):
            error = self.process.stderr.read() if self.process.stderr else ""
            raise RuntimeError(f"Controller bridge exited with {self.process.returncode}: {error}")


def deep_merge(base: dict[str, Any], overrides: dict[str, Any]) -> dict[str, Any]:
    """Merge a validated override tree without mutating the baseline config."""
    merged = dict(base)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def load_config(overrides_path: Path | None = None) -> dict[str, Any]:
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    if overrides_path is None:
        return config
    overrides = json.loads(overrides_path.read_text(encoding="utf-8"))
    if not isinstance(overrides, dict):
        raise ValueError("Simulation overrides must be a JSON object.")
    config = deep_merge(config, overrides)
    config["model_status"] += " with explicit experimental overrides"
    return config


def build_environment(config: dict[str, Any]) -> Environment:
    """Create the atmosphere and launch-site model used by every flight case."""
    values = config["environment"]
    environment = Environment(
        latitude=values["latitude_deg"],
        longitude=values["longitude_deg"],
        elevation=values["elevation_m"],
    )
    # RocketPy's custom atmosphere preserves standard pressure/temperature when
    # those fields are None while allowing the report-backed wind to be applied.
    direction_rad = math.radians(values["wind_from_direction_deg"])
    wind_u = -values["wind_speed_mps"] * math.sin(direction_rad)
    wind_v = -values["wind_speed_mps"] * math.cos(direction_rad)
    environment.set_atmospheric_model(
        type="custom_atmosphere",
        pressure=None,
        temperature=None,
        wind_u=wind_u,
        wind_v=wind_v,
    )
    return environment


def build_motor(config: dict[str, Any]) -> SolidMotor:
    """Build the selected motor from the versioned thrust curve and geometry."""
    values = config["motor"]
    # Grain geometry is a documented reference approximation. The imported
    # certified thrust curve and total propellant mass control flight impulse.
    return SolidMotor(
        thrust_source=str(MODEL_DIR / values["thrust_curve"]),
        dry_mass=values["dry_mass_kg"],
        dry_inertia=(0.02, 0.02, 0.0002),
        nozzle_radius=0.015,
        grain_number=4,
        grain_density=1368,
        grain_outer_radius=0.018,
        grain_initial_inner_radius=0.009,
        grain_initial_height=0.09,
        grain_separation=0.003,
        grains_center_of_mass_position=0.1685,
        center_of_dry_mass_position=0.1685,
        nozzle_position=0.0,
        burn_time=values["burn_time_s"],
        throat_radius=0.007,
        coordinate_system_orientation="nozzle_to_combustion_chamber",
    )


def build_rocket(config: dict[str, Any], motor: SolidMotor) -> Rocket:
    """Assemble the passive rocket model before optional airbrakes are added."""
    values = config["rocket"]
    rocket = Rocket(
        radius=values["radius_m"],
        mass=values["dry_mass_kg"],
        inertia=tuple(values["dry_inertia_kg_m2"]),
        power_off_drag=values["power_off_drag_coefficient"],
        power_on_drag=values["power_on_drag_coefficient"],
        center_of_mass_without_motor=values["center_of_mass_without_motor_m"],
        coordinate_system_orientation="tail_to_nose",
    )
    rocket.add_motor(motor, position=config["motor"]["position_m"])
    rocket.add_nose(
        length=values["nose_length_m"],
        kind="von karman",
        position=values["nose_position_m"],
    )
    rocket.add_trapezoidal_fins(
        n=values["fin_count"],
        root_chord=values["fin_root_chord_m"],
        tip_chord=values["fin_tip_chord_m"],
        span=values["fin_span_m"],
        position=values["fin_position_m"],
    )
    rocket.set_rail_buttons(
        upper_button_position=values["upper_rail_button_position_m"],
        lower_button_position=values["lower_rail_button_position_m"],
    )
    return rocket


def run_passive(config: dict[str, Any]) -> Flight:
    """Generate the no-airbrake reference trajectory used for comparison."""
    environment = build_environment(config)
    rocket = build_rocket(config, build_motor(config))
    values = config["environment"]
    return Flight(
        rocket=rocket,
        environment=environment,
        rail_length=values["rail_length_m"],
        inclination=values["inclination_deg"],
        heading=values["heading_deg"],
        terminate_on_apogee=True,
        max_time=40,
        max_time_step=0.02,
        verbose=False,
    )


def run_closed_loop(
    config: dict[str, Any],
    bridge_path: Path,
    observation_end_time_s: float,
) -> tuple[Flight, list[dict[str, Any]]]:
    """Run RocketPy while the persistent C++ controller commands airbrakes.

    The closed-loop run continues briefly after apogee. This does not model the
    recovery system; it exists so the controller's Recovery transition and the
    actuator's commanded retraction are visible in the time-history output.
    """
    environment = build_environment(config)
    rocket = build_rocket(config, build_motor(config))
    airbrake_config = config["airbrakes"]
    minimum_boost_time_s = (
        config["motor"]["burn_time_s"]
        + airbrake_config["post_burn_enable_margin_s"]
    )
    requirements = config["requirements"]
    bridge = ControllerBridge(
        bridge_path,
        minimum_boost_time_s,
        requirements["target_apogee_ft"] * FEET_TO_METERS,
        requirements["target_tolerance_ft"] * FEET_TO_METERS,
    )
    log: list[dict[str, Any]] = []
    previous_time: float | None = None
    previous_velocity_vector = (0.0, 0.0, 0.0)
    next_barometer_time = 1.0 / airbrake_config["barometer_rate_hz"]
    actual_deployment = 0.0
    sensor_model = SensorModel(config)
    truth_history: deque[tuple[float, float, float]] = deque()

    def controller(
        time: float,
        sampling_rate: float,
        state: list[float],
        state_history: list[list[float]],
        observed_variables: list[Any],
        air_brakes: Any,
        sensors: list[Any],
        controller_environment: Environment,
    ) -> tuple[float, float, float, float, str, bool] | None:
        del state_history, observed_variables, sensors
        nonlocal previous_time, previous_velocity_vector, next_barometer_time, actual_deployment

        if previous_time is not None and time <= previous_time + 1e-9:
            air_brakes.deployment_level = actual_deployment
            return None

        velocity_vector_mps = (float(state[3]), float(state[4]), float(state[5]))
        velocity_mps = velocity_vector_mps[2]
        speed_mps = math.sqrt(sum(component * component for component in velocity_vector_mps))
        altitude_m = max(0.0, float(state[2]) - controller_environment.elevation)
        if previous_time is None:
            previous_time = time
            previous_velocity_vector = velocity_vector_mps
            air_brakes.deployment_level = 0.0
            truth_history.append((time, 0.0, altitude_m))
            log.append(
                {
                    "time_s": time,
                    "truth_altitude_m": altitude_m,
                    "truth_velocity_mps": velocity_mps,
                    "truth_speed_mps": speed_mps,
                    "truth_acceleration_mps2": 0.0,
                    "truth_acceleration_magnitude_mps2": 0.0,
                    "measured_acceleration_mps2": 0.0,
                    "measured_altitude_m": None,
                    "barometer_sample": False,
                    "measurement_source_time_s": time,
                    "estimated_altitude_m": bridge.state.altitude_m,
                    "estimated_velocity_mps": bridge.state.velocity_mps,
                    "predicted_apogee_m": bridge.state.predicted_apogee_m,
                    "command_fraction": 0.0,
                    "desired_deployment_fraction": 0.0,
                    "actual_deployment_fraction": 0.0,
                    "inhibit": bridge.state.inhibit,
                    "inhibit_flags": bridge.state.inhibit_flags,
                    "phase": bridge.state.phase,
                    "healthy": bridge.state.healthy,
                }
            )
            return None
        else:
            dt_s = time - previous_time
            acceleration_vector_mps2 = tuple(
                (current - previous) / dt_s
                for current, previous in zip(velocity_vector_mps, previous_velocity_vector)
            )
            acceleration_mps2 = acceleration_vector_mps2[2]
            acceleration_magnitude_mps2 = math.sqrt(
                sum(component * component for component in acceleration_vector_mps2)
            )

        truth_history.append((time, acceleration_mps2, altitude_m))
        delayed_time = time - config["sensor_model"]["latency_s"]
        while len(truth_history) > 1 and truth_history[1][0] <= delayed_time:
            truth_history.popleft()
        measurement_source_time_s, delayed_acceleration_mps2, delayed_altitude_m = truth_history[0]
        measured_acceleration_mps2 = sensor_model.acceleration(
            delayed_acceleration_mps2
        )

        use_barometer = time + 1e-9 >= next_barometer_time
        if use_barometer:
            next_barometer_time += 1.0 / airbrake_config["barometer_rate_hz"]
        measured_altitude_m = (
            sensor_model.altitude(delayed_altitude_m) if use_barometer else None
        )

        bridge_state = bridge.step(
            timestamp_s=time,
            acceleration_mps2=measured_acceleration_mps2,
            altitude_m=measured_altitude_m if measured_altitude_m is not None else 0.0,
            altitude_std_dev_m=config["sensor_model"]["barometer_measurement_std_dev_m"],
            use_barometer=use_barometer,
        )
        desired_deployment = 0.0 if bridge_state.inhibit else bridge_state.deploy_fraction
        maximum_change = airbrake_config["maximum_rate_fraction_per_s"] * dt_s
        actual_deployment += max(
            -maximum_change,
            min(maximum_change, desired_deployment - actual_deployment),
        )
        actual_deployment = max(0.0, min(1.0, actual_deployment))
        air_brakes.deployment_level = actual_deployment

        log.append(
            {
                "time_s": time,
                "truth_altitude_m": altitude_m,
                "truth_velocity_mps": velocity_mps,
                "truth_speed_mps": speed_mps,
                "truth_acceleration_mps2": acceleration_mps2,
                "truth_acceleration_magnitude_mps2": acceleration_magnitude_mps2,
                "measured_acceleration_mps2": measured_acceleration_mps2,
                "measured_altitude_m": measured_altitude_m,
                "barometer_sample": use_barometer,
                "measurement_source_time_s": measurement_source_time_s,
                "estimated_altitude_m": bridge_state.altitude_m,
                "estimated_velocity_mps": bridge_state.velocity_mps,
                "predicted_apogee_m": bridge_state.predicted_apogee_m,
                "command_fraction": bridge_state.deploy_fraction,
                "desired_deployment_fraction": desired_deployment,
                "actual_deployment_fraction": actual_deployment,
                "inhibit": bridge_state.inhibit,
                "inhibit_flags": bridge_state.inhibit_flags,
                "phase": bridge_state.phase,
                "healthy": bridge_state.healthy,
            }
        )
        previous_time = time
        previous_velocity_vector = velocity_vector_mps
        return (
            time,
            actual_deployment,
            bridge_state.deploy_fraction,
            bridge_state.predicted_apogee_m,
            bridge_state.phase,
            bridge_state.healthy,
        )

    rocket.add_air_brakes(
        drag_coefficient_curve=lambda deployment, mach: (
            airbrake_config["drag_coefficient_at_full_deployment"]
            * deployment
            * (1.0 + 0.10 * min(max(mach, 0.0), 1.0))
        ),
        controller_function=controller,
        sampling_rate=airbrake_config["sampling_rate_hz"],
        clamp=True,
        override_rocket_drag=False,
        name="AMBAR Airbrakes",
    )

    values = config["environment"]
    try:
        flight = Flight(
            rocket=rocket,
            environment=environment,
            rail_length=values["rail_length_m"],
            inclination=values["inclination_deg"],
            heading=values["heading_deg"],
            terminate_on_apogee=False,
            max_time=observation_end_time_s,
            max_time_step=0.01,
            verbose=False,
        )
    finally:
        bridge.close()
    return flight, log


def derive_phase_transitions(log: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Compress the sample-by-sample phase field into a readable timeline."""
    transitions: list[dict[str, Any]] = []
    for entry in log:
        if not transitions or transitions[-1]["phase"] != entry["phase"]:
            transitions.append(
                {
                    "time_s": entry["time_s"],
                    "phase": entry["phase"],
                    "altitude_m": entry["truth_altitude_m"],
                    "vertical_velocity_mps": entry["truth_velocity_mps"],
                    "deployment_fraction": entry["actual_deployment_fraction"],
                }
            )
    return transitions


def validate_controller_log(log: list[dict[str, Any]]) -> list[str]:
    """Return human-readable integrity errors for the exported time history."""
    if len(log) < 2:
        return ["controller log contains fewer than two samples"]

    errors: list[str] = []
    required_numeric_fields = (
        "time_s",
        "truth_altitude_m",
        "truth_velocity_mps",
        "truth_speed_mps",
        "truth_acceleration_mps2",
        "truth_acceleration_magnitude_mps2",
        "measured_acceleration_mps2",
        "measurement_source_time_s",
        "estimated_altitude_m",
        "estimated_velocity_mps",
        "predicted_apogee_m",
        "command_fraction",
        "desired_deployment_fraction",
        "actual_deployment_fraction",
    )
    previous_time = -math.inf
    for index, entry in enumerate(log):
        missing = [field for field in required_numeric_fields if field not in entry]
        if missing:
            errors.append(f"sample {index} is missing {', '.join(missing)}")
            continue
        if not all(math.isfinite(float(entry[field])) for field in required_numeric_fields):
            errors.append(f"sample {index} contains a non-finite numeric value")
        if entry["time_s"] <= previous_time:
            errors.append(f"sample {index} timestamp is not strictly increasing")
        previous_time = entry["time_s"]
        if entry["measurement_source_time_s"] > entry["time_s"] + 1.0e-9:
            errors.append(f"sample {index} uses a measurement from the future")
        if entry["barometer_sample"]:
            measured_altitude = entry["measured_altitude_m"]
            if measured_altitude is None or not math.isfinite(float(measured_altitude)):
                errors.append(f"sample {index} marks an invalid barometer sample")
        elif entry["measured_altitude_m"] is not None:
            errors.append(f"sample {index} reports a barometer value between samples")
        if not 0.0 <= entry["command_fraction"] <= 1.0:
            errors.append(f"sample {index} command is outside [0, 1]")
        if not 0.0 <= entry["actual_deployment_fraction"] <= 1.0:
            errors.append(f"sample {index} deployment is outside [0, 1]")
        if len(errors) >= 20:
            errors.append("additional log errors omitted")
            break
    return errors


def print_case(number: int, name: str, condition: str, rule: str, passed: bool, result: str, measurements: dict[str, str], note: str = "") -> None:
    print(f"\nTEST CASE {number}: {name}")
    print(f"Condition being tested: {condition}")
    print(f"Pass rule: {rule}")
    print(f"Result: {'PASS' if passed else 'FAIL'} - {result}")
    print("Measurements:")
    for key, value in measurements.items():
        print(f"  {key}: {value}")
    if note:
        print(f"Target note: {note}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge", type=Path, required=True)
    parser.add_argument(
        "--overrides",
        type=Path,
        help="Validated JSON override file generated by the local simulation console.",
    )
    args = parser.parse_args()
    config = load_config(args.overrides)
    requirements = config["requirements"]

    if not args.bridge.exists():
        raise SystemExit(f"Controller bridge is missing: {args.bridge}")

    print("AMBAR RocketPy physics sandbox")
    print("Purpose: run RocketPy trajectory physics while the existing C++ flight computer commands the virtual airbrakes.")
    print(f"RocketPy version: 1.12.1")
    print(f"Model status: {config['model_status']}")
    print(f"Input mode: {'EXPERIMENTAL OVERRIDES' if args.overrides else 'REPORT BASELINE'}")
    print("Applied inputs:")
    print(f"  target apogee: {config['requirements']['target_apogee_ft']:.1f} ft")
    print(f"  target tolerance: +/-{config['requirements']['target_tolerance_ft']:.1f} ft")
    print(f"  rail length: {config['environment']['rail_length_m'] * METERS_TO_FEET:.2f} ft")
    print(f"  launch angle from vertical: {90.0 - config['environment']['inclination_deg']:.1f} deg")
    print(f"  constant wind: {config['environment']['wind_speed_mps']:.2f} m/s from {config['environment']['wind_from_direction_deg']:.0f} deg")
    print(f"  dry mass: {config['rocket']['dry_mass_kg'] * 2.2046226218:.2f} lb")
    print(f"  power-on drag coefficient: {config['rocket']['power_on_drag_coefficient']:.3f}")
    print(f"  power-off drag coefficient: {config['rocket']['power_off_drag_coefficient']:.3f}")
    print(f"  full-deployment airbrake drag coefficient: {config['airbrakes']['drag_coefficient_at_full_deployment']:.3f}")
    print(f"  maximum deployment rate: {config['airbrakes']['maximum_rate_fraction_per_s'] * 100.0:.1f} percent/s")
    print(f"  barometer rate: {config['airbrakes']['barometer_rate_hz']:.1f} Hz")
    print(f"  accelerometer bias/noise: {config['sensor_model']['accelerometer_bias_mps2']:.2f} / {config['sensor_model']['accelerometer_noise_std_dev_mps2']:.2f} m/s^2")
    print(f"  barometer bias/noise: {config['sensor_model']['barometer_bias_m'] * METERS_TO_FEET:.2f} / {config['sensor_model']['barometer_noise_std_dev_m'] * METERS_TO_FEET:.2f} ft")
    print(f"  sensor latency: {config['sensor_model']['latency_s'] * 1000.0:.0f} ms")
    print("PASS/FAIL meaning: PASS verifies software coupling and stated reference-model behavior; it does not validate unresolved mass or aerodynamic inputs.")

    passive = run_passive(config)
    post_apogee_observation_s = config["airbrakes"]["post_apogee_observation_s"]
    observation_end_time_s = passive.apogee_time + post_apogee_observation_s
    closed, controller_log = run_closed_loop(
        config,
        args.bridge.resolve(),
        observation_end_time_s,
    )

    passive_apogee_ft = passive.apogee * METERS_TO_FEET
    closed_apogee_ft = closed.apogee * METERS_TO_FEET
    target_ft = requirements["target_apogee_ft"]
    tolerance_ft = requirements["target_tolerance_ft"]
    reference_ft = requirements["m5_openrocket_passive_apogee_ft"]
    passive_difference_percent = 100.0 * (passive_apogee_ft - reference_ft) / reference_ft
    peak_command = max((entry["command_fraction"] for entry in controller_log), default=0.0)
    minimum_command = min((entry["command_fraction"] for entry in controller_log), default=0.0)
    peak_deployment = max((entry["actual_deployment_fraction"] for entry in controller_log), default=0.0)
    minimum_deployment = min(
        (entry["actual_deployment_fraction"] for entry in controller_log),
        default=0.0,
    )
    commands_bounded = all(
        0.0 <= entry["command_fraction"] <= 1.0
        and 0.0 <= entry["actual_deployment_fraction"] <= 1.0
        for entry in controller_log
    )
    controller_healthy = bool(controller_log) and all(entry["healthy"] for entry in controller_log)
    first_command = next(
        (entry for entry in controller_log if entry["command_fraction"] > 0.001),
        None,
    )
    first_deployment = next(
        (entry for entry in controller_log if entry["actual_deployment_fraction"] > 0.001),
        None,
    )
    minimum_deploy_time_s = (
        config["motor"]["burn_time_s"]
        + config["airbrakes"]["post_burn_enable_margin_s"]
    )
    deployment_after_burn = (
        first_command is not None
        and first_command["time_s"] + 1.0e-9 >= minimum_deploy_time_s
    )
    command_phases_valid = all(
        entry["phase"] == "AirbrakeActive"
        for entry in controller_log
        if entry["command_fraction"] > 0.001
    )
    target_error_ft = closed_apogee_ft - target_ft
    apogee_reduction_ft = passive_apogee_ft - closed_apogee_ft
    maximum_altitude_error_m = max(
        (
            abs(entry["estimated_altitude_m"] - entry["truth_altitude_m"])
            for entry in controller_log
        ),
        default=0.0,
    )
    maximum_velocity_error_mps = max(
        (
            abs(entry["estimated_velocity_mps"] - entry["truth_velocity_mps"])
            for entry in controller_log
        ),
        default=0.0,
    )
    phase_transitions = derive_phase_transitions(controller_log)
    phase_names = {transition["phase"] for transition in phase_transitions}
    log_integrity_errors = validate_controller_log(controller_log)
    barometer_sample_count = sum(
        1 for entry in controller_log if entry["barometer_sample"]
    )
    recovery_observed = "Recovery" in phase_names
    final_deployment = (
        controller_log[-1]["actual_deployment_fraction"] if controller_log else 0.0
    )
    retraction_observed = peak_deployment > 0.05 and final_deployment <= 0.02
    time_history_pass = (
        not log_integrity_errors
        and recovery_observed
        and retraction_observed
        and barometer_sample_count > 0
    )
    maximum_speed_mps = max(
        (entry["truth_speed_mps"] for entry in controller_log), default=0.0
    )
    maximum_vertical_acceleration_mps2 = max(
        (abs(entry["truth_acceleration_mps2"]) for entry in controller_log),
        default=0.0,
    )

    passive_pass = abs(passive_difference_percent) <= 10.0
    coupling_pass = (
        controller_healthy
        and commands_bounded
        and apogee_reduction_ft > 50.0
        and deployment_after_burn
        and command_phases_valid
    )
    target_pass = abs(target_error_ft) <= tolerance_ft
    maximum_mach = max(passive.max_mach_number, closed.max_mach_number)
    minimum_rail_exit_fps = min(
        passive.out_of_rail_velocity * MPS_TO_FPS,
        closed.out_of_rail_velocity * MPS_TO_FPS,
    )
    envelope_pass = (
        maximum_mach <= requirements["maximum_mach"]
        and minimum_rail_exit_fps >= requirements["minimum_rail_exit_velocity_fps"]
    )

    print_case(
        1,
        "M5 passive reference",
        "RocketPy runs the June 14 M5 launch conditions and stabilizing-fin geometry with the selected AeroTech J420R and no airbrake deployment.",
        "PASS if RocketPy completes and passive apogee is within 10% of the current 3379 ft M5 OpenRocket value while final mass and drag inputs remain provisional.",
        passive_pass,
        "RocketPy completed and the reference model stayed within the declared OpenRocket comparison band." if passive_pass else "RocketPy completed, but the reference model is outside the declared OpenRocket comparison band.",
        {
            "RocketPy passive apogee": f"{passive_apogee_ft:.0f} ft",
            "M5 OpenRocket apogee": f"{reference_ft:.0f} ft",
            "difference": f"{passive_difference_percent:+.1f}%",
            "maximum Mach": f"{passive.max_mach_number:.3f}",
            "rail exit velocity": f"{passive.out_of_rail_velocity * MPS_TO_FPS:.1f} ft/s",
        },
        "A mismatch is intentionally reported as FAIL. Do not retune placeholder mass or drag solely to force agreement.",
    )
    print_case(
        2,
        "C++ closed-loop airbrakes",
        "RocketPy feeds trajectory-derived IMU/barometer values to the persistent C++ AMBAR flight computer, then applies its bounded command to rate-limited virtual airbrakes.",
        "PASS if the estimator stays healthy, commands remain bounded, deployment starts after motor burnout plus the configured margin, commands occur only in AirbrakeActive, and deployment reduces apogee by more than 50 ft.",
        coupling_pass,
        "The real C++ controller remained healthy and changed the RocketPy trajectory." if coupling_pass else "The C++/RocketPy coupling did not meet the declared behavior check.",
        {
            "closed-loop apogee": f"{closed_apogee_ft:.0f} ft",
            "target error": f"{target_error_ft:+.0f} ft",
            "apogee reduction": f"{apogee_reduction_ft:.0f} ft",
            "C++ command range": (
                f"{minimum_command * 100.0:.1f}% to {peak_command * 100.0:.1f}%"
            ),
            "physical deployment range": (
                f"{minimum_deployment * 100.0:.1f}% to {peak_deployment * 100.0:.1f}%"
            ),
            "motor burn end": f"{config['motor']['burn_time_s']:.2f} s",
            "minimum deploy time": f"{minimum_deploy_time_s:.2f} s",
            "first C++ command": (
                f"{first_command['time_s']:.2f} s"
                if first_command is not None
                else "not reached"
            ),
            "first physical deployment": (
                f"{first_deployment['time_s']:.2f} s"
                if first_deployment is not None
                else "not reached"
            ),
            "command phases valid": "yes" if command_phases_valid else "no",
            "controller samples": str(len(controller_log)),
            "estimator healthy": "yes" if controller_healthy else "no",
            "maximum altitude error": f"{maximum_altitude_error_m:.2f} m",
            "maximum velocity error": f"{maximum_velocity_error_mps:.2f} m/s",
        },
        "This case proves bridge and safety behavior only. Target attainment is evaluated independently below.",
    )
    print_case(
        3,
        "closed-loop target attainment",
        "The closed-loop apogee from the provisional RocketPy model is compared directly with the 3000 +/-100 ft mission band.",
        "PASS if closed-loop apogee is between 2900 ft and 3100 ft. This is necessary but not sufficient evidence because source-model checks must also pass.",
        target_pass,
        "The provisional closed-loop result is inside the target band." if target_pass else "The provisional closed-loop result is outside the target band.",
        {
            "closed-loop apogee": f"{closed_apogee_ft:.0f} ft",
            "target": f"{target_ft:.0f} ft",
            "tolerance": f"+/-{tolerance_ft:.0f} ft",
            "target error": f"{target_error_ft:+.0f} ft",
        },
        "Do not claim validated target accuracy unless passive-model agreement, uncertainty studies, and hardware evidence also pass.",
    )
    print_case(
        4,
        "M5 flight envelope checks",
        "The closed-loop RocketPy trajectory is checked against the M5 subsonic and minimum rail-exit requirements.",
        "PASS if maximum Mach is no greater than 1.0 and rail-exit velocity is at least 52 ft/s.",
        envelope_pass,
        "The provisional reference trajectory stayed inside both M5 envelope checks." if envelope_pass else "At least one M5 flight-envelope check failed.",
        {
            "maximum Mach": f"{maximum_mach:.3f}",
            "Mach limit": f"{requirements['maximum_mach']:.1f}",
            "M5 reported maximum Mach": f"{requirements['m5_reported_maximum_mach']:.3f}",
            "minimum rail exit velocity": f"{minimum_rail_exit_fps:.1f} ft/s",
            "rail exit minimum": f"{requirements['minimum_rail_exit_velocity_fps']:.1f} ft/s",
            "M5 reported rail exit velocity": f"{requirements['m5_reported_rail_exit_velocity_fps']:.1f} ft/s",
        },
    )
    print_case(
        5,
        "flight-data integrity and recovery observation",
        "The exported controller time history is checked for monotonic timestamps, finite values, sparse barometer sampling, bounded commands, Recovery entry, and physical airbrake retraction after apogee.",
        "PASS if the log is internally consistent, Recovery is observed, at least one barometer sample exists, and rate-limited deployment returns below 2% during the post-apogee observation window.",
        time_history_pass,
        "The plotted data passed its structural checks and includes the post-apogee controller transition." if time_history_pass else "The plotted data or post-apogee controller behavior did not meet its declared integrity rule.",
        {
            "controller samples": str(len(controller_log)),
            "barometer samples": str(barometer_sample_count),
            "phase sequence": " -> ".join(transition["phase"] for transition in phase_transitions),
            "closed-loop apogee time": f"{closed.apogee_time:.2f} s",
            "observation end": f"{observation_end_time_s:.2f} s",
            "peak deployment": f"{peak_deployment * 100.0:.1f}%",
            "final deployment": f"{final_deployment * 100.0:.1f}%",
            "integrity errors": str(len(log_integrity_errors)),
        },
        "Recovery-system descent and parachute dynamics are not modeled. This window verifies only flight-computer phase handling and airbrake retraction.",
    )

    RESULT_PATH.parent.mkdir(parents=True, exist_ok=True)
    RESULT_PATH.write_text(
        json.dumps(
            {
                "schemaVersion": 2,
                "modelStatus": config["model_status"],
                "inputMode": "experimental-overrides" if args.overrides else "report-baseline",
                "appliedConfig": {
                    "environment": config["environment"],
                    "rocket": config["rocket"],
                    "airbrakes": config["airbrakes"],
                    "sensorModel": config["sensor_model"],
                    "requirements": config["requirements"],
                },
                "reportReference": {
                    "passiveApogeeFt": reference_ft,
                    "maximumVelocityFps": requirements["m5_reported_maximum_velocity_fps"],
                    "maximumMach": requirements["m5_reported_maximum_mach"],
                    "railExitVelocityFps": requirements["m5_reported_rail_exit_velocity_fps"],
                },
                "passive": {
                    "apogeeFt": passive_apogee_ft,
                    "maxMach": passive.max_mach_number,
                    "railExitVelocityFps": passive.out_of_rail_velocity * MPS_TO_FPS,
                },
                "closedLoop": {
                    "apogeeFt": closed_apogee_ft,
                    "targetErrorFt": target_error_ft,
                    "apogeeReductionFt": apogee_reduction_ft,
                    "maxMach": closed.max_mach_number,
                    "railExitVelocityFps": closed.out_of_rail_velocity * MPS_TO_FPS,
                    "peakCommandFraction": peak_command,
                    "minimumCommandFraction": minimum_command,
                    "peakDeploymentFraction": peak_deployment,
                    "minimumDeploymentFraction": minimum_deployment,
                    "commandsBounded": commands_bounded,
                    "firstCommandTimeS": first_command["time_s"] if first_command else None,
                    "firstDeploymentTimeS": first_deployment["time_s"] if first_deployment else None,
                    "minimumDeploymentTimeS": minimum_deploy_time_s,
                    "deploymentAfterBurn": deployment_after_burn,
                    "commandPhasesValid": command_phases_valid,
                    "maximumAltitudeErrorM": maximum_altitude_error_m,
                    "maximumVelocityErrorMps": maximum_velocity_error_mps,
                    "apogeeTimeS": closed.apogee_time,
                    "observationEndTimeS": observation_end_time_s,
                    "maximumSpeedMps": maximum_speed_mps,
                    "maximumVerticalAccelerationMps2": maximum_vertical_acceleration_mps2,
                    "barometerSampleCount": barometer_sample_count,
                    "finalDeploymentFraction": final_deployment,
                },
                "acceptance": {
                    "passiveReferencePass": bool(passive_pass),
                    "couplingSafetyPass": bool(coupling_pass),
                    "targetAttainmentPass": bool(target_pass),
                    "flightEnvelopePass": bool(envelope_pass),
                    "timeHistoryPass": bool(time_history_pass),
                },
                "phaseTransitions": phase_transitions,
                "seriesMetadata": {
                    "truth": "RocketPy trajectory state sampled at the controller callback rate.",
                    "sensor": "Deterministic delayed, biased, noisy, quantized virtual measurements supplied to the C++ bridge.",
                    "estimate": "State produced by the repository's C++ vertical EKF.",
                    "truth_acceleration_mps2": "Net vertical acceleration in the launch/navigation frame, positive upward.",
                    "measured_acceleration_mps2": "Virtual launch-frame vertical acceleration after assumed IMU alignment and gravity compensation; this is not raw body-axis specific force.",
                    "measured_altitude_m": "Barometric altitude sample. Null means the barometer was not sampled on that controller tick.",
                    "command_fraction": "C++ controller request before actuator rate limiting.",
                    "actual_deployment_fraction": "Deployment applied to RocketPy after the configured actuator rate limit.",
                },
                "limitations": [
                    "Mass, center of gravity, inertia, rocket drag, and airbrake drag remain provisional.",
                    "The virtual accelerometer starts from gravity-compensated launch-frame vertical acceleration; raw three-axis IMU, attitude estimation, axis misalignment, and vibration are not modeled.",
                    "The post-apogee window does not model parachutes, recovery electronics, landing, or deployment loads.",
                    "The embedded apogee predictor is ballistic and is not calibrated to the provisional RocketPy drag model.",
                ],
                "logIntegrityErrors": log_integrity_errors,
                "controllerLog": controller_log,
                "provenance": config["provenance"],
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    print("\nSUMMARY")
    print("scenario                     result")
    print("-----------------------------------")
    print(f"M5 passive reference         {'PASS' if passive_pass else 'FAIL'}")
    print(f"C++ closed-loop airbrakes    {'PASS' if coupling_pass else 'FAIL'}")
    print(f"Closed-loop target           {'PASS' if target_pass else 'FAIL'}")
    print(f"M5 flight envelope checks    {'PASS' if envelope_pass else 'FAIL'}")
    print(f"Flight-data integrity        {'PASS' if time_history_pass else 'FAIL'}")
    # Keep terminal output portable and independent of the user's checkout path.
    print("\nMachine-readable results: build/rocketpy-last-run.json")
    print("Next model inputs needed: measured flight-ready mass/CG/inertia, final OpenRocket .ork export, and airbrake drag coefficient vs Mach/deployment.")
    return 0 if passive_pass and coupling_pass and target_pass and envelope_pass and time_history_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
