"""RocketPy physics simulation connected to the AMBAR STM32 C flight core.

RocketPy owns the motor, changing mass, atmosphere, aerodynamics, and 6-DOF
trajectory. A persistent line-oriented bridge feeds a virtual pad-referenced
body-axis IMU channel and barometer measurements into the production C
estimator/controller compiled from the STM32 project. The returned deployment
command drives RocketPy airbrakes.

Architecture connections:
- ambar_reference_config.json supplies the versioned vehicle/model inputs.
- j420r.eng supplies the motor thrust curve.
- sim/stm32_controller_bridge.c exposes the production STM32 flight logic.
- scripts/run_rocketpy_sim.ps1 builds the bridge and launches this module.
- build/rocketpy-last-run.json is the detailed machine-readable output.

Use this file for trajectory/controller coupling and apogee studies. Use the
smaller C++ sandboxes for quick logic and fault-regression checks.
"""

from __future__ import annotations

import argparse
import json
import math
import queue
import random
import subprocess
import threading
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
STANDARD_GRAVITY_MPS2 = 9.80665
BRIDGE_RESPONSE_TIMEOUT_S = 2.0


def plant_airbrake_drag_coefficient(
    config: dict[str, Any], deployment: float, mach: float
) -> float:
    """Map the versioned total-CdA stations to RocketPy incremental Cd.

    Calibration version zero retains the legacy linear 0.95-style model.  The
    explicit M5 profile instead subtracts fitted baseline CdA and divides by the
    same frontal reference area used for the fitted rocket Cd.
    """
    airbrakes = config["airbrakes"]
    predictor = config.get("predictor", {})
    if int(predictor.get("calibration_version", 0)) == 0:
        return (
            float(airbrakes["drag_coefficient_at_full_deployment"])
            * deployment
            * (
                1.0
                + float(airbrakes["mach_drag_multiplier_at_mach_1"])
                * min(max(mach, 0.0), 1.0)
            )
        )

    fractions = predictor["deployment_fractions"]
    total_cda = predictor["deployment_drag_area_m2"]
    bounded = min(max(float(deployment), 0.0), 1.0)
    upper_index = next(
        (index for index, fraction in enumerate(fractions) if fraction >= bounded),
        len(fractions) - 1,
    )
    lower_index = max(0, upper_index - 1)
    lower_fraction = float(fractions[lower_index])
    upper_fraction = float(fractions[upper_index])
    if upper_fraction <= lower_fraction:
        interpolated_cda = float(total_cda[upper_index])
    else:
        local = (bounded - lower_fraction) / (upper_fraction - lower_fraction)
        interpolated_cda = float(total_cda[lower_index]) + local * (
            float(total_cda[upper_index]) - float(total_cda[lower_index])
        )
    incremental_cda = max(
        0.0, interpolated_cda - float(predictor["baseline_drag_area_m2"])
    )
    full_incremental_cda = max(
        1.0e-12,
        float(total_cda[-1]) - float(predictor["baseline_drag_area_m2"]),
    )
    # Preserve the five-point curve shape while allowing Monte Carlo to vary
    # plant-only full authority without leaking that sampled truth into the
    # controller predictor configuration.
    coefficient = (
        incremental_cda
        / full_incremental_cda
        * float(airbrakes["drag_coefficient_at_full_deployment"])
    )
    return coefficient * (
        1.0
        + float(airbrakes["mach_drag_multiplier_at_mach_1"])
        * min(max(mach, 0.0), 1.0)
    )


@dataclass
class BridgeState:
    deploy_fraction: float = 0.0
    inhibit: bool = True
    inhibit_flags: int = 0
    altitude_m: float = 0.0
    velocity_mps: float = 0.0
    predicted_apogee_m: float = 0.0
    closed_predicted_apogee_m: float = 0.0
    full_predicted_apogee_m: float = 0.0
    healthy: bool = True
    controller_mode_used: int = 0
    predictive_solution_valid: bool = False
    target_reachable: bool = False
    phase: str = "PadIdle"


class SensorModel:
    """Deterministic provisional sensor errors applied before the C bridge."""

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


def rocket_body_z_axis(state: list[float]) -> tuple[float, float, float]:
    """Return RocketPy's body-Z/longitudinal axis in inertial coordinates."""

    e0, e1, e2, e3 = map(float, state[6:10])
    return (
        2.0 * (e1 * e3 + e0 * e2),
        2.0 * (e2 * e3 - e0 * e1),
        e0 * e0 - e1 * e1 - e2 * e2 + e3 * e3,
    )


def pad_referenced_body_axis_acceleration(
    acceleration_world_mps2: tuple[float, float, float],
    body_z_axis: tuple[float, float, float],
    pad_reference_specific_force_mps2: float,
) -> float:
    """Model the scalar acceleration channel currently used by the firmware.

    A real accelerometer measures specific force along the configured body axis.
    ``rocket_sensors.c`` subtracts the pad reference and currently performs no
    attitude rotation.  Projecting RocketPy's world acceleration into body Z
    preserves that contract and exposes wind/tilt effects instead of supplying
    unrealistically perfect world-vertical acceleration to the EKF.
    """

    specific_force_world = (
        acceleration_world_mps2[0],
        acceleration_world_mps2[1],
        acceleration_world_mps2[2] + STANDARD_GRAVITY_MPS2,
    )
    projected_specific_force = sum(
        component * axis
        for component, axis in zip(specific_force_world, body_z_axis)
    )
    return projected_specific_force - pad_reference_specific_force_mps2


class ControllerBridge:
    """Own the persistent production-C child process and its line protocol."""

    def __init__(
        self,
        executable: Path,
        minimum_boost_time_s: float,
        target_apogee_m: float,
        target_tolerance_m: float,
        controller_config: dict[str, Any] | None = None,
        predictor_config: dict[str, Any] | None = None,
    ) -> None:
        self._failure: str | None = None
        controller_values = controller_config or {}
        predictor_values = predictor_config or {}
        cda_points = predictor_values.get(
            "deployment_drag_area_m2",
            [0.012, 0.012, 0.012, 0.012, 0.012],
        )
        if len(cda_points) != 5:
            raise ValueError("predictor deployment_drag_area_m2 must contain 5 points")
        control_mode = 1 if controller_values.get("control_mode") == "predictive" else 0
        bridge_arguments = [
            str(executable),
            "--minimum-boost-time", f"{minimum_boost_time_s:.6f}",
            "--target-apogee-m", f"{target_apogee_m:.6f}",
            "--target-tolerance-m", f"{target_tolerance_m:.6f}",
        ]
        if controller_config is not None and predictor_config is not None:
            bridge_arguments.extend(
                [
                    "--mission-tolerance-m",
                    f"{controller_values['mission_tolerance_ft'] * FEET_TO_METERS:.9f}",
                    "--control-deadband-m",
                    f"{controller_values['control_deadband_ft'] * FEET_TO_METERS:.9f}",
                    "--deployment-hysteresis",
                    f"{controller_values['deployment_hysteresis_fraction']:.9f}",
                    "--predictive-period-s",
                    f"{controller_values['predictive_update_period_s']:.9f}",
                    "--control-mode", str(control_mode),
                    "--calibration-version", str(predictor_values["calibration_version"]),
                    "--coast-mass-kg", f"{predictor_values['coast_mass_kg']:.9f}",
                    "--baseline-cda-m2",
                    f"{predictor_values['baseline_drag_area_m2']:.9f}",
                    "--cda-0-m2", f"{cda_points[0]:.9f}",
                    "--cda-25-m2", f"{cda_points[1]:.9f}",
                    "--cda-50-m2", f"{cda_points[2]:.9f}",
                    "--cda-75-m2", f"{cda_points[3]:.9f}",
                    "--cda-100-m2", f"{cda_points[4]:.9f}",
                    "--sea-level-density-kgpm3",
                    f"{predictor_values['sea_level_air_density_kgpm3']:.9f}",
                    "--density-scale-height-m",
                    f"{predictor_values['density_scale_height_m']:.6f}",
                    "--launch-site-elevation-m",
                    f"{predictor_values['launch_site_elevation_m']:.6f}",
                    "--predictor-time-step-s",
                    f"{predictor_values['time_step_s']:.9f}",
                    "--predictor-max-time-s",
                    f"{predictor_values['max_predict_time_s']:.6f}",
                    "--actuator-delay-s",
                    f"{predictor_values['actuator_delay_s']:.9f}",
                    "--actuator-open-rate",
                    f"{predictor_values['actuator_open_rate_fraction_per_s']:.9f}",
                    "--actuator-close-rate",
                    f"{predictor_values['actuator_close_rate_fraction_per_s']:.9f}",
                ]
            )
        self.process = subprocess.Popen(
            bridge_arguments,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            bufsize=1,
        )
        self._responses: queue.Queue[str | None] = queue.Queue()
        self._reader = threading.Thread(
            target=self._read_responses,
            name="ambar-controller-bridge-reader",
            daemon=True,
        )
        self._reader.start()
        self.state = self._exchange("RESET 0")

    def _read_responses(self) -> None:
        """Move blocking pipe reads to one daemon thread with EOF signaling."""

        if self.process.stdout is None:
            self._responses.put(None)
            return
        for line in self.process.stdout:
            self._responses.put(line)
        self._responses.put(None)

    def _exchange(self, command: str) -> BridgeState:
        if self.process.stdin is None:
            raise RuntimeError("Controller bridge pipes are unavailable.")
        self.process.stdin.write(command + "\n")
        self.process.stdin.flush()
        try:
            response_line = self._responses.get(timeout=BRIDGE_RESPONSE_TIMEOUT_S)
        except queue.Empty as error:
            self._failure = (
                f"controller bridge timed out after {BRIDGE_RESPONSE_TIMEOUT_S:.1f} s"
            )
            self.process.kill()
            self.process.wait(timeout=2)
            raise TimeoutError(self._failure) from error
        if response_line is None:
            self._failure = "controller bridge closed its output unexpectedly"
            raise RuntimeError(self._failure)
        response = response_line.strip()
        if not response.startswith("STATE "):
            raise RuntimeError(f"Controller bridge returned: {response or '<no output>'}")
        values = response.split()
        if len(values) not in (9, 14):
            raise RuntimeError(f"Controller bridge state is malformed: {response}")
        if len(values) == 14:
            self.state = BridgeState(
                deploy_fraction=float(values[1]),
                inhibit=values[2] == "1",
                inhibit_flags=int(values[3]),
                altitude_m=float(values[4]),
                velocity_mps=float(values[5]),
                predicted_apogee_m=float(values[6]),
                closed_predicted_apogee_m=float(values[7]),
                full_predicted_apogee_m=float(values[8]),
                healthy=values[9] == "1",
                controller_mode_used=int(values[10]),
                predictive_solution_valid=values[11] == "1",
                target_reachable=values[12] == "1",
                phase=values[13],
            )
        else:
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
        actuator_fraction: float | None = None,
    ) -> BridgeState:
        actuator_field = (
            f" {actuator_fraction:.9f}" if actuator_fraction is not None else ""
        )
        return self._exchange(
            "STEP "
            f"{timestamp_s:.9f} {acceleration_mps2:.9f} {altitude_m:.9f} "
            f"{altitude_std_dev_m:.9f} {1 if use_barometer else 0}"
            f"{actuator_field}"
        )

    def predict(
        self,
        altitude_m: float,
        velocity_mps: float,
        current_fraction: float,
        target_fraction: float,
    ) -> float:
        """Evaluate the production pure coast predictor without changing EKF state."""
        if self.process.stdin is None:
            raise RuntimeError("Controller bridge pipes are unavailable.")
        self.process.stdin.write(
            "PREDICT "
            f"{altitude_m:.9f} {velocity_mps:.9f} "
            f"{current_fraction:.9f} {target_fraction:.9f}\n"
        )
        self.process.stdin.flush()
        try:
            response_line = self._responses.get(timeout=BRIDGE_RESPONSE_TIMEOUT_S)
        except queue.Empty as error:
            self._failure = "controller predictor bridge timed out"
            self.process.kill()
            self.process.wait(timeout=2)
            raise TimeoutError(self._failure) from error
        if response_line is None:
            self._failure = "controller predictor bridge closed unexpectedly"
            raise RuntimeError(self._failure)
        response = response_line.strip().split()
        if len(response) != 2 or response[0] != "PREDICTION":
            raise RuntimeError(
                f"Controller predictor returned: {' '.join(response) or '<no output>'}"
            )
        prediction = float(response[1])
        if not math.isfinite(prediction):
            raise RuntimeError("Controller predictor returned a non-finite result")
        return prediction

    def close(self) -> None:
        if self.process.poll() is None and self._failure is not None:
            self.process.kill()
            self.process.wait(timeout=2)
        elif self.process.poll() is None and self.process.stdin is not None:
            try:
                self.process.stdin.write("QUIT\n")
                self.process.stdin.flush()
                self.process.wait(timeout=2)
            except (BrokenPipeError, subprocess.TimeoutExpired):
                self.process.kill()
                self.process.wait(timeout=2)
        error = (
            self.process.stderr.read()
            if self.process.returncode not in (None, 0) and self.process.stderr
            else ""
        )
        for stream in (self.process.stdin, self.process.stdout, self.process.stderr):
            if stream is not None and not stream.closed:
                stream.close()
        if self.process.returncode not in (None, 0):
            raise RuntimeError(
                f"Controller bridge exited with {self.process.returncode}: {error}"
            )


def deep_merge(
    base: dict[str, Any],
    overrides: dict[str, Any],
    prefix: str = "",
) -> dict[str, Any]:
    """Merge known override keys without mutating the baseline config.

    A misspelled key must stop a study instead of appearing in the provenance
    hash while having no effect on RocketPy.  Numeric int/float substitutions
    remain allowed because JSON has only one practical number contract here.
    """

    merged = dict(base)
    for key, value in overrides.items():
        path = f"{prefix}.{key}" if prefix else key
        if key not in base:
            raise ValueError(f"Unknown simulation override key: {path}")
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value, path)
        elif isinstance(value, dict) or isinstance(merged.get(key), dict):
            raise ValueError(f"Simulation override type mismatch at: {path}")
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


def flight_apogee_agl_m(flight: Flight, config: dict[str, Any]) -> float:
    """Convert RocketPy's absolute launch-site z coordinate to pad-relative AGL.

    Mission targets, OpenRocket comparisons, firmware estimates, and exported
    truth channels are all above-ground-level values.  RocketPy's ``apogee``
    property includes the site's elevation, so comparing it directly would add
    23 ft at the current launch site and can flip a boundary acceptance result.
    """

    return float(flight.apogee) - float(config["environment"]["elevation_m"])


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
        # Preserve the certified curve shape while allowing an explicitly
        # recorded burn-time/total-impulse dispersion to affect the plant.  A
        # bare burn_time argument is ignored when it exceeds the source curve;
        # reshaping prevents a CSV column from claiming variation that RocketPy
        # did not actually apply.
        reshape_thrust_curve=(
            values["burn_time_s"],
            values["total_impulse_ns"],
        ),
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
    """Run RocketPy while the production STM32 C controller commands airbrakes.

    The closed-loop run continues briefly after apogee. This does not model the
    recovery system; it exists so the controller's Recovery transition and the
    actuator's commanded retraction are visible in the time-history output.
    """
    environment = build_environment(config)
    rocket = build_rocket(config, build_motor(config))
    airbrake_config = config["airbrakes"]
    requirements = config["requirements"]
    controller_config = config["controller"]
    # Controller configuration is deliberately independent of RocketPy truth.
    # Monte Carlo changes the real/virtual motor burn time, but the controller
    # must not receive that sampled answer.  Its fixed threshold comes from the
    # production firmware configuration and safety acceptance is evaluated
    # separately against the randomized true burnout time.
    bridge = ControllerBridge(
        bridge_path,
        controller_config["minimum_boost_time_s"],
        controller_config["target_apogee_ft"] * FEET_TO_METERS,
        controller_config["target_tolerance_ft"] * FEET_TO_METERS,
        controller_config=controller_config,
        predictor_config=config["predictor"],
    )
    log: list[dict[str, Any]] = []
    previous_time: float | None = None
    previous_velocity_vector = (0.0, 0.0, 0.0)
    pad_reference_specific_force_mps2: float | None = None
    next_barometer_time = 1.0 / airbrake_config["barometer_rate_hz"]
    actual_deployment = 0.0
    sensor_model = SensorModel(config)
    truth_history: deque[tuple[float, float, float]] = deque()
    # The virtual actuator has two distinct limitations.  Command history
    # models electronic/mechanical dead time; move-toward below models the
    # measured full-stroke rate.  Keeping both visible prevents an optimistic
    # zero-delay actuator from hiding a late controller decision.
    command_history: deque[tuple[float, float]] = deque([(0.0, 0.0)])

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
        nonlocal previous_time, previous_velocity_vector, next_barometer_time
        nonlocal actual_deployment, pad_reference_specific_force_mps2

        if previous_time is not None and time <= previous_time + 1e-9:
            air_brakes.deployment_level = actual_deployment
            return None

        velocity_vector_mps = (float(state[3]), float(state[4]), float(state[5]))
        velocity_mps = velocity_vector_mps[2]
        speed_mps = math.sqrt(sum(component * component for component in velocity_vector_mps))
        altitude_m = max(0.0, float(state[2]) - controller_environment.elevation)
        body_z_axis = rocket_body_z_axis(state)
        if previous_time is None:
            previous_time = time
            previous_velocity_vector = velocity_vector_mps
            pad_reference_specific_force_mps2 = (
                STANDARD_GRAVITY_MPS2 * body_z_axis[2]
            )
            air_brakes.deployment_level = 0.0
            truth_history.append((time, 0.0, altitude_m))
            log.append(
                {
                    "time_s": time,
                    "truth_altitude_m": altitude_m,
                    "truth_velocity_mps": velocity_mps,
                    "truth_speed_mps": speed_mps,
                    "truth_acceleration_mps2": 0.0,
                    "truth_sensor_axis_acceleration_mps2": 0.0,
                    "truth_acceleration_magnitude_mps2": 0.0,
                    "measured_acceleration_mps2": 0.0,
                    "measured_altitude_m": None,
                    "barometer_sample": False,
                    "measurement_source_time_s": time,
                    "estimated_altitude_m": bridge.state.altitude_m,
                    "estimated_velocity_mps": bridge.state.velocity_mps,
                    "predicted_apogee_m": bridge.state.predicted_apogee_m,
                    "closed_predicted_apogee_m": bridge.state.closed_predicted_apogee_m,
                    "full_predicted_apogee_m": bridge.state.full_predicted_apogee_m,
                    "controller_mode_used": bridge.state.controller_mode_used,
                    "predictive_solution_valid": bridge.state.predictive_solution_valid,
                    "target_reachable": bridge.state.target_reachable,
                    "command_fraction": 0.0,
                    "desired_deployment_fraction": 0.0,
                    "delayed_desired_deployment_fraction": 0.0,
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
            sensor_axis_acceleration_mps2 = pad_referenced_body_axis_acceleration(
                acceleration_vector_mps2,
                body_z_axis,
                pad_reference_specific_force_mps2
                if pad_reference_specific_force_mps2 is not None
                else STANDARD_GRAVITY_MPS2 * body_z_axis[2],
            )

        truth_history.append((time, sensor_axis_acceleration_mps2, altitude_m))
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
            actuator_fraction=actual_deployment,
        )
        desired_deployment = 0.0 if bridge_state.inhibit else bridge_state.deploy_fraction
        command_history.append((time, desired_deployment))
        delayed_command_time = time - airbrake_config.get("actuator_delay_s", 0.0)
        while len(command_history) > 1 and command_history[1][0] <= delayed_command_time:
            command_history.popleft()
        delayed_desired_deployment = (
            command_history[0][1]
            if command_history[0][0] <= delayed_command_time
            else 0.0
        )
        actuator_rate = (
            airbrake_config.get(
                "opening_rate_fraction_per_s",
                airbrake_config["maximum_rate_fraction_per_s"],
            )
            if delayed_desired_deployment >= actual_deployment
            else airbrake_config.get(
                "closing_rate_fraction_per_s",
                airbrake_config["maximum_rate_fraction_per_s"],
            )
        )
        maximum_change = actuator_rate * dt_s
        actual_deployment += max(
            -maximum_change,
            min(maximum_change, delayed_desired_deployment - actual_deployment),
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
                "truth_sensor_axis_acceleration_mps2": sensor_axis_acceleration_mps2,
                "truth_acceleration_magnitude_mps2": acceleration_magnitude_mps2,
                "measured_acceleration_mps2": measured_acceleration_mps2,
                "measured_altitude_m": measured_altitude_m,
                "barometer_sample": use_barometer,
                "measurement_source_time_s": measurement_source_time_s,
                "estimated_altitude_m": bridge_state.altitude_m,
                "estimated_velocity_mps": bridge_state.velocity_mps,
                "predicted_apogee_m": bridge_state.predicted_apogee_m,
                "closed_predicted_apogee_m": bridge_state.closed_predicted_apogee_m,
                "full_predicted_apogee_m": bridge_state.full_predicted_apogee_m,
                "controller_mode_used": bridge_state.controller_mode_used,
                "predictive_solution_valid": bridge_state.predictive_solution_valid,
                "target_reachable": bridge_state.target_reachable,
                "command_fraction": bridge_state.deploy_fraction,
                "desired_deployment_fraction": desired_deployment,
                "delayed_desired_deployment_fraction": delayed_desired_deployment,
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
            plant_airbrake_drag_coefficient(config, deployment, mach)
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
        "truth_sensor_axis_acceleration_mps2",
        "truth_acceleration_magnitude_mps2",
        "measured_acceleration_mps2",
        "measurement_source_time_s",
        "estimated_altitude_m",
        "estimated_velocity_mps",
        "predicted_apogee_m",
        "closed_predicted_apogee_m",
        "full_predicted_apogee_m",
        "command_fraction",
        "desired_deployment_fraction",
        "delayed_desired_deployment_fraction",
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


def run_fixed_deployment(
    config: dict[str, Any], deployment_fraction: float
) -> tuple[Flight, list[dict[str, float]]]:
    """Run a RocketPy coast with one fixed airbrake station after the safe gate.

    This is predictor-calibration truth, not controller acceptance.  The plant
    retains configured command delay and directional stroke rates, and samples
    are useful as fixed-deployment references only after the requested station
    has physically settled.
    """
    environment = build_environment(config)
    rocket = build_rocket(config, build_motor(config))
    airbrake_config = config["airbrakes"]
    minimum_command_time_s = float(config["controller"]["minimum_boost_time_s"])
    requested_fraction = min(max(float(deployment_fraction), 0.0), 1.0)
    actual_deployment = 0.0
    previous_time: float | None = None
    command_history: deque[tuple[float, float]] = deque([(0.0, 0.0)])
    log: list[dict[str, float]] = []

    def fixed_controller(
        time: float,
        sampling_rate: float,
        state: list[float],
        state_history: list[list[float]],
        observed_variables: list[Any],
        air_brakes: Any,
        sensors: list[Any],
        controller_environment: Environment,
    ) -> None:
        del sampling_rate, state_history, observed_variables, sensors
        nonlocal actual_deployment, previous_time
        if previous_time is not None and time <= previous_time + 1.0e-9:
            air_brakes.deployment_level = actual_deployment
            return

        dt_s = 0.0 if previous_time is None else time - previous_time
        desired = requested_fraction if time >= minimum_command_time_s else 0.0
        command_history.append((time, desired))
        delayed_time = time - float(airbrake_config.get("actuator_delay_s", 0.0))
        while len(command_history) > 1 and command_history[1][0] <= delayed_time:
            command_history.popleft()
        delayed_desired = (
            command_history[0][1]
            if command_history[0][0] <= delayed_time
            else 0.0
        )
        rate = (
            float(
                airbrake_config.get(
                    "opening_rate_fraction_per_s",
                    airbrake_config["maximum_rate_fraction_per_s"],
                )
            )
            if delayed_desired >= actual_deployment
            else float(
                airbrake_config.get(
                    "closing_rate_fraction_per_s",
                    airbrake_config["maximum_rate_fraction_per_s"],
                )
            )
        )
        maximum_change = rate * dt_s
        actual_deployment += max(
            -maximum_change,
            min(maximum_change, delayed_desired - actual_deployment),
        )
        actual_deployment = min(max(actual_deployment, 0.0), 1.0)
        air_brakes.deployment_level = actual_deployment
        log.append(
            {
                "time_s": float(time),
                "altitude_m": max(
                    0.0, float(state[2]) - controller_environment.elevation
                ),
                "vertical_velocity_mps": float(state[5]),
                "deployment_fraction": actual_deployment,
            }
        )
        previous_time = time

    rocket.add_air_brakes(
        drag_coefficient_curve=lambda deployment, mach: (
            plant_airbrake_drag_coefficient(config, deployment, mach)
        ),
        controller_function=fixed_controller,
        sampling_rate=airbrake_config["sampling_rate_hz"],
        clamp=True,
        override_rocket_drag=False,
        name=f"AMBAR fixed deployment {requested_fraction:.2f}",
    )
    values = config["environment"]
    flight = Flight(
        rocket=rocket,
        environment=environment,
        rail_length=values["rail_length_m"],
        inclination=values["inclination_deg"],
        heading=values["heading_deg"],
        terminate_on_apogee=True,
        max_time=40,
        max_time_step=0.01,
        verbose=False,
    )
    return flight, log


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
    print("Purpose: run RocketPy trajectory physics while the production STM32 C flight computer commands the virtual airbrakes.")
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
    print(
        "  airbrake Mach-1 drag multiplier: "
        f"{100.0 * config['airbrakes']['mach_drag_multiplier_at_mach_1']:.1f}%"
    )
    print(
        "  fixed controller minimum boost time: "
        f"{config['controller']['minimum_boost_time_s']:.2f} s"
    )
    print(
        "  measured deployment rates: "
        f"open={config['airbrakes'].get('opening_rate_fraction_per_s', config['airbrakes']['maximum_rate_fraction_per_s']) * 100.0:.1f}, "
        f"close={config['airbrakes'].get('closing_rate_fraction_per_s', config['airbrakes']['maximum_rate_fraction_per_s']) * 100.0:.1f} percent/s"
    )
    print(f"  actuator command delay: {config['airbrakes'].get('actuator_delay_s', 0.0) * 1000.0:.0f} ms")
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

    passive_apogee_ft = flight_apogee_agl_m(passive, config) * METERS_TO_FEET
    closed_apogee_ft = flight_apogee_agl_m(closed, config) * METERS_TO_FEET
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
        entry["phase"] in {"Coast", "AirbrakeActive"}
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
        "STM32-C closed-loop airbrakes",
        "RocketPy feeds a pad-referenced body-axis IMU channel and delayed/noisy barometer values to the production STM32 C AMBAR flight computer, then applies its bounded command to delayed, rate-limited virtual airbrakes.",
        "PASS if the estimator stays healthy, commands remain bounded, deployment starts after motor burnout plus the configured margin, commands occur only in Coast/AirbrakeActive, and deployment reduces apogee by more than 50 ft.",
        coupling_pass,
        "The production C controller remained healthy and changed the RocketPy trajectory." if coupling_pass else "The STM32-C/RocketPy coupling did not meet the declared behavior check.",
        {
            "closed-loop apogee": f"{closed_apogee_ft:.0f} ft",
            "target error": f"{target_error_ft:+.0f} ft",
            "apogee reduction": f"{apogee_reduction_ft:.0f} ft",
            "STM32-C command range": (
                f"{minimum_command * 100.0:.1f}% to {peak_command * 100.0:.1f}%"
            ),
            "virtual deployment range": (
                f"{minimum_deployment * 100.0:.1f}% to {peak_deployment * 100.0:.1f}%"
            ),
            "motor burn end": f"{config['motor']['burn_time_s']:.2f} s",
            "independent safety enable time": f"{minimum_deploy_time_s:.2f} s",
            "first STM32-C command": (
                f"{first_command['time_s']:.2f} s"
                if first_command is not None
                else "not reached"
            ),
            "first virtual deployment": (
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
                    "sensor": "Deterministic delayed, biased, noisy, quantized virtual measurements supplied to the production-C bridge.",
                    "estimate": "State produced by the STM32 project's production vertical EKF.",
                    "truth_acceleration_mps2": "Net vertical acceleration in the launch/navigation frame, positive upward.",
                    "truth_sensor_axis_acceleration_mps2": "Pad-referenced RocketPy body-Z acceleration before sensor bias, noise, quantization, and latency.",
                    "measured_acceleration_mps2": "Virtual configured-body-axis channel after pad subtraction and provisional sensor errors; raw registers, vibration, and mounting error are not modeled.",
                    "measured_altitude_m": "Barometric altitude sample. Null means the barometer was not sampled on that controller tick.",
                    "command_fraction": "Production STM32-C controller request before actuator delay and rate limiting.",
                    "delayed_desired_deployment_fraction": "Controller request after the configured command-to-motion delay and before rate limiting.",
                    "actual_deployment_fraction": "Deployment applied to RocketPy after the configured actuator rate limit.",
                },
                "limitations": [
                    "Mass, center of gravity, inertia, rocket drag, and airbrake drag remain provisional.",
                    "The virtual accelerometer projects RocketPy motion into body Z and subtracts its pad reference; raw registers, mounting error, vibration, and firmware attitude rotation are not modeled.",
                    "The post-apogee window does not model parachutes, recovery electronics, landing, or deployment loads.",
                    "The embedded drag-aware vertical apogee predictor retains provisional mass, CdA, density, and effectiveness values and is not calibrated to the RocketPy model.",
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
    print(f"STM32-C closed-loop brakes   {'PASS' if coupling_pass else 'FAIL'}")
    print(f"Closed-loop target           {'PASS' if target_pass else 'FAIL'}")
    print(f"M5 flight envelope checks    {'PASS' if envelope_pass else 'FAIL'}")
    print(f"Flight-data integrity        {'PASS' if time_history_pass else 'FAIL'}")
    # Keep terminal output portable and independent of the user's checkout path.
    print("\nMachine-readable results: build/rocketpy-last-run.json")
    print("Next model inputs needed: measured flight-ready mass/CG/inertia, final OpenRocket .ork export, and airbrake drag coefficient vs Mach/deployment.")
    return 0 if passive_pass and coupling_pass and target_pass and envelope_pass and time_history_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
