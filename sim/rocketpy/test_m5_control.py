"""Fast regression tests for the opt-in M5 VARIABLE_HIL control profile."""

from __future__ import annotations

import ast
import os
import sys
import unittest
from pathlib import Path


MODULE_DIR = Path(__file__).resolve().parent
ROOT = MODULE_DIR.parents[1]
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

from m5_control_calibration import (  # noqa: E402
    CoastCase,
    calibrate_fixed_deployment,
    reference_predict_apogee,
    run_authority_sweep,
)
from run_monte_carlo import load_study_config  # noqa: E402
from run_rocketpy_sim import (  # noqa: E402
    FEET_TO_METERS,
    METERS_TO_FEET,
    load_config,
    plant_airbrake_drag_coefficient,
)


VARIABLE_CONFIG = MODULE_DIR / "variable_hil_m5_config.json"
VARIABLE_STUDY = MODULE_DIR / "variable_hil_monte_carlo_config.json"
BRIDGE_PATH = ROOT / "build" / (
    "ambar_stm32_controller_bridge.exe"
    if os.name == "nt"
    else "ambar_stm32_controller_bridge"
)


class ConfigurationIsolationTests(unittest.TestCase):
    def test_reference_entrypoint_reaches_flight_execution(self) -> None:
        source_path = MODULE_DIR / "run_rocketpy_sim.py"
        module = ast.parse(source_path.read_text(encoding="utf-8"))
        functions = {
            node.name: node
            for node in module.body
            if isinstance(node, ast.FunctionDef)
        }
        self.assertIn("run_fixed_deployment", functions)
        main_calls = {
            node.func.id
            for node in ast.walk(functions["main"])
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        }
        self.assertIn("run_passive", main_calls)
        self.assertIn("run_closed_loop", main_calls)

    def test_normal_reference_remains_legacy(self) -> None:
        config = load_config()
        self.assertEqual(config["airbrakes"]["sampling_rate_hz"], 100.0)
        self.assertEqual(config["airbrakes"]["maximum_rate_fraction_per_s"], 1.55)
        self.assertEqual(config["controller"]["control_mode"], "proportional")
        self.assertEqual(config["controller"]["control_deadband_ft"], 100.0)
        self.assertEqual(config["predictor"]["calibration_version"], 0)
        self.assertEqual(config["predictor"]["coast_mass_kg"], 5.0)
        self.assertEqual(config["predictor"]["baseline_drag_area_m2"], 0.012)

    def test_variable_profile_is_explicit_and_does_not_change_mass_or_motor(self) -> None:
        normal = load_config()
        variable = load_config(VARIABLE_CONFIG)
        self.assertEqual(variable["airbrakes"]["sampling_rate_hz"], 50.0)
        self.assertEqual(variable["controller"]["control_mode"], "predictive")
        self.assertEqual(variable["controller"]["control_deadband_ft"], 10.0)
        self.assertEqual(variable["airbrakes"]["opening_rate_fraction_per_s"], 0.864)
        self.assertEqual(variable["airbrakes"]["closing_rate_fraction_per_s"], 0.844)
        self.assertEqual(variable["rocket"]["dry_mass_kg"], normal["rocket"]["dry_mass_kg"])
        self.assertEqual(variable["motor"], normal["motor"])

    def test_plant_curve_uses_five_points_and_load_point_full_coefficient(self) -> None:
        config = load_config(VARIABLE_CONFIG)
        coefficients = [
            plant_airbrake_drag_coefficient(config, fraction, 0.5)
            for fraction in (0.0, 0.25, 0.5, 0.75, 1.0)
        ]
        self.assertEqual(coefficients[0], 0.0)
        self.assertTrue(all(a < b for a, b in zip(coefficients, coefficients[1:])))
        self.assertAlmostEqual(coefficients[-1], 1.866371360, places=7)
        self.assertEqual(config["airbrakes"]["mach_drag_multiplier_at_mach_1"], 0.0)

    def test_variable_study_has_required_gates_and_independent_rates(self) -> None:
        study = load_study_config(VARIABLE_STUDY)
        paths = {
            path
            for parameter in study["parameters"]
            for path in parameter["paths"]
        }
        self.assertIn("rocket.dry_mass_kg", paths)
        self.assertIn("rocket.power_off_drag_coefficient", paths)
        self.assertIn("motor.total_impulse_ns", paths)
        self.assertIn("airbrakes.drag_coefficient_at_full_deployment", paths)
        self.assertIn("airbrakes.opening_rate_fraction_per_s", paths)
        self.assertIn("airbrakes.closing_rate_fraction_per_s", paths)
        acceptance = study["acceptance"]
        self.assertEqual(acceptance["required_target_hit_rate"], 0.96)
        self.assertEqual(acceptance["maximum_p95_absolute_target_error_ft"], 100.0)
        self.assertEqual(acceptance["maximum_worst_absolute_target_error_ft"], 200.0)
        self.assertEqual(acceptance["required_safety_pass_rate"], 1.0)
        self.assertEqual(acceptance["required_effectiveness_pass_rate"], 1.0)


class GateLogicTests(unittest.TestCase):
    def test_authority_gate_uses_full_denominator(self) -> None:
        class FakeBridge:
            @staticmethod
            def predict(
                altitude_m: float,
                velocity_mps: float,
                current_fraction: float,
                target_fraction: float,
            ) -> float:
                del velocity_mps, current_fraction
                if target_fraction == 0.0:
                    return 3200.0 * FEET_TO_METERS
                return (3000.0 if altitude_m > 200.0 else 2800.0) * FEET_TO_METERS

        cases = [CoastCase(200.0, 150.0, 0.0, True) for _ in range(24)]
        cases.append(CoastCase(201.0, 150.0, 0.0, True))
        result = run_authority_sweep(FakeBridge(), cases)
        self.assertEqual(result["pass_rate"], 0.96)
        self.assertTrue(result["passed"])

        cases[-2] = CoastCase(202.0, 150.0, 0.0, True)
        result = run_authority_sweep(FakeBridge(), cases)
        self.assertEqual(result["pass_rate"], 0.92)
        self.assertFalse(result["passed"])

    def test_fixed_deployment_reference_metrics_are_deterministic(self) -> None:
        config = load_config(VARIABLE_CONFIG)
        predictor = config["predictor"]

        class ReferenceBridge:
            @staticmethod
            def predict(
                altitude_m: float,
                velocity_mps: float,
                current_fraction: float,
                target_fraction: float,
            ) -> float:
                return reference_predict_apogee(
                    predictor,
                    altitude_m,
                    velocity_mps,
                    current_fraction,
                    target_fraction,
                )

        cases = [
            CoastCase(100.0 + index * 10.0, 100.0 + index, index / 4.0, True)
            for index in range(5)
        ]
        metrics = calibrate_fixed_deployment(ReferenceBridge(), predictor, cases)
        self.assertEqual(metrics.rmse_ft, 0.0)
        self.assertEqual(metrics.first_action_bias_ft, 0.0)
        self.assertEqual(metrics.p95_absolute_error_ft, 0.0)
        self.assertTrue(metrics.passed)


@unittest.skipUnless(BRIDGE_PATH.exists(), "production C bridge has not been built")
class ProductionBridgeTests(unittest.TestCase):
    def test_normal_bridge_accepts_legacy_version_zero_profile(self) -> None:
        from m5_control_calibration import make_bridge

        config = load_config()
        bridge = make_bridge(config, BRIDGE_PATH)
        try:
            prediction_m = bridge.predict(200.0, 100.0, 0.0, 1.0)
        finally:
            bridge.close()
        altitude_m = 200.0
        velocity_mps = 100.0
        drag_k = 0.5 * 1.225 * 0.012 / 5.0
        elapsed_s = 0.0
        while elapsed_s < 30.0 and velocity_mps > 0.0:
            acceleration = -9.80665 - drag_k * velocity_mps**2
            altitude_m += velocity_mps * 0.02 + 0.5 * acceleration * 0.02**2
            velocity_mps += acceleration * 0.02
            elapsed_s += 0.02
        expected_m = max(200.0, altitude_m)
        self.assertAlmostEqual(prediction_m, expected_m, places=3)

    def test_m5_c_predictor_has_monotonic_authority_and_meets_fixed_case_error(self) -> None:
        from m5_control_calibration import make_bridge

        config = load_config(VARIABLE_CONFIG)
        bridge = make_bridge(config, BRIDGE_PATH)
        try:
            closed_m = bridge.predict(200.0, 160.0, 0.0, 0.0)
            full_m = bridge.predict(200.0, 160.0, 0.0, 1.0)
            fixed_half_m = bridge.predict(200.0, 160.0, 0.5, 0.5)
            reference_half_m = reference_predict_apogee(
                config["predictor"], 200.0, 160.0, 0.5, 0.5
            )
        finally:
            bridge.close()
        self.assertGreater(closed_m, full_m)
        self.assertLess(abs(fixed_half_m - reference_half_m) * METERS_TO_FEET, 10.0)


if __name__ == "__main__":
    unittest.main()
