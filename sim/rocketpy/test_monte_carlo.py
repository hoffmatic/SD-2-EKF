"""Deterministic unit tests for the AMBAR campaign orchestration layer.

These tests do not run RocketPy trajectories.  They protect the evidence logic
that is easiest to get subtly wrong: sampling coverage, seed reproducibility,
phase legality, failed-run denominators, and non-mutating config application.
The integration command in the operator guide performs the physics/bridge test.
"""

from __future__ import annotations

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path


MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

from run_monte_carlo import (  # noqa: E402 - path setup is intentional.
    TrialArtifact,
    apply_sampled_inputs,
    build_latin_hypercube,
    derive_run_seed,
    illegal_phase_transitions,
    load_study_config,
    summarize_campaign,
)
from run_rocketpy_sim import (  # noqa: E402
    deep_merge,
    flight_apogee_agl_m,
    load_config,
    pad_referenced_body_axis_acceleration,
    rocket_body_z_axis,
)


class SamplingTests(unittest.TestCase):
    """Check that randomized cases are reproducible and cover every stratum."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.study = load_study_config(MODULE_DIR / "monte_carlo_config.json")

    def test_same_seed_produces_identical_parameter_table(self) -> None:
        first = build_latin_hypercube(self.study["parameters"], 50, 12345)
        second = build_latin_hypercube(self.study["parameters"], 50, 12345)
        self.assertEqual(first, second)

    def test_different_seed_changes_parameter_table(self) -> None:
        first = build_latin_hypercube(self.study["parameters"], 10, 111)
        second = build_latin_hypercube(self.study["parameters"], 10, 222)
        self.assertNotEqual(first, second)

    def test_uniform_parameter_uses_every_latin_hypercube_stratum(self) -> None:
        parameter = next(
            value
            for value in self.study["parameters"]
            if value["name"] == "wind_from_direction_deg"
        )
        run_count = 50
        samples = build_latin_hypercube([parameter], run_count, 123)
        low = float(parameter["minimum"])
        high = float(parameter["maximum"])
        strata = []
        for sample in samples:
            unit = (sample[parameter["name"]] - low) / (high - low)
            strata.append(min(run_count - 1, int(unit * run_count)))
        self.assertEqual(sorted(strata), list(range(run_count)))

    def test_run_seed_depends_only_on_master_seed_and_index(self) -> None:
        self.assertEqual(derive_run_seed(42, 7), derive_run_seed(42, 7))
        self.assertNotEqual(derive_run_seed(42, 7), derive_run_seed(42, 8))
        self.assertNotEqual(derive_run_seed(42, 7), derive_run_seed(43, 7))


class ConfigurationTests(unittest.TestCase):
    """Ensure sampled truth changes do not leak back into controller baselines."""

    def test_apply_sampled_inputs_does_not_mutate_base_config(self) -> None:
        study = load_study_config(MODULE_DIR / "monte_carlo_config.json")
        base = load_config()
        original = copy.deepcopy(base)
        sampled = build_latin_hypercube(study["parameters"], 1, 9)[0]
        resolved = apply_sampled_inputs(
            base, study["parameters"], sampled, run_seed=987654321
        )
        self.assertEqual(base, original)
        self.assertNotEqual(resolved, base)
        self.assertEqual(resolved["sensor_model"]["random_seed"], 987654321)
        self.assertEqual(resolved["controller"], base["controller"])
        self.assertNotEqual(
            resolved["motor"]["burn_time_s"],
            resolved["controller"]["minimum_boost_time_s"],
        )

    def test_rocketpy_apogee_is_converted_from_asl_to_agl(self) -> None:
        class FakeFlight:
            apogee = 1007.0104

        config = {"environment": {"elevation_m": 7.0104}}
        self.assertAlmostEqual(flight_apogee_agl_m(FakeFlight(), config), 1000.0)

    def test_vertical_body_axis_reproduces_vertical_kinematic_acceleration(self) -> None:
        state = [0.0] * 13
        state[6] = 1.0  # identity quaternion: body Z is world Z
        body_axis = rocket_body_z_axis(state)
        self.assertEqual(body_axis, (0.0, 0.0, 1.0))
        measured = pad_referenced_body_axis_acceleration(
            (0.0, 0.0, 12.0), body_axis, 9.80665
        )
        self.assertAlmostEqual(measured, 12.0)

    def test_unknown_override_key_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "dry_mas_kg"):
            deep_merge(
                {"rocket": {"dry_mass_kg": 3.0}},
                {"rocket": {"dry_mas_kg": 3.2}},
            )

    def test_duplicate_sampled_config_path_is_rejected(self) -> None:
        study = load_study_config(MODULE_DIR / "monte_carlo_config.json")
        duplicate = copy.deepcopy(study["parameters"][0])
        duplicate["name"] = "duplicate_mass"
        study["parameters"].append(duplicate)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "study.json"
            path.write_text(json.dumps(study), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "sampled more than once"):
                load_study_config(path)

    def test_randomized_controller_path_is_rejected(self) -> None:
        study = load_study_config(MODULE_DIR / "monte_carlo_config.json")
        study["parameters"][0]["paths"] = ["controller.minimum_boost_time_s"]
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "study.json"
            path.write_text(json.dumps(study), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "fixed controller"):
                load_study_config(path)


class AcceptanceTests(unittest.TestCase):
    """Protect strict phase ordering and campaign-denominator semantics."""

    def test_recovery_to_boost_is_illegal(self) -> None:
        transitions = [
            {"phase": "PadIdle"},
            {"phase": "Boost"},
            {"phase": "Coast"},
            {"phase": "AirbrakeActive"},
            {"phase": "Recovery"},
            {"phase": "Boost"},
        ]
        self.assertEqual(illegal_phase_transitions(transitions), ["Recovery->Boost"])

    def test_immediate_boost_to_airbrake_active_is_accepted(self) -> None:
        transitions = [
            {"phase": "PadIdle"},
            {"phase": "Boost"},
            {"phase": "AirbrakeActive"},
            {"phase": "Recovery"},
        ]
        self.assertEqual(illegal_phase_transitions(transitions), [])

    def test_execution_error_remains_in_random_run_denominator(self) -> None:
        study = load_study_config(MODULE_DIR / "monte_carlo_config.json")
        baseline_row = {
            "run_id": "baseline-001",
            "mode": "baseline",
            "status": "PASS",
            "passive_apogee_ft": 3379.0,
            "controlled_apogee_ft": 3000.0,
            "target_error_ft": 0.0,
            "absolute_target_error_ft": 0.0,
            "apogee_reduction_ft": 379.0,
            "phase_sequence": "PadIdle->Boost->AirbrakeActive->Recovery",
            "safety_pass": "true",
            "target_band_pass": "true",
            "flight_envelope_pass": "true",
            "run_pass": "true",
        }
        passing_random = dict(
            baseline_row, run_id="monte-carlo-001", mode="monte_carlo"
        )
        error_random = {
            "run_id": "monte-carlo-002",
            "mode": "monte_carlo",
            "status": "ERROR",
            "safety_pass": "false",
            "target_band_pass": "false",
            "flight_envelope_pass": "false",
            "run_pass": "false",
        }
        artifacts = [
            TrialArtifact(baseline_row, {}, {}, []),
            TrialArtifact(passing_random, {}, {}, []),
            TrialArtifact(error_random, {}, {}, []),
        ]
        summary = summarize_campaign(
            artifacts, study["acceptance"], random_run_count=2
        )
        self.assertEqual(summary["random_runs_attempted"], 2)
        self.assertEqual(summary["random_runs_completed"], 1)
        self.assertEqual(summary["random_run_errors"], 1)
        self.assertEqual(summary["target_hit_count"], 1)
        self.assertEqual(summary["target_hit_rate"], 0.5)

    def test_repeatability_requires_two_baseline_runs(self) -> None:
        study = load_study_config(MODULE_DIR / "monte_carlo_config.json")
        random_row = {
            "run_id": "monte-carlo-001",
            "mode": "monte_carlo",
            "status": "ERROR",
            "safety_pass": "false",
            "target_band_pass": "false",
            "flight_envelope_pass": "false",
            "run_pass": "false",
        }
        summary = summarize_campaign(
            [TrialArtifact(random_row, {}, {}, [])],
            study["acceptance"],
            random_run_count=1,
        )
        self.assertFalse(summary["baseline_reproducible"])

    def test_effectiveness_failure_prevents_campaign_pass(self) -> None:
        study = load_study_config(MODULE_DIR / "monte_carlo_config.json")
        baseline = {
            "run_id": "baseline-001",
            "mode": "baseline",
            "status": "FAIL",
            "passive_reference_pass": "true",
            "passive_apogee_ft": 3379.0,
            "controlled_apogee_ft": 3000.0,
            "target_error_ft": 0.0,
            "absolute_target_error_ft": 0.0,
            "apogee_reduction_ft": 379.0,
            "phase_sequence": "PadIdle->Boost->Coast->Recovery",
            "controller_log_sha256": "ABC",
        }
        second_baseline = dict(baseline, run_id="baseline-002")
        randomized = dict(
            baseline,
            run_id="monte-carlo-001",
            mode="monte_carlo",
            status="FAIL",
            safety_pass="true",
            effectiveness_pass="false",
            target_band_pass="true",
            flight_envelope_pass="true",
            run_pass="false",
        )
        summary = summarize_campaign(
            [
                TrialArtifact(baseline, {}, {}, []),
                TrialArtifact(second_baseline, {}, {}, []),
                TrialArtifact(randomized, {}, {}, []),
            ],
            study["acceptance"],
            random_run_count=1,
        )
        self.assertTrue(summary["baseline_reproducible"])
        self.assertEqual(summary["effectiveness_pass_rate"], 0.0)
        self.assertFalse(summary["campaign_pass"])


if __name__ == "__main__":
    unittest.main()
