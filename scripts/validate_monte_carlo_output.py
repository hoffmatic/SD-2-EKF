"""Independently validate an AMBAR Monte Carlo evidence directory.

Overall role
------------
The campaign runner computes its own summary while it still has Python/RocketPy
objects in memory.  This script starts again from the durable CSV/JSON files and
recomputes the important denominators, rates, distributions, worst-run order,
identity joins, and snapshot hashes.  Keeping it independent catches reporting
bugs that an in-process assertion could repeat.

It does not rerun flight physics and therefore does not judge whether the
provisional aerodynamic assumptions are correct.  A PASS here means the saved
evidence bundle is internally consistent and reproducible, not flight-valid.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import statistics
from pathlib import Path
from typing import Any, Sequence


DESCRIBED_FAMILIES = {
    "controlled_apogee_ft": "controlled_apogee_ft",
    "target_error_ft": "target_error_ft",
    "absolute_target_error_ft": "absolute_target_error_ft",
    "apogee_reduction_ft": "apogee_reduction_ft",
}


def sha256_file(path: Path) -> str:
    """Return the uppercase digest format used by the campaign manifest."""

    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def percentile(values: Sequence[float], quantile: float) -> float:
    """Repeat the campaign's documented linear percentile convention."""

    ordered = sorted(values)
    position = quantile * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def describe(values: Sequence[float]) -> dict[str, float]:
    """Recompute every distribution statistic stored in summary.json."""

    return {
        "minimum": min(values),
        "p05": percentile(values, 0.05),
        "median": statistics.median(values),
        "mean": statistics.fmean(values),
        "p95": percentile(values, 0.95),
        "maximum": max(values),
        "sample_stddev": statistics.stdev(values) if len(values) > 1 else 0.0,
    }


def numerically_equal(left: Any, right: Any) -> bool:
    """Compare persisted numeric values with strict practical tolerance."""

    try:
        a = float(left)
        b = float(right)
    except (TypeError, ValueError):
        return left == right
    return math.isfinite(a) and math.isfinite(b) and math.isclose(
        a, b, rel_tol=1.0e-12, abs_tol=1.0e-12
    )


def validate(campaign_dir: Path) -> list[str]:
    """Return all evidence-integrity errors instead of stopping at the first."""

    errors: list[str] = []

    def require(condition: bool, message: str) -> None:
        if not condition:
            errors.append(message)

    required = (
        "runs.csv",
        "parameters.csv",
        "aggregate_metrics.csv",
        "summary.json",
        "manifest.json",
        "resolved_base_config.json",
        "resolved_study_config.json",
    )
    for name in required:
        require((campaign_dir / name).is_file(), f"missing required file: {name}")
    if errors:
        return errors

    with (campaign_dir / "runs.csv").open(encoding="utf-8", newline="") as handle:
        runs = list(csv.DictReader(handle))
    with (campaign_dir / "parameters.csv").open(
        encoding="utf-8", newline=""
    ) as handle:
        parameters = list(csv.DictReader(handle))
    summary = json.loads((campaign_dir / "summary.json").read_text(encoding="utf-8"))
    manifest = json.loads((campaign_dir / "manifest.json").read_text(encoding="utf-8"))

    require(bool(runs), "runs.csv contains no attempted runs")
    require(len(parameters) == len(runs), "parameters/runs row counts differ")
    require(manifest.get("status") == "completed", "manifest is not completed")
    require(
        int(manifest.get("completed_run_rows", -1)) == len(runs),
        "manifest completed_run_rows differs from runs.csv",
    )

    campaign_ids = {row.get("campaign_id") for row in runs}
    require(len(campaign_ids) == 1, "runs.csv contains multiple campaign IDs")
    if campaign_ids:
        require(
            manifest.get("campaign_id") in campaign_ids,
            "manifest campaign ID differs from runs.csv",
        )

    run_by_id = {row.get("run_id"): row for row in runs}
    require(len(run_by_id) == len(runs), "runs.csv contains duplicate run IDs")
    for parameter_row in parameters:
        run = run_by_id.get(parameter_row.get("run_id"))
        if run is None:
            errors.append(f"parameter row has no run: {parameter_row.get('run_id')}")
            continue
        for field in (
            "campaign_id",
            "run_index",
            "sample_index",
            "mode",
            "master_seed",
            "run_seed",
        ):
            require(
                parameter_row.get(field) == run.get(field),
                f"{run['run_id']} identity mismatch for {field}",
            )
        require(
            parameter_row.get("resolved_config_sha256") == run.get("config_sha256"),
            f"{run['run_id']} resolved-config hash mismatch",
        )

    randomized = [row for row in runs if row.get("mode") == "monte_carlo"]
    completed = [row for row in randomized if row.get("status") != "ERROR"]
    expected_counts = {
        "random_runs_attempted": len(randomized),
        "random_runs_completed": len(completed),
        "random_run_errors": len(randomized) - len(completed),
        "safety_pass_count": sum(row.get("safety_pass") == "true" for row in randomized),
        "effectiveness_pass_count": sum(
            row.get("effectiveness_pass") == "true" for row in randomized
        ),
        "target_hit_count": sum(
            row.get("target_band_pass") == "true" for row in randomized
        ),
        "envelope_pass_count": sum(
            row.get("flight_envelope_pass") == "true" for row in randomized
        ),
        "overall_pass_count": sum(row.get("run_pass") == "true" for row in randomized),
    }
    for key, expected in expected_counts.items():
        require(summary.get(key) == expected, f"summary mismatch for {key}")

    total = len(randomized)
    rate_pairs = {
        "safety_pass_rate": "safety_pass_count",
        "effectiveness_pass_rate": "effectiveness_pass_count",
        "target_hit_rate": "target_hit_count",
        "envelope_pass_rate": "envelope_pass_count",
        "overall_pass_rate": "overall_pass_count",
    }
    for rate_key, count_key in rate_pairs.items():
        expected = expected_counts[count_key] / total
        require(
            numerically_equal(summary.get(rate_key), expected),
            f"summary mismatch for {rate_key}",
        )

    for summary_key, csv_field in DESCRIBED_FAMILIES.items():
        values = [float(row[csv_field]) for row in completed]
        require(bool(values), f"no completed values for {csv_field}")
        if not values:
            continue
        recomputed = describe(values)
        for statistic_name, expected in recomputed.items():
            require(
                numerically_equal(
                    summary.get(summary_key, {}).get(statistic_name), expected
                ),
                f"summary mismatch for {summary_key}.{statistic_name}",
            )

    expected_worst = [
        row["run_id"]
        for row in sorted(
            completed,
            key=lambda item: float(item["absolute_target_error_ft"]),
            reverse=True,
        )[:5]
    ]
    require(summary.get("worst_run_ids") == expected_worst, "worst run order differs")

    aggregate = {
        row["metric"]: row["value"]
        for row in csv.DictReader(
            (campaign_dir / "aggregate_metrics.csv").open(encoding="utf-8")
        )
    }
    for key in (
        "campaign_pass",
        "baseline_reproducible",
        "random_runs_attempted",
        "safety_pass_count",
        "effectiveness_pass_count",
        "target_hit_count",
        "envelope_pass_count",
    ):
        require(
            str(summary.get(key)) == aggregate.get(key),
            f"aggregate_metrics.csv differs for {key}",
        )

    for record in manifest.get("inputs", {}).get("snapshots", []):
        snapshot = campaign_dir / record["snapshot"]
        require(snapshot.is_file(), f"snapshot is missing: {record['snapshot']}")
        if snapshot.is_file():
            require(
                sha256_file(snapshot) == record["sha256"],
                f"snapshot hash mismatch: {record['snapshot']}",
            )

    profiles = list((campaign_dir / "representative_profiles").glob("*.csv"))
    require(bool(profiles), "no representative replay profiles were written")
    for profile in profiles:
        text = profile.read_text(encoding="utf-8")
        require("Vertical velocity (m/s)" in text, f"{profile.name} lacks velocity")
        require(
            "Vertical acceleration (m/s^2)" in text,
            f"{profile.name} lacks acceleration",
        )
    return errors


def main() -> int:
    """Parse one directory, report every check, and return a CI-friendly code."""

    parser = argparse.ArgumentParser(
        description="Independently validate an AMBAR Monte Carlo output directory."
    )
    parser.add_argument("campaign_dir", type=Path)
    args = parser.parse_args()
    campaign_dir = args.campaign_dir.resolve()
    errors = validate(campaign_dir)
    if errors:
        print("Monte Carlo evidence validation: FAIL")
        for error in errors:
            print(f"- {error}")
        return 1
    print("Monte Carlo evidence validation: PASS")
    print(f"Campaign: {campaign_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
