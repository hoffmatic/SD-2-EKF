"""Validate the RocketPy machine-readable result independently of UI parsing."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


REQUIRED_SAMPLE_FIELDS = {
    "time_s",
    "truth_altitude_m",
    "truth_velocity_mps",
    "truth_speed_mps",
    "truth_acceleration_mps2",
    "measured_acceleration_mps2",
    "estimated_altitude_m",
    "estimated_velocity_mps",
    "command_fraction",
    "actual_deployment_fraction",
    "phase",
    "healthy",
    "barometer_sample",
}


def validate(path: Path) -> list[str]:
    data = json.loads(path.read_text(encoding="utf-8"))
    errors: list[str] = []
    if data.get("schemaVersion") != 2:
        errors.append("schemaVersion must be 2")

    log = data.get("controllerLog")
    if not isinstance(log, list) or len(log) < 100:
        return errors + ["controllerLog must contain at least 100 samples"]

    previous_time = -math.inf
    phases: set[str] = set()
    barometer_samples = 0
    for index, sample in enumerate(log):
        missing = REQUIRED_SAMPLE_FIELDS - set(sample)
        if missing:
            errors.append(f"sample {index} missing fields: {', '.join(sorted(missing))}")
            continue
        timestamp = sample["time_s"]
        if not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp):
            errors.append(f"sample {index} has invalid time_s")
        elif timestamp <= previous_time:
            errors.append(f"sample {index} timestamp is not strictly increasing")
        previous_time = timestamp
        phases.add(str(sample["phase"]))
        barometer_samples += int(bool(sample["barometer_sample"]))
        for field in ("command_fraction", "actual_deployment_fraction"):
            value = sample[field]
            if not isinstance(value, (int, float)) or not 0.0 <= value <= 1.0:
                errors.append(f"sample {index} has invalid {field}")

    for required_phase in ("PadIdle", "Boost", "Recovery"):
        if required_phase not in phases:
            errors.append(f"controllerLog never enters {required_phase}")
    if not ({"Coast", "AirbrakeActive"} & phases):
        errors.append("controllerLog never enters a coast-control phase")
    if barometer_samples == 0 or barometer_samples >= len(log):
        errors.append("barometer sampling must be present and slower than controller sampling")
    if data.get("logIntegrityErrors"):
        errors.append("simulation reported internal log-integrity errors")
    if not data.get("acceptance", {}).get("timeHistoryPass", False):
        errors.append("timeHistoryPass is false")
    return errors


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("path", type=Path)
    args = parser.parse_args()
    errors = validate(args.path)
    if errors:
        print("RocketPy output validation: FAIL")
        for error in errors:
            print(f"- {error}")
        return 1
    print("RocketPy output validation: PASS")
    print("The structured log is complete, monotonic, bounded, and covers post-apogee Recovery.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
