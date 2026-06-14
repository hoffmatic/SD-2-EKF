"""Local server and process dispatcher for the AMBAR simulation console.

The server intentionally uses only Python's standard library so team members do
not need npm or a Python package install. It serves the static UI and exposes one
local-only API that runs the existing sandbox executables or build scripts.

Architecture connections:
- ui/index.html, ui/styles.css, and ui/app.js are the browser client.
- /api/run maps UI requests to the PowerShell scripts or native executables.
- /api/last-run restores the latest report after a browser refresh.
- The server never implements flight logic; it only launches and reports it.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parent.parent
UI_ROOT = REPO_ROOT / "ui"
BUILD_ROOT = REPO_ROOT / "build"
LAST_RUN_PATH = BUILD_ROOT / "ui-last-run.json"
SERVER_INFO_PATH = BUILD_ROOT / "ui-server.json"
BASE_CONFIG_PATH = REPO_ROOT / "sim" / "rocketpy" / "ambar_reference_config.json"
UI_OVERRIDES_PATH = BUILD_ROOT / "ui-rocketpy-overrides.json"

FEET_TO_METERS = 0.3048
POUNDS_TO_KG = 0.45359237
INCHES_TO_METERS = 0.0254


def field_spec(
    field_id: str,
    label: str,
    group: str,
    unit: str,
    minimum: float,
    maximum: float,
    step: float,
    path: tuple[str, ...],
    source: str,
    to_config=lambda value: value,
    from_config=lambda value: value,
    integer: bool = False,
) -> dict:
    return {
        "id": field_id,
        "label": label,
        "group": group,
        "unit": unit,
        "minimum": minimum,
        "maximum": maximum,
        "step": step,
        "path": path,
        "source": source,
        "to_config": to_config,
        "from_config": from_config,
        "integer": integer,
    }


INPUT_SPECS = [
    field_spec("targetApogeeFt", "Target apogee", "Mission", "ft", 500, 12500, 10, ("requirements", "target_apogee_ft"), "Mission setting"),
    field_spec("railLengthFt", "Rail length", "Launch", "ft", 4, 30, 0.25, ("environment", "rail_length_m"), "M5 report", lambda value: value * FEET_TO_METERS, lambda value: value / FEET_TO_METERS),
    field_spec("launchAngleFromVerticalDeg", "Angle from vertical", "Launch", "deg", 0, 20, 0.5, ("environment", "inclination_deg"), "Launch setting", lambda value: 90.0 - value, lambda value: 90.0 - value),
    field_spec("headingDeg", "Heading", "Launch", "deg", 0, 360, 1, ("environment", "heading_deg"), "M5 report"),
    field_spec("dryMassLb", "Dry mass", "Vehicle", "lb", 1, 50, 0.1, ("rocket", "dry_mass_kg"), "Placeholder", lambda value: value * POUNDS_TO_KG, lambda value: value / POUNDS_TO_KG),
    field_spec("powerOnDragCoefficient", "Power-on drag coefficient", "Vehicle", "Cd", 0.05, 2.5, 0.01, ("rocket", "power_on_drag_coefficient"), "Placeholder"),
    field_spec("powerOffDragCoefficient", "Power-off drag coefficient", "Vehicle", "Cd", 0.05, 2.5, 0.01, ("rocket", "power_off_drag_coefficient"), "Placeholder"),
    field_spec("finCount", "Stabilizing fins", "Vehicle", "count", 3, 8, 1, ("rocket", "fin_count"), "M5 report", integer=True),
    field_spec("finRootChordIn", "Fin root chord", "Vehicle", "in", 1, 24, 0.1, ("rocket", "fin_root_chord_m"), "M5 report", lambda value: value * INCHES_TO_METERS, lambda value: value / INCHES_TO_METERS),
    field_spec("finTipChordIn", "Fin tip chord", "Vehicle", "in", 0.1, 24, 0.1, ("rocket", "fin_tip_chord_m"), "M5 report", lambda value: value * INCHES_TO_METERS, lambda value: value / INCHES_TO_METERS),
    field_spec("finSpanIn", "Fin span", "Vehicle", "in", 0.5, 20, 0.1, ("rocket", "fin_span_m"), "M5 report", lambda value: value * INCHES_TO_METERS, lambda value: value / INCHES_TO_METERS),
    field_spec("postBurnMarginS", "Post-burn inhibit margin", "Airbrake", "s", 0, 3, 0.05, ("airbrakes", "post_burn_enable_margin_s"), "Safety setting"),
    field_spec("maximumDeploymentRatePercentS", "Maximum deployment rate", "Airbrake", "%/s", 5, 500, 5, ("airbrakes", "maximum_rate_fraction_per_s"), "Placeholder", lambda value: value / 100.0, lambda value: value * 100.0),
    field_spec("fullDeploymentDragCoefficient", "Full-deployment drag increment", "Airbrake", "Cd", 0, 3, 0.01, ("airbrakes", "drag_coefficient_at_full_deployment"), "Placeholder"),
    field_spec("controllerRateHz", "Controller rate", "Sensors", "Hz", 10, 1000, 10, ("airbrakes", "sampling_rate_hz"), "Model setting"),
    field_spec("barometerRateHz", "Barometer rate", "Sensors", "Hz", 1, 200, 1, ("airbrakes", "barometer_rate_hz"), "Model setting"),
    field_spec("barometerStdDevFt", "Barometer standard deviation", "Sensors", "ft", 0.01, 100, 0.1, ("airbrakes", "barometer_std_dev_m"), "Placeholder", lambda value: value * FEET_TO_METERS, lambda value: value / FEET_TO_METERS),
]

SUITE_EXECUTABLES = {
    "flight": BUILD_ROOT / "sim_flight_sandbox.exe",
    "electronics": BUILD_ROOT / "sim_electronics_sandbox.exe",
    "actuator": BUILD_ROOT / "sim_actuator_sandbox.exe",
}


def value_at_path(values: dict, path: tuple[str, ...]):
    current = values
    for key in path:
        current = current[key]
    return current


def set_at_path(values: dict, path: tuple[str, ...], value) -> None:
    current = values
    for key in path[:-1]:
        current = current.setdefault(key, {})
    current[path[-1]] = value


def baseline_inputs() -> dict[str, float | int]:
    config = json.loads(BASE_CONFIG_PATH.read_text(encoding="utf-8"))
    result = {}
    for spec in INPUT_SPECS:
        value = spec["from_config"](value_at_path(config, spec["path"]))
        result[spec["id"]] = int(round(value)) if spec["integer"] else round(float(value), 6)
    return result


def public_input_schema() -> list[dict]:
    private_keys = {"path", "to_config", "from_config", "integer"}
    return [{key: value for key, value in spec.items() if key not in private_keys} for spec in INPUT_SPECS]


def validate_inputs(requested: object) -> tuple[dict[str, float | int], dict]:
    if requested is None:
        requested = {}
    if not isinstance(requested, dict):
        raise ValueError("Simulation inputs must be a JSON object.")

    specs_by_id = {spec["id"]: spec for spec in INPUT_SPECS}
    unknown = sorted(set(requested) - set(specs_by_id))
    if unknown:
        raise ValueError("Unknown simulation inputs: " + ", ".join(unknown))

    normalized = baseline_inputs()
    overrides: dict = {}
    for field_id, raw_value in requested.items():
        spec = specs_by_id[field_id]
        if isinstance(raw_value, bool):
            raise ValueError(f"{spec['label']} must be numeric.")
        try:
            numeric = float(raw_value)
        except (TypeError, ValueError) as error:
            raise ValueError(f"{spec['label']} must be numeric.") from error
        if not math.isfinite(numeric):
            raise ValueError(f"{spec['label']} must be finite.")
        if numeric < spec["minimum"] or numeric > spec["maximum"]:
            raise ValueError(
                f"{spec['label']} must be between {spec['minimum']} and {spec['maximum']} {spec['unit']}."
            )
        if spec["integer"] and not numeric.is_integer():
            raise ValueError(f"{spec['label']} must be a whole number.")
        normalized[field_id] = int(numeric) if spec["integer"] else numeric

    for spec in INPUT_SPECS:
        set_at_path(overrides, spec["path"], spec["to_config"](normalized[spec["id"]]))
    return normalized, overrides


def command_for_run(suite: str, rebuild: bool, overrides_path: Path | None = None) -> list[str]:
    """Map a UI suite name to the same commands used from a terminal."""
    if suite == "all":
        command = [
            "powershell",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(REPO_ROOT / "scripts" / "run_all_simulations.ps1"),
        ]
        if not rebuild:
            command.append("-SkipBuild")
        if overrides_path is not None:
            command.extend(["-OverridesPath", str(overrides_path)])
        return command

    if suite == "rocketpy":
        command = [
            "powershell",
            "-NoProfile",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
            str(REPO_ROOT / "scripts" / "run_rocketpy_sim.ps1"),
        ]
        if not rebuild:
            command.append("-SkipBuild")
        if overrides_path is not None:
            command.extend(["-OverridesPath", str(overrides_path)])
        return command

    executable = SUITE_EXECUTABLES.get(suite)
    if executable is None:
        raise ValueError(f"Unknown suite: {suite}")

    if rebuild or not executable.exists():
        # The existing build script is the source of truth for compiler discovery.
        # Build all targets once, then run only the requested executable for the UI.
        build = subprocess.run(
            [
                "powershell",
                "-NoProfile",
                "-ExecutionPolicy",
                "Bypass",
                "-File",
                str(REPO_ROOT / "scripts" / "run_sandboxes.ps1"),
            ],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=180,
        )
        if build.returncode != 0:
            raise RuntimeError(build.stdout + "\n" + build.stderr)

    return [str(executable)]


class SimulationHandler(SimpleHTTPRequestHandler):
    """Serve static assets and the two local JSON API endpoints."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(UI_ROOT), **kwargs)

    def log_message(self, format_string: str, *args) -> None:
        sys.stdout.write("[ui] " + format_string % args + "\n")

    def send_json(self, payload: dict, status: int = 200) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        if self.path == "/api/health":
            # Expose only a stable project identifier, not a user's local path.
            self.send_json({"ok": True, "project": "SD-2-EKF"})
            return
        if self.path == "/api/last-run":
            if LAST_RUN_PATH.exists():
                try:
                    self.send_json(json.loads(LAST_RUN_PATH.read_text(encoding="utf-8")))
                except (OSError, json.JSONDecodeError):
                    self.send_json({"lastRun": None})
            else:
                self.send_json({"lastRun": None})
            return
        if self.path == "/api/inputs":
            self.send_json(
                {
                    "baseline": baseline_inputs(),
                    "fields": public_input_schema(),
                    "fixedCriteria": {
                        "maximumMach": 1.0,
                        "minimumRailExitVelocityFps": 52.0,
                        "reportPassiveApogeeFt": 3379.0,
                        "targetToleranceFt": 100.0,
                    },
                }
            )
            return
        super().do_GET()

    def do_POST(self) -> None:
        if self.path != "/api/run":
            self.send_json({"error": "Not found"}, status=404)
            return

        try:
            content_length = int(self.headers.get("Content-Length", "0"))
            request = json.loads(self.rfile.read(content_length) or b"{}")
            suite = str(request.get("suite", "all"))
            rebuild = bool(request.get("rebuild", False))
            normalized_inputs, overrides = validate_inputs(request.get("inputs"))
            input_mode = "report-baseline"
            overrides_path = None
            if normalized_inputs != baseline_inputs():
                BUILD_ROOT.mkdir(parents=True, exist_ok=True)
                UI_OVERRIDES_PATH.write_text(json.dumps(overrides, indent=2), encoding="utf-8")
                overrides_path = UI_OVERRIDES_PATH
                input_mode = "experimental-overrides"
            command = command_for_run(suite, rebuild, overrides_path)

            start = time.monotonic()
            result = subprocess.run(
                command,
                cwd=REPO_ROOT,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=240,
            )
            duration = time.monotonic() - start
            output = result.stdout
            if result.stderr:
                output += "\n" + result.stderr

            payload = {
                "suite": suite,
                "rebuild": rebuild,
                "exitCode": result.returncode,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "durationSeconds": duration,
                "output": output,
                "inputMode": input_mode,
                "appliedInputs": normalized_inputs,
            }
            BUILD_ROOT.mkdir(parents=True, exist_ok=True)
            LAST_RUN_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            # A completed simulation may legitimately fail an engineering check.
            # Return its report normally so the UI can render the failed cases.
            self.send_json(payload)
        except subprocess.TimeoutExpired:
            self.send_json({"error": "Simulation timed out after four minutes."}, status=504)
        except (ValueError, RuntimeError, OSError, json.JSONDecodeError) as error:
            self.send_json({"error": str(error), "exitCode": 1}, status=500)


def find_port(start_port: int) -> int:
    for port in range(start_port, start_port + 20):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as candidate:
            try:
                candidate.bind(("127.0.0.1", port))
                return port
            except OSError:
                continue
    raise RuntimeError("No open local port found for the simulation UI.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the AMBAR simulation console.")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    if not UI_ROOT.exists():
        raise SystemExit(f"UI directory is missing: {UI_ROOT}")

    port = find_port(args.port)
    BUILD_ROOT.mkdir(parents=True, exist_ok=True)
    SERVER_INFO_PATH.write_text(
        json.dumps({"port": port, "pid": os.getpid(), "url": f"http://127.0.0.1:{port}"}, indent=2),
        encoding="utf-8",
    )
    server = ThreadingHTTPServer(("127.0.0.1", port), SimulationHandler)
    print(f"AMBAR Simulation Console: http://127.0.0.1:{port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
