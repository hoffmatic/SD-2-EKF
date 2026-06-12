"""Local server for the AMBAR simulation console.

The server intentionally uses only Python's standard library so team members do
not need npm or a Python package install. It serves the static UI and exposes one
local-only API that runs the existing sandbox executables or build script.
"""

from __future__ import annotations

import argparse
import json
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

SUITE_EXECUTABLES = {
    "flight": BUILD_ROOT / "sim_flight_sandbox.exe",
    "electronics": BUILD_ROOT / "sim_electronics_sandbox.exe",
    "actuator": BUILD_ROOT / "sim_actuator_sandbox.exe",
}


def command_for_run(suite: str, rebuild: bool) -> list[str]:
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
            self.send_json({"ok": True, "repo": str(REPO_ROOT)})
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
            command = command_for_run(suite, rebuild)

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
            }
            BUILD_ROOT.mkdir(parents=True, exist_ok=True)
            LAST_RUN_PATH.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            self.send_json(payload, status=200 if result.returncode == 0 else 500)
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
