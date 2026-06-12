# Simulation Console UI

The browser-based AMBAR Simulation Console runs the existing C++ sandbox
executables plus the RocketPy physics backend and presents their output as scenario tables, status summaries,
details, source gaps, and a raw terminal log.

## Start The UI

From the repository folder, run:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_simulation_ui.ps1
```

Keep that PowerShell window open. Open the local URL printed by the script,
normally:

```text
http://127.0.0.1:8765
```

## Controls

- `Run All` runs the flight, RocketPy physics, electronics, and actuator suites.
- `Run Suite` runs only the currently selected suite.
- `Rebuild` recompiles before running. Leave it off for faster repeat runs.
- Select a scenario row to inspect its condition, pass rule, measurements, and
  interpretation.
- Open `Raw Output` when the full terminal log is needed for debugging or review.
- `Sources` separates report/datasheet-backed facts from missing calibration
  inputs and unresolved requirements.

The UI does not change the simulation math. It calls the C++ sandboxes and the
RocketPy runner, then parses their human-readable output.
