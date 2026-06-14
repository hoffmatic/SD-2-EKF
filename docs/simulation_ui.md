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
- `Inputs` exposes the RocketPy assumptions that are useful for trade studies.
  Change values, select `Run RocketPy`, and the results will be labeled as an
  experimental-input run. `Reset Baseline` restores the reviewed configuration.

## Adjustable Inputs

The Inputs view groups controls by their engineering role:

- Mission: target apogee.
- Launch: rail length, angle from vertical, and heading.
- Vehicle: dry mass, powered/coast drag coefficients, and stabilizing-fin geometry.
- Airbrake: post-burn inhibit margin, deployment-rate limit, and full-deployment
  drag increment.
- Sensors: controller and barometer rates plus simulated barometer noise.

Every field has a bounded numeric range enforced by the local Python server.
Values outside that range are rejected before RocketPy starts. A changed value
is marked `Modified`, and the run output prints the applied values so a result
can be reproduced.

The target tolerance, Mach limit, minimum rail-exit velocity, and current M5
passive-apogee reference are displayed as fixed criteria. They cannot be edited
from the UI. This prevents a trade-study input from silently weakening a
pass/fail rule.

Input changes are temporary. The UI writes `build/ui-rocketpy-overrides.json`
for the selected run; it does not edit `sim/rocketpy/ambar_reference_config.json`.
Measured mass properties and aerodynamic calibration should still be committed
to the reference configuration after team review.

The UI does not change the simulation math. It calls the C++ sandboxes and the
RocketPy runner, then parses their human-readable output.
