# Simulation Console UI

The browser-based AMBAR Simulation Console runs legacy C++ sandbox executables
plus the production STM32-C/RocketPy physics backend and presents their output as scenario tables, status summaries,
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

- `Run All` runs the quick core assertions, fault/replay, legacy 1-D Monte
  Carlo, flight, electronics, actuator, and one RocketPy check. It intentionally
  excludes the several-minute production 50-run campaign.
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
- `Flight Data` plots the structured RocketPy/STM32-C time history. The altitude,
  speed, vertical-acceleration, and deployment graphs use phase-colored
  backgrounds and distinguish trajectory truth, virtual sensor measurements,
  and production EKF estimates. Hover over a graph for the time, phase, and channel
  values. CSV and JSON downloads preserve the complete sample log.

## Adjustable Inputs

The Inputs view groups controls by their engineering role:

- Mission: target apogee.
- Launch: rail length, angle from vertical, heading, constant wind speed, and
  wind-from direction.
- Vehicle: dry mass, powered/coast drag coefficients, and stabilizing-fin geometry.
- Airbrake: fixed controller minimum-boost time, deployment-rate limit, and
  full-deployment drag increment.
- Sensors: controller/barometer rates plus provisional accelerometer bias/noise,
  barometer bias/noise, and latency.

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

The UI does not change the simulation math. It calls the legacy C++ sandboxes
and production STM32-C RocketPy runner, parses their pass/fail output, and reads
`build/rocketpy-last-run.json` for the plotted time series. The RocketPy output
has a separate validator that checks timestamps, fields, bounds, sampling, and
post-apogee phase coverage before the dashboard marks the data check as passed.

The acceleration graph keeps world-vertical truth separate from the virtual
sensor. The sensor channel projects specific force into RocketPy body Z,
subtracts its pad reference, then applies configured noise/bias/latency. This
matches the current firmware's single-axis contract more closely, but still
does not model vibration, mounting error, or raw sensor registers.

Run the production robustness campaign separately with
`Run Monte Carlo Simulation.cmd`; see
[monte_carlo_campaign.md](monte_carlo_campaign.md).
