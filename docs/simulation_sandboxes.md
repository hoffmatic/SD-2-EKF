# Simulation Sandboxes

This repo includes native desktop sandboxes, C++ unit tests, and a RocketPy
reference model that exercise current AMBAR software and hardware assumptions
without needing the real PCB.

These are not final flight predictions. They are virtual workbenches for asking:

- Does the estimator/controller respond sanely to noisy sensors?
- What happens if the airbrake actuator is slow, unhomed, or jammed?
- What should the electronics boot checklist catch before flight logic is armed?

Each sandbox now prints a test-report style log:

- `Condition being tested`: the failure mode or normal case injected into the
  virtual system.
- `Pass rule`: the exact behavior the software is expected to show.
- `Result`: `PASS` or `FAIL` for that scenario.
- `Measurements` or `Detailed check log`: the values that explain the result.
- `SUMMARY`: a compact table after the detailed logs.

For these sandboxes, `PASS` means the virtual software behaved as expected for
the injected condition. It does not mean the rocket, PCB, or airbrake mechanism
is flight-proven.

## Project Review Summary

Current strengths:

- The estimator/controller code is compact and portable.
- Hardware pin constants and datasheet constants are centralized.
- M5 report-derived requirement constants are centralized in
  `include/ambar_project_requirements.hpp`.
- Deployment is inhibited unless estimator health, phase, altitude, time, and
  apogee checks all pass.
- Barometer updates use innovation gating and Joseph-form covariance updates.

Current gaps:

- No real sensor, flash, radio, or TMC5240 drivers exist yet.
- The IMU body-axis orientation is still unknown.
- The airbrake mechanism travel, step/mm, homing method, and current limit are
  still only partly known. The report gives about 1 inch of concept travel and
  about 1.5 lead-screw rotations, but final step/mm and current limits are still
  placeholders.
- The embedded apogee predictor is still ballistic, although RocketPy now
  supplies an external drag-aware physics reference.
- Final RocketPy mass properties and drag curves remain provisional.
- The M5 2.4 GHz requirement matches the SX1280 hardware.
- The airbrake PCB uses a magnetometer; independent recovery GPS remains a
  separate vehicle-level requirement.
- The June 2 PCB is a provisional placed-but-unrouted baseline, not a
  manufacturing release.

## Build All Sandboxes

Preferred CMake flow:

```powershell
cmake -S . -B build
cmake --build build
```

The executable paths depend on the generator. With Visual Studio-style
generators they may be under `build\Debug\`. With single-config generators they
may be directly under `build\`.

Direct compiler checks on Windows:

```powershell
g++ -std=c++17 -Wall -Wextra -Wpedantic -I include src/ambar_airbrake.cpp sim/flight_sandbox.cpp -o build\sim_flight_sandbox.exe
g++ -std=c++17 -Wall -Wextra -Wpedantic -I include sim/electronics_sandbox.cpp -o build\sim_electronics_sandbox.exe
g++ -std=c++17 -Wall -Wextra -Wpedantic -I include sim/actuator_sandbox.cpp -o build\sim_actuator_sandbox.exe
```

Run the RocketPy physics suite:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_rocketpy_sim.ps1
```

The first run installs pinned Python dependencies into the ignored local
`.venv` directory.

## RocketPy Physics Sandbox

What it does:

- Uses RocketPy for standard pressure/temperature, report-backed constant wind,
  J420R thrust, variable motor mass,
  aerodynamic drag, rail departure, and 6-DOF trajectory integration.
- Applies deterministic provisional sensor bias, noise, quantization,
  saturation, and latency before feeding measurements into the real C++
  `AmbarFlightComputer` through `ambar_controller_bridge`.
- Applies the C++ deployment command to rate-limited RocketPy airbrakes.
- Verifies that deployment starts only after the J420R burn time plus the
  configured post-burn margin and only while the controller reports
  `AirbrakeActive`.
- Compares passive apogee against the M5 OpenRocket decision-matrix value.
- Checks the M5 Mach and rail-exit requirements.
- Writes machine-readable controller and trajectory data to
  `build/rocketpy-last-run.json`.

The current model uses June 2 OpenRocket geometry but retains provisional dry
mass, inertia, center of mass, and drag. It predicts 3851 ft passive versus the
3379 ft report value and 42.7 ft/s rail exit versus the 52 ft/s minimum, so both
checks fail visibly. Closed-loop apogee is 2979 ft, but that is not independent
accuracy evidence while the source-model checks fail.

The bridge configures a 1.74 s minimum boost interval and checks that deployment
does not begin before that time. The current first command occurs at 1.82 s.

## Fault and Replay Sandbox

Executable:

```powershell
.\build\sim_fault_replay_sandbox.exe
```

It verifies duplicate-timestamp rejection, NaN containment, inertial
propagation during a barometer dropout, fault latching, and deterministic replay
of `sim/replay/nominal_vertical_log.csv`. The replay file is synthetic and is
only a regression fixture.

## Fixed-Seed Monte Carlo Sandbox

Executable:

```powershell
.\build\sim_monte_carlo_sandbox.exe
```

It runs 200 deterministic 1-D trials while varying provisional boost, burn,
drag, sensor, and actuator parameters. All 200 current trials preserve health,
command bounds, post-burn gating, and descending inhibition. Only 15/200 fall
inside 3000 +/-100 ft; this is an explicit calibration warning, not a
source-backed probability estimate.

## Core Unit Tests

Executable:

```powershell
.\build\ambar_core_tests.exe
```

The assertions cover timestamp recovery, invalid-value rejection, barometer
gating, controller interlocks, fault latching/reset, and the current ballistic
predictor contract. CMake also exposes these and the native sandboxes through
`ctest --test-dir build --output-on-failure`.

## Flight Sandbox

Executable:

```powershell
.\build\sim_flight_sandbox.exe
```

What it does:

- Runs multiple simulated flight profiles through the real
  `AmbarFlightComputer` class.
- Injects IMU noise, IMU bias, barometer noise, and a barometer spike.
- Models actuator deployment lag so commanded airbrake and actual airbrake can
  differ.
- Reports true simulated apogee, target error, predicted apogee, peak command,
  peak actual deployment, rejected barometer samples, final phase, and health.
- Prints the condition under test, the expected pass rule, full-command timing,
  full-physical-deployment timing, and a target-calibration warning when apogee
  is outside the +/-100 ft mission tolerance.

Useful questions:

- Does a barometer spike get rejected?
- Does IMU bias push predicted apogee around?
- How does a slow actuator shift apogee and command timing?
- Does a weak motor stay inhibited instead of deploying unnecessarily?

## Electronics Sandbox

Executable:

```powershell
.\build\sim_electronics_sandbox.exe
```

What it does:

- Models boot-time checks based on the current V3 board constants.
- Checks MPM3606A input/load limits.
- Checks BMP388, LSM6DSV32X, and LIS2MDL I2C addresses and IDs.
- Checks LSM6DSV32X acceleration range against the M5 30G requirement.
- Checks W25Q64 JEDEC ID and log capacity.
- Checks two-hour log capacity against the current virtual record size.
- Checks SX1280 SPI mode and BUSY behavior.
- Checks the M5 2.4 GHz requirement against current SX1280 hardware.
- Checks that the airbrake magnetometer and independent recovery GPS roles are
  not incorrectly merged.
- Checks TMC5240 SPI mode, IOIN version, and motor supply presence.
- Checks for duplicate GPIO assignments in the board pin constants.
- Prints one detailed log per virtual boot condition showing each chip check,
  the expected value, the observed simulated value, and whether the boot decision
  should be `ARMABLE_WITH_WARNINGS` or `BLOCKED`.

Useful questions:

- Would the boot process catch a BMP388 address strap mismatch?
- Would the firmware catch a wrong flash part?
- What happens if the radio BUSY line never clears?
- Should flight logic stay inhibited if motor VBUS is missing?

## Actuator Sandbox

Executable:

```powershell
.\build\sim_actuator_sandbox.exe
```

What it does:

- Models an airbrake actuator as a virtual stepper mechanism.
- Uses report-backed concept travel of 25.4 mm and placeholder 400 steps/mm.
- Runs nominal, slow-motor, not-homed, and jammed scenarios.
- Reports peak deployment, final deployment, peak current, homing state, and
  fault reason.
- Prints first-motion time, full-deployment time, pass rule, and whether the
  virtual actuator should fault.

Useful questions:

- Does the mechanism retract when commands are inhibited?
- Does an unhomed actuator refuse deployment?
- Does a jam create a high-current/fault condition?
- How much deployment is lost when motor speed is too low?

## Next Simulation Work

1. Replace RocketPy placeholder mass/inertia and actuator values with measured data.
2. Import the final OpenRocket `.ork` and drag-vs-Mach curves for cross-validation.
3. Add plots of altitude, velocity, predicted apogee, and command to the UI.
4. Add real sensor-driver unit tests once BMP388 and LSM6DSV32X drivers exist.
5. Replace the synthetic replay with captured hardware logs.
6. Add hardware-in-the-loop tests that compare these virtual checks with actual
   boot telemetry from the PCB.
7. Verify the separate recovery GPS hardware and SX1280-compatible ground
   station outside this repo.

See [simulation_audit.md](simulation_audit.md) for the prioritized coverage map
and the project data needed to make each proposed simulation meaningful.
