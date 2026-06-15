# SD-2-EKF

Embedded-oriented estimator and airbrake-control scaffold for Project AMBAR.

The code now follows the AMBAR M5 report: a 3-inch test vehicle targeting
3000 ft apogee with +/-100 ft tolerance, autonomous airbrake deployment after
burnout, and live flight-state telemetry.

## What Is Implemented

- 4-state vertical EKF: altitude, vertical velocity, accelerometer bias, and
  barometer bias
- Timestamp validation for asynchronous IMU input
- Barometer innovation gating and Joseph-form covariance update
- Flight phase tracking: pad idle, boost, coast, airbrake active, recovery, fault
- Apogee prediction and bounded airbrake deployment command
- Embedded-friendly interfaces with fixed-size arrays and no heap allocation
- June 2 provisional KiCad pin-map constants in `include/ambar_board_pins.hpp`
- Datasheet-derived device IDs, bus modes, addresses, and register constants in
  `include/ambar_device_constants.hpp`
- Virtual sandboxes for flight behavior, electronics bring-up checks, and
  actuator motion/fault behavior
- RocketPy 1.12.1 trajectory physics using the M5-selected J420R thrust curve
  and a persistent bridge to the real C++ estimator/controller
- Deterministic timestamp/sensor fault injection, synthetic log replay, and a
  200-trial fixed-seed Monte Carlo safety study
- CTest regression tests and GitHub Actions verification

## Current Status

This is an integration scaffold, not flight-ready software.

The June 14 M5 report cross-check currently fails: RocketPy predicts 3851 ft
passive apogee versus 3379 ft in the report and only 42.7 ft/s off the reported
72-inch rail. The closed-loop result is 2973 ft, but that does not validate
target accuracy while the passive and rail-exit checks fail. These mismatches
remain visible until the OpenRocket configuration, mass properties, and drag
data are reconciled.

Before use in flight hardware, add:

- Board support for the actual IMU, barometer, radio, logger, and actuator
- IMU body-axis alignment and gravity compensation
- Final measured mass properties and drag curves for the RocketPy model
- A drag-aware embedded apogee predictor calibrated from RocketPy/OpenRocket and flight logs
- TMC5240 motor driver limits, current monitoring, and fail-safe behavior
- Hardware-in-the-loop tests and recorded hardware-log replay
- Telemetry packet definitions for estimate, phase, command, and health

## Build

If you are new to code or GitHub, start here:
[docs/beginner_quick_start.md](docs/beginner_quick_start.md).

Windows one-command sandbox runner:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_sandboxes.ps1
```

RocketPy physics simulation:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_rocketpy_sim.ps1
```

The first RocketPy run creates a local `.venv` and installs the pinned dependency.

Browser simulation console:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_simulation_ui.ps1
```

The local dashboard provides Run All/Run Suite controls, adjustable RocketPy
trade-study inputs, scenario tables, interactive altitude/speed/acceleration/
deployment graphs, a C++ phase timeline, CSV/JSON export, source-gap tracking,
and the complete raw terminal output. See
[docs/simulation_ui.md](docs/simulation_ui.md).

```powershell
cmake -S . -B build
cmake --build build
.\build\Debug\rocket_airbrake_ekf.exe
.\build\Debug\sim_flight_sandbox.exe
.\build\Debug\sim_electronics_sandbox.exe
.\build\Debug\sim_actuator_sandbox.exe
.\build\Debug\sim_fault_replay_sandbox.exe
.\build\Debug\sim_monte_carlo_sandbox.exe
.\build\Debug\ambar_core_tests.exe
.\build\Debug\ambar_controller_bridge.exe
```

For single-config generators, the executable may be under `build/rocket_airbrake_ekf`.
The sandbox executables may likewise be directly under `build/`.

Direct compiler check, useful on Windows when CMake is not installed:

```powershell
g++ -std=c++17 -Wall -Wextra -Wpedantic -I include src/ambar_airbrake.cpp src/main.cpp -o build\rocket_airbrake_ekf.exe
.\build\rocket_airbrake_ekf.exe
```

Direct sandbox checks:

```powershell
g++ -std=c++17 -Wall -Wextra -Wpedantic -I include src/ambar_airbrake.cpp sim/flight_sandbox.cpp -o build\sim_flight_sandbox.exe
g++ -std=c++17 -Wall -Wextra -Wpedantic -I include sim/electronics_sandbox.cpp -o build\sim_electronics_sandbox.exe
g++ -std=c++17 -Wall -Wextra -Wpedantic -I include sim/actuator_sandbox.cpp -o build\sim_actuator_sandbox.exe
.\build\sim_flight_sandbox.exe
.\build\sim_electronics_sandbox.exe
.\build\sim_actuator_sandbox.exe
```

## Sensor Interface

The EKF expects vertical acceleration in the launch frame, positive upward, after
IMU axis alignment and gravity compensation:

```cpp
flightComputer.updateImu(timestamp_s, verticalAcceleration_mps2);
```

The barometer update expects altitude above the launch pad:

```cpp
flightComputer.updateBarometer(barometerAltitudeAgl_m, barometerStdDev_m);
```

The actuator should consume only `AirbrakeCommand`. If `inhibit` is true, command
the safe/retracted state.

## Software Architecture

The repository has one shared C++ flight-logic core and several adapters around
it. Desktop simulations, RocketPy, and future STM32 firmware are intended to use
the same `AmbarFlightComputer` API rather than duplicate estimator or controller
logic. The browser UI launches those programs and displays their reports; it
does not simulate the rocket itself.

See [docs/software_architecture.md](docs/software_architecture.md) for the data
flow, file dependency map, and the intended use case for each executable.

## Current Project Files

- [docs/project_requirements.md](docs/project_requirements.md): AMBAR M5
  requirements mapped to the estimator and controller.
- [docs/project_status.md](docs/project_status.md): implemented, simulated,
  provisional, and future capabilities with the current verification snapshot.
- [docs/hardware_map.md](docs/hardware_map.md): provisional June 2 V3 KiCad
  component and STM32 pin map, including routing maturity.
- [docs/datasheet_integration_notes.md](docs/datasheet_integration_notes.md):
  datasheet facts reflected in firmware constants.
- [docs/simulation_sandboxes.md](docs/simulation_sandboxes.md): project review
  summary and virtual sandbox instructions.
- [docs/simulation_ui.md](docs/simulation_ui.md): browser dashboard startup and
  controls.
- [docs/beginner_quick_start.md](docs/beginner_quick_start.md): plain-English
  instructions for downloading the repo and running the sandboxes on Windows.
- [docs/source_backed_simulation_inputs.md](docs/source_backed_simulation_inputs.md):
  values pulled from the M5 report/KiCad files versus values still treated as
  placeholders.
- [docs/m5_report_data_extract.md](docs/m5_report_data_extract.md): engineering
  values extracted from the June 14 M5 report, their confidence, and conflicts
  with older report material.
- [docs/sensor_architecture.md](docs/sensor_architecture.md): verified separation
  between the airbrake magnetometer and independent recovery GPS.
- [docs/software_architecture.md](docs/software_architecture.md): how the shared
  flight core, simulations, scripts, and browser UI connect.
- [docs/simulation_audit.md](docs/simulation_audit.md): engineering review of
  what each simulation proves, current gaps, and recommended next simulations.
- [docs/m5_report_change_guide.md](docs/m5_report_change_guide.md):
  section-by-section advice for manually correcting the Word report.

Older V2 screenshot notes and outdated BOM mismatch notes were removed from the
live repo so GitHub only presents the current working baseline.

## Safety Note

Rocket airbrake software can affect flight stability and recovery outcomes. Treat this
code as experimental until it has been validated with simulation, bench testing,
hardware-in-the-loop testing, and controlled flight data review.
