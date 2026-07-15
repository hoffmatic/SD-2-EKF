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
  and a persistent bridge compiled from the production STM32 C estimator and
  flight-controller sources
- Deterministic timestamp/sensor fault injection, synthetic log replay, an
  older 200-trial 1-D safety sandbox, and a production-controller RocketPy
  campaign with per-run CSV output and seeded Latin-hypercube variation
- CTest regression tests and GitHub Actions verification

## Current Status

This is an integration scaffold, not flight-ready software.

The current production-STM32-C RocketPy cross-check fails: the nominal model
predicts 3829 ft AGL passive apogee versus 3379 ft in the June 14 M5 report and
only 42.7 ft/s off the reported 72-inch rail. Closed loop reaches 3327 ft AGL,
about 327 ft above target. These mismatches remain visible until the OpenRocket
configuration, mass properties, and drag data are reconciled.

Before use in flight hardware:

- Calibrate and hardware-qualify the existing sensor, radio, flash, USB, and
  TMC5240 driver paths
- Validate IMU mounting-axis selection, pad reference, and attitude limitations
- Final measured mass properties and drag curves for the RocketPy model
- Calibrate the existing drag-aware embedded predictor against final
  RocketPy/OpenRocket inputs and recorded tests
- Confirm TMC5240 current, speed, travel, homing, load, and fail-safe limits on
  the final mechanism
- Hardware-in-the-loop tests and recorded hardware-log replay
- Verify the existing telemetry/config/log packet paths against the GUI and
  ground station

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

Two baseline repeats plus 50 randomized closed-loop runs:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_monte_carlo.ps1
```

You can also double-click `Run Monte Carlo Simulation.cmd`. Results are written
under `build\monte-carlo\<campaign-id>\`, including `runs.csv`, aggregate
metrics, sensitivity results, and representative profiles. See
[docs/monte_carlo_campaign.md](docs/monte_carlo_campaign.md).
`scripts/validate_monte_carlo_output.py` independently recomputes saved counts,
rates, distributions, joins, worst cases, and snapshot hashes.

Browser simulation console:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_simulation_ui.ps1
```

The local dashboard provides Run All/Run Suite controls, adjustable RocketPy
trade-study inputs, scenario tables, interactive altitude/speed/acceleration/
deployment graphs, a production STM32-C phase timeline, CSV/JSON export, source-gap tracking,
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
.\build\Debug\ambar_stm32_controller_bridge.exe
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

## Legacy C++ Sensor Interface

The older native C++ scaffold expects vertical acceleration in the launch frame:

```cpp
flightComputer.updateImu(timestamp_s, verticalAcceleration_mps2);
```

The barometer update expects altitude above the launch pad:

```cpp
flightComputer.updateBarometer(barometerAltitudeAgl_m, barometerStdDev_m);
```

The production STM32 firmware instead consumes its configured, pad-referenced
IMU body-axis channel. The main RocketPy campaign now models that current
firmware contract, including attitude projection, while retaining world-vertical
truth as a separate diagnostic.

## Software Architecture

The repository retains an older shared C++ flight-logic scaffold for native
sandboxes. RocketPy and the new Monte Carlo campaign now use a separate bridge
compiled directly from the production STM32 `ambar_ekf.c` and `ambar_flight.c`
sources, preventing controller drift between the board and the main closed-loop
physics study. The browser UI launches programs and displays their reports; it
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
- [firmware/stm32_airbrake_pcb](firmware/stm32_airbrake_pcb): STM32CubeIDE
  project copy for the Airbrake PCB, including the C EKF port, flash logging,
  radio command parsing, telemetry additions, and bench-gated actuator layer.
- [hardware/airbrake_pcb](hardware/airbrake_pcb): KiCad PCB project and local
  component libraries copied from the Airbrake PCB design folder.
- [hardware/datasheets](hardware/datasheets): local component datasheets used
  for firmware constants, bus assumptions, and safety notes.
- [firmware/rust_controller_reference](firmware/rust_controller_reference):
  standalone Rust controller reference copied from the local PCB folder.
- [tools/arduino_ide_shortcut](tools/arduino_ide_shortcut): readable extraction
  of the local Arduino IDE shortcut metadata.
- [tools/arduino_ground_receiver](tools/arduino_ground_receiver): Arduino/LILYGO
  SX1280 receiver sketch that decodes the STM32 binary EKF telemetry packets.

Older V2 screenshot notes and outdated BOM mismatch notes were removed from the
live repo so GitHub only presents the current working baseline.

## Safety Note

Rocket airbrake software can affect flight stability and recovery outcomes. Treat this
code as experimental until it has been validated with simulation, bench testing,
hardware-in-the-loop testing, and controlled flight data review.
