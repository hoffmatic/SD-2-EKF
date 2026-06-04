# SD-2-EKF

Embedded-oriented estimator and airbrake-control scaffold for Project AMBAR.

The code now follows the AMBAR M3 concept report: a 3-inch test vehicle targeting
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
- June 1 KiCad pin-map constants in `include/ambar_board_pins.hpp`
- Datasheet-derived device IDs, bus modes, addresses, and register constants in
  `include/ambar_device_constants.hpp`
- Virtual sandboxes for flight behavior, electronics bring-up checks, and
  actuator motion/fault behavior

## Current Status

This is an integration scaffold, not flight-ready software.

Before use in flight hardware, add:

- Board support for the actual IMU, barometer, radio, logger, and actuator
- IMU body-axis alignment and gravity compensation
- A drag-aware apogee predictor calibrated from OpenRocket and flight logs
- TMC5240 motor driver limits, current monitoring, and fail-safe behavior
- Unit tests, simulation replay tests, and hardware-in-the-loop tests
- Telemetry packet definitions for estimate, phase, command, and health

## Build

```powershell
cmake -S . -B build
cmake --build build
.\build\Debug\rocket_airbrake_ekf.exe
.\build\Debug\sim_flight_sandbox.exe
.\build\Debug\sim_electronics_sandbox.exe
.\build\Debug\sim_actuator_sandbox.exe
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

## Current Project Files

- [docs/project_requirements.md](docs/project_requirements.md): AMBAR M3
  requirements mapped to the estimator and controller.
- [docs/hardware_map.md](docs/hardware_map.md): current June 1 V3 KiCad
  component and STM32 pin map.
- [docs/datasheet_integration_notes.md](docs/datasheet_integration_notes.md):
  datasheet facts reflected in firmware constants.
- [docs/simulation_sandboxes.md](docs/simulation_sandboxes.md): project review
  summary and virtual sandbox instructions.

Older V2 screenshot notes and outdated BOM mismatch notes were removed from the
live repo so GitHub only presents the current working baseline.

## Safety Note

Rocket airbrake software can affect flight stability and recovery outcomes. Treat this
code as experimental until it has been validated with simulation, bench testing,
hardware-in-the-loop testing, and controlled flight data review.
