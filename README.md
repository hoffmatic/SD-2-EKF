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

## Current Status

This is an integration scaffold, not flight-ready software.

Before use in flight hardware, add:

- Board support for the actual IMU, barometer, radio, logger, and actuator
- IMU body-axis alignment and gravity compensation
- A drag-aware apogee predictor calibrated from OpenRocket and flight logs
- Servo or motor driver limits, current monitoring, and fail-safe behavior
- Unit tests, simulation replay tests, and hardware-in-the-loop tests
- Telemetry packet definitions for estimate, phase, command, and health

## Build

```powershell
cmake -S . -B build
cmake --build build
.\build\Debug\rocket_airbrake_ekf.exe
```

For single-config generators, the executable may be under `build/rocket_airbrake_ekf`.

Direct compiler check, useful on Windows when CMake is not installed:

```powershell
g++ -std=c++17 -Wall -Wextra -Wpedantic -I include src/ambar_airbrake.cpp src/main.cpp -o build\rocket_airbrake_ekf.exe
.\build\rocket_airbrake_ekf.exe
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

See [docs/ambar_project_mapping.md](docs/ambar_project_mapping.md) for the
mapping from the AMBAR M3 report to this code.

See [docs/ambar_bom_hardware_notes.md](docs/ambar_bom_hardware_notes.md) for the
BOM-derived hardware interfaces and open integration questions.

See [docs/v2_schematic_reference.md](docs/v2_schematic_reference.md) for a
provisional V2 schematic signal map to compare against the final V3 design.

## Safety Note

Rocket airbrake software can affect flight stability and recovery outcomes. Treat this
code as experimental until it has been validated with simulation, bench testing,
hardware-in-the-loop testing, and controlled flight data review.
