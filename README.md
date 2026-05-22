# Rocket Airbrake EKF

Learning-oriented error-state EKF prototype for a rocket airbrake avionics project.

This repository currently contains a standalone C++ demo that estimates NED position,
velocity, attitude, IMU biases, and barometer bias from simulated IMU, barometer, and
GPS updates.

## Current Status

This is a starting point, not flight-ready software.

Before use in flight hardware, the estimator should be refactored into a bounded
embedded module with:

- Real timestamped IMU, barometer, and GPS inputs
- Explicit body-frame axis conventions
- Sensor validation and innovation gating
- Joseph-form covariance updates
- Flight-phase-specific behavior
- A separate apogee predictor and airbrake controller
- Hardware-in-the-loop and flight-log replay tests

## Build

```powershell
cmake -S . -B build
cmake --build build
.\build\Debug\rocket_airbrake_ekf.exe
```

For single-config generators, the executable may be under `build/rocket_airbrake_ekf`.

## Safety Note

Rocket airbrake software can affect flight stability and recovery outcomes. Treat this
code as experimental until it has been validated with simulation, bench testing,
hardware-in-the-loop testing, and controlled flight data review.
