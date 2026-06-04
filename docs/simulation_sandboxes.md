# Simulation Sandboxes

This repo now includes three desktop sandboxes that exercise the current AMBAR
software and hardware assumptions without needing the real PCB.

These are not final flight predictions. They are virtual workbenches for asking:

- Does the estimator/controller respond sanely to noisy sensors?
- What happens if the airbrake actuator is slow, unhomed, or jammed?
- What should the electronics boot checklist catch before flight logic is armed?

## Project Review Summary

Current strengths:

- The estimator/controller code is compact and portable.
- Hardware pin constants and datasheet constants are centralized.
- Deployment is inhibited unless estimator health, phase, altitude, time, and
  apogee checks all pass.
- Barometer updates use innovation gating and Joseph-form covariance updates.

Current gaps:

- No real sensor, flash, radio, or TMC5240 drivers exist yet.
- The IMU body-axis orientation is still unknown.
- The airbrake mechanism travel, step/mm, homing method, and current limit are
  still placeholders.
- The apogee predictor is still ballistic and does not yet use calibrated drag.
- The SX1280 ground-station compatibility question is still open.

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
- Checks W25Q64 JEDEC ID and log capacity.
- Checks SX1280 SPI mode and BUSY behavior.
- Checks TMC5240 SPI mode, IOIN version, and motor supply presence.
- Checks for duplicate GPIO assignments in the board pin constants.

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
- Uses placeholder travel of 20 mm and 400 steps/mm.
- Runs nominal, slow-motor, not-homed, and jammed scenarios.
- Reports peak deployment, final deployment, peak current, homing state, and
  fault reason.

Useful questions:

- Does the mechanism retract when commands are inhibited?
- Does an unhomed actuator refuse deployment?
- Does a jam create a high-current/fault condition?
- How much deployment is lost when motor speed is too low?

## Next Simulation Work

1. Replace placeholder motor and actuator values with final mechanical data.
2. Import OpenRocket or measured flight profiles for replay testing.
3. Add CSV output for plots of altitude, velocity, predicted apogee, and command.
4. Add real sensor-driver unit tests once BMP388 and LSM6DSV32X drivers exist.
5. Add hardware-in-the-loop tests that compare these virtual checks with actual
   boot telemetry from the PCB.
