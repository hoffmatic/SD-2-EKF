# STM32 Airbrake PCB Firmware

This folder contains the STM32CubeIDE firmware copy for the AMBAR airbrake PCB.
It was generated from the local Desktop project named `MCU Project EKF PCB Copy`.

## What This Firmware Does

- Reads the PCB sensors over the board buses:
  - LSM6DSV32X IMU on I2C1
  - BMP388 barometer on I2C2
  - LIS2MDL magnetometer on I2C3
  - SX1280 radio on SPI1
  - TMC5240 motor driver on SPI2
  - W25Q64 flash on SPI3
- Runs a vertical EKF for altitude, velocity, accelerometer bias, and barometer bias.
- Predicts apogee with both ballistic and drag-aware estimators.
- Sends the older telemetry layout first, then appends versioned extra sections.
- Accepts radio commands for pad reset, arming, logging, config, emergency stop, and bench movement requests.
- Logs flight/bench snapshots to external flash.
- Keeps physical actuator motion disabled unless all bench gates and build flags are deliberately enabled.

## Safety Defaults

The actuator is intentionally blocked in the normal build:

```c
#define AMBAR_ENABLE_ACTUATOR 0
#define AMBAR_ENABLE_ACTUATOR_BENCH 0
```

Even if those are changed for bench work, movement still requires valid stored
configuration, healthy sensors, a homed actuator state, no emergency stop, and
an explicit command. Flight hardware should not be trusted until travel,
direction, current limits, pressure venting, mechanism stops, and hardware-in-
the-loop replay have been verified.

## How To Open It

1. Open STM32CubeIDE.
2. Import this folder as an existing STM32CubeIDE project.
3. Build the Debug configuration.
4. Confirm the size remains near the last checked build:

```text
text: 84576
data:   652
bss:   5508
total: 90736
```

## Host Replay Check

The lightweight replay script is here:

```text
HostTests/bench_replay_tests.py
```

The latest local check produced:

```text
ballistic_apogee_m=1034.20
drag_apogee_m=690.38
PASS
```

## What Is Still Missing Before Flight

- Physical actuator travel and direction measurements.
- Verified current limits and motor thermal behavior.
- Homing/limit behavior on the real mechanism.
- Pressure venting and airbrake force characterization.
- Hardware-in-the-loop replay using real sensor logs.
- Tuned EKF, drag, and deployment constants from measured data.
- Ground receiver updates to decode every appended telemetry section.
