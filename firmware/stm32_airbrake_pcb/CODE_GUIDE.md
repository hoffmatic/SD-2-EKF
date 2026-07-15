# AMBAR Firmware Code Guide

This guide is the navigation map for the project-owned firmware, host tools,
and Arduino ground station. The short identifiers such as `[ARCH-4]` are used
in source-file headers so a reader can move from a function-level comment back
to the overall system design.

Generated STM32, CMSIS, HAL, USBX middleware, startup, and linker files are not
part of the readability pass. They should stay close to the vendor generator's
output so a future CubeMX regeneration remains reviewable.

## Reading order

For a first pass through the project, read these files in order:

1. `Core/Inc/ambar_features.h` - compile-time build identity and safety locks.
2. `Core/Src/main.c` - hardware initialization and the cooperative main loop.
3. `Core/Src/ambar_app.c` - scheduler and integration point for every module.
4. `Core/Src/ambar_flight.c` - phase machine, apogee prediction, and deployment request.
5. `Core/Src/ambar_actuator.c` - final motion gates and TMC5240 position commands.
6. `Core/Src/ambar_usb.c` and `Core/Src/rocket_protocol.c` - direct USB transport and framing.
7. `tools/usb_protocol/replay_openrocket.py` - presentation/HIL operator workflow.

## Architecture map

### [ARCH-1] Startup and cooperative scheduling

`main.c` configures clocks, GPIO, I2C, SPI, USB, and the hardware drivers. It
then calls `AmbarApp_Init()` once and `AmbarApp_Task()` continuously. There is
no RTOS: every task must finish quickly and must not use an unbounded delay.

The main application schedules IMU, barometer, telemetry, logging, USB, radio,
and actuator work using `HAL_GetTick()`. A blocking flash erase or serial loop
can therefore starve sensor fusion and USB service.

### [ARCH-2] Real sensor path

`rocket_sensors.c` coordinates the LSM6DSV32X IMU, BMP388 barometer, and
LIS2MDL magnetometer drivers. It captures the pad reference and converts raw
measurements into the vertical channels consumed by the EKF.

The vertical estimator lives in `ambar_ekf.c`. IMU samples propagate altitude,
vertical velocity, accelerometer bias, and barometer bias. Barometer samples
arrive more slowly and correct the propagated state. Magnetometer values are
telemetry/attitude context and are not part of this vertical EKF.

### [ARCH-3] USB simulation path

The host encodes `RocketSimulationSamplePayload` frames with COBS and CRC-16.
`ambar_usb.c` receives the bytes, `rocket_protocol.c` validates the frame, and
`ambar_app.c` substitutes the supplied altitude, acceleration, velocity, and
barometer uncertainty for the real flight inputs.

Simulation is explicit and time-bounded. `SIM_START` begins a session; a fresh
sample must continue arriving inside the firmware timeout. `SIM_STOP`, stale
input, USB disconnect, DISARM, ESTOP, or a fault removes actuator energy.

### [ARCH-4] Estimation, phase, and deployment request

`ambar_flight.c` owns the phase machine:

```text
PAD_IDLE -> BOOST -> COAST -> AIRBRAKE_ACTIVE -> RECOVERY
                                  \-> FAULT when health policy requires it
```

The EKF estimates the current vertical state. The forward apogee predictor uses
that state to estimate where the rocket will coast. The controller converts the
predicted-apogee error into a bounded `deploy_fraction` from 0.0 to 1.0. Drag
prediction belongs here, not inside the estimator.

### [ARCH-5] Actuator safety and motion

The command path is:

```text
AmbarFlight deploy_fraction
    -> AmbarApp_Task
    -> AmbarActuator_Task safety gates and step clamp
    -> TMC5240_SetTargetPosition
    -> TMC5240 XTARGET register
```

`ambar_actuator.c` is the final authority for movement. Compile-time feature
locks, driver health, valid configuration, HOME, ESTOP, DIAG/fault state, and
flight/bench context must all agree before active-low `DRV_ENN` is released.
Only a nonzero automatic deployment that passed those gates creates a one-shot
permission to return to HOME. That powered return additionally requires a
healthy estimator and armed on-target, descent, or recovery context, and it is
bounded by progress and absolute timeouts. Explicit stop/disconnect/fault paths
remain energy-off and consume the permission.

The configured presentation geometry is three motor rotations:

```text
3 rotations * 200 full steps/revolution * 256 microsteps = 153600 counts
```

`XACTUAL` is the TMC5240 ramp generator's internal position. It is not encoder
feedback and cannot prove that the shaft moved or that the mechanism did not
skip steps. The operator-declared HOME and current settings are presentation
features until real switches, current calibration, and loaded tests exist.

### [ARCH-6] USB, radio, Arduino, and GUI outputs

`rocket_protocol.h/.c` defines the versioned binary packet format shared by the
STM32 and Python tools. `ambar_usb.c` provides direct CDC transport. The SX1280
radio path is handled by `radio_bridge.c`, and the LILYGO Arduino sketch decodes
the radio packet into detailed and presentation summaries.

Only one process should own the STM32 COM port. During a presentation, the
Python replay process owns COM and can forward decoded newline JSON to the GUI;
the GUI must not open COM independently.

### [ARCH-7] Configuration and flash logging

`ambar_config.c` validates defaults and redundant W25Q64 configuration records.
`ambar_log.c` stores fixed-size CRC-protected snapshots in a ring. Flash erases
are blocking operations, so maintenance commands remain disabled on the USB
presentation path unless explicitly enabled and rehearsed.

### [ARCH-8] Host replay, evidence, and acceptance

`replay_openrocket.py` parses and validates OpenRocket CSV data, normalizes it
to a bounded sample rate, performs board preflight, streams samples on an
absolute schedule, monitors telemetry and actuator health, and guarantees safe
cleanup. Physical motion requires two explicit CLI acknowledgements.

A presentation run should leave a machine-readable evidence bundle containing
the input identity, CLI arguments, feature bits, packets, timing statistics,
phase order, actuator tracking, final HOME/driver state, and a PASS/FAIL
verdict. Host scheduling uses a high-resolution clock; device time and host
time must not be conflated. More than 100 ms peak lag or more than 2% skipped
samples fails the current presentation acceptance policy.

## Build profiles

### Motion presentation build

`AMBAR_FEATURE_PRESENTATION_MOTION=1` enables the reviewed USB-to-motor demo
profile. Startup is still unhomed and energy-off. The host must establish HOME,
ARM the simulated flight, maintain fresh data, and pass every runtime check.

### GUI-only presentation build

Set `AMBAR_FEATURE_PRESENTATION_MOTION=0` and rebuild. USB telemetry and
simulation remain available, but actuator and direct bench movement are
compile-time inhibited.

### Future flight build

A flight profile should explicitly disable presentation/bench/simulation paths,
enable and prove the hardware watchdog, use calibrated actuator current, and
require real home/end-stop feedback. It must not be created merely by renaming
the presentation binary.

## Commenting convention

Project-owned source files use this structure:

- A file overview explaining purpose, data/control flow, safety assumptions,
  section order, and relevant `[ARCH-n]` references.
- Section banners for configuration, module state, private helpers, public API,
  and interrupt/callback entry points.
- Function comments describing responsibility, important inputs/outputs,
  side effects, failure behavior, and safety implications.
- Define and state comments that explain units, timing, ownership, and why a
  value exists. Comments should not merely translate C syntax into English.

## Presentation acceptance checklist

1. Flash the intended labeled build and probe VID:PID `0483:5740`.
2. Confirm feature bits, driver health, configuration validity, HOME state, and
   driver-off state before ARM.
3. Perform a low-travel direction check after any mechanical or firmware change.
4. Run the selected OpenRocket scenario while recording the evidence bundle.
5. Require the expected phase order, nonzero deployment when intended, bounded
   target/actual tracking, zero protocol errors, and final HOME with driver off.
6. Repeat the complete run enough times to expose heat, drift, USB loss, or a
   one-run-only success before the public demonstration.
