# Project Status and Evidence Levels

Status date: June 14, 2026

This document separates working software from simulations, provisional design
inputs, and future work. A statement in the **implemented** column means code is
present and exercised in this repository. It does not mean flight hardware has
been qualified.

## Current Baselines

| Area | Baseline | Maturity |
| --- | --- | --- |
| Flight logic | C++ `AmbarFlightComputer` in this repository | Implemented and desktop-tested |
| Apogee predictor | Ballistic `v^2/(2g)` coast estimate | Implemented; drag-aware replacement is future work |
| PCB | SharePoint `Electrical/pcb/` KiCad project uploaded June 2, 2026 | Provisional; placed but unrouted |
| Vehicle geometry | June 2 OpenRocket file, with 79.248 mm diameter, 285.75 mm nose, and approximately 1.492 m total length | Imported into the RocketPy reference config; mass/drag remain provisional |
| Mission target | 3000 ft with +/-100 ft tolerance | Used by code and tests; team should keep one canonical requirement set |
| Recovery location | Independent GPS recovery hardware | Required outside the airbrake PCB |
| Airbrake radio | SX1280 at 2.4 GHz | PCB component identified; ground station must also be SX1280-compatible |

## Implemented

- Four-state vertical EKF for altitude, velocity, accelerometer bias, and
  barometer bias.
- Timestamp validation, barometer innovation gating, Joseph-form covariance
  update, and health counters.
- Flight phases and deployment interlocks for estimator health, phase,
  altitude, flight time, descent, and predicted apogee.
- Ballistic apogee prediction and bounded deployment command.
- Board pin constants and datasheet-derived component constants.
- Native flight, electronics, actuator, fault/replay, and fixed-seed Monte
  Carlo sandboxes.
- RocketPy-to-C++ closed-loop bridge with report-backed constant wind and a
  deterministic provisional sensor-error model.
- CTest unit/regression targets and GitHub Actions verification.

## Simulated, Not Hardware-Verified

- Airbrake deployment reducing apogee in the provisional RocketPy model.
- Estimator/controller behavior under deterministic noise, bias, barometer
  spikes, timestamp faults, NaN input, dropouts, actuator lag, and jams.
- Virtual boot decisions for expected chip IDs, bus modes, power limits, and
  missing devices.
- Fixed-seed dispersion across provisional thrust, drag, sensor, and actuator
  inputs.
- Replay of the versioned synthetic log in
  `sim/replay/nominal_vertical_log.csv`.

## Current Verification Snapshot

| Check | Result | Interpretation |
| --- | --- | --- |
| Core C++ assertions | PASS, 19 assertions | Public estimator, phase, controller, and fault contracts behave as tested |
| Native flight scenarios | PASS, 5/5 | Safety behavior passes; nominal apogee calibration remains outside tolerance |
| Electronics scenarios | PASS, 6/6 | Virtual boot policy catches the injected faults |
| Actuator scenarios | PASS, 4/4 | Virtual motion policy catches unhomed and jam conditions |
| Fault/replay scenarios | PASS, 4/4 | Timing faults, invalid input, dropout propagation, and deterministic replay are explicit |
| Native Monte Carlo | PASS safety checks, 200/200 | Commands remained healthy and bounded; only 15/200 provisional trials hit 3000 +/-100 ft |
| RocketPy passive reference | FAIL, 3851 ft vs 3379 ft | Vehicle mass/drag reconstruction does not yet match OpenRocket |
| RocketPy closed-loop coupling | PASS, 2979 ft | C++ bridge and safety ordering work in the provisional model |
| RocketPy target band | PASS, -21 ft | Necessary result only; not validated while passive-model checks fail |
| RocketPy rail exit | FAIL, 42.7 ft/s vs 52 ft/s minimum | Model or launch configuration requires reconciliation |

## Provisional or Unresolved

- The June 2 PCB contains 109 footprints but no routed copper segments, vias,
  or zones. It is not a manufacturing release.
- Dry mass, center of gravity, inertia, power-on/off drag, airbrake drag, rail
  button positions, and most sensor errors are placeholders.
- The OpenRocket file contains cached results but marks its motor simulations
  as not simulated after later model changes. Rerun the selected configuration
  before using its output as current evidence.
- The ground-station BOM entry for SX1262 is incompatible with the flight
  computer's SX1280. Select an SX1280-compatible ground station or redesign the
  flight radio; do not claim an end-to-end link until resolved and range-tested.
- Recovery parachute sources conflict between the OpenRocket model/report and
  the BOM. Physically verify the installed hardware before updating descent
  simulations or report claims.
- TMC5240 brake/dump resistor values, motor current, lead-screw conversion,
  homing behavior, and stall threshold require electrical/mechanical data.

## Not Yet Implemented

- STM32 HAL, scheduler, watchdog, brownout recovery, and production firmware.
- BMP388, LSM6DSV32X, LIS2MDL, W25Q64, SX1280, and TMC5240 hardware drivers.
- IMU orientation and gravity compensation from raw six-axis measurements.
- Timestamped barometer freshness detection and frozen-sensor detection.
- Power-loss-safe flash logging and a versioned telemetry packet protocol.
- Drag-aware embedded apogee prediction calibrated from source-backed data.
- Hardware-in-the-loop, environmental, vibration, RF range, power transient,
  thermal, and controlled flight verification.

## Evidence Rule

Use the following language consistently:

- **Implemented:** code exists and compiles.
- **Desktop-tested:** automated tests exercise the behavior.
- **Simulated:** a declared model produced the result.
- **Provisional:** one or more important inputs are placeholders or conflicted.
- **Hardware-verified:** measured on the physical system with recorded evidence.
- **Flight-validated:** demonstrated in controlled flight with traceable logs.

No current result in this repository is hardware-verified or flight-validated.
