# AMBAR Project Mapping

This repository now targets Project AMBAR's M5 design rather than a generic
GPS/NED estimator demo.

## Project Requirements Reflected in Code

- Target apogee: 3000 ft, represented as `kTargetApogeeM`.
- Target tolerance: +/-100 ft, represented as `kTargetToleranceM`.
- Additional report-derived constants now live in
  `include/ambar_project_requirements.hpp`, including 30G acceleration capacity,
  2-hour logging duration, 2.4 GHz radio operation, <=1 s
  airbrake deployment, 500 ms response target, 5 lb airbrake mass limit, and
  5000 ft ground-station range.
- Airbrake sensors: the M5 report and V3 schematic identify the IMU, barometer,
  and LIS2MDL magnetometer. The magnetometer belongs in the later
  attitude/alignment layer and is not a position sensor.
- Recovery tracking: M5 RR 4.12 still requires a GPS device in the independent
  recovery system. GPS is not fitted to the current airbrake PCB.
- EKF state size: the report identifies 4 to 8 states as appropriate for AMBAR.
  This implementation starts with the 4-state vertical estimator:
  altitude, vertical velocity, accelerometer bias, and barometer bias.
- Autonomous deployment: `AirbrakeController` converts predicted apogee error
  into a bounded deployment fraction.
- Flight-phase behavior: deployment is inhibited until the tracker reaches
  coast or active-airbrake flight.
- FMEA risks: timestamp rejection, sensor validation, innovation gating, and
  safe deployment inhibition directly address the report's software, timing,
  IMU, barometer, and airbrake-control failure modes.

## Sensor Convention

The EKF expects `verticalAcceleration_mps2` as translational vertical
acceleration in the launch frame, positive upward, after IMU axis alignment and
gravity compensation. The board support layer should do that conversion from
raw body-frame accelerometer and gyro data before calling
`updateImu(timestamp_s, verticalAcceleration_mps2)`.

Barometer input is altitude above launch pad in meters. The pressure-to-altitude
conversion and launch-pad zeroing should happen in the sensor layer before
calling `updateBarometer(barometerAltitudeAgl_m, barometerStdDev_m)`.

## Integration Boundary

The airbrake actuator code should consume only `AirbrakeCommand`:

- `deployFraction = 0.0` means fully retracted.
- `deployFraction = 1.0` means maximum allowed extension.
- `inhibit = true` means command the safe/retracted state.
- `inhibitFlags` explains why deployment is blocked.

This keeps the estimator, controller, and actuator driver separated enough for
bench testing, simulation replay, and hardware-in-the-loop testing.

## Next Software Milestones

1. Add a board support layer for the actual IMU, barometer, actuator, radio,
   logger, and scheduler.
2. Replace the demo apogee predictor with an embedded drag-aware model calibrated
   from the new RocketPy backend, final OpenRocket data, ground tests, and logs.
3. Add unit tests and replay tests using simulated and recorded flight data.
4. Add telemetry packets for estimate, phase, command, health, and rejection
   counters.
5. Tune noise, gating, and phase thresholds from bench and flight data.

## Source-Backed Simulation Inputs

See [source_backed_simulation_inputs.md](source_backed_simulation_inputs.md) for
the current split between values pulled from the M5 report/KiCad files and
values that are still placeholders.
