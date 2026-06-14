# Simulation Engineering Audit

Audit date: June 14, 2026

## Overall Assessment

The repository has useful software-in-the-loop tests, but it does not yet have
a flight-validated digital twin. The current suites are strongest at checking
deterministic software reactions and weakest at representing sensor, electrical,
mechanical, aerodynamic, and manufacturing uncertainty.

Two audit defects were corrected:

1. The native flight, electronics, and actuator sandboxes now return a nonzero
   process exit code when any printed scenario fails. Terminal scripts and
   future CI can therefore enforce the results.
2. RocketPy previously commanded airbrakes at 1.53 s even though the J420R
   thrust curve ends at 1.64 s. The bridge now uses the motor burn time plus a
   0.10 s margin, and the pass rule checks command time and flight phase.

## What Each Suite Is For

| Suite | Why It Exists | What It Currently Proves | What It Does Not Prove |
| --- | --- | --- | --- |
| Flight sandbox | Fast repeatable estimator/controller regression | Noise, fixed bias, one barometer spike, actuator lag, weak-motor inhibition | Real aerodynamics, body-axis conversion, sensor timing, hardware drivers, target accuracy |
| Electronics sandbox | Review intended boot and arming policy before drivers exist | Static IDs, addresses, bus modes, pin uniqueness, and selected fault decisions | Actual bus transactions, power transients, RF link budget, flash throughput, thermal behavior |
| Actuator sandbox | Exercise the `AirbrakeCommand` consumer contract | Homing gate, rate limit, retract behavior, simple jam/current fault | Motor torque, lead-screw load, backlash, friction, thermal limits, real stall-detection delay |
| RocketPy sandbox | Couple six-degree-of-freedom trajectory physics to the real C++ controller | Persistent bridge, post-burn command gating, bounded deployment, trajectory response, reference envelope | Independent apogee accuracy, measured sensor behavior, uncertainty distribution, descent/recovery behavior |
| Browser console | Make suite output reviewable by the team | Launches the same scripts and exposes conditions, rules, and results | Independent verification; it parses text emitted by the simulations |

## Open Findings

### High Priority

- RocketPy feeds truth-derived vertical acceleration and altitude. The recorded
  maximum estimator errors are about 0.0023 m/s and 0.0012 m, which is far more
  accurate than real sensors. No noise, bias, quantization, latency, dropout,
  saturation, axis rotation, or gravity-compensation error is applied.
- The passive RocketPy model is calibrated to 4005 ft using placeholder mass,
  inertia, center of mass, and drag. Agreement with the M5 OpenRocket number is
  therefore a consistency check, not independent validation.
- No Monte Carlo or parameter-dispersion run exists. One nominal result cannot
  estimate the probability of meeting the 3000 +/-100 ft requirement.
- The embedded apogee predictor is ballistic. In the current RocketPy run its
  prediction is more than 600 m above eventual apogee near the end of boost,
  then becomes accurate later in coast. Controller success currently depends on
  placeholder airbrake drag and should not be treated as tuned flight behavior.

### Medium Priority

- RocketPy terminates at apogee, so it does not verify transition to Recovery,
  post-apogee retraction, descent, parachute interaction, or impact conditions.
- The flight phase tracker can declare liftoff from a single acceleration sample.
  Pad vibration and isolated sensor spikes are not tested for false launch.
- IMU timestamp gaps, frozen sensors, NaN/Inf samples, barometer dropout, stale
  data, and sensor-rate jitter are not covered as complete scenarios.
- Electronics checks operate on direct scenario fields rather than fake I2C/SPI
  transactions. They cannot test future driver state machines or timeouts.
- Flash capacity assumes a 64-byte record and only 10 Hz. The actual log packet
  size and rate are not defined, so the two-hour PASS is conditional.
- Radio compatibility and independent recovery GPS are architecture assertions,
  not simulated links or verified attached hardware.
- The actuator model faults immediately at an injected position. It does not
  model torque margin, load versus deployment, current filtering, missed steps,
  temperature, or limit-switch failures.
- The UI parser depends on human-readable text formatting. Machine-readable
  JSON per suite would make result ingestion less fragile.

## Additional Simulations That Can Be Added Now

These primarily test software policy and do not require final hardware numbers:

1. Controller safety-interlock matrix: phase, altitude, time, descent, health,
   and on-target inhibit flags checked independently.
2. False-liftoff tests: pad vibration, one-sample acceleration spike, and
   pressure disturbance must not permanently enter Boost.
3. Timing-fault tests: duplicate timestamps, backward time, long IMU gaps,
   irregular sample cadence, and scheduler overrun.
4. Sensor-fault tests: frozen IMU, frozen barometer, dropout windows, NaN/Inf,
   saturation, and recovery after transient faults.
5. Post-apogee test: Recovery transition and commanded/physical retraction.
6. Bridge protocol stress: malformed lines, process termination, and thousands
   of persistent updates without state reset.
7. Output-contract tests: parse each suite's JSON/text and verify UI status is
   identical to executable exit status.

## Additional Simulations That Need Project Data

1. Monte Carlo trajectory and control dispersion across mass, CG, inertia,
   thrust, drag, wind, launch angle, sensor errors, and actuator rate.
2. Sensor-in-the-loop model using actual LSM6DSV32X and BMP388 output data rate,
   filters, noise density, quantization, range, latency, and PCB orientation.
3. Airbrake aerodynamic sweep using drag coefficient versus deployment and Mach.
4. Electromechanical actuator model using motor torque curve, lead screw pitch,
   efficiency, travel, load, friction, current limits, and temperature.
5. Power/brownout model using battery impedance, rail capacitance, regulator
   efficiency, device current profiles, and motor/radio transients.
6. Flash endurance and throughput model using actual record format, write rate,
   page buffering, erase strategy, and power-loss behavior.
7. RF and telemetry model using antenna gain, orientation, packet rate, link
   budget, interference, and ground-station receiver details.
8. Flight-log replay and hardware-in-the-loop testing once real sensor captures
   and the first PCB are available.

## Current Interpretation

The simulations are credible for detecting certain software regressions and
unsafe command ordering. They are not yet credible for claiming a probability
of hitting target apogee or predicting complete PCB/mechanism behavior. The
next large increase in confidence will come from measured configuration data,
Monte Carlo dispersion, and sensor/actuator hardware-in-the-loop tests.

## Project Data Request

Provide whatever is available; unknown values should remain explicitly marked
unknown rather than estimated silently.

1. Is the AeroTech J420R the final motor? If not, provide the exact motor file
   or designation and the minimum post-burn delay the team wants.
2. What are the current flight-ready dry mass, loaded mass, center of gravity,
   and inertia estimates? Include expected measurement uncertainty if known.
3. Is there a current OpenRocket `.ork` file or exported power-on/power-off drag
   curve? Provide the final body, nose, fin, and rail-button dimensions if not.
4. What is the airbrake geometry at each deployment level, and is any CFD,
   wind-tunnel, OpenRocket, or hand-calculated drag data available?
5. What motor, lead screw pitch, gearing, microstep setting, travel, homing
   switch, current limit, stall threshold, and expected deployment load are used?
6. What LSM6DSV32X accelerometer/gyro range, output data rate, digital filter,
   and physical board-axis orientation will firmware use?
7. What BMP388 output data rate, oversampling, IIR filter, pressure vent size,
   and expected sensor delay will firmware use?
8. What battery voltage/capacity, estimated 3.3 V current budget, motor current,
   and available bulk/decoupling capacitance should the power model use?
9. What telemetry and flash log fields, bytes per record, and sample rates are
   required during pad, boost, coast, and recovery?
10. Should this repository simulate recovery GPS, parachutes, descent, landing,
    and RF range, or are those owned by separate team subsystems?
11. For each failure, what should block arming versus continue with a warning:
    magnetometer loss, radio loss, flash loss, barometer loss, actuator not
    homed, and recovery GPS unavailable?
12. What statistical success criterion should Monte Carlo enforce, such as 95%
    or 99% of flights inside 3000 +/-100 ft and all flights below 12500 ft?

## RocketPy Capabilities Available For The Next Stage

RocketPy 1.12.1 already provides supported paths for airbrake drag curves,
sensor objects, Monte Carlo dispersion, and exported flight results. The next
stage should use those library features rather than creating parallel physics
or randomization frameworks in this repository.

- Airbrake controller and drag-curve interface:
  <https://docs.rocketpy.org/en/latest/user/airbrakes.html>
- Sensor class examples:
  <https://docs.rocketpy.org/en/latest/notebooks/sensors.html>
- Monte Carlo class example:
  <https://docs.rocketpy.org/en/latest/notebooks/monte_carlo_analysis/monte_carlo_class_usage.html>
