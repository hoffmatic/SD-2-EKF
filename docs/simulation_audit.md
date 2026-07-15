# Simulation Engineering Audit

Audit updated: July 15, 2026

## Overall Assessment

The repository has useful software-in-the-loop tests, but it does not yet have
a flight-validated digital twin. The current suites are strongest at checking
deterministic software reactions and weakest at representing sensor, electrical,
mechanical, aerodynamic, and manufacturing uncertainty.

The initial audit defects were corrected, and this update adds:

1. The native flight, electronics, and actuator sandboxes now return a nonzero
   process exit code when any printed scenario fails. Terminal scripts and
   future CI can therefore enforce the results.
2. RocketPy now uses fixed production controller configuration while an
   independent gate checks every command against randomized true motor burnout
   plus margin. The nominal first command is about 4.42 s.
3. Formal CTest assertions for estimator timing, invalid values, innovation
   gating, controller interlocks, fault latching, and the ballistic predictor.
4. Dedicated fault/replay and fixed-seed Monte Carlo sandboxes.
5. Report-backed wind plus a pad-referenced body-axis IMU model, deterministic
   sensor errors, actuator delay/rate, and closed trajectory feedback.
6. A separate two-baseline plus 50-case seeded Latin-hypercube campaign writes
   checkpointed CSV evidence and exact input/binary snapshots.

The June 14 M5 report was subsequently extracted. It reports 3379 ft passive
apogee for the current OpenRocket design, while the older full-report copy used
by the original reference model reports 4005 ft. The simulation now uses 3379
ft as the comparison target and must expose any mismatch rather than passing by
remaining calibrated to the superseded design.

## What Each Suite Is For

| Suite | Why It Exists | What It Currently Proves | What It Does Not Prove |
| --- | --- | --- | --- |
| Flight sandbox | Fast repeatable estimator/controller regression | Noise, fixed bias, one barometer spike, actuator lag, weak-motor inhibition | Real aerodynamics, body-axis conversion, hardware drivers, target accuracy |
| Electronics sandbox | Review boot and arming policy apart from hardware | Static IDs, addresses, bus modes, pin uniqueness, and selected fault decisions | Actual bus transactions, power transients, RF link budget, flash throughput, thermal behavior |
| Actuator sandbox | Exercise the `AirbrakeCommand` consumer contract | Homing gate, rate limit, retract behavior, simple jam/current fault | Motor torque, lead-screw load, backlash, friction, thermal limits, real stall-detection delay |
| Fault/replay sandbox | Make failure policy and replay reproducibility explicit | Duplicate timestamps, NaN containment, barometer dropout propagation, deterministic synthetic-log replay | Driver freshness checks, raw hardware logs, scheduler faults |
| Monte Carlo sandbox | Regress safety behavior across deterministic dispersion | 200 fixed-seed trials with varied provisional thrust, drag, sensors, and actuator rate | Source-backed probability of mission success or 6-DOF uncertainty |
| RocketPy sandbox | Couple six-degree-of-freedom trajectory physics to production STM32-C flight modules | Wind, body-axis provisional sensors, persistent bridge, independent post-burn gating, bounded deployment, trajectory response, Recovery, retraction | Independent apogee accuracy, measured sensor behavior, source-backed distributions, parachute/recovery dynamics |
| Production Monte Carlo | Screen repeated closed-loop behavior with reproducible randomized truth | Seeded Latin-hypercube coverage, paired passive/control results, safety/performance gates, checkpoints, CSV/provenance | Flight qualification, board equivalence, or rare-event reliability proof |
| Browser console | Make suite output reviewable by the team | Launches the same scripts and exposes conditions, rules, and results | Independent verification; it parses text emitted by the simulations |

## Open Findings

### High Priority

- RocketPy now projects acceleration into body Z, subtracts the pad reference,
  and applies provisional bias, noise, quantization, saturation, and latency.
  These remain assumptions; vibration, mounting error, and raw device behavior
  are outside the model.
- The June 2 OpenRocket body/nose/fin geometry is now reflected in RocketPy.
  Placeholder mass, inertia, center of mass, and drag still prevent agreement
  with the 3379 ft reported reference, so the mismatch remains a failed check.
- The current report supplies updated launch conditions and stabilizing-fin
  geometry, but it does not supply mass, CG, inertia, axial component positions,
  or a reusable drag curve. Those missing values prevent independent
  reconstruction of the OpenRocket result.
- The native fixed-seed Monte Carlo now checks deterministic safety invariants.
  Its 15/200 target-hit count is informational because the 1-D model and input
  distributions are provisional; it is not a mission-success probability.
- Production firmware exposes both ballistic comparison output and a
  provisional drag-aware vertical predictor. Controller behavior still depends
  on placeholder mass, CdA, density, effectiveness, and airbrake drag, so it is
  not tuned flight evidence.

### Medium Priority

- RocketPy now continues through a bounded post-apogee controller observation
  window and verifies Recovery entry plus airbrake retraction. It still does not
  model parachutes, recovery electronics, landing, or deployment loads.
- The flight phase tracker can declare liftoff from a single acceleration sample.
  Pad vibration and isolated sensor spikes are not tested for false launch.
- Duplicate timestamps, NaN IMU input, and a barometer dropout window are now
  covered. Frozen/stale detection, long gaps, backward time, saturation policy,
  and sensor-rate jitter still require complete scenarios and scheduler APIs.
- Electronics checks operate on direct scenario fields rather than fake I2C/SPI
  transactions. They cannot test future driver state machines or timeouts.
- Flash capacity assumes a 64-byte record and only 10 Hz. The actual log packet
  size and rate are not defined, so the two-hour PASS is conditional.
- Radio compatibility and independent recovery GPS are architecture assertions,
  not simulated links or verified attached hardware.
- The actuator model faults immediately at an injected position. It does not
  model torque margin, load versus deployment, current filtering, missed steps,
  temperature, or limit-switch failures.
- The report calculates 54.8215 N per airbrake fin at 197.206 m/s, but separately
  reports a 579 ft/s maximum vehicle speed. Those cases conflict and require
  confirmation before either becomes an actuator qualification load.
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
5. Post-apogee test: commanded/physical retraction through Recovery.
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
8. Replace the synthetic replay with captured hardware logs and add
   hardware-in-the-loop testing once the first assembled PCB is available.

## Current Interpretation

The simulations are credible for detecting certain software regressions and
unsafe command ordering. They are not yet credible for claiming a probability
of hitting target apogee or predicting complete PCB/mechanism behavior. The
next large increase in confidence will come from measured configuration data,
Monte Carlo dispersion, and sensor/actuator hardware-in-the-loop tests.

With the June 14 report inputs applied, the current RocketPy cross-check is
openly failing: 3829 ft AGL simulated passive apogee versus 3379 ft AGL reported, and
42.7 ft/s simulated rail exit versus the 52 ft/s requirement and 75.5 ft/s
reported result. The production controller produces 3327 ft AGL (+327 ft target
error) inside that provisional model. This cannot be used as target-accuracy
evidence until the passive vehicle model is reconciled.

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

RocketPy 1.12.1 provides supported paths for airbrake drag curves, sensor
objects, and exported flight results. The implemented campaign keeps RocketPy
as the single physics engine and adds a small seeded Latin-hypercube
orchestration layer for fixed-controller pairing, checkpointing, and the exact
CSV/provenance contract required by this project.

- Airbrake controller and drag-curve interface:
  <https://docs.rocketpy.org/en/latest/user/airbrakes.html>
- Sensor class examples:
  <https://docs.rocketpy.org/en/latest/notebooks/sensors.html>
- Monte Carlo class example:
  <https://docs.rocketpy.org/en/latest/notebooks/monte_carlo_analysis/monte_carlo_class_usage.html>
