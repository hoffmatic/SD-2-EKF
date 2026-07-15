# Project Status and Evidence Levels

Status date: July 15, 2026

This page separates code that exists from behavior proven in simulation, on the
bench, or in flight. “Implemented” does not mean hardware-qualified.

## Current Architecture

| Area | Current baseline | Evidence level |
| --- | --- | --- |
| Production flight logic | STM32-C `ambar_ekf.c` + `ambar_flight.c` | Host-built, replay-tested, and integrated into CubeIDE project |
| Apogee prediction | Provisional vertical drag-aware predictor plus ballistic comparison | Implemented; mass/CdA/density calibration missing |
| Main physics study | RocketPy closed loop through production STM32-C bridge | Software-in-the-loop screening |
| Robustness study | Seeded Latin-hypercube paired passive/controlled campaign | Reproducible exploratory evidence, not qualification |
| Legacy regressions | C++ `AmbarFlightComputer` sandboxes | Useful fast fixtures; not production controller |
| Mission target | 3000 ft AGL with +/-100 ft tolerance | Fixed acceptance requirement |

## Implemented

- Four-state vertical EKF, barometer innovation gating, covariance/health
  tracking, phase logic, interlocks, and bounded airbrake command.
- Ballistic and provisional drag-aware apogee outputs.
- STM32 application scheduling, runtime configuration, direct USB protocol,
  simulation input, telemetry, flash logging, and feature flags.
- BMP388, LSM6DSV32X, LIS2MDL, W25Q64, SX1280, and TMC5240 code paths in the
  CubeIDE project. Their presence is not a claim of final-board qualification.
- Production-C host bridge used by both one-run RocketPy and Monte Carlo.
- Pad-referenced RocketPy body-axis IMU model, sparse barometer model, actuator
  delay/rate model, and closed aerodynamic feedback.
- Two-baseline plus configurable seeded randomized campaigns with per-run CSV,
  aggregate metrics, sensitivity ranks, representative USB replay profiles,
  atomic checkpoints, and exact input/binary snapshots.

## Current Nominal Software-in-the-Loop Snapshot

| Check | Result | Interpretation |
| --- | --- | --- |
| Deterministic baseline repeat | PASS | Repeated controller histories and outcomes match |
| Passive OpenRocket comparison | FAIL: 3829 ft AGL vs 3379 ft AGL | Vehicle mass/drag reconstruction is not calibrated |
| Production-C closed-loop coupling | PASS: 3327 ft AGL | Command changes the RocketPy trajectory by about 502 ft |
| Target band | FAIL: +327 ft | Current controller/model combination misses 3000 +/-100 ft |
| Maximum Mach | PASS: 0.494 vs 1.0 limit | Nominal model stays subsonic |
| Rail exit | FAIL: 42.7 ft/s vs 52 ft/s minimum | Launch/model configuration must be reconciled |
| First command / virtual motion | 4.42 s / 4.52 s | Both occur after true 1.64 s burnout plus margin |
| Phase sequence | PadIdle -> Boost -> Coast -> AirbrakeActive -> Recovery | Monotonic production phase path |

## Seeded 50-Run Screening Snapshot

Seed `20260715` with the provisional Latin-hypercube ranges produced:

| Metric | Result |
| --- | --- |
| Execution errors | 0/50 |
| Safety passes | 50/50 |
| Effective apogee reduction | 50/50 |
| Target-band hits | 0/50 |
| Complete Mach/rail envelope passes | 0/50; every rail-exit case is below 52 ft/s |
| Controlled apogee | 3225-3493 ft AGL; median 3307 ft; p95 3414 ft |
| Apogee reduction | 149-942 ft; median 449 ft |
| First command | 3.34-6.46 s; no command before randomized burnout plus margin |
| Recovery retraction | All below 2% within 0.66 s of Recovery |

The 0/50 target result has a two-sided 95% Wilson upper success bound of about
7.1%; it is evidence of poor performance under these assumptions, not a precise
estimate of flight success probability.

The values above are above-ground-level. RocketPy’s absolute elevation is
explicitly removed before mission comparisons.

## Simulated, Not Hardware-Verified

- Closed-loop apogee response to airbrake command.
- Estimator behavior under provisional bias, noise, latency, launch angle,
  wind, mass, drag, motor, and actuator variation.
- Timely virtual retraction after Recovery.
- Native legacy fault, electronics, and actuator scenarios.
- OpenRocket-style profiles prepared for selected USB/HIL replays.

## Provisional or Unresolved

- Final flight-ready mass, center of gravity, inertia, rail-button positions,
  power-on/off drag, and `CdA(Mach, deployment)`.
- The current airbrake Mach scaling is an explicit randomized placeholder.
- Sensor noise, bias, mounting-axis accuracy, vibration, static-port lag, and
  timing distributions require measured logs.
- Loaded actuator force, travel conversion, current, temperature, homing,
  stall threshold, and hard-limit behavior require mechanism tests.
- The host bridge uses compiled/default fixed controller values. A physical
  board may load different saved flash configuration; compare configuration
  readback before claiming equivalence.
- The 50-run campaign is an exploratory screen. Even 50/50 successes would
  only bound an unseen failure rate to roughly 5.8% at one-sided 95% confidence.

## Not Yet Proven

- Independent prediction accuracy against a held-out flight or calibrated
  high-fidelity aerodynamic model.
- Final PCB power integrity, bus signal integrity, brownout/watchdog recovery,
  flash endurance, RF range, or environmental behavior.
- Physical motor/airbrake position under aerodynamic load; TMC5240 XACTUAL is
  not an external position encoder.
- End-to-end GUI/radio/flight behavior under launch conditions.
- Flight readiness or reliability probability.

Use [monte_carlo_campaign.md](monte_carlo_campaign.md) for the campaign
workflow and [software_architecture.md](software_architecture.md) for the exact
production-versus-legacy code split.
