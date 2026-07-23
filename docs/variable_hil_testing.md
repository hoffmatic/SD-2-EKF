# AMBAR Causal Variable-Deployment HIL

## Evidence boundary

`VARIABLE_HIL` is a separate development profile. It closes the RocketPy loop
around the STM32 estimator/controller and the TMC5240 `XACTUAL` ramp state:

```text
RocketPy state at tick n
  -> simulated sensor sample
  -> physical STM32 EKF and controller
  -> normal fractional actuator target
  -> correlated TMC5240 XACTUAL reply
  -> deployment applied to RocketPy interval n+1
```

This is **TMC ramp-state-coupled HIL**. `XACTUAL` is the driver's internal ramp
generator position, not an encoder, shaft measurement, linkage measurement, or
airbrake-position measurement. A successful campaign is development evidence;
it is not mechanical validation or a claim that the real rocket will reach
3,000 ft.

## Profile isolation

The three images have deliberately different authority:

| Profile | Simulation input | Controller drives fractional actuator path | Forced FULL/HOME override |
|---|---:|---:|---:|
| `NORMAL` | no | yes | no |
| `CONTINUOUS_HIL` | yes | no | yes |
| `VARIABLE_HIL` | yes | yes | no |

`NORMAL` retains its legacy controller defaults. The provisional M5
calibration is an explicit, versioned `VARIABLE_HIL` configuration and is not
silently applied to the flight image. `CONTINUOUS_HIL` remains the independent
forced-stroke endurance setup and keeps its existing evidence meaning.

Expected build artifacts are:

```text
firmware/stm32_airbrake_pcb/Normal/ambar_normal.elf
firmware/stm32_airbrake_pcb/Continuous_HIL/ambar_continuous_hil.elf
firmware/stm32_airbrake_pcb/Variable_HIL/ambar_variable_hil.elf
```

No profile may move the motor on boot.

## Configuration handshake

The host uploads one complete versioned controller/predictor payload. The board
accepts it only while all of the following are true:

- flight is disarmed;
- simulation is stopped;
- the actuator driver is off;
- the payload version, ranges, and CRC are valid.

The runner reads the complete payload back and compares every canonical byte
and its CRC. It refuses to arm if the readback differs. Partial per-field
updates are not used for a `VARIABLE_HIL` run.

After `SIM_START`, the board intentionally has no fresh sample and therefore
cannot arm. The runner sends one correlated, zero-motion on-pad freshness
sample, verifies simulation-active/fresh, HOME, driver health, exact XACTUAL
zero, and the expected configuration CRC while the board is still disarmed,
and only then sends `ARM`. The first causal RocketPy physics sample follows
from the 50 Hz callback; the pre-arm sample is not counted as a physics tick.

The provisional M5 payload keeps the documented mass and motor inputs visible,
uses the 3,379 ft passive OpenRocket reference, and records the density assumed
for the documented airbrake load point. Its 0/25/50/75/100% CdA curve is
piecewise linear pending measured or CFD data. Opening and closing rates are
0.864 and 0.844 stroke/s. Command-to-`XACTUAL` delay remains explicitly
provisional until the commissioning measurement replaces it.

## Causal timing and faults

The runner uses one monotonic wall-clock deadline every 20 ms. It never sends a
catch-up burst. On each tick it:

1. samples the current RocketPy state and produces simulated sensor values;
2. sends one sequence-numbered simulation packet;
3. accepts only a correlated `VARIABLE_HIL` state packet echoing that sequence;
4. records controller request, actuator target, `XACTUAL`, and physics-applied
   deployment as four distinct values;
5. applies the last confirmed `XACTUAL` fraction to the next RocketPy interval.

One late tick uses the last confirmed fraction and fails the run. Two
consecutive misses, feedback older than 100 ms, a mismatched sequence, USB
loss, decoder failure, or board fault triggers `DISARM` and `SIM_STOP`, followed
by the bounded cleanup path. The session never compresses or skips simulated
time to catch up.

Motor energization additionally requires the unique `VARIABLE_HIL` identity,
simulation active and fresh, armed state, a manually acknowledged software
HOME, matching controller configuration, valid actuator geometry, and a
healthy driver. USB disconnect, stale simulation, ESTOP, descent/recovery,
driver faults, and normal flight/actuator inhibits remain authoritative.

## Calibration and controller selection

Calibration occurs without motor power before hardware commissioning:

1. **Authority sweep.** At least 96% of intended uncertainty cases must have a
   retracted apogee above 3,100 ft and an earliest-safe fully deployed apogee
   below 2,900 ft. Failure stops tuning and reports insufficient aerodynamic
   authority.
2. **Fixed-deployment predictor gate.** Held-out coast prediction must have
   RMSE at most 50 ft, first-action bias within +/-25 ft, and p95 absolute error
   at most 75 ft.
3. **Controller tuning.** Evaluate the calibrated proportional controller on
   200 fixed-seed tuning cases. Retain it only if every robust gate passes.
4. **Predictive fallback.** Otherwise use the 20 Hz predictive fraction solver:
   closed/full bracketing, eight bounded bisection iterations, altitude-varying
   density, the CdA curve, current deployment, provisional delay, asymmetric
   rates, a 10 ft controller deadband, and 2% command hysteresis. Saturation is
   allowed only when the target lies outside the current authority bracket.
5. **Frozen validation.** Freeze the payload and run 500 unseen cases. Do not
   tune on this held-out set.

The mission band remains 3,000 +/-100 ft; it is not the controller deadband.

### Current provisional gate status (2026-07-19)

- The passive fit produces 3,378.998 ft against the documented 3,379 ft
  reference without changing dry mass, motor mass, or the J420R curve.
- The plant-uncertainty authority sweep brackets 3,000 ft in every completed
  case. The deterministic fixed-deployment 200-case tuning and 500-case
  held-out predictor gates pass by a wide margin.
- The proportional controller fails the robust one-shot screen by over-braking,
  so the frozen profile selects the predictive solver as required.
- One full RocketPy proportional smoke reaches 3,002.425 ft and one predictive
  smoke reaches 3,007.433 ft; both pass their safety, effectiveness, and target
  checks. Both overall smoke campaigns remain `FAIL` because the current plant
  exits the rail at 42.586 ft/s against the pre-existing 52 ft/s launch-envelope
  requirement. That discrepancy occurs before airbrake action.
- The full 200/500 closed-loop RocketPy campaigns remain intentionally unclaimed
  until the rail/launch model discrepancy is resolved. The deterministic
  predictor gates are not a substitute for those campaigns.

## Commissioning order

Keep motor power disconnected for protocol/config replay and identical-trace
comparison. When motion is later authorized, use a guarded and current-limited
setup with an independent cutoff and proceed in this order:

1. manually close the mechanism and acknowledge software HOME;
2. track 25%, return HOME, and verify driver off;
3. repeat at 50%, 75%, and 100%;
4. measure command-to-`XACTUAL` motion delay and replace the provisional value;
5. run supervised low-, nominal-, and high-energy causal cases;
6. only then run the 25-cycle campaign.

Every run must end with target zero, `XACTUAL` zero, override off, software HOME
retained, and driver off. A failed cleanup stops the campaign.

## Acceptance

The 500-case held-out development gate requires:

- at least 96% inside 2,900--3,100 ft;
- p95 absolute apogee error at most 100 ft;
- worst absolute error at most 200 ft;
- 100% safety and effectiveness passes.

The final supervised hardware campaign requires 25/25 hardware-safety and
tracking passes, at least 24/25 RocketPy apogees in band, p95 error at most
100 ft, worst error at most 200 ft, no protocol/USB/driver/cleanup failure, and
the required HOME/driver-off ending state on every run.

The dashboard reports hardware safety and 3,000 ft performance separately. A
hardware pass never converts a RocketPy miss into a target pass.

## Results and storage

Variable-HIL sessions use a separate results root and identify their coupling
mode as `TMC_RAMP_STATE_COUPLED`. Each sample preserves:

- controller-requested fraction;
- actuator target fraction and steps;
- TMC5240 `XACTUAL` fraction and steps;
- exact fraction applied to RocketPy physics;
- feedback age/source and correlated sequence;
- closed/full predicted apogees and reachability;
- flight/actuator inhibits, profile/driver state, and configuration CRC.

`forced_hil_deployment_fraction` is always null in this setup.

## Launchers

The PowerShell launcher owns dependency setup, the read-only dashboard, the
runner, a separate `%LOCALAPPDATA%\AMBAR\VariableHilRuns` results root, and the
final portable snapshot. Without `-Hardware` it uses only the deterministic
fake transport and never imports pyserial or opens a COM port.

After all motor-off and staged commissioning gates have passed, start a
supervised 25-cycle campaign with:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\run_variable_hil.ps1 `
  -Hardware -AllowActuatorMotion -Port COMx -Cycles 25
```

The launcher will prompt for `CLOSED`; `-AcceptCurrentPositionHome` is reserved
for an operator who has already made that exact acknowledgement in the same
guarded procedure. `Run Variable HIL Test.cmd` and
`Run AMBAR Variable HIL Test.cmd` call the same PowerShell entry point.
