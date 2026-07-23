# STM32 Airbrake PCB Firmware

This is the canonical STM32CubeIDE project for the AMBAR airbrake PCB. The
Desktop `stm32_airbrake_pcb` folder should be a junction to this directory, not
an independent firmware copy.

## Flashable profiles

| Profile | Artifact | Behavior |
| --- | --- | --- |
| `NORMAL` | `Normal/ambar_normal.elf` | Real sensors, EKF, radio, controller, flash logging, actuator, watchdog, and fail-soft USB telemetry |
| `CONTINUOUS_HIL` | `Continuous_HIL/ambar_continuous_hil.elf` | USB RocketPy replay, actuator watchdog, and a forced three-rotation TMC ramp-state sequence; radio and onboard flight logging disabled |
| `VARIABLE_HIL` | `Variable_HIL/ambar_variable_hil.elf` | 50 Hz causal USB simulation, the provisional M5 predictor/controller, normal fractional actuator targets, and correlated TMC5240 `XACTUAL` feedback |

`NORMAL` is the compile-time default and rejects simulation input, HIL
override, and arbitrary bench movement. Both HIL profiles report unique
heartbeat and actuator identities. `CONTINUOUS_HIL` alone accepts forced
FULL/HOME override; `VARIABLE_HIL` rejects override and bench movement and uses
the ordinary fractional controller-to-actuator path. The host refuses testing
unless both identity checks match. Reflashing is the profile boundary, and no
image moves the motor on boot.

Build and optionally flash from the repository root:

```text
Build & Flash AMBAR Normal.cmd
Build & Flash AMBAR Continuous Test.cmd
Build & Flash AMBAR Variable HIL.cmd
```

The helper discovers STM32CubeIDE and CubeProgrammer, builds the named
configuration, checks the uniquely named ELF, programs/verifies over SWD, and
then performs read-only runtime profile verification over USB.

## Unchanged hardware and software HOME

HIL does not modify the PCB. PA0 and PA1 remain `LED_2` and `LED_1`
through R24 and R23. There is no HOME/FULL switch harness and no independent
actuator encoder.

Before each supervisor process, the operator manually places the mechanism
fully closed and explicitly acknowledges that condition. After communications,
profile, driver, fault, and energy-off checks pass, `CMD_HOME` sets the current
TMC5240 ramp position and target to zero without seeking or motor motion.

The declaration occurs once per process. Successful later cycles only verify
that HOMED, target near zero, `XACTUAL` near zero, and motor-energy-off remain
true. They never silently re-zero. Restart, resume, board reboot, lost HOMED
state, or a fault requires manual closure and a fresh acknowledgement.

Software FULL is fixed at 153,600 counts from software HOME:

```text
200 full steps/revolution * 256 microsteps * 3 rotations
```

Target, `XACTUAL`, and software HOME/FULL flags are internal TMC5240
ramp-generator evidence. They are not independent proof of motor shaft,
linkage, airbrake, or endpoint position.

## HIL override protocol

Protocol v2 keeps the actuator payload at 24 bytes. Reserved status bits report
software HOME, software FULL, geometry plausibility, active override/mode,
CONTINUOUS_HIL identity, and completed software stroke sequence. The old bit
positions remain wire-compatible.

Command `0x22`, `ROCKET_CMD_HIL_SET_OVERRIDE`, accepts:

```text
0 = OFF
1 = FORCE_FULL
2 = FORCE_HOME
```

Non-OFF override is accepted only by CONTINUOUS_HIL while simulation is active
and fresh, the controller is armed, the driver is healthy, and software HOME
and geometry gates are valid. Override and motor energy clear on stale input,
USB loss, disarm, `SIM_STOP`, ESTOP, reboot, or fault.

Raw STM32 controller demand remains telemetry only. Motor demand comes from the
separately labeled HIL override so reports cannot confuse controller output
with the forced command or with independent position evidence.

## Continuous host system

```text
Run Continuous HIL Test.cmd
```

The launcher requires the operator to type `CLOSED`. The supervisor is the sole
COM owner and:

1. checks storage, profile, communications, driver, faults, and energy-off;
2. declares the manually closed position as software HOME once per process;
3. generates a deterministic RocketPy/STM32-C case;
4. replays at 50 Hz and records raw controller demand;
5. commands software FULL after burn/coast entry;
6. commands software HOME during recovery or at the hold deadline;
7. requires target/`XACTUAL` 0 -> 153600 -> 0, completed software sequence,
   and driver-off;
8. finalizes the run and dwells retracted for 30 seconds.

Defaults are 50 Latin-hypercube randomized cases per rolling batch with a fixed
baseline after every ten randomized cases. A fault stops the session.

Evidence is stored under `%LOCALAPPDATA%\AMBAR\TestRuns\<session-id>\` using
SQLite WAL, ordered JSONL, per-run profiles/time series/configuration/verdicts,
and atomic CSV exports. The localhost dashboard reads SQLite and UDP only; it
never opens the serial port.

See `docs/continuous_hil_testing.md` and
`docs/continuous_hil_commissioning.md`.

## Normal firmware and verification

NORMAL retains the LSM6DSV32X, BMP388, LIS2MDL, SX1280, TMC5240, W25Q64,
vertical EKF, drag-aware apogee prediction, phases, controller, watchdog, and
protocol-v2 telemetry. Simulation and HIL commands return
`ROCKET_ACK_UNSUPPORTED`.

Verification covers protocol compatibility, profile rejection, override
gating/cleanup, software-zero and geometry behavior, travel/no-progress
timeouts, USB replay/evidence, deterministic session planning/recovery, SQLite
and CSV exports, dashboard behavior, the production STM32-C bridge, and
RocketPy/Monte Carlo workflows.

Software tests and successful builds do not qualify the physical mechanism.
Commission with guarding, a current-limited supply, an independent latching
motor-power or `DRV_ENN` cutoff, direct observation/measurement, one small
direction test, one full command sequence, one RocketPy replay, 20 supervised
cycles, and a 100-cycle qualification.

## Variable-deployment HIL

`VARIABLE_HIL` boots the explicit provisional M5 profile in RAM, including its
1.80 s boost dwell, and requires a versioned 52-byte control/predictor upload
before every USB connection may arm. Upload is accepted only while disarmed,
simulation stopped, the driver off, and motion stopped. The full board readback
and CRC must match the host request; simply requesting the current config does
not grant arm authority. After `SIM_START`, ARM is rejected until the board has
consumed at least one fresh, valid simulation sample.

Each accepted simulation sample produces one correlated 44-byte state packet
which echoes the input sequence and keeps controller request, actuator target,
TMC5240 `XACTUAL`, inhibits, phase, driver state, closed/full predictions, and
target reachability distinct. Stale input over 100 ms or USB loss stops the
simulation, disarms, cancels motion, and clears pending correlated output. USB
loss also revokes the uploaded-config arm latch.

This is deliberately labeled **TMC ramp-state-coupled HIL**. `XACTUAL` is a
driver ramp-generator value, not an encoder or proof that the mechanism moved.

## Navigation

- `CODE_GUIDE.md`: architecture and source-reading order.
- `Core/Inc/ambar_features.h`: profiles and feature definitions.
- `Core/Src/ambar_app.c`: scheduler, USB commands, simulation freshness, and
  cleanup.
- `Core/Src/ambar_actuator.c`: software-geometry motion authority.
- `Core/Inc/rocket_protocol.h`: protocol-v2 wire contract.
- `tools/usb_protocol/`: host codec, replay, probe, and checkout utilities.
- `HostTests/`: C and Python regression tests.

This remains experimental rocket flight software. Do not treat simulation,
internal ramp state, or successful compilation as physical flight validation.
