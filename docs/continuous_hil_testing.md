# AMBAR Continuous RocketPy/HIL Testing

## Evidence boundary

The continuous test system combines two separate lanes:

1. RocketPy plus the production STM32-C estimator/controller generates and
   replays a deterministic flight trajectory.
2. The CONTINUOUS_HIL image separately commands the actuator from 0 to 153,600
   TMC5240 counts and back to 0.

The forced actuator command does not modify the already-generated RocketPy
trajectory. Reports and charts keep RocketPy/SIL truth, raw STM32 controller
output, and forced HIL demand separately labeled.

The existing PCB is unchanged. PA0 and PA1 remain `LED_2` and `LED_1`. There
are no HOME/FULL switches and no independent actuator encoder. TMC5240 target
and `XACTUAL` are internal ramp-generator state. They do not prove that the
motor shaft, linkage, or airbrakes physically followed the commanded counts.

## Firmware profiles

- `NORMAL` is the default flight image. It uses real sensors, radio, EKF,
  controller, actuator, and watchdog and rejects simulation, HIL override, and
  arbitrary bench motion.
- `CONTINUOUS_HIL` enables USB trajectory replay and the forced 0 -> 153600 ->
  0 command sequence while disabling radio and onboard flight logging.

Reflashing is the normal/test boundary. Neither image moves the motor on boot.

```text
firmware/stm32_airbrake_pcb/Normal/ambar_normal.elf
firmware/stm32_airbrake_pcb/Continuous_HIL/ambar_continuous_hil.elf
```

## Software HOME

Before starting a supervisor process, the operator must manually place the
mechanism fully closed. The Desktop launcher requires the operator to type
`CLOSED`; direct Python use requires `--accept-current-position-home`.

Only after profile, communications, driver, fault, and energy-off checks pass
does the supervisor send `CMD_HOME`. In this switch-free configuration,
`CMD_HOME` does not seek or move. It declares the current TMC5240 ramp position
to be software HOME (`XACTUAL=0`, target 0).

That declaration occurs once per supervisor process:

- Later cycles verify HOMED, target near 0, `XACTUAL` near 0, driver healthy,
  and motor energy off.
- Later cycles never silently send `CMD_HOME` or re-zero.
- A process restart, explicit resume, board reboot, lost HOMED flag, or failed
  cycle requires the mechanism to be manually closed again and a fresh
  acknowledgement.
- A fault stops the complete session and is never automatically resumed.

This rule prevents an automatic re-zero from hiding an interrupted or
incomplete return.

## Desktop operation

Repository launchers:

```text
Build & Flash AMBAR Normal.cmd
Build & Flash AMBAR Continuous Test.cmd
Run Continuous HIL Test.cmd
```

The continuous launcher:

1. explains the software-only position boundary;
2. requires the typed `CLOSED` acknowledgement;
3. prepares the Python environment and controller bridge;
4. starts the localhost dashboard without serial-port access;
5. runs the supervisor as the sole COM-port owner.

The historical finite Monte Carlo launcher and July 15 results remain SIL
artifacts and do not move hardware.

## Continuous cycle

After the one-time software-HOME declaration, every cycle:

1. verifies the CONTINUOUS_HIL profile, communications, healthy driver,
   retained software HOME, energy-off state, and available storage;
2. generates and persists the seeded RocketPy/SIL case;
3. starts simulation, stabilizes samples, and arms;
4. replays at 50 Hz while recording raw STM32 controller demand;
5. after burn and coast entry, requests `FORCE_FULL`;
6. requires target and `XACTUAL` near 153,600 counts;
7. during recovery or at the bounded hold deadline, requests `FORCE_HOME`;
8. requires target and `XACTUAL` near zero, the software stroke sequence flag,
   and motor energy off;
9. finalizes evidence and dwells retracted for 30 seconds.

The 8-second travel deadline, stale-data checks, USB cleanup, ESTOP handling,
driver warning/fault checks, and write/free-space checks remain active.
Because `XACTUAL` is not shaft feedback, the ramp-state timeout is not
independent mechanical stall detection.

Defaults are 50 Latin-hypercube randomized cases per rolling batch, one fixed
baseline after every ten randomized cases, a persisted session master seed,
deterministic run seeds, and indefinite operation until stopped or faulted.

## Data and recovery

Live evidence is outside OneDrive:

```text
%LOCALAPPDATA%\AMBAR\TestRuns\<session-id>\
```

SQLite in WAL mode is the live query surface. Each run also retains ordered
JSONL events, the RocketPy profile, SIL time series, resolved configuration,
manifests, telemetry, and verdict. `runs.csv` and `parameters.csv` are atomic
portable exports. Finalized runs and report snapshots are mirrored to:

```text
C:\Users\hoffm\OneDrive\Desktop\AMBAR_Continuous_Test_Results\latest
```

Interrupted logs are truncated to the last complete record and an active run is
marked aborted. Recovery never infers that the mechanism is physically closed
or software-homed. `--resume` and `--rebuild-database` are explicit actions,
and every new supervisor process still requires a fresh manual-close
acknowledgement and software-HOME declaration.

The session stops below 5 GiB free space rather than deleting evidence.

## Dashboard

The dashboard binds to localhost, reads SQLite read-only, and receives
best-effort UDP event copies. It never imports serial support or opens COM.

It displays:

- RocketPy truth versus STM32 altitude and velocity;
- predicted versus target apogee;
- raw controller demand versus forced HIL demand;
- motor target versus TMC `XACTUAL` and their ramp-state tracking difference;
- software zero/full states, phases, faults, lag, and skipped samples;
- pass/fail totals, command-time trends, and recent runs.

Every position view explicitly states that software HOME/FULL and `XACTUAL`
are not switches, endstops, or encoder measurements. The dashboard refreshes
within 500 ms and produces portable snapshots at ten-run checkpoints and
shutdown. Closing it does not stop the supervisor.

## Qualification

Use a guarded, current-limited commissioning setup and an independent
latching motor-power or `DRV_ENN` cutoff. First prove direction with a small
move, then run one commanded 0 -> 153600 -> 0 sequence, one RocketPy replay,
20 supervised cycles, and 100 supervised qualification cycles.

Qualification requires zero TMC warnings, driver faults, protocol faults,
deadline failures, or incomplete records. Because the board has no endpoint
feedback, this qualification can establish repeatable commanded/ramp-state
behavior only when a person independently observes or measures the mechanism.
It must not be described as encoder-verified or endstop-verified travel.
