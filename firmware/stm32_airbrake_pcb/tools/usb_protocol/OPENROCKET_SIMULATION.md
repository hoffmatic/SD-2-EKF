# OpenRocket USB simulation operator guide

This workflow replays an OpenRocket trajectory into the STM32 over its direct
USB CDC connection. It is an **open-loop input replay**: the CSV supplies the
flight path, while STM32 telemetry shows the phase, estimated state, predicted
apogee, and airbrake deployment. The tool supports a default inhibited workflow
and a separately gated presentation-motion workflow.

## Current safety boundary

The current demo copy is explicitly identified in `Core/Inc/ambar_features.h`:

```c
#define AMBAR_FEATURE_PRESENTATION_MOTION 1
#define AMBAR_FEATURE_ACTUATOR AMBAR_FEATURE_PRESENTATION_MOTION
#define AMBAR_FEATURE_BENCH_ACTUATOR_COMMANDS AMBAR_FEATURE_PRESENTATION_MOTION
```

Set the presentation switch to `0`, rebuild, and reflash for a motion-inhibited
USB/sensor showcase. With it at `1`, startup is still unhomed and DRV_ENN stays
disabled. Physical replay additionally requires both
`--allow-actuator-motion` and `--home-at-current-position`; the board must report
the presentation, USB, simulation, actuator, and bench feature bits plus a
healthy TMC5240 and valid configuration. `HOME` does not seek a switch: it
declares the mechanism's current physical position to be fully retracted.

The presentation feature forces the reviewed prototype register profile on
each boot so saved flash data cannot silently change it: 153600 counts full
travel, VMAX 200000, AMAX 20000, IHOLD 16, IRUN 31, GLOBALSCALER 0, and the
compile-time direction setting. Those are register values copied from the
prototype that moved; they are **not calibrated mA, torque, force capacity, or
flight-qualified limits**. Use a current-limited supply, a physical power/ENN
cutoff, and an uncoupled or low-risk mechanism for the first powered checkout.
In live motion mode the host also rejects any mechanical CLI values other than
the matching 3 rotations, 200 full steps/rev, 256 microsteps, and 1:1 gearing,
and reports whether the firmware uses positive or negative extension counts.

Do not bypass preflight. Motion mode establishes DISARM/SIM_STOP, verifies an
energy-off actuator, declares HOME, verifies actual position at HOME, starts the
simulation, and arms only after fresh samples and `SIMULATION_ACTIVE`. It
continuously rejects stale telemetry or actuator status. The host uses
`time.perf_counter()` for its absolute schedule. Late rows are skipped instead
of being burst; host lag over 100 ms aborts, and a completed run fails acceptance
if more than 2% of samples were skipped. After a real automatic deployment,
healthy/armed on-target, descent, or recovery output can make one bounded return
to HOME. That return has progress and absolute timeouts. Firmware stops and
de-energizes instead on DISARM, SIM_STOP, end-of-stream, a 500 ms data gap, USB
loss, DIAG/DRV fault, or lost estimator/arming authority. At normal completion
the host also performs and verifies an explicit retract before declaring PASS.
An error/Ctrl+C stops in place and does not attempt a blind retract.

## Supplied CSV audit

Input file:

`SimulationData\Source\26-07-10_Openrocket_Run.csv`

The supplied export contains 618 numeric rows from 0 to 104.9163 s. Its peak
altitude is 1031.2646 m at 13.9185 s, and the largest source interval is 0.500 s.
It reports `TUMBLE` at 13.881 s just before `APOGEE` at 13.918 s, so that source
model event should be checked. The file has no airbrake position, deployment,
drag, or force channel.

Most importantly, the columns are **Total velocity** and **Total
acceleration**. Those are magnitudes, not signed vertical inputs. They must not
be sent directly to a vertical estimator: descending velocity would stay
positive, and coast acceleration would have the wrong sign. For this file, the
tool provisionally derives signed vertical velocity from altitude and signs the
total-acceleration magnitude using the fitted altitude curvature. It uniformly
resamples the irregular input at 50 Hz so the 500 ms source gaps do not trip the
firmware timeout. This derived path is approximate and is blocked for live use
unless `--allow-derived-vertical` is supplied explicitly.

For the final presentation dataset, re-export from OpenRocket with these
columns:

- Time (s)
- Altitude (m)
- Vertical velocity (m/s)
- Vertical acceleration (m/s^2)

The explicit vertical columns remove the approximation and the acknowledgement
flag. Keeping Total velocity and Total acceleration as extra reference columns
is fine.

The current STM32 validates but does not use the optional simulation-velocity
field to update its EKF. It estimates velocity by integrating the supplied
vertical acceleration and correcting altitude with the supplied barometer
value. Keep Vertical velocity in the dataset so the GUI can compare the
reference trajectory against the STM32 estimate; do not expect it to force the
board estimate directly.

## Step-by-step operation

Run these commands from the project's `tools\usb_protocol` folder.

1. Select Python, install the one host dependency, and confirm the PCB USB
   port. Start with the system interpreter. If it is unavailable on the
   presentation PC, uncomment the portable path to Codex's bundled runtime:

   ```powershell
   $python = "python"
   # $python = "$env:USERPROFILE\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
   & $python -m pip install pyserial==3.5
   & $python probe_usb.py --list
   & $python probe_usb.py
   ```

   This installs PySerial 3.5 into the selected interpreter. Rerunning the
   install command is safe when preparing the presentation computer.

   The probe auto-detects the AMBAR USB VID/PID. Windows may assign a new COM
   number after reconnecting; add `--port COMx` only to select one explicitly.

2. Inspect and resample the supplied data without opening any serial port:

   ```powershell
   & $python replay_openrocket.py "..\..\SimulationData\Source\26-07-10_Openrocket_Run.csv"
   ```

   Read every warning. The final line must say that this was a dry run and that
   no command was sent.

   To give the partner GUI a language-neutral, uniformly timed input file, add
   an export path:

   ```powershell
   & $python replay_openrocket.py "..\..\SimulationData\Source\26-07-10_Openrocket_Run.csv" --export-csv "..\..\SimulationData\26-07-10_Openrocket_Run_50Hz_DERIVED.csv"
   ```

   The output includes the one-second pad interval and explicit units. The tool
   also writes a `.csv.metadata.json` sidecar with the source SHA-256, schema,
   timing, target, vertical-channel status, and unverified mechanical display
   assumptions. Its velocity/acceleration are still provisional until the
   source is re-exported with the true Vertical columns.

3. For an energy-off USB streaming smoke test, first set
   `AMBAR_FEATURE_PRESENTATION_MOTION` to `0`, rebuild, and reflash. Then run:

   ```powershell
   & $python replay_openrocket.py "..\..\SimulationData\Source\26-07-10_Openrocket_Run.csv" --live --allow-derived-vertical --no-arm
   ```

   The current motion-enabled demo firmware intentionally refuses this default
   live mode; that prevents a motion-capable board from being mistaken for an
   inhibited build.

4. Before applying motor power, mechanically place the brakes fully retracted,
   remove hands/tools, use a current-limited supply, keep an immediate physical
   power/ENN cutoff reachable, and preferably disconnect the linkage for the
   first direction test. The software watchdog is disabled for debugger-friendly
   presentations, so the physical cutoff is mandatory.

5. Run a two-percent direction/travel checkout before any full trajectory:

   ```powershell
   & $python actuator_checkout.py --percent 2 --allow-actuator-motion --home-at-current-position
   ```

   This declares the current position HOME, commands 3072 counts (or -3072 when
   the compiled direction bit is inverted), waits for the motor to stop, then
   retracts and verifies the driver is off. Watch the mechanism continuously.
   If extension moves toward a hard stop, cut motor power immediately. Change
   `AMBAR_PRESENTATION_ACTUATOR_DIRECTION_INVERTED` in
   `Core/Inc/ambar_features.h`, rebuild, reflash, and repeat. Do not advance to a
   full replay until this low-travel check moves in the correct direction and
   returns to the same physical HOME.

6. Run the synthetic presentation replay and allow the real STM32 controller to
   move the motor. Record a durable evidence bundle; add the UDP option when the
   partner GUI is listening on that localhost port:

   ```powershell
   & $python replay_openrocket.py "..\..\SimulationData\Source\26-07-10_Openrocket_Run.csv" --live --allow-derived-vertical --allow-actuator-motion --home-at-current-position --run-bundle "..\..\SimulationData\Evidence\rehearsal-01" --gui-udp-port 52100
   ```

   The target apogee is 914.4 m (3000 ft), the stream is 50 Hz, and the replay
   stops two seconds after OpenRocket apogee. Omit `--port` to auto-detect
   `0483:5740`. This exact supplied profile drives the real C flight stack to a
   100% request, so it can command the full 153600-count assumed travel. Do not
   use it as the first powered direction test.

7. Progress output shows board-reported target and actual counts. A normal finish
   prints that RETRACT completed and the driver is off, followed by `Replay
   acceptance verdict: PASS`. Inspect `manifest.json`, `packets.jsonl`, and
   `verdict.json` in the selected bundle directory. If the run aborts, the tool
   sends DISARM/SIM_STOP, writes a FAIL verdict when a bundle was requested, and
   leaves the mechanism stopped in place; inspect the cause before issuing any
   separate retract.

8. In the GUI, plot at least altitude, vertical velocity, predicted apogee,
   phase, deployment percent, and actuator target/actual counts. Treat the 55 N
   value as unverified metadata, not a measured motor capability.

   Only one process can own the STM32 COM port. With `--gui-udp-port`, the replay
   process owns COM and sends decoded newline JSON datagrams to
   `127.0.0.1:PORT`; the GUI must only listen on UDP. Alternatively, close the
   replay process and let the GUI import the normalized CSV and reuse
   `rocket_protocol.py` on its one serial connection. Two programs cannot share
   one COM port.

Do not claim that the live board replay passed until step 6 has actually run and
`verdict.json` says `PASS`. Acceptance includes the expected phase order,
nonzero deployment, host lag at or below 100 ms, no more than 2% skipped samples,
fresh status throughout, zero decoder/error-counter increases, and a final
retracted/de-energized actuator status. `XACTUAL` is still internal driver state,
not encoder proof that the mechanism physically followed it.

## Mechanical assumptions used for display only

The replay defaults encode the current discussion, not validated hardware
requirements:

- 4 airbrakes
- 55 N per airbrake at full extension, or 220 N total
- 3 motor rotations from retracted to fully extended
- prototype conversion of 200 full steps/rev, 256 microsteps, and 1:1 gearing

The 55 N value is ambiguous until measurement establishes whether it means
aerodynamic force on each blade, actuator/linkage resistance, or another load
case. It is not applied to the trajectory or controller. A simple sensitivity
band is:

| Assumed force per brake | Four-brake total |
| ---: | ---: |
| 40 N | 160 N |
| 55 N | 220 N |
| 70 N | 280 N |

With the prototype step conversion, the intended display values are:

| Deployment | Motor rotations | Prototype-equivalent counts |
| ---: | ---: | ---: |
| 25% | 0.75 | 38,400 |
| 50% | 1.50 | 76,800 |
| 75% | 2.25 | 115,200 |
| 100% | 3.00 | 153,600 |

The presentation firmware now commands these counts, but that does not prove
they equal the mechanism's safe physical travel. Verify motor steps/rev,
microstepping, gearing, lead-screw pitch, usable travel, home reference, end
stops, and required current before coupling a loaded mechanism.

## What a future closed-loop flight simulation still needs

The current CSV can test USB transport, state estimation, flight-phase logic,
and intended deployment against a fixed trajectory. It cannot show the
airbrakes changing that trajectory. A closed-loop model needs, at minimum:

- rocket mass versus time and a baseline aerodynamic/drag model;
- atmospheric density (and preferably Mach/Reynolds conditions) over altitude;
- airbrake geometry and measured `CdA` or drag increment versus deployment;
- a clear reference speed/density for any stated 55 N aerodynamic load;
- actuator travel, speed, acceleration, latency, backlash, and position limits;
- linkage force/torque and motor-current measurements under representative load;
- validation trajectories for multiple fixed airbrake positions.

For an aerodynamic interpretation, force is speed-dependent:
`F = 0.5 * rho * v^2 * CdA`. A single 55 N figure cannot define `CdA` without
the reference air density and velocity. Once those inputs are measured, the
simulator can integrate the rocket dynamics and feed the resulting altitude and
acceleration back to the STM32 on every 20-50 Hz step.
