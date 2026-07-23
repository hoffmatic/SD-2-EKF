# OpenRocket USB simulation operator guide

This historical finite workflow replays an OpenRocket trajectory into the STM32
over its direct USB CDC connection. It remains useful for trajectory inspection,
CSV normalization, and bounded USB replay. It is an **open-loop input replay**:
the CSV supplies the flight path, while STM32 telemetry shows the phase,
estimated state, predicted apogee, and raw airbrake-controller demand.

For repeated RocketPy cases plus one forced 0 -> 3 rotations -> 0 TMC
ramp-state sequence per case, use the root
`Run AMBAR Continuous HIL Test.cmd` launcher and
the continuous-test commissioning guide instead. The finite replay does not
connect raw controller demand directly to physical motor demand.

## Current safety boundary

There are two compile-time build profiles in `Core/Inc/ambar_features.h`:

- `AMBAR_BUILD_PROFILE_NORMAL` is the default. It retains the real flight path
  and rejects simulation, HIL override, HOME/retract, and arbitrary bench-motion
  commands.
- `AMBAR_BUILD_PROFILE_CONTINUOUS_HIL` enables USB replay, software-zero
  geometry, guarded bench checkout, and the separate forced-motion override.

The named artifacts are `Normal/ambar_normal.elf` and
`Continuous_HIL/ambar_continuous_hil.elf`. Reflashing is the authoritative mode
boundary. The Desktop build-and-flash launchers verify the image on disk,
program it, and then query only `PING` and `REQUEST_SNAPSHOT` to prove the
connected board reports the requested profile.

Neither image moves on boot. The existing PCB is unchanged and has no
HOME/FULL switches or actuator encoder. HIL motion requires an explicitly
operator-confirmed software HOME, a healthy TMC5240, an armed and fresh
simulation, and host command `0x22`. Raw STM32 controller demand is telemetry
only in HIL. The supervisor independently commands FORCE_FULL and FORCE_HOME
and labels both signals separately.

The HIL profile fixes full travel at 153600 driver counts (three rotations with
the prototype conversion), VMAX 200000, AMAX 20000, IHOLD 16, IRUN 31,
GLOBALSCALER 0, and the compile-time direction setting. These are register
values, **not calibrated mA, torque, force capacity, encoder measurements, or
flight-qualified limits**. Use a current-limited supply, an independent
latching motor-power/ENN cutoff, and an uncoupled or low-risk mechanism for
first powered checkout. TMC `XACTUAL` is internal driver state, not independent
proof of mechanical position.

`--home-at-current-position` is an explicit operator acknowledgement that the
mechanism has been manually placed fully closed. `CMD_HOME` then declares the
current TMC ramp position as `XACTUAL=0` without seeking or motor motion.
DISARM, `SIM_STOP`, stale input, USB loss, ESTOP, reboot, or any latched fault
clears override and removes motor energy.

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

3. For an energy-off USB streaming smoke test, flash
   `Continuous_HIL/ambar_continuous_hil.elf` and run:

   ```powershell
   & $python replay_openrocket.py "..\..\SimulationData\Source\26-07-10_Openrocket_Run.csv" --live --allow-derived-vertical --no-arm
   ```

   This replays data without arming and without issuing a HIL override. The
   board reports the controller result, but the motor remains de-energized.
   NORMAL intentionally rejects simulation input.

4. Before applying motor power, mechanically place the brakes fully retracted,
   remove hands/tools, use a current-limited supply, keep an immediate physical
   power/ENN cutoff reachable, and preferably disconnect the linkage for the
   first direction test. The firmware watchdog is enabled, but it is not a
   substitute for the independent physical cutoff.

5. Run a two-percent direction/travel checkout before any full trajectory:

   ```powershell
   & $python actuator_checkout.py --percent 2 --allow-actuator-motion --home-at-current-position
   ```

   The flag confirms the mechanism is manually fully closed. Firmware declares
   that position as software zero without moving, commands 3072 counts (or
   -3072 when direction is inverted), then returns to zero and verifies the
   driver is off. Watch the mechanism continuously because `XACTUAL` is not
   physical position feedback. If extension moves toward a hard stop, cut
   motor power immediately. Change `AMBAR_ACTUATOR_DIRECTION_INVERTED` in
   `Core/Inc/ambar_features.h`, rebuild, reflash, and repeat.

6. Use `replay_openrocket.py` only for finite data replay and evidence-bundle
   compatibility. Do not use its raw controller output as a physical actuator
   command. The supported full-stroke hardware path is:

   ```powershell
   ..\..\..\..\Run AMBAR Continuous HIL Test.cmd
   ```

   That launcher requires the mechanism to be manually fully closed. The
   supervisor validates the HIL profile and driver, declares software HOME once
   per process, replays generated RocketPy cases, records raw controller
   demand, then separately forces 153600 and zero through command `0x22`.

7. Inspect continuous results under
   `%LOCALAPPDATA%\AMBAR\TestRuns\<session-id>\`. SQLite is the live source of
   truth; every finalized run also has its case, trajectory, telemetry,
   configuration, manifest, verdict, and portable CSV exports. A fault stops
   the entire session and never resumes automatically.

8. In the GUI, plot at least altitude, vertical velocity, predicted apogee,
   phase, deployment percent, and actuator target/actual counts. Treat the 55 N
   value as unverified metadata, not a measured motor capability.

   Only one process can own the STM32 COM port. With `--gui-udp-port`, the replay
   process owns COM and sends decoded newline JSON datagrams to
   `127.0.0.1:PORT`; the GUI must only listen on UDP. Alternatively, close the
   replay process and let the GUI import the normalized CSV and reuse
   `rocket_protocol.py` on its one serial connection. Two programs cannot share
   one COM port.

Do not claim that a hardware replay passed until the continuous supervisor has
actually run and the per-run `verdict.json` says `PASS`. Endurance acceptance
requires the separately documented 100-cycle qualification with independent
observation and zero thermal, driver, protocol, or evidence failures.
`XACTUAL` is still internal driver state, not encoder or endstop proof that the
mechanism physically followed it.

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

The HIL forced-motion path commands these counts, but that does not prove they
equal the mechanism's safe physical travel. Verify motor steps/rev,
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
