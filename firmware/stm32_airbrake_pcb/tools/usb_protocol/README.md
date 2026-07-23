# AMBAR direct-USB protocol

The STM32 enumerates as a CDC serial device named **AMBAR Airbrake USB**.  This
port is binary-only: do not send terminal text or redirect `printf` to it.

Each packet is `COBS(header || payload || CRC16-LE) || 0x00`.  All multi-byte
values are little-endian.  The authoritative field and command definitions are
in `Core/Inc/rocket_protocol.h`; `rocket_protocol.py` is a dependency-free host
codec intended for the presentation GUI.

## Quick USB test on Windows

1. Flash `Normal/ambar_normal.elf`,
   `Continuous_HIL/ambar_continuous_hil.elf`, or
   `Variable_HIL/ambar_variable_hil.elf` from CubeIDE. The Desktop
   build-and-flash launchers also verify the connected board reports the
   requested profile after programming.
2. Connect a data-capable USB cable to the PCB's own USB-C connector.
3. Select Python and install the one host dependency. Start with the system
   interpreter; the commented fallback selects the bundled Codex runtime used
   on the presentation PC when `python` is unavailable:

   ```powershell
   $python = "python"
   # $python = "$env:USERPROFILE\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
   & $python -m pip install pyserial==3.5
   ```

4. From this folder, list ports and probe the board:

   ```powershell
   & $python probe_usb.py --list
   & $python probe_usb.py
   ```

The probe sends `PING` and `REQUEST_SNAPSHOT`.  A working link returns binary
ACK, telemetry, and actuator-status packets.  The COM number is assigned by
Windows and may differ between computers.

## Finite USB replay compatibility

For the supplied OpenRocket flight and a finite replay, follow
[OPENROCKET_SIMULATION.md](OPENROCKET_SIMULATION.md). The accompanying
`replay_openrocket.py` defaults to a read-only dry run and uniformly resamples
the trajectory. Live simulation input is accepted only by the two HIL profiles.
A finite `--no-arm` replay is data-only: it exercises
USB, replay, estimation, phases, and controller output without energizing the
motor. The full commanded 0 -> 153600 -> 0 ramp-state sequence is owned by the continuous
supervisor, which labels raw controller demand and forced actuator demand
separately. Because the historical export contains Total rather than Vertical
velocity/acceleration, its provisional live replay also requires the explicit
`--allow-derived-vertical` acknowledgement.

The tool can export a normalized CSV plus JSON metadata sidecar for a partner
GUI. Only one process may own the Windows COM port. There are two supported
architectures:

1. Let `replay_openrocket.py` own COM and add `--gui-udp-port PORT`; the GUI
   listens on `127.0.0.1:PORT` for one newline-terminated JSON event per UDP
   datagram.
2. Close the replay process and let the GUI import the normalized rows, reuse
   `rocket_protocol.py`, and own the single serial connection itself.

The localhost mirror includes decoded transmit/receive packets, timing/skip
events, run state, and the final verdict. UDP is presentation-only and
best-effort; loss cannot alter USB cleanup or actuator safety.

A typical GUI datagram is one UTF-8 JSON object followed by `\n`:

```json
{"schema":"ambar.live_replay_event.v1","event":"packet_rx","host_elapsed_s":1.25,"packet_type":1,"sequence":42,"packet_time_ms":9876,"packet_time_domain":"stm32_device","decoded":{"altitude_m":123.4,"velocity_mps":56.7,"deployment_percent":25}}
```

The GUI should filter `event == "packet_rx"`, then use `packet_type` and the
`decoded` object. Packet type constants are in `rocket_protocol.py`. Other event
names include `packet_tx`, `sample_sent`, `sample_skipped`, `run_failed`, and
`run_verdict`. `host_elapsed_s` orders the local transcript; it is not the same
epoch as `packet_time_ms` from the STM32.

For a durable rehearsal record, add `--run-bundle DIR`. The directory contains
`manifest.json`, the ordered `packets.jsonl` transcript, and `verdict.json`.
The verdict requires the phase sequence, deployment, final software-HOME/driver-off
state where applicable, clean protocol counters, at most 100 ms host lag, and
no more than 2% skipped source samples. Scheduling uses the high-resolution
`time.perf_counter()` clock; STM32 packet time remains a separate time domain.

1. Send `SIM_START` as a command packet. The STM32 disarms and resets the
   estimator; wait for its ACK.
2. Stream `PKT_SIMULATION` samples at 20-50 Hz. Altitude is millimetres,
   acceleration is mm/s^2, and the optional velocity is reference-only: current
   firmware validates it but does not feed it into the EKF.
3. Send `SET_ARMED` with value `1` only after the simulation stream is stable.
4. Read 20 Hz telemetry. `deployment_percent` is the intended flight response;
   actuator-status separately explains whether hardware movement is inhibited.
5. In `CONTINUOUS_HIL`, `deployment_percent` remains telemetry-only and only
   `ROCKET_CMD_HIL_SET_OVERRIDE` may energize motion. In `VARIABLE_HIL`, forced
   override is unsupported and the normal fractional controller output drives
   the guarded actuator target.

`AMBAR_BUILD_PROFILE_NORMAL` is the default and rejects simulation, HIL
override, HOME/retract, and arbitrary bench-motion commands. The separately
flashable `AMBAR_BUILD_PROFILE_CONTINUOUS_HIL` image enables USB replay,
software-zero geometry, guarded bench checkout, and HIL override. The separate
`AMBAR_BUILD_PROFILE_VARIABLE_HIL` image enables causal 50 Hz
simulation and fractional actuation but rejects forced override and arbitrary
bench moves. Reflashing is the authoritative mode boundary; there is no runtime
profile switch.

Before continuous testing, manually place the mechanism fully closed and use
`actuator_checkout.py` for a two-percent direction/travel check.
`--home-at-current-position` explicitly authorizes firmware to declare the
current TMC ramp position as software zero without seeking or motion. The
checkout moves to a bounded target, returns to zero, and verifies driver-off.
There is no endpoint switch or independent encoder.

## GUI command reference

The decoded 9-byte header is `<magic:u8, version:u8, type:u8, sequence:u16,
time_ms:u32>`. A command packet (`type=0x10`) has payload
`<command:u8, data_length:u8, data...>`. Increment the 16-bit sequence for every
new command. An exact retry within the most recent eight commands receives the
cached ACK and is not executed twice; reusing a sequence with different data is
rejected.

| Code | Command data | Behavior |
| ---: | --- | --- |
| `0x00` | none | NOP |
| `0x01` | none | PING |
| `0x02` | none | Request immediate telemetry/actuator snapshot |
| `0x10` | `uint16` target in 0.1 m | Set target apogee; valid wire range is 10.0-6553.5 m; rejected while armed |
| `0x11` | `uint8`, 0 or 1 | Disarm/arm; simulation requires a valid sample before arming |
| `0x15` | none | Emergency stop and disarm |
| `0x16` | none | Declare the operator-confirmed fully closed current TMC ramp position as software HOME (`XACTUAL=0`); no seek/motion; HIL-only and rejected while armed |
| `0x17` | none | Request retract; feature-gated and rejected while armed |
| `0x18` | none | Re-capture physical pad reference; rejected while armed or simulating |
| `0x19` | none | Save config; USB flash-maintenance feature-gated and off by default |
| `0x1A` | `int32` absolute steps | Bench move; feature/range-gated and rejected while armed |
| `0x20` | none | Start simulation, disarm, and reset estimator |
| `0x21` | none | Stop simulation, disarm, and reset estimator |
| `0x22` | `uint8` (`0` OFF, `1` FORCE_FULL, `2` FORCE_HOME) | `CONTINUOUS_HIL`-only forced demand; always unsupported by `VARIABLE_HIL` |
| `0x23` | none | `VARIABLE_HIL` full versioned controller-config readback |
| `0x30` | none | Start flash logging |
| `0x31` | none | Stop flash logging |
| `0x32` | none | Erase log; USB flash-maintenance feature-gated and off by default |

Codes `0x12`-`0x14` are retained for compatibility but intentionally return
`UNSUPPORTED`. ACK payloads decode as `<command_sequence:u16, command:u8,
result:u8, detail:u16>`; result values are `0 OK`, `1 BAD_LENGTH`, `2 BAD_VALUE`,
`3 UNSUPPORTED`, `4 BUSY`, `5 EXECUTION_ERROR`, and `6 BAD_CRC`.

`rocket_protocol.py` provides builders plus decoders for ACK, telemetry, event,
actuator-status, and heartbeat packets. A simulation payload is exactly
`<flags:u16, altitude_mm:i32, acceleration_mmps2:i32, velocity_mmps:i32,
barometer_stddev_cm:u16>`. Normal samples must mark both altitude and
acceleration valid. Each altitude observation is applied to the EKF once;
acceleration is held between incoming samples.

`VARIABLE_HIL` adds packet type `0x21` for the atomic 52-byte host config
upload, type `0x05` for its full CRC-bearing readback, and type `0x04` for the
44-byte sequence-correlated board state. A valid upload ACK uses command identity
`0x24`; a readback request alone never satisfies the arm gate. The state packet
header and payload both echo the accepted simulation sequence; config-readback
headers echo the upload or GET request sequence. The state packet reports
controller fraction, motor target, and TMC5240 `XACTUAL` separately.
Feedback source `1` means ramp-state only, never an encoder.
