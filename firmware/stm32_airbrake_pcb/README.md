# STM32 Airbrake PCB Firmware

This folder contains the STM32CubeIDE firmware copy for the AMBAR airbrake PCB.
It was generated from the local Desktop project named `MCU Project EKF PCB Copy`.

## What This Firmware Does

- Reads the PCB sensors over the board buses:
  - LSM6DSV32X IMU on I2C1
  - BMP388 barometer on I2C2
  - LIS2MDL magnetometer on I2C3
  - SX1280 radio on SPI1
  - TMC5240 motor driver on SPI2
  - W25Q64 flash on SPI3
- Runs a vertical EKF for altitude, velocity, accelerometer bias, and barometer bias.
- Predicts apogee with both ballistic and drag-aware estimators.
- Sends the older telemetry layout first, then appends versioned extra sections.
- Sends the full telemetry frame at 1 Hz with nonblocking SX1280 TX so the
  125 Hz IMU and 50 Hz barometer schedules keep running while the packet is on air.
- Accepts radio commands for pad reset, arming, logging, config, emergency stop, and bench movement requests.
- Enumerates the PCB's own USB-C connector as a binary CDC serial interface
  named `AMBAR Airbrake USB`.
- Accepts framed USB commands and presentation simulation samples without
  changing the working Arduino/SX1280 version-1 packet format.
- Sends 20 Hz USB flight telemetry plus separate actuator intent/inhibit status.
- Returns a previously deployed actuator toward HOME during a healthy, armed
  on-target/descent/recovery state, with progress and absolute timeouts.
- Uses a high-resolution host replay clock, optional evidence bundles, and an
  optional localhost JSON mirror so one process remains the USB COM owner.
- Logs flight/bench snapshots to external flash.
- Provides a separately identified presentation-motion build; startup still
  keeps DRV_ENN disabled until HOME, health, arming, simulation, and flight
  controller gates all pass.

## Safety Defaults

All presentation and bench switches live in one file:

```text
Core/Inc/ambar_features.h
```

The current presentation build uses one master switch:

```c
#define AMBAR_FEATURE_PRESENTATION_MOTION 1
#define AMBAR_FEATURE_ACTUATOR AMBAR_FEATURE_PRESENTATION_MOTION
#define AMBAR_FEATURE_BENCH_ACTUATOR_COMMANDS AMBAR_FEATURE_PRESENTATION_MOTION
```

Set `AMBAR_FEATURE_PRESENTATION_MOTION` to `0` and rebuild/reflash to create an
energy-off sensor/USB showcase build. With it set to `1`, movement still
requires valid configuration, a healthy TMC5240, an operator-declared HOME, no
latched fault or emergency stop, stable simulation data, and an explicit ARM.
Only a nonzero automatic deployment that actually passed every hardware gate
authorizes one automatic return to HOME. That return is permitted only while
the estimator remains healthy and the controller remains armed in an on-target,
descending, or recovery state. It has a 2.5 s progress timeout and an 8 s
absolute timeout. DISARM, SIM_STOP, simulation timeout, USB disconnect, ESTOP,
or a fault instead cancel motion and de-energize the driver. Flight hardware
should not be trusted until travel, direction, current limits, pressure venting,
mechanism stops, and hardware-in-the-loop replay have been verified.

A rejected IMU sample inhibits the airbrake command immediately. To prevent a
single recoverable radio/scheduler timing gap from permanently stranding the
phase machine, `FAULT` is latched only after 10 consecutive rejected IMU
updates. A later good sample clears the transient health condition before that
threshold; a sustained outage remains fail-safe and latches `FAULT`.

## Feature Switches

Edit only the `0` or `1` defaults in `Core/Inc/ambar_features.h`, then rebuild
and reflash the STM32. The available switches are:

| Switch | Normal default | Purpose |
| --- | ---: | --- |
| `AMBAR_FEATURE_RADIO` | 1 | SX1280 startup, receive, and command replies |
| `AMBAR_FEATURE_TELEMETRY` | 1 | One-hertz binary telemetry |
| `AMBAR_FEATURE_RADIO_HEARTBEAT` | 1 | Five-second fallback heartbeat |
| `AMBAR_FEATURE_USB_PROTOCOL` | 1 | Direct STM32 CDC and protocol-v2 packets |
| `AMBAR_FEATURE_SIMULATION_INPUT` | 1 | Explicitly gated GUI simulation samples |
| `AMBAR_FEATURE_USB_REQUIRE_VBUS_SENSE` | 1 | Start USB only when PA9 sees cable VBUS |
| `AMBAR_FEATURE_USB_FLASH_MAINTENANCE` | 0 | Allow blocking save/erase commands over USB; keep off for demos/flight |
| `AMBAR_FEATURE_FLASH_LOGGING` | 1 | Log commands and five-hertz snapshots |
| `AMBAR_FEATURE_PRESENTATION_MOTION` | 1 | One-switch identity/gate for the USB-to-motor demo build |
| `AMBAR_FEATURE_ACTUATOR` | presentation switch | Master physical-motion lock |
| `AMBAR_FEATURE_BENCH_ACTUATOR_COMMANDS` | presentation switch | `HOME`, `RETRACT`, and `BENCH_MOVE` |
| `AMBAR_FEATURE_WATCHDOG` | 0 | Independent hardware watchdog |
| `AMBAR_FEATURE_MAGNETOMETER_TELEMETRY` | 1 | LIS2MDL startup and raw magnetometer fields |
| `AMBAR_FEATURE_VERBOSE_STATUS_TEXT` | 1 | Optional human-readable text tags |
| `AMBAR_FEATURE_AUTO_FLIGHT_PHASES` | 1 | Automatic pad/boost/coast/recovery transitions |

For a stationary real-sensor showcase, set the presentation-motion and
watchdog switches to `0`, and set `AMBAR_FEATURE_AUTO_FLIGHT_PHASES` to `0`.
For the guarded USB-to-motor presentation, leave automatic phases and the
presentation-motion switch at `1` and use the double-opt-in host procedure in
`tools/usb_protocol/OPENROCKET_SIMULATION.md`.
Disabling flash logging does not disable the W25Q64-backed saved configuration.

The enabled-feature word is included in telemetry, so the ground station can
distinguish an intentionally disabled subsystem from a hardware failure. A full
`ERASE_LOG` command is rejected while the watchdog feature is enabled because a
one-megabyte erase can exceed the watchdog interval.

## How To Open It

1. Open STM32CubeIDE.
2. Import this folder as an existing STM32CubeIDE project.
3. Read `CODE_GUIDE.md` for the architecture map and recommended source order.
4. Build the Debug configuration.
5. Confirm the size remains near the 2026-07-15 checked builds. The Debug output
   image is `Debug/stm32_airbrake_pcb.elf`:

```text
Debug:   text 132760, data 744, bss 20088, total 153592
Release: text  78136, data 728, bss 20040, total  98904
```

The BSS total includes the deliberately reserved 8 KiB main stack, USBX's 4 KiB
pool, and bounded USB queues. The earlier 1 KiB stack was too small for the
Debug EKF/radio call chain and caused an observed Cortex-M33 stack-overflow
HardFault.

## Direct STM32 USB and GUI Protocol

The shared wire definition is `Core/Inc/rocket_protocol.h`; firmware framing
and USB transport are in `Core/Src/rocket_protocol.c` and
`Core/Src/ambar_usb.c`. USB uses standalone USBX CDC ACM with HSI48 clock
recovery, explicit endpoint PMA allocation, attach/detach sensing, bounded
queues, and nonblocking main-loop servicing.

CDC carries binary data only:

```text
COBS(9-byte header || payload || CRC-16/CCITT-FALSE little-endian) || 0x00
```

The protocol has telemetry, event, command, ACK, heartbeat, simulation, and
actuator-status packet types. Duplicate USB command sequences resend their
cached ACK only when the full command payload also matches; conflicting reuse
is rejected. An ESTOP packet has a priority receive slot. Config changes are
rejected while armed and validated on a candidate copy before they become
active. Blocking flash save/erase commands are disabled on USB by default so
they cannot starve the standalone USB service; radio/bench maintenance remains
available while disarmed.

The GUI-ready Python codec, COM-port probe, exact test steps, and guarded
OpenRocket replay are in `tools/usb_protocol/`. The replay process uses
`time.perf_counter()` for its absolute schedule, can write a run bundle with a
manifest/packet transcript/PASS-FAIL verdict, and can mirror decoded newline
JSON over localhost UDP while retaining exclusive ownership of COM. Start with
`tools/usb_protocol/OPENROCKET_SIMULATION.md`. Direct USB was exercised on the
PCB as COM6; Windows may assign a different number after a reconnect, so
enumerate VID:PID `0483:5740` before each rehearsal.

## Working Arduino Ground Station

The paired LILYGO T3-S3 SX1280PA sketch and exact Arduino IDE/CLI setup are in:

```text
tools/arduino_ground_station/AMBAR_Ground_Station/
```

It uses 2445 MHz, SF7, 203.125 kHz bandwidth, CR 4/5, private sync, explicit
header, preamble 12, and CRC. Its serial console sends exact commands such as
`tx PING` and fully decodes telemetry tags `0x60` through `0x65`. The connected
STM32 and LILYGO were both programmed and verified bidirectionally on 2026-07-14.

The receiver defaults to `view both`: the original detailed tag log is printed
first, followed by a clearly delimited plain-English packet summary. Enter
`view summary` for a presentation-friendly display or `view detail` for only
the original decoder output.

## Flash Log Layout

Log records use aligned 128-byte storage slots. Two slots fit in each W25Q64
page and 32 fit in each sector, so a record cannot cross a page or sector
boundary. Existing legacy records are not overwritten in place; the scanner
advances to a safe aligned sector. Use `ERASE_LOG` before a fresh logging test
when old bench data is no longer needed and the watchdog feature is off.

## Host Replay Check

The lightweight controller math replay is here:

```text
HostTests/bench_replay_tests.py
```

The latest local check produced:

```text
ballistic_apogee_m=1034.20
drag_apogee_m=690.38
PASS
```

The presentation HIL input is generated from the supplied OpenRocket export by
`tools/usb_protocol/replay_openrocket.py`. Its default action remains a no-serial
dry run; physical motion requires both `--allow-actuator-motion` and
`--home-at-current-position`, plus the matching feature/status bits reported by
the board. Add `--run-bundle DIR` for durable evidence and `--gui-udp-port PORT`
to mirror the same decoded run to a presentation GUI without sharing COM. The
real C flight stack was replayed against the normalized 847-row profile: it
progressed through PAD, BOOST, COAST, AIRBRAKE_ACTIVE, and RECOVERY and produced
a maximum deployment request of 1.000.

The 2026-07-15 software verification set also passed:

- 34 Python protocol/replay/checkout tests;
- 8 production-actuator C tests, including recovery, timeout, and fault paths;
- the C protocol golden-vector executable;
- Debug and Release STM32 builds with `-Wall` and no warnings; and
- the LILYGO Arduino ground-station build for the T3-S3 SX1280PA profile.

These checks prove software paths and internal TMC ramp-state handling. They do
not replace loaded-mechanism, thermal, end-stop, or encoder validation.

## What Is Still Missing Before Flight

- Physical actuator travel and direction measurements.
- Verified current limits and motor thermal behavior.
- Homing/limit behavior on the real mechanism.
- Pressure venting and airbrake force characterization.
- Hardware-in-the-loop replay using real sensor logs.
- Tuned EKF, drag, and deployment constants from measured data.
- RF range, packet-loss, and command-retry validation away from the bench.
