# AMBAR BOM Hardware Notes

Source workbook: `HARGROVE-AMBAR-BOM-04262026.xlsx`

These notes translate the BOM into firmware-facing interfaces for SD-2-EKF.

## Flight Computer Parts

| Function | BOM Part | Firmware Impact |
| --- | --- | --- |
| MCU | STM32H562VGT6 | Target embedded port should use STM32 HAL/LL or a compatible RTOS/platform layer. |
| Regulator | TPSM84209RKHR | Servo/motor current should still be isolated from sensor/MCU supply assumptions. |
| Radio | SX1280IMLTRT | Needs SPI radio driver and packet format for telemetry/configuration. |
| Motor driver | DRV8434APWPR | Airbrake command should map to step/dir or driver-specific control once PCB pins are known. |
| Data logging | AT25DF321A-SH-T | Add NOR flash ring buffer for flight logs and fault recovery. |
| IMU | LSM6DSV32XTR | Primary high-rate acceleration source for `updateImu(...)`. |
| Temperature | STTS22HTR | Use for sensor health, bias compensation, and telemetry. |
| Barometer | LPS22HHTR | Primary altitude source for `updateBarometer(...)`. |
| Magnetometer | LIS2MDLTR | Later attitude/alignment layer; not required for the current vertical EKF. |

## Mechanical / Actuation Parts

- Airframe: 3-inch class vehicle.
- Motor: Aerotech J420R-14 listed in inventory checkout.
- Airbrake actuator concept: lead screw / threaded-rod deployment.
- Related parts include 1/4-inch threaded rod, TR8x8 lead screw, lead-screw nuts,
  collars, bearings, and DRV8434A motor driver.

This points the firmware toward a bounded actuator interface rather than direct
servo PWM. The EKF/controller should keep output as `deployFraction`; a separate
actuator layer should convert that fraction into motor steps, position limits,
homing behavior, and fault checks.

## Interface Assumptions to Confirm

These items are not fully specified in the BOM and should be settled before the
firmware target is locked:

1. PCB pin map for STM32H562VGT6 peripherals.
2. IMU, barometer, magnetometer, flash, and radio buses: SPI vs I2C, chip selects,
   interrupt lines, and data-ready lines.
3. IMU board orientation: which sensor axis points toward the rocket nose.
4. Actuator position feedback: limit switches, encoder, stall detection, or
   open-loop step counting.
5. Motor-driver mode for DRV8434A: step/dir, microstep settings, current limit,
   sleep/fault pins, and reset behavior.
6. Radio band and matching ground station.
7. Telemetry packet rate and contents.
8. Log record format and flash wear strategy.
9. Brownout/watchdog recovery behavior.
10. Preflight calibration flow for barometer zero, IMU bias, and actuator homing.

## Radio Compatibility Risk

The flight-computer BOM lists `SX1280IMLTRT`, while the purchases sheet lists a
Waveshare USB-to-LoRa ground station based on `SX1262`.

This needs review before ordering or PCB lock:

- SX1280 is a 2.4 GHz LoRa transceiver.
- SX1262 is a sub-GHz LoRa transceiver commonly used for 868/915 MHz links.
- A 2.4 GHz SX1280 radio will not communicate with a sub-GHz SX1262 radio.

The AMBAR M3 report also mentions operation in the 915 MHz ISM band. If 915 MHz
is the requirement, replace the flight-computer radio with a compatible sub-GHz
part/module. If SX1280 is intentional, replace the ground station with a 2.4 GHz
SX1280-compatible device and revisit range/regulatory assumptions.

## Suggested Firmware Modules

- `board_stm32h562`: clocks, timers, GPIO, SPI/I2C, DMA, interrupts.
- `sensor_lsm6dsv32x`: IMU setup, data-ready reads, axis mapping, saturation flags.
- `sensor_lps22hh`: pressure/temperature reads, pad-zero altitude conversion.
- `sensor_lis2mdl`: magnetometer reads for future alignment/attitude work.
- `storage_at25df321a`: flight-log ring buffer.
- `radio_sx1280_or_revised`: telemetry/config packets once radio choice is resolved.
- `actuator_drv8434a`: motor enable, step generation, current/fault monitoring.
- `flight_scheduler`: fixed-rate IMU propagation plus asynchronous sensor updates.

## Next Code Step

Add a `platform/` directory with pure virtual or C-style interfaces for the
sensors, actuator, logger, and radio. Keep SD-2-EKF independent from STM32 HAL so
the estimator can be tested on a laptop and then reused on the flight computer.
