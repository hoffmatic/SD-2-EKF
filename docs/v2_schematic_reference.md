# V2 Schematic Reference

Status: reference only. The current V3 schematic may differ.

These notes were read from the provided V2 schematic screenshots and should be
used as a starting checklist for the final V3 pin map, not as final firmware
truth.

## High-Level Architecture

- MCU: STM32H562RGT6 / STM32H562 family.
- Main logic rail: +3V3 from a VBUS-fed MPM3606A regulator.
- Sensors:
  - IMU: LSM6DSV32XTR
  - Magnetometer: LIS2MDLTR
  - Barometer on V2 sheet appears to be a BMP3xx-family part, while the BOM
    lists LPS22HHTR for the current flight computer.
- Radio: SX1280IMLTRT with 52 MHz crystal and RF matching network to coax.
- Flash: W25Q64JVSSIQ on V2, while the BOM lists AT25DF321A-SH-T.
- Motor driver: DRV8434A family driving two motor phase outputs.
- USB: USB-C micro connector through USBLC6-2P6 ESD protection.

## MCU Signal Names Visible in V2

### Sensor Interfaces

| Signal | V2 Purpose |
| --- | --- |
| `IMU_SCL` | IMU I2C clock |
| `IMU_SDA` | IMU I2C data |
| `IMU_INT` | IMU interrupt/data-ready |
| `BARO_SCL` | Barometer I2C clock |
| `BARO_SDA` | Barometer I2C data |
| `BARO_INT` | Barometer interrupt |
| `MAGNET_SCL` | Magnetometer I2C clock |
| `MAGNET_SDA` | Magnetometer I2C data |
| `MAGNET_INT` | Magnetometer interrupt/data-ready |

V2 shows the IMU, barometer, and magnetometer configured for I2C with pull-ups.
For firmware, the important V3 confirmation is whether these are three separate
I2C buses or shared buses with different pull-up domains and interrupt pins.

### Flash SPI

| Signal | V2 Purpose |
| --- | --- |
| `FLASH_SCK` | SPI clock |
| `FLASH_MOSI` | SPI controller output |
| `FLASH_MISO` | SPI controller input |
| `FLASH_CS` | Flash chip select |

V2 ties flash `/HOLD`, `/RESET`, and `/WP` high, so the firmware only needs the
standard SPI pins plus chip select unless V3 changes that.

### Radio SPI / Control

| Signal | V2 Purpose |
| --- | --- |
| `LORA_MOSI` | SX1280 SPI MOSI |
| `LORA_MISO` | SX1280 SPI MISO |
| `LORA_SCK` | SX1280 SPI clock |
| `LORA_CS` | SX1280 chip select |
| `LORA_NRESET` | SX1280 reset |
| `LORA_DIO1` | SX1280 interrupt |
| `LORA_BUSY` | SX1280 busy status |
| `LORA_RFIO` | RF path to matching network / coax |

V2 uses an SX1280 radio, which matches the flight-computer BOM. The ground
station still needs to be checked for SX1280 compatibility.

### Motor Driver

| Signal | V2 Purpose |
| --- | --- |
| `MOTOR_DIR` | Stepper direction |
| `MOTOR_STEP` | Step pulse |
| `MOTOR_ENABLE` | Driver enable |
| `MOTOR_NSLEEP` | Driver sleep control |
| `MOTOR_M0` | Microstep/mode input |
| `MOTOR_M1` | Microstep/mode input |
| `MOTOR_STL_MODE` | Stall-detection mode/config |
| `MOTOR_TRQ_CNT_STLTH` | Torque count / stall threshold input |
| `MOTOR_NFAULT` | Driver fault output |
| `MOTOR_STL_REP` | Stall report output |
| `MCU_VREF` | Driver current/reference control |

This confirms SD-2-EKF should treat airbrake deployment as a stepper-style
position-control problem, not a simple servo PWM problem.

## Sensor Configuration Notes

### LSM6DSV32XTR IMU

- V2 shows I2C mode.
- `SDO/SA0` appears tied low, setting the I2C address LSB.
- `CS` is pulled high for I2C mode.
- `INT1` is routed as `IMU_INT`; `INT2` appears pulled high.

Firmware implication: configure a data-ready interrupt on `IMU_INT`, read accel
and gyro at the chosen ODR, reject saturation, then convert body-frame accel into
vertical translational acceleration before calling `updateImu(...)`.

### Barometer

- V2 sheet labels the part as a BMP3xx-family barometer.
- BOM lists `LPS22HHTR`, so the current V3 part may have changed.
- V2 shows I2C mode with `BARO_SCL`, `BARO_SDA`, and `BARO_INT`.

Firmware implication: do not hardcode a barometer driver until V3 confirms
whether the part is BMP3xx or LPS22HH.

### LIS2MDLTR Magnetometer

- V2 shows I2C mode with `MAGNET_SCL`, `MAGNET_SDA`, and `MAGNET_INT`.
- This is useful for alignment/attitude work but is not required by the current
  4-state vertical EKF.

## V3 Questions to Resolve

1. Does V3 still use STM32H562RGT6, or the BOM's STM32H562VGT6 package?
2. Did the barometer change from the V2 BMP3xx-family part to BOM `LPS22HHTR`?
3. Did flash change from V2 `W25Q64JVSSIQ` to BOM `AT25DF321A-SH-T`?
4. Are the IMU, barometer, and magnetometer on shared or separate I2C buses?
5. Which exact STM32 pins map to every net above in V3?
6. Which physical IMU axis points toward the rocket nose?
7. Does the actuator have limit switches, encoder feedback, current/stall-only
   feedback, or open-loop step counting?
8. Is the final radio link SX1280 on both vehicle and ground station?
9. What are the selected motor current limit, microstep setting, and lead-screw
   steps per millimeter?
10. What is the arming/preflight sequence for actuator homing and barometer zero?

## Firmware Work Enabled by This V2 Reference

The next useful code layer is a platform boundary with these modules:

- STM32 scheduler: fixed-rate IMU propagation and asynchronous sensor updates.
- I2C sensor drivers: IMU, barometer, magnetometer.
- SPI drivers: radio and NOR flash.
- DRV8434A actuator driver: step generation, homing, limits, fault handling.
- Telemetry/logging: estimate, phase, command, health, and raw sensor records.

Keep the estimator independent from these drivers so it can continue to compile
and run on a desktop simulation before being built for STM32.
