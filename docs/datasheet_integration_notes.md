# Datasheet Integration Notes

Source datasheets reviewed on 2026-06-01:

- `stm32h562rg.pdf`
- `lsm6dsv32x.pdf`
- `bst-bmp388-ds001.pdf`
- `lis2mdl.pdf`
- `DS_SX1280-1_V3.3.pdf`
- `W25Q64JV RevN 04282026 Plus.pdf`
- `tmc5240.pdf`
- `MPM3606AGQV.pdf`

Firmware-facing constants from these datasheets are collected in
`include/ambar_device_constants.hpp`. This keeps the hardware facts in one
place while the actual sensor and actuator drivers are still being built.

## Board-Strap Assumptions Verified In KiCad

| Device | Strap / Net | Result For Firmware |
| --- | --- | --- |
| BMP388 | `SDO` pad is tied to `GND`; `CSB` is tied to `+3V3` | I2C address `0x76` |
| LSM6DSV32X | `SDO/SA0` pad is tied to `GND` | I2C address `0x6A` |
| LIS2MDL | Fixed I2C slave address from datasheet | I2C address `0x1E` |

## Device Facts Now Captured

### STM32H562RGT6

- The selected `RGT6` part is an LQFP64 STM32H562 with 1 MB flash.
- The family provides 640 KB SRAM, multiple I2C/SPI peripherals, USB FS, ADC,
  timers, and the debug/SWD interfaces used by the V3 schematic.

### BMP388 Barometer

- Expected chip ID: register `0x00` returns `0x50`.
- Board address: `0x76`.
- Pressure data starts at register `0x04`; temperature data starts at `0x07`.
- Driver setup will need `PWR_CTRL` (`0x1B`), `OSR` (`0x1C`), `ODR` (`0x1D`),
  and `CONFIG` (`0x1F`) for measurement enable, oversampling, output rate, and
  IIR filtering.
- Normal-mode ODR options useful for flight logging include 200 Hz, 100 Hz, and
  50 Hz. Final rate should be chosen after barometer noise and venting tests.

### LSM6DSV32X IMU

- Expected `WHO_AM_I`: register `0x0F` returns `0x70`.
- Board address: `0x6A`.
- Primary gyro output starts at `0x22`; primary accelerometer output starts at
  `0x28`.
- At the widest useful launch range, the accelerometer sensitivity is
  `0.976 mg/LSB` for +/-32 g.
- At +/-2000 dps, gyro sensitivity is `70 mdps/LSB`.
- Firmware still needs the PCB/body-axis orientation before the EKF can consume
  vertical acceleration operationally.

### LIS2MDL Magnetometer

- Expected `WHO_AM_I`: register `0x4F` returns `0x40`.
- Board address: `0x1E`.
- The datasheet startup example writes `CFG_REG_A = 0x80` for temperature
  compensation, 10 Hz, high-resolution, continuous mode, and `CFG_REG_C = 0x01`
  to enable the data-ready interrupt.
- The current airbrake EKF does not need heading, so initial use should be board
  health, orientation checks, and later telemetry/attitude work.

### SX1280 Radio

- SPI mode is CPOL 0 / CPHA 0.
- The radio uses a 52 MHz crystal and a 2.4 GHz to 2.5 GHz synthesizer range.
- The RF frequency command uses a step of `52e6 / 2^18`, about `198.36 Hz`.
- Firmware must wait for `BUSY` low before issuing commands and after commands
  that make the radio process or change modes.
- Captured opcodes include status, register/buffer access, standby, FS, TX/RX,
  packet setup, IRQ setup, RF frequency, and regulator mode.
- The V3 schematic includes `MCU_TCXO`; confirm whether this actually controls
  radio TCXO power or is only a spare/oscillator-related net before using it in
  the SX1280 driver.

### W25Q64JVSSIQ Flash

- JEDEC read command: `0x9F`.
- Expected manufacturer ID: `0xEF`.
- Expected JEDEC device ID for `W25Q64JVSSIQ`: `0x4017`.
- Capacity is 64 Mbit / 8 MB, organized as 256-byte pages, 4 KB sectors, and
  64 KB blocks.
- Standard SPI modes 0 and 3 are supported.
- Flight logging should use page-aligned writes, erase sectors before reuse, and
  keep a power-loss-tolerant record header or ring-buffer scheme.

### TMC5240 Motor Driver

- SPI mode is mode 3, MSB first.
- Each SPI transaction is 40 bits: one address byte plus four data bytes.
- Maximum SPI frequency is 10 MHz.
- The address MSB selects write access; read access leaves the MSB clear.
- Useful early bring-up registers are `GSTAT` (`0x01`), `IOIN` (`0x04`),
  `IHOLD_IRUN` (`0x10`), motion-position registers, and `DRV_STATUS` (`0x6F`).
- `IOIN.VERSION` is expected to read `0x40` for the first compatible IC version.
- Motor current, homing, travel, and fail-safe behavior still need the actual
  airbrake mechanism data.

### MPM3606A 3.3 V Regulator

- Input operating range is 4.5 V to 21 V.
- V3 uses the 3.3 V / 0.6 A application with 75 kOhm / 24 kOhm feedback
  resistors.
- Firmware cannot configure this part, but brownout and power-good behavior
  should be considered when defining boot, logging, and actuator safe states.

## What This Does Not Finish Yet

The constants make driver work less error-prone, but operational flight firmware
still needs:

1. Real BMP388, LSM6DSV32X, W25Q64, SX1280, and TMC5240 drivers.
2. IMU axis orientation and calibration values from the final mounted PCB.
3. Airbrake actuator travel, homing sequence, current limit, and emergency
   retract behavior.
4. Bench tests that read every device ID before any flight-control logic runs.
5. A hardware-in-the-loop profile using real sensor timing and motor motion.
