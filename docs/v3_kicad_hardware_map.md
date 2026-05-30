# V3 KiCad Hardware Map

Source files:

- `airbrake.kicad_sch`
- `airbrake.kicad_pcb`

KiCad schematic title block:

- Title: Airbrake Computer
- Revision: 3
- Date: 2026-05-17
- Company: Project AMBAR

This document is generated from the current V3 KiCad project and supersedes the
older V2 screenshot notes for firmware planning.

## Major Parts In V3

| Ref | Part | Firmware Role |
| --- | --- | --- |
| `U10` | STM32H562RGT6 | Main MCU |
| `U2` | LSM6DSV32XTR | High-rate IMU |
| `U1` | BMP388 | Barometer |
| `U3` | LIS2MDLTR | Magnetometer |
| `U8` | SX1280IMLTRT | 2.4 GHz LoRa radio |
| `U5` | W25Q64JVSSIQ | SPI NOR flash |
| `U6` | TMC5240ATJ+T | Stepper motor driver/controller |
| `U9` | IRLML6344TRPBF | Motor over-voltage / power switching support |

## MCU Pin Map

### IMU

| STM32 Pin | Net | Connected Part |
| --- | --- | --- |
| `PB8` | `/IMU_SCL` | LSM6DSV32XTR `SCL` |
| `PB7` | `/IMU_SDA` | LSM6DSV32XTR `SDA` |
| `PB6` | `/IMU_INT` | LSM6DSV32XTR `INT1` |

The IMU is configured as an I2C device. Firmware still needs the physical board
orientation so raw accelerometer data can be mapped into rocket vertical
acceleration.

### Barometer

| STM32 Pin | Net | Connected Part |
| --- | --- | --- |
| `PB10` | `/BARO_SCL` | BMP388 `SCK` in I2C mode |
| `PB12` | `/BARO_SDA` | BMP388 `SDI` in I2C mode |
| `PB14` | `/BARO_INT` | BMP388 `INT` |

The current V3 design uses BMP388, not the BOM-listed LPS22HH. The firmware
barometer driver should target BMP388 unless the schematic changes again.

### Magnetometer

| STM32 Pin | Net | Connected Part |
| --- | --- | --- |
| `PA8` | `/MAGNET_SCL` | LIS2MDLTR `SCL/SPC` |
| `PC9` | `/MAGNET_SDA` | LIS2MDLTR `SDA/SDI/SDO` |
| `PC8` | `/MAGNET_INT` | LIS2MDLTR `INT/DRDY/SDO` |

The current EKF does not require magnetometer input, but this interface can feed
a later attitude/alignment layer or telemetry health checks.

### Radio

| STM32 Pin | Net | Connected Part |
| --- | --- | --- |
| `PA5` | `/LORA_SCK` | SX1280 `SCK_RTSN` |
| `PA6` | `/LORA_MISO` | SX1280 `MISO_TX` |
| `PA7` | `/LORA_MOSI` | SX1280 `MOSI_RX` |
| `PA3` | `/LORA_CS` | SX1280 `NSS_CTS` |
| `PA2` | `/LORA_NRESET` | SX1280 `NRESET` |
| `PC4` | `/LORA_DIO1` | SX1280 `DIO1` |
| `PC5` | `/LORA_BUSY` | SX1280 `BUSY` |

V3 still uses SX1280. Confirm the ground station is also SX1280-compatible; an
SX1262 ground station will not talk to this 2.4 GHz radio.

### NOR Flash

| STM32 Pin | Net | Connected Part |
| --- | --- | --- |
| `PC10` | `/FLASH_SCK` | W25Q64 `CLK` |
| `PC11` | `/FLASH_MISO` | W25Q64 `DO(IO1)` |
| `PC12` | `/FLASH_MOSI` | W25Q64 `DI(IO0)` |
| `PD2` | `/FLASH_CS` | W25Q64 `/CS` |

The current V3 design uses W25Q64, not the BOM-listed AT25DF321A.

### Stepper Motor Driver

| STM32 Pin | Net | Connected Part |
| --- | --- | --- |
| `PB13` | `/MOTOR_SCK` | TMC5240 `SCK/AD1` |
| `PC2` | `/MOTOR_MISO` | TMC5240 `SDO/NAO` |
| `PC1` | `/MOTOR_MOSI` | TMC5240 `SDI/AD0` |
| `PB0` | `/MOTOR_CS` | TMC5240 `CSN/AD2` |
| `PB1` | `/MOTOR_DRV_ENN` | TMC5240 `DRV_ENN` |
| `PB2` | `/MOTOR_SLEEPN` | TMC5240 `SLEEPN` |
| `PB15` | `/MOTOR_DIAG1` | TMC5240 `DIAG1/SW` |
| `PC7` | `/MOTOR_DIAG0` | TMC5240 `DIAG0` |

V3 uses a TMC5240 stepper driver/controller over SPI. That means the actuator
firmware should be register-driven, not a simple GPIO step/dir driver.

Relevant TMC5240 nets:

| Net | Purpose |
| --- | --- |
| `/MOTOR_1A`, `/MOTOR_1B`, `/MOTOR_2A`, `/MOTOR_2B` | Stepper phase outputs |
| `/MOTOR_IREF` | Motor current reference |
| `/MOTOR_DIAG0`, `/MOTOR_DIAG1` | Diagnostics / switch input |
| `/MOTOR_DRV_ENN` | Driver enable |
| `/MOTOR_SLEEPN` | Sleep control |
| `VBUS` | Motor supply |

### USB / Debug / Boot

| STM32 Pin | Net |
| --- | --- |
| `PA11` | `/MCU_USB_D-` |
| `PA12` | `/MCU_USB_D+` |
| `PA9` | `/MCU_USB_VBUS_SENSE` |
| `PA13` | `/MCU_SWDIO` |
| `PA14` | `/MCU_SWCLK` |
| `PB3` | `/MCU_SWO` |
| `NRST` | `/MCU_NRESET` |
| `BOOT0` | `/MCU_BOOT0` |

### LEDs / Misc

| STM32 Pin | Net |
| --- | --- |
| `PC13` | `/LED_1` |
| `PC14` | `/LED_2` |
| `PC15` | `/LED_3` |
| `PH1` | `/LED_4` |
| `PA0` | `/LED_5` |
| `PA1` | `/LED_6` |
| `PH0` | `/MCU_TCXO` |

## V3 Changes From Earlier BOM / V2 Assumptions

- Barometer is BMP388 in V3, not LPS22HH.
- Flash is W25Q64 in V3, not AT25DF321A.
- Motor driver is TMC5240 in V3, not DRV8434A.
- MCU package shown in KiCad is STM32H562RGT6.
- Radio remains SX1280, so the ground-station radio compatibility question is
  still open.

## Firmware Work Unlocked By V3

1. Add a board pin map for STM32H562RGT6.
2. Add BMP388 pressure/temperature driver and pad-zero altitude conversion.
3. Add LSM6DSV32XTR driver with data-ready interrupt and saturation checks.
4. Add W25Q64 flight-log ring buffer.
5. Add SX1280 telemetry/config driver after the ground station radio is settled.
6. Add TMC5240 actuator driver that maps `deployFraction` to a target position.

## Info Still Needed

1. IMU physical orientation on the PCB inside the rocket.
2. Lead screw pitch and actual airbrake deployment travel.
3. Stepper motor electrical specs and selected TMC5240 current limit.
4. Whether `/MOTOR_DIAG0` or `/MOTOR_DIAG1` is wired to a limit switch, stall
   signal, or both.
5. Preflight actuator homing sequence.
6. Confirmed ground station radio hardware.
