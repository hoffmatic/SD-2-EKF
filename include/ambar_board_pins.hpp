#pragma once

#include <cstdint>

namespace ambar::board {

// Small pin-description types keep the hardware map independent from any one
// STM32 library. The final firmware can translate these into HAL/LL GPIO names.
enum class GpioPort : std::uint8_t {
    A,
    B,
    C,
    D,
    H
};

// A single physical MCU pin, for example PB8 or PA5.
struct GpioPin {
    GpioPort port;
    std::uint8_t pin;
};

// I2C sensors in this design each have clock, data, and a data-ready/interrupt
// line. Keeping interrupt with the bus avoids losing that relationship later.
struct I2cPins {
    GpioPin scl;
    GpioPin sda;
    GpioPin interrupt;
};

// Generic 4-wire SPI wiring used by radio, flash, and motor driver.
struct SpiPins {
    GpioPin sck;
    GpioPin miso;
    GpioPin mosi;
    GpioPin chipSelect;
};

// SX1280 needs SPI plus explicit reset, DIO1 interrupt, and BUSY handshake.
struct Sx1280Pins {
    SpiPins spi;
    GpioPin reset;
    GpioPin dio1;
    GpioPin busy;
};

// TMC5240 is controlled over SPI, but enable/sleep/diagnostic pins are just as
// important for safe actuator bring-up and fault handling.
struct Tmc5240Pins {
    SpiPins spi;
    GpioPin driverEnableN;
    GpioPin sleepN;
    GpioPin diag0;
    GpioPin diag1;
};

// Helpers make the pin table below readable and constexpr-friendly.
inline constexpr GpioPin pinA(std::uint8_t pin) {
    return {GpioPort::A, pin};
}

inline constexpr GpioPin pinB(std::uint8_t pin) {
    return {GpioPort::B, pin};
}

inline constexpr GpioPin pinC(std::uint8_t pin) {
    return {GpioPort::C, pin};
}

inline constexpr GpioPin pinD(std::uint8_t pin) {
    return {GpioPort::D, pin};
}

inline constexpr GpioPin pinH(std::uint8_t pin) {
    return {GpioPort::H, pin};
}

// LSM6DSV32X IMU bus from the June 1 KiCad design. This is the high-rate sensor
// that will feed vertical acceleration after body-axis alignment.
inline constexpr I2cPins kImuPins{
    pinB(8), // IMU_SCL
    pinB(7), // IMU_SDA
    pinB(6)  // IMU_INT
};

// BMP388 barometer bus. This is the slower altitude correction path for the EKF.
inline constexpr I2cPins kBarometerPins{
    pinB(10), // BARO_SCL
    pinB(12), // BARO_SDA
    pinB(14)  // BARO_INT
};

// LIS2MDL magnetometer bus. It is not required for the current vertical EKF but
// is useful for later attitude/alignment checks and telemetry health.
inline constexpr I2cPins kMagnetometerPins{
    pinA(8), // MAGNET_SCL
    pinC(9), // MAGNET_SDA
    pinC(8)  // MAGNET_INT
};

// SX1280 radio used for telemetry/configuration. Firmware must obey BUSY before
// sending commands.
inline constexpr Sx1280Pins kRadioPins{
    {
        pinA(5), // LORA_SCK
        pinA(6), // LORA_MISO
        pinA(7), // LORA_MOSI
        pinA(3)  // LORA_CS
    },
    pinA(2), // LORA_NRESET
    pinC(4), // LORA_DIO1
    pinC(5)  // LORA_BUSY
};

// W25Q64 SPI flash for future flight-log storage.
inline constexpr SpiPins kFlashPins{
    pinC(10), // FLASH_SCK
    pinC(11), // FLASH_MISO
    pinC(12), // FLASH_MOSI
    pinD(2)   // FLASH_CS
};

// TMC5240 actuator driver/controller for the stepper-based airbrake mechanism.
inline constexpr Tmc5240Pins kMotorDriverPins{
    {
        pinB(13), // MOTOR_SCK
        pinC(2),  // MOTOR_MISO
        pinC(1),  // MOTOR_MOSI
        pinB(0)   // MOTOR_CS
    },
    pinB(1),  // MOTOR_DRV_ENN
    pinB(2),  // MOTOR_SLEEPN
    pinC(7),  // MOTOR_DIAG0
    pinB(15)  // MOTOR_DIAG1
};

// User/status LEDs. These are handy for boot stage, sensor checks, arming state,
// and fault display when USB/telemetry is not connected.
inline constexpr GpioPin kLedPins[] = {
    pinA(1), // LED_1
    pinA(0), // LED_2
    pinC(3), // LED_3
    pinC(0), // LED_4
    pinB(5), // LED_5
    pinB(4)  // LED_6
};

// USB/debug pins are listed here so board bring-up code can keep them reserved.
inline constexpr GpioPin kUsbVbusSensePin = pinA(9);
inline constexpr GpioPin kSwdioPin = pinA(13);
inline constexpr GpioPin kSwclkPin = pinA(14);
inline constexpr GpioPin kSwoPin = pinB(3);

// PH0 is named MCU_TCXO in KiCad. Confirm final use before enabling it in the
// radio driver.
inline constexpr GpioPin kTcxoPin = pinH(0);

} // namespace ambar::board
