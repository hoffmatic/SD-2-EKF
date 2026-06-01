#pragma once

#include <cstdint>

namespace ambar::board {

enum class GpioPort : std::uint8_t {
    A,
    B,
    C,
    D,
    H
};

struct GpioPin {
    GpioPort port;
    std::uint8_t pin;
};

struct I2cPins {
    GpioPin scl;
    GpioPin sda;
    GpioPin interrupt;
};

struct SpiPins {
    GpioPin sck;
    GpioPin miso;
    GpioPin mosi;
    GpioPin chipSelect;
};

struct Sx1280Pins {
    SpiPins spi;
    GpioPin reset;
    GpioPin dio1;
    GpioPin busy;
};

struct Tmc5240Pins {
    SpiPins spi;
    GpioPin driverEnableN;
    GpioPin sleepN;
    GpioPin diag0;
    GpioPin diag1;
};

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

inline constexpr I2cPins kImuPins{
    pinB(8), // IMU_SCL
    pinB(7), // IMU_SDA
    pinB(6)  // IMU_INT
};

inline constexpr I2cPins kBarometerPins{
    pinB(10), // BARO_SCL
    pinB(12), // BARO_SDA
    pinB(14)  // BARO_INT
};

inline constexpr I2cPins kMagnetometerPins{
    pinA(8), // MAGNET_SCL
    pinC(9), // MAGNET_SDA
    pinC(8)  // MAGNET_INT
};

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

inline constexpr SpiPins kFlashPins{
    pinC(10), // FLASH_SCK
    pinC(11), // FLASH_MISO
    pinC(12), // FLASH_MOSI
    pinD(2)   // FLASH_CS
};

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

inline constexpr GpioPin kLedPins[] = {
    pinA(1), // LED_1
    pinA(0), // LED_2
    pinC(3), // LED_3
    pinC(0), // LED_4
    pinB(5), // LED_5
    pinB(4)  // LED_6
};

inline constexpr GpioPin kUsbVbusSensePin = pinA(9);
inline constexpr GpioPin kSwdioPin = pinA(13);
inline constexpr GpioPin kSwclkPin = pinA(14);
inline constexpr GpioPin kSwoPin = pinB(3);
inline constexpr GpioPin kTcxoPin = pinH(0);

} // namespace ambar::board
