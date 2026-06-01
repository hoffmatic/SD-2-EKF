#pragma once

#include <cstddef>
#include <cstdint>

namespace ambar::devices {

// Device constants are separated from drivers so bring-up code can verify IDs,
// addresses, and bus modes before the full hardware abstraction layer exists.

namespace stm32h562rgt6 {
// Current MCU from the June 1 KiCad design. These are useful for linker-script
// checks and rough memory budgeting.
inline constexpr std::uint32_t kFlashBytes = 1024UL * 1024UL;
inline constexpr std::uint32_t kSramBytes = 640UL * 1024UL;
inline constexpr std::uint8_t kPackagePins = 64;
} // namespace stm32h562rgt6

namespace bmp388 {
// BMP388 is strapped for I2C on this board. Reading CHIP_ID should be the first
// barometer bring-up test before trusting pressure data.
inline constexpr std::uint8_t kI2cAddress = 0x76; // SDO tied to GND on June 1 PCB.
inline constexpr std::uint8_t kChipIdRegister = 0x00;
inline constexpr std::uint8_t kExpectedChipId = 0x50;

// Data and setup registers needed by a minimal pressure/temperature driver.
inline constexpr std::uint8_t kPressureDataRegister = 0x04;
inline constexpr std::uint8_t kTemperatureDataRegister = 0x07;
inline constexpr std::uint8_t kPowerControlRegister = 0x1B;
inline constexpr std::uint8_t kOversamplingRegister = 0x1C;
inline constexpr std::uint8_t kOutputDataRateRegister = 0x1D;
inline constexpr std::uint8_t kConfigRegister = 0x1F;

// Early candidate output rates for flight logging and filtering experiments.
inline constexpr std::uint8_t kOdr200Hz = 0x00;
inline constexpr std::uint8_t kOdr100Hz = 0x01;
inline constexpr std::uint8_t kOdr50Hz = 0x02;
} // namespace bmp388

namespace lsm6dsv32x {
// IMU address is selected by the SA0 strap. WHO_AM_I verifies both the part and
// the bus wiring before the estimator consumes acceleration.
inline constexpr std::uint8_t kI2cAddress = 0x6A; // SDO/SA0 tied to GND on June 1 PCB.
inline constexpr std::uint8_t kWhoAmIRegister = 0x0F;
inline constexpr std::uint8_t kExpectedWhoAmI = 0x70;

// Primary-interface control and output registers for the first IMU driver.
inline constexpr std::uint8_t kInt1ControlRegister = 0x0D;
inline constexpr std::uint8_t kCtrl1Register = 0x10;
inline constexpr std::uint8_t kCtrl2Register = 0x11;
inline constexpr std::uint8_t kCtrl3Register = 0x12;
inline constexpr std::uint8_t kGyroOutputBaseRegister = 0x22;
inline constexpr std::uint8_t kAccelOutputBaseRegister = 0x28;

// Scale factors convert signed raw sensor counts into physical units.
inline constexpr float kAccelSensitivityMgPerLsbAt32g = 0.976F;
inline constexpr float kGyroSensitivityMdpsPerLsbAt2000Dps = 70.0F;
} // namespace lsm6dsv32x

namespace lis2mdl {
// Magnetometer is not part of the first vertical EKF, but its ID/address belong
// here so board health checks can still verify the sensor is present.
inline constexpr std::uint8_t kI2cAddress = 0x1E;
inline constexpr std::uint8_t kWhoAmIRegister = 0x4F;
inline constexpr std::uint8_t kExpectedWhoAmI = 0x40;

// Basic configuration and output registers from the datasheet startup sequence.
inline constexpr std::uint8_t kConfigARegister = 0x60;
inline constexpr std::uint8_t kConfigBRegister = 0x61;
inline constexpr std::uint8_t kConfigCRegister = 0x62;
inline constexpr std::uint8_t kStatusRegister = 0x67;
inline constexpr std::uint8_t kOutputBaseRegister = 0x68;
inline constexpr std::uint8_t kStartupConfigA10HzContinuousWithTempComp = 0x80;
inline constexpr std::uint8_t kStartupConfigCDataReadyInterrupt = 0x01;
} // namespace lis2mdl

namespace w25q64jv {
// Standard SPI flash commands. Keeping these named reduces mistakes in the
// future flight-log ring buffer driver.
inline constexpr std::uint8_t kReadStatusRegister1Command = 0x05;
inline constexpr std::uint8_t kWriteEnableCommand = 0x06;
inline constexpr std::uint8_t kPageProgramCommand = 0x02;
inline constexpr std::uint8_t kReadDataCommand = 0x03;
inline constexpr std::uint8_t kFastReadCommand = 0x0B;
inline constexpr std::uint8_t kSectorErase4kbCommand = 0x20;
inline constexpr std::uint8_t kBlockErase32kbCommand = 0x52;
inline constexpr std::uint8_t kBlockErase64kbCommand = 0xD8;
inline constexpr std::uint8_t kReadJedecIdCommand = 0x9F;

// JEDEC ID should be checked at boot before writing flight logs.
inline constexpr std::uint8_t kExpectedManufacturerId = 0xEF;
inline constexpr std::uint16_t kExpectedJedecDeviceId = 0x4017;
inline constexpr std::uint8_t kSupportedSpiMode0 = 0;
inline constexpr std::uint8_t kSupportedSpiMode3 = 3;

// Geometry drives log record alignment, erase planning, and wear management.
inline constexpr std::size_t kPageSizeBytes = 256;
inline constexpr std::size_t kSectorSizeBytes = 4UL * 1024UL;
inline constexpr std::size_t kBlockSizeBytes = 64UL * 1024UL;
inline constexpr std::size_t kTotalSizeBytes = 8UL * 1024UL * 1024UL;
} // namespace w25q64jv

namespace sx1280 {
// SX1280 is command-based over SPI mode 0. The driver must also use the BUSY pin
// from the board map before and after commands.
inline constexpr std::uint8_t kSpiMode = 0;
inline constexpr std::uint32_t kCrystalFrequencyHz = 52000000UL;
inline constexpr std::uint32_t kMinRfFrequencyHz = 2400000000UL;
inline constexpr std::uint32_t kMaxRfFrequencyHz = 2500000000UL;
inline constexpr float kRfFrequencyStepHz = 52000000.0F / 262144.0F;

// Frequently used opcodes for startup, configuration, transmit/receive, and IRQ
// handling. These are not yet a complete radio driver.
inline constexpr std::uint8_t kGetStatusCommand = 0xC0;
inline constexpr std::uint8_t kWriteRegisterCommand = 0x18;
inline constexpr std::uint8_t kReadRegisterCommand = 0x19;
inline constexpr std::uint8_t kWriteBufferCommand = 0x1A;
inline constexpr std::uint8_t kReadBufferCommand = 0x1B;
inline constexpr std::uint8_t kSetStandbyCommand = 0x80;
inline constexpr std::uint8_t kSetFsCommand = 0xC1;
inline constexpr std::uint8_t kSetTxCommand = 0x83;
inline constexpr std::uint8_t kSetRxCommand = 0x82;
inline constexpr std::uint8_t kSetPacketTypeCommand = 0x8A;
inline constexpr std::uint8_t kSetRfFrequencyCommand = 0x86;
inline constexpr std::uint8_t kSetDioIrqParamsCommand = 0x8D;
inline constexpr std::uint8_t kGetIrqStatusCommand = 0x15;
inline constexpr std::uint8_t kClearIrqStatusCommand = 0x97;
inline constexpr std::uint8_t kSetRegulatorModeCommand = 0x96;
inline constexpr std::uint8_t kRegulatorModeLdo = 0x00;
inline constexpr std::uint8_t kRegulatorModeDcDc = 0x01;
} // namespace sx1280

namespace tmc5240 {
// TMC5240 is register-driven over SPI mode 3. Every transaction is one 40-bit
// datagram: address byte plus four data bytes.
inline constexpr std::uint8_t kSpiMode = 3;
inline constexpr std::uint32_t kMaxSpiFrequencyHz = 10000000UL;
inline constexpr std::uint8_t kSpiFrameBits = 40;
inline constexpr std::uint8_t kReadAddressMask = 0x7F;
inline constexpr std::uint8_t kWriteAddressMask = 0x80;

// Registers needed for first bring-up, fault checks, and position commands.
inline constexpr std::uint8_t kGconfRegister = 0x00;
inline constexpr std::uint8_t kGstatRegister = 0x01;
inline constexpr std::uint8_t kIoinRegister = 0x04;
inline constexpr std::uint8_t kGlobalScalerRegister = 0x0B;
inline constexpr std::uint8_t kIholdIrunRegister = 0x10;
inline constexpr std::uint8_t kXActualRegister = 0x21;
inline constexpr std::uint8_t kVActualRegister = 0x22;
inline constexpr std::uint8_t kAmaxRegister = 0x26;
inline constexpr std::uint8_t kVmaxRegister = 0x27;
inline constexpr std::uint8_t kXTargetRegister = 0x2D;
inline constexpr std::uint8_t kDrvStatusRegister = 0x6F;
inline constexpr std::uint8_t kExpectedIoinVersion = 0x40;
} // namespace tmc5240

namespace mpm3606a {
// Regulator facts are not configurable in firmware, but documenting them here
// helps with power-budget, brownout, and boot-safety decisions.
inline constexpr std::uint16_t kInputVoltageMinMv = 4500;
inline constexpr std::uint16_t kInputVoltageMaxMv = 21000;
inline constexpr std::uint16_t kNominalOutputVoltageMv = 3300;
inline constexpr std::uint16_t kContinuousLoadCurrentMa = 600;
inline constexpr std::uint32_t kFeedbackTopOhms = 75000UL;
inline constexpr std::uint32_t kFeedbackBottomOhms = 24000UL;
} // namespace mpm3606a

} // namespace ambar::devices
