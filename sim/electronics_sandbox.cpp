#include "ambar_board_pins.hpp"
#include "ambar_device_constants.hpp"

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

enum class Status {
    Pass,
    Warn,
    Fail
};

struct CheckResult {
    Status status = Status::Pass;
    std::string name;
    std::string detail;
};

struct BootScenario {
    std::string name;
    std::uint16_t vin_mV = 12000;
    std::uint16_t logicLoad_mA = 260;
    bool bmpPresent = true;
    std::uint8_t bmpAddress = ambar::devices::bmp388::kI2cAddress;
    std::uint8_t bmpChipId = ambar::devices::bmp388::kExpectedChipId;
    bool imuPresent = true;
    std::uint8_t imuAddress = ambar::devices::lsm6dsv32x::kI2cAddress;
    std::uint8_t imuWhoAmI = ambar::devices::lsm6dsv32x::kExpectedWhoAmI;
    bool magPresent = true;
    std::uint8_t magAddress = ambar::devices::lis2mdl::kI2cAddress;
    std::uint8_t magWhoAmI = ambar::devices::lis2mdl::kExpectedWhoAmI;
    bool flashPresent = true;
    std::uint8_t flashSpiMode = ambar::devices::w25q64jv::kSupportedSpiMode0;
    std::uint8_t flashManufacturer = ambar::devices::w25q64jv::kExpectedManufacturerId;
    std::uint16_t flashDevice = ambar::devices::w25q64jv::kExpectedJedecDeviceId;
    bool radioPresent = true;
    std::uint8_t radioSpiMode = ambar::devices::sx1280::kSpiMode;
    bool radioBusyClears = true;
    bool groundStationCompatible = false;
    bool motorPresent = true;
    std::uint8_t motorSpiMode = ambar::devices::tmc5240::kSpiMode;
    std::uint8_t motorVersion = ambar::devices::tmc5240::kExpectedIoinVersion;
    bool motorSupplyPresent = true;
};

char portLetter(ambar::board::GpioPort port) {
    switch (port) {
    case ambar::board::GpioPort::A:
        return 'A';
    case ambar::board::GpioPort::B:
        return 'B';
    case ambar::board::GpioPort::C:
        return 'C';
    case ambar::board::GpioPort::D:
        return 'D';
    case ambar::board::GpioPort::H:
        return 'H';
    }
    return '?';
}

std::string pinName(ambar::board::GpioPin pin) {
    std::ostringstream stream;
    stream << "P" << portLetter(pin.port) << static_cast<int>(pin.pin);
    return stream.str();
}

std::string hex8(std::uint8_t value) {
    std::ostringstream stream;
    stream << "0x" << std::uppercase << std::hex << std::setw(2)
           << std::setfill('0') << static_cast<int>(value);
    return stream.str();
}

std::string hex16(std::uint16_t value) {
    std::ostringstream stream;
    stream << "0x" << std::uppercase << std::hex << std::setw(4)
           << std::setfill('0') << static_cast<int>(value);
    return stream.str();
}

void addCheck(std::vector<CheckResult>& checks,
              Status status,
              const std::string& name,
              const std::string& detail)
{
    checks.push_back({status, name, detail});
}

std::vector<CheckResult> checkPinCollisions() {
    std::vector<std::pair<std::string, ambar::board::GpioPin>> pins{
        {"IMU_SCL", ambar::board::kImuPins.scl},
        {"IMU_SDA", ambar::board::kImuPins.sda},
        {"IMU_INT", ambar::board::kImuPins.interrupt},
        {"BARO_SCL", ambar::board::kBarometerPins.scl},
        {"BARO_SDA", ambar::board::kBarometerPins.sda},
        {"BARO_INT", ambar::board::kBarometerPins.interrupt},
        {"MAGNET_SCL", ambar::board::kMagnetometerPins.scl},
        {"MAGNET_SDA", ambar::board::kMagnetometerPins.sda},
        {"MAGNET_INT", ambar::board::kMagnetometerPins.interrupt},
        {"LORA_SCK", ambar::board::kRadioPins.spi.sck},
        {"LORA_MISO", ambar::board::kRadioPins.spi.miso},
        {"LORA_MOSI", ambar::board::kRadioPins.spi.mosi},
        {"LORA_CS", ambar::board::kRadioPins.spi.chipSelect},
        {"LORA_NRESET", ambar::board::kRadioPins.reset},
        {"LORA_DIO1", ambar::board::kRadioPins.dio1},
        {"LORA_BUSY", ambar::board::kRadioPins.busy},
        {"FLASH_SCK", ambar::board::kFlashPins.sck},
        {"FLASH_MISO", ambar::board::kFlashPins.miso},
        {"FLASH_MOSI", ambar::board::kFlashPins.mosi},
        {"FLASH_CS", ambar::board::kFlashPins.chipSelect},
        {"MOTOR_SCK", ambar::board::kMotorDriverPins.spi.sck},
        {"MOTOR_MISO", ambar::board::kMotorDriverPins.spi.miso},
        {"MOTOR_MOSI", ambar::board::kMotorDriverPins.spi.mosi},
        {"MOTOR_CS", ambar::board::kMotorDriverPins.spi.chipSelect},
        {"MOTOR_DRV_ENN", ambar::board::kMotorDriverPins.driverEnableN},
        {"MOTOR_SLEEPN", ambar::board::kMotorDriverPins.sleepN},
        {"MOTOR_DIAG0", ambar::board::kMotorDriverPins.diag0},
        {"MOTOR_DIAG1", ambar::board::kMotorDriverPins.diag1},
        {"USB_VBUS", ambar::board::kUsbVbusSensePin},
        {"SWDIO", ambar::board::kSwdioPin},
        {"SWCLK", ambar::board::kSwclkPin},
        {"SWO", ambar::board::kSwoPin},
        {"MCU_TCXO", ambar::board::kTcxoPin}
    };

    for (std::size_t index = 0; index < sizeof(ambar::board::kLedPins) / sizeof(ambar::board::kLedPins[0]); ++index) {
        std::ostringstream label;
        label << "LED_" << index + 1U;
        pins.push_back({label.str(), ambar::board::kLedPins[index]});
    }

    std::set<std::string> usedPins;
    std::vector<std::string> duplicates;
    for (const auto& entry : pins) {
        const std::string name = pinName(entry.second);
        if (!usedPins.insert(name).second) {
            duplicates.push_back(name);
        }
    }

    std::vector<CheckResult> checks;
    if (duplicates.empty()) {
        addCheck(checks, Status::Pass, "pin map", "no duplicate GPIO assignments in constants");
    } else {
        std::ostringstream detail;
        for (const std::string& duplicate : duplicates) {
            detail << duplicate << " ";
        }
        addCheck(checks, Status::Fail, "pin map", "duplicate GPIO assignments: " + detail.str());
    }
    return checks;
}

std::vector<CheckResult> runBootChecks(const BootScenario& scenario) {
    std::vector<CheckResult> checks;

    const bool inputOk =
        scenario.vin_mV >= ambar::devices::mpm3606a::kInputVoltageMinMv
     && scenario.vin_mV <= ambar::devices::mpm3606a::kInputVoltageMaxMv;
    const bool loadOk =
        scenario.logicLoad_mA <= ambar::devices::mpm3606a::kContinuousLoadCurrentMa;

    addCheck(
        checks,
        inputOk && loadOk ? Status::Pass : Status::Fail,
        "3V3 regulator",
        "VIN=" + std::to_string(scenario.vin_mV) + "mV, load="
            + std::to_string(scenario.logicLoad_mA) + "mA"
    );

    const bool bmpOk = scenario.bmpPresent
                    && scenario.bmpAddress == ambar::devices::bmp388::kI2cAddress
                    && scenario.bmpChipId == ambar::devices::bmp388::kExpectedChipId;
    addCheck(
        checks,
        bmpOk ? Status::Pass : Status::Fail,
        "BMP388",
        "addr " + hex8(scenario.bmpAddress) + ", id " + hex8(scenario.bmpChipId)
    );

    const bool imuOk = scenario.imuPresent
                    && scenario.imuAddress == ambar::devices::lsm6dsv32x::kI2cAddress
                    && scenario.imuWhoAmI == ambar::devices::lsm6dsv32x::kExpectedWhoAmI;
    addCheck(
        checks,
        imuOk ? Status::Pass : Status::Fail,
        "LSM6DSV32X",
        "addr " + hex8(scenario.imuAddress) + ", whoami " + hex8(scenario.imuWhoAmI)
    );

    const bool magOk = scenario.magPresent
                    && scenario.magAddress == ambar::devices::lis2mdl::kI2cAddress
                    && scenario.magWhoAmI == ambar::devices::lis2mdl::kExpectedWhoAmI;
    addCheck(
        checks,
        magOk ? Status::Pass : Status::Warn,
        "LIS2MDL",
        "addr " + hex8(scenario.magAddress) + ", whoami " + hex8(scenario.magWhoAmI)
    );

    const bool flashModeOk =
        scenario.flashSpiMode == ambar::devices::w25q64jv::kSupportedSpiMode0
     || scenario.flashSpiMode == ambar::devices::w25q64jv::kSupportedSpiMode3;
    const bool flashOk = scenario.flashPresent
                      && flashModeOk
                      && scenario.flashManufacturer == ambar::devices::w25q64jv::kExpectedManufacturerId
                      && scenario.flashDevice == ambar::devices::w25q64jv::kExpectedJedecDeviceId;
    addCheck(
        checks,
        flashOk ? Status::Pass : Status::Fail,
        "W25Q64",
        "mode " + std::to_string(scenario.flashSpiMode)
            + ", JEDEC " + hex8(scenario.flashManufacturer)
            + " " + hex16(scenario.flashDevice)
    );

    const std::size_t recordBytes = 64;
    const std::size_t records =
        ambar::devices::w25q64jv::kTotalSizeBytes / recordBytes;
    addCheck(
        checks,
        records > 100000U ? Status::Pass : Status::Warn,
        "flash log capacity",
        std::to_string(records) + " virtual 64-byte records before wear strategy"
    );

    const bool radioOk = scenario.radioPresent
                      && scenario.radioSpiMode == ambar::devices::sx1280::kSpiMode
                      && scenario.radioBusyClears;
    addCheck(
        checks,
        radioOk ? Status::Pass : Status::Fail,
        "SX1280",
        "mode " + std::to_string(scenario.radioSpiMode)
            + (scenario.radioBusyClears ? ", BUSY clears" : ", BUSY stuck")
    );

    addCheck(
        checks,
        scenario.groundStationCompatible ? Status::Pass : Status::Warn,
        "radio link",
        scenario.groundStationCompatible
            ? "ground station marked SX1280-compatible"
            : "ground station compatibility still open"
    );

    const bool motorOk = scenario.motorPresent
                      && scenario.motorSupplyPresent
                      && scenario.motorSpiMode == ambar::devices::tmc5240::kSpiMode
                      && scenario.motorVersion == ambar::devices::tmc5240::kExpectedIoinVersion;
    addCheck(
        checks,
        motorOk ? Status::Pass : Status::Fail,
        "TMC5240",
        "mode " + std::to_string(scenario.motorSpiMode)
            + ", IOIN.VERSION " + hex8(scenario.motorVersion)
            + (scenario.motorSupplyPresent ? ", VBUS present" : ", VBUS missing")
    );

    const std::vector<CheckResult> pinChecks = checkPinCollisions();
    checks.insert(checks.end(), pinChecks.begin(), pinChecks.end());

    return checks;
}

void printScenario(const BootScenario& scenario) {
    const std::vector<CheckResult> checks = runBootChecks(scenario);

    int passCount = 0;
    int warnCount = 0;
    int failCount = 0;
    for (const CheckResult& check : checks) {
        if (check.status == Status::Pass) {
            ++passCount;
        } else if (check.status == Status::Warn) {
            ++warnCount;
        } else {
            ++failCount;
        }
    }

    std::cout << std::left << std::setw(24) << scenario.name
              << std::right << std::setw(7) << passCount
              << std::setw(7) << warnCount
              << std::setw(7) << failCount
              << "  ";

    bool first = true;
    for (const CheckResult& check : checks) {
        if (check.status == Status::Pass) {
            continue;
        }
        if (!first) {
            std::cout << " | ";
        }
        first = false;
        std::cout << check.name << ": " << check.detail;
    }

    if (first) {
        std::cout << "all checks passed";
    }
    std::cout << "\n";
}

} // namespace

int main() {
    std::vector<BootScenario> scenarios;

    BootScenario nominal{};
    nominal.name = "nominal V3 board";
    scenarios.push_back(nominal);

    BootScenario baroWrongAddress = nominal;
    baroWrongAddress.name = "BMP388 SDO high";
    baroWrongAddress.bmpAddress = 0x77;
    scenarios.push_back(baroWrongAddress);

    BootScenario overloadedRail = nominal;
    overloadedRail.name = "3V3 overloaded";
    overloadedRail.logicLoad_mA = 720;
    scenarios.push_back(overloadedRail);

    BootScenario flashMismatch = nominal;
    flashMismatch.name = "wrong flash fitted";
    flashMismatch.flashDevice = 0x7017;
    scenarios.push_back(flashMismatch);

    BootScenario busyStuck = nominal;
    busyStuck.name = "radio BUSY stuck";
    busyStuck.radioBusyClears = false;
    scenarios.push_back(busyStuck);

    BootScenario motorNoSupply = nominal;
    motorNoSupply.name = "motor VBUS missing";
    motorNoSupply.motorSupplyPresent = false;
    scenarios.push_back(motorNoSupply);

    std::cout << "AMBAR electronics bring-up sandbox\n";
    std::cout << "This models boot-time checks from the current V3 board constants.\n\n";

    std::cout << std::left << std::setw(24) << "scenario"
              << std::right << std::setw(7) << "pass"
              << std::setw(7) << "warn"
              << std::setw(7) << "fail"
              << "  notes\n";
    std::cout << std::string(106, '-') << "\n";

    for (const BootScenario& scenario : scenarios) {
        printScenario(scenario);
    }

    std::cout << "\nWarnings are intentional review items; failures are boot"
              << " conditions that should keep flight-control logic inhibited.\n";

    return 0;
}
