#include "ambar_board_pins.hpp"
#include "ambar_device_constants.hpp"
#include "ambar_project_requirements.hpp"

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

enum class ExpectedBootDecision {
    ArmableWithWarnings,
    Blocked
};

struct CheckResult {
    Status status = Status::Pass;
    std::string name;
    std::string condition;
    std::string expected;
    std::string observed;
    std::string consequence;
};

struct BootScenario {
    std::string name;
    std::string conditionUnderTest;
    std::string passRule;
    ExpectedBootDecision expectedDecision = ExpectedBootDecision::ArmableWithWarnings;
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
    bool gpsPresent = false;
    float gpsUpdateRate_hz = 0.0F;
    bool motorPresent = true;
    std::uint8_t motorSpiMode = ambar::devices::tmc5240::kSpiMode;
    std::uint8_t motorVersion = ambar::devices::tmc5240::kExpectedIoinVersion;
    bool motorSupplyPresent = true;
};

struct BootSummary {
    int passCount = 0;
    int warnCount = 0;
    int failCount = 0;
    bool armable = false;
    bool scenarioPassed = false;
    std::string decisionLabel;
    std::string resultReason;
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

std::string yesNo(bool value) {
    return value ? "yes" : "no";
}

std::string formatFloat(double value, int precision = 1) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

std::string statusName(Status status) {
    switch (status) {
    case Status::Pass:
        return "PASS";
    case Status::Warn:
        return "WARN";
    case Status::Fail:
        return "FAIL";
    }
    return "FAIL";
}

std::string expectedDecisionName(ExpectedBootDecision decision) {
    switch (decision) {
    case ExpectedBootDecision::ArmableWithWarnings:
        return "ARMABLE_WITH_WARNINGS";
    case ExpectedBootDecision::Blocked:
        return "BLOCKED";
    }
    return "BLOCKED";
}

std::string observedDecisionName(int warnCount, int failCount) {
    if (failCount > 0) {
        return "BLOCKED";
    }
    if (warnCount > 0) {
        return "ARMABLE_WITH_WARNINGS";
    }
    return "ARMABLE";
}

void addCheck(std::vector<CheckResult>& checks,
              Status status,
              const std::string& name,
              const std::string& condition,
              const std::string& expected,
              const std::string& observed,
              const std::string& consequence)
{
    checks.push_back({status, name, condition, expected, observed, consequence});
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
        addCheck(
            checks,
            Status::Pass,
            "STM32 GPIO map",
            "Each named board function must own a unique MCU pin.",
            "No duplicate GPIO names among current board constants.",
            std::to_string(pins.size()) + " named functions checked.",
            "No firmware pin conflict is visible from the constants."
        );
    } else {
        std::ostringstream detail;
        for (const std::string& duplicate : duplicates) {
            detail << duplicate << " ";
        }

        addCheck(
            checks,
            Status::Fail,
            "STM32 GPIO map",
            "Each named board function must own a unique MCU pin.",
            "No duplicate GPIO names among current board constants.",
            "Duplicate pins: " + detail.str(),
            "Firmware should block bring-up until the pin map is corrected."
        );
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
        "MPM3606A 3V3 regulator",
        "Input voltage and estimated 3V3 load must stay inside the regulator constants.",
        "VIN 4500-21000 mV and load <= 600 mA.",
        "VIN=" + std::to_string(scenario.vin_mV) + " mV, load="
            + std::to_string(scenario.logicLoad_mA) + " mA.",
        "Failing this should keep the board from arming because logic power may brown out."
    );

    const bool bmpOk = scenario.bmpPresent
                    && scenario.bmpAddress == ambar::devices::bmp388::kI2cAddress
                    && scenario.bmpChipId == ambar::devices::bmp388::kExpectedChipId;
    addCheck(
        checks,
        bmpOk ? Status::Pass : Status::Fail,
        "BMP388 barometer",
        "Barometer must answer at the expected I2C address and return the expected CHIP_ID.",
        "present=yes, address 0x76, CHIP_ID 0x50.",
        "present=" + yesNo(scenario.bmpPresent)
            + ", address=" + hex8(scenario.bmpAddress)
            + ", CHIP_ID=" + hex8(scenario.bmpChipId) + ".",
        "Failing this means pressure altitude cannot be trusted for the EKF."
    );

    const bool imuOk = scenario.imuPresent
                    && scenario.imuAddress == ambar::devices::lsm6dsv32x::kI2cAddress
                    && scenario.imuWhoAmI == ambar::devices::lsm6dsv32x::kExpectedWhoAmI;
    addCheck(
        checks,
        imuOk ? Status::Pass : Status::Fail,
        "LSM6DSV32X IMU",
        "IMU must answer at the expected I2C address and return the expected WHO_AM_I value.",
        "present=yes, address 0x6A, WHO_AM_I 0x70.",
        "present=" + yesNo(scenario.imuPresent)
            + ", address=" + hex8(scenario.imuAddress)
            + ", WHO_AM_I=" + hex8(scenario.imuWhoAmI) + ".",
        "Failing this should block flight logic because acceleration data is unavailable or wrong."
    );

    const bool imuRangeOk =
        ambar::devices::lsm6dsv32x::kAccelFullScaleG
        >= ambar::requirements::kFlightComputerAccelerationRating_g;
    addCheck(
        checks,
        imuRangeOk ? Status::Pass : Status::Fail,
        "IMU acceleration range",
        "M3 FCR 5.1 says the flight computer must withstand or record at least 30G acceleration.",
        "IMU accelerometer full-scale range >= 30G.",
        "LSM6DSV32X full-scale constant="
            + formatFloat(ambar::devices::lsm6dsv32x::kAccelFullScaleG)
            + "G, requirement="
            + formatFloat(ambar::requirements::kFlightComputerAccelerationRating_g)
            + "G.",
        "Failing this means the selected IMU range may saturate before the report requirement."
    );

    const bool magOk = scenario.magPresent
                    && scenario.magAddress == ambar::devices::lis2mdl::kI2cAddress
                    && scenario.magWhoAmI == ambar::devices::lis2mdl::kExpectedWhoAmI;
    addCheck(
        checks,
        magOk ? Status::Pass : Status::Warn,
        "LIS2MDL magnetometer",
        "Magnetometer should answer with the expected address and WHO_AM_I value.",
        "present=yes, address 0x1E, WHO_AM_I 0x40.",
        "present=" + yesNo(scenario.magPresent)
            + ", address=" + hex8(scenario.magAddress)
            + ", WHO_AM_I=" + hex8(scenario.magWhoAmI) + ".",
        "Warning only for the current vertical EKF; make it blocking once attitude/alignment depends on it."
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
        "W25Q64 flash",
        "External flash must use a supported SPI mode and return the expected JEDEC ID.",
        "present=yes, SPI mode 0 or 3, JEDEC 0xEF 0x4017.",
        "present=" + yesNo(scenario.flashPresent)
            + ", SPI mode=" + std::to_string(scenario.flashSpiMode)
            + ", JEDEC=" + hex8(scenario.flashManufacturer)
            + " " + hex16(scenario.flashDevice) + ".",
        "Failing this should block flight logging and should be visible during preflight."
    );

    const std::size_t recordBytes = 64;
    const std::size_t records =
        ambar::devices::w25q64jv::kTotalSizeBytes / recordBytes;
    const double maxTwoHourRecordRate_hz =
        static_cast<double>(records)
      / static_cast<double>(ambar::requirements::kRequiredLogDuration_s);
    addCheck(
        checks,
        maxTwoHourRecordRate_hz >= 10.0 ? Status::Pass : Status::Warn,
        "flash log capacity",
        "M3 FCR 5.8 says the flight computer shall log data for more than 2 hours.",
        "At least 10 Hz logging for 2 hours with the current 64-byte virtual record size.",
        std::to_string(records) + " records available; two-hour rate limit="
            + formatFloat(maxTwoHourRecordRate_hz, 2) + " Hz before wear strategy.",
        "If the final telemetry record is larger or faster than this, add compression, lower-rate summaries, or more storage."
    );

    const bool radioOk = scenario.radioPresent
                      && scenario.radioSpiMode == ambar::devices::sx1280::kSpiMode
                      && scenario.radioBusyClears;
    addCheck(
        checks,
        radioOk ? Status::Pass : Status::Fail,
        "SX1280 radio",
        "Radio SPI mode and BUSY handshake must match the datasheet-backed constants.",
        "present=yes, SPI mode 0, BUSY clears.",
        "present=" + yesNo(scenario.radioPresent)
            + ", SPI mode=" + std::to_string(scenario.radioSpiMode)
            + (scenario.radioBusyClears ? ", BUSY clears." : ", BUSY stuck."),
        "Failing this should block telemetry bring-up and may block arming depending on final rules."
    );

    addCheck(
        checks,
        scenario.groundStationCompatible ? Status::Pass : Status::Warn,
        "radio link compatibility",
        "M3 FCR 5.10 and GSR 7.6 call out 915 MHz, while the current SX1280 constants are 2.4-2.5 GHz.",
        "Either update the requirement or choose radio hardware/ground station that match.",
        "SX1280 RF range="
            + formatFloat(ambar::devices::sx1280::kMinRfFrequencyHz / 1000000.0, 0)
            + "-"
            + formatFloat(ambar::devices::sx1280::kMaxRfFrequencyHz / 1000000.0, 0)
            + " MHz; report frequency="
            + formatFloat(ambar::requirements::kGroundStationReportFrequency_mhz, 0)
            + " MHz; groundStationCompatible="
            + yesNo(scenario.groundStationCompatible) + ".",
        "Warning means the chip may boot, but the RF plan is not yet requirement-clean."
    );

    const bool gpsOk =
        scenario.gpsPresent
     && scenario.gpsUpdateRate_hz >= ambar::requirements::kMinimumGpsUpdateRate_hz;
    addCheck(
        checks,
        gpsOk ? Status::Pass : Status::Warn,
        "GPS requirement",
        "M3 FCR 5.9 requires a GPS chip update rate of 5 Hz or greater.",
        "GPS present and update rate >= 5 Hz.",
        "gpsPresent=" + yesNo(scenario.gpsPresent)
            + ", gpsUpdateRate="
            + formatFloat(scenario.gpsUpdateRate_hz, 1)
            + " Hz.",
        "Warning means current airbrake firmware/PCB constants do not yet prove the report GPS requirement."
    );

    const bool motorOk = scenario.motorPresent
                      && scenario.motorSupplyPresent
                      && scenario.motorSpiMode == ambar::devices::tmc5240::kSpiMode
                      && scenario.motorVersion == ambar::devices::tmc5240::kExpectedIoinVersion;
    addCheck(
        checks,
        motorOk ? Status::Pass : Status::Fail,
        "TMC5240 motor driver",
        "Motor driver must answer over SPI mode 3, report expected IOIN.VERSION, and have motor supply present.",
        "present=yes, VBUS=yes, SPI mode 3, IOIN.VERSION 0x40.",
        "present=" + yesNo(scenario.motorPresent)
            + ", VBUS=" + yesNo(scenario.motorSupplyPresent)
            + ", SPI mode=" + std::to_string(scenario.motorSpiMode)
            + ", IOIN.VERSION=" + hex8(scenario.motorVersion) + ".",
        "Failing this should block actuator commands and keep airbrakes retracted."
    );

    const std::vector<CheckResult> pinChecks = checkPinCollisions();
    checks.insert(checks.end(), pinChecks.begin(), pinChecks.end());

    return checks;
}

BootSummary summarize(const BootScenario& scenario,
                      const std::vector<CheckResult>& checks)
{
    BootSummary summary{};
    for (const CheckResult& check : checks) {
        if (check.status == Status::Pass) {
            ++summary.passCount;
        } else if (check.status == Status::Warn) {
            ++summary.warnCount;
        } else {
            ++summary.failCount;
        }
    }

    summary.armable = summary.failCount == 0;
    summary.decisionLabel = observedDecisionName(summary.warnCount, summary.failCount);

    const bool expectedBlocked =
        scenario.expectedDecision == ExpectedBootDecision::Blocked;
    summary.scenarioPassed =
        expectedBlocked ? !summary.armable : summary.armable;

    if (summary.scenarioPassed && expectedBlocked) {
        summary.resultReason = "Injected fault produced a BLOCKED boot decision.";
    } else if (summary.scenarioPassed) {
        summary.resultReason = "No blocking failures were found; warnings remain visible.";
    } else if (expectedBlocked) {
        summary.resultReason = "Injected fault did not block the boot decision.";
    } else {
        summary.resultReason = "Nominal board unexpectedly produced a blocking failure.";
    }

    return summary;
}

void printScenario(int index, const BootScenario& scenario) {
    const std::vector<CheckResult> checks = runBootChecks(scenario);
    const BootSummary summary = summarize(scenario, checks);

    std::cout << "\nTEST CASE " << index << ": " << scenario.name << "\n";
    std::cout << "Condition being tested: " << scenario.conditionUnderTest << "\n";
    std::cout << "Pass rule: " << scenario.passRule << "\n";
    std::cout << "Expected boot decision: "
              << expectedDecisionName(scenario.expectedDecision) << "\n";
    std::cout << "Observed boot decision: " << summary.decisionLabel << "\n";
    std::cout << "Result: " << (summary.scenarioPassed ? "PASS" : "FAIL")
              << " - " << summary.resultReason << "\n";
    std::cout << "Check counts: PASS=" << summary.passCount
              << ", WARN=" << summary.warnCount
              << ", FAIL=" << summary.failCount << "\n";
    std::cout << "Detailed check log:\n";

    for (const CheckResult& check : checks) {
        std::cout << "  [" << statusName(check.status) << "] "
                  << check.name << "\n";
        std::cout << "    Test:     " << check.condition << "\n";
        std::cout << "    Expected: " << check.expected << "\n";
        std::cout << "    Observed: " << check.observed << "\n";
        std::cout << "    Meaning:  " << check.consequence << "\n";
    }
}

void printSummaryRow(const BootScenario& scenario) {
    const std::vector<CheckResult> checks = runBootChecks(scenario);
    const BootSummary summary = summarize(scenario, checks);

    std::cout << std::left << std::setw(24) << scenario.name
              << std::setw(8) << (summary.scenarioPassed ? "PASS" : "FAIL")
              << std::setw(24) << expectedDecisionName(scenario.expectedDecision)
              << std::setw(24) << summary.decisionLabel
              << std::right << std::setw(7) << summary.passCount
              << std::setw(7) << summary.warnCount
              << std::setw(7) << summary.failCount
              << "\n";
}

} // namespace

int main() {
    std::vector<BootScenario> scenarios;

    BootScenario nominal{};
    nominal.name = "nominal V3 board";
    nominal.conditionUnderTest =
        "All modeled chips respond with the current V3 constants; radio ground-station compatibility is still open.";
    nominal.passRule =
        "PASS if no blocking failures are found and the open radio-link item remains visible as a warning.";
    nominal.expectedDecision = ExpectedBootDecision::ArmableWithWarnings;
    scenarios.push_back(nominal);

    BootScenario baroWrongAddress = nominal;
    baroWrongAddress.name = "BMP388 SDO high";
    baroWrongAddress.conditionUnderTest =
        "The barometer address strap is wrong, so firmware reads 0x77 instead of the expected 0x76.";
    baroWrongAddress.passRule =
        "PASS if the BMP388 check fails and the boot decision becomes BLOCKED.";
    baroWrongAddress.expectedDecision = ExpectedBootDecision::Blocked;
    baroWrongAddress.bmpAddress = 0x77;
    scenarios.push_back(baroWrongAddress);

    BootScenario overloadedRail = nominal;
    overloadedRail.name = "3V3 overloaded";
    overloadedRail.conditionUnderTest =
        "The MPM3606A 3V3 rail is asked to supply 720 mA, above the modeled 600 mA continuous limit.";
    overloadedRail.passRule =
        "PASS if the regulator check fails and the boot decision becomes BLOCKED.";
    overloadedRail.expectedDecision = ExpectedBootDecision::Blocked;
    overloadedRail.logicLoad_mA = 720;
    scenarios.push_back(overloadedRail);

    BootScenario flashMismatch = nominal;
    flashMismatch.name = "wrong flash fitted";
    flashMismatch.conditionUnderTest =
        "The flash chip responds with a JEDEC device ID that does not match W25Q64JV.";
    flashMismatch.passRule =
        "PASS if the flash check fails and the boot decision becomes BLOCKED.";
    flashMismatch.expectedDecision = ExpectedBootDecision::Blocked;
    flashMismatch.flashDevice = 0x7017;
    scenarios.push_back(flashMismatch);

    BootScenario busyStuck = nominal;
    busyStuck.name = "radio BUSY stuck";
    busyStuck.conditionUnderTest =
        "The SX1280 BUSY pin never clears after a command.";
    busyStuck.passRule =
        "PASS if the radio check fails and the boot decision becomes BLOCKED.";
    busyStuck.expectedDecision = ExpectedBootDecision::Blocked;
    busyStuck.radioBusyClears = false;
    scenarios.push_back(busyStuck);

    BootScenario motorNoSupply = nominal;
    motorNoSupply.name = "motor VBUS missing";
    motorNoSupply.conditionUnderTest =
        "The TMC5240 digital interface responds, but motor supply voltage is missing.";
    motorNoSupply.passRule =
        "PASS if the motor-driver check fails and the boot decision becomes BLOCKED.";
    motorNoSupply.expectedDecision = ExpectedBootDecision::Blocked;
    motorNoSupply.motorSupplyPresent = false;
    scenarios.push_back(motorNoSupply);

    std::cout << "AMBAR electronics bring-up sandbox\n";
    std::cout << "Purpose: model the startup checks firmware should run before"
              << " trusting the current V3 PCB/chip set.\n";
    std::cout << "PASS/FAIL meaning: PASS means the virtual boot logic made the"
              << " expected ARMABLE/BLOCKED decision for the injected condition.\n";
    std::cout << "WARN meaning: the board may continue in this model, but the item"
              << " must stay visible for team review.\n";

    int index = 1;
    for (const BootScenario& scenario : scenarios) {
        printScenario(index, scenario);
        ++index;
    }

    std::cout << "\nSUMMARY\n";
    std::cout << std::left << std::setw(24) << "scenario"
              << std::setw(8) << "result"
              << std::setw(24) << "expected decision"
              << std::setw(24) << "observed decision"
              << std::right << std::setw(7) << "pass"
              << std::setw(7) << "warn"
              << std::setw(7) << "fail"
              << "\n";
    std::cout << std::string(95, '-') << "\n";

    for (const BootScenario& scenario : scenarios) {
        printSummaryRow(scenario);
    }

    std::cout << "\nNext realism upgrade: replace these direct fields with fake"
              << " I2C/SPI transactions so future drivers can be tested against"
              << " virtual register reads before the PCB is powered.\n";

    return 0;
}
