#include "ambar_airbrake.hpp"

// Architecture role: deterministic fault-injection and recorded-input replay
// suite. It exercises timing and sensor failure policy separately from the
// nominal flight physics sandbox.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

using ambar::Scalar;

struct TestResult {
    std::string name;
    std::string condition;
    std::string rule;
    bool pass = false;
    std::string result;
    std::vector<std::pair<std::string, std::string>> measurements;
    std::string note;
};

struct ReplayKeyframe {
    Scalar time_s = 0.0F;
    Scalar acceleration_mps2 = 0.0F;
    Scalar barometerAltitude_m = 0.0F;
};

struct ReplayResult {
    std::size_t keyframes = 0U;
    std::size_t samples = 0U;
    Scalar peakCommand = 0.0F;
    ambar::FlightPhase finalPhase = ambar::FlightPhase::PadIdle;
    bool healthy = false;
    bool sawBoost = false;
    bool sawCoast = false;
    bool sawAirbrakeActive = false;
};

std::string yesNo(bool value) {
    return value ? "yes" : "no";
}

std::string number(Scalar value, int precision = 3) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

std::vector<ReplayKeyframe> loadReplay(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        return {};
    }

    std::vector<ReplayKeyframe> keyframes;
    std::string line;
    std::getline(input, line);
    while (std::getline(input, line)) {
        if (line.empty() || line.front() == '#') {
            continue;
        }
        std::istringstream row(line);
        std::string time;
        std::string acceleration;
        std::string altitude;
        if (!std::getline(row, time, ',')
            || !std::getline(row, acceleration, ',')
            || !std::getline(row, altitude, ',')) {
            continue;
        }
        keyframes.push_back({
            std::stof(time),
            std::stof(acceleration),
            std::stof(altitude)
        });
    }
    return keyframes;
}

ReplayResult replayLog(const std::vector<ReplayKeyframe>& keyframes) {
    ambar::AmbarFlightComputerConfig config{};
    config.phase.minimumBoostTime_s = 0.40F;
    config.phase.minimumCoastVelocity_mps = 10.0F;
    config.controller.targetApogee_m = 70.0F;
    config.controller.apogeeTolerance_m = 3.0F;
    config.controller.fullDeploymentError_m = 20.0F;
    config.controller.minimumDeployAltitude_m = 20.0F;
    // Leave a visible Coast interval before deployment so the replay can verify
    // the complete phase sequence instead of transitioning in one update.
    config.controller.minimumFlightTime_s = 1.40F;

    ambar::AmbarFlightComputer computer(config);
    computer.resetOnPad(0.0F);
    ReplayResult result{};
    result.keyframes = keyframes.size();

    constexpr Scalar dt_s = 0.01F;
    int barometerDivider = 0;
    for (std::size_t segment = 0; segment + 1U < keyframes.size(); ++segment) {
        const auto& start = keyframes[segment];
        const auto& end = keyframes[segment + 1U];
        const Scalar duration_s = end.time_s - start.time_s;
        if (duration_s <= 0.0F) {
            continue;
        }
        const int steps = static_cast<int>(std::round(duration_s / dt_s));
        for (int step = 1; step <= steps; ++step) {
            const Scalar ratio = static_cast<Scalar>(step) / static_cast<Scalar>(steps);
            const Scalar timestamp_s = start.time_s + ratio * duration_s;
            const Scalar acceleration = start.acceleration_mps2
                + ratio * (end.acceleration_mps2 - start.acceleration_mps2);
            const Scalar altitude = start.barometerAltitude_m
                + ratio * (end.barometerAltitude_m - start.barometerAltitude_m);

            auto output = computer.updateImu(timestamp_s, acceleration);
            if (++barometerDivider == 5) {
                output = computer.updateBarometer(altitude, 1.5F);
                barometerDivider = 0;
            }

            result.peakCommand = std::max(
                result.peakCommand, output.airbrakeCommand.deployFraction);
            result.sawBoost = result.sawBoost || output.phase == ambar::FlightPhase::Boost;
            result.sawCoast = result.sawCoast || output.phase == ambar::FlightPhase::Coast;
            result.sawAirbrakeActive = result.sawAirbrakeActive
                || output.phase == ambar::FlightPhase::AirbrakeActive;
            ++result.samples;
        }
    }

    const auto output = computer.output();
    result.finalPhase = output.phase;
    result.healthy = output.health.healthy;
    return result;
}

TestResult timestampFaultTest() {
    ambar::AmbarFlightComputer computer;
    computer.resetOnPad(0.0F);
    computer.updateImu(0.01F, 0.0F);
    const auto bad = computer.updateImu(0.01F, 0.0F);
    const auto recoveredEstimate = computer.updateImu(0.02F, 0.0F);
    const bool pass = !bad.health.healthy
        && bad.health.rejectedImuSamples == 1U
        && bad.phase == ambar::FlightPhase::Fault
        && recoveredEstimate.health.healthy
        && recoveredEstimate.phase == ambar::FlightPhase::Fault
        && recoveredEstimate.airbrakeCommand.inhibit;
    return {
        "duplicate timestamp fault",
        "Two IMU samples are delivered with the same timestamp, followed by a valid sample.",
        "PASS if the duplicate is rejected, estimator propagation recovers, and the flight-level Fault remains latched with deployment inhibited.",
        pass,
        pass ? "The bad interval was rejected and the conservative flight fault stayed latched." : "Timestamp rejection or fault-latch behavior differed from policy.",
        {
            {"rejected IMU samples", std::to_string(recoveredEstimate.health.rejectedImuSamples)},
            {"estimator recovered", yesNo(recoveredEstimate.health.healthy)},
            {"final phase", ambar::flightPhaseName(recoveredEstimate.phase)},
            {"deployment inhibited", yesNo(recoveredEstimate.airbrakeCommand.inhibit)}
        },
        "A pad reset is required to clear a flight-level Fault; automatic in-flight recovery is intentionally not implemented."
    };
}

TestResult nonFiniteSensorTest() {
    ambar::AmbarFlightComputer computer;
    computer.resetOnPad(0.0F);
    const auto nan = std::numeric_limits<Scalar>::quiet_NaN();
    const auto output = computer.updateImu(0.01F, nan);
    const bool pass = !output.health.healthy
        && output.health.rejectedImuSamples == 1U
        && output.phase == ambar::FlightPhase::Fault
        && output.airbrakeCommand.inhibit;
    return {
        "non-finite IMU sample",
        "The IMU conversion path supplies NaN acceleration.",
        "PASS if the value is rejected before covariance math and deployment is inhibited in Fault.",
        pass,
        pass ? "NaN was contained and could not reach the controller as a valid estimate." : "The invalid sensor value was not safely contained.",
        {
            {"rejected IMU samples", std::to_string(output.health.rejectedImuSamples)},
            {"final phase", ambar::flightPhaseName(output.phase)},
            {"deployment inhibited", yesNo(output.airbrakeCommand.inhibit)}
        },
        "The STM32 driver must still detect stale, saturated, and disconnected sensors before calling this API."
    };
}

TestResult barometerDropoutTest() {
    ambar::VerticalEkf4State estimator;
    estimator.resetOnPad(0.0F);
    for (int sample = 1; sample <= 200; ++sample) {
        estimator.propagateWithImuVerticalAcceleration(sample * 0.01F, 2.0F);
    }
    const auto health = estimator.health();
    const auto estimate = estimator.estimate();
    const bool pass = health.healthy
        && health.rejectedImuSamples == 0U
        && health.rejectedBarometerSamples == 0U
        && std::isfinite(estimate.altitudeAgl_m)
        && std::isfinite(estimate.verticalVelocity_mps);
    return {
        "barometer dropout window",
        "The estimator receives two seconds of valid IMU data with no barometer updates.",
        "PASS if propagation remains finite and healthy without inventing rejected barometer samples.",
        pass,
        pass ? "The estimator continued inertial propagation through the dropout window." : "The estimator became invalid during the dropout window.",
        {
            {"estimated altitude", number(estimate.altitudeAgl_m) + " m"},
            {"estimated velocity", number(estimate.verticalVelocity_mps) + " m/s"},
            {"estimator healthy", yesNo(health.healthy)}
        },
        "The current core API has no barometer timestamp, so stale/frozen detection belongs in the future scheduler/driver layer."
    };
}

TestResult replayTest(const std::string& path) {
    const auto keyframes = loadReplay(path);
    const auto first = replayLog(keyframes);
    const auto second = replayLog(keyframes);
    const bool deterministic = std::fabs(first.peakCommand - second.peakCommand) < 1.0e-6F
        && first.finalPhase == second.finalPhase
        && first.samples == second.samples;
    const bool pass = keyframes.size() >= 5U
        && first.samples >= 100U
        && first.healthy
        && first.sawBoost
        && first.sawCoast
        && first.sawAirbrakeActive
        && first.peakCommand >= 0.0F
        && first.peakCommand <= 1.0F
        && deterministic;
    return {
        "deterministic log replay",
        "A versioned synthetic vertical-flight log is interpolated at 100 Hz and replayed twice through the shared flight computer.",
        "PASS if both replays are identical, remain healthy, traverse Boost/Coast/AirbrakeActive, and keep commands bounded.",
        pass,
        pass ? "The versioned input produced repeatable estimator and controller behavior." : "The replay file was missing, unhealthy, incomplete, or nondeterministic.",
        {
            {"keyframes", std::to_string(first.keyframes)},
            {"interpolated samples", std::to_string(first.samples)},
            {"peak command", number(first.peakCommand * 100.0F, 1) + "%"},
            {"final phase", ambar::flightPhaseName(first.finalPhase)},
            {"repeatable", yesNo(deterministic)}
        },
        "This file is synthetic test evidence, not recorded flight data. Real log replay should use the same adapter once hardware captures exist."
    };
}

void printResult(int index, const TestResult& test) {
    std::cout << "\nTEST CASE " << index << ": " << test.name << "\n";
    std::cout << "Condition being tested: " << test.condition << "\n";
    std::cout << "Pass rule: " << test.rule << "\n";
    std::cout << "Result: " << (test.pass ? "PASS" : "FAIL") << " - " << test.result << "\n";
    std::cout << "Measurements:\n";
    for (const auto& measurement : test.measurements) {
        std::cout << "  " << measurement.first << ": " << measurement.second << "\n";
    }
    if (!test.note.empty()) {
        std::cout << "Target note: " << test.note << "\n";
    }
}

} // namespace

int main(int argc, char** argv) {
    const std::string replayPath = argc > 1
        ? argv[1]
        : "sim/replay/nominal_vertical_log.csv";

    std::cout << "AMBAR fault-injection and log-replay sandbox\n";
    std::cout << "Purpose: make timestamp, invalid-sensor, dropout, and replay behavior explicit and repeatable.\n";
    std::cout << "PASS/FAIL meaning: PASS verifies the stated software policy only; it does not validate physical sensor reliability.\n";

    const std::vector<TestResult> tests{
        timestampFaultTest(),
        nonFiniteSensorTest(),
        barometerDropoutTest(),
        replayTest(replayPath)
    };

    bool allPassed = true;
    for (std::size_t index = 0; index < tests.size(); ++index) {
        printResult(static_cast<int>(index + 1U), tests[index]);
        allPassed = allPassed && tests[index].pass;
    }

    std::cout << "\nSUMMARY\n";
    std::cout << "scenario                         result\n";
    std::cout << "---------------------------------------\n";
    for (const auto& test : tests) {
        std::cout << std::left << std::setw(33) << test.name
                  << (test.pass ? "PASS" : "FAIL") << "\n";
    }
    return allPassed ? 0 : 1;
}
