#include "ambar_airbrake.hpp"

// Architecture role: fixed-seed parameter-dispersion study for software safety
// behavior. It is intentionally a lightweight 1-D model; RocketPy remains the
// higher-fidelity trajectory reference.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

namespace {

using ambar::Scalar;

class RandomSequence {
public:
    explicit RandomSequence(std::uint32_t seed) : state_(seed) {}

    Scalar uniform(Scalar low, Scalar high) {
        state_ = 1664525U * state_ + 1013904223U;
        const Scalar unit = static_cast<Scalar>((state_ >> 8U) & 0x00FFFFFFU)
                          / static_cast<Scalar>(0x00FFFFFFU);
        return low + (high - low) * unit;
    }

private:
    std::uint32_t state_;
};

struct TrialResult {
    Scalar apogee_m = 0.0F;
    Scalar peakCommand = 0.0F;
    bool healthy = false;
    bool commandBounded = true;
    bool commandBeforeBurnout = false;
    bool commandWhileDescending = false;
};

struct StudyResult {
    int trials = 0;
    int targetHits = 0;
    int healthyTrials = 0;
    int boundedTrials = 0;
    int earlyCommandTrials = 0;
    int descendingCommandTrials = 0;
    Scalar minimumApogee_m = 1.0e9F;
    Scalar maximumApogee_m = 0.0F;
    Scalar meanApogee_m = 0.0F;
    Scalar checksum = 0.0F;
};

Scalar moveToward(Scalar current, Scalar target, Scalar maximumDelta) {
    if (target > current) {
        return std::min(target, current + maximumDelta);
    }
    return std::max(target, current - maximumDelta);
}

TrialResult runTrial(RandomSequence& random) {
    const Scalar boostAcceleration = random.uniform(106.0F, 128.0F);
    const Scalar burnTime_s = random.uniform(1.58F, 1.78F);
    const Scalar coastDrag = random.uniform(2.4F, 4.0F);
    const Scalar airbrakeDrag = random.uniform(12.0F, 20.0F);
    const Scalar imuBias = random.uniform(-1.0F, 1.0F);
    const Scalar actuatorRate = random.uniform(0.9F, 2.2F);

    ambar::FlightPhaseConfig phaseConfig{};
    phaseConfig.minimumBoostTime_s = 0.50F;
    ambar::AmbarFlightComputerConfig config{};
    config.phase = phaseConfig;
    ambar::AmbarFlightComputer computer(config);
    computer.resetOnPad(0.0F);

    constexpr Scalar dt_s = 0.005F;
    constexpr Scalar endTime_s = 9.0F;
    Scalar altitude_m = 0.0F;
    Scalar velocity_mps = 0.0F;
    Scalar deployment = 0.0F;
    TrialResult result{};
    auto output = computer.output();

    const int samples = static_cast<int>(endTime_s / dt_s);
    for (int sample = 1; sample <= samples; ++sample) {
        const Scalar time_s = sample * dt_s;
        deployment = moveToward(
            deployment,
            output.airbrakeCommand.inhibit ? 0.0F : output.airbrakeCommand.deployFraction,
            actuatorRate * dt_s);

        Scalar acceleration_mps2 = 0.0F;
        if (time_s >= 0.20F && time_s < 0.20F + burnTime_s) {
            acceleration_mps2 = boostAcceleration
                - 0.0015F * velocity_mps * std::fabs(velocity_mps);
        } else if (time_s >= 0.20F + burnTime_s) {
            const Scalar speedScale = std::min(std::fabs(velocity_mps) / 220.0F, 1.5F);
            acceleration_mps2 = -ambar::kStandardGravityMps2
                - coastDrag * speedScale * speedScale
                - airbrakeDrag * deployment;
        }

        velocity_mps += acceleration_mps2 * dt_s;
        altitude_m = std::max(0.0F, altitude_m + velocity_mps * dt_s);
        result.apogee_m = std::max(result.apogee_m, altitude_m);

        const Scalar measuredAcceleration = acceleration_mps2
            + imuBias
            + random.uniform(-0.25F, 0.25F);
        output = computer.updateImu(time_s, measuredAcceleration);
        if (sample % 4 == 0) {
            output = computer.updateBarometer(
                altitude_m + random.uniform(-1.0F, 1.0F), 1.5F);
        }

        const Scalar command = output.airbrakeCommand.deployFraction;
        result.peakCommand = std::max(result.peakCommand, command);
        result.commandBounded = result.commandBounded
            && command >= 0.0F && command <= 1.0F
            && deployment >= 0.0F && deployment <= 1.0F;
        if (command > 0.001F) {
            result.commandBeforeBurnout = result.commandBeforeBurnout
                || time_s < 0.20F + burnTime_s;
            result.commandWhileDescending = result.commandWhileDescending
                || velocity_mps <= 0.0F;
        }
    }
    result.healthy = computer.output().health.healthy;
    return result;
}

StudyResult runStudy(std::uint32_t seed, int trialCount) {
    RandomSequence random(seed);
    StudyResult result{};
    result.trials = trialCount;
    for (int trial = 0; trial < trialCount; ++trial) {
        const TrialResult sample = runTrial(random);
        const Scalar apogee_ft = sample.apogee_m * ambar::kMetersToFeet;
        if (std::fabs(apogee_ft - 3000.0F) <= 100.0F) {
            ++result.targetHits;
        }
        result.healthyTrials += sample.healthy ? 1 : 0;
        result.boundedTrials += sample.commandBounded ? 1 : 0;
        result.earlyCommandTrials += sample.commandBeforeBurnout ? 1 : 0;
        result.descendingCommandTrials += sample.commandWhileDescending ? 1 : 0;
        result.minimumApogee_m = std::min(result.minimumApogee_m, sample.apogee_m);
        result.maximumApogee_m = std::max(result.maximumApogee_m, sample.apogee_m);
        result.meanApogee_m += sample.apogee_m;
        result.checksum += sample.apogee_m * static_cast<Scalar>(trial + 1);
    }
    result.meanApogee_m /= static_cast<Scalar>(trialCount);
    return result;
}

void printCase(int number,
               const std::string& name,
               const std::string& condition,
               const std::string& rule,
               bool pass,
               const std::string& result) {
    std::cout << "\nTEST CASE " << number << ": " << name << "\n";
    std::cout << "Condition being tested: " << condition << "\n";
    std::cout << "Pass rule: " << rule << "\n";
    std::cout << "Result: " << (pass ? "PASS" : "FAIL") << " - " << result << "\n";
}

} // namespace

int main() {
    constexpr std::uint32_t seed = 0xA4B4C202U;
    constexpr int trials = 200;
    const StudyResult first = runStudy(seed, trials);
    const StudyResult second = runStudy(seed, trials);

    const bool reproducible = std::fabs(first.checksum - second.checksum) < 0.01F
        && first.targetHits == second.targetHits
        && first.earlyCommandTrials == second.earlyCommandTrials;
    const bool safetyPass = first.healthyTrials == trials
        && first.boundedTrials == trials
        && first.earlyCommandTrials == 0
        && first.descendingCommandTrials == 0;

    std::cout << "AMBAR fixed-seed Monte Carlo sandbox\n";
    std::cout << "Purpose: disperse provisional thrust, drag, sensor, and actuator inputs while checking software safety invariants.\n";
    std::cout << "Model limitation: this is a 1-D software study, not a probability-of-success claim for the physical rocket.\n";

    printCase(
        1,
        "fixed-seed reproducibility",
        "The same 200-trial dispersion study is executed twice with seed 0xA4B4C202.",
        "PASS if both runs produce the same checksum and event counts.",
        reproducible,
        reproducible ? "The study is deterministic enough for CI regression checks." : "The repeated study produced different results.");
    std::cout << "Measurements:\n";
    std::cout << "  trials: " << trials << "\n";
    std::cout << "  checksum: " << std::fixed << std::setprecision(2) << first.checksum << "\n";

    printCase(
        2,
        "dispersed safety invariants",
        "Boost acceleration, burn time, drag, IMU bias/noise, barometer noise, and actuator rate vary across 200 trials.",
        "PASS if every trial remains healthy and bounded, with no command before burnout or while descending.",
        safetyPass,
        safetyPass ? "All dispersed trials preserved the declared controller safety invariants." : "At least one dispersed trial violated a safety invariant.");
    std::cout << "Measurements:\n";
    std::cout << "  healthy trials: " << first.healthyTrials << "/" << trials << "\n";
    std::cout << "  bounded trials: " << first.boundedTrials << "/" << trials << "\n";
    std::cout << "  early command trials: " << first.earlyCommandTrials << "\n";
    std::cout << "  descending command trials: " << first.descendingCommandTrials << "\n";
    std::cout << "  apogee range: " << std::setprecision(0)
              << first.minimumApogee_m * ambar::kMetersToFeet << " to "
              << first.maximumApogee_m * ambar::kMetersToFeet << " ft\n";
    std::cout << "  mean apogee: " << first.meanApogee_m * ambar::kMetersToFeet << " ft\n";
    std::cout << "  inside 3000 +/-100 ft: " << first.targetHits << "/" << trials << "\n";
    std::cout << "Target note: The target-hit count is informational until RocketPy/OpenRocket inputs are reconciled and uncertainty distributions are source-backed.\n";

    std::cout << "\nSUMMARY\n";
    std::cout << "scenario                         result\n";
    std::cout << "---------------------------------------\n";
    std::cout << std::left << std::setw(33) << "fixed-seed reproducibility"
              << (reproducible ? "PASS" : "FAIL") << "\n";
    std::cout << std::left << std::setw(33) << "dispersed safety invariants"
              << (safetyPass ? "PASS" : "FAIL") << "\n";
    return reproducible && safetyPass ? 0 : 1;
}
