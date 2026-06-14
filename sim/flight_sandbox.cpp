#include "ambar_airbrake.hpp"

// Architecture role: fast deterministic regression suite for the shared C++
// estimator/controller. It injects sensor noise and faults directly, making it
// useful for software behavior checks without the cost of RocketPy physics.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

using ambar::Scalar;

class DeterministicNoise {
public:
    explicit DeterministicNoise(std::uint32_t seed)
        : state_(seed)
    {
    }

    Scalar sample(Scalar amplitude) {
        state_ = 1664525U * state_ + 1013904223U;
        const Scalar unit = static_cast<Scalar>((state_ >> 8U) & 0x00FFFFFFU)
                          / static_cast<Scalar>(0x00FFFFFFU);
        return amplitude * (2.0F * unit - 1.0F);
    }

private:
    std::uint32_t state_;
};

enum class FlightExpectation {
    NominalDeploys,
    BarometerSpikeRejected,
    BiasStaysBounded,
    SlowActuatorLagVisible,
    WeakMotorStaysInhibited
};

struct FlightScenario {
    std::string name;
    std::string conditionUnderTest;
    std::string passRule;
    FlightExpectation expectation;
    Scalar netBoostAcceleration_mps2;
    Scalar motorBurnTime_s;
    Scalar baseCoastDrag_mps2;
    Scalar airbrakeDragAtFullDeploy_mps2;
    Scalar imuBias_mps2;
    Scalar imuNoiseAmplitude_mps2;
    Scalar barometerNoiseAmplitude_m;
    bool injectBarometerSpike;
    Scalar barometerSpikeTime_s;
    Scalar barometerSpike_m;
    Scalar actuatorRate_fractionPerS;
};

struct FlightResult {
    std::string name;
    Scalar truePeakAltitude_m = 0.0F;
    Scalar estimatedAltitude_m = 0.0F;
    Scalar predictedApogee_m = 0.0F;
    Scalar peakCommandFraction = 0.0F;
    Scalar peakActualDeployFraction = 0.0F;
    Scalar finalActualDeployFraction = 0.0F;
    Scalar firstFullCommandTime_s = -1.0F;
    Scalar firstFullActualDeployTime_s = -1.0F;
    ambar::FlightPhase finalPhase = ambar::FlightPhase::PadIdle;
    std::uint32_t rejectedImuSamples = 0U;
    std::uint32_t rejectedBarometerSamples = 0U;
    bool healthy = false;
};

struct FlightVerdict {
    bool pass = false;
    std::string reason;
};

Scalar clamp(Scalar value, Scalar lower, Scalar upper) {
    return std::max(lower, std::min(value, upper));
}

Scalar moveToward(Scalar current, Scalar target, Scalar maxDelta) {
    if (target > current) {
        return std::min(target, current + maxDelta);
    }
    return std::max(target, current - maxDelta);
}

std::string passFail(bool pass) {
    return pass ? "PASS" : "FAIL";
}

std::string yesNo(bool value) {
    return value ? "yes" : "no";
}

std::string secondsOrNotReached(Scalar timestamp_s) {
    if (timestamp_s < 0.0F) {
        return "not reached";
    }

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << timestamp_s << " s";
    return stream.str();
}

Scalar simulatedAcceleration(const FlightScenario& scenario,
                             Scalar timestamp_s,
                             Scalar velocity_mps,
                             Scalar actualDeployFraction)
{
    if (timestamp_s < 0.20F) {
        return 0.0F;
    }

    if (timestamp_s < 0.20F + scenario.motorBurnTime_s) {
        const Scalar boostDrag_mps2 = 0.0015F * velocity_mps * std::fabs(velocity_mps);
        return scenario.netBoostAcceleration_mps2 - boostDrag_mps2;
    }

    const Scalar speedScale = clamp(std::fabs(velocity_mps) / 220.0F, 0.0F, 1.5F);
    const Scalar coastDrag_mps2 =
        scenario.baseCoastDrag_mps2 * speedScale * speedScale
      + scenario.airbrakeDragAtFullDeploy_mps2 * actualDeployFraction;

    return -ambar::kStandardGravityMps2 - coastDrag_mps2;
}

FlightResult runScenario(const FlightScenario& scenario) {
    ambar::AmbarFlightComputer computer;
    computer.resetOnPad(0.0F);

    DeterministicNoise imuNoise(0xA123U);
    DeterministicNoise baroNoise(0xB456U);

    constexpr Scalar dt_s = 0.002F;
    constexpr Scalar endTime_s = 9.0F;
    constexpr int barometerDecimation = 10;

    Scalar trueAltitude_m = 0.0F;
    Scalar trueVelocity_mps = 0.0F;
    Scalar actualDeployFraction = 0.0F;
    Scalar peakActualDeployFraction = 0.0F;
    Scalar peakCommandFraction = 0.0F;
    Scalar truePeakAltitude_m = 0.0F;
    Scalar firstFullCommandTime_s = -1.0F;
    Scalar firstFullActualDeployTime_s = -1.0F;
    bool spikeUsed = false;

    ambar::AmbarFlightComputerOutput output = computer.output();

    const int sampleCount = static_cast<int>(endTime_s / dt_s);
    for (int sample = 1; sample <= sampleCount; ++sample) {
        const Scalar timestamp_s = sample * dt_s;

        actualDeployFraction = moveToward(
            actualDeployFraction,
            output.airbrakeCommand.deployFraction,
            scenario.actuatorRate_fractionPerS * dt_s
        );

        const Scalar acceleration_mps2 = simulatedAcceleration(
            scenario,
            timestamp_s,
            trueVelocity_mps,
            actualDeployFraction
        );

        trueVelocity_mps += acceleration_mps2 * dt_s;
        trueAltitude_m += trueVelocity_mps * dt_s;

        if (trueAltitude_m < 0.0F) {
            trueAltitude_m = 0.0F;
            trueVelocity_mps = 0.0F;
        }

        truePeakAltitude_m = std::max(truePeakAltitude_m, trueAltitude_m);

        const Scalar measuredAcceleration_mps2 =
            acceleration_mps2
          + scenario.imuBias_mps2
          + imuNoise.sample(scenario.imuNoiseAmplitude_mps2);

        output = computer.updateImu(timestamp_s, measuredAcceleration_mps2);

        if (sample % barometerDecimation == 0) {
            Scalar measuredAltitude_m =
                trueAltitude_m + baroNoise.sample(scenario.barometerNoiseAmplitude_m);

            if (scenario.injectBarometerSpike
                && !spikeUsed
                && timestamp_s >= scenario.barometerSpikeTime_s) {
                measuredAltitude_m += scenario.barometerSpike_m;
                spikeUsed = true;
            }

            output = computer.updateBarometer(measuredAltitude_m, 1.5F);
        }

        if (output.airbrakeCommand.deployFraction >= 0.95F
            && firstFullCommandTime_s < 0.0F) {
            firstFullCommandTime_s = timestamp_s;
        }

        if (actualDeployFraction >= 0.95F
            && firstFullActualDeployTime_s < 0.0F) {
            firstFullActualDeployTime_s = timestamp_s;
        }

        peakCommandFraction =
            std::max(peakCommandFraction, output.airbrakeCommand.deployFraction);
        peakActualDeployFraction =
            std::max(peakActualDeployFraction, actualDeployFraction);
    }

    output = computer.output();

    FlightResult result{};
    result.name = scenario.name;
    result.truePeakAltitude_m = truePeakAltitude_m;
    result.estimatedAltitude_m = output.estimate.altitudeAgl_m;
    result.predictedApogee_m = output.estimate.predictedApogee_m;
    result.peakCommandFraction = peakCommandFraction;
    result.peakActualDeployFraction = peakActualDeployFraction;
    result.finalActualDeployFraction = actualDeployFraction;
    result.firstFullCommandTime_s = firstFullCommandTime_s;
    result.firstFullActualDeployTime_s = firstFullActualDeployTime_s;
    result.finalPhase = output.phase;
    result.rejectedImuSamples = output.health.rejectedImuSamples;
    result.rejectedBarometerSamples = output.health.rejectedBarometerSamples;
    result.healthy = output.health.healthy;
    return result;
}

FlightVerdict evaluateScenario(const FlightScenario& scenario,
                               const FlightResult& result)
{
    const bool commandBounded =
        result.peakCommandFraction >= 0.0F
     && result.peakCommandFraction <= 1.0001F
     && result.peakActualDeployFraction >= 0.0F
     && result.peakActualDeployFraction <= 1.0001F;

    switch (scenario.expectation) {
    case FlightExpectation::NominalDeploys: {
        const bool pass = result.healthy
                       && result.finalPhase == ambar::FlightPhase::AirbrakeActive
                       && result.peakCommandFraction >= 0.95F
                       && result.peakActualDeployFraction >= 0.95F
                       && result.rejectedBarometerSamples == 0U;
        return {
            pass,
            pass
                ? "Estimator stayed healthy, entered AirbrakeActive, and commanded full deployment."
                : "Expected healthy AirbrakeActive full-deploy behavior was not observed."
        };
    }

    case FlightExpectation::BarometerSpikeRejected: {
        const bool pass = result.healthy
                       && result.rejectedBarometerSamples >= 1U
                       && result.finalPhase != ambar::FlightPhase::Fault;
        return {
            pass,
            pass
                ? "At least one bad barometer sample was rejected and the flight computer stayed out of Fault."
                : "The injected barometer spike was not clearly rejected or it destabilized the flight computer."
        };
    }

    case FlightExpectation::BiasStaysBounded: {
        const bool pass = result.healthy
                       && commandBounded
                       && result.finalPhase != ambar::FlightPhase::Fault;
        return {
            pass,
            pass
                ? "IMU bias did not produce an invalid command or force a Fault phase."
                : "IMU bias produced an unhealthy estimate, invalid command, or Fault phase."
        };
    }

    case FlightExpectation::SlowActuatorLagVisible: {
        const bool lagMeasured =
            result.firstFullCommandTime_s >= 0.0F
         && result.firstFullActualDeployTime_s >= 0.0F
         && result.firstFullActualDeployTime_s - result.firstFullCommandTime_s > 0.25F;
        const bool pass = result.healthy && lagMeasured;
        return {
            pass,
            pass
                ? "Software reached full command before the virtual actuator reached full deployment, so lag is visible."
                : "The virtual actuator lag was not visible in the measured command/deploy timing."
        };
    }

    case FlightExpectation::WeakMotorStaysInhibited: {
        const bool pass = result.healthy
                       && result.peakCommandFraction <= 0.05F
                       && result.peakActualDeployFraction <= 0.05F
                       && result.finalPhase == ambar::FlightPhase::Coast;
        return {
            pass,
            pass
                ? "Weak-motor flight stayed in Coast with airbrake deployment effectively inhibited."
                : "Weak-motor flight unexpectedly commanded deployment or changed phase."
        };
    }
    }

    return {false, "Unknown expectation."};
}

void printDetailedResult(int index,
                         const FlightScenario& scenario,
                         const FlightResult& result)
{
    const FlightVerdict verdict = evaluateScenario(scenario, result);
    const Scalar trueApogee_ft = result.truePeakAltitude_m * ambar::kMetersToFeet;
    const Scalar estimatedAltitude_ft = result.estimatedAltitude_m * ambar::kMetersToFeet;
    const Scalar predictedApogee_ft = result.predictedApogee_m * ambar::kMetersToFeet;
    const Scalar targetError_ft = trueApogee_ft - 3000.0F;
    const bool targetWithinTolerance =
        std::fabs(targetError_ft) <= ambar::kTargetToleranceM * ambar::kMetersToFeet;

    std::cout << "\nTEST CASE " << index << ": " << scenario.name << "\n";
    std::cout << "Condition being tested: " << scenario.conditionUnderTest << "\n";
    std::cout << "Pass rule: " << scenario.passRule << "\n";
    std::cout << "Result: " << passFail(verdict.pass) << " - " << verdict.reason << "\n";

    std::cout << "Measurements:\n";
    std::cout << "  true apogee:          " << std::fixed << std::setprecision(0)
              << trueApogee_ft << " ft\n";
    std::cout << "  target error:         " << targetError_ft << " ft\n";
    std::cout << "  estimated altitude:   " << estimatedAltitude_ft << " ft\n";
    std::cout << "  predicted apogee:     " << predictedApogee_ft << " ft\n";
    std::cout << "  peak command:         " << std::setprecision(1)
              << result.peakCommandFraction * 100.0F << "%\n";
    std::cout << "  peak actual deploy:   "
              << result.peakActualDeployFraction * 100.0F << "%\n";
    std::cout << "  final actual deploy:  "
              << result.finalActualDeployFraction * 100.0F << "%\n";
    std::cout << "  full command time:    "
              << secondsOrNotReached(result.firstFullCommandTime_s) << "\n";
    std::cout << "  full deploy time:     "
              << secondsOrNotReached(result.firstFullActualDeployTime_s) << "\n";
    std::cout << "  rejected IMU samples: " << result.rejectedImuSamples << "\n";
    std::cout << "  rejected baro reads:  " << result.rejectedBarometerSamples << "\n";
    std::cout << "  final phase:          "
              << ambar::flightPhaseName(result.finalPhase) << "\n";
    std::cout << "  estimator healthy:    " << yesNo(result.healthy) << "\n";

    std::cout << "Target note: "
              << (targetWithinTolerance ? "PASS" : "WARN")
              << " - true apogee is "
              << (targetWithinTolerance ? "inside" : "outside")
              << " the +/-100 ft mission tolerance. This is a calibration note,"
              << " not the safety-logic pass/fail result above.\n";
}

void printSummaryRow(const FlightScenario& scenario,
                     const FlightResult& result)
{
    const FlightVerdict verdict = evaluateScenario(scenario, result);
    const Scalar trueApogee_ft = result.truePeakAltitude_m * ambar::kMetersToFeet;
    const Scalar predictedApogee_ft = result.predictedApogee_m * ambar::kMetersToFeet;

    std::cout << std::left << std::setw(24) << scenario.name
              << std::setw(8) << passFail(verdict.pass)
              << std::right << std::setw(10) << std::fixed << std::setprecision(0)
              << trueApogee_ft
              << std::setw(12) << predictedApogee_ft
              << std::setw(10) << std::setprecision(1)
              << result.peakCommandFraction * 100.0F
              << std::setw(10) << result.peakActualDeployFraction * 100.0F
              << std::setw(10) << result.rejectedBarometerSamples
              << std::setw(17) << ambar::flightPhaseName(result.finalPhase)
              << "\n";
}

} // namespace

int main() {
    const std::vector<FlightScenario> scenarios{
        {
            "baseline",
            "Nominal noisy IMU/barometer data with a strong motor and normal actuator speed.",
            "PASS if the estimator remains healthy, reaches AirbrakeActive, commands full deployment, and rejects no normal barometer reads.",
            FlightExpectation::NominalDeploys,
            118.0F,
            1.70F,
            3.0F,
            16.0F,
            0.0F,
            0.20F,
            0.75F,
            false,
            0.0F,
            0.0F,
            2.0F
        },
        {
            "baro spike",
            "A one-time +120 m pressure-altitude spike at 2.6 s.",
            "PASS if the EKF rejects at least one barometer read and the flight computer does not enter Fault.",
            FlightExpectation::BarometerSpikeRejected,
            118.0F,
            1.70F,
            3.0F,
            16.0F,
            0.0F,
            0.20F,
            0.75F,
            true,
            2.60F,
            120.0F,
            2.0F
        },
        {
            "imu +1.5mps2 bias",
            "The accelerometer reports 1.5 m/s^2 too high for the whole flight.",
            "PASS if the estimator stays healthy, commands remain bounded from 0% to 100%, and no Fault phase appears.",
            FlightExpectation::BiasStaysBounded,
            118.0F,
            1.70F,
            3.0F,
            16.0F,
            1.5F,
            0.20F,
            0.75F,
            false,
            0.0F,
            0.0F,
            2.0F
        },
        {
            "slow actuator",
            "The software commands normally, but the physical airbrake moves only 0.35 deployment-fraction per second.",
            "PASS if full software command happens before full physical deployment, proving actuator lag is visible in the sim.",
            FlightExpectation::SlowActuatorLagVisible,
            118.0F,
            1.70F,
            3.0F,
            16.0F,
            0.0F,
            0.20F,
            0.75F,
            false,
            0.0F,
            0.0F,
            0.35F
        },
        {
            "weak motor",
            "Lower thrust and shorter burn produce a flight that should not need airbrake deployment.",
            "PASS if airbrake command stays near 0%, actual deployment stays near 0%, and the phase remains Coast.",
            FlightExpectation::WeakMotorStaysInhibited,
            78.0F,
            1.55F,
            3.0F,
            16.0F,
            0.0F,
            0.20F,
            0.75F,
            false,
            0.0F,
            0.0F,
            2.0F
        }
    };

    std::cout << "AMBAR flight-control sandbox\n";
    std::cout << "Purpose: verify how the estimator and airbrake command logic react"
              << " to known virtual flight conditions.\n";
    std::cout << "PASS/FAIL meaning: a PASS means the software reacted as expected"
              << " for that condition. It does not mean the rocket is flight-proven.\n";

    std::vector<FlightResult> results;
    results.reserve(scenarios.size());

    int index = 1;
    for (const FlightScenario& scenario : scenarios) {
        FlightResult result = runScenario(scenario);
        printDetailedResult(index, scenario, result);
        results.push_back(result);
        ++index;
    }

    std::cout << "\nSUMMARY\n";
    std::cout << std::left << std::setw(24) << "scenario"
              << std::setw(8) << "result"
              << std::right << std::setw(10) << "apogee_ft"
              << std::setw(12) << "pred_ft"
              << std::setw(10) << "cmd_%"
              << std::setw(10) << "act_%"
              << std::setw(10) << "baro_rej"
              << std::setw(17) << "phase"
              << "\n";
    std::cout << std::string(101, '-') << "\n";

    bool allScenariosPassed = true;
    for (std::size_t row = 0; row < scenarios.size(); ++row) {
        printSummaryRow(scenarios[row], results[row]);
        allScenariosPassed = allScenariosPassed
            && evaluateScenario(scenarios[row], results[row]).pass;
    }

    std::cout << "\nNext calibration data needed: real drag curve, motor thrust,"
              << " venting, IMU alignment, and actuator timing from the final mechanism.\n";

    // The shell runner and CI can only enforce the printed verdicts if the
    // process exit code also reports a failed scenario.
    return allScenariosPassed ? 0 : 1;
}
