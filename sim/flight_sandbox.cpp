#include "ambar_airbrake.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
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

struct FlightScenario {
    std::string name;
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
    ambar::FlightPhase finalPhase = ambar::FlightPhase::PadIdle;
    std::uint32_t rejectedImuSamples = 0U;
    std::uint32_t rejectedBarometerSamples = 0U;
    bool healthy = false;
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
    result.finalPhase = output.phase;
    result.rejectedImuSamples = output.health.rejectedImuSamples;
    result.rejectedBarometerSamples = output.health.rejectedBarometerSamples;
    result.healthy = output.health.healthy;
    return result;
}

void printResult(const FlightResult& result) {
    const Scalar trueApogee_ft = result.truePeakAltitude_m * ambar::kMetersToFeet;
    const Scalar predictedApogee_ft = result.predictedApogee_m * ambar::kMetersToFeet;
    const Scalar targetError_ft = trueApogee_ft - 3000.0F;

    std::cout << std::left << std::setw(22) << result.name
              << std::right << std::setw(10) << std::fixed << std::setprecision(0)
              << trueApogee_ft
              << std::setw(11) << targetError_ft
              << std::setw(12) << predictedApogee_ft
              << std::setw(10) << std::setprecision(1)
              << result.peakCommandFraction * 100.0F
              << std::setw(10) << result.peakActualDeployFraction * 100.0F
              << std::setw(9) << result.rejectedBarometerSamples
              << std::setw(16) << ambar::flightPhaseName(result.finalPhase)
              << std::setw(9) << (result.healthy ? "yes" : "no")
              << "\n";
}

} // namespace

int main() {
    const std::vector<FlightScenario> scenarios{
        {
            "baseline",
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
    std::cout << "All scenarios run through the real AmbarFlightComputer class.\n\n";

    std::cout << std::left << std::setw(22) << "scenario"
              << std::right << std::setw(10) << "apogee_ft"
              << std::setw(11) << "err_ft"
              << std::setw(12) << "pred_ft"
              << std::setw(10) << "cmd_%"
              << std::setw(10) << "act_%"
              << std::setw(9) << "baro_rej"
              << std::setw(16) << "phase"
              << std::setw(9) << "healthy"
              << "\n";
    std::cout << std::string(109, '-') << "\n";

    for (const FlightScenario& scenario : scenarios) {
        printResult(runScenario(scenario));
    }

    std::cout << "\nRead this as a sandbox, not a flight prediction. Drag, motor thrust,"
              << " venting, and actuator constants still need real data.\n";

    return 0;
}
