#include "ambar_airbrake.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

ambar::Scalar simulatedAccelerationAtTime(ambar::Scalar timestamp_s,
                                          ambar::Scalar deployFraction)
{
    if (timestamp_s < 0.20F) {
        return 0.0F;
    }

    if (timestamp_s < 1.80F) {
        return 95.0F;
    }

    const ambar::Scalar airbrakeDrag_mps2 = 35.0F * deployFraction;
    return -ambar::kStandardGravityMps2 - airbrakeDrag_mps2;
}

} // namespace

int main() {
    ambar::AmbarFlightComputer flightComputer;
    flightComputer.resetOnPad(0.0F);

    constexpr ambar::Scalar imuDt_s = 0.002F;
    constexpr int totalSamples = static_cast<int>(8.0F / imuDt_s);
    constexpr int barometerDecimation = 10;

    ambar::Scalar trueAltitude_m = 0.0F;
    ambar::Scalar trueVelocity_mps = 0.0F;
    ambar::Scalar deployFraction = 0.0F;
    ambar::Scalar peakDeployFraction = 0.0F;

    for (int sample = 1; sample <= totalSamples; ++sample) {
        const ambar::Scalar timestamp_s = sample * imuDt_s;
        const ambar::Scalar acceleration_mps2 =
            simulatedAccelerationAtTime(timestamp_s, deployFraction);

        trueVelocity_mps += acceleration_mps2 * imuDt_s;
        trueAltitude_m += trueVelocity_mps * imuDt_s;

        if (trueAltitude_m < 0.0F) {
            trueAltitude_m = 0.0F;
            trueVelocity_mps = 0.0F;
        }

        ambar::AmbarFlightComputerOutput output =
            flightComputer.updateImu(timestamp_s, acceleration_mps2);

        if (sample % barometerDecimation == 0) {
            const ambar::Scalar syntheticBarometerNoise_m =
                0.75F * std::sin(timestamp_s * 7.0F);
            output = flightComputer.updateBarometer(
                trueAltitude_m + syntheticBarometerNoise_m,
                1.5F
            );
        }

        deployFraction = output.airbrakeCommand.deployFraction;
        peakDeployFraction = std::max(peakDeployFraction, deployFraction);
    }

    const ambar::AmbarFlightComputerOutput output = flightComputer.output();

    std::cout << "AMBAR flight computer demo\n";
    std::cout << "Phase: " << ambar::flightPhaseName(output.phase) << "\n";
    std::cout << "Altitude estimate: "
              << output.estimate.altitudeAgl_m * ambar::kMetersToFeet
              << " ft\n";
    std::cout << "Vertical velocity estimate: "
              << output.estimate.verticalVelocity_mps * ambar::kMetersToFeet
              << " ft/s\n";
    std::cout << "Predicted apogee: "
              << output.estimate.predictedApogee_m * ambar::kMetersToFeet
              << " ft\n";
    std::cout << "Airbrake command: "
              << output.airbrakeCommand.deployFraction * 100.0F
              << "%\n";
    std::cout << "Peak airbrake command: "
              << peakDeployFraction * 100.0F
              << "%\n";
    std::cout << "Rejected barometer samples: "
              << output.health.rejectedBarometerSamples
              << "\n";

    return output.health.healthy ? 0 : 1;
}
