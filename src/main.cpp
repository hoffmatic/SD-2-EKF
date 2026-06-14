#include "ambar_airbrake.hpp"
#include "ambar_board_pins.hpp"
#include "ambar_device_constants.hpp"

// Architecture role: small desktop smoke test and usage example. It connects
// synthetic sensor samples to AmbarFlightComputer and prints one final state.
// This is not the embedded firmware entry point and is not the high-fidelity
// trajectory model; use sim/rocketpy/run_rocketpy_sim.py for that use case.

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

// Compile-time sanity checks tie the demo build to the current June 1 hardware
// assumptions. If a future schematic changes an address or bus mode, this file
// should fail loudly until the constants are updated.
static_assert(ambar::devices::bmp388::kI2cAddress == 0x76);
static_assert(ambar::devices::lsm6dsv32x::kI2cAddress == 0x6A);
static_assert(ambar::devices::lis2mdl::kI2cAddress == 0x1E);
static_assert(ambar::devices::sx1280::kSpiMode == 0);
static_assert(ambar::devices::tmc5240::kSpiMode == 3);
static_assert(ambar::devices::w25q64jv::kTotalSizeBytes == 8UL * 1024UL * 1024UL);

ambar::Scalar simulatedAccelerationAtTime(ambar::Scalar timestamp_s,
                                          ambar::Scalar deployFraction)
{
    // Before motor ignition, the rocket is sitting still on the pad.
    if (timestamp_s < 0.20F) {
        return 0.0F;
    }

    // Simple boost segment: strong positive acceleration while the motor burns.
    if (timestamp_s < 1.80F) {
        return 95.0F;
    }

    // During coast, gravity and airbrake drag slow the vehicle. More deployment
    // means more drag in this toy model.
    const ambar::Scalar airbrakeDrag_mps2 = 35.0F * deployFraction;
    return -ambar::kStandardGravityMps2 - airbrakeDrag_mps2;
}

} // namespace

int main() {
    // This desktop demo exercises the estimator/controller without STM32
    // hardware. The eventual embedded scheduler will call the same class.
    ambar::AmbarFlightComputer flightComputer;
    flightComputer.resetOnPad(0.0F);

    // 500 Hz IMU simulation. Barometer is decimated below to mimic a slower
    // pressure sensor path.
    constexpr ambar::Scalar imuDt_s = 0.002F;
    constexpr int totalSamples = static_cast<int>(8.0F / imuDt_s);
    constexpr int barometerDecimation = 10;

    // "True" values are only used by the simulator to generate fake sensor
    // measurements. The flight computer never sees these directly.
    ambar::Scalar trueAltitude_m = 0.0F;
    ambar::Scalar trueVelocity_mps = 0.0F;
    ambar::Scalar deployFraction = 0.0F;
    ambar::Scalar peakDeployFraction = 0.0F;

    for (int sample = 1; sample <= totalSamples; ++sample) {
        const ambar::Scalar timestamp_s = sample * imuDt_s;
        const ambar::Scalar acceleration_mps2 =
            simulatedAccelerationAtTime(timestamp_s, deployFraction);

        // Integrate the toy physics so the barometer measurement has a plausible
        // altitude to report.
        trueVelocity_mps += acceleration_mps2 * imuDt_s;
        trueAltitude_m += trueVelocity_mps * imuDt_s;

        // Clamp the simulator to the ground after landing.
        if (trueAltitude_m < 0.0F) {
            trueAltitude_m = 0.0F;
            trueVelocity_mps = 0.0F;
        }

        // Feed the high-rate acceleration into the flight computer.
        ambar::AmbarFlightComputerOutput output =
            flightComputer.updateImu(timestamp_s, acceleration_mps2);

        if (sample % barometerDecimation == 0) {
            // Add small deterministic noise so the barometer update is not a
            // perfect copy of truth. Real pressure data will be noisier.
            const ambar::Scalar syntheticBarometerNoise_m =
                0.75F * std::sin(timestamp_s * 7.0F);
            output = flightComputer.updateBarometer(
                trueAltitude_m + syntheticBarometerNoise_m,
                1.5F
            );
        }

        // The next simulated acceleration step depends on the command computed
        // this step, approximating extra drag from deployed airbrakes.
        deployFraction = output.airbrakeCommand.deployFraction;
        peakDeployFraction = std::max(peakDeployFraction, deployFraction);
    }

    // Print the final snapshot so the command-line demo is easy to inspect.
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
