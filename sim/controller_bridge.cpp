#include "ambar_airbrake.hpp"

// Architecture role: process boundary between Python trajectory physics and
// the real C++ flight logic. run_rocketpy_sim.py keeps this executable alive,
// sends RESET/STEP/QUIT messages, and receives STATE replies. Persistence is
// essential because the EKF and phase tracker must retain state between steps.

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

// Keep bridge output machine-readable and limited to values RocketPy needs.
// Human-readable test reporting remains in run_rocketpy_sim.py.
void writeOutput(const ambar::AmbarFlightComputerOutput& output) {
    std::cout << std::fixed << std::setprecision(6)
              << "STATE "
              << output.airbrakeCommand.deployFraction << ' '
              << (output.airbrakeCommand.inhibit ? 1 : 0) << ' '
              << output.airbrakeCommand.inhibitFlags << ' '
              << output.estimate.altitudeAgl_m << ' '
              << output.estimate.verticalVelocity_mps << ' '
              << output.estimate.predictedApogee_m << ' '
              << (output.estimate.healthy ? 1 : 0) << ' '
              << ambar::flightPhaseName(output.phase)
              << '\n'
              << std::flush;
}

} // namespace

int main(int argumentCount, char* arguments[]) {
    ambar::AmbarFlightComputerConfig config{};

    // RocketPy supplies mission and motor values that are intentionally
    // adjustable in trade studies. Passing them into the shared C++ config
    // ensures the UI changes controller behavior, not only report labels.
    for (int argumentIndex = 1; argumentIndex < argumentCount; argumentIndex += 2) {
        if (argumentIndex + 1 >= argumentCount) {
            std::cerr << "Missing value for " << arguments[argumentIndex] << ".\n";
            return 2;
        }

        const std::string option = arguments[argumentIndex];
        float value = 0.0F;
        try {
            value = std::stof(arguments[argumentIndex + 1]);
        } catch (...) {
            std::cerr << "Invalid numeric value for " << option << ".\n";
            return 2;
        }

        if (!std::isfinite(value)) {
            std::cerr << option << " must be finite.\n";
            return 2;
        }

        if (option == "--minimum-boost-time" && value >= 0.0F) {
            config.phase.minimumBoostTime_s = value;
        } else if (option == "--target-apogee-m" && value > 0.0F) {
            config.controller.targetApogee_m = value;
        } else if (option == "--target-tolerance-m" && value >= 0.0F) {
            config.controller.apogeeTolerance_m = value;
        } else {
            std::cerr << "Unknown option or invalid value: " << option << ".\n"
                      << "Usage: ambar_controller_bridge"
                      << " [--minimum-boost-time seconds]"
                      << " [--target-apogee-m meters]"
                      << " [--target-tolerance-m meters]\n";
            return 2;
        }
    }

    ambar::AmbarFlightComputer computer(config);
    computer.resetOnPad(0.0F);

    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream input(line);
        std::string command;
        input >> command;

        if (command == "RESET") {
            float timestamp_s = 0.0F;
            input >> timestamp_s;
            computer.resetOnPad(timestamp_s);
            writeOutput(computer.output());
            continue;
        }

        if (command == "STEP") {
            float timestamp_s = 0.0F;
            float acceleration_mps2 = 0.0F;
            float altitude_m = 0.0F;
            float altitudeStdDev_m = 1.5F;
            int useBarometer = 0;
            if (!(input >> timestamp_s >> acceleration_mps2 >> altitude_m
                        >> altitudeStdDev_m >> useBarometer)) {
                std::cout << "ERROR malformed STEP\n" << std::flush;
                continue;
            }

            auto output = computer.updateImu(timestamp_s, acceleration_mps2);
            if (useBarometer != 0) {
                output = computer.updateBarometer(altitude_m, altitudeStdDev_m);
            }
            writeOutput(output);
            continue;
        }

        if (command == "QUIT") {
            break;
        }

        std::cout << "ERROR unknown command\n" << std::flush;
    }

    return 0;
}
