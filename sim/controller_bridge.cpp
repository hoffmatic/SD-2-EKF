#include "ambar_airbrake.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

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

int main() {
    ambar::AmbarFlightComputer computer;
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
