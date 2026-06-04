#include "ambar_airbrake.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {

using ambar::Scalar;

struct ActuatorConfig {
    Scalar travel_mm = 20.0F;
    Scalar stepsPerMm = 400.0F;
    Scalar maxStepRate_stepsPerS = 9000.0F;
    Scalar nominalCurrent_mA = 420.0F;
    Scalar stallCurrent_mA = 1200.0F;
};

struct ActuatorScenario {
    std::string name;
    bool startsHomed = true;
    bool injectJam = false;
    Scalar jamAtFraction = 0.5F;
    Scalar maxStepRateScale = 1.0F;
};

struct ActuatorResult {
    std::string name;
    Scalar peakDeployFraction = 0.0F;
    Scalar finalDeployFraction = 0.0F;
    Scalar peakCurrent_mA = 0.0F;
    bool homed = false;
    bool fault = false;
    std::string faultReason;
};

class VirtualAirbrakeActuator {
public:
    VirtualAirbrakeActuator(const ActuatorConfig& config,
                            const ActuatorScenario& scenario)
        : config_(config),
          scenario_(scenario),
          homed_(scenario.startsHomed)
    {
    }

    void update(const ambar::AirbrakeCommand& command, Scalar dt_s) {
        if (!homed_) {
            targetSteps_ = 0.0F;
            if (!command.inhibit && command.deployFraction > 0.0F) {
                fault_ = true;
                faultReason_ = "deploy command before homing";
            }
        } else {
            const Scalar safeCommand =
                command.inhibit ? 0.0F : command.deployFraction;
            targetSteps_ = safeCommand * maxTravelSteps();
        }

        if (fault_) {
            current_mA_ = config_.stallCurrent_mA;
            return;
        }

        const Scalar maxStepDelta =
            config_.maxStepRate_stepsPerS * scenario_.maxStepRateScale * dt_s;
        const Scalar requestedDelta = targetSteps_ - positionSteps_;
        const Scalar stepDelta = std::max(-maxStepDelta,
                                          std::min(requestedDelta, maxStepDelta));

        positionSteps_ += stepDelta;
        positionSteps_ = std::max(0.0F, std::min(positionSteps_, maxTravelSteps()));

        if (scenario_.injectJam
            && deploymentFraction() >= scenario_.jamAtFraction
            && targetSteps_ > positionSteps_) {
            fault_ = true;
            faultReason_ = "simulated jam / stall";
            current_mA_ = config_.stallCurrent_mA;
            return;
        }

        const Scalar motionLoad = std::min(std::fabs(stepDelta) / maxStepDelta, 1.0F);
        current_mA_ = config_.nominalCurrent_mA * (0.35F + 0.65F * motionLoad);
    }

    Scalar deploymentFraction() const {
        return positionSteps_ / maxTravelSteps();
    }

    Scalar current_mA() const {
        return current_mA_;
    }

    bool homed() const {
        return homed_;
    }

    bool fault() const {
        return fault_;
    }

    const std::string& faultReason() const {
        return faultReason_;
    }

private:
    Scalar maxTravelSteps() const {
        return config_.travel_mm * config_.stepsPerMm;
    }

    ActuatorConfig config_{};
    ActuatorScenario scenario_{};
    Scalar positionSteps_ = 0.0F;
    Scalar targetSteps_ = 0.0F;
    Scalar current_mA_ = 0.0F;
    bool homed_ = false;
    bool fault_ = false;
    std::string faultReason_ = "none";
};

ambar::AirbrakeCommand commandAtTime(Scalar timestamp_s) {
    ambar::AirbrakeCommand command{};
    command.targetApogee_m = ambar::kTargetApogeeM;
    command.predictedApogee_m = ambar::kTargetApogeeM + 200.0F * ambar::kFeetToMeters;
    command.inhibitFlags = ambar::kInhibitNone;
    command.inhibit = false;

    if (timestamp_s < 0.50F) {
        command.inhibit = true;
        command.inhibitFlags = ambar::kInhibitNotInCoast;
        command.deployFraction = 0.0F;
    } else if (timestamp_s < 2.50F) {
        command.deployFraction = 1.0F;
    } else if (timestamp_s < 4.00F) {
        command.deployFraction = 0.35F;
    } else {
        command.inhibit = true;
        command.inhibitFlags = ambar::kInhibitDescending;
        command.deployFraction = 0.0F;
    }

    return command;
}

ActuatorResult runScenario(const ActuatorScenario& scenario) {
    const ActuatorConfig config{};
    VirtualAirbrakeActuator actuator(config, scenario);

    constexpr Scalar dt_s = 0.002F;
    constexpr Scalar endTime_s = 5.0F;
    const int sampleCount = static_cast<int>(endTime_s / dt_s);

    ActuatorResult result{};
    result.name = scenario.name;

    for (int sample = 0; sample <= sampleCount; ++sample) {
        const Scalar timestamp_s = sample * dt_s;
        actuator.update(commandAtTime(timestamp_s), dt_s);
        result.peakDeployFraction =
            std::max(result.peakDeployFraction, actuator.deploymentFraction());
        result.peakCurrent_mA = std::max(result.peakCurrent_mA, actuator.current_mA());
    }

    result.finalDeployFraction = actuator.deploymentFraction();
    result.homed = actuator.homed();
    result.fault = actuator.fault();
    result.faultReason = actuator.faultReason();
    return result;
}

void printResult(const ActuatorResult& result) {
    std::cout << std::left << std::setw(24) << result.name
              << std::right << std::setw(12) << std::fixed << std::setprecision(1)
              << result.peakDeployFraction * 100.0F
              << std::setw(12) << result.finalDeployFraction * 100.0F
              << std::setw(12) << result.peakCurrent_mA
              << std::setw(9) << (result.homed ? "yes" : "no")
              << std::setw(9) << (result.fault ? "yes" : "no")
              << "  " << result.faultReason
              << "\n";
}

} // namespace

int main() {
    const std::vector<ActuatorScenario> scenarios{
        {"nominal actuator", true, false, 0.0F, 1.0F},
        {"slow motor", true, false, 0.0F, 0.25F},
        {"not homed", false, false, 0.0F, 1.0F},
        {"jam at 45 percent", true, true, 0.45F, 1.0F}
    };

    std::cout << "AMBAR actuator sandbox\n";
    std::cout << "Virtual travel: 20 mm, 400 steps/mm. Replace these once the"
              << " real airbrake mechanism is finalized.\n\n";

    std::cout << std::left << std::setw(24) << "scenario"
              << std::right << std::setw(12) << "peak_%"
              << std::setw(12) << "final_%"
              << std::setw(12) << "peak_mA"
              << std::setw(9) << "homed"
              << std::setw(9) << "fault"
              << "  reason\n";
    std::cout << std::string(88, '-') << "\n";

    for (const ActuatorScenario& scenario : scenarios) {
        printResult(runScenario(scenario));
    }

    std::cout << "\nThis sandbox is for control-behavior exploration only. The"
              << " TMC5240 current limit, homing switch logic, and step/mm must"
              << " be updated from the real mechanism.\n";

    return 0;
}
