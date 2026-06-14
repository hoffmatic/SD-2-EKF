#include "ambar_airbrake.hpp"
#include "ambar_project_requirements.hpp"

// Architecture role: behavioral model of the mechanism that will consume
// AirbrakeCommand. It tests homing, travel rate, inhibit, current, and jam
// responses without pretending to be the final TMC5240 device driver.

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

using ambar::Scalar;

struct ActuatorConfig {
    Scalar travel_mm = ambar::requirements::kConceptFullDeployTravel_mm;
    Scalar stepsPerMm = 400.0F;
    Scalar maxStepRate_stepsPerS = 15000.0F;
    Scalar nominalCurrent_mA = 420.0F;
    Scalar stallCurrent_mA = 1200.0F;
};

enum class ActuatorExpectation {
    NominalMovesAndRetracts,
    SlowMotorShowsLimitedTravel,
    NotHomedBlocksDeployment,
    JamTriggersFault
};

struct ActuatorScenario {
    std::string name;
    std::string conditionUnderTest;
    std::string passRule;
    ActuatorExpectation expectation;
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
    Scalar firstMotionTime_s = -1.0F;
    Scalar firstFullDeployTime_s = -1.0F;
    bool homed = false;
    bool fault = false;
    std::string faultReason;
};

struct ActuatorVerdict {
    bool pass = false;
    std::string reason;
};

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

std::string secondsDuration(Scalar duration_s) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << duration_s << " s";
    return stream.str();
}

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

        const Scalar motionLoad =
            maxStepDelta > 0.0F
                ? std::min(std::fabs(stepDelta) / maxStepDelta, 1.0F)
                : 0.0F;
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
        const Scalar previousDeployFraction = actuator.deploymentFraction();

        actuator.update(commandAtTime(timestamp_s), dt_s);

        if (result.firstMotionTime_s < 0.0F
            && actuator.deploymentFraction() > previousDeployFraction + 0.0001F) {
            result.firstMotionTime_s = timestamp_s;
        }

        if (result.firstFullDeployTime_s < 0.0F
            && actuator.deploymentFraction() >= 0.95F) {
            result.firstFullDeployTime_s = timestamp_s;
        }

        result.peakDeployFraction =
            std::max(result.peakDeployFraction, actuator.deploymentFraction());
        result.peakCurrent_mA =
            std::max(result.peakCurrent_mA, actuator.current_mA());
    }

    result.finalDeployFraction = actuator.deploymentFraction();
    result.homed = actuator.homed();
    result.fault = actuator.fault();
    result.faultReason = actuator.faultReason();
    return result;
}

ActuatorVerdict evaluateScenario(const ActuatorScenario& scenario,
                                 const ActuatorResult& result)
{
    switch (scenario.expectation) {
    case ActuatorExpectation::NominalMovesAndRetracts: {
        const bool deploysWithinRequirement =
            result.firstMotionTime_s >= 0.0F
         && result.firstFullDeployTime_s >= 0.0F
         && result.firstFullDeployTime_s - result.firstMotionTime_s
            <= ambar::requirements::kAirbrakeFullDeploymentLimit_s;
        const bool pass = result.homed
                       && !result.fault
                       && result.peakDeployFraction >= 0.95F
                       && result.finalDeployFraction <= 0.05F
                       && deploysWithinRequirement;
        return {
            pass,
            pass
                ? "Homed actuator reached near-full deployment within the M5 1-second limit and returned near retracted without a fault."
                : "Nominal actuator did not deploy/retract cleanly, missed the 1-second limit, or produced a fault."
        };
    }

    case ActuatorExpectation::SlowMotorShowsLimitedTravel: {
        const bool pass = result.homed
                       && !result.fault
                       && result.peakDeployFraction < 0.75F
                       && result.firstFullDeployTime_s < 0.0F;
        return {
            pass,
            pass
                ? "Slower motor never reached full deployment during the command window, making lost travel visible."
                : "Slow motor unexpectedly reached full deployment or produced a fault."
        };
    }

    case ActuatorExpectation::NotHomedBlocksDeployment: {
        const bool pass = !result.homed
                       && result.fault
                       && result.peakDeployFraction <= 0.01F;
        return {
            pass,
            pass
                ? "Unhomed actuator refused deployment and raised the expected safety fault."
                : "Unhomed actuator moved or failed to raise a safety fault."
        };
    }

    case ActuatorExpectation::JamTriggersFault: {
        const bool pass = result.homed
                       && result.fault
                       && result.peakDeployFraction >= 0.40F
                       && result.peakDeployFraction <= 0.50F;
        return {
            pass,
            pass
                ? "Jam stopped travel near the injected point and produced a stall/fault condition."
                : "Jam behavior did not match the injected stall condition."
        };
    }
    }

    return {false, "Unknown expectation."};
}

void printDetailedResult(int index,
                         const ActuatorScenario& scenario,
                         const ActuatorResult& result)
{
    const ActuatorVerdict verdict = evaluateScenario(scenario, result);

    std::cout << "\nTEST CASE " << index << ": " << scenario.name << "\n";
    std::cout << "Condition being tested: " << scenario.conditionUnderTest << "\n";
    std::cout << "Pass rule: " << scenario.passRule << "\n";
    std::cout << "Result: " << passFail(verdict.pass) << " - " << verdict.reason << "\n";

    std::cout << "Measurements:\n";
    std::cout << "  starts homed:       " << yesNo(scenario.startsHomed) << "\n";
    std::cout << "  jam injected:       " << yesNo(scenario.injectJam) << "\n";
    std::cout << "  speed scale:        " << std::fixed << std::setprecision(2)
              << scenario.maxStepRateScale << "x nominal\n";
    std::cout << "  first motion time:  "
              << secondsOrNotReached(result.firstMotionTime_s) << "\n";
    std::cout << "  full deploy time:   "
              << secondsOrNotReached(result.firstFullDeployTime_s) << "\n";
    if (result.firstMotionTime_s >= 0.0F && result.firstFullDeployTime_s >= 0.0F) {
        std::cout << "  deploy duration:    "
                  << secondsDuration(result.firstFullDeployTime_s - result.firstMotionTime_s)
                  << " (limit "
                  << secondsDuration(ambar::requirements::kAirbrakeFullDeploymentLimit_s)
                  << ")\n";
    }
    std::cout << "  peak deployment:    " << std::setprecision(1)
              << result.peakDeployFraction * 100.0F << "%\n";
    std::cout << "  final deployment:   "
              << result.finalDeployFraction * 100.0F << "%\n";
    std::cout << "  peak current:       "
              << result.peakCurrent_mA << " mA\n";
    std::cout << "  final homed state:  " << yesNo(result.homed) << "\n";
    std::cout << "  fault raised:       " << yesNo(result.fault) << "\n";
    std::cout << "  fault reason:       " << result.faultReason << "\n";
}

void printSummaryRow(const ActuatorScenario& scenario,
                     const ActuatorResult& result)
{
    const ActuatorVerdict verdict = evaluateScenario(scenario, result);

    std::cout << std::left << std::setw(24) << scenario.name
              << std::setw(8) << passFail(verdict.pass)
              << std::right << std::setw(12) << std::fixed << std::setprecision(1)
              << result.peakDeployFraction * 100.0F
              << std::setw(12) << result.finalDeployFraction * 100.0F
              << std::setw(12) << result.peakCurrent_mA
              << std::setw(9) << yesNo(result.homed)
              << std::setw(9) << yesNo(result.fault)
              << "\n";
}

} // namespace

int main() {
    const std::vector<ActuatorScenario> scenarios{
        {
            "nominal actuator",
            "Homed actuator receives deploy, partial retract, then inhibit/retract commands.",
            "PASS if it deploys near 100% within 1 second, retracts near 0%, and never faults.",
            ActuatorExpectation::NominalMovesAndRetracts,
            true,
            false,
            0.0F,
            1.0F
        },
        {
            "slow motor",
            "The motor can only move at 25% of the placeholder nominal step rate.",
            "PASS if it cannot reach full deployment during the command window but does not fault.",
            ActuatorExpectation::SlowMotorShowsLimitedTravel,
            true,
            false,
            0.0F,
            0.25F
        },
        {
            "not homed",
            "The actuator has not found its zero position before a deploy command arrives.",
            "PASS if it refuses to move and raises a deploy-before-homing fault.",
            ActuatorExpectation::NotHomedBlocksDeployment,
            false,
            false,
            0.0F,
            1.0F
        },
        {
            "jam at 45 percent",
            "The virtual mechanism stalls when deployment reaches about 45%.",
            "PASS if travel stops near the jam point and the fault/current path is triggered.",
            ActuatorExpectation::JamTriggersFault,
            true,
            true,
            0.45F,
            1.0F
        }
    };

    const ActuatorConfig config{};

    std::cout << "AMBAR actuator sandbox\n";
    std::cout << "Purpose: verify how a future TMC5240 actuator layer should"
              << " react to motion, homing, and stall conditions.\n";
    std::cout << "Virtual mechanism constants: " << config.travel_mm << " mm travel, "
              << config.stepsPerMm << " steps/mm, "
              << config.maxStepRate_stepsPerS << " steps/s nominal max.\n";
    std::cout << "Source note: M5 report concept says full 90-degree deployment"
              << " takes about 1 inch of vertical travel and about "
              << ambar::requirements::kConceptLeadScrewRotationsForFullDeploy
              << " lead-screw rotations. Step/mm and current are still placeholders.\n";
    std::cout << "PASS/FAIL meaning: PASS means the virtual actuator behaved as"
              << " expected for the injected condition, not that the mechanism"
              << " values are final.\n";

    std::vector<ActuatorResult> results;
    results.reserve(scenarios.size());

    int index = 1;
    for (const ActuatorScenario& scenario : scenarios) {
        ActuatorResult result = runScenario(scenario);
        printDetailedResult(index, scenario, result);
        results.push_back(result);
        ++index;
    }

    std::cout << "\nSUMMARY\n";
    std::cout << std::left << std::setw(24) << "scenario"
              << std::setw(8) << "result"
              << std::right << std::setw(12) << "peak_%"
              << std::setw(12) << "final_%"
              << std::setw(12) << "peak_mA"
              << std::setw(9) << "homed"
              << std::setw(9) << "fault"
              << "\n";
    std::cout << std::string(88, '-') << "\n";

    for (std::size_t row = 0; row < scenarios.size(); ++row) {
        printSummaryRow(scenarios[row], results[row]);
    }

    std::cout << "\nNext real data needed: final airbrake travel, step/mm,"
              << " homing switch behavior, current limit, and stall threshold.\n";

    return 0;
}
