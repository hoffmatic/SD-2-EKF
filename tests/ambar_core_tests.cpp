#include "ambar_airbrake.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {

int failures = 0;

void expect(bool condition, const std::string& name) {
    std::cout << (condition ? "[PASS] " : "[FAIL] ") << name << "\n";
    if (!condition) {
        ++failures;
    }
}

void testEstimatorTimingRecovery() {
    ambar::VerticalEkf4State estimator;
    estimator.resetOnPad(0.0F);

    expect(estimator.propagateWithImuVerticalAcceleration(0.01F, 0.0F),
           "valid IMU timestamp is accepted");
    expect(!estimator.propagateWithImuVerticalAcceleration(0.01F, 0.0F),
           "duplicate IMU timestamp is rejected");
    expect(estimator.health().rejectedImuSamples == 1U,
           "duplicate timestamp increments the rejection counter");
    expect(estimator.propagateWithImuVerticalAcceleration(0.02F, 0.0F),
           "estimator accepts the next valid timestamp");
    expect(estimator.health().healthy,
           "estimator health recovers after a valid sample");
}

void testNonFiniteSensorRejection() {
    ambar::VerticalEkf4State estimator;
    estimator.resetOnPad(0.0F);
    const auto nan = std::numeric_limits<ambar::Scalar>::quiet_NaN();

    expect(!estimator.propagateWithImuVerticalAcceleration(0.01F, nan),
           "NaN acceleration is rejected");
    expect(!estimator.updateWithBarometerAltitude(nan, 1.0F),
           "NaN barometer altitude is rejected");
    expect(estimator.health().rejectedImuSamples == 1U,
           "invalid IMU counter is reported");
    expect(estimator.health().rejectedBarometerSamples == 1U,
           "invalid barometer counter is reported");
}

void testBarometerInnovationGate() {
    ambar::VerticalEkf4State estimator;
    estimator.resetOnPad(0.0F);
    estimator.propagateWithImuVerticalAcceleration(0.01F, 0.0F);

    expect(!estimator.updateWithBarometerAltitude(1000.0F, 1.0F),
           "large barometer innovation is rejected");
    expect(estimator.health().rejectedBarometerSamples == 1U,
           "barometer innovation rejection is counted");
    expect(estimator.health().healthy,
           "an isolated gated barometer sample does not poison IMU propagation");
}

void testControllerInterlocks() {
    ambar::AirbrakeController controller;
    ambar::NavigationEstimate estimate{};
    estimate.initialized = true;
    estimate.healthy = true;
    estimate.altitudeAgl_m = 150.0F;
    estimate.verticalVelocity_mps = 40.0F;
    estimate.predictedApogee_m = ambar::kTargetApogeeM + 150.0F;

    const auto active = controller.computeCommand(
        estimate, ambar::FlightPhase::Coast, 2.0F, 0.0F);
    expect(!active.inhibit && active.deployFraction > 0.0F,
           "healthy coast estimate above target permits deployment");

    const auto boost = controller.computeCommand(
        estimate, ambar::FlightPhase::Boost, 2.0F, 0.0F);
    expect(boost.inhibit && (boost.inhibitFlags & ambar::kInhibitNotInCoast),
           "boost phase inhibits deployment");

    estimate.verticalVelocity_mps = -1.0F;
    const auto descending = controller.computeCommand(
        estimate, ambar::FlightPhase::Coast, 2.0F, 0.0F);
    expect(descending.inhibit
               && (descending.inhibitFlags & ambar::kInhibitDescending),
           "descending estimate inhibits deployment");
}

void testFaultLatchAndReset() {
    ambar::FlightPhaseTracker tracker;
    ambar::NavigationEstimate estimate{};
    estimate.healthy = false;

    expect(tracker.update(0.1F, estimate, 0.0F) == ambar::FlightPhase::Fault,
           "unhealthy navigation enters Fault");
    estimate.healthy = true;
    expect(tracker.update(0.2F, estimate, 0.0F) == ambar::FlightPhase::Fault,
           "Fault remains latched during a flight");
    tracker.reset();
    expect(tracker.phase() == ambar::FlightPhase::PadIdle,
           "pad reset clears the latched flight fault");
}

void testBallisticPredictorContract() {
    ambar::VerticalEkf4State estimator;
    estimator.resetOnPad(0.0F);
    for (int sample = 1; sample <= 100; ++sample) {
        estimator.propagateWithImuVerticalAcceleration(
            static_cast<ambar::Scalar>(sample) * 0.01F, 10.0F);
    }
    const auto estimate = estimator.estimate();
    const auto expected = estimate.altitudeAgl_m
        + estimate.verticalVelocity_mps * estimate.verticalVelocity_mps
            / (2.0F * ambar::kStandardGravityMps2);
    expect(std::fabs(estimate.predictedApogee_m - expected) < 0.001F,
           "implemented apogee predictor remains explicitly ballistic");
}

} // namespace

int main() {
    std::cout << "AMBAR core unit tests\n";
    testEstimatorTimingRecovery();
    testNonFiniteSensorRejection();
    testBarometerInnovationGate();
    testControllerInterlocks();
    testFaultLatchAndReset();
    testBallisticPredictorContract();

    std::cout << "\nSUMMARY: " << (failures == 0 ? "PASS" : "FAIL")
              << " - " << failures << " failed assertion(s).\n";
    return failures == 0 ? 0 : 1;
}
