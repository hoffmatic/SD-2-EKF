#pragma once

// Architecture role: mission requirements expressed as reusable constants.
// Simulation suites compare behavior against these values, keeping report
// limits out of individual tests and away from hardware-specific pin data.

#include "ambar_airbrake.hpp"

namespace ambar::requirements {

// These values are pulled from the Project AMBAR M5 report so simulations can
// check against project requirements instead of only checking local behavior.
inline constexpr Scalar kMaximumAllowedAltitude_m = 12500.0F * kFeetToMeters;
inline constexpr Scalar kMaximumGroundHitVelocity_mps = 35.0F * kFeetToMeters;
inline constexpr Scalar kTargetGroundHitVelocity_mps = 20.0F * kFeetToMeters;
inline constexpr Scalar kFlightComputerAccelerationRating_g = 30.0F;
inline constexpr Scalar kFlightComputerVelocityRecordLimit_mps = 1125.0F * kFeetToMeters;
inline constexpr Scalar kFlightComputerFootprintMaxLength_in = 3.25F;
inline constexpr Scalar kFlightComputerFootprintMaxWidth_in = 1.125F;
inline constexpr Scalar kAirbrakeFullDeploymentLimit_s = 1.0F;
inline constexpr Scalar kAirbrakeControlResponseTarget_s = 0.5F;
inline constexpr Scalar kAirbrakeMaximumMass_kg = 5.0F * 0.45359237F;
inline constexpr Scalar kGroundStationMinimumRange_m = 5000.0F * kFeetToMeters;
inline constexpr Scalar kAirbrakeRadioFrequency_mhz = 2400.0F;
inline constexpr bool kAirbrakeComputerUsesMagnetometer = true;
inline constexpr bool kRecoverySystemRequiresIndependentGps = true;
inline constexpr Scalar kConceptFullDeployTravel_mm = 25.4F;
inline constexpr Scalar kConceptLeadScrewRotationsForFullDeploy = 1.5F;
inline constexpr Scalar kRequiredLogDuration_s = 2.0F * 60.0F * 60.0F;

// June 14 M5 Critical Design Evaluation reference values. These are useful
// comparison points, not substitutes for measured flight-ready properties.
inline constexpr Scalar kM5OpenRocketPassiveApogee_m = 3379.0F * kFeetToMeters;
inline constexpr Scalar kM5ExpectedMaximumVelocity_mps = 579.0F * kFeetToMeters;
inline constexpr Scalar kM5OpenRocketMaximumMach = 0.509F;
inline constexpr Scalar kM5OpenRocketRailExitVelocity_mps = 75.5F * kFeetToMeters;
inline constexpr Scalar kM5AverageMorningWind_mps = 8.0F * 0.44704F;
inline constexpr Scalar kM5AirbrakeWorstCaseForcePerFin_N = 54.8215F;
inline constexpr Scalar kM5AirbrakeWorstCaseTotalForce_N = 219.286F;
inline constexpr Scalar kM5AirbrakeFinArea_m2 = 0.001935F;
inline constexpr Scalar kM5AirbrakeFinDeflectionLimit_m = 0.018F * 0.0254F;
inline constexpr Scalar kM5EstimatedPeakLogicCurrent_mA = 430.0F;

} // namespace ambar::requirements
