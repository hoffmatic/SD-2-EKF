#pragma once

#include "ambar_airbrake.hpp"

namespace ambar::requirements {

// These values are pulled from the Project AMBAR M3 report so simulations can
// check against project requirements instead of only checking local behavior.
inline constexpr Scalar kMaximumAllowedAltitude_m = 12500.0F * kFeetToMeters;
inline constexpr Scalar kMaximumGroundHitVelocity_mps = 35.0F * kFeetToMeters;
inline constexpr Scalar kTargetGroundHitVelocity_mps = 20.0F * kFeetToMeters;
inline constexpr Scalar kFlightComputerAccelerationRating_g = 30.0F;
inline constexpr Scalar kFlightComputerVelocityRecordLimit_mps = 1125.0F * kFeetToMeters;
inline constexpr Scalar kFlightComputerFootprintMaxLength_in = 3.25F;
inline constexpr Scalar kFlightComputerFootprintMaxWidth_in = 1.125F;
inline constexpr Scalar kMinimumGpsUpdateRate_hz = 5.0F;
inline constexpr Scalar kAirbrakeFullDeploymentLimit_s = 1.0F;
inline constexpr Scalar kAirbrakeControlResponseTarget_s = 0.5F;
inline constexpr Scalar kAirbrakeMaximumMass_kg = 5.0F * 0.45359237F;
inline constexpr Scalar kGroundStationMinimumRange_m = 5000.0F * kFeetToMeters;
inline constexpr Scalar kGroundStationReportFrequency_mhz = 915.0F;
inline constexpr Scalar kConceptFullDeployTravel_mm = 25.4F;
inline constexpr Scalar kConceptLeadScrewRotationsForFullDeploy = 1.5F;
inline constexpr Scalar kRequiredLogDuration_s = 2.0F * 60.0F * 60.0F;

} // namespace ambar::requirements
