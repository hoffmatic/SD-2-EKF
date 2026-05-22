#pragma once

#include <array>
#include <cstdint>

namespace ambar {

using Scalar = float;

constexpr Scalar kFeetToMeters = 0.3048F;
constexpr Scalar kMetersToFeet = 1.0F / kFeetToMeters;
constexpr Scalar kStandardGravityMps2 = 9.80665F;
constexpr Scalar kTargetApogeeM = 3000.0F * kFeetToMeters;
constexpr Scalar kTargetToleranceM = 100.0F * kFeetToMeters;

enum class FlightPhase : std::uint8_t {
    PadIdle,
    Boost,
    Coast,
    AirbrakeActive,
    Recovery,
    Fault
};

enum InhibitFlags : std::uint32_t {
    kInhibitNone = 0U,
    kInhibitEstimatorUnhealthy = 1U << 0U,
    kInhibitNotInCoast = 1U << 1U,
    kInhibitBelowMinimumAltitude = 1U << 2U,
    kInhibitBeforeMinimumFlightTime = 1U << 3U,
    kInhibitDescending = 1U << 4U,
    kInhibitApogeeOnTarget = 1U << 5U
};

struct VerticalEkfConfig {
    Scalar minDt_s = 0.0005F;
    Scalar maxDt_s = 0.0500F;

    Scalar accelNoiseStdDev_mps2 = 3.0F;
    Scalar accelBiasRandomWalkStdDev_mps2_perRootS = 0.08F;
    Scalar barometerBiasRandomWalkStdDev_m_perRootS = 0.03F;

    Scalar initialAltitudeVariance_m2 = 4.0F;
    Scalar initialVelocityVariance_m2ps2 = 25.0F;
    Scalar initialAccelBiasVariance_m2ps4 = 4.0F;
    Scalar initialBarometerBiasVariance_m2 = 4.0F;

    Scalar barometerInnovationGateSigma = 5.0F;
};

struct NavigationEstimate {
    Scalar altitudeAgl_m = 0.0F;
    Scalar verticalVelocity_mps = 0.0F;
    Scalar verticalAcceleration_mps2 = 0.0F;
    Scalar predictedApogee_m = 0.0F;
    Scalar altitudeVariance_m2 = 0.0F;
    Scalar velocityVariance_m2ps2 = 0.0F;
    Scalar barometerBias_m = 0.0F;
    bool initialized = false;
    bool healthy = false;
};

struct EstimatorHealth {
    bool initialized = false;
    bool healthy = false;
    std::uint32_t rejectedImuSamples = 0U;
    std::uint32_t rejectedBarometerSamples = 0U;
    Scalar lastBarometerInnovation_m = 0.0F;
    Scalar lastBarometerInnovationSigma = 0.0F;
};

class VerticalEkf4State {
public:
    explicit VerticalEkf4State(const VerticalEkfConfig& config = {});

    void resetOnPad(Scalar timestamp_s);

    bool propagateWithImuVerticalAcceleration(Scalar timestamp_s,
                                              Scalar verticalAcceleration_mps2);

    bool updateWithBarometerAltitude(Scalar barometerAltitudeAgl_m,
                                     Scalar barometerStdDev_m);

    NavigationEstimate estimate() const;
    EstimatorHealth health() const;

private:
    using StateVector = std::array<Scalar, 4>;
    using Matrix4 = std::array<std::array<Scalar, 4>, 4>;

    enum StateIndex : int {
        kAltitude = 0,
        kVelocity = 1,
        kAccelBias = 2,
        kBarometerBias = 3
    };

    Matrix4 makeIdentityMatrix() const;
    Matrix4 multiply(const Matrix4& left, const Matrix4& right) const;
    Matrix4 transpose(const Matrix4& matrix) const;
    Matrix4 add(const Matrix4& left, const Matrix4& right) const;
    Matrix4 subtract(const Matrix4& left, const Matrix4& right) const;
    void symmetrizeCovariance();
    bool valueIsFinite(Scalar value) const;

    VerticalEkfConfig config_{};
    StateVector state_{};
    Matrix4 covariance_{};

    bool initialized_ = false;
    bool healthy_ = false;
    bool hasLastImuTimestamp_ = false;
    Scalar lastImuTimestamp_s_ = 0.0F;
    Scalar lastVerticalAcceleration_mps2_ = 0.0F;
    std::uint32_t rejectedImuSamples_ = 0U;
    std::uint32_t rejectedBarometerSamples_ = 0U;
    Scalar lastBarometerInnovation_m_ = 0.0F;
    Scalar lastBarometerInnovationSigma_ = 0.0F;
};

struct FlightPhaseConfig {
    Scalar liftoffAltitude_m = 3.0F;
    Scalar liftoffAcceleration_mps2 = 15.0F;
    Scalar minimumBoostTime_s = 0.50F;
    Scalar burnoutAcceleration_mps2 = -2.0F;
    Scalar minimumCoastVelocity_mps = 20.0F;
    Scalar recoveryDescentVelocity_mps = -5.0F;
};

class FlightPhaseTracker {
public:
    explicit FlightPhaseTracker(const FlightPhaseConfig& config = {});

    void reset();
    FlightPhase update(Scalar timestamp_s,
                       const NavigationEstimate& estimate,
                       Scalar verticalAcceleration_mps2);
    void markAirbrakeActive();
    FlightPhase phase() const;
    Scalar launchTimestamp_s() const;

private:
    FlightPhaseConfig config_{};
    FlightPhase phase_ = FlightPhase::PadIdle;
    Scalar launchTimestamp_s_ = 0.0F;
    bool hasLaunchTimestamp_ = false;
};

struct AirbrakeControllerConfig {
    Scalar targetApogee_m = kTargetApogeeM;
    Scalar apogeeTolerance_m = kTargetToleranceM;
    Scalar fullDeploymentError_m = 250.0F * kFeetToMeters;
    Scalar maximumDeployFraction = 1.0F;
    Scalar minimumDeployAltitude_m = 200.0F * kFeetToMeters;
    Scalar minimumFlightTime_s = 1.0F;
};

struct AirbrakeCommand {
    Scalar deployFraction = 0.0F;
    Scalar predictedApogee_m = 0.0F;
    Scalar targetApogee_m = kTargetApogeeM;
    bool inhibit = true;
    std::uint32_t inhibitFlags = kInhibitEstimatorUnhealthy;
};

class AirbrakeController {
public:
    explicit AirbrakeController(const AirbrakeControllerConfig& config = {});

    AirbrakeCommand computeCommand(const NavigationEstimate& estimate,
                                   FlightPhase phase,
                                   Scalar timestamp_s,
                                   Scalar launchTimestamp_s) const;

private:
    Scalar clamp(Scalar value, Scalar lower, Scalar upper) const;

    AirbrakeControllerConfig config_{};
};

struct AmbarFlightComputerConfig {
    VerticalEkfConfig estimator{};
    FlightPhaseConfig phase{};
    AirbrakeControllerConfig controller{};
};

struct AmbarFlightComputerOutput {
    NavigationEstimate estimate{};
    EstimatorHealth health{};
    AirbrakeCommand airbrakeCommand{};
    FlightPhase phase = FlightPhase::PadIdle;
};

class AmbarFlightComputer {
public:
    explicit AmbarFlightComputer(const AmbarFlightComputerConfig& config = {});

    void resetOnPad(Scalar timestamp_s);
    AmbarFlightComputerOutput updateImu(Scalar timestamp_s,
                                        Scalar verticalAcceleration_mps2);
    AmbarFlightComputerOutput updateBarometer(Scalar barometerAltitudeAgl_m,
                                              Scalar barometerStdDev_m);
    AmbarFlightComputerOutput output() const;

private:
    AmbarFlightComputerOutput buildOutput() const;

    VerticalEkf4State estimator_;
    FlightPhaseTracker phase_;
    AirbrakeController controller_;
    Scalar latestTimestamp_s_ = 0.0F;
};

const char* flightPhaseName(FlightPhase phase);

} // namespace ambar
