#pragma once

#include <array>
#include <cstdint>

namespace ambar {

// Use one scalar alias so the estimator can move between desktop simulation and
// embedded targets without changing every numeric type in the codebase.
using Scalar = float;

// Common unit conversions and mission-level targets. The controller works in SI
// units internally, even though the AMBAR requirement is written in feet.
constexpr Scalar kFeetToMeters = 0.3048F;
constexpr Scalar kMetersToFeet = 1.0F / kFeetToMeters;
constexpr Scalar kStandardGravityMps2 = 9.80665F;
constexpr Scalar kTargetApogeeM = 3000.0F * kFeetToMeters;
constexpr Scalar kTargetToleranceM = 100.0F * kFeetToMeters;

// The phase tracker is deliberately simple: it gates airbrake deployment based
// on broad flight states instead of letting the controller act at any time.
enum class FlightPhase : std::uint8_t {
    PadIdle,
    Boost,
    Coast,
    AirbrakeActive,
    Recovery,
    Fault
};

// Bit flags make it possible to explain why deployment is blocked. That is
// useful for telemetry, debugging, and preflight checks.
enum InhibitFlags : std::uint32_t {
    kInhibitNone = 0U,
    kInhibitEstimatorUnhealthy = 1U << 0U,
    kInhibitNotInCoast = 1U << 1U,
    kInhibitBelowMinimumAltitude = 1U << 2U,
    kInhibitBeforeMinimumFlightTime = 1U << 3U,
    kInhibitDescending = 1U << 4U,
    kInhibitApogeeOnTarget = 1U << 5U
};

// Tuning for the 4-state vertical EKF. These defaults are conservative starting
// values; real values should be tuned from bench data and flight logs.
struct VerticalEkfConfig {
    // Reject IMU samples with impossible timing so one bad timestamp cannot
    // explode the state prediction.
    Scalar minDt_s = 0.0005F;
    Scalar maxDt_s = 0.0500F;

    // Process noise describes how much unmodeled acceleration and bias drift we
    // allow the filter to absorb between measurements.
    Scalar accelNoiseStdDev_mps2 = 3.0F;
    Scalar accelBiasRandomWalkStdDev_mps2_perRootS = 0.08F;
    Scalar barometerBiasRandomWalkStdDev_m_perRootS = 0.03F;

    // Initial uncertainty is intentionally nonzero so early barometer updates
    // can pull the state into agreement with the real pad-zero altitude.
    Scalar initialAltitudeVariance_m2 = 4.0F;
    Scalar initialVelocityVariance_m2ps2 = 25.0F;
    Scalar initialAccelBiasVariance_m2ps4 = 4.0F;
    Scalar initialBarometerBiasVariance_m2 = 4.0F;

    // Innovation gating rejects barometer samples that are wildly inconsistent
    // with the current estimate, which helps with pressure spikes and bad reads.
    Scalar barometerInnovationGateSigma = 5.0F;
};

// The estimator exposes navigation values and uncertainty, but keeps the raw
// covariance private so the rest of the program cannot accidentally corrupt it.
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

// Health is separated from NavigationEstimate so telemetry can report both the
// current state and how trustworthy the estimator has been recently.
struct EstimatorHealth {
    bool initialized = false;
    bool healthy = false;
    std::uint32_t rejectedImuSamples = 0U;
    std::uint32_t rejectedBarometerSamples = 0U;
    Scalar lastBarometerInnovation_m = 0.0F;
    Scalar lastBarometerInnovationSigma = 0.0F;
};

// A small vertical EKF for airbrake control. It estimates altitude, velocity,
// accelerometer bias, and barometer bias, which are the minimum states needed
// for a vertical apogee controller before full 6-DOF attitude work is added.
class VerticalEkf4State {
public:
    explicit VerticalEkf4State(const VerticalEkfConfig& config = {});

    // Pad reset establishes altitude zero and clears accumulated health flags.
    void resetOnPad(Scalar timestamp_s);

    // IMU propagation runs at the fastest sensor rate. The input is already
    // expected to be launch-frame vertical acceleration with gravity removed.
    bool propagateWithImuVerticalAcceleration(Scalar timestamp_s,
                                              Scalar verticalAcceleration_mps2);

    // Barometer updates run slower than the IMU and correct both altitude and
    // barometer-bias drift.
    bool updateWithBarometerAltitude(Scalar barometerAltitudeAgl_m,
                                     Scalar barometerStdDev_m);

    NavigationEstimate estimate() const;
    EstimatorHealth health() const;

private:
    using StateVector = std::array<Scalar, 4>;
    using Matrix4 = std::array<std::array<Scalar, 4>, 4>;

    // State ordering is fixed here so matrix math and measurement updates use
    // names instead of magic indexes.
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

// Thresholds for converting continuous estimates into coarse flight phases.
struct FlightPhaseConfig {
    Scalar liftoffAltitude_m = 3.0F;
    Scalar liftoffAcceleration_mps2 = 15.0F;
    Scalar minimumBoostTime_s = 0.50F;
    Scalar burnoutAcceleration_mps2 = -2.0F;
    Scalar minimumCoastVelocity_mps = 20.0F;
    Scalar recoveryDescentVelocity_mps = -5.0F;
};

// Tracks the part of flight the vehicle appears to be in. The controller uses
// this as a safety interlock so airbrakes only deploy during coast.
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

// Parameters that turn predicted apogee error into a normalized deployment
// request. Mechanical limits stay in the actuator driver; this remains unitless.
struct AirbrakeControllerConfig {
    Scalar targetApogee_m = kTargetApogeeM;
    Scalar apogeeTolerance_m = kTargetToleranceM;
    Scalar fullDeploymentError_m = 250.0F * kFeetToMeters;
    Scalar maximumDeployFraction = 1.0F;
    Scalar minimumDeployAltitude_m = 200.0F * kFeetToMeters;
    Scalar minimumFlightTime_s = 1.0F;
};

// Command consumed by the future actuator layer. `deployFraction` is ignored
// when `inhibit` is true.
struct AirbrakeCommand {
    Scalar deployFraction = 0.0F;
    Scalar predictedApogee_m = 0.0F;
    Scalar targetApogee_m = kTargetApogeeM;
    bool inhibit = true;
    std::uint32_t inhibitFlags = kInhibitEstimatorUnhealthy;
};

// Computes the desired airbrake deployment. It does not know about motor steps,
// current limits, or homing; those details belong to the TMC5240 actuator layer.
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

// One top-level config object keeps test code and embedded bring-up code from
// having to configure each subsystem separately.
struct AmbarFlightComputerConfig {
    VerticalEkfConfig estimator{};
    FlightPhaseConfig phase{};
    AirbrakeControllerConfig controller{};
};

// A snapshot of everything the scheduler or telemetry layer needs after an
// update: estimate, health, command, and phase.
struct AmbarFlightComputerOutput {
    NavigationEstimate estimate{};
    EstimatorHealth health{};
    AirbrakeCommand airbrakeCommand{};
    FlightPhase phase = FlightPhase::PadIdle;
};

// Facade that wires the estimator, phase tracker, and controller together. This
// is the class the eventual STM32 scheduler should call from sensor tasks.
class AmbarFlightComputer {
public:
    explicit AmbarFlightComputer(const AmbarFlightComputerConfig& config = {});

    // Called during preflight after the rocket is powered and sitting still.
    void resetOnPad(Scalar timestamp_s);

    // High-rate path: propagate state, update flight phase, and compute the
    // latest airbrake command.
    AmbarFlightComputerOutput updateImu(Scalar timestamp_s,
                                        Scalar verticalAcceleration_mps2);

    // Lower-rate path: correct altitude/bias from pressure altitude.
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
