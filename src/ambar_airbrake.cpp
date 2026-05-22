#include "ambar_airbrake.hpp"

#include <algorithm>
#include <cmath>

namespace ambar {

namespace {

Scalar ballisticApogee(Scalar altitude_m, Scalar verticalVelocity_mps) {
    if (verticalVelocity_mps <= 0.0F) {
        return altitude_m;
    }

    return altitude_m
         + (verticalVelocity_mps * verticalVelocity_mps)
             / (2.0F * kStandardGravityMps2);
}

} // namespace

VerticalEkf4State::VerticalEkf4State(const VerticalEkfConfig& config)
    : config_(config)
{
    covariance_ = {};
}

void VerticalEkf4State::resetOnPad(Scalar timestamp_s) {
    state_ = {};
    covariance_ = {};
    covariance_[kAltitude][kAltitude] = config_.initialAltitudeVariance_m2;
    covariance_[kVelocity][kVelocity] = config_.initialVelocityVariance_m2ps2;
    covariance_[kAccelBias][kAccelBias] = config_.initialAccelBiasVariance_m2ps4;
    covariance_[kBarometerBias][kBarometerBias] = config_.initialBarometerBiasVariance_m2;

    initialized_ = true;
    healthy_ = valueIsFinite(timestamp_s);
    hasLastImuTimestamp_ = healthy_;
    lastImuTimestamp_s_ = timestamp_s;
    lastVerticalAcceleration_mps2_ = 0.0F;
    rejectedImuSamples_ = 0U;
    rejectedBarometerSamples_ = 0U;
    lastBarometerInnovation_m_ = 0.0F;
    lastBarometerInnovationSigma_ = 0.0F;
}

bool VerticalEkf4State::propagateWithImuVerticalAcceleration(Scalar timestamp_s,
                                                             Scalar verticalAcceleration_mps2)
{
    if (!valueIsFinite(timestamp_s) || !valueIsFinite(verticalAcceleration_mps2)) {
        healthy_ = false;
        ++rejectedImuSamples_;
        return false;
    }

    if (!initialized_) {
        resetOnPad(timestamp_s);
        lastVerticalAcceleration_mps2_ = verticalAcceleration_mps2;
        return true;
    }

    if (!hasLastImuTimestamp_) {
        hasLastImuTimestamp_ = true;
        lastImuTimestamp_s_ = timestamp_s;
        return true;
    }

    const Scalar dt_s = timestamp_s - lastImuTimestamp_s_;
    lastImuTimestamp_s_ = timestamp_s;

    if (dt_s < config_.minDt_s || dt_s > config_.maxDt_s) {
        healthy_ = false;
        ++rejectedImuSamples_;
        return false;
    }

    const Scalar correctedAcceleration_mps2 =
        verticalAcceleration_mps2 - state_[kAccelBias];
    lastVerticalAcceleration_mps2_ = correctedAcceleration_mps2;

    state_[kAltitude] += state_[kVelocity] * dt_s
                       + 0.5F * correctedAcceleration_mps2 * dt_s * dt_s;
    state_[kVelocity] += correctedAcceleration_mps2 * dt_s;

    Matrix4 transition = makeIdentityMatrix();
    transition[kAltitude][kVelocity] = dt_s;
    transition[kAltitude][kAccelBias] = -0.5F * dt_s * dt_s;
    transition[kVelocity][kAccelBias] = -dt_s;

    const Scalar accelVariance =
        config_.accelNoiseStdDev_mps2 * config_.accelNoiseStdDev_mps2;

    Matrix4 processNoise{};
    processNoise[kAltitude][kAltitude] = 0.25F * dt_s * dt_s * dt_s * dt_s * accelVariance;
    processNoise[kAltitude][kVelocity] = 0.5F * dt_s * dt_s * dt_s * accelVariance;
    processNoise[kVelocity][kAltitude] = processNoise[kAltitude][kVelocity];
    processNoise[kVelocity][kVelocity] = dt_s * dt_s * accelVariance;
    processNoise[kAccelBias][kAccelBias] =
        config_.accelBiasRandomWalkStdDev_mps2_perRootS
      * config_.accelBiasRandomWalkStdDev_mps2_perRootS
      * dt_s;
    processNoise[kBarometerBias][kBarometerBias] =
        config_.barometerBiasRandomWalkStdDev_m_perRootS
      * config_.barometerBiasRandomWalkStdDev_m_perRootS
      * dt_s;

    covariance_ = add(
        multiply(multiply(transition, covariance_), transpose(transition)),
        processNoise
    );
    symmetrizeCovariance();
    healthy_ = true;
    return true;
}

bool VerticalEkf4State::updateWithBarometerAltitude(Scalar barometerAltitudeAgl_m,
                                                    Scalar barometerStdDev_m)
{
    if (!initialized_) {
        return false;
    }

    if (!valueIsFinite(barometerAltitudeAgl_m)
        || !valueIsFinite(barometerStdDev_m)
        || barometerStdDev_m <= 0.0F) {
        healthy_ = false;
        ++rejectedBarometerSamples_;
        return false;
    }

    const Scalar predictedMeasurement_m = state_[kAltitude] + state_[kBarometerBias];
    const Scalar innovation_m = barometerAltitudeAgl_m - predictedMeasurement_m;
    const Scalar measurementVariance_m2 = barometerStdDev_m * barometerStdDev_m;

    const Scalar innovationVariance =
        covariance_[kAltitude][kAltitude]
      + covariance_[kBarometerBias][kBarometerBias]
      + 2.0F * covariance_[kAltitude][kBarometerBias]
      + measurementVariance_m2;

    if (!valueIsFinite(innovationVariance) || innovationVariance <= 1.0e-6F) {
        healthy_ = false;
        ++rejectedBarometerSamples_;
        return false;
    }

    const Scalar innovationSigma = std::sqrt(innovationVariance);
    lastBarometerInnovation_m_ = innovation_m;
    lastBarometerInnovationSigma_ = innovationSigma;

    if (std::fabs(innovation_m)
        > config_.barometerInnovationGateSigma * innovationSigma) {
        ++rejectedBarometerSamples_;
        return false;
    }

    std::array<Scalar, 4> gain{};
    for (int row = 0; row < 4; ++row) {
        gain[row] = (covariance_[row][kAltitude]
                  + covariance_[row][kBarometerBias])
                  / innovationVariance;
        state_[row] += gain[row] * innovation_m;
    }

    Matrix4 measurement{};
    measurement[0][kAltitude] = 1.0F;
    measurement[0][kBarometerBias] = 1.0F;

    Matrix4 gainMeasurement{};
    for (int row = 0; row < 4; ++row) {
        gainMeasurement[row][kAltitude] = gain[row];
        gainMeasurement[row][kBarometerBias] = gain[row];
    }

    const Matrix4 identity = makeIdentityMatrix();
    const Matrix4 correction = subtract(identity, gainMeasurement);

    Matrix4 josephNoise{};
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            josephNoise[row][column] = gain[row] * measurementVariance_m2 * gain[column];
        }
    }

    covariance_ = add(
        multiply(multiply(correction, covariance_), transpose(correction)),
        josephNoise
    );
    symmetrizeCovariance();
    return true;
}

NavigationEstimate VerticalEkf4State::estimate() const {
    NavigationEstimate result{};
    result.altitudeAgl_m = state_[kAltitude];
    result.verticalVelocity_mps = state_[kVelocity];
    result.verticalAcceleration_mps2 = lastVerticalAcceleration_mps2_;
    result.predictedApogee_m = ballisticApogee(state_[kAltitude], state_[kVelocity]);
    result.altitudeVariance_m2 = covariance_[kAltitude][kAltitude];
    result.velocityVariance_m2ps2 = covariance_[kVelocity][kVelocity];
    result.barometerBias_m = state_[kBarometerBias];
    result.initialized = initialized_;
    result.healthy = healthy_;
    return result;
}

EstimatorHealth VerticalEkf4State::health() const {
    EstimatorHealth result{};
    result.initialized = initialized_;
    result.healthy = healthy_;
    result.rejectedImuSamples = rejectedImuSamples_;
    result.rejectedBarometerSamples = rejectedBarometerSamples_;
    result.lastBarometerInnovation_m = lastBarometerInnovation_m_;
    result.lastBarometerInnovationSigma = lastBarometerInnovationSigma_;
    return result;
}

VerticalEkf4State::Matrix4 VerticalEkf4State::makeIdentityMatrix() const {
    Matrix4 identity{};
    for (int index = 0; index < 4; ++index) {
        identity[index][index] = 1.0F;
    }
    return identity;
}

VerticalEkf4State::Matrix4 VerticalEkf4State::multiply(const Matrix4& left,
                                                       const Matrix4& right) const
{
    Matrix4 result{};
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            Scalar sum = 0.0F;
            for (int inner = 0; inner < 4; ++inner) {
                sum += left[row][inner] * right[inner][column];
            }
            result[row][column] = sum;
        }
    }
    return result;
}

VerticalEkf4State::Matrix4 VerticalEkf4State::transpose(const Matrix4& matrix) const {
    Matrix4 result{};
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            result[row][column] = matrix[column][row];
        }
    }
    return result;
}

VerticalEkf4State::Matrix4 VerticalEkf4State::add(const Matrix4& left,
                                                  const Matrix4& right) const
{
    Matrix4 result{};
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            result[row][column] = left[row][column] + right[row][column];
        }
    }
    return result;
}

VerticalEkf4State::Matrix4 VerticalEkf4State::subtract(const Matrix4& left,
                                                       const Matrix4& right) const
{
    Matrix4 result{};
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            result[row][column] = left[row][column] - right[row][column];
        }
    }
    return result;
}

void VerticalEkf4State::symmetrizeCovariance() {
    for (int row = 0; row < 4; ++row) {
        for (int column = row + 1; column < 4; ++column) {
            const Scalar average = 0.5F * (covariance_[row][column]
                                         + covariance_[column][row]);
            covariance_[row][column] = average;
            covariance_[column][row] = average;
        }
    }
}

bool VerticalEkf4State::valueIsFinite(Scalar value) const {
    return std::isfinite(value);
}

FlightPhaseTracker::FlightPhaseTracker(const FlightPhaseConfig& config)
    : config_(config)
{
}

void FlightPhaseTracker::reset() {
    phase_ = FlightPhase::PadIdle;
    launchTimestamp_s_ = 0.0F;
    hasLaunchTimestamp_ = false;
}

FlightPhase FlightPhaseTracker::update(Scalar timestamp_s,
                                       const NavigationEstimate& estimate,
                                       Scalar verticalAcceleration_mps2)
{
    if (!estimate.healthy) {
        phase_ = FlightPhase::Fault;
        return phase_;
    }

    switch (phase_) {
    case FlightPhase::PadIdle:
        if (estimate.altitudeAgl_m > config_.liftoffAltitude_m
            || verticalAcceleration_mps2 > config_.liftoffAcceleration_mps2) {
            phase_ = FlightPhase::Boost;
            launchTimestamp_s_ = timestamp_s;
            hasLaunchTimestamp_ = true;
        }
        break;

    case FlightPhase::Boost:
        if (hasLaunchTimestamp_
            && timestamp_s - launchTimestamp_s_ >= config_.minimumBoostTime_s
            && verticalAcceleration_mps2 < config_.burnoutAcceleration_mps2
            && estimate.verticalVelocity_mps > config_.minimumCoastVelocity_mps) {
            phase_ = FlightPhase::Coast;
        }
        break;

    case FlightPhase::Coast:
    case FlightPhase::AirbrakeActive:
        if (estimate.verticalVelocity_mps <= config_.recoveryDescentVelocity_mps) {
            phase_ = FlightPhase::Recovery;
        }
        break;

    case FlightPhase::Recovery:
    case FlightPhase::Fault:
        break;
    }

    return phase_;
}

void FlightPhaseTracker::markAirbrakeActive() {
    if (phase_ == FlightPhase::Coast) {
        phase_ = FlightPhase::AirbrakeActive;
    }
}

FlightPhase FlightPhaseTracker::phase() const {
    return phase_;
}

Scalar FlightPhaseTracker::launchTimestamp_s() const {
    return launchTimestamp_s_;
}

AirbrakeController::AirbrakeController(const AirbrakeControllerConfig& config)
    : config_(config)
{
}

AirbrakeCommand AirbrakeController::computeCommand(const NavigationEstimate& estimate,
                                                   FlightPhase phase,
                                                   Scalar timestamp_s,
                                                   Scalar launchTimestamp_s) const
{
    AirbrakeCommand command{};
    command.targetApogee_m = config_.targetApogee_m;
    command.predictedApogee_m = estimate.predictedApogee_m;
    command.inhibitFlags = kInhibitNone;

    if (!estimate.healthy) {
        command.inhibitFlags |= kInhibitEstimatorUnhealthy;
    }

    if (phase != FlightPhase::Coast && phase != FlightPhase::AirbrakeActive) {
        command.inhibitFlags |= kInhibitNotInCoast;
    }

    if (estimate.altitudeAgl_m < config_.minimumDeployAltitude_m) {
        command.inhibitFlags |= kInhibitBelowMinimumAltitude;
    }

    if (timestamp_s - launchTimestamp_s < config_.minimumFlightTime_s) {
        command.inhibitFlags |= kInhibitBeforeMinimumFlightTime;
    }

    if (estimate.verticalVelocity_mps <= 0.0F) {
        command.inhibitFlags |= kInhibitDescending;
    }

    const Scalar apogeeError_m = estimate.predictedApogee_m - config_.targetApogee_m;
    if (apogeeError_m <= config_.apogeeTolerance_m) {
        command.inhibitFlags |= kInhibitApogeeOnTarget;
    }

    command.inhibit = command.inhibitFlags != kInhibitNone;
    if (command.inhibit) {
        command.deployFraction = 0.0F;
        return command;
    }

    command.deployFraction = clamp(
        apogeeError_m / config_.fullDeploymentError_m,
        0.0F,
        config_.maximumDeployFraction
    );
    return command;
}

Scalar AirbrakeController::clamp(Scalar value, Scalar lower, Scalar upper) const {
    return std::max(lower, std::min(value, upper));
}

AmbarFlightComputer::AmbarFlightComputer(const AmbarFlightComputerConfig& config)
    : estimator_(config.estimator),
      phase_(config.phase),
      controller_(config.controller)
{
}

void AmbarFlightComputer::resetOnPad(Scalar timestamp_s) {
    estimator_.resetOnPad(timestamp_s);
    phase_.reset();
    latestTimestamp_s_ = timestamp_s;
}

AmbarFlightComputerOutput AmbarFlightComputer::updateImu(Scalar timestamp_s,
                                                         Scalar verticalAcceleration_mps2)
{
    latestTimestamp_s_ = timestamp_s;
    estimator_.propagateWithImuVerticalAcceleration(timestamp_s, verticalAcceleration_mps2);
    phase_.update(timestamp_s, estimator_.estimate(), verticalAcceleration_mps2);

    AmbarFlightComputerOutput result = buildOutput();
    if (!result.airbrakeCommand.inhibit && result.airbrakeCommand.deployFraction > 0.0F) {
        phase_.markAirbrakeActive();
        result = buildOutput();
    }
    return result;
}

AmbarFlightComputerOutput AmbarFlightComputer::updateBarometer(Scalar barometerAltitudeAgl_m,
                                                               Scalar barometerStdDev_m)
{
    estimator_.updateWithBarometerAltitude(barometerAltitudeAgl_m, barometerStdDev_m);
    return buildOutput();
}

AmbarFlightComputerOutput AmbarFlightComputer::output() const {
    return buildOutput();
}

AmbarFlightComputerOutput AmbarFlightComputer::buildOutput() const {
    AmbarFlightComputerOutput result{};
    result.estimate = estimator_.estimate();
    result.health = estimator_.health();
    result.phase = phase_.phase();
    result.airbrakeCommand = controller_.computeCommand(
        result.estimate,
        result.phase,
        latestTimestamp_s_,
        phase_.launchTimestamp_s()
    );
    return result;
}

const char* flightPhaseName(FlightPhase phase) {
    switch (phase) {
    case FlightPhase::PadIdle:
        return "PadIdle";
    case FlightPhase::Boost:
        return "Boost";
    case FlightPhase::Coast:
        return "Coast";
    case FlightPhase::AirbrakeActive:
        return "AirbrakeActive";
    case FlightPhase::Recovery:
        return "Recovery";
    case FlightPhase::Fault:
        return "Fault";
    }
    return "Unknown";
}

} // namespace ambar
