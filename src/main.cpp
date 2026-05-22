#include <array>
#include <cmath>
#include <iostream>

struct NEDVector3 {
    double north{};
    double east{};
    double down{};
};

static inline NEDVector3 operator+(const NEDVector3& leftVector, const NEDVector3& rightVector) {
    return {
        leftVector.north + rightVector.north,
        leftVector.east + rightVector.east,
        leftVector.down + rightVector.down
    };
}

static inline NEDVector3 operator-(const NEDVector3& leftVector, const NEDVector3& rightVector) {
    return {
        leftVector.north - rightVector.north,
        leftVector.east - rightVector.east,
        leftVector.down - rightVector.down
    };
}

static inline NEDVector3 operator*(double scalarValue, const NEDVector3& vectorValue) {
    return {
        scalarValue * vectorValue.north,
        scalarValue * vectorValue.east,
        scalarValue * vectorValue.down
    };
}

static inline NEDVector3 operator*(const NEDVector3& vectorValue, double scalarValue) {
    return scalarValue * vectorValue;
}

static inline double dotProduct3(const NEDVector3& leftVector, const NEDVector3& rightVector) {
    return leftVector.north * rightVector.north
         + leftVector.east  * rightVector.east
         + leftVector.down  * rightVector.down;
}

struct BodyToNEDQuaternion {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

static inline BodyToNEDQuaternion multiplyQuaternions(const BodyToNEDQuaternion& firstQuaternion,
                                                      const BodyToNEDQuaternion& secondQuaternion)
{
    return {
        firstQuaternion.w * secondQuaternion.w
            - firstQuaternion.x * secondQuaternion.x
            - firstQuaternion.y * secondQuaternion.y
            - firstQuaternion.z * secondQuaternion.z,

        firstQuaternion.w * secondQuaternion.x
            + firstQuaternion.x * secondQuaternion.w
            + firstQuaternion.y * secondQuaternion.z
            - firstQuaternion.z * secondQuaternion.y,

        firstQuaternion.w * secondQuaternion.y
            - firstQuaternion.x * secondQuaternion.z
            + firstQuaternion.y * secondQuaternion.w
            + firstQuaternion.z * secondQuaternion.x,

        firstQuaternion.w * secondQuaternion.z
            + firstQuaternion.x * secondQuaternion.y
            - firstQuaternion.y * secondQuaternion.x
            + firstQuaternion.z * secondQuaternion.w
    };
}

static inline void normalizeQuaternion(BodyToNEDQuaternion& quaternionToNormalize) {
    double quaternionNorm = std::sqrt(
        quaternionToNormalize.w * quaternionToNormalize.w
      + quaternionToNormalize.x * quaternionToNormalize.x
      + quaternionToNormalize.y * quaternionToNormalize.y
      + quaternionToNormalize.z * quaternionToNormalize.z
    );

    if (quaternionNorm > 0.0) {
        quaternionToNormalize.w /= quaternionNorm;
        quaternionToNormalize.x /= quaternionNorm;
        quaternionToNormalize.y /= quaternionNorm;
        quaternionToNormalize.z /= quaternionNorm;
    }
}

static inline BodyToNEDQuaternion buildSmallRotationQuaternionFromBodyRates(
    const NEDVector3& bodyAngularRate_rps,
    double timeStep_s)
{
    double rotationMagnitude_rad = std::sqrt(dotProduct3(bodyAngularRate_rps, bodyAngularRate_rps)) * timeStep_s;

    if (rotationMagnitude_rad < 1e-12) {
        return {
            1.0,
            0.5 * bodyAngularRate_rps.north * timeStep_s,
            0.5 * bodyAngularRate_rps.east  * timeStep_s,
            0.5 * bodyAngularRate_rps.down  * timeStep_s
        };
    }

    double halfRotationAngle_rad = 0.5 * rotationMagnitude_rad;
    double sineScale = std::sin(halfRotationAngle_rad) / rotationMagnitude_rad;

    return {
        std::cos(halfRotationAngle_rad),
        sineScale * bodyAngularRate_rps.north * timeStep_s,
        sineScale * bodyAngularRate_rps.east  * timeStep_s,
        sineScale * bodyAngularRate_rps.down  * timeStep_s
    };
}

static inline NEDVector3 rotateBodyVectorIntoNedFrame(const BodyToNEDQuaternion& bodyToNedQuaternion,
                                                      const NEDVector3& bodyFrameVector)
{
    BodyToNEDQuaternion conjugateQuaternion{
        bodyToNedQuaternion.w,
        -bodyToNedQuaternion.x,
        -bodyToNedQuaternion.y,
        -bodyToNedQuaternion.z
    };

    BodyToNEDQuaternion bodyVectorAsQuaternion{
        0.0,
        bodyFrameVector.north,
        bodyFrameVector.east,
        bodyFrameVector.down
    };

    BodyToNEDQuaternion intermediateQuaternion = multiplyQuaternions(bodyToNedQuaternion, bodyVectorAsQuaternion);
    BodyToNEDQuaternion rotatedQuaternion = multiplyQuaternions(intermediateQuaternion, conjugateQuaternion);

    return {rotatedQuaternion.x, rotatedQuaternion.y, rotatedQuaternion.z};
}

static constexpr int kErrorStateSize = 16;
using ErrorStateVector16 = std::array<double, kErrorStateSize>;

struct Matrix16x16 {
    std::array<double, kErrorStateSize * kErrorStateSize> elements{};

    double& operator()(int rowIndex, int columnIndex) {
        return elements[rowIndex * kErrorStateSize + columnIndex];
    }

    double operator()(int rowIndex, int columnIndex) const {
        return elements[rowIndex * kErrorStateSize + columnIndex];
    }
};

static inline Matrix16x16 makeIdentityMatrix16() {
    Matrix16x16 identityMatrix;
    for (int diagonalIndex = 0; diagonalIndex < kErrorStateSize; ++diagonalIndex) {
        identityMatrix(diagonalIndex, diagonalIndex) = 1.0;
    }
    return identityMatrix;
}

static inline Matrix16x16 addMatrix16(const Matrix16x16& leftMatrix, const Matrix16x16& rightMatrix) {
    Matrix16x16 resultMatrix;
    for (int flatIndex = 0; flatIndex < kErrorStateSize * kErrorStateSize; ++flatIndex) {
        resultMatrix.elements[flatIndex] = leftMatrix.elements[flatIndex] + rightMatrix.elements[flatIndex];
    }
    return resultMatrix;
}

static inline Matrix16x16 subtractMatrix16(const Matrix16x16& leftMatrix, const Matrix16x16& rightMatrix) {
    Matrix16x16 resultMatrix;
    for (int flatIndex = 0; flatIndex < kErrorStateSize * kErrorStateSize; ++flatIndex) {
        resultMatrix.elements[flatIndex] = leftMatrix.elements[flatIndex] - rightMatrix.elements[flatIndex];
    }
    return resultMatrix;
}

static inline Matrix16x16 transposeMatrix16(const Matrix16x16& inputMatrix) {
    Matrix16x16 transposedMatrix;
    for (int rowIndex = 0; rowIndex < kErrorStateSize; ++rowIndex) {
        for (int columnIndex = 0; columnIndex < kErrorStateSize; ++columnIndex) {
            transposedMatrix(rowIndex, columnIndex) = inputMatrix(columnIndex, rowIndex);
        }
    }
    return transposedMatrix;
}

static inline Matrix16x16 multiplyMatrix16(const Matrix16x16& leftMatrix, const Matrix16x16& rightMatrix) {
    Matrix16x16 resultMatrix;
    for (int rowIndex = 0; rowIndex < kErrorStateSize; ++rowIndex) {
        for (int columnIndex = 0; columnIndex < kErrorStateSize; ++columnIndex) {
            double accumulatedSum = 0.0;
            for (int innerIndex = 0; innerIndex < kErrorStateSize; ++innerIndex) {
                accumulatedSum += leftMatrix(rowIndex, innerIndex) * rightMatrix(innerIndex, columnIndex);
            }
            resultMatrix(rowIndex, columnIndex) = accumulatedSum;
        }
    }
    return resultMatrix;
}

class ErrorStateEkfNed {
public:
    NEDVector3 nominalPosition_ned_m{0.0, 0.0, 0.0};
    NEDVector3 nominalVelocity_ned_mps{0.0, 0.0, 0.0};
    BodyToNEDQuaternion nominalAttitude_bodyToNed_q{};
    NEDVector3 estimatedAccelBias_body_mps2{0.0, 0.0, 0.0};
    NEDVector3 estimatedGyroBias_body_rps{0.0, 0.0, 0.0};
    double estimatedBarometerBias_m{0.0};

    Matrix16x16 errorCovariance_P = makeIdentityMatrix16();

    double accelWhiteNoiseStdDev_mps2_perRootHz = 0.8;
    double gyroWhiteNoiseStdDev_rps_perRootHz = 0.02;
    double accelBiasRandomWalkStdDev_mps2_perRootHz = 0.08;
    double gyroBiasRandomWalkStdDev_rps_perRootHz = 0.001;
    double barometerBiasRandomWalkStdDev_m_perRootHz = 0.05;

    NEDVector3 gravityVector_ned_mps2{0.0, 0.0, 9.80665};

    void propagateUsingImu(const NEDVector3& measuredSpecificForce_body_mps2,
                           const NEDVector3& measuredAngularRate_body_rps,
                           double timeStep_s)
    {
        NEDVector3 unbiasedAngularRate_body_rps{
            measuredAngularRate_body_rps.north - estimatedGyroBias_body_rps.north,
            measuredAngularRate_body_rps.east  - estimatedGyroBias_body_rps.east,
            measuredAngularRate_body_rps.down  - estimatedGyroBias_body_rps.down
        };

        NEDVector3 unbiasedSpecificForce_body_mps2{
            measuredSpecificForce_body_mps2.north - estimatedAccelBias_body_mps2.north,
            measuredSpecificForce_body_mps2.east  - estimatedAccelBias_body_mps2.east,
            measuredSpecificForce_body_mps2.down  - estimatedAccelBias_body_mps2.down
        };

        BodyToNEDQuaternion incrementalBodyRotation_q =
            buildSmallRotationQuaternionFromBodyRates(unbiasedAngularRate_body_rps, timeStep_s);

        nominalAttitude_bodyToNed_q = multiplyQuaternions(nominalAttitude_bodyToNed_q, incrementalBodyRotation_q);
        normalizeQuaternion(nominalAttitude_bodyToNed_q);

        NEDVector3 specificForce_ned_mps2 =
            rotateBodyVectorIntoNedFrame(nominalAttitude_bodyToNed_q, unbiasedSpecificForce_body_mps2);

        NEDVector3 translationalAcceleration_ned_mps2 = specificForce_ned_mps2 + gravityVector_ned_mps2;

        nominalVelocity_ned_mps = nominalVelocity_ned_mps + translationalAcceleration_ned_mps2 * timeStep_s;
        nominalPosition_ned_m   = nominalPosition_ned_m   + nominalVelocity_ned_mps * timeStep_s;

        Matrix16x16 stateTransition_F = makeIdentityMatrix16();

        stateTransition_F(0, 3) = timeStep_s;
        stateTransition_F(1, 4) = timeStep_s;
        stateTransition_F(2, 5) = timeStep_s;

        NEDVector3 specificForce_ned_noGravity_mps2 = specificForce_ned_mps2;

        stateTransition_F(3, 7) =  specificForce_ned_noGravity_mps2.down * timeStep_s;
        stateTransition_F(3, 8) = -specificForce_ned_noGravity_mps2.east * timeStep_s;

        stateTransition_F(4, 6) = -specificForce_ned_noGravity_mps2.down * timeStep_s;
        stateTransition_F(4, 8) =  specificForce_ned_noGravity_mps2.north * timeStep_s;

        stateTransition_F(5, 6) =  specificForce_ned_noGravity_mps2.east * timeStep_s;
        stateTransition_F(5, 7) = -specificForce_ned_noGravity_mps2.north * timeStep_s;

        NEDVector3 bodyXAxis_ned = rotateBodyVectorIntoNedFrame(nominalAttitude_bodyToNed_q, {1.0, 0.0, 0.0});
        NEDVector3 bodyYAxis_ned = rotateBodyVectorIntoNedFrame(nominalAttitude_bodyToNed_q, {0.0, 1.0, 0.0});
        NEDVector3 bodyZAxis_ned = rotateBodyVectorIntoNedFrame(nominalAttitude_bodyToNed_q, {0.0, 0.0, 1.0});

        stateTransition_F(3,  9) = -bodyXAxis_ned.north * timeStep_s;
        stateTransition_F(3, 10) = -bodyYAxis_ned.north * timeStep_s;
        stateTransition_F(3, 11) = -bodyZAxis_ned.north * timeStep_s;

        stateTransition_F(4,  9) = -bodyXAxis_ned.east * timeStep_s;
        stateTransition_F(4, 10) = -bodyYAxis_ned.east * timeStep_s;
        stateTransition_F(4, 11) = -bodyZAxis_ned.east * timeStep_s;

        stateTransition_F(5,  9) = -bodyXAxis_ned.down * timeStep_s;
        stateTransition_F(5, 10) = -bodyYAxis_ned.down * timeStep_s;
        stateTransition_F(5, 11) = -bodyZAxis_ned.down * timeStep_s;

        stateTransition_F(6, 12) = -timeStep_s;
        stateTransition_F(7, 13) = -timeStep_s;
        stateTransition_F(8, 14) = -timeStep_s;

        Matrix16x16 processNoise_Q{};
        auto setProcessNoiseDiagonal = [&](int stateIndex, double varianceValue) {
            processNoise_Q(stateIndex, stateIndex) = varianceValue;
        };

        double accelNoiseVariance =
            (accelWhiteNoiseStdDev_mps2_perRootHz * accelWhiteNoiseStdDev_mps2_perRootHz) * timeStep_s;
        setProcessNoiseDiagonal(3, accelNoiseVariance);
        setProcessNoiseDiagonal(4, accelNoiseVariance);
        setProcessNoiseDiagonal(5, accelNoiseVariance);

        double gyroNoiseVariance =
            (gyroWhiteNoiseStdDev_rps_perRootHz * gyroWhiteNoiseStdDev_rps_perRootHz) * timeStep_s;
        setProcessNoiseDiagonal(6, gyroNoiseVariance);
        setProcessNoiseDiagonal(7, gyroNoiseVariance);
        setProcessNoiseDiagonal(8, gyroNoiseVariance);

        double accelBiasRandomWalkVariance =
            (accelBiasRandomWalkStdDev_mps2_perRootHz * accelBiasRandomWalkStdDev_mps2_perRootHz) * timeStep_s;
        setProcessNoiseDiagonal(9,  accelBiasRandomWalkVariance);
        setProcessNoiseDiagonal(10, accelBiasRandomWalkVariance);
        setProcessNoiseDiagonal(11, accelBiasRandomWalkVariance);

        double gyroBiasRandomWalkVariance =
            (gyroBiasRandomWalkStdDev_rps_perRootHz * gyroBiasRandomWalkStdDev_rps_perRootHz) * timeStep_s;
        setProcessNoiseDiagonal(12, gyroBiasRandomWalkVariance);
        setProcessNoiseDiagonal(13, gyroBiasRandomWalkVariance);
        setProcessNoiseDiagonal(14, gyroBiasRandomWalkVariance);

        double barometerBiasRandomWalkVariance =
            (barometerBiasRandomWalkStdDev_m_perRootHz * barometerBiasRandomWalkStdDev_m_perRootHz) * timeStep_s;
        setProcessNoiseDiagonal(15, barometerBiasRandomWalkVariance);

        errorCovariance_P = addMatrix16(
            multiplyMatrix16(
                multiplyMatrix16(stateTransition_F, errorCovariance_P),
                transposeMatrix16(stateTransition_F)
            ),
            processNoise_Q
        );
    }

    void updateUsingGpsPositionVelocity(const NEDVector3& measuredGpsPosition_ned_m,
                                        const NEDVector3& measuredGpsVelocity_ned_mps,
                                        double gpsPositionStdDev_m,
                                        double gpsVelocityStdDev_mps)
    {
        double gpsInnovation_y[6] = {
            measuredGpsPosition_ned_m.north - nominalPosition_ned_m.north,
            measuredGpsPosition_ned_m.east  - nominalPosition_ned_m.east,
            measuredGpsPosition_ned_m.down  - nominalPosition_ned_m.down,
            measuredGpsVelocity_ned_mps.north - nominalVelocity_ned_mps.north,
            measuredGpsVelocity_ned_mps.east  - nominalVelocity_ned_mps.east,
            measuredGpsVelocity_ned_mps.down  - nominalVelocity_ned_mps.down
        };

        double gpsPositionVariance = gpsPositionStdDev_m * gpsPositionStdDev_m;
        double gpsVelocityVariance = gpsVelocityStdDev_mps * gpsVelocityStdDev_mps;

        double innovationCovariance_S[6][6]{};

        for (int rowIndex = 0; rowIndex < 3; ++rowIndex) {
            for (int columnIndex = 0; columnIndex < 3; ++columnIndex) {
                innovationCovariance_S[rowIndex][columnIndex] = errorCovariance_P(rowIndex, columnIndex);
                innovationCovariance_S[rowIndex][columnIndex + 3] = errorCovariance_P(rowIndex, columnIndex + 3);
                innovationCovariance_S[rowIndex + 3][columnIndex] = errorCovariance_P(rowIndex + 3, columnIndex);
                innovationCovariance_S[rowIndex + 3][columnIndex + 3] =
                    errorCovariance_P(rowIndex + 3, columnIndex + 3);
            }
        }

        for (int axisIndex = 0; axisIndex < 3; ++axisIndex) {
            innovationCovariance_S[axisIndex][axisIndex] += gpsPositionVariance;
        }
        for (int axisIndex = 3; axisIndex < 6; ++axisIndex) {
            innovationCovariance_S[axisIndex][axisIndex] += gpsVelocityVariance;
        }

        double augmentedMatrix_S_and_I[6][12]{};
        for (int rowIndex = 0; rowIndex < 6; ++rowIndex) {
            for (int columnIndex = 0; columnIndex < 6; ++columnIndex) {
                augmentedMatrix_S_and_I[rowIndex][columnIndex] = innovationCovariance_S[rowIndex][columnIndex];
                augmentedMatrix_S_and_I[rowIndex][columnIndex + 6] = (rowIndex == columnIndex) ? 1.0 : 0.0;
            }
        }

        for (int pivotRowIndex = 0; pivotRowIndex < 6; ++pivotRowIndex) {
            double pivotValue = augmentedMatrix_S_and_I[pivotRowIndex][pivotRowIndex];
            if (std::fabs(pivotValue) < 1e-12) {
                return;
            }

            double inversePivotValue = 1.0 / pivotValue;
            for (int columnIndex = 0; columnIndex < 12; ++columnIndex) {
                augmentedMatrix_S_and_I[pivotRowIndex][columnIndex] *= inversePivotValue;
            }

            for (int rowIndex = 0; rowIndex < 6; ++rowIndex) {
                if (rowIndex == pivotRowIndex) {
                    continue;
                }

                double eliminationFactor = augmentedMatrix_S_and_I[rowIndex][pivotRowIndex];
                for (int columnIndex = 0; columnIndex < 12; ++columnIndex) {
                    augmentedMatrix_S_and_I[rowIndex][columnIndex] -=
                        eliminationFactor * augmentedMatrix_S_and_I[pivotRowIndex][columnIndex];
                }
            }
        }

        double innovationCovarianceInverse_S_inv[6][6]{};
        for (int rowIndex = 0; rowIndex < 6; ++rowIndex) {
            for (int columnIndex = 0; columnIndex < 6; ++columnIndex) {
                innovationCovarianceInverse_S_inv[rowIndex][columnIndex] =
                    augmentedMatrix_S_and_I[rowIndex][columnIndex + 6];
            }
        }

        double kalmanGain_K[16][6]{};
        for (int stateRowIndex = 0; stateRowIndex < 16; ++stateRowIndex) {
            for (int measurementColumnIndex = 0; measurementColumnIndex < 6; ++measurementColumnIndex) {
                kalmanGain_K[stateRowIndex][measurementColumnIndex] =
                    errorCovariance_P(stateRowIndex, 0) * innovationCovarianceInverse_S_inv[0][measurementColumnIndex]
                  + errorCovariance_P(stateRowIndex, 1) * innovationCovarianceInverse_S_inv[1][measurementColumnIndex]
                  + errorCovariance_P(stateRowIndex, 2) * innovationCovarianceInverse_S_inv[2][measurementColumnIndex]
                  + errorCovariance_P(stateRowIndex, 3) * innovationCovarianceInverse_S_inv[3][measurementColumnIndex]
                  + errorCovariance_P(stateRowIndex, 4) * innovationCovarianceInverse_S_inv[4][measurementColumnIndex]
                  + errorCovariance_P(stateRowIndex, 5) * innovationCovarianceInverse_S_inv[5][measurementColumnIndex];
            }
        }

        ErrorStateVector16 estimatedErrorState_dx{};
        for (int stateRowIndex = 0; stateRowIndex < 16; ++stateRowIndex) {
            double accumulatedCorrection = 0.0;
            for (int measurementRowIndex = 0; measurementRowIndex < 6; ++measurementRowIndex) {
                accumulatedCorrection += kalmanGain_K[stateRowIndex][measurementRowIndex]
                                       * gpsInnovation_y[measurementRowIndex];
            }
            estimatedErrorState_dx[stateRowIndex] = accumulatedCorrection;
        }

        injectEstimatedErrorStateIntoNominalState(estimatedErrorState_dx);

        Matrix16x16 kalmanTimesMeasurementMatrix_KH{};
        for (int stateRowIndex = 0; stateRowIndex < 16; ++stateRowIndex) {
            kalmanTimesMeasurementMatrix_KH(stateRowIndex, 0) = kalmanGain_K[stateRowIndex][0];
            kalmanTimesMeasurementMatrix_KH(stateRowIndex, 1) = kalmanGain_K[stateRowIndex][1];
            kalmanTimesMeasurementMatrix_KH(stateRowIndex, 2) = kalmanGain_K[stateRowIndex][2];
            kalmanTimesMeasurementMatrix_KH(stateRowIndex, 3) = kalmanGain_K[stateRowIndex][3];
            kalmanTimesMeasurementMatrix_KH(stateRowIndex, 4) = kalmanGain_K[stateRowIndex][4];
            kalmanTimesMeasurementMatrix_KH(stateRowIndex, 5) = kalmanGain_K[stateRowIndex][5];
        }

        errorCovariance_P = multiplyMatrix16(
            subtractMatrix16(makeIdentityMatrix16(), kalmanTimesMeasurementMatrix_KH),
            errorCovariance_P
        );
    }

    void updateUsingBarometerDownPosition(double measuredBarometerDown_m,
                                          double barometerStdDev_m)
    {
        double predictedBarometerDown_m = nominalPosition_ned_m.down + estimatedBarometerBias_m;
        double barometerInnovation_y = measuredBarometerDown_m - predictedBarometerDown_m;
        double barometerMeasurementVariance_R = barometerStdDev_m * barometerStdDev_m;

        double scalarInnovationVariance_S =
            errorCovariance_P(2, 2)
          + errorCovariance_P(15, 15)
          + 2.0 * errorCovariance_P(2, 15)
          + barometerMeasurementVariance_R;

        if (std::fabs(scalarInnovationVariance_S) < 1e-12) {
            return;
        }

        double kalmanGainFromBarometer_K[16]{};
        for (int stateRowIndex = 0; stateRowIndex < 16; ++stateRowIndex) {
            kalmanGainFromBarometer_K[stateRowIndex] =
                (errorCovariance_P(stateRowIndex, 2) + errorCovariance_P(stateRowIndex, 15))
                / scalarInnovationVariance_S;
        }

        ErrorStateVector16 estimatedErrorState_dx{};
        for (int stateRowIndex = 0; stateRowIndex < 16; ++stateRowIndex) {
            estimatedErrorState_dx[stateRowIndex] =
                kalmanGainFromBarometer_K[stateRowIndex] * barometerInnovation_y;
        }

        injectEstimatedErrorStateIntoNominalState(estimatedErrorState_dx);

        Matrix16x16 kalmanTimesMeasurementMatrix_KH{};
        for (int stateRowIndex = 0; stateRowIndex < 16; ++stateRowIndex) {
            kalmanTimesMeasurementMatrix_KH(stateRowIndex, 2)  = kalmanGainFromBarometer_K[stateRowIndex];
            kalmanTimesMeasurementMatrix_KH(stateRowIndex, 15) = kalmanGainFromBarometer_K[stateRowIndex];
        }

        errorCovariance_P = multiplyMatrix16(
            subtractMatrix16(makeIdentityMatrix16(), kalmanTimesMeasurementMatrix_KH),
            errorCovariance_P
        );
    }

private:
    void injectEstimatedErrorStateIntoNominalState(const ErrorStateVector16& estimatedErrorState_dx)
    {
        nominalPosition_ned_m = nominalPosition_ned_m + NEDVector3{
            estimatedErrorState_dx[0],
            estimatedErrorState_dx[1],
            estimatedErrorState_dx[2]
        };

        nominalVelocity_ned_mps = nominalVelocity_ned_mps + NEDVector3{
            estimatedErrorState_dx[3],
            estimatedErrorState_dx[4],
            estimatedErrorState_dx[5]
        };

        NEDVector3 smallAngleAttitudeError_rad{
            estimatedErrorState_dx[6],
            estimatedErrorState_dx[7],
            estimatedErrorState_dx[8]
        };

        BodyToNEDQuaternion smallAngleCorrection_q{
            1.0,
            0.5 * smallAngleAttitudeError_rad.north,
            0.5 * smallAngleAttitudeError_rad.east,
            0.5 * smallAngleAttitudeError_rad.down
        };

        nominalAttitude_bodyToNed_q = multiplyQuaternions(nominalAttitude_bodyToNed_q, smallAngleCorrection_q);
        normalizeQuaternion(nominalAttitude_bodyToNed_q);

        estimatedAccelBias_body_mps2 = estimatedAccelBias_body_mps2 + NEDVector3{
            estimatedErrorState_dx[9],
            estimatedErrorState_dx[10],
            estimatedErrorState_dx[11]
        };

        estimatedGyroBias_body_rps = estimatedGyroBias_body_rps + NEDVector3{
            estimatedErrorState_dx[12],
            estimatedErrorState_dx[13],
            estimatedErrorState_dx[14]
        };

        estimatedBarometerBias_m += estimatedErrorState_dx[15];
    }
};

int main() {
    ErrorStateEkfNed ekf;

    double imuSamplePeriod_s = 0.001;

    for (int imuSampleIndex = 1; imuSampleIndex <= 5000; ++imuSampleIndex) {
        NEDVector3 demoImuSpecificForce_body_mps2{0.0, 0.0, -9.80665};
        NEDVector3 demoImuAngularRate_body_rps{0.0, 0.0, 0.0};

        ekf.propagateUsingImu(
            demoImuSpecificForce_body_mps2,
            demoImuAngularRate_body_rps,
            imuSamplePeriod_s
        );

        if (imuSampleIndex % 20 == 0) {
            double demoBarometerDown_m = 0.0;
            ekf.updateUsingBarometerDownPosition(
                demoBarometerDown_m,
                0.8
            );
        }

        if (imuSampleIndex % 100 == 0) {
            NEDVector3 demoGpsPosition_ned_m{0.0, 0.0, 0.0};
            NEDVector3 demoGpsVelocity_ned_mps{0.0, 0.0, 0.0};

            ekf.updateUsingGpsPositionVelocity(
                demoGpsPosition_ned_m,
                demoGpsVelocity_ned_mps,
                2.5,
                0.3
            );
        }
    }

    std::cout << "Final position (NED m): "
              << ekf.nominalPosition_ned_m.north << ", "
              << ekf.nominalPosition_ned_m.east  << ", "
              << ekf.nominalPosition_ned_m.down  << "\n";

    return 0;
}
