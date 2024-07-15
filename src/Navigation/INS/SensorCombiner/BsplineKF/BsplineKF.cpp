// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "BsplineKF.hpp"

#include "QuadraticBsplines.hpp"

#include "Navigation/Transformations/Units.hpp"
#include "util/Logger.hpp"

Eigen::MatrixXd NAV::BsplineKF::initialErrorCovarianceMatrix_P0(const std::vector<Eigen::VectorXd>& initCovariances, const uint8_t& numStates, const bool& imuCharacteristicsIdentical)
{
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(numStates, numStates);

    P.block<9, 9>(0, 0).diagonal() = initCovariances.at(0); // Angular rate B-spline factors
    P.block<9, 9>(9, 9).diagonal() = initCovariances.at(1); // Acceleration B-spline factors

    // Biases
    size_t j = 2;
    for (uint32_t i = 18; i < numStates; i += 6)
    {
        if (!imuCharacteristicsIdentical)
        {
            j = 2 + (i - 18) / 3; // access bias variances for each sensor from the third element onwards
        }
        P.block<3, 3>(i, i).diagonal() = initCovariances.at(j);
        P.block<3, 3>(i + 3, i + 3).diagonal() = initCovariances.at(j + 1);
    }

    return P;
}

void NAV::BsplineKF::processNoiseMatrix_Q(Eigen::MatrixXd& Q, const double& dt, const std::vector<Eigen::VectorXd>& processNoiseVariances, const uint8_t& numStates, const bool& imuCharacteristicsIdentical)
{
    Q.block<9, 9>(0, 0).diagonal() = processNoiseVariances.at(0) * dt; // Random walk of the angular rate B-spline factors
    Q.block<9, 9>(9, 9).diagonal() = processNoiseVariances.at(1) * dt; // Random walk of the acceleration B-spline factors

    // Random walk of the bias states
    size_t j = 2;
    for (uint32_t i = 18; i < numStates; i += 6) // FIXME: i = 'numStatesEst' better than '12'? (magic numbers)
    {
        if (!imuCharacteristicsIdentical)
        {
            j = 2 + (i - 18) / 3; // access 2 bias variances for each sensor from the third element onwards
        }
        Q.block<3, 3>(i, i).diagonal() = processNoiseVariances.at(j) * dt;             // variance for the process noise of the angular rate
        Q.block<3, 3>(i + 3, i + 3).diagonal() = processNoiseVariances.at(j + 1) * dt; // variance for the process noise of the acceleration
    }
}

void NAV::BsplineKF::rotateCoeffStates(Eigen::MatrixXd& x)
{
    x.block<6, 1>(0, 0) = x.block<6, 1>(3, 0); // Angular rate coeffs (c_{omega,k+1} = c_{omega,k})
    x.block<6, 1>(9, 0) = x.block<6, 1>(9, 0); // Acceleration coeffs (c_{f,k+1}     = c_{f,k})
}

void NAV::BsplineKF::rotateErrorCovariances(Eigen::MatrixXd& P, uint8_t& numStates, const double& sigmaScalingFactorAngRate, const double& sigmaScalingFactorAccel)
{
    int nCoeffsPerStackedBspline = 9;
    auto nCoeffsRemainingAngRate = static_cast<int>(numStates) - nCoeffsPerStackedBspline;
    auto nCoeffsRemainingAccel = static_cast<int>(numStates) - 2 * nCoeffsPerStackedBspline;

    // Shift all error covariances corresponding to angular rate B-spline coeffs
    P.block<6, 6>(0, 0) = P.block<6, 6>(3, 3);                                             // Variances of the angular rate
    P.block(0, 9, 6, nCoeffsRemainingAngRate) = P.block(3, 9, 6, nCoeffsRemainingAngRate); // Upper right corner
    P.block(9, 0, nCoeffsRemainingAngRate, 6) = P.block(9, 3, nCoeffsRemainingAngRate, 6); // Lower left corner

    // Set angular rate error covariances for the new B-spline coeff
    P.block(0, 6, numStates, 3) = Eigen::MatrixXd::Zero(numStates, 3);                                        // Column
    P.block(6, 0, 3, numStates) = Eigen::MatrixXd::Zero(3, numStates);                                        // Row
    P.block<3, 3>(6, 6).diagonal() = std::pow(sigmaScalingFactorAngRate, 2) * P.block<3, 3>(3, 3).diagonal(); // Variance (i.e. squared)

    // Shift all error covariances corresponding to acceleration B-spline coeffs
    P.block<6, 6>(9, 9) = P.block<6, 6>(12, 12);                                          // Variances of the acceleration
    P.block<9, 6>(0, 9) = P.block<9, 6>(0, 12);                                           // Top middle part
    P.block(9, 18, 6, nCoeffsRemainingAccel) = P.block(12, 18, 6, nCoeffsRemainingAccel); // Middle right part
    P.block<6, 9>(9, 0) = P.block<6, 9>(12, 0);                                           // Middle left part
    P.block(18, 9, nCoeffsRemainingAccel, 6) = P.block(18, 12, nCoeffsRemainingAccel, 6); // Lower middle part

    // Set acceleration error covariances for the new B-spline coeff
    P.block(0, 15, numStates, 3) = Eigen::MatrixXd::Zero(numStates, 3);                                         // Column
    P.block(15, 0, 3, numStates) = Eigen::MatrixXd::Zero(3, numStates);                                         // Row
    P.block<3, 3>(15, 15).diagonal() = std::pow(sigmaScalingFactorAccel, 2) * P.block<3, 3>(12, 12).diagonal(); // Variance (i.e. squared)
}

Eigen::MatrixXd NAV::BsplineKF::designMatrix_H(const double& ti, const double& splineSpacing, const Eigen::Matrix3d& DCM_accel, const Eigen::Matrix3d& DCM_gyro, const size_t& pinIndex, const uint8_t& numMeasurements, const uint8_t& numStates, const uint8_t& numStatesEst, const uint8_t& numStatesPerPin)
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(numMeasurements, numStates);

    auto qBspline = NAV::BsplineKF::quadraticBsplines(ti, splineSpacing);

    // Set quadratic B-splines in design matrix H
    for (size_t knotIdx = 0; knotIdx < 3; knotIdx++)
    {
        auto stateIdx = static_cast<Eigen::Index>(3 * knotIdx);
        H.block<3, 3>(0, stateIdx).diagonal() = qBspline.at(knotIdx) * Eigen::Vector3d::Ones();
        LOG_DATA("stateIdx = {}, knotIdx = {}, H =\n{}", stateIdx, knotIdx, H);
    }

    // Set the three stacked quadratic B-spline elements for the acceleration (analogous to the angular rate block)
    H.block<3, 9>(3, 9) = DCM_accel.transpose() * H.block<3, 9>(0, 0);
    // Rotate the angular rate blocks
    H.block<3, 9>(0, 0) = DCM_gyro.transpose() * H.block<3, 9>(0, 0);
    LOG_DATA("H =\n{}", H);

    // Map the biases to the states as in the IRWKF
    if (pinIndex > 0)
    {
        const auto stateIndex = static_cast<uint8_t>(numStatesEst + numStatesPerPin * (pinIndex - 1));
        LOG_DATA("stateIndex = {}", stateIndex);

        H.block<6, 6>(0, stateIndex) = Eigen::MatrixXd::Identity(6, 6);
    }

    return H;
}

NAV::KalmanFilter NAV::BsplineKF::initializeKalmanFilterManually(const size_t& numInputPins, const std::vector<PinData>& pinData, const PinDataBsplineKF& pinDataBsplineKF, const uint8_t& numStates, const double& dtInit, std::vector<Eigen::VectorXd>& processNoiseVariances, KalmanFilter& kalmanFilter, const bool& imuCharacteristicsIdentical, const bool& imuBiasesIdentical)
{
    // ------------------------------------------ State vector x ---------------------------------------------
    std::vector<Eigen::VectorXd> initStates;
    initStates.resize(numStates);

    // Initial B-spline coefficients of the angular rate in [rad/s]
    if (pinDataBsplineKF.initCoeffsAngularRateUnit == PinData::AngRateUnit::deg_s)
    {
        initStates[0] = deg2rad(pinDataBsplineKF.initCoeffsAngRate);
    }
    if (pinDataBsplineKF.initCoeffsAngularRateUnit == PinData::AngRateUnit::rad_s)
    {
        initStates[0] = pinDataBsplineKF.initCoeffsAngRate;
    }

    // Initial B-spline coefficients of the acceleration in [m/s]
    if (pinDataBsplineKF.initCoeffsAccelUnit == PinData::AccelerationUnit::m_s2)
    {
        initStates[1] = pinDataBsplineKF.initCoeffsAccel;
    }

    size_t pinDataBiasesIdenticalIdx = 1;
    for (size_t pinIndex = 1; pinIndex < numInputPins; ++pinIndex)
    {
        if (!imuBiasesIdentical)
        {
            pinDataBiasesIdenticalIdx = pinIndex;
        }

        // Initial bias of the angular rate in [rad/s]
        if (pinData[pinDataBiasesIdenticalIdx].initAngularRateBiasUnit == PinData::AngRateUnit::rad_s)
        {
            initStates[2 * pinIndex] = pinData[pinDataBiasesIdenticalIdx].initAngularRateBias.array();
        }
        else if (pinData[pinDataBiasesIdenticalIdx].initAngularRateBiasUnit == PinData::AngRateUnit::deg_s)
        {
            initStates[2 * pinIndex] = deg2rad(pinData[pinDataBiasesIdenticalIdx].initAngularRateBias).array();
        }

        // Initial bias of the acceleration in [m/s²]
        if (pinData[pinDataBiasesIdenticalIdx].initAccelerationBiasUnit == PinData::AccelerationUnit::m_s2)
        {
            initStates[1 + 2 * pinIndex] = pinData[pinDataBiasesIdenticalIdx].initAccelerationBias.array();
        }
    }

    kalmanFilter.x.block<9, 1>(0, 0) = initStates[0];
    kalmanFilter.x.block<9, 1>(9, 0) = initStates[1];
    LOG_DATA("Initial B-spline coefficients in the state vector x:\n{}", kalmanFilter.x.block<18, 1>(0, 0));

    uint32_t containerIndex = 2;
    for (uint32_t pinIndex = 0; pinIndex < numInputPins - 1UL; ++pinIndex)
    {
        if (!imuBiasesIdentical)
        {
            containerIndex = 2 + 2 * pinIndex;
        }
        kalmanFilter.x.block<3, 1>(18 + 6 * pinIndex, 0) = initStates[containerIndex];
        kalmanFilter.x.block<3, 1>(21 + 6 * pinIndex, 0) = initStates[1 + containerIndex];
    }
    LOG_DATA("Initial bias values:\n{}", kalmanFilter.x.block<18, 1>(18, 0));

    // -------------------------------------- State transition matrix ----------------------------------------
    kalmanFilter.Phi = Eigen::MatrixXd::Identity(numStates, numStates);

    // ------------------------------------- Error covariance matrix P ---------------------------------------

    std::vector<Eigen::VectorXd> initErrorCovariances;
    initErrorCovariances.resize(2 + 2 * (numInputPins - 1));

    // Initial error covariance of the B-spline KF angular rate coefficients in [rad²/s²]
    if (pinDataBsplineKF.initCovarianceCoeffsAngRateUnit == PinData::AngRateVarianceUnit::rad2_s2)
    {
        initErrorCovariances[0] = pinDataBsplineKF.initCovarianceCoeffsAngRate;
    }
    else if (pinDataBsplineKF.initCovarianceCoeffsAngRateUnit == PinData::AngRateVarianceUnit::deg2_s2)
    {
        initErrorCovariances[0] = deg2rad(pinDataBsplineKF.initCovarianceCoeffsAngRate);
    }
    else if (pinDataBsplineKF.initCovarianceCoeffsAngRateUnit == PinData::AngRateVarianceUnit::rad_s)
    {
        initErrorCovariances[0] = pinDataBsplineKF.initCovarianceCoeffsAngRate.array().pow(2);
    }
    else if (pinDataBsplineKF.initCovarianceCoeffsAngRateUnit == PinData::AngRateVarianceUnit::deg_s)
    {
        initErrorCovariances[0] = deg2rad(pinDataBsplineKF.initCovarianceCoeffsAngRate).array().pow(2);
    }

    // Initial error covariance of the B-spline KF acceleration coefficients in [(m^2)/(s^4)]
    if (pinDataBsplineKF.initCovarianceCoeffsAccelUnit == PinData::AccelerationVarianceUnit::m2_s4)
    {
        initErrorCovariances[1] = pinDataBsplineKF.initCovarianceCoeffsAccel;
    }
    else if (pinDataBsplineKF.initCovarianceCoeffsAccelUnit == PinData::AccelerationVarianceUnit::m_s2)
    {
        initErrorCovariances[1] = pinDataBsplineKF.initCovarianceCoeffsAccel.array().pow(2);
    }

    size_t pinDataIdx = 1;
    for (size_t pinIndex = 1; pinIndex < numInputPins; ++pinIndex)
    {
        if (!imuCharacteristicsIdentical)
        {
            pinDataIdx = pinIndex;
        }

        // Initial Covariance of the bias of the angular rate in [rad²/s²]
        if (pinData[pinDataIdx].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::rad2_s2)
        {
            initErrorCovariances[2 * pinIndex] = pinData[pinDataIdx].initCovarianceBiasAngRate;
        }
        else if (pinData[pinDataIdx].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::deg2_s2)
        {
            initErrorCovariances[2 * pinIndex] = deg2rad(pinData[pinDataIdx].initCovarianceBiasAngRate);
        }
        else if (pinData[pinDataIdx].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::rad_s)
        {
            initErrorCovariances[2 * pinIndex] = pinData[pinDataIdx].initCovarianceBiasAngRate.array().pow(2);
        }
        else if (pinData[pinDataIdx].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::deg_s)
        {
            initErrorCovariances[2 * pinIndex] = deg2rad(pinData[pinDataIdx].initCovarianceBiasAngRate).array().pow(2);
        }

        // Initial Covariance of the bias of the acceleration in [(m^2)/(s^4)]
        if (pinData[pinDataIdx].initCovarianceBiasAccUnit == PinData::AccelerationVarianceUnit::m2_s4)
        {
            initErrorCovariances[1 + 2 * pinIndex] = pinData[pinDataIdx].initCovarianceBiasAcc;
        }
        else if (pinData[pinDataIdx].initCovarianceBiasAccUnit == PinData::AccelerationVarianceUnit::m_s2)
        {
            initErrorCovariances[1 + 2 * pinIndex] = pinData[pinDataIdx].initCovarianceBiasAcc.array().pow(2);
        }
    }

    kalmanFilter.P = NAV::BsplineKF::initialErrorCovarianceMatrix_P0(initErrorCovariances, numStates, imuCharacteristicsIdentical);

    // -------------------------------------- Process noise matrix Q -----------------------------------------
    processNoiseVariances.resize(2 * numInputPins);

    // 𝜎_AngRateFactors: Standard deviation of the noise on the B-spline coefficients angular rate states [rad/s]
    switch (pinDataBsplineKF.varCoeffsAngRateUnit)
    {
    case PinData::AngRateVarianceUnit::rad2_s2:
        processNoiseVariances[0] = pinDataBsplineKF.varCoeffsAngRateNoise;
        break;
    case PinData::AngRateVarianceUnit::deg2_s2:
        processNoiseVariances[0] = deg2rad(pinDataBsplineKF.varCoeffsAngRateNoise);
        break;
    case PinData::AngRateVarianceUnit::deg_s:
        processNoiseVariances[0] = deg2rad(pinDataBsplineKF.varCoeffsAngRateNoise).array().pow(2);
        break;
    case PinData::AngRateVarianceUnit::rad_s:
        processNoiseVariances[0] = pinDataBsplineKF.varCoeffsAngRateNoise.array().pow(2);
        break;
    }

    // 𝜎_AccelFactors: Standard deviation of the noise on the B-spline coefficients acceleration states [m/s]
    switch (pinDataBsplineKF.varCoeffsAccelUnit)
    {
    case PinData::AccelerationVarianceUnit::m2_s4:
        processNoiseVariances[1] = pinDataBsplineKF.varCoeffsAccelNoise;
        break;
    case PinData::AccelerationVarianceUnit::m_s2:
        processNoiseVariances[1] = pinDataBsplineKF.varCoeffsAccelNoise.array().pow(2);
        break;
    }

    for (size_t pinIndex = 1; pinIndex < numInputPins; ++pinIndex)
    {
        if (!imuCharacteristicsIdentical)
        {
            pinDataIdx = pinIndex;
        }

        // 𝜎_biasAngRate Standard deviation of the bias on the angular rate state [rad/s²]
        switch (pinData[pinDataIdx].varBiasAngRateNoiseUnit)
        {
        case PinData::AngRateVarianceUnit::rad2_s2:
            processNoiseVariances[2 * pinIndex] = pinData[pinDataIdx].varBiasAngRateNoise;
            break;
        case PinData::AngRateVarianceUnit::deg2_s2:
            processNoiseVariances[2 * pinIndex] = deg2rad(pinData[pinDataIdx].varBiasAngRateNoise);
            break;
        case PinData::AngRateVarianceUnit::deg_s:
            processNoiseVariances[2 * pinIndex] = deg2rad(pinData[pinDataIdx].varBiasAngRateNoise).array().pow(2);
            break;
        case PinData::AngRateVarianceUnit::rad_s:
            processNoiseVariances[2 * pinIndex] = pinData[pinDataIdx].varBiasAngRateNoise.array().pow(2);
            break;
        }

        // 𝜎_biasAcceleration Standard deviation of the noise on the acceleration state [m/s³]
        switch (pinData[pinDataIdx].varBiasAccelerationNoiseUnit)
        {
        case PinData::AccelerationVarianceUnit::m2_s4:
            processNoiseVariances[1 + 2 * pinIndex] = pinData[pinDataIdx].varBiasAccelerationNoise;
            break;
        case PinData::AccelerationVarianceUnit::m_s2:
            processNoiseVariances[1 + 2 * pinIndex] = pinData[pinDataIdx].varBiasAccelerationNoise.array().pow(2);
            break;
        }
    }

    NAV::BsplineKF::processNoiseMatrix_Q(kalmanFilter.Q, dtInit, processNoiseVariances, numStates, imuCharacteristicsIdentical);

    return kalmanFilter;
}