// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "IRWKF.hpp"

#include "Navigation/Transformations/Units.hpp"
#include "util/Logger.hpp"

Eigen::MatrixXd NAV::IRWKF::initialStateTransitionMatrix_Phi(const double& dt, const uint8_t& numStates)
{
    const auto nStates = static_cast<Eigen::Index>(numStates);
    Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(nStates, nStates);

    Phi.block<3, 3>(0, 3).diagonal().setConstant(dt); // dependency of angular rate on angular acceleration
    Phi.block<3, 3>(6, 9).diagonal().setConstant(dt); // dependency of acceleration on jerk

    return Phi;
}

Eigen::MatrixXd NAV::IRWKF::initialErrorCovarianceMatrix_P0(const std::vector<Eigen::Vector3d>& initCovariances, const uint8_t& numStates, const bool& imuCharacteristicsIdentical)
{
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(numStates, numStates);

    P.block<3, 3>(0, 0).diagonal() = initCovariances.at(0); // initial covariance of the angular rate
    P.block<3, 3>(3, 3).diagonal() = initCovariances.at(1); // initial covariance of the angular acceleration
    P.block<3, 3>(6, 6).diagonal() = initCovariances.at(2); // initial covariance of the acceleration
    P.block<3, 3>(9, 9).diagonal() = initCovariances.at(3); // initial covariance of the jerk

    size_t j = 4;
    for (uint32_t i = 12; i < numStates; i += 6)
    {
        if (!imuCharacteristicsIdentical)
        {
            j = 4 + (i - 12) / 3; // access bias variances for each sensor from the fifth element onwards
        }

        P.block<3, 3>(i, i).diagonal() = initCovariances.at(j);
        P.block<3, 3>(i + 3, i + 3).diagonal() = initCovariances.at(j + 1);
    }

    return P;
}

void NAV::IRWKF::processNoiseMatrix_Q(Eigen::MatrixXd& Q, const double& dt, const std::vector<Eigen::VectorXd>& processNoiseVariances, const uint8_t& numStates, const bool& imuCharacteristicsIdentical)
{
    // Integrated Random Walk of the angular rate
    Q.block<3, 3>(0, 0).diagonal() = processNoiseVariances.at(0) / 3. * std::pow(dt, 3);
    Q.block<3, 3>(0, 3).diagonal() = processNoiseVariances.at(0) / 2. * std::pow(dt, 2);
    Q.block<3, 3>(3, 0).diagonal() = processNoiseVariances.at(0) / 2. * std::pow(dt, 2);
    Q.block<3, 3>(3, 3).diagonal() = processNoiseVariances.at(0) * dt;

    // Integrated Random Walk of the acceleration
    Q.block<3, 3>(6, 6).diagonal() = processNoiseVariances.at(1) / 3. * std::pow(dt, 3);
    Q.block<3, 3>(6, 9).diagonal() = processNoiseVariances.at(1) / 2. * std::pow(dt, 2);
    Q.block<3, 3>(9, 6).diagonal() = processNoiseVariances.at(1) / 2. * std::pow(dt, 2);
    Q.block<3, 3>(9, 9).diagonal() = processNoiseVariances.at(1) * dt;

    // Random Walk of the bias states
    size_t j = 4;
    for (uint32_t i = 12; i < numStates; i += 6) // FIXME: i = 'numStatesEst' better than '12'? (magic numbers)
    {
        if (!imuCharacteristicsIdentical)
        {
            j = 2 + (i - 12) / 3; // access 2 bias variances for each sensor from the third element onwards
        }
        Q.block<3, 3>(i, i).diagonal() = processNoiseVariances.at(j) * dt;             // variance for the process noise of the angular rate
        Q.block<3, 3>(i + 3, i + 3).diagonal() = processNoiseVariances.at(j + 1) * dt; // variance for the process noise of the acceleration
    }
}

void NAV::IRWKF::stateTransitionMatrix_Phi(Eigen::MatrixXd& Phi, const double& dt)
{
    Phi.block<3, 3>(0, 3).diagonal().setConstant(dt); // dependency of angular rate on angular acceleration
    Phi.block<3, 3>(6, 9).diagonal().setConstant(dt); // dependency of acceleration on jerk
}

Eigen::MatrixXd NAV::IRWKF::designMatrix_H(const Eigen::Matrix3d& DCM_accel, const Eigen::Matrix3d& DCM_gyro, const size_t& pinIndex, const uint8_t& numMeasurements, const uint8_t& numStates, const uint8_t& numStatesEst, const uint8_t& numStatesPerPin)
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(numMeasurements, numStates);

    // Mounting angles of sensor with latest measurement
    H.block<3, 3>(0, 0) = DCM_accel.transpose(); // Inverse rotation for angular rate
    H.block<3, 3>(3, 6) = DCM_gyro.transpose();  // Inverse rotation for acceleration

    // Mapping of bias states on sensor with the latest measurement
    if (pinIndex > 0)
    {
        const auto stateIndex = static_cast<uint8_t>(numStatesEst + numStatesPerPin * (pinIndex - 1));
        LOG_DATA("stateIndex = {}", stateIndex);

        H.block<6, 6>(0, stateIndex) = Eigen::MatrixXd::Identity(6, 6);
    }

    return H;
}

NAV::KalmanFilter NAV::IRWKF::initializeKalmanFilterManually(const size_t& numInputPins, const std::vector<PinData>& pinData, const PinDataIRWKF& pinDataIRWKF, const uint8_t& numStates, const double& dtInit, std::vector<Eigen::VectorXd>& processNoiseVariances, KalmanFilter& kalmanFilter, const bool& imuCharacteristicsIdentical, const bool& imuBiasesIdentical)
{
    // ------------------------------------------ State vector x ---------------------------------------------

    std::vector<Eigen::Vector3d> initStates;
    initStates.resize(6 + 2 * numInputPins); // FIXME: Shouldn't this be 'numStates'?

    // Initial angular rate in [rad/s]
    if (pinDataIRWKF.initAngularRateUnit == PinData::AngRateUnit::deg_s)
    {
        initStates[0] = deg2rad(pinDataIRWKF.initAngularRate);
    }
    if (pinDataIRWKF.initAngularRateUnit == PinData::AngRateUnit::rad_s)
    {
        initStates[0] = pinDataIRWKF.initAngularRate;
    }

    // Initial acceleration in [m/s²]
    if (pinDataIRWKF.initAccelerationUnit == PinData::AccelerationUnit::m_s2)
    {
        initStates[1] = pinDataIRWKF.initAcceleration;
    }

    // Initial angular acceleration in [rad/s²]
    if (pinDataIRWKF.initAngularAccUnit == PinDataIRWKF::AngularAccUnit::deg_s2)
    {
        initStates[2] = deg2rad(pinDataIRWKF.initAngularAcc);
    }
    if (pinDataIRWKF.initAngularAccUnit == PinDataIRWKF::AngularAccUnit::rad_s2)
    {
        initStates[2] = pinDataIRWKF.initAngularAcc;
    }

    // Initial jerk in [m/s³]
    if (pinDataIRWKF.initJerkUnit == PinDataIRWKF::JerkUnit::m_s3)
    {
        initStates[3] = pinDataIRWKF.initJerk;
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
            initStates[2 + 2 * pinIndex] = pinData[pinDataBiasesIdenticalIdx].initAngularRateBias.array();
        }
        else if (pinData[pinDataBiasesIdenticalIdx].initAngularRateBiasUnit == PinData::AngRateUnit::deg_s)
        {
            initStates[2 + 2 * pinIndex] = deg2rad(pinData[pinDataBiasesIdenticalIdx].initAngularRateBias).array();
        }

        // Initial bias of the acceleration in [m/s²]
        if (pinData[pinDataBiasesIdenticalIdx].initAccelerationBiasUnit == PinData::AccelerationUnit::m_s2)
        {
            initStates[3 + 2 * pinIndex] = pinData[pinDataBiasesIdenticalIdx].initAccelerationBias.array();
        }
    }

    kalmanFilter.x.block<3, 1>(0, 0) = initStates[0];
    kalmanFilter.x.block<3, 1>(3, 0) = initStates[1];
    kalmanFilter.x.block<3, 1>(6, 0) = initStates[2];
    kalmanFilter.x.block<3, 1>(9, 0) = initStates[3];

    uint32_t containerIndex = 4;
    for (uint32_t pinIndex = 0; pinIndex < numInputPins - 1UL; ++pinIndex)
    {
        if (!imuBiasesIdentical)
        {
            containerIndex = 4 + 2 * pinIndex;
        }
        kalmanFilter.x.block<3, 1>(12 + 6 * pinIndex, 0) = initStates[containerIndex];
        kalmanFilter.x.block<3, 1>(15 + 6 * pinIndex, 0) = initStates[1 + containerIndex];
    }

    // -------------------------------------- State transition matrix ----------------------------------------
    kalmanFilter.Phi = NAV::IRWKF::initialStateTransitionMatrix_Phi(dtInit, numStates);

    // ------------------------------------- Error covariance matrix P ---------------------------------------

    std::vector<Eigen::Vector3d> initErrorCovariances;
    initErrorCovariances.resize(6 + 2 * numInputPins);

    // Initial Covariance of the angular rate in [rad²/s²]
    if (pinData[0].initCovarianceAngularRateUnit == PinData::AngRateVarianceUnit::rad2_s2)
    {
        initErrorCovariances[0] = pinData[0].initCovarianceAngularRate;
    }
    else if (pinData[0].initCovarianceAngularRateUnit == PinData::AngRateVarianceUnit::deg2_s2)
    {
        initErrorCovariances[0] = deg2rad(pinData[0].initCovarianceAngularRate);
    }
    else if (pinData[0].initCovarianceAngularRateUnit == PinData::AngRateVarianceUnit::rad_s)
    {
        initErrorCovariances[0] = pinData[0].initCovarianceAngularRate.array().pow(2);
    }
    else if (pinData[0].initCovarianceAngularRateUnit == PinData::AngRateVarianceUnit::deg_s)
    {
        initErrorCovariances[0] = deg2rad(pinData[0].initCovarianceAngularRate).array().pow(2);
    }

    // Initial Covariance of the angular acceleration in [(rad^2)/(s^4)]
    if (pinDataIRWKF.initCovarianceAngularAccUnit == PinDataIRWKF::AngularAccVarianceUnit::rad2_s4)
    {
        initErrorCovariances[1] = pinDataIRWKF.initCovarianceAngularAcc;
    }
    else if (pinDataIRWKF.initCovarianceAngularAccUnit == PinDataIRWKF::AngularAccVarianceUnit::deg2_s4)
    {
        initErrorCovariances[1] = deg2rad(pinDataIRWKF.initCovarianceAngularAcc);
    }
    else if (pinDataIRWKF.initCovarianceAngularAccUnit == PinDataIRWKF::AngularAccVarianceUnit::rad_s2)
    {
        initErrorCovariances[1] = pinDataIRWKF.initCovarianceAngularAcc.array().pow(2);
    }
    else if (pinDataIRWKF.initCovarianceAngularAccUnit == PinDataIRWKF::AngularAccVarianceUnit::deg_s2)
    {
        initErrorCovariances[1] = deg2rad(pinDataIRWKF.initCovarianceAngularAcc).array().pow(2);
    }

    // Initial Covariance of the acceleration in [(m^2)/(s^4)]
    if (pinData[0].initCovarianceAccelerationUnit == PinData::AccelerationVarianceUnit::m2_s4)
    {
        initErrorCovariances[2] = pinData[0].initCovarianceAcceleration;
    }
    else if (pinData[0].initCovarianceAccelerationUnit == PinData::AccelerationVarianceUnit::m_s2)
    {
        initErrorCovariances[2] = pinData[0].initCovarianceAcceleration.array().pow(2);
    }

    // Initial Covariance of the jerk in [(m^2)/(s^6)]
    if (pinDataIRWKF.initCovarianceJerkUnit == PinDataIRWKF::JerkVarianceUnit::m2_s6)
    {
        initErrorCovariances[3] = pinDataIRWKF.initCovarianceJerk;
    }
    else if (pinDataIRWKF.initCovarianceJerkUnit == PinDataIRWKF::JerkVarianceUnit::m_s3)
    {
        initErrorCovariances[3] = pinDataIRWKF.initCovarianceJerk.array().pow(2);
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
            initErrorCovariances[2 + 2 * pinIndex] = pinData[pinDataIdx].initCovarianceBiasAngRate;
        }
        else if (pinData[pinDataIdx].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::deg2_s2)
        {
            initErrorCovariances[2 + 2 * pinIndex] = deg2rad(pinData[pinDataIdx].initCovarianceBiasAngRate);
        }
        else if (pinData[pinDataIdx].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::rad_s)
        {
            initErrorCovariances[2 + 2 * pinIndex] = pinData[pinDataIdx].initCovarianceBiasAngRate.array().pow(2);
        }
        else if (pinData[pinDataIdx].initCovarianceBiasAngRateUnit == PinData::AngRateVarianceUnit::deg_s)
        {
            initErrorCovariances[2 + 2 * pinIndex] = deg2rad(pinData[pinDataIdx].initCovarianceBiasAngRate).array().pow(2);
        }

        // Initial Covariance of the bias of the acceleration in [(m^2)/(s^4)]
        if (pinData[pinDataIdx].initCovarianceBiasAccUnit == PinData::AccelerationVarianceUnit::m2_s4)
        {
            initErrorCovariances[3 + 2 * pinIndex] = pinData[pinDataIdx].initCovarianceBiasAcc;
        }
        else if (pinData[pinDataIdx].initCovarianceBiasAccUnit == PinData::AccelerationVarianceUnit::m_s2)
        {
            initErrorCovariances[3 + 2 * pinIndex] = pinData[pinDataIdx].initCovarianceBiasAcc.array().pow(2);
        }
    }

    kalmanFilter.P = NAV::IRWKF::initialErrorCovarianceMatrix_P0(initErrorCovariances, numStates, imuCharacteristicsIdentical);

    // -------------------------------------- Process noise matrix Q -----------------------------------------
    processNoiseVariances.resize(2 * numInputPins);

    // 𝜎_AngAcc Standard deviation of the noise on the angular acceleration state [rad/s²]
    switch (pinDataIRWKF.varAngularAccNoiseUnit)
    {
    case PinDataIRWKF::AngularAccVarianceUnit::rad2_s4:
        processNoiseVariances[0] = pinDataIRWKF.varAngularAccNoise;
        break;
    case PinDataIRWKF::AngularAccVarianceUnit::deg2_s4:
        processNoiseVariances[0] = deg2rad(pinDataIRWKF.varAngularAccNoise);
        break;
    case PinDataIRWKF::AngularAccVarianceUnit::deg_s2:
        processNoiseVariances[0] = deg2rad(pinDataIRWKF.varAngularAccNoise).array().pow(2);
        break;
    case PinDataIRWKF::AngularAccVarianceUnit::rad_s2:
        processNoiseVariances[0] = pinDataIRWKF.varAngularAccNoise.array().pow(2);
        break;
    }

    // 𝜎_jerk Standard deviation of the noise on the jerk state [m/s³]
    switch (pinDataIRWKF.varJerkNoiseUnit)
    {
    case PinDataIRWKF::JerkVarianceUnit::m2_s6:
        processNoiseVariances[1] = pinDataIRWKF.varJerkNoise;
        break;
    case PinDataIRWKF::JerkVarianceUnit::m_s3:
        processNoiseVariances[1] = pinDataIRWKF.varJerkNoise.array().pow(2);
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
        switch (pinData[pinIndex].varBiasAccelerationNoiseUnit)
        {
        case PinData::AccelerationVarianceUnit::m2_s4:
            processNoiseVariances[1 + 2 * pinIndex] = pinData[pinDataIdx].varBiasAccelerationNoise;
            break;
        case PinData::AccelerationVarianceUnit::m_s2:
            processNoiseVariances[1 + 2 * pinIndex] = pinData[pinDataIdx].varBiasAccelerationNoise.array().pow(2);
            break;
        }
    }

    NAV::IRWKF::processNoiseMatrix_Q(kalmanFilter.Q, dtInit, processNoiseVariances, numStates, imuCharacteristicsIdentical);

    return kalmanFilter;
}

NAV::KalmanFilter NAV::IRWKF::initializeKalmanFilterAuto(const size_t& nInputPins, const std::vector<PinData>& pinData, const PinDataIRWKF& pinDataIRWKF, const std::vector<size_t>& cumulatedPinIds, const std::vector<std::shared_ptr<const NAV::ImuObs>>& cumulatedImuObs, const bool& initJerkAngAcc, const double& dtInit, const uint8_t& numStates, const uint8_t& numMeasurements, std::vector<Eigen::VectorXd>& processNoiseVariances, KalmanFilter& kalmanFilter)
{
    // ---------------------------- Measurements (for initial state vector x0) -------------------------------

    std::vector<std::vector<std::shared_ptr<const NAV::ImuObs>>> sensorMeasurements; // pinIndex / msgIndex(imuObs)
    sensorMeasurements.resize(nInputPins);

    std::vector<Eigen::Vector3d> initStates;
    initStates.resize(6 + 2 * nInputPins); // state vector x

    // Split cumulated imuObs into vectors for each sensor
    for (size_t msgIndex = 0; msgIndex < cumulatedImuObs.size(); msgIndex++)
    {
        sensorMeasurements[cumulatedPinIds[msgIndex]].push_back(cumulatedImuObs[msgIndex]); // 'push_back' instead of 'resize()' and 'operator[]' since number of msgs of a certain pin are not known in advance
    }

    std::vector<std::vector<std::vector<double>>> sensorComponents; // pinIndex / axisIndex / msgIndex(double)
    sensorComponents.resize(nInputPins);

    for (size_t pinIndex = 0; pinIndex < nInputPins; pinIndex++) // loop thru connected sensors
    {
        sensorComponents[pinIndex].resize(numMeasurements);
        for (size_t axisIndex = 0; axisIndex < numMeasurements; axisIndex++) // loop thru the 6 measurements: AccX, GyroX, AccY, GyroY, AccZ, GyroZ
        {
            for (size_t msgIndex = 0; msgIndex < sensorMeasurements[pinIndex].size(); msgIndex++) // loop thru the msg of each measurement axis
            {
                if (axisIndex < 3) // Accelerations X/Y/Z
                {
                    sensorComponents[pinIndex][axisIndex].push_back(sensorMeasurements[pinIndex][msgIndex]->p_acceleration(static_cast<uint32_t>(axisIndex)));
                }
                else // Gyro X/Y/Z
                {
                    sensorComponents[pinIndex][axisIndex].push_back(sensorMeasurements[pinIndex][msgIndex]->p_angularRate(static_cast<uint32_t>(axisIndex - 3)));
                }
            }
        }
    }

    // --------------------------- Averaging single measurements of each sensor ------------------------------
    // Accelerations X/Y/Z (pos. 6,7,8 in state vector) - init value is mean of reference sensor, i.e. pinIndex = 0
    initStates[2] = mean(sensorComponents[0], 0);
    // Jerk X/Y/Z
    initStates[3] = Eigen::Vector3d::Zero();
    // Angular Rate X/Y/Z (pos. 0,1,2 in state vector) - init value is mean of reference sensor, i.e. pinIndex = 0
    initStates[0] = mean(sensorComponents[0], 3);
    // Angular Acceleration X/Y/Z
    initStates[1] = Eigen::Vector3d::Zero();

    // Bias-inits
    for (size_t pinIndex = 0; pinIndex < nInputPins - 1; pinIndex++) // nInputPins - 1 since there are only relative biases
    {
        auto stateIndex = 4 + 2 * pinIndex;                                                   // 4 states are Acceleration, Jerk, Angular Rate, Angular Acceleration (see above), plus 2 bias states per sensor
        initStates[stateIndex + 1] = mean(sensorComponents[pinIndex + 1], 0) - initStates[2]; // Acceleration biases
        initStates[stateIndex] = mean(sensorComponents[pinIndex + 1], 3) - initStates[0];     // Angular rate biases
    }

    // ------------------------------------ State transition matrix Phi --------------------------------------

    kalmanFilter.Phi = IRWKF::initialStateTransitionMatrix_Phi(dtInit, numStates);

    // ----------------------- Variance of each sensor (initial error covariance P0) -------------------------

    std::vector<Eigen::Vector3d> initErrorCovariances;
    initErrorCovariances.resize(6 + 2 * nInputPins); // error covariance matrix P

    // Acceleration variances X/Y/Z (pos. 6,7,8 on diagonal of P matrix) - init value is variance of reference sensor, i.e. pinIndex = 0
    initErrorCovariances[2] = variance(sensorComponents[0], 0);
    // Angular Rate variances X/Y/Z (pos. 0,1,2 on diagonal of P matrix) - init value is variance of reference sensor, i.e. pinIndex = 0
    initErrorCovariances[0] = variance(sensorComponents[0], 3);

    if (initJerkAngAcc)
    {
        // Jerk variances X/Y/Z
        initErrorCovariances[3] = initErrorCovariances[2];
        // Angular Acceleration variances X/Y/Z
        initErrorCovariances[1] = initErrorCovariances[0];
    }
    else
    {
        // Jerk variances X/Y/Z
        initErrorCovariances[3] = Eigen::Vector3d::Zero();
        // Angular Acceleration variances X/Y/Z
        initErrorCovariances[1] = Eigen::Vector3d::Zero();
    }

    // P-matrix bias inits
    for (size_t pinIndex = 0; pinIndex < nInputPins - 1; pinIndex++) // nInputPins - 1 since there are only relative biases
    {
        auto stateIndex = 4 + 2 * pinIndex;                                                 // 4 states are Acceleration, Jerk, Angular Rate, Angular Acceleration (see above), plus 2 bias states per sensor
        initErrorCovariances[stateIndex + 1] = variance(sensorComponents[pinIndex + 1], 0); // Acceleration biases
        initErrorCovariances[stateIndex] = variance(sensorComponents[pinIndex + 1], 3);     // Angular rate biases

        // Choose the bigger one of the two variances, i.e. of sensor #'pinIndex' and the reference sensor #0 (since bias is the difference of these two sensors)
        for (int axisIndex = 0; axisIndex < 3; axisIndex++)
        {
            // Acceleration variance
            if (initErrorCovariances[stateIndex + 1](axisIndex) < initErrorCovariances[2](axisIndex))
            {
                initErrorCovariances[stateIndex + 1](axisIndex) = initErrorCovariances[2](axisIndex);
            }
            // Angular rate variance
            if (initErrorCovariances[stateIndex](axisIndex) < initErrorCovariances[0](axisIndex))
            {
                initErrorCovariances[stateIndex](axisIndex) = initErrorCovariances[0](axisIndex);
            }
        }
    }

    kalmanFilter.x.block<3, 1>(0, 0) = initStates[0];
    kalmanFilter.x.block<3, 1>(3, 0) = initStates[1];
    kalmanFilter.x.block<3, 1>(6, 0) = initStates[2];
    kalmanFilter.x.block<3, 1>(9, 0) = initStates[3];
    for (uint32_t pinIndex = 0; pinIndex < nInputPins - 1UL; ++pinIndex)
    {
        auto containerIndex = 4 + 2 * pinIndex;
        kalmanFilter.x.block<3, 1>(12 + 6 * pinIndex, 0) = initStates[containerIndex];
        kalmanFilter.x.block<3, 1>(15 + 6 * pinIndex, 0) = initStates[1 + containerIndex];
    }

    kalmanFilter.P = IRWKF::initialErrorCovarianceMatrix_P0(initErrorCovariances, numStates);

    // ------------------------------------------------------- Process noise matrix Q ----------------------------------------------------------
    processNoiseVariances.resize(2 * nInputPins);

    // 𝜎_AngAcc Standard deviation of the noise on the angular acceleration state [rad/s²]
    switch (pinDataIRWKF.varAngularAccNoiseUnit)
    {
    case PinDataIRWKF::AngularAccVarianceUnit::rad2_s4:
        processNoiseVariances[0] = pinDataIRWKF.varAngularAccNoise;
        break;
    case PinDataIRWKF::AngularAccVarianceUnit::deg2_s4:
        processNoiseVariances[0] = deg2rad(pinDataIRWKF.varAngularAccNoise);
        break;
    case PinDataIRWKF::AngularAccVarianceUnit::deg_s2:
        processNoiseVariances[0] = deg2rad(pinDataIRWKF.varAngularAccNoise).array().pow(2);
        break;
    case PinDataIRWKF::AngularAccVarianceUnit::rad_s2:
        processNoiseVariances[0] = pinDataIRWKF.varAngularAccNoise.array().pow(2);
        break;
    }

    // 𝜎_jerk Standard deviation of the noise on the jerk state [m/s³]
    switch (pinDataIRWKF.varJerkNoiseUnit)
    {
    case PinDataIRWKF::JerkVarianceUnit::m2_s6:
        processNoiseVariances[1] = pinDataIRWKF.varJerkNoise;
        break;
    case PinDataIRWKF::JerkVarianceUnit::m_s3:
        processNoiseVariances[1] = pinDataIRWKF.varJerkNoise.array().pow(2);
        break;
    }

    for (size_t pinIndex = 0; pinIndex < nInputPins - 1; ++pinIndex)
    {
        // 𝜎_biasAngRate Standard deviation of the bias on the angular rate state [rad/s²]
        switch (pinData[1 + pinIndex].varBiasAngRateNoiseUnit)
        {
        case PinData::AngRateVarianceUnit::rad2_s2:
            processNoiseVariances[2 + 2 * pinIndex] = pinData[1 + pinIndex].varBiasAngRateNoise;
            break;
        case PinData::AngRateVarianceUnit::deg2_s2:
            processNoiseVariances[2 + 2 * pinIndex] = deg2rad(pinData[1 + pinIndex].varBiasAngRateNoise);
            break;
        case PinData::AngRateVarianceUnit::deg_s:
            processNoiseVariances[2 + 2 * pinIndex] = deg2rad(pinData[1 + pinIndex].varBiasAngRateNoise).array().pow(2);
            break;
        case PinData::AngRateVarianceUnit::rad_s:
            processNoiseVariances[2 + 2 * pinIndex] = pinData[1 + pinIndex].varBiasAngRateNoise.array().pow(2);
            break;
        }

        // 𝜎_biasAcceleration Standard deviation of the noise on the acceleration state [m/s³]
        switch (pinData[pinIndex].varBiasAccelerationNoiseUnit)
        {
        case PinData::AccelerationVarianceUnit::m2_s4:
            processNoiseVariances[3 + 2 * pinIndex] = pinData[1 + pinIndex].varBiasAccelerationNoise;
            break;
        case PinData::AccelerationVarianceUnit::m_s2:
            processNoiseVariances[3 + 2 * pinIndex] = pinData[1 + pinIndex].varBiasAccelerationNoise.array().pow(2);
            break;
        }
    }

    IRWKF::processNoiseMatrix_Q(kalmanFilter.Q, dtInit, processNoiseVariances, numStates);

    return kalmanFilter;
}

Eigen::Vector3d NAV::IRWKF::mean(const std::vector<std::vector<double>>& sensorType, const size_t& containerPos)
{
    Eigen::Vector3d meanVector = Eigen::Vector3d::Zero();

    for (size_t axisIndex = 0; axisIndex < 3; axisIndex++)
    {
        meanVector(static_cast<int>(axisIndex)) = std::accumulate(sensorType[axisIndex + containerPos].begin(), sensorType[axisIndex + containerPos].end(), 0.) / static_cast<double>(sensorType[axisIndex + containerPos].size());
    }

    return meanVector;
}

Eigen::Vector3d NAV::IRWKF::variance(const std::vector<std::vector<double>>& sensorType, const size_t& containerPos)
{
    Eigen::Vector3d varianceVector = Eigen::Vector3d::Zero();

    auto means = mean(sensorType, containerPos); // mean values for each axis

    for (size_t axisIndex = 0; axisIndex < 3; axisIndex++)
    {
        auto N = sensorType.at(axisIndex + containerPos).size(); // Number of msgs along the specific axis

        std::vector<double> absolSquared(N, 0.); // Inner part of the variance calculation (squared absolute values)

        for (size_t msgIndex = 0; msgIndex < N; msgIndex++)
        {
            absolSquared[msgIndex] = std::pow(std::abs(sensorType[axisIndex + containerPos][msgIndex] - means(static_cast<int>(axisIndex))), 2);
        }

        varianceVector(static_cast<int>(axisIndex)) = (1. / (static_cast<double>(N) - 1.)) * std::accumulate(absolSquared.begin(), absolSquared.end(), 0.);
    }

    return varianceVector;
}