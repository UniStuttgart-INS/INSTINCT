// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file IRWKF.hpp
/// @brief Kalman filter matrices for the Integrated-Random-Walk (IRW) Multi-IMU fusion
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2024-04-02

#pragma once

#include "util/Eigen.hpp"
#include "../PinData.hpp"
#include "Navigation/Math/KalmanFilter.hpp"
#include "NodeData/IMU/ImuObs.hpp"

#include <vector>

// TODO: Refactor: 'NAV::ImuFusion::IRWKF'
namespace NAV::IRWKF
{
/// @brief Calculates the state-transition-matrix 𝚽
/// @param[in] dt Time difference between two successive measurements
/// @param[in] numStates Number of KF states
/// @return State-transition-matrix 𝚽
[[nodiscard]] Eigen::MatrixXd initialStateTransitionMatrix_Phi(const double& dt, const uint8_t& numStates);

/// @brief Initial error covariance matrix P_0
/// @param[in] initCovariances Initial covariances of the states
/// @param[in] numStates Number of KF states
/// @param[in] imuCharacteristicsIdentical If the multiple IMUs have the same characteristics, GUI input cells can be reduced considerably
/// @return The (numStates) x (numStates) matrix of initial state variances
[[nodiscard]] Eigen::MatrixXd initialErrorCovarianceMatrix_P0(const std::vector<Eigen::Vector3d>& initCovariances, const uint8_t& numStates, const bool& imuCharacteristicsIdentical = false);

/// @brief Calculates the process noise matrix Q
/// @param[in] Q Process noise matrix of the previous time step
/// @param[in] dt Time difference between two successive measurements
/// @param[in] processNoiseVariances Vector that contains the variances for each state's process noise
/// @param[in] numStates Number of KF states
/// @param[in] imuCharacteristicsIdentical If the multiple IMUs have the same characteristics, GUI input cells can be reduced considerably
void processNoiseMatrix_Q(Eigen::MatrixXd& Q, const double& dt, const std::vector<Eigen::VectorXd>& processNoiseVariances, const uint8_t& numStates, const bool& imuCharacteristicsIdentical = false);

/// @brief Calculates the state-transition-matrix 𝚽
/// @param[in] Phi State transition matrix from previous iteration. Returns the matrix for the current iteration.
/// @param[in] dt Time difference between two successive measurements
void stateTransitionMatrix_Phi(Eigen::MatrixXd& Phi, const double& dt);

/// @brief Calculates the design matrix H
/// @param[in] DCM_accel Rotation matrix of mounting angles of an accelerometer w.r.t. a common reference
/// @param[in] DCM_gyro Rotation matrix of mounting angles of a gyroscope w.r.t. a common reference
/// @param[in] pinIndex Index of pin to identify sensor
/// @param[in] numMeasurements Number of measurements
/// @param[in] numStates Number of KF states
/// @param[in] numStatesEst Number of estimated states (depends on imuFusionType)
/// @param[in] numStatesPerPin Number of states per pin (biases of accel and gyro)
/// @return Design matrix H
[[nodiscard]] Eigen::MatrixXd designMatrix_H(const Eigen::Matrix3d& DCM_accel,
                                             const Eigen::Matrix3d& DCM_gyro,
                                             const size_t& pinIndex,
                                             const uint8_t& numMeasurements,
                                             const uint8_t& numStates,
                                             const uint8_t& numStatesEst,
                                             const uint8_t& numStatesPerPin);

/// @brief Initializes the IRWKF manually (i.e. using GUI inputs instead of averaging)
/// @param[in] numInputPins Number of input pins of the IMU fusion node
/// @param[in] pinData pin data (i.e. GUI settings for the IRWKF with manual initialization)
/// @param[in] pinDataIRWKF pin data specific to the IRWKF
/// @param[in] numStates Number of states of the IRWKF
/// @param[in] dtInit Initial value for dt (discrete state transition time)
/// @param[in] processNoiseVariances Container for process noise of each state
/// @param[in] kalmanFilter Initial IRWKF object, which has the size defined in the GUI (numStates and numMeasurements)
/// @param[in] imuCharacteristicsIdentical If the multiple IMUs have the same characteristics, GUI input cells can be reduced considerably
/// @param[in] imuBiasesIdentical If the multiple IMUs have the same characteristics, GUI input cells can be reduced considerably
/// @return Initialized IRWKF object
KalmanFilter initializeKalmanFilterManually(const size_t& numInputPins,
                                            const std::vector<PinData>& pinData,
                                            const PinDataIRWKF& pinDataIRWKF,
                                            const uint8_t& numStates,
                                            const double& dtInit,
                                            std::vector<Eigen::VectorXd>& processNoiseVariances,
                                            KalmanFilter& kalmanFilter,
                                            const bool& imuCharacteristicsIdentical,
                                            const bool& imuBiasesIdentical);

/// @brief Initializes the IRWKF automatically, i.e. init values are calculated by averaging the data in the first T seconds
/// @param[in] nInputPins Number of input pins of the IMU fusion node
/// @param[in] pinData pin data (i.e. GUI settings for the IRWKF)
/// @param[in] pinDataIRWKF pin data (i.e. GUI settings specific to the IRWKF)
/// @param[in] cumulatedPinIds Container that collects all pinIds for averaging for auto-init of the KF
/// @param[in] cumulatedImuObs Container that collects all imuObs for averaging for auto-init of the KF
/// @param[in] initJerkAngAcc flag to determine how jerk and angular acceleration states are initialized if '_autoInitKF = true'
/// @param[in] dtInit Initial value for dt (discrete state transition time)
/// @param[in] processNoiseVariances Container for process noise of each state
/// @param[in] numStates Number of states of the IRWKF
/// @param[in] numMeasurements Number of measurements of the IRWKF
/// @param[in] kalmanFilter Initial IRWKF object, which has the size defined in the GUI (numStates and numMeasurements)
/// @return Initialized IRWKF object
KalmanFilter initializeKalmanFilterAuto(const size_t& nInputPins,
                                        const std::vector<PinData>& pinData,
                                        const PinDataIRWKF& pinDataIRWKF,
                                        const std::vector<size_t>& cumulatedPinIds,
                                        const std::vector<std::shared_ptr<const NAV::ImuObs>>& cumulatedImuObs,
                                        const bool& initJerkAngAcc,
                                        const double& dtInit,
                                        const uint8_t& numStates,
                                        const uint8_t& numMeasurements,
                                        std::vector<Eigen::VectorXd>& processNoiseVariances,
                                        KalmanFilter& kalmanFilter);

/// @brief Calculates the mean values of each axis in a vector that contains 3d measurements of a certain sensor type
/// @param[in] sensorType type of measurement, i.e. Acceleration or gyro measurements in 3d (axisIndex / msgIndex)
/// @param[in] containerPos position-Index in 'sensorType' where data starts (e.g. Accel at 0, Gyro at 3)
/// @return Vector of mean values in 3d of a certain sensor type
Eigen::Vector3d mean(const std::vector<std::vector<double>>& sensorType, const size_t& containerPos);

/// @brief Calculates the variance of each axis in a vector that contains 3d measurements of a certain sensor type
/// @param[in] sensorType type of measurement, i.e. Acceleration or gyro measurements in 3d (axisIndex / msgIndex)
/// @param[in] containerPos position-Index in 'sensorType' where data starts (e.g. Accel at 0, Gyro at 3)
/// @return Vector of variance values in 3d of a certain sensor type
Eigen::Vector3d variance(const std::vector<std::vector<double>>& sensorType, const size_t& containerPos);
} // namespace NAV::IRWKF