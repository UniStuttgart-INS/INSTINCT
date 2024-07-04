// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file BsplineKF.hpp
/// @brief Kalman filter matrices for the B-spline Multi-IMU fusion
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2024-04-02

#pragma once

#include "util/Eigen.hpp"
#include "../PinData.hpp"
#include "Navigation/Math/KalmanFilter.hpp"

#include <vector>

namespace NAV::BsplineKF
{
/// @brief Initial error covariance matrix P_0
/// @param[in] initCovariances Initial covariances of the states
/// @param[in] numStates Number of KF states
/// @param[in] imuCharacteristicsIdentical If the multiple IMUs have the same characteristics, GUI input cells can be reduced considerably
/// @return The initial P matrix, i.e. P_0, which contains the initial state variances (numStates) x (numStates)
[[nodiscard]] Eigen::MatrixXd initialErrorCovarianceMatrix_P0(const std::vector<Eigen::VectorXd>& initCovariances, const uint8_t& numStates, const bool& imuCharacteristicsIdentical = false);

/// @brief Calculates the process noise matrix Q
/// @param[in] Q Process noise matrix of the previous time step
/// @param[in] dt Time difference between two successive measurements
/// @param[in] processNoiseVariances Vector that contains the variances for each state's process noise
/// @param[in] numStates Number of KF states
/// @param[in] imuCharacteristicsIdentical If the multiple IMUs have the same characteristics, GUI input cells can be reduced considerably
void processNoiseMatrix_Q(Eigen::MatrixXd& Q, const double& dt, const std::vector<Eigen::VectorXd>& processNoiseVariances, const uint8_t& numStates, const bool& imuCharacteristicsIdentical = false);

/// @brief Rotates the B-spline coefficient states in the state vector x, once a new B-spline is introduced
/// @param[in] x Old state vector
void rotateCoeffStates(Eigen::MatrixXd& x);

/// @brief Rotates the B-spline coefficient error covariances in P, once a new B-spline is introduced
/// @param[in] P Old error covariance matrix
/// @param[in] numStates Number of states of the B-spline KF
/// @param[in] sigmaScalingFactorAngRate Scaling factor relating the error covariance of the newly introduced B-spline coefficients for angular rate to their old error covariance
/// @param[in] sigmaScalingFactorAccel Scaling factor relating the error covariance of the newly introduced B-spline coefficients for acceleration to their old error covariance
void rotateErrorCovariances(Eigen::MatrixXd& P,
                            uint8_t& numStates,
                            const double& sigmaScalingFactorAngRate = 3.0,
                            const double& sigmaScalingFactorAccel = 3.0);

/// @brief Calculates the design matrix H
/// @param[in] ti Current point in time
/// @param[in] splineSpacing distance - in time - between two splines of the stacked B-spline
/// @param[in] DCM_accel Rotation matrix of mounting angles of an accelerometer w.r.t. a common reference
/// @param[in] DCM_gyro Rotation matrix of mounting angles of a gyroscope w.r.t. a common reference
/// @param[in] pinIndex Index of pin to identify sensor
/// @param[in] numMeasurements Number of measurements
/// @param[in] numStates Number of KF states
/// @param[in] numStatesEst Number of estimated states (depends on imuFusionType)
/// @param[in] numStatesPerPin Number of states per pin (biases of accel and gyro)
/// @return Design matrix H
[[nodiscard]] Eigen::MatrixXd designMatrix_H(const double& ti,
                                             const double& splineSpacing,
                                             const Eigen::Matrix3d& DCM_accel,
                                             const Eigen::Matrix3d& DCM_gyro,
                                             const size_t& pinIndex,
                                             const uint8_t& numMeasurements,
                                             const uint8_t& numStates,
                                             const uint8_t& numStatesEst,
                                             const uint8_t& numStatesPerPin);

/// @brief Initializes the BsplineKF manually (i.e. using GUI inputs instead of averaging)
/// @param[in] numInputPins Number of input pins of the IMU fusion node
/// @param[in] pinData pin data (i.e. GUI settings for the BsplineKF with manual initialization)
/// @param[in] pinDataBsplineKF pin data specific to the B-spline-KF
/// @param[in] numStates Number of states of the BsplineKF
/// @param[in] dtInit Initial value for dt (discrete state transition time)
/// @param[in] processNoiseVariances Container for process noise of each state
/// @param[in] kalmanFilter Initial BsplineKF object, which has the size defined in the GUI (numStates and numMeasurements)
/// @param[in] imuCharacteristicsIdentical If the multiple IMUs have the same characteristics, GUI input cells can be reduced considerably
/// @param[in] imuBiasesIdentical If the multiple IMUs have the same characteristics, GUI input cells can be reduced considerably
/// @return Initialized BsplineKF object
KalmanFilter initializeKalmanFilterManually(const size_t& numInputPins,
                                            const std::vector<PinData>& pinData,
                                            const PinDataBsplineKF& pinDataBsplineKF,
                                            const uint8_t& numStates,
                                            const double& dtInit,
                                            std::vector<Eigen::VectorXd>& processNoiseVariances,
                                            KalmanFilter& kalmanFilter,
                                            const bool& imuCharacteristicsIdentical,
                                            const bool& imuBiasesIdentical);

} // namespace NAV::BsplineKF