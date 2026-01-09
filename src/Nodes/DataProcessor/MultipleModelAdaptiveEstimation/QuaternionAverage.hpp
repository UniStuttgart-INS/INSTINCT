// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file QuaternionAverage.hpp
/// @brief Calculates the weighted average of quaternions
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2025-10-01

#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>

namespace NAV
{
/// @brief Calculates the weighted average of a quaternion
/// @param[in] quaternions Quaternions that are to be averaged
/// @param[in] weights Weights of the respective quaternions
/// @param[in] maxEigenValue Maximum eigenvalue
/// @return Averaged quaternion
/// @note See "Averaging Quaternions" from Markley et al. (2007)
Eigen::Matrix<std::complex<double>, 4, 1> calcQuaternionAverage(std::vector<Eigen::Quaterniond>& quaternions, std::vector<double>& weights, double& maxEigenValue);

/// @brief Calculates the error covariance of the weighted average of a quaternion
/// @param[in] quaternions Quaternions that are to be averaged
/// @param[in] weights Weights of the single Kalman filters
/// @param[in] sigmaQ Error covariance of the attitude quaternion
/// @param[in] maxEigenValue Maximum Eigenvalue
/// @param[in] eigenVector EigenVector of the maximum Eigenvalue
/// @return Error covariance of the weighted average of a quaternion
/// @note See "A tutorial on SE(3) transformation parameterizations and on-manifold optimization" from José Luis Blanco Claraco (2022)
/// @attention While it is entirely valid to calculate the average uncertainty of a quaternion using this function, there is a problem if using it in combination with non-quaternion states. This is regarding the calculation of the covariances between the quaternion and the other states
Eigen::Matrix4d calcQuaternionAverageUncertainty(std::vector<Eigen::Quaterniond>& quaternions, std::vector<double>& weights, std::vector<Eigen::Matrix4d>& sigmaQ, double& maxEigenValue, Eigen::Matrix<std::complex<double>, 4, 1>& eigenVector);
} // namespace NAV