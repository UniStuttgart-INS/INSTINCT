// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SppLeastSquares.hpp
/// @brief SPP with a Least Squares Estimator
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-07-21

#pragma once

#include "Navigation/GNSS/Positioning/SppAlgorithm.hpp"
#include "NodeData/GNSS/SppSolution.hpp"
#include "util/Eigen.hpp"
#include "util/Container/UncertainValue.hpp"

#include <unordered_map>
#include <memory>

namespace NAV::GNSS::Positioning::SPP
{

/// @brief Function to call to set the position or velocity and standard deviation in the sppSol
using SppSolSetPosOrVelAndStdDev_e = void (SppSolution::*)(const Eigen::Vector3d&, const Eigen::Matrix3d&);
/// @brief Function to call to set the position or velocity in the sppSol
using SppSolSetPosOrVel_e = void (SppSolution::*)(const Eigen::Vector3d&);
/// @brief Function to call to set the covariance matrix in the sppSol
using SppSolSetErrorCovarianceMatrix = void (SppSolution::*)(const Eigen::MatrixXd&);

/// @brief Compute the linear least squares estimates and uncertainties and assign the solution
/// @param[in] dy Innovation of pseudoranges or pseudorange rates
/// @param[in] e_H Corresponding design matrix
/// @param[in] W Corresponding weights of observations
/// @param[in] estimatorType Estimator Type, either LSQ or WLSQ
/// @param[in] nMeas Number of measurements
/// @param[in] interSysErrOrDrifts Inter-system clock error or drift keys
/// @param[out] posOrVel Output for the position or velocity
/// @param[out] statesPosOrVel State keys for position or velocity
/// @param[out] biasOrDrift Output for the bias or drift
/// @param[out] statesBiasOrDrift State key for receiver clock error or drift
/// @param[out] sysTimeOrDriftDiff Output for output for the inter system bias or drifts
/// @param[out] sppSol SPP solution
/// @param[out] sppSolBiasOrDrift Output for the bias or drift (assigned inside the sppSol)
/// @param[out] sppSolSysTimeOrDriftDiff Output for output for the inter system bias or drifts (assigned inside the sppSol)
/// @param[out] sppSolSetPosOrVelAndStdDev_e Function to call to set the position or velocity and standard deviation in the sppSol
/// @param[out] sppSolSetPosOrVel_e Function to call to set the position or velocity in the sppSol
/// @param[out] sppSolSetErrorCovarianceMatrix Function to call to set the covariance matrix in the sppSol
/// @return True if the solution change is below 1e-4
bool solveLeastSquaresAndAssignSolution(const KeyedVectorXd<Meas::MeasKeyTypes>& dy,
                                        const KeyedMatrixXd<Meas::MeasKeyTypes, States::StateKeyTypes>& e_H,
                                        const KeyedMatrixXd<Meas::MeasKeyTypes, Meas::MeasKeyTypes>& W,
                                        const EstimatorType& estimatorType,
                                        size_t nMeas,
                                        const std::vector<States::StateKeyTypes>& interSysErrOrDrifts,
                                        Eigen::Vector3d& posOrVel, std::vector<States::StateKeyTypes> statesPosOrVel,
                                        UncertainValue<double>& biasOrDrift, States::StateKeyTypes statesBiasOrDrift,
                                        std::unordered_map<NAV::SatelliteSystem, NAV::UncertainValue<double>>& sysTimeOrDriftDiff,
                                        const std::shared_ptr<SppSolution>& sppSol,
                                        UncertainValue<double>& sppSolBiasOrDrift,
                                        std::unordered_map<NAV::SatelliteSystem, NAV::UncertainValue<double>>& sppSolSysTimeOrDriftDiff,
                                        SppSolSetPosOrVelAndStdDev_e sppSolSetPosOrVelAndStdDev_e,
                                        SppSolSetPosOrVel_e sppSolSetPosOrVel_e,
                                        SppSolSetErrorCovarianceMatrix sppSolSetErrorCovarianceMatrix);

} // namespace NAV::GNSS::Positioning::SPP