// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file LeastSquares.hpp
/// @brief Least Squares Algorithm
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-05-04

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include "util/Logger.hpp"

namespace NAV
{

/// @brief Least Squares Uncertainties return value
template<typename SolutionVector, typename VarianceMatrix>
struct LeastSquaresResult
{
    SolutionVector solution; ///< Least squares solution
    VarianceMatrix variance; ///< Least squares variance
};

/// @brief Finds the "least squares" solution for the equation \f$ \mathbf{v} = \mathbf{dz} - \mathbf{H} \mathbf{x} \f$
///
/// Minimizes the functional
/// \anchor eq-LinearLeastSquares-functional \f{equation}{ \label{eq:eq-LinearLeastSquares-functional}
///   J(\mathbf{x}) \equiv \sum_{i=1}^m v_i^2 = \mathbf{v}^T \mathbf{v} = (\mathbf{dz} - \mathbf{H} \mathbf{x})^T (\mathbf{dz} - \mathbf{H} \mathbf{x})
/// \f}
/// which has the solution (assuming that the inverse to \f$ \mathbf{H}^T \mathbf{H} \f$ exists)
/// \anchor eq-LinearLeastSquares-solution \f{equation}{ \label{eq:eq-LinearLeastSquares-solution}
///   \mathbf{x} = \left(\mathbf{H}^T \mathbf{H} \right)^{-1} \mathbf{H}^T \mathbf{dz}
/// \f}
/// @param[in] H Design Matrix
/// @param[in] dz Residual vector
/// @return Least squares solution
template<typename DerivedA, typename DerivedB>
Eigen::Vector<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime> solveLinearLeastSquares(const Eigen::MatrixBase<DerivedA>& H, const Eigen::MatrixBase<DerivedB>& dz)
{
    return (H.transpose() * H).inverse() * H.transpose() * dz;
}

/// @brief Finds the "weighted least squares" solution
///
/// \anchor eq-WeightedLinearLeastSquares \f{equation}{ \label{eq:eq-WeightedLinearLeastSquares}
///   \mathbf{x} = \left(\mathbf{H}^T \mathbf{W} \mathbf{H} \right)^{-1} \mathbf{H}^T \mathbf{W} \mathbf{dz}
/// \f}
/// @param[in] H Design Matrix
/// @param[in] W Weight matrix
/// @param[in] dz Residual vector
/// @return Least squares solution
template<typename DerivedA, typename DerivedW, typename DerivedB>
Eigen::Vector<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime> solveWeightedLinearLeastSquares(const Eigen::MatrixBase<DerivedA>& H, const Eigen::MatrixBase<DerivedW>& W, const Eigen::MatrixBase<DerivedB>& dz)
{
    return (H.transpose() * W * H).inverse() * H.transpose() * W * dz;
}

/// @brief Finds the "least squares" solution for the equation \f$ \mathbf{v} = \mathbf{dz} - \mathbf{H} \mathbf{x} \f$
/// @param[in] H Design Matrix
/// @param[in] dz Residual vector
/// @return Least squares solution and variance
template<typename DerivedA, typename DerivedB>
LeastSquaresResult<Eigen::Vector<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime>, Eigen::Matrix<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime, DerivedA::ColsAtCompileTime>>
    solveLinearLeastSquaresUncertainties(const Eigen::MatrixBase<DerivedA>& H, const Eigen::MatrixBase<DerivedB>& dz)
{
    // Cofactor matrix
    Eigen::Matrix<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime, DerivedA::ColsAtCompileTime> Q = (H.transpose() * H).inverse();
    LOG_DATA("Q = \n{}", Q);
    // Least squares solution
    Eigen::Vector<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime> dx = Q * H.transpose() * dz;
    LOG_DATA("dx = {}", dx.transpose());

    // Residual sum of squares
    double RSS = std::pow(dz.norm(), 2);
    LOG_DATA("RSS = {}", RSS);

    // Amount of equations
    auto n = H.rows();
    // Amount of variables
    auto m = H.cols();
    // Statistical degrees of freedom
    auto dof = n - m;
    LOG_DATA("dof = {}", dof);

    // Estimated error variance (reduced chi-squared statistic)
    double sigma2 = RSS / static_cast<double>(dof);
    LOG_DATA("sigma2 = {}", sigma2);

    // Covariance matrix
    Q *= sigma2;
    LOG_DATA("variance = \n{}", Q);

    return { .solution = dx, .variance = Q.cwiseAbs() };
}

/// @brief Finds the "weighted least squares" solution
/// @param[in] H Design Matrix
/// @param[in] W Weight matrix
/// @param[in] dz Residual vector
/// @return Weighted least squares solution and variance
template<typename DerivedA, typename DerivedW, typename DerivedB>
LeastSquaresResult<Eigen::Vector<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime>, Eigen::Matrix<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime, DerivedA::ColsAtCompileTime>>
    solveWeightedLinearLeastSquaresUncertainties(const Eigen::MatrixBase<DerivedA>& H, const Eigen::MatrixBase<DerivedW>& W, const Eigen::MatrixBase<DerivedB>& dz)
{
    // Cofactor matrix
    Eigen::Matrix<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime, DerivedA::ColsAtCompileTime> Q = (H.transpose() * W * H).inverse();
    LOG_DATA("Cofactor matrix = \n{}", Q);
    // Least squares solution
    Eigen::Vector<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime> dx = Q * H.transpose() * W * dz;
    LOG_DATA("dx = {}", dx.transpose());

    // Residual sum of squares
    double RSS = dz.transpose() * W * dz;
    LOG_DATA("RSS = {}", RSS);

    // Amount of equations
    auto n = H.rows();
    // Amount of variables
    auto m = H.cols();
    // Statistical degrees of freedom
    auto dof = n - m;
    LOG_DATA("dof = {}", dof);

    // Estimated error variance (reduced chi-squared statistic)
    double sigma2 = RSS / static_cast<double>(dof);
    LOG_DATA("sigma2 = {}", sigma2);

    // Covariance matrix
    Q *= sigma2;
    LOG_DATA("Covariance matrix = \n{}", Q);

    return { .solution = dx, .variance = Q.cwiseAbs() };
}

} // namespace NAV
