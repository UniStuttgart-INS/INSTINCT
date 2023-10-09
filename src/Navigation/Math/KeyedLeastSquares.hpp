// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file KeyedLeastSquares.hpp
/// @brief Least squares with keyed states
/// @author P. Peitschat (HiWi)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-26

#pragma once

#include "util/Container/KeyedMatrix.hpp"

namespace NAV
{
/// @brief Least Squares Uncertainties return value
template<typename SolutionVector, typename VarianceMatrix>
struct KeyedLeastSquaresResult
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
template<typename Scalar, typename StateKeyType, typename MeasKeyType>
KeyedVectorX<Scalar, StateKeyType> solveLinearLeastSquares(const KeyedMatrixX<Scalar, MeasKeyType, StateKeyType>& H, const KeyedVectorX<Scalar, MeasKeyType>& dz)
{
    KeyedVectorX<Scalar, StateKeyType> dx = KeyedVectorX<Scalar, StateKeyType>(Eigen::VectorX<Scalar>::Zero(H.cols()), H.colKeys());
    dx(all) = (H(all, all).transpose() * H(all, all)).inverse() * H(all, all).transpose() * dz(all);
    return dx;
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
template<typename Scalar, typename StateKeyType, typename MeasKeyType>
KeyedVectorX<Scalar, StateKeyType> solveWeightedLinearLeastSquares(const KeyedMatrixX<Scalar, MeasKeyType, StateKeyType>& H, const KeyedMatrixX<Scalar, MeasKeyType, MeasKeyType>& W, const KeyedVectorX<Scalar, MeasKeyType>& dz)
{
    KeyedVectorX<Scalar, StateKeyType> dx = KeyedVectorX<Scalar, StateKeyType>(Eigen::VectorX<Scalar>::Zero(H.cols()), H.colKeys());
    dx(all) = (H(all, all).transpose() * W(all, all) * H(all, all)).inverse() * H(all, all).transpose() * W(all, all) * dz(all);
    return dx;
}

/// @brief Finds the "least squares" solution for the equation \f$ \mathbf{v} = \mathbf{dz} - \mathbf{H} \mathbf{x} \f$
/// @param[in] H Design Matrix
/// @param[in] dz Residual vector
/// @return Least squares solution and variance
template<typename Scalar, typename StateKeyType, typename MeasKeyType>
KeyedLeastSquaresResult<KeyedVectorX<Scalar, StateKeyType>, KeyedMatrixX<Scalar, StateKeyType, StateKeyType>>
    solveLinearLeastSquaresUncertainties(const KeyedMatrixX<Scalar, MeasKeyType, StateKeyType>& H, const KeyedVectorX<Scalar, MeasKeyType>& dz)
{
    // Amount of equations
    auto m = static_cast<int>(H.rows());
    // Amount of variables
    auto n = static_cast<int>(H.cols());

    // Cofactor matrix
    KeyedMatrixX<Scalar, StateKeyType, StateKeyType> Q = KeyedMatrixX<Scalar, StateKeyType, StateKeyType>(Eigen::MatrixX<Scalar>::Zero(n, n), H.colKeys(), H.colKeys());
    Q(all, all) = (H(all, all).transpose() * H(all, all)).inverse();
    LOG_DATA("Q = \n{}", Q(all, all));
    // Least squares solution
    KeyedVectorX<Scalar, StateKeyType> dx = KeyedVectorX<Scalar, StateKeyType>(Eigen::VectorX<Scalar>::Zero(n), H.colKeys());
    dx(all) = Q(all, all) * H(all, all).transpose() * dz(all);
    LOG_DATA("dx = {}", dx(all).transpose());

    // Residual sum of squares
    double RSS = std::pow(dz(all).norm(), 2);
    LOG_DATA("RSS = {}", RSS);

    // Statistical degrees of freedom
    auto dof = m - n;
    LOG_DATA("dof = {}", dof);

    // Estimated error variance (reduced chi-squared statistic)
    double sigma2 = RSS / static_cast<double>(dof);
    LOG_DATA("sigma2 = {}", sigma2);

    // Covariance matrix
    Q(all, all) *= sigma2;
    LOG_DATA("variance = \n{}", Q(all, all));

    return { .solution = dx, .variance = Q };
}

/// @brief Finds the "weighted least squares" solution
/// @param[in] H Design Matrix
/// @param[in] W Weight matrix
/// @param[in] dz Residual vector
/// @return Weighted least squares solution and variance
template<typename Scalar, typename StateKeyType, typename MeasKeyType>
KeyedLeastSquaresResult<KeyedVectorX<Scalar, StateKeyType>, KeyedMatrixX<Scalar, StateKeyType, StateKeyType>>
    solveWeightedLinearLeastSquaresUncertainties(const KeyedMatrixX<Scalar, MeasKeyType, StateKeyType>& H, const KeyedMatrixX<Scalar, MeasKeyType, MeasKeyType>& W, const KeyedVectorX<Scalar, MeasKeyType>& dz)
{
    // Amount of equations
    auto m = static_cast<int>(H.rows());
    // Amount of variables
    auto n = static_cast<int>(H.cols());

    // Cofactor matrix
    KeyedMatrixX<Scalar, StateKeyType, StateKeyType> Q = KeyedMatrixX<Scalar, StateKeyType, StateKeyType>(Eigen::MatrixX<Scalar>::Zero(n, n), H.colKeys(), H.colKeys());
    Q(all, all) = (H(all, all).transpose() * W(all, all) * H(all, all)).inverse();
    LOG_DATA("Q = \n{}", Q(all, all));

    // Least squares solution
    KeyedVectorX<Scalar, StateKeyType> dx = KeyedVectorX<Scalar, StateKeyType>(Eigen::VectorX<Scalar>::Zero(n), H.colKeys());
    dx(all) = Q(all, all) * H(all, all).transpose() * W(all, all) * dz(all);
    LOG_DATA("dx = {}", dx(all).transpose());

    // Residual sum of squares
    double RSS = dz(all).transpose() * W(all, all) * dz(all);
    LOG_DATA("RSS = {}", RSS);

    // Statistical degrees of freedom
    auto dof = m - n;
    LOG_DATA("dof = {}", dof);

    // Estimated error variance (reduced chi-squared statistic)
    double sigma2 = RSS / static_cast<double>(dof);
    LOG_DATA("sigma2 = {}", sigma2);

    // Covariance matrix
    Q(all, all) *= sigma2;
    LOG_DATA("Covariance matrix = \n{}", Q(all, all));

    return { .solution = dx, .variance = Q };
}

} // namespace NAV