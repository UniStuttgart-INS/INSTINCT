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

/// @brief Finds the "least squares" solution for the equation \f$ \mathbf{v} = \mathbf{b} - \mathbf{A} \mathbf{x} \f$
///
/// Minimizes the functional
/// \anchor eq-LinearLeastSquares-functional \f{equation}{ \label{eq:eq-LinearLeastSquares-functional}
///   J(\mathbf{x}) \equiv \sum_{i=1}^m v_i^2 = \mathbf{v}^T \mathbf{v} = (\mathbf{b} - \mathbf{A} \mathbf{x})^T (\mathbf{b} - \mathbf{A} \mathbf{x})
/// \f}
/// which has the solution (assuming that the inverse to \f$ \mathbf{A}^T \mathbf{A} \f$ exists)
/// \anchor eq-LinearLeastSquares-solution \f{equation}{ \label{eq:eq-LinearLeastSquares-solution}
///   \mathbf{x} = \left(\mathbf{A}^T \mathbf{A} \right)^{-1} \mathbf{A}^T \mathbf{b}
/// \f}
/// @param[in] A Measurement or design Matrix
/// @param[in] b Measurement vector
/// @return Least squares solution
template<typename DerivedA, typename DerivedB>
Eigen::Vector<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime> solveLinearLeastSquares(const Eigen::MatrixBase<DerivedA>& A, const Eigen::MatrixBase<DerivedB>& b)
{
    return (A.transpose() * A).inverse() * A.transpose() * b;
}

/// @brief Finds the "weighted least squares" solution
///
/// \anchor eq-WeightedLinearLeastSquares \f{equation}{ \label{eq:eq-WeightedLinearLeastSquares}
///   \mathbf{x} = \left(\mathbf{A}^T \mathbf{W} \mathbf{A} \right)^{-1} \mathbf{A}^T \mathbf{W} \mathbf{b}
/// \f}
/// @param[in] A Measurement or design Matrix
/// @param[in] W Weight matrix
/// @param[in] b Measurement vector
/// @return Least squares solution
template<typename DerivedA, typename DerivedW, typename DerivedB>
Eigen::Vector<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime> solveWeightedLinearLeastSquares(const Eigen::MatrixBase<DerivedA>& A, const Eigen::MatrixBase<DerivedW>& W, const Eigen::MatrixBase<DerivedB>& b)
{
    return (A.transpose() * W * A).inverse() * A.transpose() * W * b;
}

/// @brief Finds the "least squares" solution for the equation \f$ \mathbf{v} = \mathbf{b} - \mathbf{A} \mathbf{x} \f$
/// @param[in] A Measurement or design Matrix
/// @param[in] b Measurement vector
/// @return Least squares solution and variance
template<typename DerivedA, typename DerivedB>
LeastSquaresResult<Eigen::Vector<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime>, Eigen::Matrix<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime, DerivedA::ColsAtCompileTime>>
    solveLinearLeastSquaresUncertainties(const Eigen::MatrixBase<DerivedA>& A, const Eigen::MatrixBase<DerivedB>& b)
{
    // Covariance / Cofactor matrix
    Eigen::Matrix<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime, DerivedA::ColsAtCompileTime> Q = (A.transpose() * A).inverse();
    LOG_DATA("Q = \n{}", Q);
    // Least squares solution
    Eigen::Vector<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime> dx = Q * A.transpose() * b;
    LOG_DATA("dx = {}", dx.transpose());

    // Residual vector
    typename DerivedB::PlainObject e = b - A * dx;
    LOG_DATA("e = {}", e.transpose());
    // Residual sum of squares
    double RSS = std::pow(e.norm(), 2);
    LOG_DATA("RSS = {}", RSS);

    // Amount of equations
    auto n = A.rows();
    // Amount of variables
    auto m = A.cols();
    // Statistical degrees of freedom
    auto dof = n - m;
    LOG_DATA("dof = {}", dof);

    // Estimated error variance (reduced chi-squared statistic)
    double sigma2 = RSS / static_cast<double>(dof);
    LOG_DATA("sigma2 = {}", sigma2);

    Q *= sigma2;
    LOG_DATA("variance = \n{}", Q);

    return { .solution = dx, .variance = Q.cwiseAbs() };
}

/// @brief Finds the "weighted least squares" solution
/// @param[in] A Measurement or design Matrix
/// @param[in] W Weight matrix
/// @param[in] b Measurement vector
/// @return Weighted least squares solution and variance
template<typename DerivedA, typename DerivedW, typename DerivedB>
LeastSquaresResult<Eigen::Vector<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime>, Eigen::Matrix<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime, DerivedA::ColsAtCompileTime>>
    solveWeightedLinearLeastSquaresUncertainties(const Eigen::MatrixBase<DerivedA>& A, const Eigen::MatrixBase<DerivedW>& W, const Eigen::MatrixBase<DerivedB>& b)
{
    // Covariance / Cofactor matrix
    Eigen::Matrix<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime, DerivedA::ColsAtCompileTime> Q = (A.transpose() * W * A).inverse();
    LOG_DATA("Q = \n{}", Q);
    // Least squares solution
    Eigen::Vector<typename DerivedA::Scalar, DerivedA::ColsAtCompileTime> dx = Q * A.transpose() * W * b;
    LOG_DATA("dx = {}", dx.transpose());

    // Residual vector
    typename DerivedB::PlainObject e = b - A * dx;
    LOG_DATA("e = {}", e.transpose());
    // Residual sum of squares
    double RSS = e.transpose() * W * e;
    LOG_DATA("RSS = {}", RSS);

    // Amount of equations
    auto n = A.rows();
    // Amount of variables
    auto m = A.cols();
    // Statistical degrees of freedom
    auto dof = n - m;
    LOG_DATA("dof = {}", dof);

    // Estimated error variance (reduced chi-squared statistic)
    double sigma2 = RSS / static_cast<double>(dof);
    LOG_DATA("sigma2 = {}", sigma2);

    Q *= sigma2;
    LOG_DATA("variance = \n{}", Q);

    return { .solution = dx, .variance = Q.cwiseAbs() };
}

} // namespace NAV
