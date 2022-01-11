/// @file VanLoan.hpp
/// @brief Van Loan Method \cite Loan1978
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-01-11

#pragma once

#include <utility>
#include <Eigen/Core>

namespace NAV
{

/// @brief Numerical Method to calculate the State transition matrix 𝚽 and System/Process noise covariance matrix 𝐐
/// @tparam _Scalar Data type of the Matrix
/// @tparam _Dim Dimension of the square matrix F
/// @tparam _ColsG Columns of the matrix G
/// @param[in] F System model matrix
/// @param[in] G Noise model matrix (includes scale factors)
/// @return A pair with the matrices {𝚽, 𝐐}
///
/// @note See C.F. van Loan (1978) - Computing Integrals Involving the Matrix Exponential \cite Loan1978
template<typename _Scalar, int _Dim, int _ColsG>
[[nodiscard]] std::pair<Eigen::Matrix<_Scalar, _Dim, _Dim>, Eigen::Matrix<_Scalar, _Dim, _Dim>>
    calcPhiAndQWithVanLoanMethod(const Eigen::Matrix<_Scalar, _Dim, _Dim>& F, const Eigen::Matrix<_Scalar, _Dim, _ColsG>& G, _Scalar dt)
{
    //     ┌            ┐
    //     │ -F  ┊ GWG^T│
    // A = │------------│ * dT
    //     │  0  ┊  F^T │
    //     └            ┘
    // W = Power Spectral Density of u (See Brown & Hwang (2012) chapter 3.9, p. 126 - footnote)
    // W = Identity, as noise scale factor is included within G matrix
    Eigen::Matrix<_Scalar, 2 * _Dim, 2 * _Dim> A = Eigen::Matrix<_Scalar, 2 * _Dim, 2 * _Dim>::Zero();
    A.template topLeftCorner<_Dim, _Dim>() = -F; // template keyword: http://eigen.tuxfamily.org/dox-devel/TopicTemplateKeyword.html
    A.template topRightCorner<_Dim, _Dim>() = G /* * W */ * G.transpose();
    A.template bottomRightCorner<_Dim, _Dim>() = F.transpose();
    A *= dt;

    // Exponential Matrix of A (https://eigen.tuxfamily.org/dox/unsupported/group__MatrixFunctions__Module.html#matrixbase_exp)
    Eigen::Matrix<_Scalar, 2 * _Dim, 2 * _Dim> B = A.exp();

    //               ┌                ┐
    //               │ ... ┊ Phi^-1 Q │
    // B = expm(A) = │----------------│
    //               │  0  ┊   Phi^T  │
    //               └                ┘
    Eigen::Matrix<_Scalar, _Dim, _Dim> Phi = B.template bottomRightCorner<_Dim, _Dim>().transpose();
    Eigen::Matrix<_Scalar, _Dim, _Dim> Q = Phi * B.template topRightCorner<_Dim, _Dim>();

    return { Phi, Q };
}

} // namespace NAV
