/// @file VanLoan.hpp
/// @brief Van Loan Method \cite Loan1978
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-01-11

#pragma once

#include <utility>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#include "util/Logger.hpp"

namespace NAV
{

/// @brief Numerical Method to calculate the State transition matrix 𝚽 and System/Process noise covariance matrix 𝐐
/// @tparam _Scalar Data type of the Matrix
/// @tparam _Dim Dimension of the square matrix F
/// @tparam _ColsG Columns of the matrix G
/// @param[in] F System model matrix
/// @param[in] G Noise model matrix (includes scale factors)
/// @param[in] dt Time step in [s]
/// @return A pair with the matrices {𝚽, 𝐐}
///
/// 1. Form a \f$ 2n \times 2n \f$ matrix called \f$ \mathbf{A} \f$ (\f$ n \f$ is the dimension of \f$ \mathbf{x} \f$ and \f$ \mathbf{W} \f$ is the power spectral density of the noise \f$ W(t) \f$)
/// \anchor eq-Loan-A \f{equation}{ \label{eq:eq-Loan-A}
///   \mathbf{A} = \begin{bmatrix} -\mathbf{F} & \mathbf{G} \mathbf{W} \mathbf{G} \\
///                                 \mathbf{0} &            \mathbf{F}^T          \end{bmatrix} \Delta t
/// \f}
///
/// 2. Calculate the exponential of \f$ \mathbf{A} \f$
/// \anchor eq-Loan-B \f{equation}{ \label{eq:eq-Loan-B}
///   \mathbf{B} = \text{expm}(\mathbf{A}) = \left[ \begin{array}{c;{2pt/2pt}c}
///                                             \dots          & \mathbf{\Phi}^{-1} \mathbf{Q} \\[2mm]
///                                             \hdashline[2pt/2pt] &                                  \\[-2mm]
///                                             \mathbf{0}          & \mathbf{\Phi}^T                     \end{array} \right]
///                                        = \begin{bmatrix} \mathbf{B}_{11} & \mathbf{B}_{12} \\
///                                                          \mathbf{B}_{21} & \mathbf{B}_{22} \end{bmatrix}
/// \f}
///
/// 3. Calculate the state transition matrix \f$ \mathbf{\Phi} \f$ as
/// \anchor eq-Loan-Phi \f{equation}{ \label{eq:eq-Loan-Phi}
///   \mathbf{\Phi} = \mathbf{B}_{22}^T
/// \f}
///
/// 4. Calculate the process noise covariance matrix \f$ \mathbf{Q} \f$ as
/// \anchor eq-Loan-Q \f{equation}{ \label{eq:eq-Loan-Q}
///   \mathbf{Q} = \mathbf{\Phi} \mathbf{B}_{12}
/// \f}
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

    LOG_DATA("Phi =\n{}", Phi);
    LOG_DATA("Q =\n{}", Q);

    return { Phi, Q };
}

} // namespace NAV
