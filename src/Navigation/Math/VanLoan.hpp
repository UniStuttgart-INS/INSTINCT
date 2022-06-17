/// @file VanLoan.hpp
/// @brief Van Loan Method \cite Loan1978
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-01-11

#pragma once

#include <utility>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

namespace NAV
{

/// @brief Numerical Method to calculate the State transition matrix 𝚽 and System/Process noise covariance matrix 𝐐
/// @tparam DerivedF Matrix type of the F matrix
/// @tparam DerivedG Matrix type of the G matrix
/// @tparam DerivedW Matrix type of the W matrix
/// @param[in] F System model matrix
/// @param[in] G Noise model matrix
/// @param[in] W Noise scale factors
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
template<typename DerivedF, typename DerivedG, typename DerivedW>
[[nodiscard]] std::pair<typename DerivedF::PlainObject, typename DerivedF::PlainObject>
    calcPhiAndQWithVanLoanMethod(const Eigen::MatrixBase<DerivedF>& F,
                                 const Eigen::MatrixBase<DerivedG>& G,
                                 const Eigen::MatrixBase<DerivedW>& W,
                                 typename DerivedF::Scalar dt)
{
    //     ┌            ┐
    //     │ -F  ┊ GWG^T│
    // A = │------------│ * dT
    //     │  0  ┊  F^T │
    //     └            ┘
    // W = Power Spectral Density of u (See Brown & Hwang (2012) chapter 3.9, p. 126 - footnote)
    // W = Identity, as noise scale factor is included within G matrix
    Eigen::Matrix<typename DerivedF::Scalar, Eigen::Dynamic, Eigen::Dynamic>
        A = Eigen::Matrix<typename DerivedF::Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(2 * F.rows(), 2 * F.cols());
    A.topLeftCorner(F.rows(), F.cols()) = -F;
    A.topRightCorner(F.rows(), F.cols()) = G * W * G.transpose();
    A.bottomRightCorner(F.rows(), F.cols()) = F.transpose();
    A *= dt;

    // Exponential Matrix of A (https://eigen.tuxfamily.org/dox/unsupported/group__MatrixFunctions__Module.html#matrixbase_exp)
    Eigen::Matrix<typename DerivedF::Scalar, Eigen::Dynamic, Eigen::Dynamic> B = A.exp();

    //               ┌                ┐
    //               │ ... ┊ Phi^-1 Q │
    // B = expm(A) = │----------------│
    //               │  0  ┊   Phi^T  │
    //               └                ┘
    typename DerivedF::PlainObject Phi = B.bottomRightCorner(F.rows(), F.cols()).transpose();
    typename DerivedF::PlainObject Q = Phi * B.topRightCorner(F.rows(), F.cols());

    return { Phi, Q };
}

} // namespace NAV
