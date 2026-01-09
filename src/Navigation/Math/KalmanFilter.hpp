// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file KalmanFilter.hpp
/// @brief Generalized Kalman Filter class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-11-25

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include "Navigation/Math/Math.hpp"

namespace NAV
{
/// @brief Generalized Kalman Filter class
class KalmanFilter
{
  public:
    /// @brief Constructor
    /// @param[in] n Number of States
    /// @param[in] m Number of Measurements
    KalmanFilter(int n, int m)
    {
        // x̂ State vector
        x = Eigen::MatrixXd::Zero(n, 1);

        // 𝐏 Error covariance matrix
        P = Eigen::MatrixXd::Zero(n, n);

        // 𝚽 State transition matrix
        Phi = Eigen::MatrixXd::Zero(n, n);

        // 𝐐 System/Process noise covariance matrix
        Q = Eigen::MatrixXd::Zero(n, n);

        // 𝐳 Measurement vector
        z = Eigen::MatrixXd::Zero(m, 1);

        // 𝐇 Measurement sensitivity Matrix
        H = Eigen::MatrixXd::Zero(m, n);

        // 𝐑 = 𝐸{𝐰ₘ𝐰ₘᵀ} Measurement noise covariance matrix
        R = Eigen::MatrixXd::Zero(m, m);

        // 𝗦 Measurement prediction covariance matrix
        S = Eigen::MatrixXd::Zero(m, m);

        // 𝐊 Kalman gain matrix
        K = Eigen::MatrixXd::Zero(n, m);

        // 𝑰 Identity Matrix
        I = Eigen::MatrixXd::Identity(n, n);
    }

    /// @brief Default constructor
    KalmanFilter() = delete;

    /// @brief Sets all Vectors and matrices to 0
    void setZero()
    {
        // x̂ State vector
        x.setZero();

        // 𝐏 Error covariance matrix
        P.setZero();

        // 𝚽 State transition matrix
        Phi.setZero();

        // 𝐐 System/Process noise covariance matrix
        Q.setZero();

        // 𝐳 Measurement vector
        z.setZero();

        // 𝐇 Measurement sensitivity Matrix
        H.setZero();

        // 𝐑 = 𝐸{𝐰ₘ𝐰ₘᵀ} Measurement noise covariance matrix
        R.setZero();

        // 𝗦 Measurement prediction covariance matrix
        S.setZero();

        // 𝐊 Kalman gain matrix
        K.setZero();
    }

    /// @brief Do a Time Update
    /// @attention Update the State transition matrix (𝚽) and the Process noise covariance matrix (𝐐) before calling this
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 3.2.2)
    void predict()
    {
        // Math: \mathbf{\hat{x}}_k^- = \mathbf{\Phi}_{k-1}\mathbf{\hat{x}}_{k-1}^+ \qquad \text{P. Groves}\,(3.14)
        x = Phi * x;

        // Math: \mathbf{P}_k^- = \mathbf{\Phi}_{k-1} P_{k-1}^+ \mathbf{\Phi}_{k-1}^T + \mathbf{Q}_{k-1} \qquad \text{P. Groves}\,(3.15)
        P = Phi * P * Phi.transpose() + Q;
    }

    /// @brief Do a Measurement Update with a Measurement 𝐳
    /// @attention Update the Measurement sensitivity Matrix (𝐇), the Measurement noise covariance matrix (𝐑)
    ///            and the Measurement vector (𝐳) before calling this
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 3.2.2)
    void correct()
    {
        S = H * P * H.transpose() + R;

        // Math: \mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T (\mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + R_k)^{-1} \qquad \text{P. Groves}\,(3.21)
        K = P * H.transpose() * S.inverse();

        // Math: \begin{align*} \mathbf{\hat{x}}_k^+ &= \mathbf{\hat{x}}_k^- + \mathbf{K}_k (\mathbf{z}_k - \mathbf{H}_k \mathbf{\hat{x}}_k^-) \\ &= \mathbf{\hat{x}}_k^- + \mathbf{K}_k \mathbf{\delta z}_k^{-} \end{align*} \qquad \text{P. Groves}\,(3.24)
        x = x + K * (z - H * x);

        // Math: \mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- \qquad \text{P. Groves}\,(3.25)
        P = (I - K * H) * P;
    }

    /// @brief Do a Measurement Update with a Measurement Innovation 𝜹𝐳
    /// @attention Update the Measurement sensitivity Matrix (𝐇), the Measurement noise covariance matrix (𝐑)
    ///            and the Measurement vector (𝐳) before calling this
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 3.2.2)
    /// @note See Brown & Hwang (2012) - Introduction to Random Signals and Applied Kalman Filtering (ch. 5.5 - figure 5.5)
    void correctWithMeasurementInnovation()
    {
        S = H * P * H.transpose() + R;

        // Math: \mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T (\mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + R_k)^{-1} \qquad \text{P. Groves}\,(3.21)
        K = P * H.transpose() * S.inverse();

        // Math: \begin{align*} \mathbf{\hat{x}}_k^+ &= \mathbf{\hat{x}}_k^- + \mathbf{K}_k (\mathbf{z}_k - \mathbf{H}_k \mathbf{\hat{x}}_k^-) \\ &= \mathbf{\hat{x}}_k^- + \mathbf{K}_k \mathbf{\delta z}_k^{-} \end{align*} \qquad \text{P. Groves}\,(3.24)
        x = x + K * z;

        // Math: \mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- \qquad \text{P. Groves}\,(3.25)
        // P = (I - K * H) * P;

        // Math: \mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T \qquad \text{Brown & Hwang}\,(p. 145, eq. 4.2.11)
        P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();
    }

    /// @brief Updates the state transition matrix 𝚽 limited to first order in 𝐅𝜏ₛ
    /// @param[in] F System Matrix
    /// @param[in] tau_s time interval in [s]
    /// @note See Groves (2013) chapter 14.2.4, equation (14.72)
    static Eigen::MatrixXd transitionMatrix(const Eigen::MatrixXd& F, double tau_s)
    {
        // Transition matrix 𝚽
        return Eigen::MatrixXd::Identity(F.rows(), F.cols()) + F * tau_s;
    }

    /// x̂ State vector
    Eigen::MatrixXd x;

    /// 𝐏 Error covariance matrix
    Eigen::MatrixXd P;

    /// 𝚽 State transition matrix
    Eigen::MatrixXd Phi;

    /// 𝐐 System/Process noise covariance matrix
    Eigen::MatrixXd Q;

    /// 𝐳 Measurement vector
    Eigen::MatrixXd z;

    /// 𝐇 Measurement sensitivity Matrix
    Eigen::MatrixXd H;

    /// 𝐑 = 𝐸{𝐰ₘ𝐰ₘᵀ} Measurement noise covariance matrix
    Eigen::MatrixXd R;

    /// 𝗦 Measurement prediction covariance matrix
    Eigen::MatrixXd S;

    /// 𝐊 Kalman gain matrix
    Eigen::MatrixXd K;

  private:
    /// 𝑰 Identity Matrix (n x n)
    Eigen::MatrixXd I;
};

/// @brief Calculates the state transition matrix 𝚽 limited to specified order in 𝐅𝜏ₛ
/// @param[in] F System Matrix
/// @param[in] tau_s time interval in [s]
/// @param[in] order The order of the Taylor polynom to calculate
/// @note See \cite Groves2013 Groves, ch. 3.2.3, eq. 3.34, p. 98
template<typename Derived>
typename Derived::PlainObject transitionMatrix_Phi_Taylor(const Eigen::MatrixBase<Derived>& F, double tau_s, size_t order)
{
    // Transition matrix 𝚽
    typename Derived::PlainObject Phi;

    if constexpr (Derived::RowsAtCompileTime == Eigen::Dynamic)
    {
        Phi = Eigen::MatrixBase<Derived>::Identity(F.rows(), F.cols());
    }
    else
    {
        Phi = Eigen::MatrixBase<Derived>::Identity();
    }
    // std::cout << "Phi = I";

    for (size_t i = 1; i <= order; i++)
    {
        typename Derived::PlainObject Fpower = F;
        // std::cout << " + (F";

        for (size_t j = 1; j < i; j++) // F^j
        {
            // std::cout << "*F";
            Fpower *= F;
        }
        // std::cout << "*tau_s^" << i << ")";
        // std::cout << "/" << math::factorial(i);
        Phi += (Fpower * std::pow(tau_s, i)) / math::factorial(i);
    }
    // std::cout << "\n";

    return Phi;
}

/// @brief Calculates the state transition matrix 𝚽 using the exponential matrix
/// @param[in] F System Matrix
/// @param[in] tau_s time interval in [s]
/// @note See \cite Groves2013 Groves, ch. 3.2.3, eq. 3.33, p. 97
/// @attention The cost of the computation is approximately 20n^3 for matrices of size n. The number 20 depends weakly on the norm of the matrix.
template<typename Derived>
typename Derived::PlainObject transitionMatrix_Phi_exp(const Eigen::MatrixBase<Derived>& F, typename Derived::Scalar tau_s)
{
    // Transition matrix 𝚽
    return (F * tau_s).exp();
}

} // namespace NAV
