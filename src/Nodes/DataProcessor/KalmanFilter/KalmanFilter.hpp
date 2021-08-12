/// @file KalmanFilter.hpp
/// @brief Generalized Kalman Filter class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-11-25

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

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
        x = Eigen::VectorXd::Zero(n);

        // 𝐏 Error covariance matrix
        P = Eigen::MatrixXd::Zero(n, n);

        /// 𝚽 State transition matrix
        Phi = Eigen::MatrixXd::Zero(n, n);

        /// 𝐐 System/Process noise covariance matrix
        Q = Eigen::MatrixXd::Zero(n, n);

        /// 𝐳 Measurement vector
        z = Eigen::VectorXd::Zero(m);

        /// 𝐇 Measurement sensitivity Matrix
        H = Eigen::MatrixXd::Zero(m, n);

        /// 𝐑 = 𝐸{𝐰ₘ𝐰ₘᵀ} Measurement noise covariance matrix
        R = Eigen::MatrixXd::Zero(m, m);

        /// 𝐊 Kalman gain matrix
        K = Eigen::MatrixXd::Zero(n, m);
    }

    /// @brief Default constructor
    KalmanFilter() = delete;

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

    /// @brief Do a Measurement Update
    /// @attention Update the Measurement sensitivity Matrix (𝐇), the Measurement noise covariance matrix (𝐑)
    ///            and the Measurement vector (𝐳) before calling this
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 3.2.2)
    void correct()
    {
        // Math: \mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T (\mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + R_k)^{-1} \qquad \text{P. Groves}\,(3.21)
        K = P * H.transpose() * (R + H * P * H.transpose());

        // Math: \mathbf{\hat{x}}_k^+ = \mathbf{\hat{x}}_k^- + \mathbf{K}_k (\mathbf{z}_k - \mathbf{H}_k \mathbf{\hat{x}}_k^-) \qquad \text{P. Groves}\,(3.24)
        x = x + K * (z - H * x);

        // Math: \mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- \qquad \text{P. Groves}\,(3.25)
        P = P - K * H * P;
    }

    /// x̂ State vector
    Eigen::VectorXd x;

    /// 𝐏 Error covariance matrix
    Eigen::MatrixXd P;

    /// 𝚽 State transition matrix
    Eigen::MatrixXd Phi;

    /// 𝐐 System/Process noise covariance matrix
    Eigen::MatrixXd Q;

    /// 𝐳 Measurement vector
    Eigen::VectorXd z;

    /// 𝐇 Measurement sensitivity Matrix
    Eigen::MatrixXd H;

    /// 𝐑 = 𝐸{𝐰ₘ𝐰ₘᵀ} Measurement noise covariance matrix
    Eigen::MatrixXd R;

    /// 𝐊 Kalman gain matrix
    Eigen::MatrixXd K;
};

} // namespace NAV
