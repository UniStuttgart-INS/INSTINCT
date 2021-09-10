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
        x = Eigen::MatrixXd::Zero(n, 1);

        // 𝐏 Error covariance matrix
        P = Eigen::MatrixXd::Zero(n, n);

        /// 𝚽 State transition matrix
        Phi = Eigen::MatrixXd::Zero(n, n);

        /// 𝐐 System/Process noise covariance matrix
        Q = Eigen::MatrixXd::Zero(n, n);

        /// 𝐳 Measurement vector
        z = Eigen::MatrixXd::Zero(m, 1);

        /// 𝐇 Measurement sensitivity Matrix
        H = Eigen::MatrixXd::Zero(m, n);

        /// 𝐑 = 𝐸{𝐰ₘ𝐰ₘᵀ} Measurement noise covariance matrix
        R = Eigen::MatrixXd::Zero(m, m);

        /// 𝐊 Kalman gain matrix
        K = Eigen::MatrixXd::Zero(n, m);

        /// 𝑰 Identity Matrix
        I = Eigen::MatrixXd::Identity(n, n);
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

    /// @brief Do a Measurement Update with a Measurement 𝐳
    /// @attention Update the Measurement sensitivity Matrix (𝐇), the Measurement noise covariance matrix (𝐑)
    ///            and the Measurement vector (𝐳) before calling this
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 3.2.2)
    void correct()
    {
        // Math: \mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T (\mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + R_k)^{-1} \qquad \text{P. Groves}\,(3.21)
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

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
        // Math: \mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T (\mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + R_k)^{-1} \qquad \text{P. Groves}\,(3.21)
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

        // Math: \begin{align*} \mathbf{\hat{x}}_k^+ &= \mathbf{\hat{x}}_k^- + \mathbf{K}_k (\mathbf{z}_k - \mathbf{H}_k \mathbf{\hat{x}}_k^-) \\ &= \mathbf{\hat{x}}_k^- + \mathbf{K}_k \mathbf{\delta z}_k^{-} \end{align*} \qquad \text{P. Groves}\,(3.24)
        x = x + K * z;

        // Math: \mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- \qquad \text{P. Groves}\,(3.25)
        // P = (I - K * H) * P;

        // TODO: Fix math comment
        // Math: \mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- \qquad \text{Brown & Hwang}\,(p. 185, fig. 5.5)
        P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();
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

    /// 𝐊 Kalman gain matrix
    Eigen::MatrixXd K;

  private:
    /// 𝑰 Identity Matrix (n x n)
    Eigen::MatrixXd I;
};

} // namespace NAV
