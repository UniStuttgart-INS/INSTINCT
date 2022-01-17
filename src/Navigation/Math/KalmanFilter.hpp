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
template<typename _Scalar, int _n, int _m>
class KalmanFilter
{
  public:
    /// @brief Sets all Vectors and matrices to 0
    void setZero()
    {
        /// x̂ State vector
        x = Eigen::Vector<_Scalar, _n>::Zero();

        /// 𝐏 Error covariance matrix
        P = Eigen::Matrix<_Scalar, _n, _n>::Zero();

        /// 𝚽 State transition matrix
        Phi = Eigen::Matrix<_Scalar, _n, _n>::Zero();

        /// 𝐐 System/Process noise covariance matrix
        Q = Eigen::Matrix<_Scalar, _n, _n>::Zero();

        /// 𝐳 Measurement vector
        z = Eigen::Vector<_Scalar, _m>::Zero();

        /// 𝐇 Measurement sensitivity Matrix
        H = Eigen::Matrix<_Scalar, _m, _n>::Zero();

        /// 𝐑 = 𝐸{𝐰ₘ𝐰ₘᵀ} Measurement noise covariance matrix
        R = Eigen::Matrix<_Scalar, _m, _m>::Zero();

        /// 𝐊 Kalman gain matrix
        K = Eigen::Matrix<_Scalar, _n, _m>::Zero();
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

        // Math: \mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T \qquad \text{Brown & Hwang}\,(p. 145, eq. 4.2.11)
        P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();
    }

    /// x̂ State vector
    Eigen::Vector<_Scalar, _n> x = Eigen::Vector<_Scalar, _n>::Zero();

    /// 𝐏 Error covariance matrix
    Eigen::Matrix<_Scalar, _n, _n> P = Eigen::Matrix<_Scalar, _n, _n>::Zero();

    /// 𝚽 State transition matrix
    Eigen::Matrix<_Scalar, _n, _n> Phi = Eigen::Matrix<_Scalar, _n, _n>::Zero();

    /// 𝐐 System/Process noise covariance matrix
    Eigen::Matrix<_Scalar, _n, _n> Q = Eigen::Matrix<_Scalar, _n, _n>::Zero();

    /// 𝐳 Measurement vector
    Eigen::Vector<_Scalar, _m> z = Eigen::Vector<_Scalar, _m>::Zero();

    /// 𝐇 Measurement sensitivity Matrix
    Eigen::Matrix<_Scalar, _m, _n> H = Eigen::Matrix<_Scalar, _m, _n>::Zero();

    /// 𝐑 = 𝐸{𝐰ₘ𝐰ₘᵀ} Measurement noise covariance matrix
    Eigen::Matrix<_Scalar, _m, _m> R = Eigen::Matrix<_Scalar, _m, _m>::Zero();

    /// 𝐊 Kalman gain matrix
    Eigen::Matrix<_Scalar, _n, _m> K = Eigen::Matrix<_Scalar, _n, _m>::Zero();

  private:
    /// 𝑰 Identity Matrix (n x n)
    const Eigen::Matrix<_Scalar, _n, _n> I = Eigen::Matrix<_Scalar, _n, _n>::Identity();
};

/// @brief Updates the state transition matrix 𝚽 limited to first order in 𝐅𝜏ₛ
/// @param[in] F System Matrix
/// @param[in] tau_s time interval in [s]
/// @note See Groves (2013) chapter 14.2.4, equation (14.72)
template<typename _Scalar, int _n>
Eigen::Matrix<_Scalar, _n, _n> transitionMatrixApproxOrder1(const Eigen::Matrix<_Scalar, _n, _n>& F, double tau_s)
{
    // Transition matrix 𝚽
    return Eigen::Matrix<_Scalar, _n, _n>::Identity() + F * tau_s;
}

} // namespace NAV
