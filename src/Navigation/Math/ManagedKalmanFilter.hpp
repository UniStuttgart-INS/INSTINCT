// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ManagedKalmanFilter.hpp
/// @brief Kalman Filter with managed states
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-06-21

#pragma once

#include <unordered_map>

#include "util/Eigen.hpp"
#include "Navigation/Math/Math.hpp"

namespace NAV
{
/// @brief Generalized Kalman Filter class
template<typename StateKey, typename MeasKey>
class ManagedKalmanFilter
{
  public:
    /// Kalman filter entries
    struct Entry
    {
        size_t index;  ///< Index of the entry
        size_t length; ///< Length of the entry
    };

    /// @brief Do a Time Update
    /// @attention Update the State transition matrix (𝚽) and the Process noise covariance matrix (𝐐) before calling this
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 3.2.2)
    void predict()
    {
        // Math: \mathbf{\hat{x}}_k^- = \mathbf{\Phi}_{k-1}\mathbf{\hat{x}}_{k-1}^+ \qquad \text{P. Groves}\,(3.14)
        _x = Phi * _x;

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
        _x = _x + K * (z - H * _x);

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
        _x = _x + K * z;

        // Math: \mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- \qquad \text{P. Groves}\,(3.25)
        // P = (I - K * H) * P;

        // Math: \mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T \qquad \text{Brown & Hwang}\,(p. 145, eq. 4.2.11)
        P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();
    }

    /// @brief Calculates the state transition matrix 𝚽 limited to first order in 𝐅𝜏ₛ
    /// @param[in] F System Matrix
    /// @param[in] tau_s time interval in [s]
    /// @note See Groves (2013) chapter 14.2.4, equation (14.72)
    static Eigen::MatrixXd calcTransitionMatrix(const Eigen::MatrixXd& F, double tau_s)
    {
        // Transition matrix 𝚽
        return Eigen::MatrixXd::Identity(F.rows(), F.cols()) + F * tau_s;
    }

    Eigen::MatrixXd P;   ///< 𝐏 Error covariance matrix (n x n)
    Eigen::MatrixXd Phi; ///< 𝚽 State transition matrix (n x n)
    Eigen::MatrixXd Q;   ///< 𝐐 System/Process noise covariance matrix (n x n)
    Eigen::MatrixXd z;   ///< 𝐳 Measurement vector (m x 1)
    Eigen::MatrixXd H;   ///< 𝐇 Measurement sensitivity Matrix (m x n)
    Eigen::MatrixXd R;   ///< 𝐑 = 𝐸{𝐰ₘ𝐰ₘᵀ} Measurement noise covariance matrix (m x m)
    Eigen::MatrixXd S;   ///< 𝗦 Measurement prediction covariance matrix (m x m)
    Eigen::MatrixXd K;   ///< 𝐊 Kalman gain matrix (n x m)

    void addState(const StateKey& key, size_t length)
    {
        if (states.contains(key))
        {
            if (states.at(key).length == length) { return; }

            removeState(key);
        }

        states.insert(std::make_pair(key, Entry{ .index = static_cast<size_t>(_x.rows()), .length = length }));
        auto n = _x.rows() + static_cast<Eigen::Index>(length);
        auto m = z.rows();
        _x.conservativeResizeLike(Eigen::VectorXd::Zero(n));
        P.conservativeResizeLike(Eigen::MatrixXd::Zero(n, n));
        Phi.conservativeResizeLike(Eigen::MatrixXd::Zero(n, n));
        Q.conservativeResizeLike(Eigen::MatrixXd::Zero(n, n));
        // z.conservativeResizeLike(Eigen::VectorXd::Zero(m));
        H.conservativeResizeLike(Eigen::MatrixXd::Zero(m, n));
        // R.conservativeResizeLike(Eigen::MatrixXd::Zero(m, m));
        // S.conservativeResizeLike(Eigen::MatrixXd::Zero(m, m));
        K.conservativeResizeLike(Eigen::MatrixXd::Zero(n, m));
        I = Eigen::MatrixXd::Identity(n, n);
    }

    void removeState(const StateKey& key)
    {
        if (states.contains(key))
        {
            auto index = states.at(key).index;
            auto length = states.at(key).length;
            auto n = _x.rows() - static_cast<Eigen::Index>(length);

            removeRows(_x, index, length);
            removeRowsAndCols(P, index, length, index, length);
            removeRowsAndCols(Phi, index, length, index, length);
            removeRowsAndCols(Q, index, length, index, length);
            removeCols(H, index, length);
            removeRows(K, index, length);
            I = Eigen::MatrixXd::Identity(n, n);

            states.erase(key);
        }
    }

    void resetMeasurements()
    {
        // TODO
    }

    void addMeasurement(const MeasKey& key, size_t length)
    {
        if (measurements.contains(key))
        {
            if (measurements.at(key).length == length) { return; }

            removeMeasurement(key);
        }

        measurements.insert(std::make_pair(key, Entry{ .index = z.rows(), .length = length }));
        int n = _x.rows();
        int m = z.rows() + length;
        // _x.conservativeResizeLike(Eigen::VectorXd::Zero(n));
        // P.conservativeResizeLike(Eigen::MatrixXd::Zero(n, n));
        // Phi.conservativeResizeLike(Eigen::MatrixXd::Zero(n, n));
        // Q.conservativeResizeLike(Eigen::MatrixXd::Zero(n, n));
        z.conservativeResizeLike(Eigen::VectorXd::Zero(m));
        H.conservativeResizeLike(Eigen::MatrixXd::Zero(m, n));
        R.conservativeResizeLike(Eigen::MatrixXd::Zero(m, m));
        S.conservativeResizeLike(Eigen::MatrixXd::Zero(m, m));
        K.conservativeResizeLike(Eigen::MatrixXd::Zero(n, m));
        // I = Eigen::MatrixXd::Identity(n, n);
    }

    void removeMeasurement(const MeasKey& key)
    {
        if (measurements.contains(key))
        {
            auto index = measurements.at(key).index;
            auto length = measurements.at(key).length;

            removeRows(z, index, length);
            removeRows(H, index, length);
            removeRowsAndCols(R, index, length, index, length);
            removeRowsAndCols(S, index, length, index, length);
            removeCols(K, index, length);

            measurements.erase(key);
        }
    }

    Eigen::Ref<Eigen::VectorXd> x(const StateKey& key)
    {
        const auto& entry = states.at(key);
        return _x.segment(static_cast<Eigen::Index>(entry.index), static_cast<Eigen::Index>(entry.index + entry.length));
    }

    Eigen::VectorXd _x; ///< x̂ State vector (n x 1)
  private:
    Eigen::MatrixXd I; ///< 𝑰 Identity Matrix (n x n)

    std::unordered_map<StateKey, Entry> states;
    std::unordered_map<MeasKey, Entry> measurements;
};

} // namespace NAV
