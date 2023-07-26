// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file KeyedKalmanFilter.hpp
/// @brief Kalman Filter with keyed states
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-07-11

#pragma once

#include <unordered_map>

#include "util/Eigen.hpp"
#include "util/Container/KeyedMatrix.hpp"
#include "Navigation/Math/Math.hpp"

namespace NAV
{
/// @brief Keyed Kalman Filter class
/// @tparam Scalar Numeric type, e.g. float, double, int or std::complex<float>.
/// @tparam StateKeyType Type of the key used for state lookup
/// @tparam MeasKeyType Type of the key used for measurement lookup
template<typename Scalar, typename StateKeyType, typename MeasKeyType>
class KeyedKalmanFilter
{
  public:
    /// @brief Default Constructor
    KeyedKalmanFilter() = default;

    /// @brief Constructor
    /// @param stateKeys State keys
    /// @param measKeys Measurement keys
    KeyedKalmanFilter(const std::vector<StateKeyType>& stateKeys, const std::vector<MeasKeyType>& measKeys)
    {
        std::unordered_set<StateKeyType> stateSet = { stateKeys.begin(), stateKeys.end() };
        INS_ASSERT_USER_ERROR(stateSet.size() == stateKeys.size(), "Each state key must be unique");
        std::unordered_set<MeasKeyType> measSet = { measKeys.begin(), measKeys.end() };
        INS_ASSERT_USER_ERROR(measSet.size() == measKeys.size(), "Each measurement key must be unique");

        auto n = static_cast<int>(stateKeys.size());
        auto m = static_cast<int>(measKeys.size());

        x = KeyedVectorX<Scalar, StateKeyType>(Eigen::VectorX<Scalar>::Zero(n), stateKeys);
        P = KeyedMatrixX<Scalar, StateKeyType, StateKeyType>(Eigen::MatrixX<Scalar>::Zero(n, n), stateKeys);
        Phi = KeyedMatrixX<Scalar, StateKeyType, StateKeyType>(Eigen::MatrixX<Scalar>::Zero(n, n), stateKeys);
        Q = KeyedMatrixX<Scalar, StateKeyType, StateKeyType>(Eigen::MatrixX<Scalar>::Zero(n, n), stateKeys);
        z = KeyedVectorX<Scalar, MeasKeyType>(Eigen::VectorX<Scalar>::Zero(m), measKeys);
        H = KeyedMatrixX<Scalar, MeasKeyType, StateKeyType>(Eigen::MatrixX<Scalar>::Zero(m, n), measKeys, stateKeys);
        R = KeyedMatrixX<Scalar, MeasKeyType, MeasKeyType>(Eigen::MatrixX<Scalar>::Zero(m, m), measKeys, measKeys);
        S = KeyedMatrixX<Scalar, MeasKeyType, MeasKeyType>(Eigen::MatrixX<Scalar>::Zero(m, m), measKeys, measKeys);
        K = KeyedMatrixX<Scalar, StateKeyType, MeasKeyType>(Eigen::MatrixX<Scalar>::Zero(n, m), stateKeys, measKeys);
        I = Eigen::MatrixX<Scalar>::Identity(n, n);
    }

    /// @brief Sets all Vectors and matrices to 0
    void setZero()
    {
        x(all).setZero();        // x̂ State vector
        P(all, all).setZero();   // 𝐏 Error covariance matrix
        Phi(all, all).setZero(); // 𝚽 State transition matrix
        Q(all, all).setZero();   // 𝐐 System/Process noise covariance matrix
        z(all).setZero();        // 𝐳 Measurement vector
        H(all, all).setZero();   // 𝐇 Measurement sensitivity Matrix
        R(all, all).setZero();   // 𝐑 = 𝐸{𝐰ₘ𝐰ₘᵀ} Measurement noise covariance matrix
        S(all, all).setZero();   // 𝗦 Measurement prediction covariance matrix
        K(all, all).setZero();   // 𝐊 Kalman gain matrix
    }

    /// @brief Do a Time Update
    /// @attention Update the State transition matrix (𝚽) and the Process noise covariance matrix (𝐐) before calling this
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 3.2.2)
    void predict()
    {
        // Math: \mathbf{\hat{x}}_k^- = \mathbf{\Phi}_{k-1}\mathbf{\hat{x}}_{k-1}^+ \qquad \text{P. Groves}\,(3.14)
        x(all) = Phi(all, all) * x(all);

        // Math: \mathbf{P}_k^- = \mathbf{\Phi}_{k-1} P_{k-1}^+ \mathbf{\Phi}_{k-1}^T + \mathbf{Q}_{k-1} \qquad \text{P. Groves}\,(3.15)
        P(all, all) = Phi(all, all) * P(all, all) * Phi(all, all).transpose() + Q(all, all);
    }

    /// @brief Do a Measurement Update with a Measurement 𝐳
    /// @attention Update the Measurement sensitivity Matrix (𝐇), the Measurement noise covariance matrix (𝐑)
    ///            and the Measurement vector (𝐳) before calling this
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 3.2.2)
    void correct()
    {
        S(all, all) = H(all, all) * P(all, all) * H(all, all).transpose() + R(all, all);

        // Math: \mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T (\mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + R_k)^{-1} \qquad \text{P. Groves}\,(3.21)
        K(all, all) = P(all, all) * H(all, all).transpose() * S(all, all).inverse();

        // Math: \begin{align*} \mathbf{\hat{x}}_k^+ &= \mathbf{\hat{x}}_k^- + \mathbf{K}_k (\mathbf{z}_k - \mathbf{H}_k \mathbf{\hat{x}}_k^-) \\ &= \mathbf{\hat{x}}_k^- + \mathbf{K}_k \mathbf{\delta z}_k^{-} \end{align*} \qquad \text{P. Groves}\,(3.24)
        x(all) = x(all) + K(all, all) * (z(all) - H(all, all) * x(all));

        // Math: \mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- \qquad \text{P. Groves}\,(3.25)
        P(all, all) = (I - K(all, all) * H(all, all)) * P(all, all);
    }

    /// @brief Do a Measurement Update with a Measurement Innovation 𝜹𝐳
    /// @attention Update the Measurement sensitivity Matrix (𝐇), the Measurement noise covariance matrix (𝐑)
    ///            and the Measurement vector (𝐳) before calling this
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 3.2.2)
    /// @note See Brown & Hwang (2012) - Introduction to Random Signals and Applied Kalman Filtering (ch. 5.5 - figure 5.5)
    void correctWithMeasurementInnovation()
    {
        // Math: \mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T (\mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + R_k)^{-1} \qquad \text{P. Groves}\,(3.21)
        K(all, all) = P(all, all) * H(all, all).transpose() * (H(all, all) * P(all, all) * H(all, all).transpose() + R(all, all)).inverse();

        // Math: \begin{align*} \mathbf{\hat{x}}_k^+ &= \mathbf{\hat{x}}_k^- + \mathbf{K}_k (\mathbf{z}_k - \mathbf{H}_k \mathbf{\hat{x}}_k^-) \\ &= \mathbf{\hat{x}}_k^- + \mathbf{K}_k \mathbf{\delta z}_k^{-} \end{align*} \qquad \text{P. Groves}\,(3.24)
        x(all) = x(all) + K(all, all) * z(all);

        // Math: \mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- \qquad \text{P. Groves}\,(3.25)
        // _P(all, all) = (I - K(all, all) * H(all, all)) * _P(all, all);

        // Math: \mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T \qquad \text{Brown & Hwang}\,(p. 145, eq. 4.2.11)
        P(all, all) = (I - K(all, all) * H(all, all)) * P(all, all) * (I - K(all, all) * H(all, all)).transpose() + K(all, all) * R(all, all) * K(all, all).transpose();
    }

    /// @brief Calculates the state transition matrix 𝚽 limited to first order in 𝐅𝜏ₛ
    /// @param[in] F System Matrix
    /// @param[in] tau_s time interval in [s]
    /// @note See Groves (2013) chapter 14.2.4, equation (14.72)
    static KeyedMatrixXd<StateKeyType> calcTransitionMatrix(const KeyedMatrixXd<StateKeyType>& F, double tau_s)
    {
        // Transition matrix 𝚽
        return { Eigen::MatrixXd::Identity(F(all, all).rows(), F(all, all).cols()) + F(all, all) * tau_s, F.rowKeys() };
    }

    /// @brief Add a new state to the filter
    /// @param stateKey State key
    void addState(const StateKeyType& stateKey) { addStates({ stateKey }); }

    /// @brief Add new states to the filter
    /// @param stateKeys State keys
    void addStates(const std::vector<StateKeyType>& stateKeys)
    {
        INS_ASSERT_USER_ERROR(!x.hasAnyRows(stateKeys), "You cannot add a state key which is already in the Kalman filter.");
        std::unordered_set<StateKeyType> stateSet = { stateKeys.begin(), stateKeys.end() };
        INS_ASSERT_USER_ERROR(stateSet.size() == stateKeys.size(), "Each state key must be unique");

        auto n = x(all).rows() + static_cast<int>(stateKeys.size());

        x.addRows(stateKeys);
        P.addRowsCols(stateKeys, stateKeys);
        Phi.addRowsCols(stateKeys, stateKeys);
        Q.addRowsCols(stateKeys, stateKeys);
        H.addCols(stateKeys);
        K.addRows(stateKeys);
        I = Eigen::MatrixX<Scalar>::Identity(n, n);
    }

    /// @brief Remove a state from the filter
    /// @param stateKey State key
    void removeState(const StateKeyType& stateKey) { removeStates({ stateKey }); }

    /// @brief Remove states from the filter
    /// @param stateKeys State keys
    void removeStates(const std::vector<StateKeyType>& stateKeys)
    {
        INS_ASSERT_USER_ERROR(x.hasRows(stateKeys), "Not all state keys you are trying to remove are in the Kalman filter.");
        std::unordered_set<StateKeyType> stateSet = { stateKeys.begin(), stateKeys.end() };
        INS_ASSERT_USER_ERROR(stateSet.size() == stateKeys.size(), "Each state key must be unique");

        auto n = x(all).rows() - static_cast<int>(stateKeys.size());

        x.removeRows(stateKeys);
        P.removeRowsCols(stateKeys, stateKeys);
        Phi.removeRowsCols(stateKeys, stateKeys);
        Q.removeRowsCols(stateKeys, stateKeys);
        H.removeCols(stateKeys);
        K.removeRows(stateKeys);
        I = Eigen::MatrixX<Scalar>::Identity(n, n);
    }

    /// @brief Sets the measurement keys and initializes matrices z, H, R, S, K with Zero
    /// @param measKeys Measurement keys
    void setMeasurements(const std::vector<MeasKeyType>& measKeys)
    {
        std::unordered_set<MeasKeyType> measSet = { measKeys.begin(), measKeys.end() };
        INS_ASSERT_USER_ERROR(measSet.size() == measKeys.size(), "Each measurement key must be unique");

        auto n = static_cast<int>(x(all).rows());
        auto m = static_cast<int>(measKeys.size());

        const auto& stateKeys = x.rowKeys();

        z = KeyedVectorX<Scalar, MeasKeyType>(Eigen::VectorX<Scalar>::Zero(m), measKeys);
        H = KeyedMatrixX<Scalar, MeasKeyType, StateKeyType>(Eigen::MatrixX<Scalar>::Zero(m, n), measKeys, stateKeys);
        R = KeyedMatrixX<Scalar, MeasKeyType, MeasKeyType>(Eigen::MatrixX<Scalar>::Zero(m, m), measKeys, measKeys);
        S = KeyedMatrixX<Scalar, MeasKeyType, MeasKeyType>(Eigen::MatrixX<Scalar>::Zero(m, m), measKeys, measKeys);
        K = KeyedMatrixX<Scalar, StateKeyType, MeasKeyType>(Eigen::MatrixX<Scalar>::Zero(n, m), stateKeys, measKeys);
    }

    KeyedVectorX<Scalar, StateKeyType> x;                 ///< x̂ State vector (n x 1)
    KeyedMatrixX<Scalar, StateKeyType, StateKeyType> P;   ///< 𝐏 Error covariance matrix (n x n)
    KeyedMatrixX<Scalar, StateKeyType, StateKeyType> Phi; ///< 𝚽 State transition matrix (n x n)
    KeyedMatrixX<Scalar, StateKeyType, StateKeyType> Q;   ///< 𝐐 System/Process noise covariance matrix (n x n)
    KeyedVectorX<Scalar, MeasKeyType> z;                  ///< 𝐳 Measurement vector (m x 1)
    KeyedMatrixX<Scalar, MeasKeyType, StateKeyType> H;    ///< 𝐇 Measurement sensitivity matrix (m x n)
    KeyedMatrixX<Scalar, MeasKeyType, MeasKeyType> R;     ///< 𝐑 = 𝐸{𝐰ₘ𝐰ₘᵀ} Measurement noise covariance matrix (m x m)
    KeyedMatrixX<Scalar, MeasKeyType, MeasKeyType> S;     ///< 𝗦 Measurement prediction covariance matrix (m x m)
    KeyedMatrixX<Scalar, StateKeyType, MeasKeyType> K;    ///< 𝐊 Kalman gain matrix (n x m)

  private:
    Eigen::MatrixXd I; ///< 𝑰 Identity matrix (n x n)
};

/// @brief Keyed Kalman Filter class with double as type
/// @tparam StateKeyType Type of the key used for state lookup
/// @tparam MeasKeyType Type of the key used for measurement lookup
template<typename StateKeyType, typename MeasKeyType>
using KeyedKalmanFilterD = KeyedKalmanFilter<double, StateKeyType, MeasKeyType>;

/// @brief Calculates the state transition matrix 𝚽 limited to specified order in 𝐅𝜏ₛ
/// @param[in] F System Matrix
/// @param[in] tau_s time interval in [s]
/// @param[in] order The order of the Taylor polynom to calculate
/// @note See \cite Groves2013 Groves, ch. 3.2.3, eq. 3.34, p. 98
template<typename Scalar, typename RowKeyType, typename ColKeyType, int Rows, int Cols>
KeyedMatrix<Scalar, RowKeyType, ColKeyType, Rows, Cols> transitionMatrix_Phi_Taylor(const KeyedMatrix<Scalar, RowKeyType, ColKeyType, Rows, Cols>& F, Scalar tau_s, size_t order)
{
    // Transition matrix 𝚽
    Eigen::Matrix<Scalar, Rows, Cols> Phi;

    if constexpr (Rows == Eigen::Dynamic)
    {
        Phi = Eigen::Matrix<Scalar, Rows, Cols>::Identity(F.rows(), F.cols());
    }
    else
    {
        Phi = Eigen::Matrix<Scalar, Rows, Cols>::Identity();
    }
    // std::cout << "Phi = I";

    for (size_t i = 1; i <= order; i++)
    {
        Eigen::Matrix<Scalar, Rows, Cols> Fpower = F(all, all);
        // std::cout << " + (F";

        for (size_t j = 1; j < i; j++) // F^j
        {
            // std::cout << "*F";
            Fpower *= F(all, all);
        }
        // std::cout << "*tau_s^" << i << ")";
        // std::cout << "/" << math::factorial(i);
        Phi += (Fpower * std::pow(tau_s, i)) / math::factorial(i);
    }
    // std::cout << "\n";

    return { Phi, F.rowKeys(), F.colKeys() };
}

/// @brief Calculates the state transition matrix 𝚽 using the exponential matrix
/// @param[in] F System Matrix
/// @param[in] tau_s time interval in [s]
/// @note See \cite Groves2013 Groves, ch. 3.2.3, eq. 3.33, p. 97
/// @attention The cost of the computation is approximately 20n^3 for matrices of size n. The number 20 depends weakly on the norm of the matrix.
template<typename Scalar, typename RowKeyType, typename ColKeyType, int Rows, int Cols>
KeyedMatrix<Scalar, RowKeyType, ColKeyType, Rows, Cols> transitionMatrix_Phi_exp(const KeyedMatrix<Scalar, RowKeyType, ColKeyType, Rows, Cols>& F, Scalar tau_s)
{
    // Transition matrix 𝚽
    return { (F(all, all) * tau_s).exp(), F.rowKeys(), F.colKeys() };
}

} // namespace NAV
