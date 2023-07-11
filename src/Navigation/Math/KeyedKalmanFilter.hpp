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

        _x = KeyedVectorX<Scalar, StateKeyType>(Eigen::VectorX<Scalar>::Zero(n), stateKeys);
        _P = KeyedMatrixX<Scalar, StateKeyType, StateKeyType>(Eigen::MatrixX<Scalar>::Zero(n, n), stateKeys);
        _Phi = KeyedMatrixX<Scalar, StateKeyType, StateKeyType>(Eigen::MatrixX<Scalar>::Zero(n, n), stateKeys);
        _Q = KeyedMatrixX<Scalar, StateKeyType, StateKeyType>(Eigen::MatrixX<Scalar>::Zero(n, n), stateKeys);
        _z = KeyedVectorX<Scalar, MeasKeyType>(Eigen::VectorX<Scalar>::Zero(m), measKeys);
        _H = KeyedMatrixX<Scalar, MeasKeyType, StateKeyType>(Eigen::MatrixX<Scalar>::Zero(m, n), measKeys, stateKeys);
        _R = KeyedMatrixX<Scalar, MeasKeyType, MeasKeyType>(Eigen::MatrixX<Scalar>::Zero(m, m), measKeys, measKeys);
        _S = KeyedMatrixX<Scalar, MeasKeyType, MeasKeyType>(Eigen::MatrixX<Scalar>::Zero(m, m), measKeys, measKeys);
        _K = KeyedMatrixX<Scalar, StateKeyType, MeasKeyType>(Eigen::MatrixX<Scalar>::Zero(n, m), stateKeys, measKeys);
    }

    /// @brief Do a Time Update
    /// @attention Update the State transition matrix (𝚽) and the Process noise covariance matrix (𝐐) before calling this
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 3.2.2)
    void predict()
    {
        // Math: \mathbf{\hat{x}}_k^- = \mathbf{\Phi}_{k-1}\mathbf{\hat{x}}_{k-1}^+ \qquad \text{P. Groves}\,(3.14)
        _x(all) = _Phi(all, all) * _x(all);

        // Math: \mathbf{P}_k^- = \mathbf{\Phi}_{k-1} P_{k-1}^+ \mathbf{\Phi}_{k-1}^T + \mathbf{Q}_{k-1} \qquad \text{P. Groves}\,(3.15)
        _P(all, all) = _Phi(all, all) * _P(all, all) * _Phi(all, all).transpose() + _Q(all, all);
    }

    /// @brief Do a Measurement Update with a Measurement 𝐳
    /// @attention Update the Measurement sensitivity Matrix (𝐇), the Measurement noise covariance matrix (𝐑)
    ///            and the Measurement vector (𝐳) before calling this
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 3.2.2)
    void correct()
    {
        _S(all, all) = _H(all, all) * _P(all, all) * _H(all, all).transpose() + _R(all, all);

        // Math: \mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T (\mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + R_k)^{-1} \qquad \text{P. Groves}\,(3.21)
        _K(all, all) = _P(all, all) * _H(all, all).transpose() * _S(all, all).inverse();

        // Math: \begin{align*} \mathbf{\hat{x}}_k^+ &= \mathbf{\hat{x}}_k^- + \mathbf{K}_k (\mathbf{z}_k - \mathbf{H}_k \mathbf{\hat{x}}_k^-) \\ &= \mathbf{\hat{x}}_k^- + \mathbf{K}_k \mathbf{\delta z}_k^{-} \end{align*} \qquad \text{P. Groves}\,(3.24)
        _x(all) = _x(all) + _K(all, all) * (_z(all) - _H(all, all) * _x(all));

        // Math: \mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- \qquad \text{P. Groves}\,(3.25)
        _P(all, all) = (_I - _K(all, all) * _H(all, all)) * _P(all, all);
    }

    /// @brief Do a Measurement Update with a Measurement Innovation 𝜹𝐳
    /// @attention Update the Measurement sensitivity Matrix (𝐇), the Measurement noise covariance matrix (𝐑)
    ///            and the Measurement vector (𝐳) before calling this
    /// @note See P. Groves (2013) - Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems (ch. 3.2.2)
    /// @note See Brown & Hwang (2012) - Introduction to Random Signals and Applied Kalman Filtering (ch. 5.5 - figure 5.5)
    void correctWithMeasurementInnovation()
    {
        // Math: \mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T (\mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + R_k)^{-1} \qquad \text{P. Groves}\,(3.21)
        _K(all, all) = _P(all, all) * _H(all, all).transpose() * (_H(all, all) * _P(all, all) * _H(all, all).transpose() + _R(all, all)).inverse();

        // Math: \begin{align*} \mathbf{\hat{x}}_k^+ &= \mathbf{\hat{x}}_k^- + \mathbf{K}_k (\mathbf{z}_k - \mathbf{H}_k \mathbf{\hat{x}}_k^-) \\ &= \mathbf{\hat{x}}_k^- + \mathbf{K}_k \mathbf{\delta z}_k^{-} \end{align*} \qquad \text{P. Groves}\,(3.24)
        _x(all) = _x(all) + _K(all, all) * _z(all);

        // Math: \mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- \qquad \text{P. Groves}\,(3.25)
        // _P(all, all) = (_I - _K(all, all) * _H(all, all)) * _P(all, all);

        // Math: \mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T \qquad \text{Brown & Hwang}\,(p. 145, eq. 4.2.11)
        _P(all, all) = (_I - _K(all, all) * _H(all, all)) * _P(all, all) * (_I - _K(all, all) * _H(all, all)).transpose() + _K(all, all) * _R(all, all) * _K(all, all).transpose();
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

    /// @brief Add a new state to the filter
    /// @param stateKey State key
    void addState(const StateKeyType& stateKey) { addStates({ stateKey }); }

    /// @brief Add new states to the filter
    /// @param stateKeys State keys
    void addStates(const std::vector<StateKeyType>& stateKeys)
    {
        INS_ASSERT_USER_ERROR(!_x.hasAnyRows(stateKeys), "You cannot add a state key which is already in the Kalman filter.");
        std::unordered_set<StateKeyType> stateSet = { stateKeys.begin(), stateKeys.end() };
        INS_ASSERT_USER_ERROR(stateSet.size() == stateKeys.size(), "Each state key must be unique");

        auto n = _x(all).rows() + static_cast<int>(stateKeys.size());

        _x.addRows(stateKeys);
        _P.addRowsCols(stateKeys, stateKeys);
        _Phi.addRowsCols(stateKeys, stateKeys);
        _Q.addRowsCols(stateKeys, stateKeys);
        _H.addCols(stateKeys);
        _K.addRows(stateKeys);
        _I = Eigen::MatrixX<Scalar>::Identity(n, n);
    }

    /// @brief Remove a state from the filter
    /// @param stateKey State key
    void removeState(const StateKeyType& stateKey) { removeStates({ stateKey }); }

    /// @brief Remove states from the filter
    /// @param stateKeys State keys
    void removeStates(const std::vector<StateKeyType>& stateKeys)
    {
        INS_ASSERT_USER_ERROR(_x.hasRows(stateKeys), "Not all state keys you are trying to remove are in the Kalman filter.");
        std::unordered_set<StateKeyType> stateSet = { stateKeys.begin(), stateKeys.end() };
        INS_ASSERT_USER_ERROR(stateSet.size() == stateKeys.size(), "Each state key must be unique");

        auto n = _x(all).rows() - static_cast<int>(stateKeys.size());

        _x.removeRows(stateKeys);
        _P.removeRowsCols(stateKeys, stateKeys);
        _Phi.removeRowsCols(stateKeys, stateKeys);
        _Q.removeRowsCols(stateKeys, stateKeys);
        _H.removeCols(stateKeys);
        _K.removeRows(stateKeys);
        _I = Eigen::MatrixX<Scalar>::Identity(n, n);
    }

    /// @brief Sets the measurement keys and initializes matrices z, H, R, S, K with Zero
    /// @param measKeys Measurement keys
    void setMeasurements(const std::vector<MeasKeyType>& measKeys)
    {
        std::unordered_set<MeasKeyType> measSet = { measKeys.begin(), measKeys.end() };
        INS_ASSERT_USER_ERROR(measSet.size() == measKeys.size(), "Each measurement key must be unique");

        auto n = static_cast<int>(_x(all).rows());
        auto m = static_cast<int>(measKeys.size());

        const auto& stateKeys = _x.rowKeys();

        _z = KeyedVectorX<Scalar, MeasKeyType>(Eigen::VectorX<Scalar>::Zero(m), measKeys);
        _H = KeyedMatrixX<Scalar, MeasKeyType, StateKeyType>(Eigen::MatrixX<Scalar>::Zero(m, n), measKeys, stateKeys);
        _R = KeyedMatrixX<Scalar, MeasKeyType, MeasKeyType>(Eigen::MatrixX<Scalar>::Zero(m, m), measKeys, measKeys);
        _S = KeyedMatrixX<Scalar, MeasKeyType, MeasKeyType>(Eigen::MatrixX<Scalar>::Zero(m, m), measKeys, measKeys);
        _K = KeyedMatrixX<Scalar, StateKeyType, MeasKeyType>(Eigen::MatrixX<Scalar>::Zero(n, m), stateKeys, measKeys);
    }

    // #######################################################################################################
    //                                             Member access
    // #######################################################################################################

    /// @brief Access into x̂ State vector
    /// @tparam T Key Type
    /// @param[in] key Key
    template<typename T>
    decltype(auto) x(const T& key)
    {
        return _x(key);
    }
    /// @brief Access into
    /// @tparam T Key Type
    /// @param[in] key Key
    template<typename T>
    decltype(auto) x(const T& key) const
    {
        return _x(key);
    }

    // ###########################################################################################################

    /// @brief Access diagonal elements from 𝐏 Error covariance matrix
    /// @tparam T Key Type
    /// @param[in] key Key
    template<typename T>
    decltype(auto) P(const T& key)
    {
        return _P(key, key);
    }
    /// @brief Access diagonal elements from 𝐏 Error covariance matrix
    /// @tparam T Key Type
    /// @param[in] key Key
    template<typename T>
    decltype(auto) P(const T& key) const
    {
        return _P(key, key);
    }
    /// @brief Access into 𝐏 Error covariance matrix
    /// @tparam T Key Type
    /// @param[in] rowKey Row Key
    /// @param[in] colKey Col Key
    template<typename T>
    decltype(auto) P(const T& rowKey, const T& colKey)
    {
        return _P(rowKey, colKey);
    }
    /// @brief Access into 𝐏 Error covariance matrix
    /// @tparam T Key Type
    /// @param[in] rowKey Row Key
    /// @param[in] colKey Col Key
    template<typename T>
    decltype(auto) P(const T& rowKey, const T& colKey) const
    {
        return _P(rowKey, colKey);
    }

    // ###########################################################################################################

    /// @brief Access diagonal elements from 𝚽 State transition matrix
    /// @tparam T Key Type
    /// @param[in] key Key
    template<typename T>
    decltype(auto) Phi(const T& key)
    {
        return _Phi(key, key);
    }
    /// @brief Access diagonal elements from 𝚽 State transition matrix
    /// @tparam T Key Type
    /// @param[in] key Key
    template<typename T>
    decltype(auto) Phi(const T& key) const
    {
        return _Phi(key, key);
    }
    /// @brief Access into 𝚽 State transition matrix
    /// @tparam T Key Type
    /// @param[in] rowKey Row Key
    /// @param[in] colKey Col Key
    template<typename T>
    decltype(auto) Phi(const T& rowKey, const T& colKey)
    {
        return _Phi(rowKey, colKey);
    }
    /// @brief Access into 𝚽 State transition matrix
    /// @tparam T Key Type
    /// @param[in] rowKey Row Key
    /// @param[in] colKey Col Key
    template<typename T>
    decltype(auto) Phi(const T& rowKey, const T& colKey) const
    {
        return _Phi(rowKey, colKey);
    }

    // ###########################################################################################################

    /// @brief Access diagonal elements from 𝐐 System/Process noise covariance matrix
    /// @tparam T Key Type
    /// @param[in] key Key
    template<typename T>
    decltype(auto) Q(const T& key)
    {
        return _Q(key, key);
    }
    /// @brief Access diagonal elements from 𝐐 System/Process noise covariance matrix
    /// @tparam T Key Type
    /// @param[in] key Key
    template<typename T>
    decltype(auto) Q(const T& key) const
    {
        return _Q(key, key);
    }
    /// @brief Access into 𝐐 System/Process noise covariance matrix
    /// @tparam T Key Type
    /// @param[in] rowKey Row Key
    /// @param[in] colKey Col Key
    template<typename T>
    decltype(auto) Q(const T& rowKey, const T& colKey)
    {
        return _Q(rowKey, colKey);
    }
    /// @brief Access into 𝐐 System/Process noise covariance matrix
    /// @tparam T Key Type
    /// @param[in] rowKey Row Key
    /// @param[in] colKey Col Key
    template<typename T>
    decltype(auto) Q(const T& rowKey, const T& colKey) const
    {
        return _Q(rowKey, colKey);
    }

    // ###########################################################################################################

    /// @brief Access into 𝐳 Measurement vector
    /// @tparam T Key Type
    /// @param[in] key Key
    template<typename T>
    decltype(auto) z(const T& key)
    {
        return _z(key);
    }
    /// @brief Access into 𝐳 Measurement vector
    /// @tparam T Key Type
    /// @param[in] key Key
    template<typename T>
    decltype(auto) z(const T& key) const
    {
        return _z(key);
    }

    // ###########################################################################################################

    /// @brief Access into 𝐇 Measurement sensitivity matrix
    /// @tparam S State Key Type
    /// @tparam M Measurement Key Type
    /// @param[in] measKey Measurement key
    /// @param[in] stateKey State key
    template<typename S, typename M>
    decltype(auto) H(const M& measKey, const S& stateKey)
    {
        return _H(measKey, stateKey);
    }
    /// @brief Access into 𝐇 Measurement sensitivity matrix
    /// @tparam S State Key Type
    /// @tparam M Measurement Key Type
    /// @param[in] measKey Measurement key
    /// @param[in] stateKey State key
    template<typename S, typename M>
    decltype(auto) H(const M& measKey, const S& stateKey) const
    {
        return _H(measKey, stateKey);
    }

    // ###########################################################################################################

    /// @brief Access diagonal elements from 𝐑 Measurement noise covariance matrix
    /// @tparam T Key Type
    /// @param[in] key Key
    template<typename T>
    decltype(auto) R(const T& key)
    {
        return _R(key, key);
    }
    /// @brief Access diagonal elements from 𝐑 Measurement noise covariance matrix
    /// @tparam T Key Type
    /// @param[in] key Key
    template<typename T>
    decltype(auto) R(const T& key) const
    {
        return _R(key, key);
    }
    /// @brief Access into 𝐑 Measurement noise covariance matrix
    /// @tparam T Key Type
    /// @param[in] rowKey Row Key
    /// @param[in] colKey Col Key
    template<typename T>
    decltype(auto) R(const T& rowKey, const T& colKey)
    {
        return _R(rowKey, colKey);
    }
    /// @brief Access into 𝐑 Measurement noise covariance matrix
    /// @tparam T Key Type
    /// @param[in] rowKey Row Key
    /// @param[in] colKey Col Key
    template<typename T>
    decltype(auto) R(const T& rowKey, const T& colKey) const
    {
        return _R(rowKey, colKey);
    }

    // ###########################################################################################################

    /// @brief Access diagonal elements from 𝗦 Measurement prediction covariance matrix
    /// @tparam T Key Type
    /// @param[in] key Key
    template<typename T>
    decltype(auto) S(const T& key)
    {
        return _S(key, key);
    }
    /// @brief Access diagonal elements from 𝗦 Measurement prediction covariance matrix
    /// @tparam T Key Type
    /// @param[in] key Key
    template<typename T>
    decltype(auto) S(const T& key) const
    {
        return _S(key, key);
    }
    /// @brief Access into 𝗦 Measurement prediction covariance matrix
    /// @tparam T Key Type
    /// @param[in] rowKey Row Key
    /// @param[in] colKey Col Key
    template<typename T>
    decltype(auto) S(const T& rowKey, const T& colKey)
    {
        return _S(rowKey, colKey);
    }
    /// @brief Access into 𝗦 Measurement prediction covariance matrix
    /// @tparam T Key Type
    /// @param[in] rowKey Row Key
    /// @param[in] colKey Col Key
    template<typename T>
    decltype(auto) S(const T& rowKey, const T& colKey) const
    {
        return _S(rowKey, colKey);
    }

    // ###########################################################################################################

    /// @brief Access into 𝐊 Kalman gain matrix
    /// @tparam S State Key Type
    /// @tparam M Measurement Key Type
    /// @param[in] measKey Measurement key
    /// @param[in] stateKey State key
    template<typename S, typename M>
    decltype(auto) K(const S& stateKey, const M& measKey)
    {
        return _K(stateKey, measKey);
    }
    /// @brief Access into 𝐊 Kalman gain matrix
    /// @tparam S State Key Type
    /// @tparam M Measurement Key Type
    /// @param[in] measKey Measurement key
    /// @param[in] stateKey State key
    template<typename S, typename M>
    decltype(auto) K(const S& stateKey, const M& measKey) const
    {
        return _K(stateKey, measKey);
    }

  private:
    KeyedVectorX<Scalar, StateKeyType> _x;                 ///< x̂ State vector (n x 1)
    KeyedMatrixX<Scalar, StateKeyType, StateKeyType> _P;   ///< 𝐏 Error covariance matrix (n x n)
    KeyedMatrixX<Scalar, StateKeyType, StateKeyType> _Phi; ///< 𝚽 State transition matrix (n x n)
    KeyedMatrixX<Scalar, StateKeyType, StateKeyType> _Q;   ///< 𝐐 System/Process noise covariance matrix (n x n)
    KeyedVectorX<Scalar, MeasKeyType> _z;                  ///< 𝐳 Measurement vector (m x 1)
    KeyedMatrixX<Scalar, MeasKeyType, StateKeyType> _H;    ///< 𝐇 Measurement sensitivity matrix (m x n)
    KeyedMatrixX<Scalar, MeasKeyType, MeasKeyType> _R;     ///< 𝐑 = 𝐸{𝐰ₘ𝐰ₘᵀ} Measurement noise covariance matrix (m x m)
    KeyedMatrixX<Scalar, MeasKeyType, MeasKeyType> _S;     ///< 𝗦 Measurement prediction covariance matrix (m x m)
    KeyedMatrixX<Scalar, StateKeyType, MeasKeyType> _K;    ///< 𝐊 Kalman gain matrix (n x m)
    Eigen::MatrixXd _I;                                    ///< 𝑰 Identity matrix (n x n)
};

} // namespace NAV
