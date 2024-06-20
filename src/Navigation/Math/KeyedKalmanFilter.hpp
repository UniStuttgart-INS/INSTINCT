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

#include <boost/math/distributions/chi_squared.hpp>
#include <imgui.h>

#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/KeyedMatrix.hpp"
#include "util/Eigen.hpp"
#include "util/Json.hpp"
#include "util/Container/KeyedMatrix.hpp"
#include "Navigation/Time/InsTime.hpp"
#include "Navigation/Math/Math.hpp"
#include "Navigation/Math/VanLoan.hpp"

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
        F = KeyedMatrixX<Scalar, StateKeyType, StateKeyType>(Eigen::MatrixX<Scalar>::Zero(n, n), stateKeys);
        Phi = KeyedMatrixX<Scalar, StateKeyType, StateKeyType>(Eigen::MatrixX<Scalar>::Zero(n, n), stateKeys);

        G = KeyedMatrixX<Scalar, StateKeyType, StateKeyType>(Eigen::MatrixX<Scalar>::Zero(n, n), stateKeys);
        W = KeyedMatrixX<Scalar, StateKeyType, StateKeyType>(Eigen::MatrixX<Scalar>::Zero(n, n), stateKeys);
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
        F(all, all).setZero();   // 𝐅 System model matrix (n x n)
        Phi(all, all).setZero(); // 𝚽 State transition matrix
        G(all, all).setZero();   // 𝐆 Noise input matrix (n x o)
        W(all, all).setZero();   // 𝐖 Noise scale matrix (o x o)
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
        S(all, all) = H(all, all) * P(all, all) * H(all, all).transpose() + R(all, all);

        // Math: \mathbf{K}_k = \mathbf{P}_k^- \mathbf{H}_k^T (\mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^T + R_k)^{-1} \qquad \text{P. Groves}\,(3.21)
        K(all, all) = P(all, all) * H(all, all).transpose() * S(all, all).inverse();

        // Math: \begin{align*} \mathbf{\hat{x}}_k^+ &= \mathbf{\hat{x}}_k^- + \mathbf{K}_k \left(\mathbf{z}_k - \mathbf{h}(\mathbf{\hat{x}}_k^-)\right) \\ &= \mathbf{\hat{x}}_k^- + \mathbf{K}_k \mathbf{\delta z}_k^{-} \end{align*} \qquad \text{P. Groves}\,(3.24)
        x(all) = x(all) + K(all, all) * z(all);

        // Math: \mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- \qquad \text{P. Groves}\,(3.25)
        // P(all, all) = (I - K(all, all) * H(all, all)) * P(all, all);

        // Math: \mathbf{P}_k^+ = (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k) \mathbf{P}_k^- (\mathbf{I} - \mathbf{K}_k \mathbf{H}_k)^T + \mathbf{K}_k \mathbf{R}_k \mathbf{K}_k^T \qquad \text{Brown & Hwang}\,(p. 145, eq. 4.2.11)
        P(all, all) = (I - K(all, all) * H(all, all)) * P(all, all) * (I - K(all, all) * H(all, all)).transpose() + K(all, all) * R(all, all) * K(all, all).transpose();
    }

    /// @brief Checks if the filter has the key
    /// @param stateKey State key
    [[nodiscard]] bool hasState(const StateKeyType& stateKey) const { return x.hasRow(stateKey); }
    /// @brief Checks if the filter has the keys
    /// @param stateKeys State keys
    [[nodiscard]] bool hasStates(const std::vector<StateKeyType>& stateKeys) const { return x.hasStates(stateKeys); }
    /// @brief Checks if the filter has any of the provided keys
    /// @param stateKeys State keys
    [[nodiscard]] bool hasAnyStates(const std::vector<StateKeyType>& stateKeys) const { return x.hasAnyStates(stateKeys); }

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
        F.addRowsCols(stateKeys, stateKeys);
        Phi.addRowsCols(stateKeys, stateKeys);
        G.addRowsCols(stateKeys, stateKeys);
        W.addRowsCols(stateKeys, stateKeys);
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

        auto n = x.rows() - static_cast<int>(stateKeys.size());

        x.removeRows(stateKeys);
        P.removeRowsCols(stateKeys, stateKeys);
        F.removeRowsCols(stateKeys, stateKeys);
        Phi.removeRowsCols(stateKeys, stateKeys);
        G.removeRowsCols(stateKeys, stateKeys);
        W.removeRowsCols(stateKeys, stateKeys);
        Q.removeRowsCols(stateKeys, stateKeys);
        H.removeCols(stateKeys);
        K.removeRows(stateKeys);
        I = Eigen::MatrixX<Scalar>::Identity(n, n);
    }

    /// @brief Replace the old with the new key
    /// @param[in] oldKey Old key to replace
    /// @param[in] newKey New key to use instead
    void replaceState(const StateKeyType& oldKey, const StateKeyType& newKey)
    {
        x.replaceRowKey(oldKey, newKey);
        P.replaceRowKey(oldKey, newKey);
        P.replaceColKey(oldKey, newKey);
        F.replaceRowKey(oldKey, newKey);
        F.replaceColKey(oldKey, newKey);
        Phi.replaceRowKey(oldKey, newKey);
        Phi.replaceColKey(oldKey, newKey);
        G.replaceRowKey(oldKey, newKey);
        G.replaceColKey(oldKey, newKey);
        W.replaceRowKey(oldKey, newKey);
        W.replaceColKey(oldKey, newKey);
        Q.replaceRowKey(oldKey, newKey);
        Q.replaceColKey(oldKey, newKey);
        H.replaceColKey(oldKey, newKey);
        K.replaceRowKey(oldKey, newKey);
    }

    /// @brief Sets the measurement keys and initializes matrices z, H, R, S, K with Zero
    /// @param measKeys Measurement keys
    void setMeasurements(const std::vector<MeasKeyType>& measKeys)
    {
        std::unordered_set<MeasKeyType> measSet = { measKeys.begin(), measKeys.end() };
        INS_ASSERT_USER_ERROR(measSet.size() == measKeys.size(), "Each measurement key must be unique");

        auto n = static_cast<int>(x.rows());
        auto m = static_cast<int>(measKeys.size());

        const auto& stateKeys = x.rowKeys();

        z = KeyedVectorX<Scalar, MeasKeyType>(Eigen::VectorX<Scalar>::Zero(m), measKeys);
        H = KeyedMatrixX<Scalar, MeasKeyType, StateKeyType>(Eigen::MatrixX<Scalar>::Zero(m, n), measKeys, stateKeys);
        R = KeyedMatrixX<Scalar, MeasKeyType, MeasKeyType>(Eigen::MatrixX<Scalar>::Zero(m, m), measKeys, measKeys);
        S = KeyedMatrixX<Scalar, MeasKeyType, MeasKeyType>(Eigen::MatrixX<Scalar>::Zero(m, m), measKeys, measKeys);
        K = KeyedMatrixX<Scalar, StateKeyType, MeasKeyType>(Eigen::MatrixX<Scalar>::Zero(n, m), stateKeys, measKeys);
    }

    /// @brief Remove a measurement from the filter
    /// @param measKey Measurement key
    void removeMeasurement(const MeasKeyType& measKey) { removeMeasurements({ measKey }); }

    /// @brief Remove measurements from the filter
    /// @param measKeys Measurement keys
    void removeMeasurements(const std::vector<MeasKeyType>& measKeys)
    {
        INS_ASSERT_USER_ERROR(z.hasRows(measKeys), "Not all measurement keys you are trying to remove are in the Kalman filter.");
        std::unordered_set<MeasKeyType> measurementSet = { measKeys.begin(), measKeys.end() };
        INS_ASSERT_USER_ERROR(measurementSet.size() == measKeys.size(), "Each measurement key must be unique");

        z.removeRows(measKeys);
        H.removeRows(measKeys);
        R.removeRowsCols(measKeys, measKeys);
        S.removeRowsCols(measKeys, measKeys);
        K.removeCols(measKeys);
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

    KeyedMatrixX<Scalar, StateKeyType, StateKeyType> F; ///< 𝐅 System model matrix (n x n)
    KeyedMatrixX<Scalar, StateKeyType, StateKeyType> G; ///< 𝐆 Noise input matrix (n x o)
    KeyedMatrixX<Scalar, StateKeyType, StateKeyType> W; ///< 𝐖 Noise scale matrix (o x o)

    /// @brief Calculates the state transition matrix 𝚽 limited to specified order in 𝐅𝜏ₛ
    /// @param[in] tau Time interval in [s]
    /// @param[in] order The order of the Taylor polynom to calculate
    /// @note See \cite Groves2013 Groves, ch. 3.2.3, eq. 3.34, p. 98
    void calcTransitionMatrix_Phi_Taylor(Scalar tau, size_t order)
    {
        INS_ASSERT_USER_ERROR(F.rowKeys() == Phi.rowKeys(), "The system model matrix F and the state transition matrix 𝚽 need to have the same keys.");

        Phi = transitionMatrix_Phi_Taylor(F, tau, order);
    }

    /// @brief Calculates the state transition matrix 𝚽 using the exponential matrix
    /// @param[in] tau Time interval in [s]
    /// @note See \cite Groves2013 Groves, ch. 3.2.3, eq. 3.33, p. 97
    /// @attention The cost of the computation is approximately 20n^3 for matrices of size n. The number 20 depends weakly on the norm of the matrix.
    void calcTransitionMatrix_Phi_exp(Scalar tau)
    {
        INS_ASSERT_USER_ERROR(F.rowKeys() == Phi.rowKeys(), "The system model matrix F and the state transition matrix 𝚽 need to have the same keys.");

        Phi = transitionMatrix_Phi_exp(F, tau);
    }

    /// @brief Numerical Method to calculate the State transition matrix 𝚽 and System/Process noise covariance matrix 𝐐
    /// @param[in] dt Time step in [s]
    /// @note See C.F. van Loan (1978) - Computing Integrals Involving the Matrix Exponential \cite Loan1978
    void calcPhiAndQWithVanLoanMethod(Scalar dt)
    {
        INS_ASSERT_USER_ERROR(G.colKeys() == W.rowKeys(), "The columns of the noise input matrix G and rows of the noise scale matrix W must match. (G * W * G^T)");
        INS_ASSERT_USER_ERROR(G.rowKeys() == Q.rowKeys(), "The rows of the noise input matrix G and the System/Process noise covariance matrix Q must match.");
        INS_ASSERT_USER_ERROR(G.colKeys() == Q.colKeys(), "The cols of the noise input matrix G and the System/Process noise covariance matrix Q must match.");

        auto [Phi, Q] = NAV::calcPhiAndQWithVanLoanMethod(F(all, all), G(all, all), W(all, all), dt);
        this->Phi(all, all) = Phi;
        this->Q(all, all) = Q;
    }

    /// @brief Whether a pre-update was saved
    [[nodiscard]] bool isPreUpdateSaved() const { return _savedPreUpdate.saved; }

    /// @brief Saves x̂, 𝐏, 𝐳, 𝐇, 𝐑 a-priori (pre-update)
    void savePreUpdate()
    {
        INS_ASSERT_USER_ERROR(!_savedPreUpdate.saved, "Cannot save the pre-update without restoring or discarding the old one first.");
        _savedPreUpdate.saved = true;

        _savedPreUpdate.x = x;
        _savedPreUpdate.P = P;
        _savedPreUpdate.Phi = Phi;
        _savedPreUpdate.Q = Q;
        _savedPreUpdate.z = z;
        _savedPreUpdate.H = H;
        _savedPreUpdate.R = R;
        _savedPreUpdate.S = S;
        _savedPreUpdate.K = K;

        _savedPreUpdate.F = F;
        _savedPreUpdate.G = G;
        _savedPreUpdate.W = W;
    }

    /// @brief Restores the saved x̂, 𝐏, 𝐳, 𝐇, 𝐑 a-priori (pre-update)
    void restorePreUpdate()
    {
        INS_ASSERT_USER_ERROR(_savedPreUpdate.saved, "Cannot restore the pre-update without saving one first.");
        _savedPreUpdate.saved = false;

        x = _savedPreUpdate.x;
        P = _savedPreUpdate.P;
        Phi = _savedPreUpdate.Phi;
        Q = _savedPreUpdate.Q;
        z = _savedPreUpdate.z;
        H = _savedPreUpdate.H;
        R = _savedPreUpdate.R;
        S = _savedPreUpdate.S;
        K = _savedPreUpdate.K;

        F = _savedPreUpdate.F;
        G = _savedPreUpdate.G;
        W = _savedPreUpdate.W;
    }
    /// @brief Discards the saved x̂, 𝐏, 𝐳, 𝐇, 𝐑 a-priori (pre-update)
    void discardPreUpdate()
    {
        INS_ASSERT_USER_ERROR(_savedPreUpdate.saved, "Cannot discard the pre-update without saving one first.");
        _savedPreUpdate.saved = false;
    }

    /// @brief Whether the NIS check should be performed
    [[nodiscard]] bool isNISenabled() const { return _checkNIS; }

    /// Normalized Innovation Squared (NIS) test results
    struct NISResult
    {
        bool triggered = false; ///< Whether the test triggered
        double NIS = 0.0;       ///< Normalized Innovation Squared (NIS)
        double r2 = 0.0;        ///< Upper boundary of one-sided acceptance interval
    };

    /// @brief Performs the Normalized Innovation Squared (NIS) test on the measurement innovation 𝜹𝐳
    /// @param[in] nameId Caller node for debug output
    ///
    /// H_0: The measurement residual 𝜹𝐳 is consistent with the innovation covariance matrix 𝗦
    /// The acceptance interval is chosen such that the probability that H_0 is accepted is (1 - alpha)
    /// @return The hypothesis test result if it failed, otherwise nothing.
    /// @attention Needs to be called before the update
    [[nodiscard]] auto checkForOutliersNIS([[maybe_unused]] const std::string& nameId)
    {
        NISResult ret{};
        if (z.rows() == 0) { return ret; }
        S(all, all) = H(all, all) * P(all, all) * H(all, all).transpose() + R(all, all);

        ret.NIS = std::abs(z(all).transpose() * S(all, all).inverse() * z(all));

        boost::math::chi_squared dist(static_cast<double>(z.rows()));

        ret.r2 = boost::math::quantile(dist, 1.0 - _alphaNIS);

        ret.triggered = ret.NIS >= ret.r2;

        if (ret.triggered)
        {
            LOG_DEBUG("{}: NIS test triggered because: NIS = {:.3f} > {:.3f} = r2", nameId, ret.NIS, ret.r2);
        }
        else
        {
            LOG_DATA("{}: NIS test passed: NIS = {:.3f} <= {:.3f} = r2", nameId, ret.NIS, ret.r2);
        }

        return ret;
    }

    /// @brief Shows GUI elements for the Kalman Filter
    /// @param[in] id Unique id for ImGui
    /// @param[in] width Width of the widget
    /// @return True if something was changed
    bool showKalmanFilterGUI(const char* id, float width = 0.0F)
    {
        bool changed = false;

        changed |= ImGui::Checkbox(fmt::format("Enable outlier NIS check##{}", id).c_str(), &_checkNIS);
        ImGui::SameLine();
        gui::widgets::HelpMarker("If the check has too many false positives, try increasing the process noise.");

        if (_checkNIS)
        {
            double alpha = _alphaNIS * 100.0;
            ImGui::SetNextItemWidth(width - ImGui::GetStyle().IndentSpacing);
            if (ImGui::DragDouble(fmt::format("NIS alpha (failure rate)##{}", id).c_str(), &alpha, 1.0F, 0.0, 100.0, "%.2f %%"))
            {
                _alphaNIS = alpha / 100.0;
                changed = true;
            }
        }

        return changed;
    }

    /// @brief Shows ImGui Tree nodes for all matrices
    /// @param[in] id Unique id for ImGui
    /// @param[in] nRows Amount of rows to show
    void showKalmanFilterMatrixViews(const char* id, int nRows = -2)
    {
        ImGui::PushFont(Application::MonoFont());
        float matrixTableHeight = ImGui::GetTextLineHeightWithSpacing() * static_cast<float>(nRows + 1);
        float vectorTableHeight = ImGui::GetTextLineHeightWithSpacing() * static_cast<float>(nRows);
        ImGui::PopFont();

        if (ImGui::TreeNode(fmt::format("x - State vector##{}", id).c_str()))
        {
            gui::widgets::KeyedVectorView(fmt::format("Kalman Filter x##{}", id).c_str(), &x, vectorTableHeight);
            ImGui::TreePop();
        }
        if (ImGui::TreeNode(fmt::format("P - Error covariance matrix##{}", id).c_str()))
        {
            gui::widgets::KeyedMatrixView(fmt::format("Kalman Filter P##{}", id).c_str(), &P, matrixTableHeight);
            ImGui::TreePop();
        }
        if (ImGui::TreeNode(fmt::format("Phi - State transition matrix##{}", id).c_str()))
        {
            gui::widgets::KeyedMatrixView(fmt::format("Kalman Filter Phi##{}", id).c_str(), &Phi, matrixTableHeight);
            ImGui::TreePop();
        }
        if (ImGui::TreeNode(fmt::format("Q System/Process noise covariance matrix##{}", id).c_str()))
        {
            gui::widgets::KeyedMatrixView(fmt::format("Kalman Filter Q##{}", id).c_str(), &Q, matrixTableHeight);
            ImGui::TreePop();
        }
        if (ImGui::TreeNode(fmt::format("z - Measurement vector##{}", id).c_str()))
        {
            gui::widgets::KeyedVectorView(fmt::format("Kalman Filter z##{}", id).c_str(), &z, vectorTableHeight);
            ImGui::TreePop();
        }
        if (ImGui::TreeNode(fmt::format("H - Measurement sensitivity matrix##{}", id).c_str()))
        {
            gui::widgets::KeyedMatrixView(fmt::format("Kalman Filter H##{}", id).c_str(), &H, matrixTableHeight);
            ImGui::TreePop();
        }
        if (ImGui::TreeNode(fmt::format("R - Measurement noise covariance matrix##{}", id).c_str()))
        {
            gui::widgets::KeyedMatrixView(fmt::format("Kalman Filter R##{}", id).c_str(), &R, matrixTableHeight);
            ImGui::TreePop();
        }
        if (ImGui::TreeNode(fmt::format("S - Measurement prediction covariance matrix##{}", id).c_str()))
        {
            gui::widgets::KeyedMatrixView(fmt::format("Kalman Filter S##{}", id).c_str(), &S, matrixTableHeight);
            ImGui::TreePop();
        }
        if (ImGui::TreeNode(fmt::format("K - Kalman gain matrix##{}", id).c_str()))
        {
            gui::widgets::KeyedMatrixView(fmt::format("Kalman Filter K##{}", id).c_str(), &K, matrixTableHeight);
            ImGui::TreePop();
        }
        if (ImGui::TreeNode(fmt::format("F - System model matrix##{}", id).c_str()))
        {
            gui::widgets::KeyedMatrixView(fmt::format("Kalman Filter F##{}", id).c_str(), &F, matrixTableHeight);
            ImGui::TreePop();
        }
        if (ImGui::TreeNode(fmt::format("G - Noise input matrix##{}", id).c_str()))
        {
            gui::widgets::KeyedMatrixView(fmt::format("Kalman Filter G##{}", id).c_str(), &G, matrixTableHeight);
            ImGui::TreePop();
        }
        if (ImGui::TreeNode(fmt::format("W - Noise scale matrix##{}", id).c_str()))
        {
            gui::widgets::KeyedMatrixView(fmt::format("Kalman Filter W##{}", id).c_str(), &W, matrixTableHeight);
            ImGui::TreePop();
        }
    }

    /// @brief Saved pre-update state and measurement
    struct SavedPreUpdate
    {
        bool saved = false; ///< Flag whether the state was saved

        KeyedVectorX<Scalar, StateKeyType> x;                 ///< x̂ State vector (n x 1)
        KeyedMatrixX<Scalar, StateKeyType, StateKeyType> P;   ///< 𝐏 Error covariance matrix (n x n)
        KeyedMatrixX<Scalar, StateKeyType, StateKeyType> Phi; ///< 𝚽 State transition matrix (n x n)
        KeyedMatrixX<Scalar, StateKeyType, StateKeyType> Q;   ///< 𝐐 System/Process noise covariance matrix (n x n)
        KeyedVectorX<Scalar, MeasKeyType> z;                  ///< 𝐳 Measurement vector (m x 1)
        KeyedMatrixX<Scalar, MeasKeyType, StateKeyType> H;    ///< 𝐇 Measurement sensitivity matrix (m x n)
        KeyedMatrixX<Scalar, MeasKeyType, MeasKeyType> R;     ///< 𝐑 = 𝐸{𝐰ₘ𝐰ₘᵀ} Measurement noise covariance matrix (m x m)
        KeyedMatrixX<Scalar, MeasKeyType, MeasKeyType> S;     ///< 𝗦 Measurement prediction covariance matrix (m x m)
        KeyedMatrixX<Scalar, StateKeyType, MeasKeyType> K;    ///< 𝐊 Kalman gain matrix (n x m)
                                                              ///
        KeyedMatrixX<Scalar, StateKeyType, StateKeyType> F;   ///< 𝐅 System model matrix (n x n)
        KeyedMatrixX<Scalar, StateKeyType, StateKeyType> G;   ///< 𝐆 Noise input matrix (n x o)
        KeyedMatrixX<Scalar, StateKeyType, StateKeyType> W;   ///< 𝐖 Noise scale matrix (o x o)
    };

    /// @brief Accesses the saved pre-update matrices
    [[nodiscard]] const SavedPreUpdate& savedPreUpdate() const { return _savedPreUpdate; }

  private:
    Eigen::MatrixXd I; ///< 𝑰 Identity matrix (n x n)

    SavedPreUpdate _savedPreUpdate; ///< Saved pre-update state and measurement

    /// @brief Normalized Innovation Squared (NIS) test
    bool _checkNIS = true;

    /// @brief NIS Test Hypothesis testing failure rate (probability that H_0 is accepted is (1 - alpha))
    double _alphaNIS = 0.05;

    /// @brief Converts the provided object into json
    /// @param[out] j Json object which gets filled with the info
    /// @param[in] obj Object to convert into json
    friend void to_json(json& j, const KeyedKalmanFilter<Scalar, StateKeyType, MeasKeyType>& obj)
    {
        j = json{
            { "checkNIS", obj._checkNIS },
            { "alphaNIS", obj._alphaNIS },
        };
    }
    /// @brief Converts the provided json object into a node object
    /// @param[in] j Json object with the needed values
    /// @param[out] obj Object to fill from the json
    friend void from_json(const json& j, KeyedKalmanFilter<Scalar, StateKeyType, MeasKeyType>& obj)
    {
        if (j.contains("checkNIS")) { j.at("checkNIS").get_to(obj._checkNIS); }
        if (j.contains("alphaNIS")) { j.at("alphaNIS").get_to(obj._alphaNIS); }
    }
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
