// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ReceiverClockModel.hpp
/// @brief Receiver Clock System Model
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-08-20

#pragma once

#include <set>
#include <vector>

#include "Units.hpp"
#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "util/Assert.h"
#include "util/Container/KeyedMatrix.hpp"
#include "Navigation/GNSS/SystemModel/SystemModel.hpp"
#include "Navigation/Math/KalmanFilter.hpp"
#include "Navigation/Math/VanLoan.hpp"

namespace NAV
{
namespace Keys
{

/// @brief Receiver clock error [m]
struct RecvClkBias
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const RecvClkBias& rhs) const { return satSys == rhs.satSys; }
    /// @brief Satellite system
    SatelliteSystem satSys;
};
/// @brief Receiver clock drift [m/s]
struct RecvClkDrift
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const RecvClkDrift& rhs) const { return satSys == rhs.satSys; }
    /// @brief Satellite system
    SatelliteSystem satSys;
};

} // namespace Keys

/// Receiver Clock System Model
template<typename StateKeyType>
class ReceiverClockModel
{
  public:
    /// @brief Initializes the receiver clock model
    /// @param[in, out] F System model matrix
    /// @param[in, out] G Noise input matrix
    /// @param[in, out] W Noise scale matrix
    template<typename Scalar, int Size>
    void initialize(KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& F,
                    KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& G,
                    KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& W)
    {
        // Covariance of the clock phase drift [m²/s]
        _covarianceClkPhaseDrift = convertUnit(_gui_covarianceClkPhaseDrift, _gui_covarianceClkPhaseDriftUnit);
        // Covariance of the frequency phase drift [m²/s³]
        _covarianceClkFrequencyDrift = convertUnit(_gui_covarianceClkFrequencyDrift, _gui_covarianceClkFrequencyDriftUnit);

        for (const auto& key : F.rowKeys())
        {
            if (const auto* biasKey = std::get_if<Keys::RecvClkBias>(&key))
            {
                auto driftKey = Keys::RecvClkDrift{ biasKey->satSys };
                INS_ASSERT_USER_ERROR(F.hasRow(driftKey), "The model should have bias and drift");

                F(*biasKey, driftKey) = 1;
                G(*biasKey, *biasKey) = 1;
                G(driftKey, driftKey) = 1;
                W(*biasKey, *biasKey) = _covarianceClkPhaseDrift;
                W(driftKey, driftKey) = _covarianceClkFrequencyDrift;
            }
        }
    }

    /// @brief Updates the provided Phi and Q matrix
    /// @param[in, out] Phi State transition matrix
    /// @param[in, out] Q System/Process noise covariance matrix
    /// @param[in] F System model matrix
    /// @param[in] G Noise input matrix
    /// @param[in] W Noise scale matrix
    /// @param[in] dt Time step size in [s]
    /// @param[in] algorithm Algorithm to use for the calculation
    template<typename Scalar, int Size>
    void updatePhiAndQ(KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& Phi,
                       KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& Q,
                       const KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& F,
                       const KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& G,
                       const KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& W,
                       double dt,
                       SystemModelCalcAlgorithm algorithm) const
    {
        for (const auto& key : Phi.rowKeys())
        {
            if (const auto* bias = std::get_if<Keys::RecvClkBias>(&key))
            {
                auto drift = Keys::RecvClkDrift{ bias->satSys };
                if (Phi.hasRow(drift))
                {
                    std::vector<StateKeyType> keys = { *bias, drift };
                    if (algorithm == SystemModelCalcAlgorithm::VanLoan)
                    {
                        auto [PhiMot, QMot] = NAV::calcPhiAndQWithVanLoanMethod(
                            F.template block<2>(keys, keys),
                            G.template block<2>(keys, keys),
                            W.template block<2>(keys, keys),
                            dt);
                        Phi.template block<2>(keys, keys) = PhiMot;
                        Q.template block<2>(keys, keys) = QMot;
                    }
                    else // QCalculationAlgorithm::Taylor1
                    {
                        Phi.template block<2>(keys, keys) = transitionMatrix_Phi_Taylor(F.template block<2>(keys, keys), dt, 1);
                        Q.template block<2>(keys, keys) = calcProcessNoiseMatrixTaylor(dt, bias->satSys, keys)(all, all);
                    }
                }
                else
                {
                    Phi(*bias, *bias) = 1;
                    Q(*bias, *bias) = _covarianceClkPhaseDrift * dt;
                }
            }
        }
    }

    /// @brief Calculates the state transition matrix (𝚽) and the process noise covariance matrix (𝐐)
    /// @param[in] dt Time step size in [s]
    /// @param[in] satSys Satellite systems to use as keys
    /// @param[in] algorithm Algorithm to use for the calculation
    /// @return Phi and Q matrix
    [[nodiscard]] std::pair<KeyedMatrix2d<StateKeyType>, KeyedMatrix2d<StateKeyType>>
        calcPhiAndQ(double dt, SatelliteSystem satSys, SystemModelCalcAlgorithm algorithm)
    {
        std::vector<StateKeyType> keys = {
            Keys::RecvClkBias{ satSys },
            Keys::RecvClkDrift{ satSys },
        };

        KeyedMatrix2d<StateKeyType> F(Eigen::Matrix2d::Zero(), keys, keys);
        KeyedMatrix2d<StateKeyType> G(Eigen::Matrix2d::Zero(), keys, keys);
        KeyedMatrix2d<StateKeyType> W(Eigen::Matrix2d::Zero(), keys, keys);
        initialize(F, G, W);

        KeyedMatrix2d<StateKeyType> Phi(Eigen::Matrix2d::Zero(), keys, keys);
        KeyedMatrix2d<StateKeyType> Q(Eigen::Matrix2d::Zero(), keys, keys);
        updatePhiAndQ(Phi, Q, F, G, W, dt, algorithm);

        return { Phi, Q };
    }

    /// @brief Shows a GUI
    /// @param[in] itemWidth Width of the space for the config items
    /// @param[in] unitWidth Width of the units
    /// @param[in] id Unique id for ImGui
    /// @return True if something was changed
    bool ShowGui(float itemWidth, float unitWidth, const char* id)
    {
        bool changed = false;

        if (gui::widgets::InputDoubleWithUnit(fmt::format("{} of the receiver clock phase drift (RW)##{}",
                                                          _gui_covarianceClkPhaseDriftUnit == Units::CovarianceClkPhaseDriftUnits::m_sqrts
                                                              ? "StdDev"
                                                              : "Variance",
                                                          id)
                                                  .c_str(),
                                              itemWidth, unitWidth, &_gui_covarianceClkPhaseDrift,
                                              reinterpret_cast<int*>(&_gui_covarianceClkPhaseDriftUnit),
                                              MakeComboItems<Units::CovarianceClkPhaseDriftUnits>().c_str(),
                                              0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _gui_covarianceClkPhaseDrift changed to {}", id, _gui_covarianceClkPhaseDrift);
            LOG_DEBUG("{}: _gui_covarianceClkPhaseDriftUnit changed to {}", id, to_string(_gui_covarianceClkPhaseDriftUnit));
            changed = true;
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker(fmt::format("Typical values for a TCXO are {} {}",
                                             _gui_covarianceClkPhaseDriftUnit == Units::CovarianceClkPhaseDriftUnits::m_sqrts ? 0.1 : 0.01,
                                             to_string(_gui_covarianceClkPhaseDriftUnit))
                                     .c_str());

        if (gui::widgets::InputDoubleWithUnit(fmt::format("{} of the receiver clock frequency drift (IRW)##{}",
                                                          _gui_covarianceClkFrequencyDriftUnit == Units::CovarianceClkFrequencyDriftUnits::m_sqrts3
                                                              ? "StdDev"
                                                              : "Variance",
                                                          id)
                                                  .c_str(),
                                              itemWidth, unitWidth, &_gui_covarianceClkFrequencyDrift,
                                              reinterpret_cast<int*>(&_gui_covarianceClkFrequencyDriftUnit),
                                              MakeComboItems<Units::CovarianceClkFrequencyDriftUnits>().c_str(),
                                              0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _gui_covarianceClkFrequencyDrift changed to {}", id, _gui_covarianceClkFrequencyDrift);
            LOG_DEBUG("{}: _gui_covarianceClkFrequencyDriftUnit changed to {}", id, to_string(_gui_covarianceClkFrequencyDriftUnit));
            changed = true;
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker(fmt::format("Typical values for a TCXO are {} {}",
                                             _gui_covarianceClkFrequencyDriftUnit == Units::CovarianceClkFrequencyDriftUnits::m_sqrts3 ? 0.2 : 0.04,
                                             to_string(_gui_covarianceClkFrequencyDriftUnit))
                                     .c_str());

        return changed;
    }

  private:
    /// @brief Calculates the process noise covariance matrix with Taylor first order
    /// @param[in] dt Time step size in [s]
    /// @param[in] satSys Satellite system to update the keys for
    /// @param[in] keys List of keys (bias, drift)
    [[nodiscard]] KeyedMatrix<double, StateKeyType, StateKeyType, 2, 2>
        calcProcessNoiseMatrixTaylor(double dt, const SatelliteSystem& satSys, const std::vector<StateKeyType>& keys) const
    {
        double dt2 = std::pow(dt, 2);
        double dt3 = dt2 * dt;

        KeyedMatrix<double, StateKeyType, StateKeyType, 2, 2> Q(Eigen::Matrix<double, 2, 2>::Zero(), keys, keys);

        auto bias = Keys::RecvClkBias{ satSys };
        auto drift = Keys::RecvClkDrift{ satSys };
        Q(bias, bias) = _covarianceClkPhaseDrift * dt + _covarianceClkFrequencyDrift * dt3 / 3.0;
        Q(bias, drift) = _covarianceClkFrequencyDrift * dt2 / 2.0;
        Q(drift, bias) = Q(bias, drift);
        Q(drift, drift) = _covarianceClkFrequencyDrift * dt;

        return Q;
    }
    /// Gui selection for the Unit of the input covarianceClkPhaseDrift parameter
    Units::CovarianceClkPhaseDriftUnits _gui_covarianceClkPhaseDriftUnit = Units::CovarianceClkPhaseDriftUnits::m2_s;
    /// @brief GUI selection for the Standard deviation of the clock phase drift
    double _gui_covarianceClkPhaseDrift = 0.01 /*[ m^2 / s ] */;

    /// @brief Covariance of the clock phase drift [m²/s]
    double _covarianceClkPhaseDrift = 0.01;

    /// Gui selection for the Unit of the input covarianceClkFrequencyDrift parameter
    Units::CovarianceClkFrequencyDriftUnits _gui_covarianceClkFrequencyDriftUnit = Units::CovarianceClkFrequencyDriftUnits::m2_s3;
    /// @brief GUI selection for the Standard deviation of the clock frequency drift
    double _gui_covarianceClkFrequencyDrift = 0.04 /* [ m^2 / s^3 ] */;

    /// @brief Covariance of the clock frequency drift [m²/s³]
    double _covarianceClkFrequencyDrift = 0.04;

    /// @brief Converts the provided data into a json object
    /// @param[out] j Json object which gets filled with the info
    /// @param[in] data Data to convert into json
    friend void to_json(json& j, const ReceiverClockModel& data)
    {
        j = {
            { "covarianceClkPhaseDriftUnit", data._gui_covarianceClkPhaseDriftUnit },
            { "covarianceClkPhaseDrift", data._gui_covarianceClkPhaseDrift },
            { "covarianceClkFrequencyDriftUnit", data._gui_covarianceClkFrequencyDriftUnit },
            { "covarianceClkFrequencyDrift", data._gui_covarianceClkFrequencyDrift },
        };
    }

    /// @brief Converts the provided json object into the data object
    /// @param[in] j Json object with the needed values
    /// @param[out] data Object to fill from the json
    friend void from_json(const json& j, ReceiverClockModel& data)
    {
        if (j.contains("covarianceClkPhaseDriftUnit")) { j.at("covarianceClkPhaseDriftUnit").get_to(data._gui_covarianceClkPhaseDriftUnit); }
        if (j.contains("covarianceClkPhaseDrift")) { j.at("covarianceClkPhaseDrift").get_to(data._gui_covarianceClkPhaseDrift); }
        if (j.contains("covarianceClkFrequencyDriftUnit")) { j.at("covarianceClkFrequencyDriftUnit").get_to(data._gui_covarianceClkFrequencyDriftUnit); }
        if (j.contains("covarianceClkFrequencyDrift")) { j.at("covarianceClkFrequencyDrift").get_to(data._gui_covarianceClkFrequencyDrift); }
    }
};

} // namespace NAV

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::Keys::RecvClkBias& obj);

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::Keys::RecvClkDrift& obj);

namespace std
{

/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::Keys::RecvClkBias>
{
    /// @brief Hash function
    /// @param[in] recvClkErr Receiver clock errors
    size_t operator()(const NAV::Keys::RecvClkBias& recvClkErr) const
    {
        return std::hash<NAV::SatelliteSystem>()(recvClkErr.satSys);
    }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::Keys::RecvClkDrift>
{
    /// @brief Hash function
    /// @param[in] recvClkDrift Receiver clock drifts
    size_t operator()(const NAV::Keys::RecvClkDrift& recvClkDrift) const
    {
        return std::hash<NAV::SatelliteSystem>()(recvClkDrift.satSys);
    }
};

} // namespace std

#ifndef DOXYGEN_IGNORE

/// @brief Formatter
template<>
struct fmt::formatter<NAV::Keys::RecvClkBias> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] recvClkBias Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::Keys::RecvClkBias& recvClkBias, FormatContext& ctx) const
    {
        return fmt::formatter<std::string>::format(fmt::format("RecvClkBias({})", recvClkBias.satSys), ctx);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::Keys::RecvClkDrift> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] recvClkDrift Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::Keys::RecvClkDrift& recvClkDrift, FormatContext& ctx) const
    {
        return fmt::formatter<std::string>::format(fmt::format("RecvClkDrift({})", recvClkDrift.satSys), ctx);
    }
};

#endif