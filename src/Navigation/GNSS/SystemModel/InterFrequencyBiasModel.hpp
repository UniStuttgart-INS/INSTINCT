// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file InterFrequencyBiasModel.hpp
/// @brief Inter Frequency Bias System Model
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-08-20

#pragma once

#include "Units.hpp"

#include "Navigation/GNSS/SystemModel/SystemModel.hpp"
#include "util/Container/KeyedMatrix.hpp"
#include <fmt/format.h>
#include "Navigation/GNSS/SystemModel/Units.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "Navigation/GNSS/Core/Frequency.hpp"

#include "util/Logger.hpp"

namespace NAV
{

namespace Keys
{

/// @brief Inter-frequency bias
struct InterFreqBias
{
    /// @brief Equal comparison operator
    /// @param rhs Right-hand side
    bool operator==(const InterFreqBias& rhs) const { return freq == rhs.freq; }
    /// @brief Frequency
    Frequency freq;
};

} // namespace Keys

/// Inter Frequency Bias System Model
template<typename StateKeyType>
class InterFrequencyBiasModel
{
  public:
    /// @brief Initializes the inter-frequency bias
    /// @param[in] bias Bias to init
    /// @param[in, out] F System model matrix
    /// @param[in, out] G Noise input matrix
    /// @param[in, out] W Noise scale matrix
    template<typename Scalar, int Size>
    void initialize(Keys::InterFreqBias bias,
                    KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& F,
                    KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& G,
                    KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& W)
    {
        // Covariance of the inter-frequency bias [m²/s]
        _covarianceInterFrequencyBias = convertUnit(_gui_covarianceInterFrequencyBias, _gui_covarianceInterFrequencyBiasUnit);

        F(bias, bias) = 0;
        G(bias, bias) = 1;
        W(bias, bias) = _covarianceInterFrequencyBias;
    }

    /// @brief Updates the provided Phi and Q matrix
    /// @param[in, out] Phi State transition matrix
    /// @param[in, out] Q System/Process noise covariance matrix
    /// @param[in] dt Time step size in [s]
    template<typename Scalar, int Size>
    void updatePhiAndQ(KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& Phi,
                       KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& Q,
                       double dt)
    {
        for (const auto& key : Phi.rowKeys())
        {
            if (const auto* bias = std::get_if<Keys::InterFreqBias>(&key))
            {
                Phi(*bias, *bias) = 1;
                Q(*bias, *bias) = _covarianceInterFrequencyBias * dt;
            }
        }
    }

    /// @brief Calculates the state transition matrix (𝚽) and the process noise covariance matrix (𝐐)
    /// @param[in] dt Time step size in [s]
    /// @param[in] freq Frequency to use
    /// @return Phi and Q matrix
    [[nodiscard]] std::pair<double, double> calcPhiAndQ(double dt, const Frequency& freq)
    {
        std::vector<StateKeyType> key = { Keys::InterFreqBias{ freq } };

        KeyedMatrix<double, StateKeyType, StateKeyType, 1, 1> Phi(Eigen::Matrix<double, 1, 1>::Zero(), key, key);
        KeyedMatrix<double, StateKeyType, StateKeyType, 1, 1> Q(Eigen::Matrix<double, 1, 1>::Zero(), key, key);
        updatePhiAndQ(Phi, Q, dt);

        return { Phi(all, all)(0), Q(all, all)(0) };
    }

    /// @brief Shows a GUI
    /// @param[in] itemWidth Width of the space for the config items
    /// @param[in] unitWidth Width of the units
    /// @param[in] id Unique id for ImGui
    /// @return True if something was changed
    bool ShowGui(float itemWidth, float unitWidth, const char* id)
    {
        bool changed = false;

        if (gui::widgets::InputDoubleWithUnit(fmt::format("{} of the inter-frequency bias (RW)##{}",
                                                          _gui_covarianceInterFrequencyBiasUnit == Units::CovarianceClkPhaseDriftUnits::m_sqrts
                                                              ? "StdDev"
                                                              : "Variance",
                                                          id)
                                                  .c_str(),
                                              itemWidth, unitWidth, &_gui_covarianceInterFrequencyBias,
                                              reinterpret_cast<int*>(&_gui_covarianceInterFrequencyBiasUnit),
                                              MakeComboItems<Units::CovarianceClkPhaseDriftUnits>().c_str(),
                                              0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _gui_covarianceInterFrequencyBias changed to {}", id, _gui_covarianceInterFrequencyBias);
            LOG_DEBUG("{}: _gui_covarianceInterFrequencyBiasUnit changed to {}", id, to_string(_gui_covarianceInterFrequencyBiasUnit));
            changed = true;
        }

        return changed;
    }

  private:
    /// Gui selection for the Unit of the inter-frequency covarianceInterFrequencyBias parameter
    Units::CovarianceClkPhaseDriftUnits _gui_covarianceInterFrequencyBiasUnit = Units::CovarianceClkPhaseDriftUnits::m2_s;

    /// @brief GUI selection for the Standard deviation of the inter-frequency bias
    double _gui_covarianceInterFrequencyBias = 1e-6 /* [m²/s] */;
    /// @brief Covariance of the inter-frequency bias [m²/s]
    double _covarianceInterFrequencyBias = 1e-6;

    /// @brief Converts the provided data into a json object
    /// @param[out] j Json object which gets filled with the info
    /// @param[in] data Data to convert into json
    friend void to_json(json& j, const InterFrequencyBiasModel& data)
    {
        j = {
            { "covarianceInterFrequencyBiasUnit", data._gui_covarianceInterFrequencyBiasUnit },
            { "covarianceInterFrequencyBias", data._gui_covarianceInterFrequencyBias },
        };
    }

    /// @brief Converts the provided json object into the data object
    /// @param[in] j Json object with the needed values
    /// @param[out] data Object to fill from the json
    friend void from_json(const json& j, InterFrequencyBiasModel& data)
    {
        if (j.contains("covarianceInterFrequencyBiasUnit")) { j.at("covarianceInterFrequencyBiasUnit").get_to(data._gui_covarianceInterFrequencyBiasUnit); }
        if (j.contains("covarianceInterFrequencyBias")) { j.at("covarianceInterFrequencyBias").get_to(data._gui_covarianceInterFrequencyBias); }
    }
};

} // namespace NAV

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::Keys::InterFreqBias& obj);

namespace std
{

/// @brief Hash function (needed for unordered_map)
template<>
struct hash<NAV::Keys::InterFreqBias>
{
    /// @brief Hash function
    /// @param[in] interFreqBias Inter-frequency bias
    size_t operator()(const NAV::Keys::InterFreqBias& interFreqBias) const
    {
        return std::hash<NAV::Frequency>()(interFreqBias.freq);
    }
};

} // namespace std

#ifndef DOXYGEN_IGNORE

/// @brief Formatter
template<>
struct fmt::formatter<NAV::Keys::InterFreqBias> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in] interFreqBias Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::Keys::InterFreqBias& interFreqBias, FormatContext& ctx) const
    {
        return fmt::formatter<std::string>::format(fmt::format("InterFreqBias({})", interFreqBias.freq), ctx);
    }
};

#endif