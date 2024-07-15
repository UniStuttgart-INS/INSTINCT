// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SNRMask.hpp
/// @brief Signal to Noise Ratio Mask
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-12-17

#pragma once

#include <array>

#include "Navigation/GNSS/Core/Frequency.hpp"
#include "Navigation/Transformations/Units.hpp"

namespace NAV
{

/// Signal to Noise Ratio Mask
class SNRMask
{
  public:
    /// @brief Default Constructor
    SNRMask();

    /// @brief Shows a button to select the SNR Mask
    /// @param[in] label Text to display on the button. If empty, just 'SNR Mask' will be written
    bool ShowGuiWidgets(const char* label = nullptr);

    /// @brief Checks wether all SNR values are 0
    [[nodiscard]] bool isInactive() const;

    /// @brief Checks wether the SNR values passes the SNR mask
    /// @param[in] freq Frequency of the signal
    /// @param[in] elevation Elevation in [rad] of the satellite transmitting the signal
    /// @param[in] SNR Signal to Noise Ratio in [dbHz]
    /// @return True if the value passed the mask
    [[nodiscard]] bool checkSNRMask(Frequency freq, double elevation, double SNR) const;

  private:
    /// @brief Elevations [rad]. Checks to smaller or equal than the value
    static constexpr std::array<double, 10> elevations = {
        deg2rad(5.0),
        deg2rad(15.0),
        deg2rad(25.0),
        deg2rad(35.0),
        deg2rad(45.0),
        deg2rad(55.0),
        deg2rad(65.0),
        deg2rad(75.0),
        deg2rad(85.0),
        deg2rad(90.0),
    };

    /// @brief Values when changed override all others
    std::pair<std::array<double, elevations.size()>, bool> allOverride = { {}, true };

    /// @brief Masks for all frequencies and SNR [dbHz] values + lock together boolean
    std::array<std::pair<Frequency, std::pair<std::array<double, elevations.size()>, bool>>, Frequency::GetAll().size()> mask;

    friend void to_json(json& j, const SNRMask& obj);
    friend void from_json(const json& j, SNRMask& obj);
};

/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
void to_json(json& j, const SNRMask& obj);
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
void from_json(const json& j, SNRMask& obj);

} // namespace NAV
