// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Ionosphere.hpp
/// @brief Ionosphere Models
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-05-26

#pragma once

#include <vector>
#include <Eigen/Core>
#include "Navigation/GNSS/Core/Frequency.hpp"
#include "IonosphericCorrections.hpp"

namespace NAV
{

/// Available Ionosphere Models
enum class IonosphereModel : int
{
    None,      ///< Ionosphere model turned off
    Klobuchar, ///< Klobuchar model (GPS), also called Broadcast sometimes
    COUNT,     ///< Amount of items in the enum
};

/// @brief Converts the enum to a string
/// @param[in] ionosphereModel Enum value to convert into text
/// @return String representation of the enum
const char* to_string(IonosphereModel ionosphereModel);

/// @brief Shows a ComboBox to select the ionosphere model
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in] ionosphereModel Reference to the ionosphere model to select
bool ComboIonosphereModel(const char* label, IonosphereModel& ionosphereModel);

/// @brief Calculates the ionospheric time delay with the Klobuchar model
/// @param[in] tow GPS time of week in [s]
/// @param[in] freq Frequency of the signal
/// @param[in] lla_pos [𝜙, λ, h]^T Geodetic latitude, longitude and height in [rad, rad, m]
/// @param[in] elevation Angle between the user and satellite [rad]
/// @param[in] azimuth Angle between the user and satellite, measured clockwise positive from the true North [rad]
/// @param[in] ionosphereModel Ionosphere model to use
/// @param[in] corrections Ionospheric correction parameters
/// @return Ionospheric time delay in [s]
double calcIonosphericTimeDelay(double tow, Frequency freq,
                                const Eigen::Vector3d& lla_pos,
                                double elevation, double azimuth,
                                IonosphereModel ionosphereModel = IonosphereModel::None,
                                const IonosphericCorrections* corrections = nullptr);

/// @brief Calculates the ionospheric error variance
/// @param[in] dpsr_I Ionosphere propagation error [m]
/// @param[in] freq Frequency
/// @param[in] num Frequency number. Only used for GLONASS G1 and G2
/// @return Variance of the error [m^2]
double ionoErrorVar(double dpsr_I, Frequency freq, int8_t num = -128);

} // namespace NAV
