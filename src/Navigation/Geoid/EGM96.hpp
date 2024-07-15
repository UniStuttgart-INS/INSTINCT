// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/

/// @file EGM96.hpp
/// @brief EGM96 geoid model
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-07-05
/// @note See https://github.com/emericg/EGM96/blob/master/EGM96.c

#pragma once

/* ************************************************************************** */
namespace NAV
{

/// @brief Compute the geoid undulation from the EGM96 potential coefficient model to The WGS84 ellipsoid.
/// @param[in] lat Latitude in [rad]
/// @param[in] lon Longitude in [rad]
/// @return The geoid undulation / altitude offset in [m]
double egm96_compute_altitude_offset(double lat, double lon);

} // namespace NAV