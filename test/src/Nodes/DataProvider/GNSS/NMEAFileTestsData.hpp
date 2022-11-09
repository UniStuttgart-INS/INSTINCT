// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file NMEAFileTestsData.hpp
/// @brief Data definitions for the NMEAFileTests
/// @author T. Hobiger (thomas.hobiger@ins.uni-stuttgart.de)
/// @date 2022-11-08

#pragma once

#include <array>

namespace NAV::TEST::NMEAFileTests
{

enum ImuRef : size_t
{
    NMEA_Year,
	NMEA_Month,
	NMEA_Day,
	NMEA_Hour,
	NMEA_Minute,
	NMEA_Second,
	NMEA_Latitude_rad,
	NMEA_Longitude_rad,
	NMEA_Height,
};

constexpr std::array<std::array<long double, 9>,3> NMEA_REFERENCE_DATA = { {
	{ 2022, 11, 5, 16, 16 ,  0.756, 0.916818838078743, 0.233908153687654, 0.0},
	{ 2022, 11, 5, 16, 16 ,  1.756, 0.916802548339058, 0.234095485694035, 0.0},
	{ 2022, 11, 3, 14, 05 , 18.000, 0.851383885268465, 0.160074500200709, 327.812},
} };


} // namespace NAV::TEST::NMEAFileTests