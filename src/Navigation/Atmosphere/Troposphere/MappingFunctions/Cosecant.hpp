/// @file Cosecant.hpp
/// @brief Cosecant tropospheric mapping function
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-05-26

#pragma once

namespace NAV
{

/// @brief Calculates the mapping factor as cosecant of the elevation
/// @param[in] elevation Angle between the user and satellite [rad]
/// @return Mapping factor for converting tropospheric zenith delay into slant delay
double calcTropoMapFunc_cosecant(double elevation);

/// @brief Calculates the mapping factor as cosecant of the elevation (= secant of the zenith distance)
/// @param[in] zenithDistance Zenith distance/angle of the satellite [rad]
/// @return Mapping factor for converting tropospheric zenith delay into slant delay
double calcTropoMapFunc_secant(double zenithDistance);

} // namespace NAV
