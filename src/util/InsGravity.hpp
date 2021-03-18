/// @file InsGravity.hpp
/// @brief Different Gravity Models
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-09-15

#pragma once

namespace NAV::gravity
{
/// @brief Calculates the magnitude of the of local gravity at the WGS84 reference elliposid
///        using the Somigliana model and makes corrections for altitude
/// @param[in] latitude Latitude where to calculate the gravity for
/// @param[in] altitude Altitude where to calculate the gravity for
/// @return Magnitude of the gravity vector in [m/s^2]
///
/// @note See S. Gleason (2009) - GNSS Applications and Methods (Chapter 6.2.3.2 - eq. 6.16)
[[nodiscard]] double gravityMagnitude_SomiglianaAltitude(const double& latitude, const double& altitude);

} // namespace NAV::gravity
