/// @file InsGravity.hpp
/// @brief Different Gravity Models
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-09-15

#pragma once

namespace NAV::gravity
{
/// @brief Calculates the magnitude of the gravity vector using WGS84 ellipsoid
/// @param[in] latitude Latitude where to calculate the gravity for
/// @return Magnitude of the gravity vector
///
/// @note See S. Gleason (2009) - GNSS Applications and Methods (Chapter 6.2.3.2 - eq. 6.16)
[[nodiscard]] double gravityMagnitude_Gleason(const double& latitude);

} // namespace NAV::gravity
