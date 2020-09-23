/// @file InsMath.hpp
/// @brief
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-09-23

#pragma once

#include "InsConstants.hpp"

namespace NAV
{
/// @brief Measure the distance between two points
/// @param[in] lat1 Latitude of first point in [rad]
/// @param[in] lon1 Longitude of first point in [rad]
/// @param[in] lat2 Latitude of second point in [rad]
/// @param[in] lon2 Longitude of second point in [rad]
/// @return The distance in [m]
///
/// @note See Haversine Formula (https://www.movable-type.co.uk/scripts/latlong.html)
double measureDistance(double lat1, double lon1, double lat2, double lon2);

} // namespace NAV
