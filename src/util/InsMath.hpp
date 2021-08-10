/// @file InsMath.hpp
/// @brief
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-09-23

#pragma once

#include "InsConstants.hpp"

#include "util/Eigen.hpp"

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

/// @brief Calculates the roll angle from a static acceleration measurement
/// @param[in] accel_b Acceleration measurement in static condition in [m/s^2]
/// @return The roll angle in [rad]
///
/// @note See E.-H. Shin (2005) - Estimation Techniques for Low-Cost Inertial Navigation (Chapter 2.6)
double rollFromStaticAccelerationObs(const Eigen::Vector3d& accel_b);

/// @brief Calculates the pitch angle from a static acceleration measurement
/// @param[in] accel_b Acceleration measurement in static condition in [m/s^2]
/// @return The pitch angle in [rad]
///
/// @note See E.-H. Shin (2005) - Estimation Techniques for Low-Cost Inertial Navigation (Chapter 2.6)
double pitchFromStaticAccelerationObs(const Eigen::Vector3d& accel_b);

/// @brief Calculates the factorial of an unsigned integer
/// @param[in] n Unsigned integer
/// @return The factorial of 'n'
uint64_t factorial(uint64_t n);

/// @brief Calculates the skew symmetric matrix of the given vector.
///        This is needed to perform the cross product with a scalar product operation
/// @tparam _System The coordinate system
/// @param[in] a The vector
/// @return Skew symmetric matrix
/// @note See Groves (2013) equation (2.50)
template<typename _Scalar,
         typename = std::enable_if_t<std::is_arithmetic_v<_Scalar>>>
Eigen::Matrix<_Scalar, 3, 3> skewSymmetricMatrix(const Eigen::Matrix<_Scalar, 3, 1>& a)
{
    Eigen::Matrix<_Scalar, 3, 3> skewMat;
    skewMat << 0, -a(2), a(1),
        a(2), 0, -a(0),
        -a(1), a(0), 0;

    return skewMat;
}

/// @brief Calculates the secant of a value (1 / cos(x))
template<typename T,
         typename = std::enable_if_t<std::is_floating_point_v<T>>>
T secant(const T& x)
{
    return 1.0 / std::cos(x);
}

} // namespace NAV
