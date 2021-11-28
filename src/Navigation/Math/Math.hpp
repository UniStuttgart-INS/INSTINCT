/// @file InsMath.hpp
/// @brief Simple Math functions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-09-23

#pragma once

#include <cstdint>
#include <Eigen/Core>

namespace NAV::Math
{

/// @brief Calculates the factorial of an unsigned integer
/// @param[in] n Unsigned integer
/// @return The factorial of 'n'
uint64_t factorial(uint64_t n);

/// @brief Calculates the skew symmetric matrix of the given vector.
///        This is needed to perform the cross product with a scalar product operation
/// @tparam _Scalar Data type of the Matrix
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

} // namespace NAV::Math
