// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Math.hpp
/// @brief Simple Math functions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author N. Stahl (HiWi: Elliptical integral)
/// @date 2023-07-04

#pragma once

#include "util/Assert.h"
#include <cstdint>
#include <Eigen/Core>

namespace NAV::math
{

/// @brief Calculates the factorial of an unsigned integer
/// @param[in] n Unsigned integer
/// @return The factorial of 'n'
uint64_t factorial(uint64_t n);

/// @brief Calculates the skew symmetric matrix of the given vector.
///        This is needed to perform the cross product with a scalar product operation
/// @tparam Derived Derived Eigen Type
/// @param[in] a The vector
/// @return Skew symmetric matrix
/// @note See Groves (2013) equation (2.50)
template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetricMatrix(const Eigen::MatrixBase<Derived>& a)
{
    INS_ASSERT_USER_ERROR(a.cols() == 1, "Given Eigen Object must be a vector");
    INS_ASSERT_USER_ERROR(a.rows() == 3, "Given Vector must have 3 Rows");

    Eigen::Matrix<typename Derived::Scalar, 3, 3> skewMat;
    skewMat << 0, -a(2), a(1),
        a(2), 0, -a(0),
        -a(1), a(0), 0;

    return skewMat;
}

/// @brief Calculates the square of a skew symmetric matrix of the given vector.
/// @tparam Derived Derived Eigen Type
/// @param[in] a The vector
/// @return Square of skew symmetric matrix
template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetricMatrixSquared(const Eigen::MatrixBase<Derived>& a)
{
    INS_ASSERT_USER_ERROR(a.cols() == 1, "Given Eigen Object must be a vector");
    INS_ASSERT_USER_ERROR(a.rows() == 3, "Given Vector must have 3 Rows");

    Eigen::Matrix<typename Derived::Scalar, 3, 3> skewMat2;
    skewMat2 << std::pow(a(2), 2) + std::pow(a(1), 2), a(0) * a(1), a(0) * a(2),
        a(0) * a(1), std::pow(a(2), 2) + std::pow(a(0), 2), a(1) * a(2),
        a(0) * a(2), a(1) * a(2), std::pow(a(0), 2) + std::pow(a(1), 2);

    return skewMat2;
}

/// @brief Calculates the secant of a value (sec(x) = csc(pi/2 - x) = 1 / cos(x))
template<typename T,
         typename = std::enable_if_t<std::is_floating_point_v<T>>>
T sec(const T& x)
{
    return 1.0 / std::cos(x);
}

/// @brief Calculates the cosecant of a value (csc(x) = sec(pi/2 - x) = 1 / sin(x))
template<typename T,
         typename = std::enable_if_t<std::is_floating_point_v<T>>>
T csc(const T& x)
{
    return 1.0 / std::sin(x);
}

/// @brief Returns the sign of the given value
/// @param[in] val Value to get the sign from
/// @return Sign of the given value
template<typename T>
int sgn(const T& val)
{
    return (T(0) < val) - (val < T(0));
}

/// @brief Change the sign of x according to the value of y
/// @param[in] x input value
/// @param[in] y input value
/// @return -x or +x
template<typename T>
T sign(const T& x, const T& y)
{
    // similar function 'sign' in fortran
    if (y >= 0.0)
    {
        return fabs(x);
    }
    return -1.0 * fabs(x);
}

/// @brief Calculates the incomplete elliptical integral of the second kind
/// @param[in] phi Interval bound the integration uses from 0 to phi
/// @param[in] m Function parameter that is integrated 1-m*sin(t)^2
/// @return Incomplete elliptical integral of the second kind
/// @note See http://www2.iap.fr/users/pichon/doc/html_xref/elliptic-es.html
double calcEllipticalIntegral(double phi, double m);

} // namespace NAV::math