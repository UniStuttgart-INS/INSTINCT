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

#include <cstdint>
#include <type_traits>
#include <Eigen/Core>
#include <gcem.hpp>
#include <fmt/format.h>

#include "util/Assert.h"

namespace NAV::math
{

/// @brief Calculates the factorial of an unsigned integer
/// @param[in] n Unsigned integer
/// @return The factorial of 'n'
uint64_t factorial(uint64_t n);

/// @brief Round the number to the specified amount of digits
/// @param[in] value Value to round
/// @param[in] digits Amount of digits
/// @return The rounded value
template<typename T,
         typename = std::enable_if_t<std::is_floating_point_v<T>>>
constexpr T round(const T& value, size_t digits)
{
    auto factor = std::pow(10, digits);
    return std::round(value * factor) / factor;
}

/// @brief Round the number to the specified amount of significant digits
/// @param[in] value Value to round
/// @param[in] digits Amount of digits
/// @return The rounded value
template<typename T,
         typename = std::enable_if_t<std::is_floating_point_v<T>>>
constexpr T roundSignificantDigits(T value, size_t digits)
{
    if (value == 0.0) { return 0.0; }
    // LOG_DEBUG("value = {:.13e} --> Round to {} digits", value, digits);
    auto absVal = gcem::abs(value);
    auto log10 = static_cast<int32_t>(gcem::log10(absVal));
    auto exp = log10 + (log10 > 0 || (log10 == 0 && absVal >= 1.0));
    auto fac = static_cast<T>(digits) - static_cast<T>(exp);
    // LOG_DEBUG("  log10  = {}, exp = {}, fac = {}", log10, exp, fac);
    auto factor = static_cast<T>(gcem::pow(10.0, fac));
    // LOG_DEBUG("  factor = {:.0e} --> value * factor = {}", factor, value * factor);
    // LOG_DEBUG("  round = {} --> ... / factor = {}", gcem::round(value * factor), gcem::round(value * factor) / factor);
    return static_cast<T>(gcem::round(value * factor) / factor);
}

/// @brief Interprets the input integer with certain amount of Bits as Output type. Takes care of sign extension
/// @tparam Out Output type
/// @tparam Bits Size of the input data
/// @tparam In Input data type (needs to be bigger than the amount of Bits)
/// @param[in] in Number as two's complement, with the sign bit (+ or -) occupying the MSB
/// @return Output type
template<typename Out, size_t Bits, typename In,
         typename = std::enable_if_t<std::is_integral_v<Out>>,
         typename = std::enable_if_t<std::is_integral_v<In>>>
constexpr Out interpretAs(In in)
{
    static_assert(Bits < sizeof(In) * 8);
    static_assert(Bits < sizeof(Out) * 8);

    constexpr size_t N = sizeof(Out) * 8 - Bits;
    return static_cast<Out>(static_cast<Out>((in & static_cast<In>(gcem::pow(2, Bits) - 1)) << N) >> N);
}

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