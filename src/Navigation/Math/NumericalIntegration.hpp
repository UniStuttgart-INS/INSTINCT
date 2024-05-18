// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file NumericalIntegration.hpp
/// @brief Provides Numerical integration methods
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-12-11

#pragma once

#include <array>
#include <numeric>
#include <type_traits>
#include <gcem.hpp>

#include "util/Assert.h"
#include "util/Logger.hpp"

namespace NAV
{

namespace ButcherTableau
{

#pragma GCC diagnostic push
#if !defined(__clang__)
    #pragma GCC diagnostic ignored "-Wmaybe-uninitialized" // NOLINT(clang-diagnostic-unknown-warning-option)
#endif

/// @brief Calculates explicit Runge-Kutta methods. Order is defined by the Butcher tableau
/// @param[in] y_n State vector at time t_n
/// @param[in] z Array of measurements, one for each evaluation point of the Runge Kutta
/// @param[in] h Integration step in [s]
/// @param[in] f Time derivative function
/// @param[in] constParam Constant parameters passed to each time derivative function call
/// @param[in] t_n Time t_n
/// @return State vector at time t_(n+1)
template<typename Y, typename Z, typename Scalar, size_t s, std::array<std::array<Scalar, s + 1>, s + 1> butcherTableau,
         typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
inline Y RungeKuttaExplicit(const Y& y_n, const std::array<Z, s> z, const Scalar& h, const auto& f, const auto& constParam, const Scalar& t_n)
{
    static_assert(gcem::abs(static_cast<Scalar>(1.0) - std::accumulate(butcherTableau[s].begin(), butcherTableau[s].end(), static_cast<Scalar>(0.0))) < 1e-8,
                  "The sum of the last row in the Butcher tableau has to be 1");
    for (size_t r = 0; r <= s; ++r)
    {
        for (size_t c = r + 1; c <= s; ++c)
        {
            INS_ASSERT_USER_ERROR(butcherTableau.at(r).at(c) == 0.0, "All terms in the upper triangle have to be 0");
        }
    }

    // std::string result = "y_(n+1) = y_n";

    // std::string sum_b_str;
    // std::string sum_b_val;
    Y sum_b{};
    std::array<Y, s> k{};
    for (size_t i = 1; i <= s; ++i)
    {
        auto b = butcherTableau[s].at(i);
        auto c = butcherTableau.at(i - 1)[0];
        Y sum_a{};
        // std::string sum_a_str;
        // std::string sum_a_val;
        for (size_t j = 2; j <= i; ++j)
        {
            auto a = butcherTableau.at(i - 1).at(j - 1);
            if (j == 2) { sum_a = a * k.at(j - 2); }
            else { sum_a += a * k.at(j - 2); }
            // sum_a_str += fmt::format("a{}{} * k{}", i, j-1, j-1);
            // sum_a_val += fmt::format("{:^3.1f} * k{}", a, j - 1);
            // if (j < i) {
            //     sum_a_str += " + ";
            //     sum_a_val += " + ";
            // }
        }

        if (i < 2) { k.at(i - 1) = f(y_n, z.at(i - 1), constParam, t_n + c * h); } // Do not add sum_a, because can have nan values
        else { k.at(i - 1) = f(y_n + sum_a * h, z.at(i - 1), constParam, t_n + c * h); }
        // if (sum_a_str.empty()) { sum_a_str = "0"; }
        // if (sum_a_val.empty()) { sum_a_val = "0"; }
        // fmt::println("k{} = f(t_n +  c{} * h, y_n + ({}) * h)", i, i, sum_a_str);
        // fmt::println("k{} = f(t_n + {:^3.1f} * h, y_n + ({:^{w}}) * h)", i, c, sum_a_val, fmt::arg("w",sum_a_str.length()));

        if (i == 1) { sum_b = b * k.at(i - 1); }
        else { sum_b += b * k.at(i - 1); }
        // sum_b_str += fmt::format("b{} * k{}", i, i);
        // sum_b_val += fmt::format("{:.2f} * k{}", butcherTableau[s].at(i), i);
        // if (i < s)
        // {
        //     sum_b_str += " + ";
        //     sum_b_val += " + ";
        // }
    }

    // std::cout << result + fmt::format(" + h * ({})", sum_b_str) << std::endl;
    // std::cout << result + fmt::format(" + h * ({})", sum_b_val) << std::endl;
    return y_n + h * sum_b;
}

#pragma GCC diagnostic pop

/// @brief Butcher tableau for Runge-Kutta 1st order (explicit) / (Forward) Euler method
template<typename Scalar>
constexpr std::array<std::array<Scalar, 2>, 2> RK1 = { { { 0.0, /*|*/ },
                                                         //------------------
                                                         { 0.0, /*|*/ 1.0 } } };

/// @brief Butcher tableau for Runge-Kutta 2nd order (explicit) / Explicit midpoint method
template<typename Scalar>
constexpr std::array<std::array<Scalar, 3>, 3> RK2 = { { { 0.0, /*|*/ },
                                                         { 0.5, /*|*/ 0.5 },
                                                         //-----------------------
                                                         { 0.0, /*|*/ 0.0, 1.0 } } };

/// @brief Butcher tableau for Heun's method (2nd order) (explicit)
template<typename Scalar>
constexpr std::array<std::array<Scalar, 3>, 3> Heun2 = { { { 0.0, /*|*/ },
                                                           { 1.0, /*|*/ 1.0 },
                                                           //-----------------------
                                                           { 0.0, /*|*/ 0.5, 0.5 } } };

/// @brief Butcher tableau for Runge-Kutta 3rd order (explicit) / Simpson's rule
template<typename Scalar>
constexpr std::array<std::array<Scalar, 4>, 4> RK3 = { { { 0.0, /*|*/ },
                                                         { 0.5, /*|*/ 0.5 },
                                                         { 1.0, /*|*/ -1.0, 2.0 },
                                                         //----------------------------------------------
                                                         { 0.0, /*|*/ 1.0 / 6.0, 4.0 / 6.0, 1.0 / 6.0 } } };

/// @brief Butcher tableau for Heun's method (3nd order) (explicit)
template<typename Scalar>
constexpr std::array<std::array<Scalar, 4>, 4> Heun3 = { { { 0.0, /*      |*/ },
                                                           { 1.0 / 3.0, /*|*/ 1.0 / 3.0 },
                                                           { 2.0 / 3.0, /*|*/ 0.0, 2.0 / 3.0 },
                                                           //----------------------------------------
                                                           { 0.0, /*|*/ 1.0 / 4.0, 0.0, 3.0 / 4.0 } } };

/// @brief Butcher tableau for Runge-Kutta 4th order (explicit)
template<typename Scalar>
constexpr std::array<std::array<Scalar, 5>, 5> RK4 = { { { 0.0, /*|*/ },
                                                         { 0.5, /*|*/ 0.5 },
                                                         { 0.5, /*|*/ 0.0, 0.5 },
                                                         { 1.0, /*|*/ 0.0, 0.0, 1.0 },
                                                         //------------------
                                                         { 0.0, /*|*/ 1.0 / 6.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 6.0 } } };

} // namespace ButcherTableau

/// @brief Runge-Kutta 1st order (explicit) / (Forward) Euler method
/// \anchor eq-RungeKutta1-explicit \f{equation}{ \label{eq:eq-RungeKutta1-explicit}
///   y_{n+1} = y_n + h f(t_n, y_n)
/// \f}
/// Butcher tableau:
/// \anchor eq-RungeKutta1-explicit-bt \f{equation}{ \label{eq:eq-RungeKutta1-explicit-bt}
/// \renewcommand\arraystretch{1.2}
/// \begin{array}{c|c}
/// 0 \\ \hline
///   & 1 \\
/// \end{array}
/// \f}
/// @param[in] y_n State vector at time t_n
/// @param[in] z Array of measurements, one for each evaluation point of the Runge Kutta
/// @param[in] h Integration step in [s]
/// @param[in] f Time derivative function
/// @param[in] constParam Constant parameters passed to each time derivative function call
/// @param[in] t_n Time t_n
template<typename Y, typename Z, typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y RungeKutta1(const Y& y_n, const std::array<Z, 1> z, const Scalar& h, const auto& f, const auto& constParam, const Scalar& t_n = 0)
{
    return ButcherTableau::RungeKuttaExplicit<Y, Z, Scalar, 1, ButcherTableau::RK1<Scalar>>(y_n, z, h, f, constParam, t_n);
}

/// @brief Runge-Kutta 2nd order (explicit) / Explicit midpoint method
/// \anchor eq-RungeKutta2-explicit \f{equation}{ \label{eq:eq-RungeKutta2-explicit}
/// \begin{aligned}
///   y_{n+1} &= y_n + h k_2 \\[1em]
///   k_1     &= f(t_n, y_n) \\
///   k_2     &= f(t_n + \frac{h}{2}, y_n + \frac{h}{2} k_1) \\
/// \end{aligned}
/// \f}
/// Butcher tableau:
/// \anchor eq-RungeKutta2-explicit-bt \f{equation}{ \label{eq:eq-RungeKutta2-explicit-bt}
/// \renewcommand\arraystretch{1.2}
/// \begin{array}{c|cc}
/// 0 \\
/// \frac{1}{2} & \frac{1}{2} \\ \hline
///             &      0      & 1 \\
/// \end{array}
/// \f}
/// @param[in] y_n State vector at time t_n
/// @param[in] z Array of measurements, one for each evaluation point of the Runge Kutta
/// @param[in] h Integration step in [s]
/// @param[in] f Time derivative function
/// @param[in] constParam Constant parameters passed to each time derivative function call
/// @param[in] t_n Time t_n
template<typename Y, typename Z, typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y RungeKutta2(const Y& y_n, const std::array<Z, 2> z, const Scalar& h, const auto& f, const auto& constParam, const Scalar& t_n = 0)
{
    return ButcherTableau::RungeKuttaExplicit<Y, Z, Scalar, 2, ButcherTableau::RK2<Scalar>>(y_n, z, h, f, constParam, t_n);
}

/// @brief Heun's method (2nd order) (explicit)
/// \anchor eq-Heun2 \f{equation}{ \label{eq:eq-Heun2}
/// \begin{aligned}
///   y_{n+1} &= y_n + \frac{h}{2} (k_1 + k_2) \\[1em]
///   k_1     &= f(t_n, y_n) \\
///   k_2     &= f(t_n + h, y_n + h k_1) \\
/// \end{aligned}
/// \f}
/// Butcher tableau:
/// \anchor eq-Heun2-bt \f{equation}{ \label{eq:eq-Heun2-bt}
/// \renewcommand\arraystretch{1.2}
/// \begin{array}{c|cc}
/// 0 \\
/// 1 & 1 \\ \hline
///   & \frac{1}{2} & \frac{1}{2} \\
/// \end{array}
/// \f}
/// @param[in] y_n State vector at time t_n
/// @param[in] z Array of measurements, one for each evaluation point of the Runge Kutta
/// @param[in] h Integration step in [s]
/// @param[in] f Time derivative function
/// @param[in] constParam Constant parameters passed to each time derivative function call
/// @param[in] t_n Time t_n
template<typename Y, typename Z, typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y Heun2(const Y& y_n, const std::array<Z, 2> z, const Scalar& h, const auto& f, const auto& constParam, const Scalar& t_n = 0)
{
    return ButcherTableau::RungeKuttaExplicit<Y, Z, Scalar, 2, ButcherTableau::Heun2<Scalar>>(y_n, z, h, f, constParam, t_n);
}

/// @brief Runge-Kutta 3rd order (explicit) / Simpson's rule
/// \anchor eq-RungeKutta3-explicit \f{equation}{ \label{eq:eq-RungeKutta3-explicit}
/// \begin{aligned}
///   y_{n+1} &= y_n + \frac{h}{6} ( k_1 + 4 k_2 + k_3 ) \\[1em]
///   k_1     &= f(t_n, y_n) \\
///   k_2     &= f(t_n + \frac{h}{2}, y_n + \frac{h}{2} k_1) \\
///   k_3     &= f(t_n + h, y_n + h (-k_1 + 2 k_2)) \\
/// \end{aligned}
/// \f}
/// Butcher tableau:
/// \anchor eq-RungeKutta3-explicit-bt \f{equation}{ \label{eq:eq-RungeKutta3-explicit-bt}
/// \renewcommand\arraystretch{1.2}
/// \begin{array}{c|ccc}
/// 0 \\
/// \frac{1}{2} & \frac{1}{2} \\
///      1      &     -1      & 2 \\ \hline
///             & \frac{1}{6} & \frac{4}{6} & \frac{1}{6} \\
/// \end{array}
/// \f}
/// @param[in] y_n State vector at time t_n
/// @param[in] z Array of measurements, one for each evaluation point of the Runge Kutta
/// @param[in] h Integration step in [s]
/// @param[in] f Time derivative function
/// @param[in] constParam Constant parameters passed to each time derivative function call
/// @param[in] t_n Time t_n
template<typename Y, typename Z, typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y RungeKutta3(const Y& y_n, const std::array<Z, 3> z, const Scalar& h, const auto& f, const auto& constParam, const Scalar& t_n = 0)
{
    return ButcherTableau::RungeKuttaExplicit<Y, Z, Scalar, 3, ButcherTableau::RK3<Scalar>>(y_n, z, h, f, constParam, t_n);
}

/// @brief Heun's method (3nd order) (explicit)
/// \anchor eq-Heun3 \f{equation}{ \label{eq:eq-Heun3}
/// \begin{aligned}
///   y_{n+1} &= y_n + \frac{h}{4} (k_1 + 3 k_3) \\[1em]
///   k_1     &= f(t_n, y_n) \\
///   k_2     &= f(t_n + \frac{h}{3}, y_n + \frac{h}{3} k_1) \\
///   k_3     &= f(t_n + \frac{2 h}{3}, y_n + \frac{2 h}{3} k_2) \\
/// \end{aligned}
/// \f}
/// Butcher tableau:
/// \anchor eq-Heun3-bt \f{equation}{ \label{eq:eq-Heun3-bt}
/// \renewcommand\arraystretch{1.2}
/// \begin{array}{c|ccc}
/// 0 \\
/// \frac{1}{3} & \frac{1}{3} \\
/// \frac{2}{3} &      0      & \frac{2}{3} \\ \hline
///             & \frac{1}{4} &      0      & \frac{3}{4} \\
/// \end{array}
/// \f}
/// @param[in] y_n State vector at time t_n
/// @param[in] z Array of measurements, one for each evaluation point of the Runge Kutta
/// @param[in] h Integration step in [s]
/// @param[in] f Time derivative function
/// @param[in] constParam Constant parameters passed to each time derivative function call
/// @param[in] t_n Time t_n
template<typename Y, typename Z, typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y Heun3(const Y& y_n, const std::array<Z, 3> z, const Scalar& h, const auto& f, const auto& constParam, const Scalar& t_n = 0)
{
    return ButcherTableau::RungeKuttaExplicit<Y, Z, Scalar, 3, ButcherTableau::Heun3<Scalar>>(y_n, z, h, f, constParam, t_n);
}

/// @brief Runge-Kutta 4th order (explicit)
/// \anchor eq-RungeKutta4-explicit \f{equation}{ \label{eq:eq-RungeKutta4-explicit}
/// \begin{aligned}
///   y_{n+1} &= y_n + \frac{h}{6} ( k_1 + 2 k_2 + 2 k_3 + k_4 ) \\[1em]
///   k_1     &= f(t_n, y_n) \\
///   k_2     &= f(t_n + \frac{h}{2}, y_n + \frac{h}{2} k_1) \\
///   k_3     &= f(t_n + \frac{h}{2}, y_n + \frac{h}{2} k_2) \\
///   k_4     &= f(t_n + h, y_n + h k_3) \\
/// \end{aligned}
/// \f}
/// Butcher tableau:
/// \anchor eq-RungeKutta4-explicit-bt \f{equation}{ \label{eq:eq-RungeKutta4-explicit-bt}
/// \renewcommand\arraystretch{1.2}
/// \begin{array}{c|cccc}
/// 0 \\
/// \frac{1}{2} & \frac{1}{2} \\
/// \frac{1}{2} &      0      & \frac{1}{2} \\
///      1      &      0      &      0      &      1      \\ \hline
///             & \frac{1}{6} & \frac{1}{3} & \frac{1}{3} & \frac{1}{6} \\
/// \end{array}
/// \f}
/// @param[in] y_n State vector at time t_n
/// @param[in] z Array of measurements, one for each evaluation point of the Runge Kutta
/// @param[in] h Integration step in [s]
/// @param[in] f Time derivative function
/// @param[in] constParam Constant parameters passed to each time derivative function call
/// @param[in] t_n Time t_n
template<typename Y, typename Z, typename Scalar, typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y RungeKutta4(const Y& y_n, const std::array<Z, 4> z, const Scalar& h, const auto& f, const auto& constParam, const Scalar& t_n = 0)
{
    return ButcherTableau::RungeKuttaExplicit<Y, Z, Scalar, 4, ButcherTableau::RK4<Scalar>>(y_n, z, h, f, constParam, t_n);
}

} // namespace NAV
