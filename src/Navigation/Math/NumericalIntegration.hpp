/// @file NumericalIntegration.hpp
/// @brief Provides Numerical integration methods
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2021-09-08

#pragma once

#include <type_traits>

namespace NAV
{
/// @brief Runge-Kutta First Order Algorithm (analogous to Euler Method)
/// @param[in] f Model function
/// @param[in] h Integration step in [s] (Time difference Δtₖ = (tₖ - tₖ₋₁))
/// @param[in] y__t1 State vector at time tₖ₋₁
/// @param[in] x__t1 Measurement at time tₖ₋₁
/// @return State vector at time tₖ
/// @note See C. Jekeli (2001) - Inertial Navigation Systems with Geodetic Applications (Chapter 2.4.1, eq. 2.105)
template<typename Y, typename Scalar, typename X,
         typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y rungeKutta1(Y (*f)(const X&, const Y&), const Scalar& h, const Y& y__t1, const X& x__t1)
{
    return y__t1 + h * f(x__t1, y__t1);
}

/// @brief Runge-Kutta Third Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s] (Time difference Δtₖ = (tₖ - tₖ₋₂))
/// @param[in] y__t2 State vector at time tₖ₋₂
/// @param[in] x__t2 Measurement at time tₖ₋₂
/// @param[in] x__t1 Measurement at time tₖ₋₁
/// @param[in] x__t0 Measurement at time tₖ
/// @return State vector at time tₖ
/// @note See Munz & Westermann (2012) - Numerische Behandlung gewöhnlicher und partieller Differenzialgleichungen (ch. 2.5, p. 97)
template<typename Y, typename Scalar, typename X,
         typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y rungeKutta3(Y (*f)(const X&, const Y&), const Scalar& h, const Y& y__t2, const X& x__t2, const X& x__t1, const X& x__t0)
{
    // Runge-Kutta coefficient k₁
    const auto k1 = f(x__t2, y__t2);
    // Runge-Kutta coefficient k₂
    const auto k2 = f(x__t1, y__t2
                                 + h / 2.0 * k1);
    // Runge-Kutta coefficient k₃
    const auto k3 = f(x__t0, y__t2
                                 - h * k1
                                 + 2.0 * h * k2);

    return y__t2 + h / 6.0 * (k1 + 4.0 * k2 + k3);
}

/// @brief Runge-Kutta First Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] y_n State vector at time t_n
/// @param[in] t_n Time to evaluate the model function at in [s]
/// @return State vector at time t_(n+1)
template<typename Y, typename Scalar,
         typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y RungeKutta1(Y (*f)(const Y&, const Scalar&), const Scalar& h, const Y& y_n, const Scalar& t_n)
{
    return y_n + h * f(y_n, t_n);
}

/// @brief Runge-Kutta Second Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] y_n State vector at time t_n
/// @param[in] t_n Time to evaluate the model function at in [s]
/// @return State vector at time t_(n+1)
template<typename Y, typename Scalar,
         typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y RungeKutta2(Y (*f)(const Y&, const Scalar&), const Scalar& h, const Y& y_n, const Scalar& t_n)
{
    auto k1 = f(y_n, t_n);
    auto k2 = f(y_n + h * k1, t_n + h);
    return y_n + h / 2 * (k1 + k2);
}

/// @brief Runge-Kutta Third Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] y_n State vector at time t_n
/// @param[in] t_n Time to evaluate the model function at in [s]
/// @return State vector at time t_(n+1)
template<typename Y, typename Scalar,
         typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y RungeKutta3(Y (*f)(const Y&, const Scalar&), const Scalar& h, const Y& y_n, const Scalar& t_n)
{
    auto k1 = f(y_n, t_n);
    auto k2 = f(y_n + h / 2 * k1, t_n + h / 2);
    auto k3 = f(y_n - h * k1 + 2 * h * k2, t_n + h);
    return y_n + h / 6 * (k1 + 4 * k2 + k3);
}

/// @brief Runge-Kutta Fourth Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] y_n State vector at time t_n
/// @param[in] t_n Time to evaluate the model function at in [s]
/// @return State vector at time t_(n+1)
template<typename Y, typename Scalar,
         typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y RungeKutta4(Y (*f)(const Y&, const Scalar&), const Scalar& h, const Y& y_n, const Scalar& t_n)
{
    auto k1 = f(y_n, t_n);
    auto k2 = f(y_n + h / 2 * k1, t_n + h / 2);
    auto k3 = f(y_n + h / 2 * k2, t_n + h / 2);
    auto k4 = f(y_n + h * k3, t_n + h);
    return y_n + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
}

} // namespace NAV
