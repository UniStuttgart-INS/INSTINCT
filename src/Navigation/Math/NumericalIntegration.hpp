/// @file NumericalIntegration.hpp
/// @brief Provides Numerical integration methods
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2021-09-08

#pragma once

#include <type_traits>

namespace NAV
{
/// Available Integration Algorithms
enum class IntegrationAlgorithm
{
    // RectangularRule, ///< Rectangular rule
    // Simpson,         ///< Simpson
    Heun,        ///< Heun's method
    RungeKutta1, ///< Runge-Kutta 1st order
    RungeKutta2, ///< Runge-Kutta 2nd order
    RungeKutta3, ///< Runge-Kutta 3rd order
    RungeKutta4, ///< Runge-Kutta 4th order
    COUNT,       ///< Amount of available integration algorithms
};

/// @brief Heun's method
/// @param[in] f  Model function
/// @param[in] h Integration step in [s]
/// @param[in] n_y State vector at time n_t
/// @param[in] n_t Time to evaluate the model function at in [s]
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y, typename Scalar,
         typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y Heun(const auto& f, const Scalar& h, const Y& n_y, const Scalar& n_t, const auto& c)
{
    Y y_n_p1 = n_y + h * f(n_y, n_t, c);

    return 0.5 * n_y + 0.5 * (y_n_p1 + h * f(y_n_p1, n_t + h, c));
}

/// @brief Heun's method
/// @param[in] f  Model function
/// @param[in] h Integration step in [s]
/// @param[in] n_y State vector at time n_t
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y, typename Scalar,
         typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y Heun(const auto& f, const Scalar& h, const Y& n_y, const auto& c)
{
    Y y_n_p1 = n_y + h * f(n_y, c);

    return 0.5 * n_y + 0.5 * (y_n_p1 + h * f(y_n_p1, c));
}

/// @brief Runge-Kutta First Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] n_y State vector at time n_t
/// @param[in] n_t Time to evaluate the model function at in [s]
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y, typename Scalar,
         typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y RungeKutta1(const auto& f, const Scalar& h, const Y& n_y, const Scalar& n_t, const auto& c)
{
    return n_y + h * f(n_y, n_t, c);
}

/// @brief Runge-Kutta First Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] n_y State vector at time n_t
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y, typename Scalar,
         typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y RungeKutta1(const auto& f, const Scalar& h, const Y& n_y, const auto& c)
{
    return n_y + h * f(n_y, c);
}

/// @brief Runge-Kutta Second Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] n_y State vector at time n_t
/// @param[in] n_t Time to evaluate the model function at in [s]
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y, typename Scalar,
         typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y RungeKutta2(const auto& f, const Scalar& h, const Y& n_y, const Scalar& n_t, const auto& c)
{
    Y k1 = f(n_y, n_t, c);
    Y k2 = f(n_y + h * k1, n_t + h, c);
    return n_y + h / 2 * (k1 + k2);
}

/// @brief Runge-Kutta Second Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] n_y State vector at time n_t
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y, typename Scalar,
         typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y RungeKutta2(const auto& f, const Scalar& h, const Y& n_y, const auto& c)
{
    Y k1 = f(n_y, c);
    Y k2 = f(n_y + h * k1, c);
    return n_y + h / 2 * (k1 + k2);
}

/// @brief Runge-Kutta Third Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] n_y State vector at time n_t
/// @param[in] n_t Time to evaluate the model function at in [s]
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y, typename Scalar,
         typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y RungeKutta3(const auto& f, const Scalar& h, const Y& n_y, const Scalar& n_t, const auto& c)
{
    Y k1 = f(n_y, n_t, c);
    Y k2 = f(n_y + h / 2 * k1, n_t + h / 2, c);
    Y k3 = f(n_y - h * k1 + 2 * h * k2, n_t + h, c);
    return n_y + h / 6 * (k1 + 4 * k2 + k3);
}

/// @brief Runge-Kutta Third Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] n_y State vector at time n_t
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y, typename Scalar,
         typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y RungeKutta3(const auto& f, const Scalar& h, const Y& n_y, const auto& c)
{
    Y k1 = f(n_y, c);
    Y k2 = f(n_y + h / 2 * k1, c);
    Y k3 = f(n_y - h * k1 + 2 * h * k2, c);
    return n_y + h / 6 * (k1 + 4 * k2 + k3);
}

/// @brief Runge-Kutta Fourth Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] n_y State vector at time n_t
/// @param[in] n_t Time to evaluate the model function at in [s]
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y, typename Scalar,
         typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y RungeKutta4(const auto& f, const Scalar& h, const Y& n_y, const Scalar& n_t, const auto& c)
{
    Y k1 = f(n_y, n_t, c);
    Y k2 = f(n_y + h / 2 * k1, n_t + h / 2, c);
    Y k3 = f(n_y + h / 2 * k2, n_t + h / 2, c);
    Y k4 = f(n_y + h * k3, n_t + h, c);
    return n_y + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
}

/// @brief Runge-Kutta Fourth Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] n_y State vector at time n_t
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y, typename Scalar,
         typename = std::enable_if_t<std::is_floating_point_v<Scalar>>>
Y RungeKutta4(const auto& f, const Scalar& h, const Y& n_y, const auto& c)
{
    Y k1 = f(n_y, c);
    Y k2 = f(n_y + h / 2 * k1, c);
    Y k3 = f(n_y + h / 2 * k2, c);
    Y k4 = f(n_y + h * k3, c);
    return n_y + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
}

/// @brief Converts the enum to a string
/// @param[in] algorithm Enum value to convert into text
/// @return String representation of the enum
const char* to_string(IntegrationAlgorithm algorithm);

} // namespace NAV
