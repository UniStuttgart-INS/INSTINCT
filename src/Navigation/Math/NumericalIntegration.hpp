/// @file NumericalIntegration.hpp
/// @brief Provides Numerical integration methods
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2021-09-08

#pragma once

#include <concepts>

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
/// @param[in] y_n State vector at time t_n
/// @param[in] z_n Measurement vector at time t_n
/// @param[in] z_n_p1 Measurement vector at time t_(n+1)
/// @param[in] t_n Time to evaluate the model function at in [s]
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y, typename Z>
Y Heun(const auto& f, const std::floating_point auto& h, const Y& y_n, const Z& z_n, const Z& z_n_p1, const std::floating_point auto& t_n, const auto& c)
{
    Y y_n_p1 = y_n + h * f(y_n, z_n, t_n, c);

    return 0.5 * y_n + 0.5 * (y_n_p1 + h * f(y_n_p1, z_n_p1, t_n + h, c));
}

/// @brief Heun's method
/// @param[in] f  Model function
/// @param[in] h Integration step in [s]
/// @param[in] y_n State vector at time t_n
/// @param[in] z_n Measurement vector at time t_n
/// @param[in] z_n_p1 Measurement vector at time t_(n+1)
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y, typename Z>
Y Heun(const auto& f, const std::floating_point auto& h, const Y& y_n, const Z& z_n, const Z& z_n_p1, const auto& c)
{
    Y y_n_p1 = y_n + h * f(y_n, z_n, c);

    return 0.5 * y_n + 0.5 * (y_n_p1 + h * f(y_n_p1, z_n_p1, c));
}

/// @brief Runge-Kutta First Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] y_n State vector at time t_n
/// @param[in] t_n Time to evaluate the model function at in [s]
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y>
Y RungeKutta1(const auto& f, const std::floating_point auto& h, const Y& y_n, const std::floating_point auto& t_n, const auto& c)
{
    return y_n + h * f(y_n, t_n, c);
}

/// @brief Runge-Kutta First Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] y_n State vector at time t_n
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y>
Y RungeKutta1(const auto& f, const std::floating_point auto& h, const Y& y_n, const auto& c)
{
    return y_n + h * f(y_n, c);
}

/// @brief Runge-Kutta Second Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] y_n State vector at time t_n
/// @param[in] t_n Time to evaluate the model function at in [s]
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y>
Y RungeKutta2(const auto& f, const std::floating_point auto& h, const Y& y_n, const std::floating_point auto& t_n, const auto& c)
{
    Y k1 = f(y_n, t_n, c);
    Y k2 = f(y_n + h * k1, t_n + h, c);
    return y_n + h / 2 * (k1 + k2);
}

/// @brief Runge-Kutta Second Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] y_n State vector at time t_n
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y>
Y RungeKutta2(const auto& f, const std::floating_point auto& h, const Y& y_n, const auto& c)
{
    Y k1 = f(y_n, c);
    Y k2 = f(y_n + h * k1, c);
    return y_n + h / 2 * (k1 + k2);
}

/// @brief Runge-Kutta Third Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] y_n State vector at time t_n
/// @param[in] t_n Time to evaluate the model function at in [s]
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y>
Y RungeKutta3(const auto& f, const std::floating_point auto& h, const Y& y_n, const std::floating_point auto& t_n, const auto& c)
{
    Y k1 = f(y_n, t_n, c);
    Y k2 = f(y_n + h / 2 * k1, t_n + h / 2, c);
    Y k3 = f(y_n - h * k1 + 2 * h * k2, t_n + h, c);
    return y_n + h / 6 * (k1 + 4 * k2 + k3);
}

/// @brief Runge-Kutta Third Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] y_n State vector at time t_n
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y>
Y RungeKutta3(const auto& f, const std::floating_point auto& h, const Y& y_n, const auto& c)
{
    Y k1 = f(y_n, c);
    Y k2 = f(y_n + h / 2 * k1, c);
    Y k3 = f(y_n - h * k1 + 2 * h * k2, c);
    return y_n + h / 6 * (k1 + 4 * k2 + k3);
}

/// @brief Runge-Kutta Fourth Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] y_n State vector at time t_n
/// @param[in] t_n Time to evaluate the model function at in [s]
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y>
Y RungeKutta4(const auto& f, const std::floating_point auto& h, const Y& y_n, const std::floating_point auto& t_n, const auto& c)
{
    Y k1 = f(y_n, t_n, c);
    Y k2 = f(y_n + h / 2 * k1, t_n + h / 2, c);
    Y k3 = f(y_n + h / 2 * k2, t_n + h / 2, c);
    Y k4 = f(y_n + h * k3, t_n + h, c);
    return y_n + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
}

/// @brief Runge-Kutta Fourth Order Algorithm
/// @param[in] f Model function
/// @param[in] h Integration step in [s]
/// @param[in] y_n State vector at time t_n
/// @param[in] c Vector with constant information needed to calculate the model function
/// @return State vector at time t_(n+1)
template<typename Y>
Y RungeKutta4(const auto& f, const std::floating_point auto& h, const Y& y_n, const auto& c)
{
    Y k1 = f(y_n, c);
    Y k2 = f(y_n + h / 2 * k1, c);
    Y k3 = f(y_n + h / 2 * k2, c);
    Y k4 = f(y_n + h * k3, c);
    return y_n + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
}

/// @brief Converts the enum to a string
/// @param[in] algorithm Enum value to convert into text
/// @return String representation of the enum
const char* to_string(IntegrationAlgorithm algorithm);

} // namespace NAV
