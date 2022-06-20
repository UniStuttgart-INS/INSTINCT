#pragma once

#include <numbers>
#include <Eigen/Core>

namespace NAV
{

/// @brief Convert Degree to Radians
/// @param[in] deg Value to convert in [deg]
/// @return The converted value in [rad]
template<class T>
[[nodiscard]] constexpr auto deg2rad(const T& deg)
{
    return deg * std::numbers::pi_v<double> / 180.0;
}

/// @brief Convert Degree to Radians
/// @param[in] deg Value to convert in [deg]
/// @return The converted value in [rad]
template<>
[[nodiscard]] inline auto deg2rad(const Eigen::Vector3d& deg)
{
    Eigen::Vector3d ret = deg * std::numbers::pi_v<double> / 180.0;
    return ret;
}

/// @brief Convert Radians to Degree
/// @param[in] rad Value to convert in [rad]
/// @return The converted value in [deg]
template<class T>
[[nodiscard]] constexpr auto rad2deg(const T& rad)
{
    return rad * 180.0 / std::numbers::pi_v<double>;
}

/// @brief Convert Radians to Degree
/// @param[in] rad Value to convert in [rad]
/// @return The converted value in [deg]
template<>
[[nodiscard]] inline auto rad2deg(const Eigen::Vector3d& rad)
{
    Eigen::Vector3d ret = rad * 180.0 / std::numbers::pi_v<double>;
    return ret;
}

/// @brief Convert Semicircles to Radians
/// @param[in] semicircles Value to convert in [semicircles]
/// @return The converted value in [rad]
template<class T>
[[nodiscard]] constexpr auto semicircles2rad(const T& semicircles)
{
    return semicircles * std::numbers::pi_v<double>;
}

/// @brief Convert Radians to Semicircles
/// @param[in] rad Value to convert in [rad]
/// @return The converted value in [semicircles]
template<class T>
[[nodiscard]] constexpr auto rad2semicircles(const T& rad)
{
    return rad / std::numbers::pi_v<double>;
}

/// @brief Converts [degree] to [radian]
constexpr long double operator"" _deg(long double deg)
{
    return deg2rad(deg);
}

/// @brief Converts [milliarcseconds] to [radian]
constexpr long double operator"" _mas(long double mas)
{
    return mas * std::numbers::pi_v<long double> / 648000000.0L;
}

/// @brief Converts [milliarcseconds] to [radian]
constexpr long double operator"" _mas(unsigned long long mas) // NOLINT(google-runtime-int)
{
    return static_cast<long double>(mas) * std::numbers::pi_v<long double> / 648000000.0L;
}

/// @brief Parts per billion
constexpr long double operator"" _ppb(long double ppb)
{
    return ppb * 1e-9L;
}

/// @brief Parts per billion
constexpr long double operator"" _ppb(unsigned long long ppb) // NOLINT(google-runtime-int)
{
    return static_cast<long double>(ppb) * 1e-9L;
}

} // namespace NAV
