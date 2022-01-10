/// @file angular_velocity.hpp
/// @brief Angular velocity related units
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-01-10

#pragma once

#include <units/isq/si/angular_velocity.h>

namespace units::isq::si
{

/// @brief Milliradian per second [mrad / s]
// struct milliradian_per_second : prefixed_unit<milliradian_per_second, milli, radian_per_second>
// {};
struct milliradian_per_second : named_scaled_unit<milliradian_per_second, basic_symbol_text{ "mω", "mw" }, no_prefix,
                                                  ratio(1, 1, -3), radian_per_second>
{};

#ifndef UNITS_NO_LITERALS

inline namespace literals
{

/// @brief Milliradian per second [mrad / s]
constexpr auto operator"" _q_mrad_per_s(unsigned long long l) // NOLINT(google-runtime-int)
{
    gsl_ExpectsAudit(std::in_range<std::int64_t>(l));
    return angular_velocity<milliradian_per_second, std::int64_t>(static_cast<std::int64_t>(l));
}
/// @brief Milliradian per second [mrad / s]
constexpr auto operator"" _q_mrad_per_s(long double l) { return angular_velocity<milliradian_per_second, long double>(l); }

} // namespace literals

#endif // UNITS_NO_LITERALS

#ifndef UNITS_NO_REFERENCES

namespace angular_velocity_references
{

/// @brief Radian per second [mrad / s]
inline constexpr auto rad_per_s = reference<dim_angular_velocity, radian_per_second>{};
/// @brief Milliradian per second [mrad / s]
inline constexpr auto mrad_per_s = reference<dim_angular_velocity, milliradian_per_second>{};

} // namespace angular_velocity_references

namespace references
{

using namespace angular_velocity_references; // NOLINT(google-build-using-namespace)

} // namespace references

#endif // UNITS_NO_REFERENCES

} // namespace units::isq::si

#ifndef UNITS_NO_ALIASES

namespace units::aliases::isq::si::inline acceleration
{

/// @brief Milliradian per second [mrad / s]
template<units::Representation Rep = double>
using mrad_per_s = units::isq::si::angular_velocity<units::isq::si::milliradian_per_second, Rep>;

// clang-format off
} // namespace units::aliases::isq::si::acceleration
// clang-format on

#endif // UNITS_NO_ALIASES