/// @file StandardGravity.hpp
/// @brief Standard gravity related units [g]
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-01-04

#pragma once

#include <units/isq/si/acceleration.h>
#include <units/isq/si/constants.h>

namespace units::isq::si
{

/// @brief Standard gravity [g]
struct standardgravity : named_scaled_unit<standardgravity, "g", prefix, ratio(static_cast<intmax_t>(si2019::standard_gravity<double>.number() * 1e5), 1e5, 0), metre_per_second_sq>
{};
/// @brief Milli standard gravity [mg]
struct millistandardgravity : prefixed_unit<millistandardgravity, milli, standardgravity>
{};

#ifndef UNITS_NO_LITERALS

inline namespace literals
{
/* Do not work as name conflict with 'gram'

/// @brief Standard gravity [g]
constexpr auto operator"" _q_g(unsigned long long l) // NOLINT(google-runtime-int)
{
    gsl_ExpectsAudit(std::in_range<int64_t>(l));
    return acceleration<standardgravity, int64_t>(static_cast<int64_t>(l));
}
/// @brief Standard gravity [g]
constexpr auto operator"" _q_g(long double l) { return acceleration<standardgravity, long double>(l); }

/// @brief Milli standard gravity [mg]
constexpr auto operator"" _q_mg(unsigned long long l) // NOLINT(google-runtime-int)
{
    gsl_ExpectsAudit(std::in_range<int64_t>(l));
    return acceleration<millistandardgravity, int64_t>(static_cast<int64_t>(l));
}
/// @brief Milli standard gravity [mg]
constexpr auto operator"" _q_mg(long double l) { return acceleration<millistandardgravity, long double>(l); }

*/
} // namespace literals

#endif // UNITS_NO_LITERALS

#ifndef UNITS_NO_REFERENCES

namespace acceleration_references
{
/* Do not work as name conflict with 'gram'

inline constexpr auto g = reference<dim_acceleration, standardgravity>{};
inline constexpr auto mg = reference<dim_acceleration, millistandardgravity>{};

*/
} // namespace acceleration_references

namespace references
{

using namespace acceleration_references; // NOLINT(google-build-using-namespace)

} // namespace references

#endif // UNITS_NO_REFERENCES

} // namespace units::isq::si

#ifndef UNITS_NO_ALIASES

namespace units::aliases::isq::si::inline acceleration
{

template<units::Representation Rep = double>
using g = units::isq::si::acceleration<units::isq::si::standardgravity, Rep>;
template<units::Representation Rep = double>
using mg = units::isq::si::acceleration<units::isq::si::millistandardgravity, Rep>;

// clang-format off
} // namespace units::aliases::isq::si::acceleration
// clang-format on

#endif // UNITS_NO_ALIASES