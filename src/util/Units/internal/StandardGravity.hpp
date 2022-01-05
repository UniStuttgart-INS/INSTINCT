/// @file Units.hpp
/// @brief Standard gravity related units [g]
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-01-04

#pragma once

#include <units/isq/si/acceleration.h>

#include <Navigation/Constants.hpp>

namespace units::isq::si
{

/// @brief Standard gravity [g]
struct standardgravity : named_scaled_unit<standardgravity, "g", prefix, ratio(NAV::InsConst::G_NORM * 1e5, 1e5, 0), metre_per_second_sq>
{};
/// @brief Milli standard gravity [mg]
struct millistandardgravity : prefixed_unit<millistandardgravity, milli, standardgravity>
{};

#ifndef UNITS_NO_LITERALS

inline namespace literals
{

// standard gravity [g]
/// @brief
constexpr auto operator"" _q_g(unsigned long long l)
{
    gsl_ExpectsAudit(std::in_range<std::int64_t>(l));
    return acceleration<standardgravity, std::int64_t>(static_cast<std::int64_t>(l));
}
constexpr auto operator"" _q_g(long double l) { return acceleration<standardgravity, long double>(l); }

// milli standard gravity [mg]
constexpr auto operator"" _q_mg(unsigned long long l)
{
    gsl_ExpectsAudit(std::in_range<std::int64_t>(l));
    return acceleration<millistandardgravity, std::int64_t>(static_cast<std::int64_t>(l));
}
constexpr auto operator"" _q_mg(long double l) { return acceleration<millistandardgravity, long double>(l); }

} // namespace literals

#endif // UNITS_NO_LITERALS

#ifndef UNITS_NO_REFERENCES

namespace acceleration_references
{

inline constexpr auto g = reference<dim_acceleration, standardgravity>{};
inline constexpr auto mg = reference<dim_acceleration, millistandardgravity>{};

} // namespace acceleration_references

namespace references
{

using namespace acceleration_references;

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

} // namespace units::aliases::isq::si::inline acceleration

#endif // UNITS_NO_ALIASES