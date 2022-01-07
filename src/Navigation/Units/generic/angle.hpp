/// @file Units.hpp
/// @brief Angle related units
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-01-07

#pragma once

#include <units/generic/angle.h>
#include <cmath>

namespace units
{

/* Currently only rational numbers are supported for scale factors.
 * This would lead to loss of precision for scale factors like Pi.
 * See https://github.com/mpusz/units/discussions/195
 * TODO: Check if the library implements this at some point

/// @brief Degree [°]
struct degree : named_scaled_unit<degree, "deg", units::no_prefix, ratio(static_cast<intmax_t>(M_PI * 1e17), 1e17, 0), units::radian>
{};
*/

/// @brief Pseudometre [m]
struct pseudometre : named_scaled_unit<pseudometre, "m", units::isq::si::prefix, ratio(6378134), units::radian>
{};

#ifndef UNITS_NO_LITERALS

inline namespace literals
{
/// @brief Degree [°]
constexpr auto operator"" _q_deg(unsigned long long l) // NOLINT(google-runtime-int)
{
    gsl_ExpectsAudit(std::in_range<int64_t>(l));
    return angle<degree, int64_t>(static_cast<int64_t>(l));
}
/// @brief Degree [°]
constexpr auto operator"" _q_g(long double l) { return angle<degree, long double>(l); }

} // namespace literals

#endif // UNITS_NO_LITERALS

#ifndef UNITS_NO_REFERENCES

namespace angle_references
{

/* inline constexpr auto deg = reference<units::dim_angle<>, units::degree>{}; */

} // namespace angle_references

namespace references
{

/* using namespace angle_references; // NOLINT(google-build-using-namespace) */

} // namespace references

#endif // UNITS_NO_REFERENCES

} // namespace units

#ifndef UNITS_NO_ALIASES

namespace units::aliases::isq::si::inline angle
{

template<units::Representation Rep = double>
using rad = units::angle<units::radian, Rep>;
/* template<units::Representation Rep = double>
using deg = units::angle<units::degree, Rep>; */
template<units::Representation Rep = double>
using m = units::angle<units::pseudometre, Rep>;

} // namespace units::aliases::isq::si::inline angle

#endif // UNITS_NO_ALIASES