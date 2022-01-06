/// @file Units.hpp
/// @brief Includes all user defined units
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-01-04

#pragma once

// #########################################################################################################################################
//                                                                  Usage
// #########################################################################################################################################

/* Unit-Specific Aliases (see https://mpusz.github.io/units/framework/quantities.html#unit-specific-aliases-experimental)
 *
 * using namespace units::isq::si::references;
 * auto d = 123. * km;       // si::length<si::kilometre, double>
 * auto v = 70 * (km / h);   // si::speed<si::kilometre_per_hour, int>
 */
// #define UNITS_NO_ALIASES

/* Quantity References (see https://mpusz.github.io/units/framework/quantities.html#quantity-references-experimental)
 *
 * using namespace units::aliases::isq;
 * si::length::km d(123.);           // si::length<si::kilometre, double>
 * si::speed::km_per_h<int> v(70);   // si::speed<si::kilometre_per_hour, int>
 */
// #define UNITS_NO_REFERENCES

/* User Defined Literals (see https://mpusz.github.io/units/framework/quantities.html#user-defined-literals-experimental)
 *
 * using namespace units::isq::si::literals;
 * auto d = 123._q_km;     // si::length<si::kilometre, long double>
 * auto v = 70_q_km_per_h; // si::speed<si::kilometre_per_hour, std::int64_t>
 */
// #define UNITS_NO_LITERALS

// #########################################################################################################################################
//                                                         Custom Unit Definition
// #########################################################################################################################################

/* Scaled Units (see https://mpusz.github.io/units/framework/units.html#scaled-units)
 *
 * struct minute : named_scaled_unit<minute, "min", no_prefix, ratio(60), second> {};
 *
 * struct electronvolt : named_scaled_unit<electronvolt, "eV", prefix, ratio(1'602'176'634, 1'000'000'000, -19), joule> {};
 * struct millielectronvolt : prefixed_unit<millielectronvolt, milli, electronvolt> {};
 */

/* Derived Units (see https://mpusz.github.io/units/framework/units.html#derived-units)
 *
 * struct dim_momentum : derived_dimension<dim_momentum, kilogram_metre_per_second,
 *                                         exponent<si::dim_mass, 1>,
 *                                         exponent<si::dim_length, 1>,
 *                                         exponent<si::dim_time, -1>> {};    // kg ⋅ m/s
 *
 * struct dim_momentum : derived_dimension<dim_momentum, kilogram_metre_per_second,
 *                                         exponent<si::dim_mass, 1>,
 *                                         exponent<si::dim_speed, 1>> {}; // kg ⋅ m/s
 */

/* Aliased Units (see https://mpusz.github.io/units/framework/units.html#aliased-units)
 *
 * struct litre : alias_unit<cubic_decimetre, "l", prefix> {};
 * struct millilitre : prefixed_alias_unit<cubic_centimetre, milli, litre> {};
 */

#include <units/isq/si/frequency.h>
#include <units/math.h>

#include "internal/StandardGravity.hpp"