// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RtklibPosObsComparisons.hpp
/// @brief Comparison operators to compare the RtklibPosObs type
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author N. Stahl (HiWi)
/// @date 2023-04-26

#pragma once

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"

// This is a small hack, which lets us change private/protected parameters
#pragma GCC diagnostic push
#if defined(__clang__)
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "NodeData/GNSS/RtklibPosObs.hpp"
#include <cmath>
#undef protected
#undef private
#pragma GCC diagnostic pop

namespace NAV
{

inline bool operator==(const RtklibPosObs& lhs, const RtklibPosObs& rhs)
{
    REQUIRE(lhs.insTime == rhs.insTime);
    REQUIRE(lhs._e_position == rhs._e_position);
    REQUIRE(lhs._lla_position == rhs._lla_position);

    if (std::isnan(lhs._e_velocity[0])) { REQUIRE(std::isnan(rhs._e_velocity[0])); }
    else { REQUIRE(lhs._e_velocity == rhs._e_velocity); };

    if (std::isnan(lhs._n_velocity[0])) { REQUIRE(std::isnan(rhs._n_velocity[0])); }
    else { REQUIRE(lhs._n_velocity == rhs._n_velocity); };

    REQUIRE(lhs.Q == rhs.Q);
    REQUIRE(lhs.ns == rhs.ns);
    REQUIRE(lhs.age == rhs.age);
    REQUIRE(lhs.ratio == rhs.ratio);

    REQUIRE_THAT(lhs.e_position(), Catch::Matchers::WithinAbs(rhs.e_position(), 1e-10));
    REQUIRE_THAT(lhs.lla_position(), Catch::Matchers::WithinAbs(rhs.lla_position(), 1e-10));
    REQUIRE_THAT(lhs.e_velocity(), Catch::Matchers::WithinAbs(rhs.e_velocity(), 1e-10));
    REQUIRE_THAT(lhs.n_velocity(), Catch::Matchers::WithinAbs(rhs.n_velocity(), 1e-10));

    REQUIRE(lhs.e_positionStdev().has_value() == rhs.e_positionStdev().has_value());
    REQUIRE(lhs.n_positionStdev().has_value() == rhs.n_positionStdev().has_value());
    REQUIRE(lhs.e_velocityStdev().has_value() == rhs.e_velocityStdev().has_value());
    REQUIRE(lhs.n_velocityStdev().has_value() == rhs.n_velocityStdev().has_value());

    if (lhs.e_positionStdev()) { REQUIRE_THAT(lhs.e_positionStdev()->get(), Catch::Matchers::WithinAbs(rhs.e_positionStdev()->get(), 1e-10)); }
    if (lhs.n_positionStdev()) { REQUIRE_THAT(lhs.n_positionStdev()->get(), Catch::Matchers::WithinAbs(rhs.n_positionStdev()->get(), 1e-10)); }
    if (lhs.e_velocityStdev()) { REQUIRE_THAT(lhs.e_velocityStdev()->get(), Catch::Matchers::WithinAbs(rhs.e_velocityStdev()->get(), 1e-10)); }
    if (lhs.n_velocityStdev()) { REQUIRE_THAT(lhs.n_velocityStdev()->get(), Catch::Matchers::WithinAbs(rhs.n_velocityStdev()->get(), 1e-10)); }

    return true;
}

} // namespace NAV