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

    REQUIRE(lhs.sdXYZ == rhs.sdXYZ);
    REQUIRE(lhs.sdNED == rhs.sdNED);

    REQUIRE(lhs.sdxy == rhs.sdxy);
    REQUIRE(lhs.sdyz == rhs.sdyz);
    REQUIRE(lhs.sdzx == rhs.sdzx);
    REQUIRE(lhs.sdne == rhs.sdne);
    REQUIRE(lhs.sded == rhs.sded);
    REQUIRE(lhs.sddn == rhs.sddn);

    REQUIRE(lhs.age == rhs.age);
    REQUIRE(lhs.ratio == rhs.ratio);

    REQUIRE(lhs.sdvNED == rhs.sdvNED);
    REQUIRE(lhs.sdvXYZ == rhs.sdvXYZ);

    REQUIRE(lhs.sdvne == rhs.sdvne);
    REQUIRE(lhs.sdved == rhs.sdved);
    REQUIRE(lhs.sdvdn == rhs.sdvdn);

    REQUIRE(lhs.sdvxy == rhs.sdvxy);
    REQUIRE(lhs.sdvyz == rhs.sdvyz);
    REQUIRE(lhs.sdvzx == rhs.sdvzx);

    return true;
}

} // namespace NAV