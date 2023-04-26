// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RtklibPosObsComparisons.hpp
/// @brief
/// @author
/// @author
/// @date

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

    // if (std::isnan(lhs._e_velocity[0]) == true) { REQUIRE(std::isnan(rhs._e_velocity[0])); }
    // else { REQUIRE(lhs._e_velocity == rhs._e_velocity); };

    if (std::isnan(lhs._n_velocity[0]) == true) { REQUIRE(std::isnan(rhs._n_velocity[0])); }
    else { REQUIRE(lhs._n_velocity == rhs._n_velocity); };

    REQUIRE(lhs.Q == rhs.Q);
    REQUIRE(lhs.ns == rhs.ns);

    if (std::isnan(lhs.sdXYZ[0]) == true) { REQUIRE(std::isnan(rhs.sdXYZ[0])); }
    else { REQUIRE(lhs.sdXYZ == rhs.sdXYZ); };

    REQUIRE(lhs.sdNED == rhs.sdNED);

    if (std::isnan(lhs.sdxy)) { REQUIRE(std::isnan(rhs.sdxy)); }
    else { REQUIRE(lhs.sdxy == rhs.sdxy); };

    if (std::isnan(lhs.sdyz)) { REQUIRE(std::isnan(rhs.sdyz)); }
    else { REQUIRE(lhs.sdyz == rhs.sdyz); };

    if (std::isnan(lhs.sdzx)) { REQUIRE(std::isnan(rhs.sdzx)); }
    else { REQUIRE(lhs.sdzx == rhs.sdzx); };

    if (std::isnan(lhs.sdne)) { REQUIRE(std::isnan(rhs.sdne)); }
    else { REQUIRE(lhs.sdne == rhs.sdne); };

    if (std::isnan(lhs.sded)) { REQUIRE(std::isnan(rhs.sded)); }
    else { REQUIRE(lhs.sded == rhs.sded); };

    if (std::isnan(lhs.sddn)) { REQUIRE(std::isnan(rhs.sddn)); }
    else { REQUIRE(lhs.sddn == rhs.sddn); };

    REQUIRE(lhs.age == rhs.age);
    REQUIRE(lhs.ratio == rhs.ratio);
    REQUIRE(lhs.ratio == rhs.ratio);

    if (std::isnan(lhs.sdvNED[0])) { REQUIRE(std::isnan(rhs.sdvNED[0])); }
    else { REQUIRE(lhs.sdvNED == rhs.sdvNED); };

    if (std::isnan(lhs.sdvne)) { REQUIRE(std::isnan(rhs.sdvne)); }
    else { REQUIRE(lhs.sdvne == rhs.sdvne); };

    if (std::isnan(lhs.sdved)) { REQUIRE(std::isnan(rhs.sdved)); }
    else { REQUIRE(lhs.sdved == rhs.sdved); };

    if (std::isnan(lhs.sdvdn)) { REQUIRE(std::isnan(rhs.sdvdn)); }
    else { REQUIRE(lhs.sdvdn == rhs.sdvdn); };
    return true;
}

} // namespace NAV