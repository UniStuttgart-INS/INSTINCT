// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GnssNavInfoComparisons.hpp
/// @brief Comparison operators to compare the GnssNavInfo type
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2023-01-11

#pragma once

// This is a small hack, which lets us change private/protected parameters
#pragma GCC diagnostic push
#if defined(__clang__)
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "NodeData/GNSS/GnssObs.hpp"
#undef protected
#undef private
#pragma GCC diagnostic pop

namespace NAV
{

inline bool operator==(const GnssObs::ObservationData::Pseudorange& lhs, const GnssObs::ObservationData::Pseudorange& rhs)
{
    REQUIRE(lhs.value == rhs.value);
    REQUIRE(lhs.SSI == rhs.SSI);
    return true;
}

inline bool operator==(const GnssObs::ObservationData::CarrierPhase& lhs, const GnssObs::ObservationData::CarrierPhase& rhs)
{
    REQUIRE(lhs.value == rhs.value);
    REQUIRE(lhs.SSI == rhs.SSI);
    REQUIRE(lhs.LLI == rhs.LLI);
    return true;
}

inline bool operator==(const GnssObs::ObservationData& lhs, const GnssObs::ObservationData& rhs)
{
    REQUIRE(lhs.satSigId == rhs.satSigId);
    REQUIRE(lhs.pseudorange == rhs.pseudorange);
    REQUIRE(lhs.carrierPhase == rhs.carrierPhase);
    REQUIRE(lhs.doppler == rhs.doppler);
    REQUIRE(lhs.CN0 == rhs.CN0);
    return true;
}

inline bool operator==(const GnssObs& lhs, const GnssObs& rhs)
{
    REQUIRE(lhs.data.size() == rhs.data.size());
    REQUIRE(lhs.insTime == rhs.insTime);
    REQUIRE(lhs.data == rhs.data);
    return true;
}

} // namespace NAV