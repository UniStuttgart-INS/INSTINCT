// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GnssObsComparisons.hpp
/// @brief Comparison operators to compare the GnssObs type
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-01-21

#pragma once

#include "CatchMatchers.hpp"

#include "NodeData/GNSS/GnssObs.hpp"

#include "Logger.hpp"

namespace NAV
{

inline bool operator==(const GnssObs::ObservationData::Pseudorange& lhs, const GnssObs::ObservationData::Pseudorange& rhs)
{
    LOG_DEBUG("      [{}] == [{}] | value", lhs.value, rhs.value);
    REQUIRE_THAT(lhs.value, Catch::Matchers::WithinAbs(rhs.value, 1e-3)); // RINEX files have 3 digits
    LOG_DEBUG("      [{}] == [{}] | SSI", lhs.SSI, rhs.SSI);
    REQUIRE(lhs.SSI == rhs.SSI);
    return true;
}

inline bool operator==(const GnssObs::ObservationData::CarrierPhase& lhs, const GnssObs::ObservationData::CarrierPhase& rhs)
{
    LOG_DEBUG("      [{}] == [{}] | value", lhs.value, rhs.value);
    REQUIRE_THAT(lhs.value, Catch::Matchers::WithinAbs(rhs.value, 1e-3)); // RINEX files have 3 digits
    LOG_DEBUG("      [{}] == [{}] | SSI", lhs.SSI, rhs.SSI);
    REQUIRE(lhs.SSI == rhs.SSI);
    LOG_DEBUG("      [{}] == [{}] | LLI", lhs.LLI, rhs.LLI);
    REQUIRE(lhs.LLI == rhs.LLI);
    return true;
}

inline bool operator==(const GnssObs::ObservationData& lhs, const GnssObs::ObservationData& rhs)
{
    LOG_DEBUG("    [{}] == [{}] | satId", lhs.satSigId, rhs.satSigId);
    REQUIRE(lhs.satSigId == rhs.satSigId);
    LOG_DEBUG("    pseudorange");
    REQUIRE(lhs.pseudorange == rhs.pseudorange);
    LOG_DEBUG("    carrierPhase");
    REQUIRE(lhs.carrierPhase == rhs.carrierPhase);
    LOG_DEBUG("    [{}] == [{}] | doppler", lhs.doppler, rhs.doppler);
    REQUIRE(lhs.doppler.has_value() == rhs.doppler.has_value());
    if (lhs.doppler.has_value())
    {
        REQUIRE_THAT(*lhs.doppler, Catch::Matchers::WithinAbs(*rhs.doppler, 1e-3)); // RINEX files have 3 digits
    }
    LOG_DEBUG("    [{}] == [{}] | CN0", lhs.CN0, rhs.CN0);
    REQUIRE(lhs.CN0 == rhs.CN0);
    return true;
}

inline bool operator==(const GnssObs::SatelliteData& lhs, const GnssObs::SatelliteData& rhs)
{
    LOG_DEBUG("    [{}] == [{}] | satId", lhs.satId, rhs.satId);
    REQUIRE(lhs.satId == rhs.satId);
    LOG_DEBUG("    [{}] == [{}] | frequencies", lhs.frequencies, rhs.frequencies);
    REQUIRE(lhs.frequencies == rhs.frequencies);
    return true;
}

inline bool operator==(const GnssObs& lhs, const GnssObs& rhs)
{
    LOG_DEBUG("[{}] == [{}] | insTime", lhs.insTime.toYMDHMS(GPST), rhs.insTime.toYMDHMS(GPST));
    REQUIRE_THAT(std::chrono::abs(lhs.insTime - rhs.insTime).count(), Catch::Matchers::WithinAbs(0.0, 1e-7)); // RINEX Format: F11.7
    LOG_DEBUG("  [{}] == [{}] | data.size()", lhs.data.size(), rhs.data.size());
    REQUIRE(lhs.data.size() == rhs.data.size());
    for (const auto& l : lhs.data)
    {
        LOG_DEBUG("  [{}] | satId", l.satSigId);
        auto r = std::find_if(rhs.data.begin(), rhs.data.end(), [&](const auto& r) { return l.satSigId == r.satSigId; });
        REQUIRE(r != rhs.data.end());
        REQUIRE(*r == l);
    }
    LOG_DEBUG("  [{}] == [{}] | getSatData().size()", lhs.getSatData().size(), rhs.getSatData().size());
    REQUIRE(lhs.getSatData().size() == rhs.getSatData().size());
    for (const auto& l : lhs.getSatData())
    {
        auto r = std::find_if(rhs.getSatData().begin(), rhs.getSatData().end(), [&](const auto& r) { return l.satId == r.satId; });
        REQUIRE(r != rhs.getSatData().end());
        REQUIRE(*r == l);
    }
    return true;
}

} // namespace NAV