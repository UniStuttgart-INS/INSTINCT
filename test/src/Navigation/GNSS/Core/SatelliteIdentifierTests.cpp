// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SatelliteIdentifierTests.cpp
/// @brief SatelliteIdentifier Tests
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-07-17

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"
#include "Logger.hpp"

#include <unordered_map>
#include <bitset>
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"

namespace NAV::TESTS::RealTimeKinematicTests
{

TEST_CASE("[SatelliteIdentifier] SatId hash unique", "[SatelliteIdentifier]")
{
    auto logger = initializeTestLogger();

    // auto frequencies = Frequency::GetAll();
    std::vector satSystems = { SatSys_None, GPS, GAL, GLO, BDS, QZSS, IRNSS, SBAS };
    const uint16_t MAX_SATELLITES = 200;

    std::unordered_map<size_t, SatId> unique;

    for (const auto& satSys : satSystems)
    {
        for (uint16_t satNum = 1; satNum <= MAX_SATELLITES; satNum++)
        {
            auto satId = SatId{ satSys, satNum };
            auto hash = std::hash<SatId>()(satId);
            if (unique.contains(hash))
            {
                LOG_ERROR("Hash ({}) for SatId {} was already in the map but for SatId {}",
                          std::bitset<64>(hash).to_string(),
                          satId,
                          unique.at(hash));
            }
            unique[hash] = satId;
        }
    }
    REQUIRE(unique.size() == satSystems.size() * MAX_SATELLITES);

    for (const auto& satSys : satSystems)
    {
        for (uint16_t satNum = 1; satNum <= MAX_SATELLITES; satNum++)
        {
            auto satId = SatId{ satSys, satNum };
            auto hash = std::hash<SatId>()(satId);
            REQUIRE(unique.at(hash) == satId);
        }
    }
}

TEST_CASE("[SatelliteIdentifier] SatSigId hash unique", "[SatelliteIdentifier]")
{
    auto logger = initializeTestLogger();

    auto codes = Code::GetAll();
    const uint16_t MAX_SATELLITES = 200;

    std::unordered_map<size_t, SatSigId> unique;

    for (const auto& code : codes)
    {
        for (uint16_t satNum = 1; satNum <= MAX_SATELLITES; satNum++)
        {
            auto satSigId = SatSigId{ code, satNum };
            auto hash = std::hash<SatSigId>()(satSigId);
            if (unique.contains(hash))
            {
                LOG_ERROR("Hash ({}) for SatSigId {} was already in the map but for SatSigId {}",
                          std::bitset<64>(hash).to_string(),
                          satSigId,
                          unique.at(hash));
            }
            unique[hash] = satSigId;
        }
    }
    REQUIRE(unique.size() == codes.size() * MAX_SATELLITES);

    for (const auto& code : codes)
    {
        for (uint16_t satNum = 1; satNum <= MAX_SATELLITES; satNum++)
        {
            auto satSigId = SatSigId{ code, satNum };
            auto hash = std::hash<SatSigId>()(satSigId);
            REQUIRE(unique.at(hash) == satSigId);
        }
    }
}

} // namespace NAV::TESTS::RealTimeKinematicTests