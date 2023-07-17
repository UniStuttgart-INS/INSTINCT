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
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"

namespace NAV::TESTS::RealTimeKinematicTests
{

TEST_CASE("[SatelliteIdentifier] SatSigId hash unique", "[SatelliteIdentifier]")
{
    auto logger = initializeTestLogger();

    auto frequencies = Frequency::GetAll();
    const uint16_t MAX_SATELLITES = 200;

    std::unordered_map<size_t, size_t> unique;
    size_t iter = 0;

    for (const auto& freq : frequencies)
    {
        for (uint16_t satNum = 0; satNum < MAX_SATELLITES; satNum++)
        {
            unique[std::hash<SatSigId>()(SatSigId{ freq, satNum })] = iter++;
        }
    }
    REQUIRE(unique.size() == frequencies.size() * MAX_SATELLITES);

    iter = 0;
    for (const auto& freq : frequencies)
    {
        for (uint16_t satNum = 0; satNum < MAX_SATELLITES; satNum++)
        {
            REQUIRE(unique.at(std::hash<SatSigId>()(SatSigId{ freq, satNum })) == iter++);
        }
    }
}

} // namespace NAV::TESTS::RealTimeKinematicTests