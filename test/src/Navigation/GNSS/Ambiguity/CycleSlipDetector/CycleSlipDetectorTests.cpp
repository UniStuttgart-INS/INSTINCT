// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file CycleSlipDetectorTests.cpp
/// @brief CycleSlipDetector Tests
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-07-17

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"
#include "Logger.hpp"

#include <unordered_map>
#include <bitset>
#include "Navigation/GNSS/Ambiguity/CycleSlipDetector.hpp"

namespace NAV::TESTS::CycleSlipDetectorTests
{

TEST_CASE("[CycleSlipDetector] DualFrequencyCombination hash unique", "[CycleSlipDetector]")
{
    auto logger = initializeTestLogger();

    auto codes = Code::GetAll();
    const uint16_t MAX_SATELLITES = 200;

    std::unordered_map<size_t, CycleSlipDetector::DualFrequencyCombination> unique;

    size_t count = 0;
    for (const Code& code1 : codes)
    {
        for (const Code& code2 : codes)
        {
            if (code1.getFrequency().getSatSys() != code2.getFrequency().getSatSys()
                || code1.getFrequency() == code2.getFrequency()) { continue; }

            for (uint16_t satNum = 1; satNum <= MAX_SATELLITES; satNum++)
            {
                count++;
                auto freqComb = CycleSlipDetector::DualFrequencyCombination{ .satId = SatId(code1.getFrequency().getSatSys(), satNum), .sig1 = code1, .sig2 = code2 };
                size_t hash = std::hash<CycleSlipDetector::DualFrequencyCombination>()(freqComb);

                if (unique.contains(hash))
                {
                    [[maybe_unused]] auto other = unique.at(hash);
                    LOG_ERROR("Hash ({}) for DualFrequencyCombination [{}, {}, {}] was already in the map but for [{}, {}, {}]",
                              std::bitset<64>(hash).to_string(),
                              freqComb.satId, freqComb.sig1, freqComb.sig2,
                              other.satId, other.sig1, other.sig2);
                }
                unique[hash] = freqComb;
            }
        }
    }
    REQUIRE(unique.size() == count);

    // Test lookup
    for (const Code& code1 : codes)
    {
        for (const Code& code2 : codes)
        {
            if (code1.getFrequency().getSatSys() != code2.getFrequency().getSatSys()
                || code1.getFrequency() == code2.getFrequency()) { continue; }

            for (uint16_t satNum = 1; satNum <= MAX_SATELLITES; satNum++)
            {
                auto freqComb = CycleSlipDetector::DualFrequencyCombination{ .satId = SatId(code1.getFrequency().getSatSys(), satNum), .sig1 = code1, .sig2 = code2 };
                size_t hash = std::hash<CycleSlipDetector::DualFrequencyCombination>()(freqComb);

                REQUIRE(unique.contains(hash));
                auto other = unique.at(hash);
                REQUIRE(other.satId == freqComb.satId);
                REQUIRE(other.sig1 == freqComb.sig1);
                REQUIRE(other.sig2 == freqComb.sig2);
            }
        }
    }
}

} // namespace NAV::TESTS::CycleSlipDetectorTests