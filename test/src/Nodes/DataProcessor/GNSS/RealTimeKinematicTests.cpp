// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RealTimeKinematicTests.cpp
/// @brief Tests for the RealTimeKinematic node
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-07-17

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"
#include "Logger.hpp"

#include <unordered_map>

#include "Nodes/DataProcessor/GNSS/RealTimeKinematic.hpp"

namespace NAV::TESTS::RealTimeKinematicTests
{

TEST_CASE("[RealTimeKinematic] State keys unique", "[RealTimeKinematic]")
{
    auto logger = initializeTestLogger();

    namespace States = NAV::RealTimeKinematicKF::States;

    auto frequencies = Frequency::GetAll();
    const uint16_t MAX_SATELLITES = 200;

    std::unordered_map<size_t, SatSigId> unique;

    unique[std::hash<States::KFStates>()(States::PosX)] = SatSigId{ Freq_None, 201 };
    unique[std::hash<States::KFStates>()(States::PosY)] = SatSigId{ Freq_None, 202 };
    unique[std::hash<States::KFStates>()(States::PosZ)] = SatSigId{ Freq_None, 203 };
    unique[std::hash<States::KFStates>()(States::VelX)] = SatSigId{ Freq_None, 204 };
    unique[std::hash<States::KFStates>()(States::VelY)] = SatSigId{ Freq_None, 205 };
    unique[std::hash<States::KFStates>()(States::VelZ)] = SatSigId{ Freq_None, 206 };

    for (const auto& freq : frequencies)
    {
        for (uint16_t satNum = 1; satNum <= MAX_SATELLITES; satNum++)
        {
            auto satSigId = SatSigId{ freq, satNum };
            auto hash = std::hash<States::AmbiguitySD>()(States::AmbiguitySD{ satSigId });
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
    REQUIRE(unique.size() == 6 + frequencies.size() * MAX_SATELLITES);

    REQUIRE(unique.at(std::hash<States::KFStates>()(States::PosX)) == SatSigId{ Freq_None, 201 });
    REQUIRE(unique.at(std::hash<States::KFStates>()(States::PosY)) == SatSigId{ Freq_None, 202 });
    REQUIRE(unique.at(std::hash<States::KFStates>()(States::PosZ)) == SatSigId{ Freq_None, 203 });
    REQUIRE(unique.at(std::hash<States::KFStates>()(States::VelX)) == SatSigId{ Freq_None, 204 });
    REQUIRE(unique.at(std::hash<States::KFStates>()(States::VelY)) == SatSigId{ Freq_None, 205 });
    REQUIRE(unique.at(std::hash<States::KFStates>()(States::VelZ)) == SatSigId{ Freq_None, 206 });

    for (const auto& freq : frequencies)
    {
        for (uint16_t satNum = 1; satNum <= MAX_SATELLITES; satNum++)
        {
            auto satSigId = SatSigId{ freq, satNum };
            auto hash = std::hash<States::AmbiguitySD>()(States::AmbiguitySD{ satSigId });
            REQUIRE(unique.at(hash) == satSigId);
        }
    }
}

TEST_CASE("[RealTimeKinematic] Meas keys unique", "[RealTimeKinematic]")
{
    auto logger = initializeTestLogger();

    namespace Meas = NAV::RealTimeKinematicKF::Meas;

    auto frequencies = Frequency::GetAll();
    const uint16_t MAX_SATELLITES = 200;

    std::unordered_map<size_t, SatSigId> unique;

    for (const auto& freq : frequencies)
    {
        for (uint16_t satNum = 1; satNum <= MAX_SATELLITES; satNum++)
        {
            auto satSigId = SatSigId{ freq, satNum };
            auto hash = std::hash<Meas::PsrDD>()(Meas::PsrDD{ satSigId });
            if (unique.contains(hash))
            {
                LOG_ERROR("PsrDD Hash ({}) for SatSigId {} was already in the map but for SatSigId {}",
                          std::bitset<64>(hash).to_string(),
                          satSigId,
                          unique.at(hash));
            }
            unique[hash] = satSigId;

            satSigId = SatSigId{ freq, static_cast<uint16_t>(satNum + 300) };
            hash = std::hash<Meas::CarrierDD>()(Meas::CarrierDD{ satSigId });
            if (unique.contains(hash))
            {
                LOG_ERROR("CarrierDD Hash ({}) for SatSigId {} was already in the map but for SatSigId {}",
                          std::bitset<64>(hash).to_string(),
                          satSigId,
                          unique.at(hash));
            }
            unique[hash] = satSigId;
        }
    }
    REQUIRE(unique.size() == 2 * frequencies.size() * MAX_SATELLITES);

    for (const auto& freq : frequencies)
    {
        for (uint16_t satNum = 1; satNum <= MAX_SATELLITES; satNum++)
        {
            auto satSigId = SatSigId{ freq, satNum };
            auto hash = std::hash<Meas::PsrDD>()(Meas::PsrDD{ satSigId });
            REQUIRE(unique.at(hash) == satSigId);

            satSigId = SatSigId{ freq, static_cast<uint16_t>(satNum + 300) };
            hash = std::hash<Meas::CarrierDD>()(Meas::CarrierDD{ satSigId });
            REQUIRE(unique.at(hash) == satSigId);
        }
    }
}

} // namespace NAV::TESTS::RealTimeKinematicTests