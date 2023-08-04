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

    auto codes = Code::GetAll();
    const uint16_t MAX_SATELLITES = 200;

    std::unordered_map<size_t, SatSigId> unique;

    unique[std::hash<States::KFStates>()(States::PosX)] = SatSigId{ Code::None, 201 };
    unique[std::hash<States::KFStates>()(States::PosY)] = SatSigId{ Code::None, 202 };
    unique[std::hash<States::KFStates>()(States::PosZ)] = SatSigId{ Code::None, 203 };
    unique[std::hash<States::KFStates>()(States::VelX)] = SatSigId{ Code::None, 204 };
    unique[std::hash<States::KFStates>()(States::VelY)] = SatSigId{ Code::None, 205 };
    unique[std::hash<States::KFStates>()(States::VelZ)] = SatSigId{ Code::None, 206 };

    for (const auto& code : codes)
    {
        for (uint16_t satNum = 1; satNum <= MAX_SATELLITES; satNum++)
        {
            auto satSigId = SatSigId{ code, satNum };
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
    REQUIRE(unique.size() == 6 + codes.size() * MAX_SATELLITES);

    REQUIRE(unique.at(std::hash<States::KFStates>()(States::PosX)) == SatSigId{ Code::None, 201 });
    REQUIRE(unique.at(std::hash<States::KFStates>()(States::PosY)) == SatSigId{ Code::None, 202 });
    REQUIRE(unique.at(std::hash<States::KFStates>()(States::PosZ)) == SatSigId{ Code::None, 203 });
    REQUIRE(unique.at(std::hash<States::KFStates>()(States::VelX)) == SatSigId{ Code::None, 204 });
    REQUIRE(unique.at(std::hash<States::KFStates>()(States::VelY)) == SatSigId{ Code::None, 205 });
    REQUIRE(unique.at(std::hash<States::KFStates>()(States::VelZ)) == SatSigId{ Code::None, 206 });

    for (const auto& code : codes)
    {
        for (uint16_t satNum = 1; satNum <= MAX_SATELLITES; satNum++)
        {
            auto satSigId = SatSigId{ code, satNum };
            auto hash = std::hash<States::AmbiguitySD>()(States::AmbiguitySD{ satSigId });
            REQUIRE(unique.at(hash) == satSigId);
        }
    }
}

TEST_CASE("[RealTimeKinematic] Meas keys unique", "[RealTimeKinematic]")
{
    auto logger = initializeTestLogger();

    namespace Meas = NAV::RealTimeKinematicKF::Meas;

    auto codes = Code::GetAll();
    const uint16_t MAX_SATELLITES = 200;

    std::unordered_map<size_t, SatSigId> unique;

    for (const auto& code : codes)
    {
        for (uint16_t satNum = 1; satNum <= MAX_SATELLITES; satNum++)
        {
            auto satSigId = SatSigId{ code, satNum };
            auto hash = std::hash<Meas::PsrDD>()(Meas::PsrDD{ satSigId });
            if (unique.contains(hash))
            {
                LOG_ERROR("PsrDD Hash ({}) for SatSigId {} was already in the map but for SatSigId {}",
                          std::bitset<64>(hash).to_string(),
                          satSigId,
                          unique.at(hash));
            }
            unique[hash] = satSigId;

            satSigId = SatSigId{ code, satNum };
            hash = std::hash<Meas::CarrierDD>()(Meas::CarrierDD{ satSigId });
            if (unique.contains(hash))
            {
                LOG_ERROR("CarrierDD Hash ({}) for SatSigId {} was already in the map but for SatSigId {}",
                          std::bitset<64>(hash).to_string(),
                          satSigId,
                          unique.at(hash));
            }
            unique[hash] = satSigId;

            satSigId = SatSigId{ code, satNum };
            hash = std::hash<Meas::DopplerDD>()(Meas::DopplerDD{ satSigId });
            if (unique.contains(hash))
            {
                LOG_ERROR("DopplerDD Hash ({}) for SatSigId {} was already in the map but for SatSigId {}",
                          std::bitset<64>(hash).to_string(),
                          satSigId,
                          unique.at(hash));
            }
            unique[hash] = satSigId;
        }
    }
    REQUIRE(unique.size() == 3 * codes.size() * MAX_SATELLITES);

    for (const auto& code : codes)
    {
        for (uint16_t satNum = 1; satNum <= MAX_SATELLITES; satNum++)
        {
            auto satSigId = SatSigId{ code, satNum };
            auto hash = std::hash<Meas::PsrDD>()(Meas::PsrDD{ satSigId });
            REQUIRE(unique.at(hash) == satSigId);

            satSigId = SatSigId{ code, satNum };
            hash = std::hash<Meas::CarrierDD>()(Meas::CarrierDD{ satSigId });
            REQUIRE(unique.at(hash) == satSigId);

            satSigId = SatSigId{ code, satNum };
            hash = std::hash<Meas::DopplerDD>()(Meas::DopplerDD{ satSigId });
            REQUIRE(unique.at(hash) == satSigId);
        }
    }
}

} // namespace NAV::TESTS::RealTimeKinematicTests