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

    std::unordered_map<size_t, size_t> unique;
    size_t iter = 0;

    unique[std::hash<States::KFStates>()(States::PosX)] = iter++;
    unique[std::hash<States::KFStates>()(States::PosY)] = iter++;
    unique[std::hash<States::KFStates>()(States::PosZ)] = iter++;
    unique[std::hash<States::KFStates>()(States::VelX)] = iter++;
    unique[std::hash<States::KFStates>()(States::VelY)] = iter++;
    unique[std::hash<States::KFStates>()(States::VelZ)] = iter++;

    for (const auto& freq : frequencies)
    {
        for (uint16_t satNum = 0; satNum < MAX_SATELLITES; satNum++)
        {
            unique[std::hash<States::AmbiguitySD>()(States::AmbiguitySD{ SatSigId{ freq, satNum } })] = iter++;
        }
    }
    REQUIRE(unique.size() == 6 + frequencies.size() * MAX_SATELLITES);

    iter = 0;
    REQUIRE(unique.at(std::hash<States::KFStates>()(States::PosX)) == iter++);
    REQUIRE(unique.at(std::hash<States::KFStates>()(States::PosY)) == iter++);
    REQUIRE(unique.at(std::hash<States::KFStates>()(States::PosZ)) == iter++);
    REQUIRE(unique.at(std::hash<States::KFStates>()(States::VelX)) == iter++);
    REQUIRE(unique.at(std::hash<States::KFStates>()(States::VelY)) == iter++);
    REQUIRE(unique.at(std::hash<States::KFStates>()(States::VelZ)) == iter++);

    for (const auto& freq : frequencies)
    {
        for (uint16_t satNum = 0; satNum < MAX_SATELLITES; satNum++)
        {
            REQUIRE(unique.at(std::hash<States::AmbiguitySD>()(States::AmbiguitySD{ SatSigId{ freq, satNum } })) == iter++);
        }
    }
}

TEST_CASE("[RealTimeKinematic] Meas keys unique", "[RealTimeKinematic]")
{
    auto logger = initializeTestLogger();

    namespace Meas = NAV::RealTimeKinematicKF::Meas;

    auto frequencies = Frequency::GetAll();
    const uint16_t MAX_SATELLITES = 200;

    std::unordered_map<size_t, size_t> unique;
    size_t iter = 0;

    for (const auto& freq : frequencies)
    {
        for (uint16_t satNum = 0; satNum < MAX_SATELLITES; satNum++)
        {
            unique[std::hash<Meas::PsrDD>()(Meas::PsrDD{ SatSigId{ freq, satNum } })] = iter++;
            unique[std::hash<Meas::CarrierDD>()(Meas::CarrierDD{ SatSigId{ freq, satNum } })] = iter++;
        }
    }
    REQUIRE(unique.size() == 2 * frequencies.size() * MAX_SATELLITES);

    iter = 0;

    for (const auto& freq : frequencies)
    {
        for (uint16_t satNum = 0; satNum < MAX_SATELLITES; satNum++)
        {
            REQUIRE(unique.at(std::hash<Meas::PsrDD>()(Meas::PsrDD{ SatSigId{ freq, satNum } })) == iter++);
            REQUIRE(unique.at(std::hash<Meas::CarrierDD>()(Meas::CarrierDD{ SatSigId{ freq, satNum } })) == iter++);
        }
    }
}

} // namespace NAV::TESTS::RealTimeKinematicTests