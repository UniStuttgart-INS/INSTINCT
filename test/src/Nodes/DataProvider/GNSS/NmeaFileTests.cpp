// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file NMEAFileTests.cpp
/// @brief Tests for the NmeaFile node
/// @author T. Hobiger (thomas.hobiger@ins.uni-stuttgart.de)
/// @date 2022-11-08

#include <catch2/catch_test_macros.hpp>
#include <CatchMatchers.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <limits>
#include <array>

#include "FlowTester.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "Logger.hpp"

#include "NodeData/State/PosVel.hpp"

namespace NAV::TESTS::NMEAFileTests
{

enum NmeaRef : size_t
{
    NMEA_Year,
    NMEA_Month,
    NMEA_Day,
    NMEA_Hour,
    NMEA_Minute,
    NMEA_Second,
    NMEA_Latitude_rad,
    NMEA_Longitude_rad,
    NMEA_Height,
};

constexpr std::array<std::array<double, 9>, 3> NMEA_REFERENCE_DATA = { {
    { 2022, 11, 5, 16, 16, 0.756, 0.916818838078743, 0.233908153687654, 0.0 },
    { 2022, 11, 5, 16, 16, 1.756, 0.916802548339058, 0.234095485694035, 0.0 },
    { 2022, 11, 3, 14, 05, 18.000, 0.851383885268465, 0.160074500200709, 327.812 },
} };

void compareNMEAData(const std::shared_ptr<const NAV::PosVel>& obs, size_t messageCounterNMEA)
{
    // ------------------------------------------------ InsTime --------------------------------------------------
    REQUIRE(!obs->insTime.empty());
    LOG_DEBUG(" Time = {}", obs->insTime);

    REQUIRE(obs->insTime.toYMDHMS().year == static_cast<int32_t>(NMEA_REFERENCE_DATA.at(messageCounterNMEA).at(NMEA_Year)));
    REQUIRE(obs->insTime.toYMDHMS().month == static_cast<int32_t>(NMEA_REFERENCE_DATA.at(messageCounterNMEA).at(NMEA_Month)));
    REQUIRE(obs->insTime.toYMDHMS().day == static_cast<int32_t>(NMEA_REFERENCE_DATA.at(messageCounterNMEA).at(NMEA_Day)));

    REQUIRE(obs->insTime.toYMDHMS().hour == static_cast<int32_t>(NMEA_REFERENCE_DATA.at(messageCounterNMEA).at(NMEA_Hour)));
    REQUIRE(obs->insTime.toYMDHMS().min == static_cast<int32_t>(NMEA_REFERENCE_DATA.at(messageCounterNMEA).at(NMEA_Minute)));
    LOG_DATA("{}", obs->insTime.toYMDHMS().sec - NMEA_REFERENCE_DATA.at(messageCounterNMEA).at(NMEA_Second));
    REQUIRE_THAT(obs->insTime.toYMDHMS().sec - NMEA_REFERENCE_DATA.at(messageCounterNMEA).at(NMEA_Second), Catch::Matchers::WithinAbs(0.0, 9e-12));

    REQUIRE_THAT(obs->latitude(), Catch::Matchers::WithinAbs(NMEA_REFERENCE_DATA.at(messageCounterNMEA).at(NMEA_Latitude_rad), EPSILON));
    REQUIRE_THAT(obs->longitude(), Catch::Matchers::WithinAbs(NMEA_REFERENCE_DATA.at(messageCounterNMEA).at(NMEA_Longitude_rad), EPSILON));
    REQUIRE_THAT(obs->altitude(), Catch::Matchers::WithinAbs(NMEA_REFERENCE_DATA.at(messageCounterNMEA).at(NMEA_Height), EPSILON));

    REQUIRE(std::isnan(obs->e_velocity()[0]));
    REQUIRE(std::isnan(obs->e_velocity()[1]));
    REQUIRE(std::isnan(obs->e_velocity()[2]));
}

TEST_CASE("[NMEAFile][flow] Read 'data/NMEA/test.nmea' and compare content with hardcoded values", "[NMEAFile][flow]")
{
    auto logger = initializeTestLogger();

    // ###########################################################################################################
    //                                                  NMEAFile.flow"
    // ###########################################################################################################
    //
    //  NmeaFile (95)                 Plot (101)
    //     (94) PosVel |>  --(102)->  |> Pin 1 (96)
    //
    // ###########################################################################################################

    size_t messageCounter = 0;
    nm::RegisterWatcherCallbackToInputPin(96, [&messageCounter](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        LOG_TRACE("messageCounter = {}", messageCounter);

        compareNMEAData(std::dynamic_pointer_cast<const NAV::PosVel>(queue.front()), messageCounter);

        messageCounter++;
    });

    REQUIRE(testFlow("test/flow/Nodes/DataProvider/GNSS/NMEAFile.flow"));

    REQUIRE(messageCounter == NMEA_REFERENCE_DATA.size());
}

} // namespace NAV::TESTS::NMEAFileTests
