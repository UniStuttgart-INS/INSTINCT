// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <catch2/catch.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <limits>

#include "FlowTester.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "util/Logger.hpp"

#include "NodeData/State/PosVel.hpp"

#include "NMEAFileTestsData.hpp"

namespace NAV::TEST::NMEAFileTests
{


constexpr double EPSILON = 10.0 * std::numeric_limits<double>::epsilon();

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
	REQUIRE(obs->insTime.toYMDHMS().sec ==  Approx(NMEA_REFERENCE_DATA.at(messageCounterNMEA).at(NMEA_Second)).margin(EPSILON));
	
	
	REQUIRE(obs->latitude() ==  Approx(NMEA_REFERENCE_DATA.at(messageCounterNMEA).at(NMEA_Latitude_rad)).margin(EPSILON));
	REQUIRE(obs->longitude() ==  Approx(NMEA_REFERENCE_DATA.at(messageCounterNMEA).at(NMEA_Longitude_rad)).margin(EPSILON));
    REQUIRE(obs->altitude() ==  Approx(NMEA_REFERENCE_DATA.at(messageCounterNMEA).at(NMEA_Height)).margin(EPSILON));
	
    REQUIRE(isnan(obs->e_velocity()[0]));
    REQUIRE(isnan(obs->e_velocity()[1]));
    REQUIRE(isnan(obs->e_velocity()[2]));

}

size_t messageCounterNMEA = 0; ///< Message Counter

TEST_CASE("[NMEAFile][flow] Read 'data/NMEA/test.nmea' and compare content with hardcoded values", "[NMEAFile][flow]")
{
    messageCounterNMEA = 0;

    Logger logger;

    // ###########################################################################################################
    //                                                  NMEAFile.flow"
    // ###########################################################################################################
    //
    // NmeaFile("test/flow/Nodes/DataProvider/GNSS/NMEAFile.flow") (95)           Plot (101)
    //                                         (94) PosVel |>  --(102)->  |> Pin 1 (96)
    //
    // ###########################################################################################################

    nm::RegisterWatcherCallbackToInputPin(96, [](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        LOG_TRACE("messageCounterNMEACsv = {}", messageCounterNMEA);

        compareNMEAData(std::dynamic_pointer_cast<const NAV::PosVel>(queue.front()), messageCounterNMEA);

         messageCounterNMEA++;
    });

    testFlow("test/flow/Nodes/DataProvider/GNSS/NMEAFile.flow");

    REQUIRE(messageCounterNMEA == NMEA_REFERENCE_DATA.size());
}



} // namespace NAV::TEST::NMEAFileTests
