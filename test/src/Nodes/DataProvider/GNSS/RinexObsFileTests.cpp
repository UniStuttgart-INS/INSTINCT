// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RinexObsFileTests.cpp
/// @brief RinexObsFile unit test
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2022-11-11

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
#include "NodeData/GNSS/GnssObs.hpp"

#include "RinexObsFileTestsData.hpp"

// This is a small hack, which lets us change private/protected parameters
#pragma GCC diagnostic push
#if defined(__clang__)
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "Nodes/DataProvider/GNSS/FileReader/RINEX/RinexObsFile.hpp"
#undef protected
#undef private
#pragma GCC diagnostic pop

namespace NAV::TEST::RinexObsFileTests
{

constexpr double EPSILON = 10.0 * std::numeric_limits<double>::epsilon();

void compareObservation(const std::shared_ptr<const NAV::GnssObs>& obs, size_t messageCounter)
{
    // ---------------------------------------------- InsTime ------------------------------------------------
    if (messageCounter == 0) // check Epoch timestamp just once at the beginning
    {
        REQUIRE(!obs->insTime.empty());

        REQUIRE(obs->insTime.toYMDHMS().year == static_cast<int32_t>(RINEX_REFERENCE_EPOCH.at(RINEX_Year)));
        REQUIRE(obs->insTime.toYMDHMS().month == static_cast<int32_t>(RINEX_REFERENCE_EPOCH.at(RINEX_Month)));
        REQUIRE(obs->insTime.toYMDHMS().day == static_cast<int32_t>(RINEX_REFERENCE_EPOCH.at(RINEX_Day)));
        REQUIRE(obs->insTime.toYMDHMS().hour == static_cast<int32_t>(RINEX_REFERENCE_EPOCH.at(RINEX_Hour)));
        REQUIRE(obs->insTime.toYMDHMS().min == static_cast<int32_t>(RINEX_REFERENCE_EPOCH.at(RINEX_Minute)));
        REQUIRE(obs->insTime.toYMDHMS().sec == Approx(RINEX_REFERENCE_EPOCH.at(RINEX_Second) - Gps_LeapSec).margin(EPSILON));
    }

    // -------------------------------------------- Observation ----------------------------------------------
    REQUIRE(obs->data.at(messageCounter).pseudorange == Approx(RINEX_REFERENCE_DATA.at(messageCounter).at(RINEX_Obs_Pseudorange)).margin(EPSILON));
    REQUIRE(obs->data.at(messageCounter).carrierPhase == Approx(RINEX_REFERENCE_DATA.at(messageCounter).at(RINEX_Obs_CarrierPhase)).margin(EPSILON));
    REQUIRE(obs->data.at(messageCounter).doppler == Approx(RINEX_REFERENCE_DATA.at(messageCounter).at(RINEX_Obs_Doppler)).margin(EPSILON));
    REQUIRE(obs->data.at(messageCounter).CN0 == Approx(RINEX_REFERENCE_DATA.at(messageCounter).at(RINEX_Obs_SigStrength)).margin(EPSILON));
}

TEST_CASE("[RinexObsFile][flow] Read RINEX file (v3.03) and compare content with hardcoded values", "[RinexObsFile][flow]")
{
    Logger logger;

    // ###########################################################################################################
    //                                             RinexObsFile.flow
    // ###########################################################################################################
    //
    // RinexObsFile (2)                Combiner (28)
    //       (1) GnssObs |> --(29)--> |> GnssObs (25)
    //                                |> <not linked> (26)
    //
    // ###########################################################################################################

    // TODO: Add tests for more Rinex versions
    // nm::RegisterPreInitCallback([&]() { dynamic_cast<RinexObsFile*>(nm::FindNode(2))->_path = "Rinex/FixedSize/vn310-imu.csv"; });

    size_t messageCounter = 0;

    nm::RegisterWatcherCallbackToInputPin(25, [&messageCounter](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        LOG_TRACE("messageCounter = {}", messageCounter);

        compareObservation(std::dynamic_pointer_cast<const NAV::GnssObs>(queue.front()), messageCounter);

        messageCounter++;
    });

    REQUIRE(testFlow("test/flow/Nodes/DataProvider/GNSS/RinexObsFile.flow"));
}

} // namespace NAV::TEST::RinexObsFileTests