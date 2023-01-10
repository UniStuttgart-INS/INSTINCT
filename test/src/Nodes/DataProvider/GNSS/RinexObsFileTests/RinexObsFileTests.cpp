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

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"
#include <string>
#include <fstream>
#include <sstream>
#include <limits>

#include "FlowTester.hpp"
#include "Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "util/Logger.hpp"

#include "NodeData/State/PosVel.hpp"

#include "RinexObsFileTests.hpp"
#include "v3_03/reach-m2-01_raw_202211021639_test_22O.hpp"

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

namespace NAV::TESTS::RinexObsFileTests
{

void compareObservation(const std::shared_ptr<const NAV::GnssObs>& obs)
{
    // ---------------------------------------------- InsTime ------------------------------------------------
    REQUIRE(!obs->insTime.empty());

    REQUIRE(obs->insTime.toYMDHMS().year == static_cast<int32_t>(RINEX_REFERENCE_EPOCH.at(RINEX_Year)));
    REQUIRE(obs->insTime.toYMDHMS().month == static_cast<int32_t>(RINEX_REFERENCE_EPOCH.at(RINEX_Month)));
    REQUIRE(obs->insTime.toYMDHMS().day == static_cast<int32_t>(RINEX_REFERENCE_EPOCH.at(RINEX_Day)));
    REQUIRE(obs->insTime.toYMDHMS().hour == static_cast<int32_t>(RINEX_REFERENCE_EPOCH.at(RINEX_Hour)));
    REQUIRE(obs->insTime.toYMDHMS().min == static_cast<int32_t>(RINEX_REFERENCE_EPOCH.at(RINEX_Minute)));
    REQUIRE_THAT(obs->insTime.toYMDHMS().sec, Catch::Matchers::WithinAbs(RINEX_REFERENCE_EPOCH.at(RINEX_Second) - Gps_LeapSec, 9e-12));

    // -------------------------------------------- Observation ----------------------------------------------
    for (size_t obsCounter = 0; obsCounter < RINEX_REFDATA_SATSYS.size(); obsCounter++)
    {
        REQUIRE(obs->data.at(obsCounter).satSigId.freq == RINEX_REFDATA_SATSYS.at(obsCounter));
        REQUIRE(obs->data.at(obsCounter).satSigId.satNum == static_cast<uint16_t>(RINEX_REFERENCE_DATA.at(obsCounter).at(RINEX_SatNum)));
        REQUIRE_THAT(obs->data.at(obsCounter).pseudorange, Catch::Matchers::WithinAbs(RINEX_REFERENCE_DATA.at(obsCounter).at(RINEX_Obs_Pseudorange), EPSILON_LDOUBLE));
        if (!std::isnan(obs->data.at(obsCounter).carrierPhase))
        {
            REQUIRE_THAT(obs->data.at(obsCounter).carrierPhase, Catch::Matchers::WithinAbs(RINEX_REFERENCE_DATA.at(obsCounter).at(RINEX_Obs_CarrierPhase), EPSILON_LDOUBLE));
        }
        if (!std::isnan(obs->data.at(obsCounter).doppler))
        {
            REQUIRE_THAT(obs->data.at(obsCounter).doppler, Catch::Matchers::WithinAbs(RINEX_REFERENCE_DATA.at(obsCounter).at(RINEX_Obs_Doppler), EPSILON_LDOUBLE));
        }
        if (!std::isnan(obs->data.at(obsCounter).CN0))
        {
            REQUIRE_THAT(obs->data.at(obsCounter).CN0, Catch::Matchers::WithinAbs(RINEX_REFERENCE_DATA.at(obsCounter).at(RINEX_Obs_SigStrength), EPSILON_LDOUBLE));
        }
    }
}

void testRinexObsFileFlow(const std::string& path)
{
    auto logger = initializeTestLogger();

    nm::RegisterPreInitCallback([&]() {
        dynamic_cast<RinexObsFile*>(nm::FindNode(2))->_path = path;
    });

    // ###########################################################################################################
    //                                             RinexObsFile.flow
    // ###########################################################################################################
    //
    // RinexObsFile (2)                Combiner (28)
    //       (1) GnssObs |> --(29)--> |> GnssObs (25)
    //                                |> <not linked> (26)
    //
    // ###########################################################################################################

    nm::RegisterWatcherCallbackToInputPin(25, [](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        compareObservation(std::dynamic_pointer_cast<const NAV::GnssObs>(queue.front()));
    });

    // TODO: inlcude test for Rinex Obs Header?: 'RINEX_SYS_NUM_OBS_TYPES_TEST' -- not possible, since gnssObs (the only output) doesn't contain that info

    REQUIRE(testFlow("test/flow/Nodes/DataProvider/GNSS/RinexObsFile.flow"));
}

// ###########################################################################################################
//                                                   v2.01
// ###########################################################################################################

// TODO: find data

// ###########################################################################################################
//                                                   v2.10
// ###########################################################################################################

// TODO: find data

// ###########################################################################################################
//                                                   v2.11
// ###########################################################################################################

// TODO: find data

// ###########################################################################################################
//                                                   v3.02
// ###########################################################################################################

// TODO: find data

// ###########################################################################################################
//                                                   v3.04
// ###########################################################################################################

// TODO: find data

// ###########################################################################################################
//                                                   v3.03
// ###########################################################################################################

TEST_CASE("[RinexObsFile][flow] Read v3_03/reach-m2-01_raw_202211021639_test.22O", "[RinexObsFile][flow][debug]")
{
    testRinexObsFileFlow("DataProvider/GNSS/RinexObsFile/v3_03/reach-m2-01_raw_202211021639_test.22O");
}

// ###########################################################################################################
//                                                   v3.04
// ###########################################################################################################

// TODO: find data

// ###########################################################################################################
//                                                   v4.00
// ###########################################################################################################

// TODO: find data

// ###########################################################################################################
//                                               Corrupt Files
// ###########################################################################################################

// TODO: find data

} // namespace NAV::TESTS::RinexObsFileTests