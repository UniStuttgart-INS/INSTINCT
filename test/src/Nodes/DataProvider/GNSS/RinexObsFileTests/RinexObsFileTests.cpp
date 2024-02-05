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

#include "NodeData/GNSS/GnssObsComparisons.hpp"
#include "v3_02/INSA11DEU_R_MO_rnx.hpp"
#include "v3_03/reach-m2-01_22O.hpp"
#include "v3_04/INS_1581_19O.hpp"

// This is a small hack, which lets us change private/protected parameters
#pragma GCC diagnostic push
#if defined(__clang__)
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "Nodes/DataProvider/GNSS/FileReader/RinexObsFile.hpp"
#undef protected
#undef private
#pragma GCC diagnostic pop

namespace NAV::TESTS::RinexObsFileTests
{

void testRinexObsFileFlow(const std::string& path, const std::vector<GnssObs>& gnssObsRef)
{
    auto logger = initializeTestLogger();

    nm::RegisterPreInitCallback([&]() {
        dynamic_cast<RinexObsFile*>(nm::FindNode(2))->_path = path;
    });

    // ###########################################################################################################
    //                                             RinexObsFile.flow
    // ###########################################################################################################
    //
    // RinexObsFile (2)
    //       (1) GnssObs |> --(32)--> |> (30) Terminator (31)
    constexpr size_t PIN_ID_GNSS_OBS = 30;
    //
    // ###########################################################################################################

    size_t msgCounter = 0;

    nm::RegisterWatcherCallbackToInputPin(PIN_ID_GNSS_OBS, [&](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        auto gnssObs = std::dynamic_pointer_cast<const NAV::GnssObs>(queue.front());
        REQUIRE(gnssObs != nullptr);

        CAPTURE(msgCounter);
        REQUIRE(*gnssObs == gnssObsRef[msgCounter]);

        msgCounter++;
    });

    REQUIRE(testFlow("test/flow/Nodes/DataProvider/GNSS/RinexObsFile.flow"));

    REQUIRE(msgCounter == gnssObsRef.size());
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

TEST_CASE("[RinexObsFile][flow] Read v3_02/INSA11DEU_R_MO.rnx", "[RinexObsFile][flow]")
{
    testRinexObsFileFlow("DataProvider/GNSS/RinexObsFile/v3_02/INSA11DEU_R_MO.rnx", v3_02::gnssObs_INSA11DEU_R_MO_rnx);
}

// ###########################################################################################################
//                                                   v3.03
// ###########################################################################################################

TEST_CASE("[RinexObsFile][flow] Read v3_03/reach-m2-01_raw.22O", "[RinexObsFile][flow]")
{
    testRinexObsFileFlow("DataProvider/GNSS/RinexObsFile/v3_03/reach-m2-01_raw.22O", v3_03::gnssObs_reach_m2_01_22O);
}

// ###########################################################################################################
//                                                   v3.04
// ###########################################################################################################

TEST_CASE("[RinexObsFile][flow] Read v3_04/INS_1581.19O", "[RinexObsFile][flow]")
{
    testRinexObsFileFlow("DataProvider/GNSS/RinexObsFile/v3_04/INS_1581.19O", v3_04::gnssObs_INS_1581_19O);
}

// ###########################################################################################################
//                                                   v4.00
// ###########################################################################################################

// TODO: find data

// ###########################################################################################################
//                                               Corrupt Files
// ###########################################################################################################

// TODO: find data

} // namespace NAV::TESTS::RinexObsFileTests