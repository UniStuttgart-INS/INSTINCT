// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RtklibPosFileTests.cpp
/// @brief RtklibPosFile unit test
/// @author
/// @date

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

#include "RtklibPosObsComparisons.hpp"
#include "lla/lla_gpst2_111_dd_pos.hpp"
// This is a small hack, which lets us change private/protected parameters
#pragma GCC diagnostic push
#if defined(__clang__)
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "Nodes/DataProvider/GNSS/FileReader/RtklibPosFile.hpp"
#undef protected
#undef private
#pragma GCC diagnostic pop

namespace NAV::TESTS::RtklibPosFileTests
{
void testRtklibPosFileFlow(const std::string& path, const std::vector<RtklibPosObs>& rtklibPosObsRef)
{
    auto logger = initializeTestLogger();

    nm::RegisterPreInitCallback([&]() {
        dynamic_cast<RtklibPosFile*>(nm::FindNode(31))->_path = path;
    });

    // ###########################################################################################################
    //                                            RtklibPosFile.flow
    // ###########################################################################################################
    //
    // RtklibPosFile (31)                Combiner (28)
    //    (30) RtklibPosObs |> --(32)--> |> GnssObs (25)
    //                                   |> <not linked> (26)
    //
    // ###########################################################################################################

    size_t msgCounter = 0;

    nm::RegisterWatcherCallbackToInputPin(25, [&](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        auto rtklibPosObs = std::dynamic_pointer_cast<const NAV::RtklibPosObs>(queue.front());
        REQUIRE(rtklibPosObs != nullptr);

        CAPTURE(msgCounter);
        REQUIRE(*rtklibPosObs == rtklibPosObsRef[msgCounter]);

        msgCounter++;
    });

    REQUIRE(testFlow("test/flow/Nodes/DataProvider/GNSS/RtklibPosFile.flow"));

    REQUIRE(msgCounter == rtklibPosObsRef.size());
}
// ###########################################################################################################
//                                              LONG/LAT/ALT + GPST [hh:mm:ss]
// ###########################################################################################################

//  Lat Long Format [ddd.dddddd]

TEST_CASE("[RtklibPosFile][flow] Read lla/lla_gpst2_111_dd.pos", "[RtklibPosFile][flow]")
{
    testRtklibPosFileFlow("DataProvider/GNSS/RtklibPosFile/lla/lla_gpst2_111_dd.pos", lla::lla_gpst2_111_dd_pos);
}

// ######################################################## TODO ##################################################################
// Lat Long Format [dd mm ss.ss]

// TEST_CASE("[RtklibPosFile][flow] Read lla/gpst2/lla_gpst2_111_ds.pos", "[RtklibPosFile][flow]")
//{
//    testRtklibPosFileFlow("DataProvider/GNSS/RtklibPosFile/lla/gpst2/lla_gpst2_111_ds.pos", lla::gpst2::lla_gpst2_111_ds_pos);
//}

// ###########################################################################################################
//                                              LONG/LAT/ALT + GPST [ww ssss]
// ###########################################################################################################

// Lat Long Format [ddd.dddddd]

// TEST_CASE("[RtklibPosFile][flow] Read lla/gpst/lla_gpst_111_dd.pos", "[RtklibPosFile][flow]")
//{
//     testRtklibPosFileFlow("DataProvider/GNSS/RtklibPosFile/lla/gpst/lla_gpst_111_dd.pos", lla::gpst::lla_gpst_111_dd_pos);
// }

} // namespace NAV::TESTS::RtklibPosFileTests