// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file TimeWindowTests.cpp
/// @brief Time Window Tests
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2022-11-07

#include <catch2/catch.hpp>

#include "FlowTester.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "util/Logger.hpp"

namespace NAV::TEST::TimeWindowTests
{

TEST_CASE("[TimeWindow][flow] Simulate IMU and cut off start and end time", "[TimeWindow][flow]")
{
    Logger logger;

    // ###########################################################################################################
    //                                              TimeWindow.flow
    // ###########################################################################################################
    //
    //  ImuSimulator (6)             TimeWindow (3)                           Plot (13)
    //     (4) ImuObs |>  ---(7)-->  |> Input (1)  (2) Output |>  ---(14)-->  |> Pin 1 (8)
    //  (5) PosVelAtt |>
    //
    // ###########################################################################################################

    size_t messageCounterInput = 0;
    nm::RegisterWatcherCallbackToInputPin(1, [&messageCounterInput](const Node* /* node */, const InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */) {
        messageCounterInput++;
    });

    size_t messageCounterOutput = 0;
    nm::RegisterWatcherCallbackToInputPin(8, [&messageCounterOutput](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        REQUIRE(queue.front()->insTime >= InsTime(2000, 1, 1, 0, 0, 1.5)); // Start time
        REQUIRE(queue.front()->insTime <= InsTime(2000, 1, 1, 0, 0, 8.5)); // End time

        messageCounterOutput++;
    });

    REQUIRE(testFlow("test/flow/Nodes/util/TimeWindow.flow"));

    REQUIRE(messageCounterInput == 101);
    REQUIRE(messageCounterOutput == 71);
}

} // namespace NAV::TEST::TimeWindowTests
