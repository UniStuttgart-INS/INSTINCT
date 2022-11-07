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

size_t messageCounterInput = 0;  ///< Message Counter for the incoming simulated Imu data
size_t messageCounterOutput = 0; ///< Message Counter for the outgoing simulated Imu data

TEST_CASE("[TimeWindow][flow] Simulate IMU and cut off start and end time", "[TimeWindow][flow]")
{
    messageCounterInput = 0;
    messageCounterOutput = 0;

    Logger logger;

    // ###########################################################################################################
    //                                              TimeWindow.flow
    // ###########################################################################################################
    //
    // ImuSimulator (4) |>--->| (1) TimeWindow (2) |>--->| (8) Plot
    //
    // ###########################################################################################################

    nm::RegisterWatcherCallbackToInputPin(1, [](const Node* /* node */, const InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */) {
        messageCounterInput++;
    });

    nm::RegisterWatcherCallbackToInputPin(8, [](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        REQUIRE(queue.front()->insTime >= InsTime(2000, 1, 1, 0, 0, 1.5)); // Start time
        REQUIRE(queue.front()->insTime <= InsTime(2000, 1, 1, 0, 0, 8.5)); // End time

        messageCounterOutput++;
    });

    testFlow("test/flow/Nodes/util/TimeWindow.flow");

    REQUIRE(messageCounterInput == 101);
    REQUIRE(messageCounterOutput == 71);
}

} // namespace NAV::TEST::TimeWindowTests
