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

#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>

#include "FlowTester.hpp"
#include "CatchMatchers.hpp"

#include "internal/FlowManager.hpp"

#include "Logger.hpp"
#include <chrono>

// This is a small hack, which lets us change private/protected parameters
#if defined(__clang__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "Nodes/Utility/Combiner.hpp"
#undef protected
#undef private
#if defined(__clang__)
    #pragma GCC diagnostic pop
#endif

namespace NAV::TESTS::CombinerTests
{

TEST_CASE("[Combiner][flow] Simulate IMU and cut off start and end time", "[Combiner][flow]")
{
    auto logger = initializeTestLogger("Combiner");

    // ###########################################################################################################
    //                                              Combiner.flow
    // ###########################################################################################################
    //
    //  Circle 0.00 (6)
    //         (7) ImuObs |>
    //  (8) PosVelAtt 5Hz |> -- --------------------------------------------
    //                         |                                            |
    //  Circle 0.00 (3)        |          Combiner (17)                     |         Plot (23)
    //         (1) ImuObs |>    --(19)--> |> (14) C0.00 5Hz   Comb (16) |> ---(24)--> |> Pin 1 (22)
    //  (2) PosVelAtt 1Hz |> -----(20)--> |> (15) C0.00 1Hz                 --(52)--> |> Pin 2 (47)
    //                           -(21)--> |> (18) L0.00 5Hz                ---(48)--> |> Pin 3 (48)
    //  Linear 0.01 (11)        |    ---> |> (34) L0.01 5Hz                ---(50)--> |> Pin 4 (49)
    //         (12) ImuObs |>   |   |                                     |       --> |> Pin 5 (36)
    //  (13) PosVelAtt 5Hz |> --    |                                 ----       |
    //                              |                                            |
    //  Linear 0.01 (31)           (35)                                          |
    //         (32) ImuObs |>       |                                            |
    //  (33) PosVelAtt 5Hz |> ------ -------------------------(37)---------------
    //
    //   Messages            | 0.0 | 0.1 | 0.2 | 0.3 | 0.4 | 0.5 | 0.6 | 0.7 | 0.8 | 0.9 | 1.0 | 1.1 | 1.2 | 1.4 | 1.6 | 1.8 | 2.0 |
    // --------------------- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
    // Circle 0.0 5Hz (6)    |  X  |     |  X  |     |  X  |     |  X  |     |  X  |     |  X  |     |  X  |  X  |  X  |  X  |  X  |
    // Circle 0.0 1Hz (3)    |  X  |     |     |     |     |     |     |     |     |     |  X  |     |     |     |     |     |  X  |
    // + C0.0 5Hz - C0.0 1Hz |  X  |     |     |     |     |     |     |     |     |     |  X  |     |     |     |     |     |  X  |
    // --------------------- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
    // Linear 0.0 5Hz (11)   |  X  |     |  X  |     |  X  |     |  X  |     |  X  |     |  X  |     |     |     |     |     |     |
    // Linear 0.1 5Hz (31)   |     |  X  |     |  X  |     |  X  |     |  X  |     |  X  |     |  X  |     |     |     |     |     |
    // + L0.0 5Hz - L0.1 5Hz |     |  X  |     |  X  |     |  X  |     |  X  |     |  X  |     |     |     |     |     |     |     |
    // --------------------- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
    //
    // ###########################################################################################################

    InsTime startTime(2000, 1, 1, 0, 0, 0.0, GPST);

    size_t messageCounter1 = 0;
    flow::RegisterWatcherCallbackToInputPin(14, [&](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        CAPTURE(messageCounter1);
        InsTime expectedTime = startTime + std::chrono::milliseconds(messageCounter1 * 200);
        REQUIRE_THAT(queue.front()->insTime, Catch::Matchers::WithinAbs(expectedTime, std::chrono::microseconds(1)));
        messageCounter1++;
    });

    size_t messageCounter2 = 0;
    flow::RegisterWatcherCallbackToInputPin(15, [&](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        CAPTURE(messageCounter2);
        InsTime expectedTime = startTime + std::chrono::seconds(messageCounter2);
        REQUIRE_THAT(queue.front()->insTime, Catch::Matchers::WithinAbs(expectedTime, std::chrono::microseconds(1)));
        messageCounter2++;
    });

    size_t messageCounter3 = 0;
    flow::RegisterWatcherCallbackToInputPin(18, [&](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        CAPTURE(messageCounter3);
        InsTime expectedTime = startTime + std::chrono::milliseconds(messageCounter3 * 200);
        REQUIRE_THAT(queue.front()->insTime, Catch::Matchers::WithinAbs(expectedTime, std::chrono::microseconds(1)));
        messageCounter3++;
    });

    size_t messageCounter4 = 0;
    flow::RegisterWatcherCallbackToInputPin(34, [&](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        CAPTURE(messageCounter4);
        InsTime expectedTime = startTime + std::chrono::milliseconds(100 + messageCounter4 * 200);
        REQUIRE_THAT(queue.front()->insTime, Catch::Matchers::WithinAbs(expectedTime, std::chrono::microseconds(1)));
        messageCounter4++;
    });

    size_t messageCounterComb = 0;
    flow::RegisterWatcherCallbackToInputPin(22, [&](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        CAPTURE(messageCounterComb);
        InsTime expectedTime = startTime + std::chrono::milliseconds(messageCounterComb * 200 - 100);
        const auto& descriptors = queue.front()->dynamicDataDescriptors();
        CHECK(descriptors.size() == 1);
        if (messageCounterComb == 0) // Circle
        {
            expectedTime = startTime;
            CHECK(descriptors.front() == "+ North/South [m] (C0.0 5Hz) - North/South [m] (C0.0 1Hz)");
        }
        else if (messageCounterComb <= 5) // Line
        {
            CHECK(descriptors.front() == "+ North/South [m] (L0.0 5Hz) - North/South [m] (L0.1 5Hz)");
        }
        else if (messageCounterComb == 6) // Circle
        {
            expectedTime = startTime + std::chrono::milliseconds(1000);
            CHECK(descriptors.front() == "+ North/South [m] (C0.0 5Hz) - North/South [m] (C0.0 1Hz)");
        }
        else if (messageCounterComb == 7) // Circle
        {
            expectedTime = startTime + std::chrono::milliseconds(2000);
            CHECK(descriptors.front() == "+ North/South [m] (C0.0 5Hz) - North/South [m] (C0.0 1Hz)");
        }
        CHECK_THAT(queue.front()->insTime, Catch::Matchers::WithinAbs(expectedTime, std::chrono::microseconds(1)));
        LOG_DATA("CombinerTests: [{}] {}", queue.front()->insTime.toYMDHMS(GPST), queue.front()->dynamicDataDescriptors());

        messageCounterComb++;
    });

    REQUIRE(testFlow("test/flow/Nodes/Utility/Combiner.flow"));

    REQUIRE(messageCounter1 == 11);
    REQUIRE(messageCounter2 == 3);
    REQUIRE(messageCounter3 == 6);
    REQUIRE(messageCounter4 == 6);
    REQUIRE(messageCounterComb == 8);
}

} // namespace NAV::TESTS::CombinerTests
