// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file MultiIMUFileTests.cpp
/// @brief Tests for the MultiIMUFile node
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2023-08-09

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"

#include <string>
#include <fstream>
#include <sstream>
#include <limits>

#include "FlowTester.hpp"

#include "NodeData/IMU/VectorNavBinaryOutput.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "Logger.hpp"

#include "MultiImuFileTestsData.hpp"

// This is a small hack, which lets us change private/protected parameters
#pragma GCC diagnostic push
#if defined(__clang__)
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "Nodes/DataProvider/IMU/FileReader/MultiImuFile.hpp"
#undef protected
#undef private
#pragma GCC diagnostic pop

namespace NAV::TESTS::MultiImuFileTests
{
void compareImuObservation(const std::shared_ptr<const NAV::ImuObs>& obs, size_t /*messageCounterData*/, size_t /*pinIdx*/)
{
    // --------------------------------------------- Sensor ID -----------------------------------------------
    // REQUIRE(pinIdx == static_cast<size_t>(IMU_REFERENCE_DATA.at(messageCounterData).at(SensorId)) - 1); // '-1' due to 1-based SensorIds

    // --------------------------------------------- Ins Time ------------------------------------------------
    REQUIRE(!obs->insTime.empty());

    REQUIRE(obs->insTime.toGPSweekTow().gpsCycle == IMU_STARTTIME.toGPSweekTow().gpsCycle);
    REQUIRE(obs->insTime.toGPSweekTow().gpsWeek == IMU_STARTTIME.toGPSweekTow().gpsWeek);
    // REQUIRE(obs->insTime.toGPSweekTow().tow == timestamp(IMU_REFERENCE_DATA.at(messageCounterData).at(GpsSec), IMU_REFERENCE_DATA.at(messageCounterData).at(GpsNum), IMU_REFERENCE_DATA.at(messageCounterData).at(GpsDen), IMU_STARTTIME.toGPSweekTow().tow));

    // ------------------------------------------- Accelerations ---------------------------------------------
    // TODO: Add 'REQUIRE' for accels

    // ------------------------------------------- Angular rates ---------------------------------------------
    // TODO: Add 'REQUIRE' for gyro
}

long double timestamp(double gpsSecond, double timeNumerator, double timeDenominator, long double startupGpsSecond)
{
    return static_cast<long double>(gpsSecond + timeNumerator / timeDenominator) - startupGpsSecond;
}

TEST_CASE("[MultiImuFile][flow] Read 'data/DataProvider/IMU/2023-08-09_Multi-IMU_commaDelim.txt' and compare content with hardcoded values", "[MultiImuFile][flow]")
{
    auto logger = initializeTestLogger();

    // #######################################################################################################
    //                                           MultiImuFile.flow
    // #######################################################################################################
    //
    //   MultiImuFile (6)            Plot (12)
    //       (1) ImuObs 1 |> --(13)-->  |> Pin 1 (7)
    //       (2) ImuObs 2 |> --(18)-->  |> Pin 1 (14)
    //       (3) ImuObs 3 |> --(19)-->  |> Pin 1 (15)
    //       (4) ImuObs 4 |> --(20)-->  |> Pin 1 (16)
    //       (5) ImuObs 5 |> --(21)-->  |> Pin 1 (17)
    //
    // #######################################################################################################

    nm::RegisterPreInitCallback([&]() {
        dynamic_cast<MultiImuFile*>(nm::FindNode(6))->_path = "DataProvider/IMU/2023-08-09_Multi-IMU_commaDelim.txt";
    });

    size_t messageCounter = 0;

    std::array<size_t, 5> inputPinIds = { 7, 14, 15, 16, 17 };

    for (size_t i = 0; i < inputPinIds.size(); i++)
    {
        nm::RegisterWatcherCallbackToInputPin(inputPinIds.at(i), [&messageCounter](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t pinIdx) {
            LOG_TRACE("messageCounter = {}", messageCounter);

            compareImuObservation(std::dynamic_pointer_cast<const NAV::ImuObs>(queue.front()), messageCounter, pinIdx);

            messageCounter++;
        });
    }

    REQUIRE(testFlow("test/flow/Nodes/DataProvider/IMU/MultiImuFile.flow"));

    REQUIRE(messageCounter == IMU_REFERENCE_DATA.size());
}
} // namespace NAV::TESTS::MultiImuFileTests