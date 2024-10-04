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

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "Logger.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "MultiImuFileTestsData.hpp"

// This is a small hack, which lets us change private/protected parameters
#if defined(__clang__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "Nodes/DataProvider/IMU/FileReader/MultiImuFile.hpp"
#undef protected
#undef private
#if defined(__clang__)
    #pragma GCC diagnostic pop
#endif

namespace NAV::TESTS::MultiImuFileTests
{
void compareImuObservation(const std::shared_ptr<const NAV::ImuObs>& obs, size_t messageCounterData, size_t pinIdx)
{
    // --------------------------------------------- Sensor ID -----------------------------------------------
    REQUIRE(pinIdx == static_cast<size_t>(IMU_REFERENCE_DATA.at(messageCounterData).at(SensorId)) - 1); // '-1' due to 1-based SensorIds

    // --------------------------------------------- Ins Time ------------------------------------------------
    REQUIRE(!obs->insTime.empty());

    REQUIRE(obs->insTime.toGPSweekTow().gpsCycle == IMU_STARTTIME.toGPSweekTow().gpsCycle);
    REQUIRE(obs->insTime.toGPSweekTow().gpsWeek == IMU_STARTTIME.toGPSweekTow().gpsWeek);
    long double gpsDayOfWeek = std::floor(IMU_STARTTIME.toGPSweekTow().tow / InsTimeUtil::SECONDS_PER_DAY);
    REQUIRE_THAT(obs->insTime.toGPSweekTow().tow
                     - gpsDayOfWeek * static_cast<long double>(InsTimeUtil::SECONDS_PER_DAY)
                     - timestamp(IMU_REFERENCE_DATA.at(messageCounterData).at(GpsSec), IMU_REFERENCE_DATA.at(messageCounterData).at(GpsNum), IMU_REFERENCE_DATA.at(messageCounterData).at(GpsDen))
                     - static_cast<long double>(IMU_STARTTIME.differenceToUTC(GPST)),
                 Catch::Matchers::WithinAbs(0.0L, 5e-7L));

    // ------------------------------------------- Accelerations ---------------------------------------------
    REQUIRE_THAT(obs->p_acceleration(0) - IMU_REFERENCE_DATA.at(messageCounterData).at(AccX) * SCALEFACTOR_ACCEL, Catch::Matchers::WithinAbs(0.0L, 5e-7L));
    REQUIRE_THAT(obs->p_acceleration(1) - IMU_REFERENCE_DATA.at(messageCounterData).at(AccY) * SCALEFACTOR_ACCEL, Catch::Matchers::WithinAbs(0.0L, 5e-7L));
    REQUIRE_THAT(obs->p_acceleration(2) - IMU_REFERENCE_DATA.at(messageCounterData).at(AccZ) * SCALEFACTOR_ACCEL, Catch::Matchers::WithinAbs(0.0L, 5e-7L));

    // ------------------------------------------- Angular rates ---------------------------------------------
    REQUIRE_THAT(obs->p_angularRate(0) - deg2rad(IMU_REFERENCE_DATA.at(messageCounterData).at(GyroX)) * SCALEFACTOR_GYRO, Catch::Matchers::WithinAbs(0.0L, 5e-7L));
    REQUIRE_THAT(obs->p_angularRate(1) - deg2rad(IMU_REFERENCE_DATA.at(messageCounterData).at(GyroY)) * SCALEFACTOR_GYRO, Catch::Matchers::WithinAbs(0.0L, 5e-7L));
    REQUIRE_THAT(obs->p_angularRate(2) - deg2rad(IMU_REFERENCE_DATA.at(messageCounterData).at(GyroZ)) * SCALEFACTOR_GYRO, Catch::Matchers::WithinAbs(0.0L, 5e-7L));
}

long double timestamp(double gpsSecond, double timeNumerator, double timeDenominator)
{
    return static_cast<long double>(gpsSecond + timeNumerator / timeDenominator);
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

    for (size_t inputPinId : inputPinIds)
    {
        nm::RegisterWatcherCallbackToInputPin(inputPinId, [&messageCounter](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t pinIdx) {
            LOG_TRACE("messageCounter = {}", messageCounter);

            compareImuObservation(std::dynamic_pointer_cast<const NAV::ImuObs>(queue.front()), messageCounter, pinIdx);

            messageCounter++;
        });
    }

    REQUIRE(testFlow("test/flow/Nodes/DataProvider/IMU/MultiImuFile.flow"));

    REQUIRE(messageCounter == IMU_REFERENCE_DATA.size());
}
} // namespace NAV::TESTS::MultiImuFileTests