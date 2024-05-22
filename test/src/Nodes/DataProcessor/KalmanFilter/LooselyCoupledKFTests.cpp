// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file LooselyCoupledKFTests.cpp
/// @brief Tests for the LooselyCoupledKF node
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-11-01

#include <catch2/catch_test_macros.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <deque>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <Eigen/Core>

#include "FlowTester.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "Logger.hpp"
#include "util/Container/CartesianProduct.hpp"

// This is a small hack, which lets us change private/protected parameters
#pragma GCC diagnostic push
#if defined(__clang__)
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "Nodes/DataProvider/IMU/FileReader/VectorNavFile.hpp"
#include "Nodes/DataProcessor/KalmanFilter/LooselyCoupledKF.hpp"
#undef protected
#undef private
#pragma GCC diagnostic pop

#include "NodeData/State/PosVelAtt.hpp"
#include "NodeData/State/InsGnssLCKFSolution.hpp"
#include "util/Logger/CommonLog.hpp"

namespace NAV::TESTS::LooselyCoupledKFTests
{

void testLCKFwithImuFile(const char* imuFilePath, size_t MESSAGE_COUNT_GNSS, size_t MESSAGE_COUNT_GNSS_FIX, size_t MESSAGE_COUNT_IMU, size_t MESSAGE_COUNT_IMU_FIX)
{
    auto logger = initializeTestLogger();

    bool imuAfter = std::string(imuFilePath) == "VectorNav/Static/vn310-imu-after.csv";

    std::array<std::vector<std::function<void()>>, 6> settings = { {
        { [&]() { LOG_WARN("Setting ImuIntegrator - _path to: {}", imuFilePath);
                  dynamic_cast<VectorNavFile*>(nm::FindNode(2))->_path = imuFilePath; } },
        { []() { LOG_WARN("Setting LooselyCoupledKF - _integrationFrame to: NED");
                 dynamic_cast<LooselyCoupledKF*>(nm::FindNode(10))->_inertialIntegrator._integrationFrame = InertialIntegrator::IntegrationFrame::NED; },
          []() { LOG_WARN("Setting LooselyCoupledKF - _integrationFrame to: ECEF");
                 dynamic_cast<LooselyCoupledKF*>(nm::FindNode(10))->_inertialIntegrator._integrationFrame = InertialIntegrator::IntegrationFrame::ECEF; } },
        { []() { LOG_WARN("Setting LooselyCoupledKF - _phiCalculationAlgorithm to: Taylor");
                 dynamic_cast<LooselyCoupledKF*>(nm::FindNode(10))->_phiCalculationAlgorithm = LooselyCoupledKF::PhiCalculationAlgorithm::Taylor; },
          []() { LOG_WARN("Setting LooselyCoupledKF - _phiCalculationAlgorithm to: Exponential");
                 dynamic_cast<LooselyCoupledKF*>(nm::FindNode(10))->_phiCalculationAlgorithm = LooselyCoupledKF::PhiCalculationAlgorithm::Exponential; } },
        { []() { LOG_WARN("Setting LooselyCoupledKF - _qCalculationAlgorithm to: Taylor1");
                 dynamic_cast<LooselyCoupledKF*>(nm::FindNode(10))->_qCalculationAlgorithm = LooselyCoupledKF::QCalculationAlgorithm::Taylor1; },
          []() { LOG_WARN("Setting LooselyCoupledKF - _qCalculationAlgorithm to: VanLoan");
                 dynamic_cast<LooselyCoupledKF*>(nm::FindNode(10))->_qCalculationAlgorithm = LooselyCoupledKF::QCalculationAlgorithm::VanLoan; } },
        {
            []() { LOG_WARN("Setting LooselyCoupledKF - _randomProcessAccel to: GaussMarkov1");
                 dynamic_cast<LooselyCoupledKF*>(nm::FindNode(10))->_randomProcessAccel = LooselyCoupledKF::RandomProcess::GaussMarkov1; },
            //   []() { LOG_WARN("Setting LooselyCoupledKF - _randomProcessAccel to: RandomWalk");
            //          dynamic_cast<LooselyCoupledKF*>(nm::FindNode(10))->_randomProcessAccel = LooselyCoupledKF::RandomProcess::RandomWalk; }
        },
        {
            []() { LOG_WARN("Setting LooselyCoupledKF - _randomProcessGyro to: GaussMarkov1");
                 dynamic_cast<LooselyCoupledKF*>(nm::FindNode(10))->_randomProcessGyro = LooselyCoupledKF::RandomProcess::GaussMarkov1; },
            //   []() { LOG_WARN("Setting LooselyCoupledKF - _randomProcessGyro to: RandomWalk");
            //          dynamic_cast<LooselyCoupledKF*>(nm::FindNode(10))->_randomProcessGyro = LooselyCoupledKF::RandomProcess::RandomWalk; }
        },
    } };

    cartesian_product_idx([&](size_t i0, size_t i1, size_t i2, size_t i3, size_t i4, size_t i5) {
        size_t messageCounter_VectorNavBinaryConverterImu_BinaryOutput = 0;
        size_t messageCounter_VectorNavBinaryConverterGnss_BinaryOutput = 0;
        size_t messageCounter_LCKF_ImuObs = 0;
        size_t messageCounter_LCKF_PosVel = 0;
        size_t messageCounter_LCKF_InitPVA = 0;

        auto timeOfFirstGnssObs = imuAfter ? InsTime() : InsTime(2, 255, 403575.870125);

        nm::RegisterPreInitCallback([&]() {
            settings[0][i0]();
            settings[1][i1]();
            settings[2][i2]();
            settings[3][i3]();
            settings[4][i4]();
            settings[5][i5]();
        });

        // VectorNavBinaryConverter (5) |> Binary Output (4)
        nm::RegisterWatcherCallbackToInputPin(4, [&](const Node* /* node */, const InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */) {
            messageCounter_VectorNavBinaryConverterImu_BinaryOutput++;
        });

        // VectorNavBinaryConverter (16) |> Binary Output (15)
        nm::RegisterWatcherCallbackToInputPin(15, [&](const Node* /* node */, const InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */) {
            messageCounter_VectorNavBinaryConverterGnss_BinaryOutput++;
        });

        // INS/GNSS LCKF (10) |> ImuObsIn (7)
        nm::RegisterWatcherCallbackToInputPin(7, [&]([[maybe_unused]] const Node* node, const InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */) {
            LOG_DEBUG("ImuObsIn [{}]: ImuObsIn: {}", messageCounter_LCKF_ImuObs, node->inputPins[0].queue.size());
            LOG_DEBUG("ImuObsIn [{}]: PosVel:   {}", messageCounter_LCKF_ImuObs, node->inputPins[1].queue.size());
            messageCounter_LCKF_ImuObs++;
        });

        // INS/GNSS LCKF (10) |> Init PVA (19)
        nm::RegisterWatcherCallbackToInputPin(19, [&]([[maybe_unused]] const Node* node, const InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */) {
            LOG_DEBUG("Init PVA [{}]: ImuObsIn: {}", messageCounter_LCKF_InitPVA, node->inputPins[0].queue.size());
            LOG_DEBUG("Init PVA [{}]: PosVel:   {}", messageCounter_LCKF_InitPVA, node->inputPins[1].queue.size());
            messageCounter_LCKF_InitPVA++;
        });

        // INS/GNSS LCKF (10) |> PosVel (8)
        nm::RegisterWatcherCallbackToInputPin(8, [&]([[maybe_unused]] const Node* node, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
            LOG_DEBUG("PosVel   [{}]: ImuObsIn: {}", messageCounter_LCKF_PosVel, node->inputPins[0].queue.size());
            LOG_DEBUG("PosVel   [{}]: PosVel:   {}", messageCounter_LCKF_PosVel, node->inputPins[1].queue.size());
            messageCounter_LCKF_PosVel++;

            auto obs = std::static_pointer_cast<const PosVelAtt>(queue.front());
            // LOG_TRACE("PosVelAtt time = [{} - {}]", obs->insTime.toYMDHMS(), obs->insTime.toGPSweekTow());

            Eigen::Vector3d refPos_lla(deg2rad(48.780704498291016), deg2rad(9.171577453613281), 327.3);
            Eigen::Vector3d refRollPitchYaw(-1.745, -0.567, -92.049);
            Eigen::Vector3d allowedPositionOffsetImuOnly_n(2.5, 1.2, 2.0);
            Eigen::Vector3d allowedPositionOffsetCombined_n(2.5, 1.2, 2.0);
            Eigen::Vector3d allowedVelocityErrorImuOnly_e(0.25, 0.17, 0.25);
            Eigen::Vector3d allowedVelocityErrorCombined_e(0.25, 0.17, 0.25);
            Eigen::Vector3d allowedRollPitchYawOffsetImuOnly(1.9, 3.0, 31.0);
            Eigen::Vector3d allowedRollPitchYawOffsetCombined(1.9, 3.0, 31.0);

            // North/South deviation [m]
            double northSouth = calcGeographicalDistance(obs->latitude(), obs->longitude(),
                                                         refPos_lla.x(), obs->longitude());

            // East/West deviation [m]
            double eastWest = calcGeographicalDistance(obs->latitude(), obs->longitude(),
                                                       obs->latitude(), refPos_lla.y());

            REQUIRE(northSouth <= (obs->insTime < timeOfFirstGnssObs ? allowedPositionOffsetImuOnly_n(0) : allowedPositionOffsetCombined_n(0)));
            REQUIRE(eastWest <= (obs->insTime < timeOfFirstGnssObs ? allowedPositionOffsetImuOnly_n(1) : allowedPositionOffsetCombined_n(1)));
            REQUIRE(std::abs(obs->altitude() - refPos_lla(2)) <= (obs->insTime < timeOfFirstGnssObs ? allowedPositionOffsetImuOnly_n(2) : allowedPositionOffsetCombined_n(2)));

            REQUIRE(obs->e_velocity()(0) <= (obs->insTime < timeOfFirstGnssObs ? allowedVelocityErrorImuOnly_e(0) : allowedVelocityErrorCombined_e(0)));
            REQUIRE(obs->e_velocity()(1) <= (obs->insTime < timeOfFirstGnssObs ? allowedVelocityErrorImuOnly_e(1) : allowedVelocityErrorCombined_e(1)));
            REQUIRE(obs->e_velocity()(2) <= (obs->insTime < timeOfFirstGnssObs ? allowedVelocityErrorImuOnly_e(2) : allowedVelocityErrorCombined_e(2)));

            REQUIRE(std::abs(rad2deg(obs->rollPitchYaw()(0)) - refRollPitchYaw(0)) <= (obs->insTime < timeOfFirstGnssObs ? allowedRollPitchYawOffsetImuOnly(0) : allowedRollPitchYawOffsetCombined(0)));
            REQUIRE(std::abs(rad2deg(obs->rollPitchYaw()(1)) - refRollPitchYaw(1)) <= (obs->insTime < timeOfFirstGnssObs ? allowedRollPitchYawOffsetImuOnly(1) : allowedRollPitchYawOffsetCombined(1)));
            REQUIRE(std::abs(rad2deg(obs->rollPitchYaw()(2)) - refRollPitchYaw(2)) <= (obs->insTime < timeOfFirstGnssObs ? allowedRollPitchYawOffsetImuOnly(2) : allowedRollPitchYawOffsetCombined(2)));
        });

        // ###########################################################################################################
        //                                         LooselyCoupledKF.flow
        // ###########################################################################################################
        //
        // VectorNavFile - IMU (2)           VectorNavBinaryConverter (5)
        //    (1) Binary Output |> --(6)-->  |> Binary Input (4)    (3) ImuObsWDelta |> \         INS/GNSS LCKF (10)                           Plot (26)
        //                                                                               \(36)--> |> ImuObsIn (7)   (9) PosVelAtt |> --(27)--> |> LCKF (21)
        // VectorNavFile - GNSS (13)            VectorNavBinaryConverter (16)           /-(18)--> |> PosVel (8)                               -|> VectorNav (37)
        //     (12) Binary Output |> --(17)-->  |> Binary Input (15)  (14) PosVelAtt |> --(20)--> |> Init PVA (19)                           /
        //                                                                              \---------------------------(38)--------------------/
        //
        // ###########################################################################################################
        REQUIRE(testFlow("test/flow/Nodes/DataProcessor/KalmanFilter/LooselyCoupledKF.flow"));

        REQUIRE(messageCounter_VectorNavBinaryConverterImu_BinaryOutput == MESSAGE_COUNT_IMU);
        REQUIRE(messageCounter_VectorNavBinaryConverterGnss_BinaryOutput == MESSAGE_COUNT_GNSS);
        REQUIRE(messageCounter_LCKF_ImuObs == MESSAGE_COUNT_IMU_FIX);
        REQUIRE(messageCounter_LCKF_InitPVA == 1);
        REQUIRE(messageCounter_LCKF_PosVel == MESSAGE_COUNT_GNSS_FIX); // First GNSS message is used to initialize filter, does not update
    },
                          settings);
}

TEST_CASE("[LooselyCoupledKF][flow] Test flow with IMU data arriving before GNSS data", "[LooselyCoupledKF][flow]")
{
    size_t MESSAGE_COUNT_GNSS = 163;
    size_t MESSAGE_COUNT_GNSS_FIX = 163;
    size_t MESSAGE_COUNT_IMU = 1612;
    size_t MESSAGE_COUNT_IMU_FIX = 1612;

    testLCKFwithImuFile("DataProcessor/lckf/vn310-imu.csv", MESSAGE_COUNT_GNSS, MESSAGE_COUNT_GNSS_FIX, MESSAGE_COUNT_IMU, MESSAGE_COUNT_IMU_FIX);
}

TEST_CASE("[LooselyCoupledKF][flow] Test flow with IMU data arriving after GNSS data", "[LooselyCoupledKF][flow]")
{
    size_t MESSAGE_COUNT_GNSS = 163;
    size_t MESSAGE_COUNT_GNSS_FIX = 163;
    size_t MESSAGE_COUNT_IMU = 1610;
    size_t MESSAGE_COUNT_IMU_FIX = 1610;

    testLCKFwithImuFile("DataProcessor/lckf/vn310-imu-after.csv", MESSAGE_COUNT_GNSS, MESSAGE_COUNT_GNSS_FIX, MESSAGE_COUNT_IMU, MESSAGE_COUNT_IMU_FIX);
}

} // namespace NAV::TESTS::LooselyCoupledKFTests
