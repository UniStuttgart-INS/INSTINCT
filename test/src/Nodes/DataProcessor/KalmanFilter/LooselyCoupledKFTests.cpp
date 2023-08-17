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
#include "Nodes/DataProcessor/Integrator/ImuIntegrator.hpp"
#undef protected
#undef private
#pragma GCC diagnostic pop

#include "NodeData/State/InertialNavSol.hpp"
#include "NodeData/State/LcKfInsGnssErrors.hpp"
#include "Nodes/DataLogger/Protocol/CommonLog.hpp"

namespace NAV::TESTS::LooselyCoupledKFTests
{

void testLCKFwithImuFile(const char* imuFilePath, size_t MESSAGE_COUNT_GNSS, size_t MESSAGE_COUNT_GNSS_FIX, size_t MESSAGE_COUNT_IMU, size_t MESSAGE_COUNT_IMU_FIX)
{
    auto logger = initializeTestLogger();

    bool imuAfter = std::string(imuFilePath) == "VectorNav/Static/vn310-imu-after.csv";

    std::array<std::vector<std::function<void()>>, 6> settings = { {
        { [&]() { LOG_WARN("Setting ImuIntegrator - _path to: {}", imuFilePath);
                  dynamic_cast<VectorNavFile*>(nm::FindNode(324))->_path = imuFilePath; } },
        { []() { LOG_WARN("Setting LooselyCoupledKF - _frame to: NED");
                 dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_frame = LooselyCoupledKF::Frame::NED; },
          []() { LOG_WARN("Setting LooselyCoupledKF - _frame to: ECEF");
                 dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_frame = LooselyCoupledKF::Frame::ECEF; } },
        { []() { LOG_WARN("Setting LooselyCoupledKF - _phiCalculationAlgorithm to: Taylor");
                 dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_phiCalculationAlgorithm = LooselyCoupledKF::PhiCalculationAlgorithm::Taylor; },
          []() { LOG_WARN("Setting LooselyCoupledKF - _phiCalculationAlgorithm to: Exponential");
                 dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_phiCalculationAlgorithm = LooselyCoupledKF::PhiCalculationAlgorithm::Exponential; } },
        { []() { LOG_WARN("Setting LooselyCoupledKF - _qCalculationAlgorithm to: Taylor1");
                 dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_qCalculationAlgorithm = LooselyCoupledKF::QCalculationAlgorithm::Taylor1; },
          []() { LOG_WARN("Setting LooselyCoupledKF - _qCalculationAlgorithm to: VanLoan");
                 dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_qCalculationAlgorithm = LooselyCoupledKF::QCalculationAlgorithm::VanLoan; } },
        {
            []() { LOG_WARN("Setting LooselyCoupledKF - _randomProcessAccel to: GaussMarkov1");
                 dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_randomProcessAccel = LooselyCoupledKF::RandomProcess::GaussMarkov1; },
            //   []() { LOG_WARN("Setting LooselyCoupledKF - _randomProcessAccel to: RandomWalk");
            //          dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_randomProcessAccel = LooselyCoupledKF::RandomProcess::RandomWalk; }
        },
        {
            []() { LOG_WARN("Setting LooselyCoupledKF - _randomProcessGyro to: GaussMarkov1");
                 dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_randomProcessGyro = LooselyCoupledKF::RandomProcess::GaussMarkov1; },
            //   []() { LOG_WARN("Setting LooselyCoupledKF - _randomProcessGyro to: RandomWalk");
            //          dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_randomProcessGyro = LooselyCoupledKF::RandomProcess::RandomWalk; }
        },
    } };

    cartesian_product_idx([&](size_t i0, size_t i1, size_t i2, size_t i3, size_t i4, size_t i5) {
        size_t messageCounter_VectorNavBinaryConverterImu_BinaryOutput = 0;
        size_t messageCounter_VectorNavBinaryConverterGnss_BinaryOutput = 0;
        size_t messageCounter_ImuIntegrator_ImuObs = 0;
        size_t messageCounter_ImuIntegrator_PosVelAttInit = 0;
        size_t messageCounter_ImuIntegrator_PVAError = 0;
        size_t messageCounter_ImuIntegrator_Sync = 0;
        size_t messageCounter_LooselyCoupledKF_InertialNavSol = 0;
        size_t messageCounter_LooselyCoupledKF_GNSSNavigationSolution = 0;

        auto timeOfFirstGnssObs = imuAfter ? InsTime() : InsTime(2, 185, 281201.999719);

        nm::RegisterPreInitCallback([&]() {
            settings[0][i0]();
            settings[1][i1]();
            settings[2][i2]();
            settings[3][i3]();
            settings[4][i4]();
            settings[5][i5]();
        });

        // VectorNavBinaryConverter (333) |> Binary Output (332)
        nm::RegisterWatcherCallbackToInputPin(332, [&](const Node* /* node */, const InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */) {
            messageCounter_VectorNavBinaryConverterImu_BinaryOutput++;
        });

        // VectorNavBinaryConverter (337) |> Binary Output (338)
        nm::RegisterWatcherCallbackToInputPin(338, [&](const Node* /* node */, const InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */) {
            messageCounter_VectorNavBinaryConverterGnss_BinaryOutput++;
        });

        // ImuIntegrator (163) |> ImuObs (164)
        nm::RegisterWatcherCallbackToInputPin(164, [&](const Node* /* node */, const InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */) {
            messageCounter_ImuIntegrator_ImuObs++;
        });

        // ImuIntegrator (163) |> PosVelAttInit (165)
        nm::RegisterWatcherCallbackToInputPin(165, [&](const Node* /* node */, const InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */) {
            messageCounter_ImuIntegrator_PosVelAttInit++;
        });

        // ImuIntegrator (163) |> PVAError (224)
        nm::RegisterWatcherCallbackToInputPin(224, [&](const Node* /* node */, const InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */) {
            messageCounter_ImuIntegrator_PVAError++;

            // TODO: Test PVA Error
            // auto obs = std::static_pointer_cast<LcKfInsGnssErrors>(queue.front());
        });

        // ImuIntegrator (163) |> Sync (6)
        nm::RegisterWatcherCallbackToInputPin(6, [&](const Node* /* node */, const InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */) {
            messageCounter_ImuIntegrator_Sync++;
        });

        // LooselyCoupledKF (239) |> InertialNavSol (226)
        nm::RegisterWatcherCallbackToInputPin(226, [&](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
            messageCounter_LooselyCoupledKF_InertialNavSol++;

            auto obs = std::static_pointer_cast<const InertialNavSol>(queue.front());
            // LOG_TRACE("InertialNavSol time = [{} - {}]", obs->insTime.toYMDHMS(), obs->insTime.toGPSweekTow());

            Eigen::Vector3d refPos_lla(deg2rad(48.780704498291016), deg2rad(9.171577453613281), 325.1);
            Eigen::Vector3d refRollPitchYaw(0.428, -5.278, -61.865);
            Eigen::Vector3d allowedPositionOffsetImuOnly_n(2.0, 5.2, 1.0);
            Eigen::Vector3d allowedPositionOffsetCombined_n(0.15, 0.1, 0.1);
            Eigen::Vector3d allowedVelocityErrorImuOnly_e(0.14, 13.7, 0.1);
            Eigen::Vector3d allowedVelocityErrorCombined_e(0.09, 0.05, 0.08);
            Eigen::Vector3d allowedRollPitchYawOffsetImuOnly(1.3, 1.3, 90.0);
            Eigen::Vector3d allowedRollPitchYawOffsetCombined(2.7, 2.0, 94.0);

            if (i1 == 1) // LooselyCoupledKF::Frame::ECEF
            {
                allowedRollPitchYawOffsetImuOnly = { 1.3, 2.8, 90.0 };
                allowedRollPitchYawOffsetCombined = { 2.7, 3.2, 94.0 };
            }

            if (imuAfter)
            {
                allowedPositionOffsetCombined_n = { 0.13, 0.05, 0.07 };
                allowedVelocityErrorCombined_e = { 0.088, 0.05, 0.08 };
                allowedRollPitchYawOffsetCombined = { 5, 5, 242.0 };
            }

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

        // LooselyCoupledKF (239) |> GNSSNavigationSolution (227)
        nm::RegisterWatcherCallbackToInputPin(227, [&](const Node* /* node */, const InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */) {
            messageCounter_LooselyCoupledKF_GNSSNavigationSolution++;
        });

        // ###########################################################################################################
        //                                           LooselyCoupledKF.flow
        // ###########################################################################################################
        //
        //                                                                                                                                                                                                    Plot (9)
        //                                                                    ImuSimulator (577) (disabled)                            (585)--------------------------------------------------------------->  |> ImuObs (4)
        //                                                                       (575) ImuObs |>                       (555)----------/-------------------------------------------------------------------->  |> Nominal (5)
        //                                                                    (576) PosVelAtt |>                      /              /                                                                   -->  |> Filter (121)
        //                                                                                       \                   /              /                                                                   /
        //                                                                                        \          Combiner (344)        /            PosVelAttInitializer (21)                               |
        // VectorNavFile - IMU (324)              VectorNavBinaryConverter (333)                   (581)-->  |> (345)    (347) |> -                      (20) PosVelAtt |>                              |
        //    (323) Binary Output |>  --(334)-->  |> Binary Output (332)   (331) ImuObsWDelta |> --(561)-->  |> (346)              \                                       \                          (383)
        //                                                                                                      /                   \                                      /                            |
        // VectorNavFile - GNSS (326)              VectorNavBinaryConverter (337)               /---------------                     \           --------(322)-------------                             |
        //     (325) Binary Output |>  --(334)-->  |> Binary Output (338)   (339) PosVelAtt |> -                                      \         /                                                       |
        //                                                                                      \                                      \       |     ImuIntegrator (163)                               /
        //                                                                                       \                                      (583)----->  |> ImuObs (164)          (166) InertialNavSol |> -
        //                                                                                        \                                            \-->  |> PosVelAttInit (165)                            \
        //                                                                                         \                                        ------>  |> PVAError (224)                                 |
        //                                                                                          \                                      /    -->  |> Sync (6)                                       |
        //                                                                                           \                                    |    /                                                       |
        //                                                                                            \                                   |    \-------------------------------------------------------|---------------------------------------------------------------(504)
        //                                                                                             \                                   \                                                           |                                                                    \
        //                                                                                              \                                   (240)------------------------------------------------------|----------------------------------------------------------------    |
        //                                                                                               \                                                                                             |                                                                \   |
        //                                                                                                \                                                                                            \          LooselyCoupledKF (239)                                /   |
        //                                                                                                 \                                                                                            (242)-->  |> InertialNavSol (226)            (228) PVAError |> ----/---              Plot (250)
        //                                                                                                  (557)---------------------------------------------------------------------------------------------->  |> GNSSNavigationSolution (227)        (441) Sync |> ----    \-(266)---->  |> PVAError (245)
        //                                                                                                                                                                                                                                                  (496) x |> ----------(505)---->  |> KF.x (252)
        //                                                                                                                                                                                                                                                  (497) P |> ----------(506)---->  |> KF.P (253)
        //                                                                                                                                                                                                                                                (498) Phi |> ----------(507)---->  |> KF.Phi (254)
        //                                                                                                                                                                                                                                                  (499) Q |> ----------(508)---->  |> KF.Q (255)
        //                                                                                                                                                                                                                                                  (500) z |> ----------(509)---->  |> KF.z (256)
        //                                                                                                                                                                                                                                                  (501) H |> ----------(510)---->  |> KF.H (263)
        //                                                                                                                                                                                                                                                  (502) R |> ----------(511)---->  |> KF.R (264)
        //                                                                                                                                                                                                                                                  (503) K |> ----------(512)---->  |> KF.K (456)
        //
        // ###########################################################################################################
        REQUIRE(testFlow("test/flow/Nodes/DataProcessor/KalmanFilter/LooselyCoupledKF.flow"));

        REQUIRE(messageCounter_VectorNavBinaryConverterImu_BinaryOutput == MESSAGE_COUNT_IMU);
        REQUIRE(messageCounter_VectorNavBinaryConverterGnss_BinaryOutput == MESSAGE_COUNT_GNSS);
        REQUIRE(messageCounter_ImuIntegrator_ImuObs == MESSAGE_COUNT_IMU_FIX);
        REQUIRE(messageCounter_ImuIntegrator_PosVelAttInit == 1);
        REQUIRE(messageCounter_ImuIntegrator_PVAError == MESSAGE_COUNT_GNSS_FIX);
        REQUIRE(messageCounter_ImuIntegrator_Sync == MESSAGE_COUNT_GNSS_FIX);
        REQUIRE(messageCounter_LooselyCoupledKF_InertialNavSol == MESSAGE_COUNT_IMU_FIX + MESSAGE_COUNT_GNSS_FIX - 1); // First GNSS message is used to initialize filter, does not update
        REQUIRE(messageCounter_LooselyCoupledKF_GNSSNavigationSolution == MESSAGE_COUNT_GNSS_FIX);
    },
                          settings);
}

TEST_CASE("[LooselyCoupledKF][flow] Test flow with IMU data arriving before GNSS data", "[LooselyCoupledKF][flow]")
{
    // GNSS: 176 messages, 162 messages with InsTime, 48 messages with fix (first GNSS message at 22.799s)
    size_t MESSAGE_COUNT_GNSS = 162;
    size_t MESSAGE_COUNT_GNSS_FIX = 48;
    // IMU:  690 messages, 466 messages with InsTime, 170 messages with fix (first IMU message at 9.037s)
    size_t MESSAGE_COUNT_IMU = 466;
    size_t MESSAGE_COUNT_IMU_FIX = 170;

    testLCKFwithImuFile("VectorNav/Static/vn310-imu.csv", MESSAGE_COUNT_GNSS, MESSAGE_COUNT_GNSS_FIX, MESSAGE_COUNT_IMU, MESSAGE_COUNT_IMU_FIX);
}

TEST_CASE("[LooselyCoupledKF][flow] Test flow with IMU data arriving after GNSS data", "[LooselyCoupledKF][flow]")
{
    // GNSS: 176 messages, 162 messages with InsTime, 48 messages with fix (first GNSS message at 22.799717387000001s)
    size_t MESSAGE_COUNT_GNSS = 162;
    size_t MESSAGE_COUNT_GNSS_FIX = 48;
    // IMU:  167 messages (first IMU message at 24.017697899000002s)
    size_t MESSAGE_COUNT_IMU = 167;

    testLCKFwithImuFile("VectorNav/Static/vn310-imu-after.csv", MESSAGE_COUNT_GNSS, MESSAGE_COUNT_GNSS_FIX, MESSAGE_COUNT_IMU, MESSAGE_COUNT_IMU);
}

} // namespace NAV::TESTS::LooselyCoupledKFTests
