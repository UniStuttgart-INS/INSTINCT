// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file TightlyCoupledKFTests.cpp
/// @brief Tests for the TightlyCoupledKF node
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2023-06-26

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
#include "Nodes/DataProcessor/KalmanFilter/TightlyCoupledKF.hpp"
#include "Nodes/DataProcessor/Integrator/ImuIntegrator.hpp"
#undef protected
#undef private
#pragma GCC diagnostic pop

#include "NodeData/State/InertialNavSol.hpp"
#include "NodeData/State/LcKfInsGnssErrors.hpp"
#include "Nodes/DataLogger/Protocol/CommonLog.hpp"

namespace NAV::TESTS::TightlyCoupledKFTests
{

void testTCKFwithImuFile(const char* imuFilePath, const char* gnssFilePath, size_t MESSAGE_COUNT_GNSS, size_t MESSAGE_COUNT_IMU)
{
    auto logger = initializeTestLogger();

    bool imuAfter = std::string(imuFilePath) == "DataProcessor/tckf/vn310-imu-after.csv";
    bool gnssRinex = std::string(gnssFilePath) == "DataProcessor/tckf/reach-m2-01_raw_202306291111.23O" || std::string(gnssFilePath) == "DataProcessor/tckf/reach-m2-01_raw_202306291111_noDoppler.23O" || std::string(gnssFilePath) == "DataProcessor/tckf/reach-m2-01_raw_202306291111_psrGaps.23O";

    std::array<std::vector<std::function<void()>>, 6> settings = { {
        { [&]() { LOG_WARN("Setting ImuIntegrator - _path to: {}", imuFilePath);
                  dynamic_cast<VectorNavFile*>(nm::FindNode(324))->_path = imuFilePath; } },
        {
            []() { LOG_WARN("Setting TightlyCoupledKF - _frame to: NED");
                 dynamic_cast<TightlyCoupledKF*>(nm::FindNode(591))->_frame = TightlyCoupledKF::Frame::NED; },
            //   []() { LOG_WARN("Setting TightlyCoupledKF - _frame to: ECEF");
            //          dynamic_cast<TightlyCoupledKF*>(nm::FindNode(591))->_frame = TightlyCoupledKF::Frame::ECEF; }
        }, // TODO: enable ECEF option once this is implemented in the TCKF
        { []() { LOG_WARN("Setting TightlyCoupledKF - _phiCalculationAlgorithm to: Taylor");
                 dynamic_cast<TightlyCoupledKF*>(nm::FindNode(591))->_phiCalculationAlgorithm = TightlyCoupledKF::PhiCalculationAlgorithm::Taylor; },
          []() { LOG_WARN("Setting TightlyCoupledKF - _phiCalculationAlgorithm to: Exponential");
                 dynamic_cast<TightlyCoupledKF*>(nm::FindNode(591))->_phiCalculationAlgorithm = TightlyCoupledKF::PhiCalculationAlgorithm::Exponential; } },
        { []() { LOG_WARN("Setting TightlyCoupledKF - _qCalculationAlgorithm to: Taylor1");
                 dynamic_cast<TightlyCoupledKF*>(nm::FindNode(591))->_qCalculationAlgorithm = TightlyCoupledKF::QCalculationAlgorithm::Taylor1; },
          []() { LOG_WARN("Setting TightlyCoupledKF - _qCalculationAlgorithm to: VanLoan");
                 dynamic_cast<TightlyCoupledKF*>(nm::FindNode(591))->_qCalculationAlgorithm = TightlyCoupledKF::QCalculationAlgorithm::VanLoan; } },
        { []() { LOG_WARN("Setting TightlyCoupledKF - _randomProcessAccel to: GaussMarkov1");
                 dynamic_cast<TightlyCoupledKF*>(nm::FindNode(591))->_randomProcessAccel = TightlyCoupledKF::RandomProcess::GaussMarkov1; },
          []() { LOG_WARN("Setting TightlyCoupledKF - _randomProcessAccel to: RandomWalk");
                 dynamic_cast<TightlyCoupledKF*>(nm::FindNode(591))->_randomProcessAccel = TightlyCoupledKF::RandomProcess::RandomWalk; } },
        { []() { LOG_WARN("Setting TightlyCoupledKF - _randomProcessGyro to: GaussMarkov1");
                 dynamic_cast<TightlyCoupledKF*>(nm::FindNode(591))->_randomProcessGyro = TightlyCoupledKF::RandomProcess::GaussMarkov1; },
          []() { LOG_WARN("Setting TightlyCoupledKF - _randomProcessGyro to: RandomWalk");
                 dynamic_cast<TightlyCoupledKF*>(nm::FindNode(591))->_randomProcessGyro = TightlyCoupledKF::RandomProcess::RandomWalk; } },
    } };

    cartesian_product_idx([&](size_t i0, size_t i1, size_t i2, size_t i3, size_t i4, size_t i5) {
        size_t messageCounter_VectorNavBinaryConverterImu_BinaryOutput = 0;
        size_t messageCounter_VectorNavBinaryConverterGnss_BinaryOutput = 0;
        size_t messageCounter_ImuIntegrator_ImuObs = 0;
        size_t messageCounter_ImuIntegrator_PosVelAttInit = 0;
        size_t messageCounter_ImuIntegrator_PVAError = 0;
        size_t messageCounter_ImuIntegrator_Sync = 0;
        size_t messageCounter_TightlyCoupledKF_InertialNavSol = 0;
        size_t messageCounter_TightlyCoupledKF_GnssObs = 0;

        auto timeOfFirstGnssObs = imuAfter ? InsTime() : InsTime(2, 220, 385980.783684);

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

        if (!gnssRinex)
        {
            // VectorNavBinaryConverter (624) |> Binary Output (623)
            nm::RegisterWatcherCallbackToInputPin(623, [&](const Node* /* node */, const InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */) {
                messageCounter_VectorNavBinaryConverterGnss_BinaryOutput++;
            });
        }

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

        // TightlyCoupledKF (591) |> InertialNavSol (586)
        nm::RegisterWatcherCallbackToInputPin(586, [&](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
            messageCounter_TightlyCoupledKF_InertialNavSol++;

            auto obs = std::static_pointer_cast<const InertialNavSol>(queue.front());
            // LOG_TRACE("InertialNavSol time = [{} - {}]", obs->insTime.toYMDHMS(), obs->insTime.toGPSweekTow());

            Eigen::Vector3d refPos_lla(deg2rad(48.780660038), deg2rad(9.171496838), 329.2047);
            Eigen::Vector3d refRollPitchYaw(0., 0., 0.);
            Eigen::Vector3d allowedPositionOffsetImuOnly_n(0., 0., 0.);
            Eigen::Vector3d allowedPositionOffsetCombined_n(0., 0., 0.);
            Eigen::Vector3d allowedVelocityErrorImuOnly_e(0., 0., 0.);
            Eigen::Vector3d allowedVelocityErrorCombined_e(0., 0., 0.);
            Eigen::Vector3d allowedRollPitchYawOffsetImuOnly(0., 0., 0.);
            Eigen::Vector3d allowedRollPitchYawOffsetCombined(0., 0., 0.);

            if (gnssRinex)
            {
                allowedPositionOffsetImuOnly_n << 5.4, 2.3, 35.;
                allowedPositionOffsetCombined_n << 14., 5., 120.;
                allowedVelocityErrorImuOnly_e << 0.14, 13.7, 0.1;
                allowedVelocityErrorCombined_e << 3., 2., 35.;
                allowedRollPitchYawOffsetImuOnly << 1.3, 1.3, 90.0;
                allowedRollPitchYawOffsetCombined << 11., 5., 90.;
            }
            else
            {
                allowedPositionOffsetImuOnly_n << 2.0, 5.2, 1.0;
                allowedPositionOffsetCombined_n << 8.55, 3., 51.;
                allowedVelocityErrorImuOnly_e << 0.14, 13.7, 0.1;
                allowedVelocityErrorCombined_e << 0.3, 0.05, 0.3;
                allowedRollPitchYawOffsetImuOnly << 1.3, 1.3, 90.0;
                allowedRollPitchYawOffsetCombined << 2.7, 12., 94.0;
            }

            // if (i1 == 1) // TightlyCoupledKF::Frame::ECEF // TODO: enable ECEF option once this is implemented in the TCKF
            // {
            //     allowedRollPitchYawOffsetImuOnly = { 1.3, 2.8, 90.0 };
            //     allowedRollPitchYawOffsetCombined = { 2.7, 3.2, 94.0 };
            // }

            if (imuAfter)
            {
                allowedRollPitchYawOffsetCombined = { 9., 5., 94. };
            }

            // North/South deviation [m]
            [[maybe_unused]] double northSouth = calcGeographicalDistance(obs->latitude(), obs->longitude(),
                                                                          refPos_lla.x(), obs->longitude());

            // East/West deviation [m]
            [[maybe_unused]] double eastWest = calcGeographicalDistance(obs->latitude(), obs->longitude(),
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

        // TightlyCoupledKF (591) |> GnssObs (587)
        nm::RegisterWatcherCallbackToInputPin(587, [&](const Node* /* node */, const InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */) {
            messageCounter_TightlyCoupledKF_GnssObs++;
        });

        // ###########################################################################################################
        //                                           TightlyCoupledKF.flow
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
        //     (325) Binary Output |>  --(340)-->  |> Binary Output (338)   (339) PosVelAtt |> -                                      \         /                                                       |
        //                           \                                                                                                 \       |     ImuIntegrator (163)                               /
        //                            \            VectorNavBinaryConverter (624)                                                       (583)----->  |> ImuObs (164)          (166) InertialNavSol |> -
        //                             \--(625)--> |> Binary Output (623)   (622) GnssObs |> ------                                            \-->  |> PosVelAttInit (165)                            \
        //                                                                                         \                                        ------>  |> PVAError (224)                                 |
        //                                                                                          \                                      /    -->  |> Sync (6)                                       |
        //                                                                                           \                                    |    /                                                       |
        //                                                                                            \                                   |    \-------------------------------------------------------|---------------------------------------------------------------(601)
        //                                                                                             \                                   \                                                           |                                                                    \
        //                                                                                              \                                   (592)------------------------------------------------------|----------------------------------------------------------------    |
        //                                                                                               \                                                                                             |                                                                \   |
        //                                                                                                \                                                                                            \          TightlyCoupledKF (591)                                /   |
        //                                                                                                 \                                                                                            (611)-->  |> InertialNavSol (586)            (589) PVAError |> ----/---              Plot (250)
        // RinexNavFile         (616)                                                                       (626)---------------------------------------------------------------------------------------------->  |> GnssObs (587)                       (590) Sync |> ----    \-(610)---->  |> PVAError (245)
        //     (615) GnssNavInfo   |>-----------------------------------------------------------------------(617)---------------------------------------------------------------------------------------------->  |> GnssNavInfo (588)                      (593) x |> ----------(632)---->  |> KF.x (252)
        //                                                                                                                                                                                                                                                  (594) P |> ----------(633)---->  |> KF.P (253)
        //                                                                                                                                                                                                                                                (595) Phi |> ----------(634)---->  |> KF.Phi (254)
        //                                                                                                                                                                                                                                                  (596) Q |> ----------(635)---->  |> KF.Q (255)
        //                                                                                                                                                                                                                                                  (597) z |> ----------(636)---->  |> KF.z (256)
        //                                                                                                                                                                                                                                                  (598) H |> ----------(637)---->  |> KF.H (263)
        //                                                                                                                                                                                                                                                  (599) R |> ----------(638)---->  |> KF.R (264)
        //                                                                                                                                                                                                                                                  (600) K |> ----------(639)---->  |> KF.K (456)
        //
        // ###########################################################################################################

        // ###########################################################################################################
        //                                           TightlyCoupledKF_Rinex.flow
        // ###########################################################################################################
        //
        //                                                                                                                                                                                                    Plot (9)
        //                                                                    ImuSimulator (577) (disabled)                            (585)--------------------------------------------------------------->  |> ImuObs (4)
        //                                                                       (575) ImuObs |>                                      /                                                                       |> Nominal (5)
        //                                                                    (576) PosVelAtt |>                                     /                                                                   -->  |> Filter (121)
        //                                                                                       \                                  /                                                                   /
        //                                                                                        \          Combiner (344)        /            PosVelAttInitializer (21)                               |
        // VectorNavFile - IMU (324)              VectorNavBinaryConverter (333)                   (581)-->  |> (345)    (347) |> -                      (20) PosVelAtt |>                              |
        //    (323) Binary Output |>  --(334)-->  |> Binary Output (332)   (331) ImuObsWDelta |> --(561)-->  |> (346)              \                                       \                          (383)
        //                                                                                                                          \                                      /                            |
        //                                                                                                                           \           --------(322)-------------                             |
        //                                                                                                                            \         /                                                       |
        //                                                                                                                             \       |     ImuIntegrator (163)                               /
        //                                                                                                                              (583)----->  |> ImuObs (164)          (166) InertialNavSol |> -
        //                                                                                                                                     \-->  |> PosVelAttInit (165)                            \
        //                                                                                                                                  ------>  |> PVAError (224)                                 |
        //                                                                                                                                 /    -->  |> Sync (6)                                       |
        //                                                                                                                                |    /                                                       |
        //                                                                                                                                |    \-------------------------------------------------------|---------------------------------------------------------------(601)
        //                                                                                                                                 \                                                           |                                                                    \
        // RinexObsFile   (633)                                                                                                             (592)------------------------------------------------------|----------------------------------------------------------------    |
        //     (632) RinexObsFile  |>--------------------------------------------------------------------                                                                                              |                                                                \   |
        //                                                                                               \                                                                                             \          TightlyCoupledKF (591)                                /   |
        //                                                                                                \                                                                                             (611)-->  |> InertialNavSol (586)            (589) PVAError |> ----/---              Plot (250)
        // RinexNavFile         (616)                                                                      \(632)---------------------------------------------------------------------------------------------->  |> GnssObs (587)                       (590) Sync |> ----    \-(610)---->  |> PVAError (245)
        //     (615) GnssNavInfo   |>-----------------------------------------------------------------------(617)---------------------------------------------------------------------------------------------->  |> GnssNavInfo (588)                      (593) x |> ----------(632)---->  |> KF.x (252)
        //                                                                                                                                                                                                                                                  (594) P |> ----------(633)---->  |> KF.P (253)
        //                                                                                                                                                                                                                                                (595) Phi |> ----------(634)---->  |> KF.Phi (254)
        //                                                                                                                                                                                                                                                  (596) Q |> ----------(635)---->  |> KF.Q (255)
        //                                                                                                                                                                                                                                                  (597) z |> ----------(636)---->  |> KF.z (256)
        //                                                                                                                                                                                                                                                  (598) H |> ----------(637)---->  |> KF.H (263)
        //                                                                                                                                                                                                                                                  (599) R |> ----------(638)---->  |> KF.R (264)
        //                                                                                                                                                                                                                                                  (600) K |> ----------(639)---->  |> KF.K (456)
        //
        // ###########################################################################################################
        if (gnssRinex)
        {
            REQUIRE(testFlow("test/flow/Nodes/DataProcessor/KalmanFilter/TightlyCoupledKF_Rinex.flow"));
        }
        else
        {
            REQUIRE(testFlow("test/flow/Nodes/DataProcessor/KalmanFilter/TightlyCoupledKF.flow"));
            REQUIRE(messageCounter_VectorNavBinaryConverterGnss_BinaryOutput == MESSAGE_COUNT_GNSS);
        }

        REQUIRE(messageCounter_VectorNavBinaryConverterImu_BinaryOutput == MESSAGE_COUNT_IMU);
        REQUIRE(messageCounter_ImuIntegrator_ImuObs == MESSAGE_COUNT_IMU);
        REQUIRE(messageCounter_ImuIntegrator_PosVelAttInit == 1);
        REQUIRE(messageCounter_ImuIntegrator_PVAError == MESSAGE_COUNT_GNSS);
        REQUIRE(messageCounter_ImuIntegrator_Sync == MESSAGE_COUNT_GNSS);
        REQUIRE(messageCounter_TightlyCoupledKF_InertialNavSol == MESSAGE_COUNT_IMU + MESSAGE_COUNT_GNSS - 1); // First GNSS message is used to initialize filter, does not update
        REQUIRE(messageCounter_TightlyCoupledKF_GnssObs == MESSAGE_COUNT_GNSS);
    },
                          settings);
}

TEST_CASE("[TightlyCoupledKF][flow] Test flow with IMU data arriving before GNSS data", "[TightlyCoupledKF][flow]")
{
    // GNSS: 166 messages, 166 messages with InsTime (first GNSS message at 0.004999876 s)
    size_t MESSAGE_COUNT_GNSS = 166;
    // IMU:  1638 messages, 1638 messages with InsTime (first IMU message at 0 s)
    size_t MESSAGE_COUNT_IMU = 1638;

    testTCKFwithImuFile("DataProcessor/tckf/vn310-imu.csv", "DataProcessor/tckf/vn310-gnss.csv", MESSAGE_COUNT_GNSS, MESSAGE_COUNT_IMU);
}

TEST_CASE("[TightlyCoupledKF][flow] Test flow with IMU data arriving after GNSS data", "[TightlyCoupledKF][flow]")
{
    // GNSS: 166 messages, 166 messages with InsTime (first GNSS message at 0.004999876 s)
    size_t MESSAGE_COUNT_GNSS = 166;
    // IMU:  1636 messages, 1636 messages with InsTime (first IMU message at 0.039999962 s)
    size_t MESSAGE_COUNT_IMU = 1636;

    testTCKFwithImuFile("DataProcessor/tckf/vn310-imu-after.csv", "DataProcessor/tckf/vn310-gnss.csv", MESSAGE_COUNT_GNSS, MESSAGE_COUNT_IMU);
}

TEST_CASE("[TightlyCoupledKF][flow] Test flow with GNSS containing only psr, no doppler", "[TightlyCoupledKF][flow]")
{
    // GNSS: 610 messages, 610 messages with InsTime (first GNSS message at 0.004999876 s)
    size_t MESSAGE_COUNT_GNSS = 610;
    // IMU:  1638 messages, 1638 messages with InsTime (first IMU message at 0.039999962 s)
    size_t MESSAGE_COUNT_IMU = 1638;

    testTCKFwithImuFile("DataProcessor/tckf/vn310-imu.csv", "DataProcessor/tckf/reach-m2-01_raw_202306291111_noDoppler.23O", MESSAGE_COUNT_GNSS, MESSAGE_COUNT_IMU);
}

TEST_CASE("[TightlyCoupledKF][flow] Test flow with GNSS containing only doppler (no psr) in some epochs", "[TightlyCoupledKF][flow]")
{
    // GNSS: 166 messages, 166 messages with InsTime (first GNSS message at 0.004999876 s)
    size_t MESSAGE_COUNT_GNSS = 610;
    // IMU:  1638 messages, 1638 messages with InsTime (first IMU message at 0.039999962 s)
    size_t MESSAGE_COUNT_IMU = 1638;

    testTCKFwithImuFile("DataProcessor/tckf/vn310-imu.csv", "DataProcessor/tckf/reach-m2-01_raw_202306291111_psrGaps.23O", MESSAGE_COUNT_GNSS, MESSAGE_COUNT_IMU);
}

} // namespace NAV::TESTS::TightlyCoupledKFTests
