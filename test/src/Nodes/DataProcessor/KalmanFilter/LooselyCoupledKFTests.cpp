// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <catch2/catch.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <deque>
#include <algorithm>
#include <atomic>
#include <mutex>

#include "FlowTester.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "util/Logger.hpp"
#include "util/Container/CartesianProduct.hpp"

// This is a small hack, which lets us change private/protected parameters for
#pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-w"
#if defined(__clang__) && !(defined(__GNUC__) || defined(__GNUG__))
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

namespace NAV::TEST::LooselyCoupledKFTests
{

TEST_CASE("[LooselyCoupledKF][flow] Test flow when IMU messages arrive after GNSS messages", "[LooselyCoupledKF][flow][debug]")
{
    Logger consoleSink;

    // ###########################################################################################################
    //                                           LooselyCoupledKF.flow
    // ###########################################################################################################
    //
    //                                                                                                                                                                                                                                Plot (9)
    //                                                                                                ImuSimulator (577) (disabled)                            (585)--------------------------------------------------------------->  |> ImuObs (4)
    //                                                                                                   (575) ImuObs |>                       (555)----------/-------------------------------------------------------------------->  |> Nominal (5)
    //                                                                                                (576) PosVelAtt |>                      /              /                                                                   -->  |> Filter (121)
    //                                                                                                                    \                  /              /                                                                   /
    //                                                                                                                     \         Combiner (344)        /            PosVelAttInitializer (21)                               |
    // VectorNavFile("VectorNav/Static/vn310-imu.csv") (324)              VectorNavBinaryConverter (333)                   (581)-->  |> (345)    (347) |>   --(585)-->  |> ImuObs (18)          (20) PosVelAtt |>               |
    //                                (323) Binary Output |>  --(334)-->  |> Binary Output (332)   (331) ImuObsWDelta |> --(561)-->  |> (346)              \    ----->  |> PosVelAttInit (19)                      \          (383)
    //                                                                                                                                   /                  \  /                                                   /            |
    // VectorNavFile("VectorNav/Static/vn310-gnss.csv") (326)              VectorNavBinaryConverter (337)                 /--------------                    \/          ---------------(322)----------------------             |
    //                                 (325) Binary Output |>  --(334)-->  |> Binary Output (338)   (339) ImuObsWDelta |> -------------------------------(556)\         /                                                       |
    //                                                                                                                    \                                    \       |     ImuIntegrator (163)                               /
    //                                                                                                                     \                                    (583)----->  |> ImuObs (164)          (166) InertialNavSol |> -
    //                                                                                                                      \                                          \-->  |> PosVelAttInit (165)                            \
    //                                                                                                                       \                                      ------>  |> PVAError (224)                                 |
    //                                                                                                                        \                                    /    -->  |> Sync (6)                                       |
    //                                                                                                                         \                                  |    /                                                       |
    //                                                                                                                          \                                 |    \-------------------------------------------------------|---------------------------------------------------------------(504)
    //                                                                                                                           \                                 \                                                           |                                                                    \
    //                                                                                                                            \                                 (240)------------------------------------------------------|----------------------------------------------------------------    |
    //                                                                                                                             \                                                                                           |                                                                \   |
    //                                                                                                                              \                                                                                          \          LooselyCoupledKF (239)                                /   |
    //                                                                                                                               \                                                                                          (242)-->  |> InertialNavSol (226)            (228) PVAError |> ----/---              Plot (250)
    //                                                                                                                                (557)-------------------------------------------------------------------------------------------->  |> GNSSNavigationSolution (227)        (441) Sync |> ----    \-(266)---->  |> PVAError (245)
    //                                                                                                                                                                                                                                                                              (496) x |> ----------(505)---->  |> KF.x (252)
    //                                                                                                                                                                                                                                                                              (497) P |> ----------(506)---->  |> KF.P (253)
    //                                                                                                                                                                                                                                                                            (498) Phi |> ----------(507)---->  |> KF.Phi (254)
    //                                                                                                                                                                                                                                                                              (499) Q |> ----------(508)---->  |> KF.Q (255)
    //                                                                                                                                                                                                                                                                              (500) z |> ----------(509)---->  |> KF.z (256)
    //                                                                                                                                                                                                                                                                              (501) H |> ----------(510)---->  |> KF.H (263)
    //                                                                                                                                                                                                                                                                              (502) R |> ----------(511)---->  |> KF.R (264)
    //                                                                                                                                                                                                                                                                              (503) K |> ----------(512)---->  |> KF.K (456)
    //
    // ###########################################################################################################

    std::array<std::vector<void (*)()>, 9> settings = { {
        { []() { dynamic_cast<VectorNavFile*>(nm::FindNode(324))->_path = "VectorNav/Static/vn310-imu-after.csv"; },
          []() { dynamic_cast<VectorNavFile*>(nm::FindNode(324))->_path = "VectorNav/Static/vn310-imu-after.csv"; } },
        { []() { dynamic_cast<ImuIntegrator*>(nm::FindNode(163))->_integrationFrame = ImuIntegrator::IntegrationFrame::NED; },
          []() { dynamic_cast<ImuIntegrator*>(nm::FindNode(163))->_integrationFrame = ImuIntegrator::IntegrationFrame::ECEF; } },
        { []() { dynamic_cast<ImuIntegrator*>(nm::FindNode(163))->_integrationAlgorithm = IntegrationAlgorithm::Heun; },
          []() { dynamic_cast<ImuIntegrator*>(nm::FindNode(163))->_integrationAlgorithm = IntegrationAlgorithm::RungeKutta1; },
          []() { dynamic_cast<ImuIntegrator*>(nm::FindNode(163))->_integrationAlgorithm = IntegrationAlgorithm::RungeKutta2; },
          []() { dynamic_cast<ImuIntegrator*>(nm::FindNode(163))->_integrationAlgorithm = IntegrationAlgorithm::RungeKutta3; },
          []() { dynamic_cast<ImuIntegrator*>(nm::FindNode(163))->_integrationAlgorithm = IntegrationAlgorithm::RungeKutta4; } },
        { []() { dynamic_cast<ImuIntegrator*>(nm::FindNode(163))->_gravitationModel = GravitationModel::Somigliana; },
          []() { dynamic_cast<ImuIntegrator*>(nm::FindNode(163))->_gravitationModel = GravitationModel::EGM96; } },
        { []() { dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_frame = LooselyCoupledKF::Frame::NED; },
          []() { dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_frame = LooselyCoupledKF::Frame::ECEF; } },
        { []() { dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_phiCalculationAlgorithm = LooselyCoupledKF::PhiCalculationAlgorithm::Taylor; },
          []() { dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_phiCalculationAlgorithm = LooselyCoupledKF::PhiCalculationAlgorithm::Exponential; } },
        { []() { dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_qCalculationAlgorithm = LooselyCoupledKF::QCalculationAlgorithm::Taylor1; },
          []() { dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_qCalculationAlgorithm = LooselyCoupledKF::QCalculationAlgorithm::VanLoan; } },
        { []() { dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_randomProcessAccel = LooselyCoupledKF::RandomProcess::GaussMarkov1; },
          []() { dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_randomProcessAccel = LooselyCoupledKF::RandomProcess::RandomWalk; } },
        { []() { dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_randomProcessGyro = LooselyCoupledKF::RandomProcess::GaussMarkov1; },
          []() { dynamic_cast<LooselyCoupledKF*>(nm::FindNode(239))->_randomProcessGyro = LooselyCoupledKF::RandomProcess::RandomWalk; } },
    } };
    std::vector<size_t> indices(settings.size(), 0);

    cartesian_product([&](void (*c1)(), void (*c2)(), void (*c3)(), void (*c4)(), void (*c5)(), void (*c6)(), void (*c7)(), void (*c8)(), void (*c9)()) {
        size_t messageCounter_VectorNavBinaryConverterImu_BinaryOutput = 0;
        size_t messageCounter_VectorNavBinaryConverterGnss_BinaryOutput = 0;
        size_t messageCounter_ImuIntegrator_ImuObs = 0;
        size_t messageCounter_ImuIntegrator_PosVelAttInit = 0;
        size_t messageCounter_ImuIntegrator_PVAError = 0;
        size_t messageCounter_ImuIntegrator_Sync = 0;
        size_t messageCounter_LooselyCoupledKF_InertialNavSol = 0;
        size_t messageCounter_LooselyCoupledKF_GNSSNavigationSolution = 0;

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
        });

        // ImuIntegrator (163) |> Sync (6)
        nm::RegisterWatcherCallbackToInputPin(6, [&](const Node* /* node */, const InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */) {
            messageCounter_ImuIntegrator_Sync++;
        });

        // LooselyCoupledKF (239) |> InertialNavSol (226)
        nm::RegisterWatcherCallbackToInputPin(226, [&](const Node* /* node */, const InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */) {
            messageCounter_LooselyCoupledKF_InertialNavSol++;
        });

        // LooselyCoupledKF (239) |> GNSSNavigationSolution (227)
        nm::RegisterWatcherCallbackToInputPin(227, [&](const Node* /* node */, const InputPin::NodeDataQueue& /* queue */, size_t /* pinIdx */) {
            messageCounter_LooselyCoupledKF_GNSSNavigationSolution++;
        });

        nm::RegisterPreInitCallback([&]() {
            c1();
            c2();
            c3();
            c4();
            c5();
            c6();
            c7();
            c8();
            c9();
        });

        REQUIRE(testFlow("test/flow/Nodes/DataProcessor/KalmanFilter/LooselyCoupledKF.flow"));

        // GNSS: 176 messages, 162 messages with InsTime, 48 messages with fix (first 22.799717387000001s)
        // IMU:  167 messages, 167 messages after GNSS fix (first IMU message at 24.017697899000002s)
        constexpr size_t MESSAGE_COUNT_GNSS = 162;
        constexpr size_t MESSAGE_COUNT_GNSS_FIX = 48;
        constexpr size_t MESSAGE_COUNT_IMU = 167;

        REQUIRE(messageCounter_VectorNavBinaryConverterImu_BinaryOutput == MESSAGE_COUNT_IMU);
        REQUIRE(messageCounter_VectorNavBinaryConverterGnss_BinaryOutput == MESSAGE_COUNT_GNSS);
        REQUIRE(messageCounter_ImuIntegrator_ImuObs == MESSAGE_COUNT_IMU);
        REQUIRE(messageCounter_ImuIntegrator_PosVelAttInit == 1);
        REQUIRE(messageCounter_ImuIntegrator_PVAError == MESSAGE_COUNT_GNSS_FIX);
        REQUIRE(messageCounter_ImuIntegrator_Sync == MESSAGE_COUNT_GNSS_FIX);
        REQUIRE(messageCounter_LooselyCoupledKF_InertialNavSol == 214); // TODO: 214 - 167 = 47, so one GNSS message does not trigger? find out which
        REQUIRE(messageCounter_LooselyCoupledKF_GNSSNavigationSolution == MESSAGE_COUNT_GNSS_FIX);
    },
                      settings);
    // nm::RegisterPreInitCallback([&settings, &indices]() {
    //     for (size_t i = 0; i < settings.size(); i++)
    //     {
    //         settings.at(i).at(indices.at(i))();
    //     }
    //     for (int i = static_cast<int>(indices.size()); i >= 0; i--)
    //     {
    //         if (indices[static_cast<size_t>(i)] < settings[static_cast<size_t>(i)].size() - 1)
    //         {
    //             indices[static_cast<size_t>(i)]++;
    //             break;
    //         }
    //         // indices[static_cast<size_t>(i)] == settings[static_cast<size_t>(i)].size() - 1
    //         indices[static_cast<size_t>(i)] = 0;
    //     }
    // });

    // REQUIRE(testFlow("test/flow/Nodes/DataProcessor/KalmanFilter/LooselyCoupledKF.flow"));
    // GNSS: 176 messages, 162 messages with InsTime, 48 messages with fix (first 22.799717387000001s)
    // IMU:  167 messages, 167 messages after GNSS fix (first IMU message at 24.017697899000002s)

    // constexpr size_t MESSAGE_COUNT_GNSS = 162;
    // constexpr size_t MESSAGE_COUNT_GNSS_FIX = 48;
    // constexpr size_t MESSAGE_COUNT_IMU = 167;

    // CHECK(messageCounter_VectorNavBinaryConverterImu_BinaryOutput == MESSAGE_COUNT_IMU);
    // CHECK(messageCounter_VectorNavBinaryConverterGnss_BinaryOutput == MESSAGE_COUNT_GNSS);
    // CHECK(messageCounter_ImuIntegrator_ImuObs == MESSAGE_COUNT_IMU);
    // CHECK(messageCounter_ImuIntegrator_PosVelAttInit == 1);
    // CHECK(messageCounter_ImuIntegrator_PVAError == MESSAGE_COUNT_GNSS_FIX);
    // CHECK(messageCounter_ImuIntegrator_Sync == MESSAGE_COUNT_GNSS_FIX);
    // CHECK(messageCounter_LooselyCoupledKF_InertialNavSol == 214); // TODO: 214 - 167 = 47, so one GNSS message does not trigger? find out which
    // CHECK(messageCounter_LooselyCoupledKF_GNSSNavigationSolution == MESSAGE_COUNT_GNSS_FIX);

    // TODO: REQUIRE(testFlow("test/flow/Nodes/DataProcessor/KalmanFilter/LooselyCoupledKF-imu-before-gnss.flow"));
}

} // namespace NAV::TEST::LooselyCoupledKFTests
