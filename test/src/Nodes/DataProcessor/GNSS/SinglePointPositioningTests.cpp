// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SinglePointPositioningTests.cpp
/// @brief Tests for the SinglePointPositioning node
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-06-18

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"

#include <string>
#include <vector>
#include <fstream>
#include <algorithm>

#include "FlowTester.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "Logger.hpp"
#include "util/Container/STL.hpp"
#include "util/StringUtil.hpp"

#include "NodeData/GNSS/SppSolution.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "data/SkydelSatData.hpp"
#include "data/SpirentAsciiSatelliteData.hpp"

// This is a small hack, which lets us change private/protected parameters
#if defined(__clang__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "Nodes/DataProvider/GNSS/FileReader/RinexObsFile.hpp"
#include "Nodes/DataProvider/GNSS/FileReader/RinexNavFile.hpp"
#include "Nodes/DataProcessor/GNSS/SinglePointPositioning.hpp"
#undef protected
#undef private
#if defined(__clang__)
    #pragma GCC diagnostic pop
#endif

namespace NAV::TESTS::SinglePointPositioningTests
{

TEST_CASE("[SinglePointPositioning][flow] SPP with Skydel data (GPS L1 C/A - no Iono - no Tropo)", "[SinglePointPositioning][flow]")
{
    auto logger = initializeTestLogger();

    nm::RegisterPreInitCallback([&]() {
        dynamic_cast<RinexObsFile*>(nm::FindNode(65))->_path = "GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/SkydelRINEX_S_20230080000_04H_MO.rnx";
        dynamic_cast<RinexNavFile*>(nm::FindNode(54))->_path = "GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/SkydelRINEX_S_20238959_7200S_GN.rnx";

        auto* sppNode = dynamic_cast<SinglePointPositioning*>(nm::FindNode(91));

        sppNode->_algorithm._obsFilter._filterFreq = G01;
        sppNode->_algorithm._obsFilter._filterCode = Code::G1C;
        sppNode->_algorithm._obsFilter._elevationMask = 0;

        sppNode->_algorithm._obsEstimator._ionosphereModel = IonosphereModel::None;
        AtmosphereModels atmosphere{
            .pressureModel = PressureModel::ISA,
            .temperatureModel = TemperatureModel::ISA,
            .waterVaporModel = WaterVaporModel::ISA,
        };
        sppNode->_algorithm._obsEstimator._troposphereModels = TroposphereModelSelection{
            .zhdModel = std::make_pair(TroposphereModel::None, atmosphere),
            .zwdModel = std::make_pair(TroposphereModel::None, atmosphere),
            .zhdMappingFunction = std::make_pair(MappingFunction::Cosecant, atmosphere),
            .zwdMappingFunction = std::make_pair(MappingFunction::Cosecant, atmosphere),
        };
    });

    // ###########################################################################################################
    //                                           SinglePointPositioning.flow
    // ###########################################################################################################
    //
    // RinexObsFile (65)                          SinglePointPositioning (91)
    //         (64) PosVelAtt |>  --(92)-->  |> GnssObs (88)      (90) SppSolution |>  --(97)-->  |> (95) Terminator (96)
    //                              (93)-->  |> GnssNavInfo (89)
    // RinexNavFile() (54)         /
    //         (53) PosVelAtt <>  -
    //
    // ###########################################################################################################

    const Eigen::Vector3d e_refRecvPos{ -481819.31349724, 5507219.95375401, 3170373.73538364 }; // Receiver position simulated by Skydel
    Eigen::Vector3d lla_refRecvPos = trafo::ecef2lla_WGS84(e_refRecvPos);
    LOG_DEBUG("lla_refRecvPos {}, {}, {}", rad2deg(lla_refRecvPos.x()), rad2deg(lla_refRecvPos.y()), lla_refRecvPos.z());

    std::string folder = "test/data/GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/sat_data/";
    std::vector<SkydelReference> sppReference;
    sppReference.emplace_back(SatSigId(Code::G1C, 1), folder + "L1CA 01.csv");
    sppReference.emplace_back(SatSigId(Code::G1C, 3), folder + "L1CA 03.csv");
    sppReference.emplace_back(SatSigId(Code::G1C, 6), folder + "L1CA 06.csv");
    sppReference.emplace_back(SatSigId(Code::G1C, 7), folder + "L1CA 07.csv");
    sppReference.emplace_back(SatSigId(Code::G1C, 8), folder + "L1CA 08.csv");
    sppReference.emplace_back(SatSigId(Code::G1C, 9), folder + "L1CA 09.csv");
    sppReference.emplace_back(SatSigId(Code::G1C, 11), folder + "L1CA 11.csv");
    sppReference.emplace_back(SatSigId(Code::G1C, 13), folder + "L1CA 13.csv");
    sppReference.emplace_back(SatSigId(Code::G1C, 14), folder + "L1CA 14.csv");
    sppReference.emplace_back(SatSigId(Code::G1C, 17), folder + "L1CA 17.csv");
    sppReference.emplace_back(SatSigId(Code::G1C, 19), folder + "L1CA 19.csv");
    sppReference.emplace_back(SatSigId(Code::G1C, 21), folder + "L1CA 21.csv");
    sppReference.emplace_back(SatSigId(Code::G1C, 24), folder + "L1CA 24.csv");
    sppReference.emplace_back(SatSigId(Code::G1C, 30), folder + "L1CA 30.csv");

    size_t messageCounter = 0; // Message Counter
    nm::RegisterWatcherCallbackToInputPin(95, [&](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        auto sppSol = std::dynamic_pointer_cast<const NAV::SppSolution>(queue.front());

        LOG_DEBUG("    e_refRecvPos         {} [m]", e_refRecvPos.transpose());
        LOG_DEBUG("    sppSol->e_position() {} [m]", sppSol->e_position().transpose());
        Eigen::Vector3d n_diff = trafo::n_Quat_e(lla_refRecvPos.x(), lla_refRecvPos.y()) * (sppSol->e_position() - e_refRecvPos);
        double hDist = n_diff.head<2>().norm();
        double vDist = std::abs(n_diff.z());
        LOG_DEBUG("    hDist {} [m]", hDist);
        LOG_DEBUG("    vDist {} [m]", vDist);
        REQUIRE(hDist < 6e-4); // Determined by running the test and adapting
        REQUIRE(vDist < 2e-3); // Determined by running the test and adapting

        LOG_DEBUG("    Satellites (# {}):", sppSol->satData.size());
        for ([[maybe_unused]] const auto& data : sppSol->satData)
        {
            LOG_DEBUG("      {} ({}°)", data.first, rad2deg(data.second.satElevation));
        }

        std::set<SatSigId> signalsCompared;
        std::set<SatId> satellitesCompared;
        for (auto& ref : sppReference)
        {
            if (ref.counter == ref.refData.size())
            {
                auto iter = std::ranges::find_if(sppSol->satData, [&ref](const auto& satData) { return satData.first == ref.satSigId.toSatId(); });
                REQUIRE(iter == sppSol->satData.end());
                continue;
            }
            const auto& refData = ref.refData.at(ref.counter);
            REQUIRE(sppSol->insTime <= refData.recvTime);

            if (sppSol->insTime == refData.recvTime)
            {
                auto iter = std::ranges::find_if(sppSol->satData, [&ref](const auto& satData) {
                    return satData.first == ref.satSigId.toSatId();
                });
                REQUIRE(iter != sppSol->satData.end()); // This means something was calculated for the satellite
                ref.counter++;
                signalsCompared.insert(ref.satSigId);
                satellitesCompared.insert(ref.satSigId.toSatId());

                LOG_DEBUG("Checking {} line {}/{}. Elapsed time: {:.0f} ms", ref.satSigId, ref.counter, ref.refData.size(), refData.Elapsed_Time);
                const auto& satData = std::ranges::find_if(sppSol->satData, [&ref](const auto& data) {
                                          return data.first == ref.satSigId.toSatId();
                                      })->second;

                LOG_DEBUG("    satData.satElevation {} [deg]", rad2deg(satData.satElevation));
                LOG_DEBUG("    refSatElevation       {} [deg]", rad2deg(refData.Receiver_Antenna_Elevation));
                LOG_DEBUG("    satData.satElevation - refSatElevation {} [°]", rad2deg(satData.satElevation - refData.Receiver_Antenna_Elevation));
                REQUIRE_THAT(rad2deg(satData.satElevation - refData.Receiver_Antenna_Elevation), Catch::Matchers::WithinAbs(0.0, 1e-7)); // Determined by running the test and adapting

                LOG_DEBUG("    satData.satAzimuth {} [°]", rad2deg(satData.satAzimuth));
                LOG_DEBUG("    refSatAzimuth       {} [°]", rad2deg(refData.Receiver_Antenna_Azimuth));
                LOG_DEBUG("    satData.satAzimuth - refSatAzimuth {} [°]", rad2deg(satData.satAzimuth - refData.Receiver_Antenna_Azimuth));
                REQUIRE_THAT(rad2deg(satData.satAzimuth - refData.Receiver_Antenna_Azimuth), Catch::Matchers::WithinAbs(0.0, 1e-7)); // Determined by running the test and adapting
            }
        }
        REQUIRE(sppSol->nSatellites == sppSol->satData.size());
        LOG_DEBUG("Satellites compared: {}", joinToString(satellitesCompared));
        REQUIRE(sppSol->satData.size() == satellitesCompared.size());
        LOG_DEBUG("Signals compared: {}", joinToString(signalsCompared));
        REQUIRE(sppSol->nMeasPsr == signalsCompared.size());
        messageCounter++;
    });

    REQUIRE(testFlow("test/flow/Nodes/DataProcessor/GNSS/SinglePointPositioning.flow"));

    CHECK(messageCounter == 49);
    for (auto& ref : sppReference)
    {
        LOG_DEBUG("Checking if all messages from satellite {} were read.", ref.satSigId);
        REQUIRE(ref.counter == ref.refData.size());
    }
}

TEST_CASE("[SinglePointPositioning][flow] SPP with Spirent data (GPS L1 C/A - no Sat clk - no Iono - no Tropo)", "[SinglePointPositioning][flow]")
{
    auto logger = initializeTestLogger();

    Frequency filterFreq = G01;
    Code filterCode = Code::G1C;

    nm::RegisterPreInitCallback([&]() {
        dynamic_cast<RinexObsFile*>(nm::FindNode(65))->_path = "GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none/Spirent_RINEX_MO.obs";
        dynamic_cast<RinexNavFile*>(nm::FindNode(54))->_path = "GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Spirent_RINEX_GN.23N";

        auto* sppNode = dynamic_cast<SinglePointPositioning*>(nm::FindNode(91));

        sppNode->_algorithm._obsFilter._filterFreq = filterFreq;
        sppNode->_algorithm._obsFilter._filterCode = filterCode;
        sppNode->_algorithm._obsFilter._elevationMask = 0;

        sppNode->_algorithm._obsEstimator._ionosphereModel = IonosphereModel::None;
        AtmosphereModels atmosphere{
            .pressureModel = PressureModel::ISA,
            .temperatureModel = TemperatureModel::ISA,
            .waterVaporModel = WaterVaporModel::ISA,
        };
        sppNode->_algorithm._obsEstimator._troposphereModels = TroposphereModelSelection{
            .zhdModel = std::make_pair(TroposphereModel::None, atmosphere),
            .zwdModel = std::make_pair(TroposphereModel::None, atmosphere),
            .zhdMappingFunction = std::make_pair(MappingFunction::Cosecant, atmosphere),
            .zwdMappingFunction = std::make_pair(MappingFunction::Cosecant, atmosphere),
        };
    });

    // ###########################################################################################################
    //                                           SinglePointPositioning.flow
    // ###########################################################################################################
    //
    // RinexObsFile (65)                          SinglePointPositioning (91)
    //         (64) PosVelAtt |>  --(92)-->  |> GnssObs (88)      (90) SppSolution |>  --(97)-->  |> (95) Terminator (96)
    //                              (93)-->  |> GnssNavInfo (89)
    // RinexNavFile() (54)         /
    //         (53) PosVelAtt <>  -
    //
    // ###########################################################################################################

    const Eigen::Vector3d lla_refRecvPos = { deg2rad(30.0), deg2rad(95.0), 0.0 };
    const Eigen::Vector3d e_refRecvPos = trafo::lla2ecef_WGS84(lla_refRecvPos);

    SpirentSatDataFile spirentSatelliteData("test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none/sat_data_V1A1.csv");

    REQUIRE(spirentSatelliteData.refData.size() == 2114);

    size_t messageCounter = 0; // Message Counter
    nm::RegisterWatcherCallbackToInputPin(95, [&](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        messageCounter++;
        auto sppSol = std::dynamic_pointer_cast<const NAV::SppSolution>(queue.front());

        LOG_DEBUG("{} ({}):", sppSol->insTime.toYMDHMS(GPST), sppSol->insTime.toGPSweekTow());
        LOG_DEBUG("    e_refRecvPos - sppSol->e_position() {} [m]", (e_refRecvPos - sppSol->e_position()).transpose());
        Eigen::Vector3d n_diff = trafo::n_Quat_e(lla_refRecvPos.x(), lla_refRecvPos.y()) * (sppSol->e_position() - e_refRecvPos);
        double hDist = n_diff.head<2>().norm();
        double vDist = std::abs(n_diff.z());
        LOG_DEBUG("    hDist {} [m]", hDist);
        LOG_DEBUG("    vDist {} [m]", vDist);
        REQUIRE(hDist < 7.2e-4); // Determined by running the test and adapting
        REQUIRE(vDist < 1.2e-3); // Determined by running the test and adapting

        LOG_DEBUG("    Satellites (# {}):", sppSol->satData.size());
        for ([[maybe_unused]] const auto& data : sppSol->satData)
        {
            LOG_DEBUG("      {} ({}°)", data.first, rad2deg(data.second.satElevation));
        }

        for (const auto& [satId, sppSatData] : sppSol->satData)
        {
            auto ref = spirentSatelliteData.get(sppSol->insTime, satId);
            REQUIRE(ref.has_value());
            LOG_DEBUG("    {}:", satId);

            LOG_DEBUG("        spp.satElevation - refElevation = {}°", rad2deg(sppSatData.satElevation - ref->get().Elevation));
            REQUIRE_THAT(rad2deg(sppSatData.satElevation - ref->get().Elevation), Catch::Matchers::WithinAbs(0.0, 3.2e-3)); // Determined by running the test and adapting
            LOG_DEBUG("        spp.satAzimuth - refAzimuth = {}°", rad2deg(sppSatData.satAzimuth - ref->get().Azimuth));
            REQUIRE_THAT(rad2deg(sppSatData.satAzimuth - ref->get().Azimuth), Catch::Matchers::WithinAbs(0.0, 4.5e-3)); // Determined by running the test and adapting

            ref->get().checked = true;
        }
    });

    REQUIRE(testFlow("test/flow/Nodes/DataProcessor/GNSS/SinglePointPositioning.flow"));

    CHECK(messageCounter == 49);

    for (const auto& satData : spirentSatelliteData.refData)
    {
        if ((satData.satId.satSys & filterFreq.getSatSys()) != SatSys_None)
        {
            LOG_DEBUG("[{}][{}] Checking if ref data was used: {}", satData.recvTime.toYMDHMS(GPST), satData.satId, satData.checked);
            REQUIRE(satData.checked);
        }
    }
}

} // namespace NAV::TESTS::SinglePointPositioningTests
