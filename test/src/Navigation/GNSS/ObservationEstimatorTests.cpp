// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ObservationEstimatorTests.cpp
/// @brief Tests for the SinglePointPositioning node
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-02-04

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
#pragma GCC diagnostic push
#if defined(__clang__)
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "Nodes/DataProvider/GNSS/FileReader/RinexObsFile.hpp"
#include "Nodes/DataProvider/GNSS/FileReader/RinexNavFile.hpp"
#include "Nodes/DataProvider/GNSS/FileReader/RtklibPosFile.hpp"
#include "Nodes/DataProcessor/GNSS/SinglePointPositioning.hpp"
#undef protected
#undef private
#pragma GCC diagnostic pop

namespace NAV::TESTS::ObservationEstimatorTests
{

#if !__APPLE__ && !defined(WIN32) && !defined(_WIN32) && !defined(__WIN32)

TEST_CASE("[ObservationEstimator][flow] Check estimates with Skydel data (GPS L1 C/A - no Iono - no Tropo)", "[ObservationEstimator][flow]")
{
    auto logger = initializeTestLogger();

    nm::RegisterPreInitCallback([&]() {
        dynamic_cast<RinexObsFile*>(nm::FindNode(65))->_path = "GNSS/Orolia-Skydel_static_duration-4h_rate-5min_sys-GERCQIS_iono-none_tropo-none/SkydelRINEX_S_20230080000_04H_MO.rnx";
        dynamic_cast<RinexNavFile*>(nm::FindNode(54))->_path = "GNSS/Orolia-Skydel_static_duration-4h_rate-5min_sys-GERCQIS_iono-none_tropo-none/SkydelRINEX_S_20238959_7200S_GN.rnx";
        dynamic_cast<RtklibPosFile*>(nm::FindNode(80))->_path = "GNSS/Orolia-Skydel_static_duration-4h_rate-5min_sys-GERCQIS_iono-none_tropo-none/RTKLIB/SkydelRINEX_S_20230080000_04H_G.pos";

        dynamic_cast<SinglePointPositioning*>(nm::FindNode(91))->_algorithm._obsFilter._filterFreq = G01;
        dynamic_cast<SinglePointPositioning*>(nm::FindNode(91))->_algorithm._obsFilter._filterCode = Code::G1C;

        dynamic_cast<SinglePointPositioning*>(nm::FindNode(91))->_algorithm._obsEstimator._ionosphereModel = IonosphereModel::None;
        AtmosphereModels atmosphere{
            .pressureModel = PressureModel::ISA,
            .temperatureModel = TemperatureModel::ISA,
            .waterVaporModel = WaterVaporModel::ISA,
        };
        dynamic_cast<SinglePointPositioning*>(nm::FindNode(91))->_algorithm._obsEstimator._troposphereModels = TroposphereModelSelection{
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
    // RinexObsFile (65)                          SinglePointPositioning (91)                     Plot (77)
    //         (64) PosVelAtt |>  --(92)-->  |> GnssObs (88)      (90) SppSolution |>  --(94)-->  |> SPP (72)
    //                              (93)-->  |> GnssNavInfo (89)                         (81)-->  |> RTKLIB (76)
    // RinexNavFile() (54)         /                                                    /
    //         (53) PosVelAtt <>  -                                                    /
    //                                                         RtklibPosFile (80)     /
    //                                                         (79) RtklibPosObs |>  -
    //
    // ###########################################################################################################

    const Eigen::Vector3d e_refRecvPos{ -481819.31349724, 5507219.95375401, 3170373.73538364 }; // Receiver position simulated by Skydel
    Eigen::Vector3d lla_refRecvPos = trafo::ecef2lla_WGS84(e_refRecvPos);
    LOG_DEBUG("lla_refRecvPos {}, {}, {}", rad2deg(lla_refRecvPos.x()), rad2deg(lla_refRecvPos.y()), lla_refRecvPos.z());

    std::string folder = "test/data/GNSS/Orolia-Skydel_static_duration-4h_rate-5min_sys-GERCQIS_iono-none_tropo-none/sat_data/";
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
    nm::RegisterWatcherCallbackToInputPin(88, [&](const Node* node, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        const auto* spp = dynamic_cast<const NAV::SinglePointPositioning*>(node);
        auto gnssObs = std::static_pointer_cast<const GnssObs>(queue.front());
        // Collection of all connected navigation data providers
        std::vector<const GnssNavInfo*> gnssNavInfos;
        for (size_t i = 0; i < spp->_dynamicInputPins.getNumberOfDynamicPins(); i++)
        {
            if (const auto* gnssNavInfo = spp->getInputValue<const GnssNavInfo>(NAV::SinglePointPositioning::INPUT_PORT_INDEX_GNSS_NAV_INFO + i))
            {
                gnssNavInfos.push_back(gnssNavInfo);
            }
        }
        if (gnssNavInfos.empty()) { return; }
        // Collection of all connected Ionospheric Corrections
        IonosphericCorrections ionosphericCorrections(gnssNavInfos);

        std::string nameId = "SPP TEST";
        SPP::Algorithm algorithm = spp->_algorithm;
        algorithm._receiver[SPP::Algorithm::Rover].gnssObs = gnssObs;
        algorithm._receiver[SPP::Algorithm::Rover].e_pos = e_refRecvPos;
        algorithm._receiver[SPP::Algorithm::Rover].lla_pos = lla_refRecvPos;
        algorithm._receiver[SPP::Algorithm::Rover].e_vel.setZero();

        auto observations = algorithm._obsFilter.selectObservationsForCalculation(algorithm._receiver, gnssNavInfos, nameId, SPP::Algorithm::Rover, true);
        algorithm.updateInterSystemTimeDifferences(observations.systems, nameId);
        algorithm._obsEstimator.calcObservationEstimates(observations, algorithm._receiver, ionosphericCorrections, nameId, ObservationEstimator::NoDifference);

        for (auto& ref : sppReference)
        {
            if (ref.counter == ref.refData.size())
            {
                REQUIRE(!observations.signals.contains(ref.satSigId));
                continue;
            }
            const auto& refData = ref.refData.at(ref.counter);
            REQUIRE(gnssObs->insTime <= refData.recvTime);

            if (gnssObs->insTime == refData.recvTime)
            {
                REQUIRE(observations.signals.contains(ref.satSigId));
                ref.counter++;

                LOG_DEBUG("Checking {} observation line {}/{}. Elapsed time: {:.0f} ms", ref.satSigId, ref.counter, ref.refData.size(), refData.Elapsed_Time);
                const Observations::SignalObservation& sigObs = observations.signals.at(ref.satSigId);

                Eigen::Vector3d e_refSatPos(refData.ECEF_X, refData.ECEF_Y, refData.ECEF_Z);
                LOG_DEBUG("    satData.e_satPos {} [m]", sigObs.e_satPos().transpose());
                LOG_DEBUG("    e_refSatPos       {} [m]", e_refSatPos.transpose());
                LOG_DEBUG("      satData.pos - e_refPos   = {}", (sigObs.e_satPos() - e_refSatPos).transpose());
                LOG_DEBUG("    | satData.pos - e_refPos | = {} [m]", (sigObs.e_satPos() - e_refSatPos).norm());
                REQUIRE_THAT((sigObs.e_satPos() - e_refSatPos).norm(), Catch::Matchers::WithinAbs(0.0, 2e-4)); // Determined by running the test and adapting

                // e_satVel

                LOG_DEBUG("    satClkBias       {} [s]", sigObs.satClock().bias);
                LOG_DEBUG("    refClkCorrection {} [s]", refData.Clock_Correction);
                LOG_DEBUG("    clkBias - ref    {} [s]", sigObs.satClock().bias - refData.Clock_Correction);
                REQUIRE_THAT(sigObs.satClock().bias - refData.Clock_Correction, Catch::Matchers::WithinAbs(0.0, 4e-15)); // Determined by running the test and adapting

                // satClkDrift

                LOG_DEBUG("    satElevation          {} [deg]", rad2deg(sigObs.recvObs[SPP::Algorithm::Rover].satElevation()));
                LOG_DEBUG("    refSatElevation       {} [deg]", rad2deg(refData.Receiver_Antenna_Elevation));
                LOG_DEBUG("    satElevation - refSatElevation {} [°]", rad2deg(sigObs.recvObs[SPP::Algorithm::Rover].satElevation() - refData.Receiver_Antenna_Elevation));
                REQUIRE_THAT(rad2deg(sigObs.recvObs[SPP::Algorithm::Rover].satElevation() - refData.Receiver_Antenna_Elevation),
                             Catch::Matchers::WithinAbs(0.0, 1e-7)); // Determined by running the test and adapting

                LOG_DEBUG("    satAzimuth          {} [°]", rad2deg(sigObs.recvObs[SPP::Algorithm::Rover].satAzimuth()));
                LOG_DEBUG("    refSatAzimuth       {} [°]", rad2deg(refData.Receiver_Antenna_Azimuth));
                LOG_DEBUG("    satAzimuth - refSatAzimuth {} [°]", rad2deg(sigObs.recvObs[SPP::Algorithm::Rover].satAzimuth() - refData.Receiver_Antenna_Azimuth));
                REQUIRE_THAT(rad2deg(sigObs.recvObs[SPP::Algorithm::Rover].satAzimuth() - refData.Receiver_Antenna_Azimuth),
                             Catch::Matchers::WithinAbs(0.0, 1e-8)); // Determined by running the test and adapting

                LOG_DEBUG("    satData.dpsr_I   {} [m]", sigObs.recvObs[SPP::Algorithm::Rover].terms.dpsr_I_r_s);
                LOG_DEBUG("    refIonoCorrection {} [m]", refData.Iono_Correction);
                LOG_DEBUG("    satData.dpsr_I - refIonoCorrection = {} [m]", sigObs.recvObs[SPP::Algorithm::Rover].terms.dpsr_I_r_s - refData.Iono_Correction);
                REQUIRE_THAT(sigObs.recvObs[SPP::Algorithm::Rover].terms.dpsr_I_r_s - refData.Iono_Correction, Catch::Matchers::WithinAbs(0.0, 1e-16));

                LOG_DEBUG("    satData.dpsr_T    {} [m]", sigObs.recvObs[SPP::Algorithm::Rover].terms.dpsr_T_r_s);
                LOG_DEBUG("    refTropoCorrection {} [m]", refData.Tropo_Correction);
                LOG_DEBUG("    satData.dpsr_T - refTropoCorrection = {} [m]", sigObs.recvObs[SPP::Algorithm::Rover].terms.dpsr_T_r_s - refData.Tropo_Correction);
                REQUIRE_THAT(sigObs.recvObs[SPP::Algorithm::Rover].terms.dpsr_T_r_s - refData.Tropo_Correction, Catch::Matchers::WithinAbs(0.0, 1e-16));

                double timeDiffRange_ref = (refData.Elapsed_Time - refData.PSR_satellite_time) * 1e-3;                       // [s]
                double timeDiffRecvTrans = static_cast<double>((gnssObs->insTime - sigObs.satClock().transmitTime).count()); // [s]
                timeDiffRecvTrans += sigObs.recvObs[SPP::Algorithm::Rover].terms.dpsr_I_r_s / InsConst::C;
                timeDiffRecvTrans += sigObs.recvObs[SPP::Algorithm::Rover].terms.dpsr_T_r_s / InsConst::C;
                LOG_DEBUG("    timeDiffRecvTrans {} [s]", timeDiffRecvTrans);
                LOG_DEBUG("    timeDiffRange_ref {} [s]", timeDiffRange_ref);
                LOG_DEBUG("    timeDiffRecvTrans - timeDiffRange_ref {} [s]", timeDiffRecvTrans - timeDiffRange_ref);
                REQUIRE_THAT(timeDiffRecvTrans - timeDiffRange_ref, Catch::Matchers::WithinAbs(0.0, 7e-4)); // Determined by running the test and adapting

                LOG_DEBUG("    geometricDist {} [m]", sigObs.recvObs[SPP::Algorithm::Rover].terms.rho_r_s);
                LOG_DEBUG("    refGeometricDist       {} [m]", refData.Range);
                LOG_DEBUG("    geometricDist - refGeometricDist = {} [m]", sigObs.recvObs[SPP::Algorithm::Rover].terms.rho_r_s - refData.Range);
                REQUIRE_THAT(sigObs.recvObs[SPP::Algorithm::Rover].terms.rho_r_s + sigObs.recvObs[SPP::Algorithm::Rover].terms.dpsr_ie_r_s - refData.Range,
                             Catch::Matchers::WithinAbs(0.0, 2e-4)); // Determined by running the test and adapting
            }
        }
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

TEST_CASE("[ObservationEstimator][flow] Check estimates with Spirent data (GPS L1 C/A - no Sat clk - no Iono - no Tropo)", "[ObservationEstimator][flow]")
{
    auto logger = initializeTestLogger();

    Frequency filterFreq = G01;
    Code filterCode = Code::G1C;

    nm::RegisterPreInitCallback([&]() {
        dynamic_cast<RinexObsFile*>(nm::FindNode(65))->_path = "GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQ_iono-none_tropo-none/Spirent_RINEX_MO.obs";
        dynamic_cast<RinexNavFile*>(nm::FindNode(54))->_path = "GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQ_iono-none_tropo-none/Spirent_RINEX_GN.23N";
        dynamic_cast<RtklibPosFile*>(nm::FindNode(80))->_path = "GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQ_iono-none_tropo-none/RTKLIB/Spirent_RINEX_G.pos";

        dynamic_cast<SinglePointPositioning*>(nm::FindNode(91))->_algorithm._obsFilter._filterFreq = filterFreq;
        dynamic_cast<SinglePointPositioning*>(nm::FindNode(91))->_algorithm._obsFilter._filterCode = filterCode;

        dynamic_cast<SinglePointPositioning*>(nm::FindNode(91))->_algorithm._obsEstimator._ionosphereModel = IonosphereModel::None;
        AtmosphereModels atmosphere{
            .pressureModel = PressureModel::ISA,
            .temperatureModel = TemperatureModel::ISA,
            .waterVaporModel = WaterVaporModel::ISA,
        };
        dynamic_cast<SinglePointPositioning*>(nm::FindNode(91))->_algorithm._obsEstimator._troposphereModels = TroposphereModelSelection{
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
    // RinexObsFile (65)                          SinglePointPositioning (91)                     Plot (77)
    //         (64) PosVelAtt |>  --(92)-->  |> GnssObs (88)      (90) SppSolution |>  --(94)-->  |> SPP (72)
    //                              (93)-->  |> GnssNavInfo (89)                         (81)-->  |> RTKLIB (76)
    // RinexNavFile() (54)         /                                                    /
    //         (53) PosVelAtt <>  -                                                    /
    //                                                         RtklibPosFile (80)     /
    //                                                         (79) RtklibPosObs |>  -
    //
    // ###########################################################################################################

    const Eigen::Vector3d lla_refRecvPos = { deg2rad(30.0), deg2rad(95.0), 0.0 };
    const Eigen::Vector3d e_refRecvPos = trafo::lla2ecef_WGS84(lla_refRecvPos);

    SpirentSatDataFile spirentSatelliteData("test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQ_iono-none_tropo-none/sat_data_V1A1.csv");
    REQUIRE(spirentSatelliteData.refData.size() == 1917);

    size_t messageCounter = 0; // Message Counter
    nm::RegisterWatcherCallbackToInputPin(88, [&](const Node* node, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        messageCounter++;
        const auto* spp = dynamic_cast<const NAV::SinglePointPositioning*>(node);
        auto gnssObs = std::static_pointer_cast<const GnssObs>(queue.front());
        // Collection of all connected navigation data providers
        std::vector<const GnssNavInfo*> gnssNavInfos;
        for (size_t i = 0; i < spp->_dynamicInputPins.getNumberOfDynamicPins(); i++)
        {
            if (const auto* gnssNavInfo = spp->getInputValue<const GnssNavInfo>(NAV::SinglePointPositioning::INPUT_PORT_INDEX_GNSS_NAV_INFO + i))
            {
                gnssNavInfos.push_back(gnssNavInfo);
            }
        }
        if (gnssNavInfos.empty()) { return; }
        // Collection of all connected Ionospheric Corrections
        IonosphericCorrections ionosphericCorrections(gnssNavInfos);

        std::string nameId = "SPP TEST";
        SPP::Algorithm algorithm = spp->_algorithm;
        algorithm._receiver[SPP::Algorithm::Rover].gnssObs = gnssObs;
        algorithm._receiver[SPP::Algorithm::Rover].e_pos = e_refRecvPos;
        algorithm._receiver[SPP::Algorithm::Rover].lla_pos = lla_refRecvPos;
        algorithm._receiver[SPP::Algorithm::Rover].e_vel.setZero();

        auto observations = algorithm._obsFilter.selectObservationsForCalculation(algorithm._receiver, gnssNavInfos, nameId, SPP::Algorithm::Rover, true);
        algorithm.updateInterSystemTimeDifferences(observations.systems, nameId);
        algorithm._obsEstimator.calcObservationEstimates(observations, algorithm._receiver, ionosphericCorrections, nameId, ObservationEstimator::NoDifference);

        LOG_DEBUG("{}:", gnssObs->insTime.toYMDHMS(GPST));

        for (const auto& [satSigId, sigObs] : observations.signals) // TODO: Check if all references visited
        {
            auto ref = spirentSatelliteData.get(gnssObs->insTime, satSigId.toSatId());
            REQUIRE(ref.has_value());
            LOG_DEBUG("    {}:", satSigId.toSatId());

            LOG_DEBUG("        | pos - e_refPos | = {}", (sigObs.e_satPos() - ref->get().Sat_Pos).norm());
            REQUIRE_THAT((sigObs.e_satPos() - ref->get().Sat_Pos).norm(), Catch::Matchers::WithinAbs(0.0, 2.0e-4)); // Determined by running the test and adapting

            LOG_DEBUG("        | vel - e_refVel | = {}", (sigObs.e_satVel() - ref->get().Sat_Vel).norm());
            REQUIRE_THAT((sigObs.e_satVel() - ref->get().Sat_Vel).norm(), Catch::Matchers::WithinAbs(0.0, 9.2e-4)); // Determined by running the test and adapting

            LOG_DEBUG("        satElevation - refElevation = {}°", rad2deg(sigObs.recvObs[SPP::Algorithm::Rover].satElevation() - ref->get().Elevation));
            REQUIRE_THAT(rad2deg(sigObs.recvObs[SPP::Algorithm::Rover].satElevation() - ref->get().Elevation), Catch::Matchers::WithinAbs(0.0, 3.2e-3)); // Determined by running the test and adapting
            LOG_DEBUG("        satAzimuth - refAzimuth = {}°", rad2deg(sigObs.recvObs[SPP::Algorithm::Rover].satAzimuth() - ref->get().Azimuth));
            REQUIRE_THAT(rad2deg(sigObs.recvObs[SPP::Algorithm::Rover].satAzimuth() - ref->get().Azimuth), Catch::Matchers::WithinAbs(0.0, 4.5e-3)); // Determined by running the test and adapting

            LOG_DEBUG("            dpsr_I = {}", sigObs.recvObs[SPP::Algorithm::Rover].terms.dpsr_I_r_s);
            LOG_DEBUG("        ref.dpsr_I = {}", ref->get().Iono_delay_Group_A);
            REQUIRE_THAT(sigObs.recvObs[SPP::Algorithm::Rover].terms.dpsr_I_r_s - ref->get().Iono_delay_Group_A, Catch::Matchers::WithinAbs(0.0, 1e-16));
            LOG_DEBUG("            dpsr_T = {}", sigObs.recvObs[SPP::Algorithm::Rover].terms.dpsr_T_r_s);
            LOG_DEBUG("        ref.dpsr_T = {}", ref->get().Tropo_delay);
            REQUIRE_THAT(sigObs.recvObs[SPP::Algorithm::Rover].terms.dpsr_T_r_s - ref->get().Tropo_delay, Catch::Matchers::WithinAbs(0.0, 1e-16));

            LOG_DEBUG("        psrEst - refPsr = {}", sigObs.recvObs[SPP::Algorithm::Rover].obs.at(GnssObs::Pseudorange).estimate - ref->get().P_Range_Group_A);
            REQUIRE_THAT(sigObs.recvObs[SPP::Algorithm::Rover].obs.at(GnssObs::Pseudorange).estimate - ref->get().P_Range_Group_A,
                         Catch::Matchers::WithinAbs(0.0, 6.7e-4)); // Determined by running the test and adapting

            LOG_DEBUG("        geometricDist - refRange = {}", sigObs.recvObs[SPP::Algorithm::Rover].terms.rho_r_s + sigObs.recvObs[SPP::Algorithm::Rover].terms.dpsr_ie_r_s - ref->get().Range);
            REQUIRE_THAT(sigObs.recvObs[SPP::Algorithm::Rover].terms.rho_r_s + sigObs.recvObs[SPP::Algorithm::Rover].terms.dpsr_ie_r_s - ref->get().Range,
                         Catch::Matchers::WithinAbs(0.0, 1.9e-4)); // Determined by running the test and adapting

            ref->get().checked = true;
        }
    });

    REQUIRE(testFlow("test/flow/Nodes/DataProcessor/GNSS/SinglePointPositioning.flow"));

    CHECK(messageCounter == 49);

    for (const auto& satData : spirentSatelliteData.refData)
    {
        if ((satData.satId.satSys & filterFreq.getSatSys()) != SatSys_None)
        {
            LOG_DEBUG("[{}][{}] Checking if ref data was used", satData.recvTime.toYMDHMS(GPST), satData.satId);
            REQUIRE(satData.checked);
        }
    }
}

#endif

} // namespace NAV::TESTS::ObservationEstimatorTests
