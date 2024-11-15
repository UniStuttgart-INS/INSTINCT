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

namespace NAV::TESTS::ObservationEstimatorTests
{

#if !__APPLE__ && !defined(WIN32) && !defined(_WIN32) && !defined(__WIN32)
namespace
{

void testSkydelData(Frequency filterFreq, Code filterCode, IonosphereModel ionoModel, TroposphereModelSelection tropoModel, double elevationMaskDeg,
                    const Eigen::Vector3d& lla_refRecvPos,
                    const std::string& rinexObsFile, const std::string& rinexNavFile, std::vector<SkydelReference> sppReference,
                    size_t obsCount, const std::unordered_map<Frequency, SkydelReference::Margin>& margins)
{
    auto logger = initializeTestLogger();

    nm::RegisterPreInitCallback([&]() {
        dynamic_cast<RinexObsFile*>(nm::FindNode(65))->_path = rinexObsFile;
        dynamic_cast<RinexNavFile*>(nm::FindNode(54))->_path = rinexNavFile;

        auto* sppNode = dynamic_cast<SinglePointPositioning*>(nm::FindNode(91));

        sppNode->_algorithm._obsFilter._filterFreq = filterFreq;
        sppNode->_algorithm._obsFilter._filterCode = filterCode;
        sppNode->_algorithm._obsFilter._elevationMask = deg2rad(elevationMaskDeg);

        sppNode->_algorithm._obsEstimator._ionosphereModel = ionoModel;
        sppNode->_algorithm._obsEstimator._troposphereModels = tropoModel;
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

    const Eigen::Vector3d e_refRecvPos = trafo::lla2ecef_WGS84(lla_refRecvPos);
    LOG_DEBUG("lla_refRecvPos {}, {}, {}", rad2deg(lla_refRecvPos.x()), rad2deg(lla_refRecvPos.y()), lla_refRecvPos.z());

    std::unordered_map<Frequency, SkydelReference::Margin> marginsMax{};

    size_t messageCounter = 0; // Message Counter
    nm::RegisterWatcherCallbackToInputPin(88, [&](const Node* node, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        const auto* spp = dynamic_cast<const NAV::SinglePointPositioning*>(node);
        auto gnssObs = std::static_pointer_cast<const GnssObs>(queue.front());
        // Collection of all connected navigation data providers
        std::vector<InputPin::IncomingLink::ValueWrapper<GnssNavInfo>> gnssNavInfoWrappers;
        std::vector<const GnssNavInfo*> gnssNavInfos;
        for (size_t i = 0; i < spp->_dynamicInputPins.getNumberOfDynamicPins(); i++)
        {
            if (auto gnssNavInfo = spp->getInputValue<GnssNavInfo>(NAV::SinglePointPositioning::INPUT_PORT_INDEX_GNSS_NAV_INFO + i))
            {
                gnssNavInfoWrappers.push_back(*gnssNavInfo);
                gnssNavInfos.push_back(gnssNavInfo->v);
            }
        }
        if (gnssNavInfos.empty()) { return; }
        // Collection of all connected Ionospheric Corrections
        auto ionosphericCorrections = std::make_shared<const IonosphericCorrections>(gnssNavInfos);

        std::string nameId = "SPP TEST";
        SPP::Algorithm algorithm = spp->_algorithm;
        algorithm._receiver.gnssObs = gnssObs;
        algorithm._receiver.e_posMarker = e_refRecvPos;
        algorithm._receiver.lla_posMarker = lla_refRecvPos;
        algorithm._receiver.e_vel.setZero();

        Observations observations;
        algorithm._obsFilter.selectObservationsForCalculation(algorithm._receiver.type,
                                                              algorithm._receiver.e_posMarker,
                                                              algorithm._receiver.lla_posMarker,
                                                              algorithm._receiver.gnssObs,
                                                              gnssNavInfos, observations, nullptr, nameId, false);
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
                LOG_DEBUG("Checking {} observation line {}/{}. Elapsed time: {:.0f} ms", ref.satSigId, ref.counter, ref.refData.size(), refData.Elapsed_Time);
                REQUIRE(observations.signals.contains(ref.satSigId));
                ref.counter++;

                auto marginIter = std::ranges::find_if(margins, [&](const auto& m) { return m.first & ref.satSigId.freq(); });
                REQUIRE(marginIter != margins.end());
                const auto& margin = marginIter->second;
                auto& marginMax = marginsMax[ref.satSigId.freq()];

                const Observations::SignalObservation& sigObs = observations.signals.at(ref.satSigId);

                Eigen::Vector3d e_refSatPos(refData.ECEF_X, refData.ECEF_Y, refData.ECEF_Z);
                LOG_DEBUG("    satData.e_satPos {} [m]", sigObs.recvObs.at(SPP::Algorithm::Rover)->e_satPos().transpose());
                LOG_DEBUG("    e_refSatPos       {} [m]", e_refSatPos.transpose());
                LOG_DEBUG("      satData.pos - e_refPos   = {}", (sigObs.recvObs.at(SPP::Algorithm::Rover)->e_satPos() - e_refSatPos).transpose());
                LOG_DEBUG("    | satData.pos - e_refPos | = {:.4e} [m]", (sigObs.recvObs.at(SPP::Algorithm::Rover)->e_satPos() - e_refSatPos).norm());
                CHECK_THAT((sigObs.recvObs.at(SPP::Algorithm::Rover)->e_satPos() - e_refSatPos).norm(), Catch::Matchers::WithinAbs(0.0, margin.pos));
                marginMax.pos = std::max(marginMax.pos, (sigObs.recvObs.at(SPP::Algorithm::Rover)->e_satPos() - e_refSatPos).norm());

                // e_satVel

                LOG_DEBUG("    satClkBias       {:.7e} [s]", sigObs.recvObs.at(SPP::Algorithm::Rover)->satClock().bias);
                LOG_DEBUG("    refClkCorrection {:.7e} [s]", refData.Clock_Correction);
                LOG_DEBUG("    clkBias - ref    {:.4e} [s]", sigObs.recvObs.at(SPP::Algorithm::Rover)->satClock().bias - refData.Clock_Correction);
                CHECK_THAT(sigObs.recvObs.at(SPP::Algorithm::Rover)->satClock().bias - refData.Clock_Correction, Catch::Matchers::WithinAbs(0.0, margin.clock));
                marginMax.clock = std::max(marginMax.clock, std::abs(sigObs.recvObs.at(SPP::Algorithm::Rover)->satClock().bias - refData.Clock_Correction));

                // satClkDrift

                LOG_DEBUG("    satElevation          {} [deg]", rad2deg(sigObs.recvObs.at(SPP::Algorithm::Rover)->satElevation(e_refRecvPos, lla_refRecvPos)));
                LOG_DEBUG("    refSatElevation       {} [deg]", rad2deg(refData.Receiver_Antenna_Elevation));
                LOG_DEBUG("    satElevation - refSatElevation {:.4e} [°]", rad2deg(sigObs.recvObs.at(SPP::Algorithm::Rover)->satElevation(e_refRecvPos, lla_refRecvPos) - refData.Receiver_Antenna_Elevation));
                CHECK_THAT(rad2deg(sigObs.recvObs.at(SPP::Algorithm::Rover)->satElevation(e_refRecvPos, lla_refRecvPos) - refData.Receiver_Antenna_Elevation),
                           Catch::Matchers::WithinAbs(0.0, margin.satElevation));
                marginMax.satElevation = std::max(marginMax.satElevation, std::abs(rad2deg(sigObs.recvObs.at(SPP::Algorithm::Rover)->satElevation(e_refRecvPos, lla_refRecvPos) - refData.Receiver_Antenna_Elevation)));

                LOG_DEBUG("    satAzimuth          {} [°]", rad2deg(sigObs.recvObs.at(SPP::Algorithm::Rover)->satAzimuth(e_refRecvPos, lla_refRecvPos)));
                LOG_DEBUG("    refSatAzimuth       {} [°]", rad2deg(refData.Receiver_Antenna_Azimuth));
                LOG_DEBUG("    satAzimuth - refSatAzimuth {:.4e} [°]", rad2deg(sigObs.recvObs.at(SPP::Algorithm::Rover)->satAzimuth(e_refRecvPos, lla_refRecvPos) - refData.Receiver_Antenna_Azimuth));
                CHECK_THAT(rad2deg(sigObs.recvObs.at(SPP::Algorithm::Rover)->satAzimuth(e_refRecvPos, lla_refRecvPos) - refData.Receiver_Antenna_Azimuth),
                           Catch::Matchers::WithinAbs(0.0, margin.satAzimuth));
                marginMax.satAzimuth = std::max(marginMax.satAzimuth, std::abs(rad2deg(sigObs.recvObs.at(SPP::Algorithm::Rover)->satAzimuth(e_refRecvPos, lla_refRecvPos) - refData.Receiver_Antenna_Azimuth)));

                LOG_DEBUG("    satData.dpsr_I   {:.4f} [m]", sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_I_r_s);
                LOG_DEBUG("    refIonoCorrection {:.4f} [m]", refData.Iono_Correction);
                LOG_DEBUG("    satData.dpsr_I - refIonoCorrection = {:.4e} [m]", sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_I_r_s - refData.Iono_Correction);
                CHECK_THAT(sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_I_r_s - refData.Iono_Correction, Catch::Matchers::WithinAbs(0.0, margin.dpsr_I));
                marginMax.dpsr_I = std::max(marginMax.dpsr_I, std::abs(sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_I_r_s - refData.Iono_Correction));

                LOG_DEBUG("    satData.dpsr_T    {:.4f} [m]", sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_T_r_s);
                LOG_DEBUG("    refTropoCorrection {:.4f} [m]", refData.Tropo_Correction);
                LOG_DEBUG("    satData.dpsr_T - refTropoCorrection = {:.4e} [m]", sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_T_r_s - refData.Tropo_Correction);
                CHECK_THAT(sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_T_r_s - refData.Tropo_Correction, Catch::Matchers::WithinAbs(0.0, margin.dpsr_T));
                marginMax.dpsr_T = std::max(marginMax.dpsr_T, std::abs(sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_T_r_s - refData.Tropo_Correction));

                double timeDiffRange_ref = (refData.Elapsed_Time - refData.PSR_satellite_time) * 1e-3;                                                          // [s]
                double timeDiffRecvTrans = static_cast<double>((gnssObs->insTime - sigObs.recvObs.at(SPP::Algorithm::Rover)->satClock().transmitTime).count()); // [s]
                timeDiffRecvTrans += sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_I_r_s / InsConst::C;
                timeDiffRecvTrans += sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_T_r_s / InsConst::C;
                LOG_DEBUG("    timeDiffRecvTrans {:.4f} [s]", timeDiffRecvTrans);
                LOG_DEBUG("    timeDiffRange_ref {:.4f} [s]", timeDiffRange_ref);
                LOG_DEBUG("    timeDiffRecvTrans - timeDiffRange_ref {:.4e} [s]", timeDiffRecvTrans - timeDiffRange_ref);
                CHECK_THAT(timeDiffRecvTrans - timeDiffRange_ref, Catch::Matchers::WithinAbs(0.0, margin.timeDiffRecvTrans));
                marginMax.timeDiffRecvTrans = std::max(marginMax.timeDiffRecvTrans, std::abs(timeDiffRecvTrans - timeDiffRange_ref));

                LOG_DEBUG("    geometricDist {:.4f} + dpsr_ie_r_s {:.4f} = {:.4f} [m]", sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.rho_r_s,
                          sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_ie_r_s,
                          sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.rho_r_s + sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_ie_r_s);
                LOG_DEBUG("    refGeometricDist       {:.4f} [m]", refData.Range);
                LOG_DEBUG("    geometricDist - refGeometricDist = {:.4e} [m]",
                          sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.rho_r_s + sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_ie_r_s - refData.Range);
                CHECK_THAT(sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.rho_r_s + sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_ie_r_s - refData.Range,
                           Catch::Matchers::WithinAbs(0.0, margin.geometricDist));
                marginMax.geometricDist = std::max(marginMax.geometricDist, std::abs(sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.rho_r_s + sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_ie_r_s - refData.Range));
            }
        }
        messageCounter++;
    });

    REQUIRE(testFlow("test/flow/Nodes/DataProcessor/GNSS/SinglePointPositioning.flow"));

    for ([[maybe_unused]] const auto& [freq, margin] : marginsMax)
    {
        LOG_DEBUG("[{}] margin maxima", freq);
        LOG_DEBUG("  clock = {:e}", margin.clock);
        LOG_DEBUG("  pos = {:e}", margin.pos);
        LOG_DEBUG("  satElevation = {:e}", margin.satElevation);
        LOG_DEBUG("  satAzimuth = {:e}", margin.satAzimuth);
        LOG_DEBUG("  dpsr_I = {:e}", margin.dpsr_I);
        LOG_DEBUG("  dpsr_T = {:e}", margin.dpsr_T);
        LOG_DEBUG("  timeDiffRecvTrans = {:e}", margin.timeDiffRecvTrans);
        LOG_DEBUG("  geometricDist = {:e}", margin.geometricDist);
    }

    CHECK(messageCounter == obsCount);
    for (auto& ref : sppReference)
    {
        LOG_DEBUG("Checking if all messages from satellite {} were read.", ref.satSigId);
        REQUIRE(ref.counter == ref.refData.size());
    }
}

void testSpirentData(Frequency filterFreq, Code filterCode, IonosphereModel ionoModel, TroposphereModelSelection tropoModel, double elevationMaskDeg,
                     const Eigen::Vector3d& lla_refRecvPos,
                     const std::string& rinexObsFile, const std::string& rinexNavFile, const std::string& spirentSatDataFile,
                     size_t refDataSize, size_t obsCount, const std::unordered_map<Frequency, SpirentSatDataFile::Margin>& margins)
{
    auto logger = initializeTestLogger();

    nm::RegisterPreInitCallback([&]() {
        dynamic_cast<RinexObsFile*>(nm::FindNode(65))->_path = rinexObsFile;
        dynamic_cast<RinexNavFile*>(nm::FindNode(54))->_path = rinexNavFile;

        auto* sppNode = dynamic_cast<SinglePointPositioning*>(nm::FindNode(91));

        sppNode->_algorithm._obsFilter._filterFreq = filterFreq;
        sppNode->_algorithm._obsFilter._filterCode = filterCode;
        sppNode->_algorithm._obsFilter._elevationMask = deg2rad(elevationMaskDeg);

        sppNode->_algorithm._obsEstimator._ionosphereModel = ionoModel;
        sppNode->_algorithm._obsEstimator._troposphereModels = tropoModel;
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

    const Eigen::Vector3d e_refRecvPos = trafo::lla2ecef_WGS84(lla_refRecvPos);

    SpirentSatDataFile spirentSatelliteData(spirentSatDataFile);
    REQUIRE(spirentSatelliteData.refData.size() == refDataSize);

    std::unordered_map<Frequency, SpirentSatDataFile::Margin> marginsMax{};

    size_t messageCounter = 0; // Message Counter
    nm::RegisterWatcherCallbackToInputPin(88, [&](const Node* node, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        messageCounter++;
        const auto* spp = dynamic_cast<const NAV::SinglePointPositioning*>(node);
        auto gnssObs = std::static_pointer_cast<const GnssObs>(queue.front());
        // Collection of all connected navigation data providers
        std::vector<InputPin::IncomingLink::ValueWrapper<GnssNavInfo>> gnssNavInfoWrappers;
        std::vector<const GnssNavInfo*> gnssNavInfos;
        for (size_t i = 0; i < spp->_dynamicInputPins.getNumberOfDynamicPins(); i++)
        {
            if (auto gnssNavInfo = spp->getInputValue<GnssNavInfo>(NAV::SinglePointPositioning::INPUT_PORT_INDEX_GNSS_NAV_INFO + i))
            {
                gnssNavInfoWrappers.push_back(*gnssNavInfo);
                gnssNavInfos.push_back(gnssNavInfo->v);
            }
        }
        if (gnssNavInfos.empty()) { return; }
        // Collection of all connected Ionospheric Corrections
        auto ionosphericCorrections = std::make_shared<const IonosphericCorrections>(gnssNavInfos);

        std::string nameId = "ObservationEstimator TEST";
        SPP::Algorithm algorithm = spp->_algorithm;
        algorithm._receiver.gnssObs = gnssObs;
        algorithm._receiver.e_posMarker = e_refRecvPos;
        algorithm._receiver.lla_posMarker = lla_refRecvPos;
        algorithm._receiver.e_vel.setZero();

        Observations observations;
        algorithm._obsFilter.selectObservationsForCalculation(algorithm._receiver.type,
                                                              algorithm._receiver.e_posMarker,
                                                              algorithm._receiver.lla_posMarker,
                                                              algorithm._receiver.gnssObs,
                                                              gnssNavInfos, observations, nullptr, nameId, false);
        algorithm._obsEstimator.calcObservationEstimates(observations, algorithm._receiver, ionosphericCorrections, nameId, ObservationEstimator::NoDifference);

        LOG_DEBUG("{}:", gnssObs->insTime.toYMDHMS(GPST));

        for (const auto& [satSigId, sigObs] : observations.signals) // TODO: Check if all references visited
        {
            auto ref = spirentSatelliteData.get(gnssObs->insTime, satSigId.toSatId());
            REQUIRE(ref.has_value());
            LOG_DEBUG("    {}:", satSigId);

            auto marginIter = std::ranges::find_if(margins, [&satSigId = satSigId](const auto& m) { return m.first & satSigId.freq(); });
            REQUIRE(marginIter != margins.end());
            const auto& margin = marginIter->second;
            auto& marginMax = marginsMax[satSigId.freq()];

            LOG_DEBUG("      satData.e_satPos {} [m]", sigObs.recvObs.at(SPP::Algorithm::Rover)->e_satPos().transpose());
            LOG_DEBUG("        e_refSatPos    {} [m]", ref->get().Sat_Pos.transpose());
            LOG_DEBUG("          satData.pos - e_refPos   = {}", (sigObs.recvObs.at(SPP::Algorithm::Rover)->e_satPos() - ref->get().Sat_Pos).transpose());
            LOG_DEBUG("        | satData.pos - e_refPos | = {:.4e} [m]", (sigObs.recvObs.at(SPP::Algorithm::Rover)->e_satPos() - ref->get().Sat_Pos).norm());
            CHECK_THAT((sigObs.recvObs.at(SPP::Algorithm::Rover)->e_satPos() - ref->get().Sat_Pos).norm(), Catch::Matchers::WithinAbs(0.0, margin.pos));
            marginMax.pos = std::max(marginMax.pos, (sigObs.recvObs.at(SPP::Algorithm::Rover)->e_satPos() - ref->get().Sat_Pos).norm());

            LOG_DEBUG("      satData.e_satVel {} [m]", sigObs.recvObs.at(SPP::Algorithm::Rover)->e_satVel().transpose());
            LOG_DEBUG("        e_refSatVel    {} [m]", ref->get().Sat_Vel.transpose());
            LOG_DEBUG("          satData.vel - e_refVel   = {}", (sigObs.recvObs.at(SPP::Algorithm::Rover)->e_satVel() - ref->get().Sat_Vel).transpose());
            LOG_DEBUG("        | satData.vel - e_refVel | = {:.4e} [m]", (sigObs.recvObs.at(SPP::Algorithm::Rover)->e_satVel() - ref->get().Sat_Vel).norm());
            CHECK_THAT((sigObs.recvObs.at(SPP::Algorithm::Rover)->e_satVel() - ref->get().Sat_Vel).norm(), Catch::Matchers::WithinAbs(0.0, margin.vel));
            marginMax.vel = std::max(marginMax.vel, (sigObs.recvObs.at(SPP::Algorithm::Rover)->e_satVel() - ref->get().Sat_Vel).norm());

            LOG_DEBUG("      satElevation      {} [deg]", rad2deg(sigObs.recvObs.at(SPP::Algorithm::Rover)->satElevation(e_refRecvPos, lla_refRecvPos)));
            LOG_DEBUG("        refSatElevation {} [deg]", rad2deg(ref->get().Elevation));
            LOG_DEBUG("        satElevation - refSatElevation {:.4e} [°]", rad2deg(sigObs.recvObs.at(SPP::Algorithm::Rover)->satElevation(e_refRecvPos, lla_refRecvPos) - ref->get().Elevation));
            CHECK_THAT(rad2deg(sigObs.recvObs.at(SPP::Algorithm::Rover)->satElevation(e_refRecvPos, lla_refRecvPos) - ref->get().Elevation), Catch::Matchers::WithinAbs(0.0, margin.satElevation));
            marginMax.satElevation = std::max(marginMax.satElevation, std::abs(rad2deg(sigObs.recvObs.at(SPP::Algorithm::Rover)->satElevation(e_refRecvPos, lla_refRecvPos) - ref->get().Elevation)));

            LOG_DEBUG("      satAzimuth      {} [°]", rad2deg(sigObs.recvObs.at(SPP::Algorithm::Rover)->satAzimuth(e_refRecvPos, lla_refRecvPos)));
            LOG_DEBUG("        refSatAzimuth {} [°]", rad2deg(ref->get().Azimuth));
            LOG_DEBUG("        satAzimuth - refSatAzimuth {:.4e} [°]", rad2deg(sigObs.recvObs.at(SPP::Algorithm::Rover)->satAzimuth(e_refRecvPos, lla_refRecvPos) - ref->get().Azimuth));
            CHECK_THAT(rad2deg(sigObs.recvObs.at(SPP::Algorithm::Rover)->satAzimuth(e_refRecvPos, lla_refRecvPos) - ref->get().Azimuth), Catch::Matchers::WithinAbs(0.0, margin.satAzimuth));
            marginMax.satAzimuth = std::max(marginMax.satAzimuth, std::abs(rad2deg(sigObs.recvObs.at(SPP::Algorithm::Rover)->satAzimuth(e_refRecvPos, lla_refRecvPos) - ref->get().Azimuth)));

            double refIono_delay = ref->get().getIono_delay(satSigId.freq());
            LOG_DEBUG("      satData.dpsr_I      {:.4e} [m]", sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_I_r_s);
            LOG_DEBUG("        refIonoCorrection {:.4e} [m]", refIono_delay);
            LOG_DEBUG("        satData.dpsr_I - refIonoCorrection = {:.4e} [m]", sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_I_r_s - refIono_delay);
            CHECK_THAT(sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_I_r_s - refIono_delay, Catch::Matchers::WithinAbs(0.0, margin.dpsr_I));
            marginMax.dpsr_I = std::max(marginMax.dpsr_I, std::abs(sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_I_r_s - refIono_delay));

            LOG_DEBUG("      satData.dpsr_T       {:.4e} [m]", sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_T_r_s);
            LOG_DEBUG("        refTropoCorrection {:.4e} [m]", ref->get().Tropo_delay);
            LOG_DEBUG("        satData.dpsr_T - refTropoCorrection = {:.4e} [m]", sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_T_r_s - ref->get().Tropo_delay);
            CHECK_THAT(sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_T_r_s - ref->get().Tropo_delay, Catch::Matchers::WithinAbs(0.0, margin.dpsr_T));
            marginMax.dpsr_T = std::max(marginMax.dpsr_T, std::abs(sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_T_r_s - ref->get().Tropo_delay));

            double refPsrRange = ref->get().getP_Range(satSigId.freq());

            LOG_DEBUG("      satData.psrEst    = {:.4f} [m]", sigObs.recvObs.at(SPP::Algorithm::Rover)->obs.at(GnssObs::Pseudorange).estimate);
            LOG_DEBUG("        refPsr          = {:.4f} [m]", refPsrRange);
            LOG_DEBUG("        psrEst - refPsr = {:.4e} [m]", sigObs.recvObs.at(SPP::Algorithm::Rover)->obs.at(GnssObs::Pseudorange).estimate - refPsrRange);
            CHECK_THAT(sigObs.recvObs.at(SPP::Algorithm::Rover)->obs.at(GnssObs::Pseudorange).estimate - refPsrRange,
                       Catch::Matchers::WithinAbs(0.0, margin.pseudorange));
            marginMax.pseudorange = std::max(marginMax.pseudorange, std::abs(sigObs.recvObs.at(SPP::Algorithm::Rover)->obs.at(GnssObs::Pseudorange).estimate - refPsrRange));

            LOG_DEBUG("      geometricDist {:.4f} + dpsr_ie_r_s {:.4f} = {:.4f} [m]", sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.rho_r_s,
                      sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_ie_r_s,
                      sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.rho_r_s + sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_ie_r_s);
            LOG_DEBUG("        refGeometricDist       {:.4f} [m]", ref->get().Range);
            LOG_DEBUG("        geometricDist - refGeometricDist = {:.4e} [m]",
                      sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.rho_r_s + sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_ie_r_s - ref->get().Range);
            CHECK_THAT(sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.rho_r_s + sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_ie_r_s - ref->get().Range,
                       Catch::Matchers::WithinAbs(0.0, margin.geometricDist));
            marginMax.geometricDist = std::max(marginMax.geometricDist, std::abs(sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.rho_r_s + sigObs.recvObs.at(SPP::Algorithm::Rover)->terms.dpsr_ie_r_s - ref->get().Range));

            ref->get().checked = true;
        }
    });

    REQUIRE(testFlow("test/flow/Nodes/DataProcessor/GNSS/SinglePointPositioning.flow"));

    for ([[maybe_unused]] const auto& [freq, margin] : marginsMax)
    {
        LOG_DEBUG("[{}] margin maxima", freq);
        LOG_DEBUG("  pos = {:e}", margin.pos);
        LOG_DEBUG("  vel = {:e}", margin.vel);
        LOG_DEBUG("  satElevation = {:e}", margin.satElevation);
        LOG_DEBUG("  satAzimuth = {:e}", margin.satAzimuth);
        LOG_DEBUG("  dpsr_I = {:e}", margin.dpsr_I);
        LOG_DEBUG("  dpsr_T = {:e}", margin.dpsr_T);
        LOG_DEBUG("  pseudorange = {:e}", margin.pseudorange);
        LOG_DEBUG("  geometricDist = {:e}", margin.geometricDist);
    }

    CHECK(messageCounter == obsCount);

    for (const auto& satData : spirentSatelliteData.refData)
    {
        if ((satData.satId.satSys & filterFreq.getSatSys()) != SatSys_None)
        {
            LOG_DEBUG("[{}][{}] Checking if ref data was used", satData.recvTime.toYMDHMS(GPST), satData.satId);
            if (rad2deg(satData.Elevation) >= elevationMaskDeg)
            {
                REQUIRE(satData.checked);
            }
        }
    }
}

} // namespace

TEST_CASE("[ObservationEstimator][flow] Check estimates with Skydel data (GPS - no Iono - no Tropo)", "[ObservationEstimator][flow]")
{
    Frequency filterFreq = G01 | G02 | G05;
    Code filterCode = Code_ALL | G01 | G02 | G05;
    double elevationMaskDeg = 0;

    IonosphereModel ionoModel = IonosphereModel::None;
    AtmosphereModels atmosphere{
        .pressureModel = PressureModel::ISA,
        .temperatureModel = TemperatureModel::ISA,
        .waterVaporModel = WaterVaporModel::ISA,
    };
    auto tropoModel = TroposphereModelSelection{
        .zhdModel = std::make_pair(TroposphereModel::None, atmosphere),
        .zwdModel = std::make_pair(TroposphereModel::None, atmosphere),
        .zhdMappingFunction = std::make_pair(MappingFunction::Cosecant, atmosphere),
        .zwdMappingFunction = std::make_pair(MappingFunction::Cosecant, atmosphere),
    };

    const Eigen::Vector3d lla_refRecvPos(deg2rad(30.0), deg2rad(95.0), 0.0);
    size_t obsCount = 49;

    // Determined by running the test and adapting
    std::unordered_map<Frequency, SkydelReference::Margin> margins = {
        { G01 | G02, SkydelReference::Margin{ .clock = 4.8e-15,
                                              .pos = 1.9e-4,
                                              .satElevation = 3.7e-10,
                                              .satAzimuth = 7.0e-9,
                                              .dpsr_I = 0,
                                              .dpsr_T = 0,
                                              .timeDiffRecvTrans = 7.0e-4,
                                              .geometricDist = 1.2e-4 } },
    };
    margins[G05] = margins.at(G01 | G02);
    margins.at(G05).clock = 1.3e-8; // Skydel applies T_GD to the satellite clock, which is wrong according to IS-GPS-705J GPS ICD L5, ch. 20.3.3.3.2.1, p.78
    std::string folder = "test/data/GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/sat_data/";
    std::vector<SkydelReference> sppReference;
    {
        sppReference.emplace_back(SatSigId(Code::G1X, 1), folder + "L1C 01.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 3), folder + "L1C 03.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 6), folder + "L1C 06.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 7), folder + "L1C 07.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 8), folder + "L1C 08.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 9), folder + "L1C 09.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 11), folder + "L1C 11.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 13), folder + "L1C 13.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 14), folder + "L1C 14.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 17), folder + "L1C 17.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 19), folder + "L1C 19.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 21), folder + "L1C 21.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 24), folder + "L1C 24.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 30), folder + "L1C 30.csv");
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
        sppReference.emplace_back(SatSigId(Code::G1P, 1), folder + "L1P 01.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 3), folder + "L1P 03.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 6), folder + "L1P 06.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 7), folder + "L1P 07.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 8), folder + "L1P 08.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 9), folder + "L1P 09.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 11), folder + "L1P 11.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 13), folder + "L1P 13.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 14), folder + "L1P 14.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 17), folder + "L1P 17.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 19), folder + "L1P 19.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 21), folder + "L1P 21.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 24), folder + "L1P 24.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 30), folder + "L1P 30.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 1), folder + "L2C 01.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 3), folder + "L2C 03.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 6), folder + "L2C 06.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 7), folder + "L2C 07.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 8), folder + "L2C 08.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 9), folder + "L2C 09.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 11), folder + "L2C 11.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 13), folder + "L2C 13.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 14), folder + "L2C 14.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 17), folder + "L2C 17.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 19), folder + "L2C 19.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 21), folder + "L2C 21.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 24), folder + "L2C 24.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 30), folder + "L2C 30.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 1), folder + "L2P 01.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 3), folder + "L2P 03.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 6), folder + "L2P 06.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 7), folder + "L2P 07.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 8), folder + "L2P 08.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 9), folder + "L2P 09.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 11), folder + "L2P 11.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 13), folder + "L2P 13.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 14), folder + "L2P 14.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 17), folder + "L2P 17.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 19), folder + "L2P 19.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 21), folder + "L2P 21.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 24), folder + "L2P 24.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 30), folder + "L2P 30.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 1), folder + "L5 01.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 3), folder + "L5 03.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 6), folder + "L5 06.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 7), folder + "L5 07.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 8), folder + "L5 08.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 9), folder + "L5 09.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 11), folder + "L5 11.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 13), folder + "L5 13.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 14), folder + "L5 14.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 17), folder + "L5 17.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 19), folder + "L5 19.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 21), folder + "L5 21.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 24), folder + "L5 24.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 30), folder + "L5 30.csv");
    }

    testSkydelData(filterFreq, filterCode, ionoModel, tropoModel, elevationMaskDeg, lla_refRecvPos,
                   "GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/SkydelRINEX_S_20230080000_04H_MO.rnx",
                   "GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/SkydelRINEX_S_20238959_7200S_GN.rnx",
                   sppReference, obsCount, margins);
}

TEST_CASE("[ObservationEstimator][flow] Check estimates with Skydel data (Klobuchar, Saastamoinen)", "[ObservationEstimator][flow]")
{
    Frequency filterFreq = G01 | G02 | G05;
    Code filterCode = Code_ALL | G01 | G02 | G05;
    double elevationMaskDeg = 0;

    IonosphereModel ionoModel = IonosphereModel::Klobuchar;
    AtmosphereModels atmosphere{
        .pressureModel = PressureModel::ISA,
        .temperatureModel = TemperatureModel::ISA,
        .waterVaporModel = WaterVaporModel::ISA,
    };
    auto tropoModel = TroposphereModelSelection{
        .zhdModel = std::make_pair(TroposphereModel::Saastamoinen, atmosphere),
        .zwdModel = std::make_pair(TroposphereModel::Saastamoinen, atmosphere),
        .zhdMappingFunction = std::make_pair(MappingFunction::Cosecant, atmosphere),
        .zwdMappingFunction = std::make_pair(MappingFunction::Cosecant, atmosphere),
    };

    const Eigen::Vector3d lla_refRecvPos(deg2rad(30.0), deg2rad(95.0), 0.0);
    size_t obsCount = 49;

    // Determined by running the test and adapting
    std::unordered_map<Frequency, SkydelReference::Margin> margins = {
        { G01 | G02, SkydelReference::Margin{ .clock = 4.8e-15,
                                              .pos = 3.7e-4,
                                              .satElevation = 6.4e-10,
                                              .satAzimuth = 1.4e-8,
                                              .dpsr_I = 1.5e-1,
                                              .dpsr_T = 9.8e-1,
                                              .timeDiffRecvTrans = 7.0e-4,
                                              .geometricDist = 1.5e-4 } },
    };
    margins[G05] = margins.at(G01 | G02);
    margins.at(G05).clock = 1.3e-8; // Skydel applies T_GD to the satellite clock, which is wrong according to IS-GPS-705J GPS ICD L5, ch. 20.3.3.3.2.1, p.78
    std::string folder = "test/data/GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-Klob_tropo-Saast/sat_data/";
    std::vector<SkydelReference> sppReference;
    {
        sppReference.emplace_back(SatSigId(Code::G1X, 1), folder + "L1C 01.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 3), folder + "L1C 03.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 6), folder + "L1C 06.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 7), folder + "L1C 07.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 8), folder + "L1C 08.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 9), folder + "L1C 09.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 11), folder + "L1C 11.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 13), folder + "L1C 13.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 14), folder + "L1C 14.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 17), folder + "L1C 17.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 19), folder + "L1C 19.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 21), folder + "L1C 21.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 24), folder + "L1C 24.csv");
        sppReference.emplace_back(SatSigId(Code::G1X, 30), folder + "L1C 30.csv");
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
        sppReference.emplace_back(SatSigId(Code::G1P, 1), folder + "L1P 01.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 3), folder + "L1P 03.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 6), folder + "L1P 06.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 7), folder + "L1P 07.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 8), folder + "L1P 08.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 9), folder + "L1P 09.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 11), folder + "L1P 11.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 13), folder + "L1P 13.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 14), folder + "L1P 14.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 17), folder + "L1P 17.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 19), folder + "L1P 19.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 21), folder + "L1P 21.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 24), folder + "L1P 24.csv");
        sppReference.emplace_back(SatSigId(Code::G1P, 30), folder + "L1P 30.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 1), folder + "L2C 01.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 3), folder + "L2C 03.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 6), folder + "L2C 06.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 7), folder + "L2C 07.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 8), folder + "L2C 08.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 9), folder + "L2C 09.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 11), folder + "L2C 11.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 13), folder + "L2C 13.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 14), folder + "L2C 14.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 17), folder + "L2C 17.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 19), folder + "L2C 19.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 21), folder + "L2C 21.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 24), folder + "L2C 24.csv");
        sppReference.emplace_back(SatSigId(Code::G2C, 30), folder + "L2C 30.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 1), folder + "L2P 01.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 3), folder + "L2P 03.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 6), folder + "L2P 06.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 7), folder + "L2P 07.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 8), folder + "L2P 08.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 9), folder + "L2P 09.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 11), folder + "L2P 11.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 13), folder + "L2P 13.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 14), folder + "L2P 14.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 17), folder + "L2P 17.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 19), folder + "L2P 19.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 21), folder + "L2P 21.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 24), folder + "L2P 24.csv");
        sppReference.emplace_back(SatSigId(Code::G2P, 30), folder + "L2P 30.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 1), folder + "L5 01.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 3), folder + "L5 03.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 6), folder + "L5 06.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 7), folder + "L5 07.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 8), folder + "L5 08.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 9), folder + "L5 09.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 11), folder + "L5 11.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 13), folder + "L5 13.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 14), folder + "L5 14.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 17), folder + "L5 17.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 19), folder + "L5 19.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 21), folder + "L5 21.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 24), folder + "L5 24.csv");
        sppReference.emplace_back(SatSigId(Code::G5X, 30), folder + "L5 30.csv");
    }

    testSkydelData(filterFreq, filterCode, ionoModel, tropoModel, elevationMaskDeg, lla_refRecvPos,
                   "GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-Klob_tropo-Saast/SkydelRINEX_S_20230080959_04H_02Z_MO.rnx",
                   "GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/SkydelRINEX_S_20238959_7200S_GN.rnx",
                   sppReference, obsCount, margins);
}

TEST_CASE("[ObservationEstimator][flow] Check estimates with Spirent data (GPS - no Iono - no Tropo)", "[ObservationEstimator][flow]")
{
    Frequency filterFreq = G01 | G02 | G05;
    Code filterCode = Code_ALL | G01 | G02 | G05;
    double elevationMaskDeg = 0;

    IonosphereModel ionoModel = IonosphereModel::None;
    AtmosphereModels atmosphere{
        .pressureModel = PressureModel::ISA,
        .temperatureModel = TemperatureModel::ISA,
        .waterVaporModel = WaterVaporModel::ISA,
    };
    auto tropoModel = TroposphereModelSelection{
        .zhdModel = std::make_pair(TroposphereModel::None, atmosphere),
        .zwdModel = std::make_pair(TroposphereModel::None, atmosphere),
        .zhdMappingFunction = std::make_pair(MappingFunction::Cosecant, atmosphere),
        .zwdMappingFunction = std::make_pair(MappingFunction::Cosecant, atmosphere),
    };

    const Eigen::Vector3d lla_refRecvPos(deg2rad(30.0), deg2rad(95.0), 0.0);
    size_t refDataSize = 2114;
    size_t obsCount = 49;

    // Determined by running the test and adapting
    std::unordered_map<Frequency, SpirentSatDataFile::Margin> margins = {
        { G01 | G02 | G05, SpirentSatDataFile::Margin{ .pos = 9.1e-5,
                                                       .vel = 8.4e-4,
                                                       .satElevation = 3.2e-3, // High because Satellite position is calculated in ECEF frame at transmit time
                                                       .satAzimuth = 4.5e-3,
                                                       .dpsr_I = 0,
                                                       .dpsr_T = 0,
                                                       .pseudorange = 5.2e-4,
                                                       .geometricDist = 1.5e-4 } },
    };

    testSpirentData(filterFreq, filterCode, ionoModel, tropoModel, elevationMaskDeg, lla_refRecvPos,
                    "GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none/Spirent_RINEX_MO.obs",
                    "GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Spirent_RINEX_GN.23N",
                    "test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none/sat_data_V1A1.csv",
                    refDataSize, obsCount, margins);
}

TEST_CASE("[ObservationEstimator][flow] Check estimates with Spirent data (GPS - Klobuchar, Saastamoinen)", "[ObservationEstimator][flow]")
{
    Frequency filterFreq = G01 | G02 | G05;
    Code filterCode = Code_ALL | G01 | G02 | G05;
    double elevationMaskDeg = 10;

    IonosphereModel ionoModel = IonosphereModel::Klobuchar;
    AtmosphereModels atmosphere{
        .pressureModel = PressureModel::ISA,
        .temperatureModel = TemperatureModel::ISA,
        .waterVaporModel = WaterVaporModel::ISA,
    };
    auto tropoModel = TroposphereModelSelection{
        .zhdModel = std::make_pair(TroposphereModel::Saastamoinen, atmosphere),
        .zwdModel = std::make_pair(TroposphereModel::Saastamoinen, atmosphere),
        .zhdMappingFunction = std::make_pair(MappingFunction::GMF, atmosphere),
        .zwdMappingFunction = std::make_pair(MappingFunction::GMF, atmosphere),
    };

    const Eigen::Vector3d lla_refRecvPos(deg2rad(30.0), deg2rad(95.0), 0.0);
    size_t refDataSize = 2114;
    size_t obsCount = 49;

    // Determined by running the test and adapting
    std::unordered_map<Frequency, SpirentSatDataFile::Margin> margins = {
        { G01 | G02 | G05, SpirentSatDataFile::Margin{ .pos = 2.7e-4,
                                                       .vel = 8.4e-4,
                                                       .satElevation = 3.2e-3, // High because Satellite position is calculated in ECEF frame at transmit time
                                                       .satAzimuth = 4.5e-3,
                                                       .dpsr_I = 3.9e-4,
                                                       .dpsr_T = 3.5e-1,
                                                       .pseudorange = 5.0e-1,
                                                       .geometricDist = 1.8e-4 } },
    };

    testSpirentData(filterFreq, filterCode, ionoModel, tropoModel, elevationMaskDeg, lla_refRecvPos,
                    "GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-Klob_tropo-Saast/Spirent_RINEX_MO.obs",
                    "GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Spirent_RINEX_GN.23N",
                    "test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-Klob_tropo-Saast/sat_data_V1A1.csv",
                    refDataSize, obsCount, margins);
}

#endif

} // namespace NAV::TESTS::ObservationEstimatorTests
