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

#include "data/OroliaRawLogging.hpp"

#include <string>
#include <vector>
#include <fstream>
#include <algorithm>

#include "FlowTester.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "Logger.hpp"
#include "util/StringUtil.hpp"

#include "NodeData/GNSS/SppSolution.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/Transformations/Units.hpp"

namespace NAV::TESTS::SinglePointPositioningTests
{

/// @brief SppReference for SPP calcualtion
struct SppReference
{
    /// @brief Constructor
    /// @param[in] freq GNSS frequency
    /// @param[in] satNum Satellite number (SVID)
    /// @param[in] code Signal code
    /// @param[in] path Path to the reference file
    SppReference(Frequency freq, uint16_t satNum, Code code, const std::string& path)
        : satSigId(freq, satNum), code(code), fs("test/data/GNSS/Orolia-Skydel_static_duration-4h_rate-5min_sys-GERCQIS_iono-none_tropo-none/sat_data/" + path, std::ios_base::binary)
    {
        REQUIRE(fs.good());
        std::string line;
        std::getline(fs, line); // Header line

        auto pos = fs.tellg();
        dataCnt = std::count(std::istreambuf_iterator<char>(fs),
                             std::istreambuf_iterator<char>(), '\n');
        fs.seekg(pos);
    }

    SatSigId satSigId;   ///< GNSS frequency and satellite number
    Code code;           ///< Signal code
    std::ifstream fs;    ///< File stream to the reference file
    int64_t dataCnt;     ///< Amount of data lines in the file
    int64_t counter = 0; ///< Counter to see how many lines were read
};

TEST_CASE("[SinglePointPositioning][flow] SPP with Skydel data (GPS L1 C/A - no Iono - no Tropo)", "[SinglePointPositioning][flow]")
{
    auto logger = initializeTestLogger();

    // ###########################################################################################################
    //                                       SinglePointPositioningSkydel.flow
    // ###########################################################################################################
    //
    // PosVelAttInitializer (70)
    //         (69) PosVelAtt |>  -
    //                             \              SinglePointPositioning (50)                     Plot (77)
    // RinexObsFile (65)            (71)-->  |> PosVelInit (46)   (49) SppSolution |>  --(78)-->  |> SPP (72)
    //         (64) PosVelAtt |>  --(66)-->  |> GnssObs (47)                             (81)-->  |> RTKLIB (76)
    //                              (55)-->  |> GnssNavInfo (48)                        /
    // RinexNavFile() (54)         /                                                   /
    //         (53) PosVelAtt <>  -                            RtklibPosFile (80)     /
    //                                                         (79) RtklibPosObs |>  -
    //
    // ###########################################################################################################

    const Eigen::Vector3d e_refRecvPos{ -481819.31349724, 5507219.95375401, 3170373.73538364 }; // Receiver position simulated by Skydel
    Eigen::Vector3d lla_refRecvPos = trafo::ecef2lla_WGS84(e_refRecvPos);
    LOG_DEBUG("lla_refRecvPos {}, {}, {}", rad2deg(lla_refRecvPos.x()), rad2deg(lla_refRecvPos.y()), lla_refRecvPos.z());

    std::vector<SppReference> sppReference;
    sppReference.emplace_back(G01, 1, Code::G1C, "L1CA 01.csv");
    sppReference.emplace_back(G01, 3, Code::G1C, "L1CA 03.csv");
    sppReference.emplace_back(G01, 6, Code::G1C, "L1CA 06.csv");
    sppReference.emplace_back(G01, 7, Code::G1C, "L1CA 07.csv");
    sppReference.emplace_back(G01, 8, Code::G1C, "L1CA 08.csv");
    sppReference.emplace_back(G01, 9, Code::G1C, "L1CA 09.csv");
    sppReference.emplace_back(G01, 11, Code::G1C, "L1CA 11.csv");
    sppReference.emplace_back(G01, 13, Code::G1C, "L1CA 13.csv");
    sppReference.emplace_back(G01, 14, Code::G1C, "L1CA 14.csv");
    sppReference.emplace_back(G01, 17, Code::G1C, "L1CA 17.csv");
    sppReference.emplace_back(G01, 19, Code::G1C, "L1CA 19.csv");
    sppReference.emplace_back(G01, 21, Code::G1C, "L1CA 21.csv");
    sppReference.emplace_back(G01, 24, Code::G1C, "L1CA 24.csv");
    sppReference.emplace_back(G01, 30, Code::G1C, "L1CA 30.csv");

    size_t messageCounter = 0; // Message Counter

    nm::RegisterWatcherCallbackToInputPin(72, [&](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        messageCounter++;
        auto sppSol = std::dynamic_pointer_cast<const NAV::SppSolutionExtended>(queue.front());

        LOG_DEBUG("    e_refRecvPos         {} [m]", e_refRecvPos.transpose());
        LOG_DEBUG("    sppSol->e_position() {} [m]", sppSol->e_position().transpose());
        Eigen::Vector3d n_diff = trafo::n_Quat_e(lla_refRecvPos.x(), lla_refRecvPos.y()) * (sppSol->e_position() - e_refRecvPos);
        double hDist = n_diff.head<2>().norm();
        double vDist = std::abs(n_diff.z());
        LOG_DEBUG("    hDist {} [m]", hDist);
        LOG_DEBUG("    vDist {} [m]", vDist);
        REQUIRE(hDist < 6e-4); // Determined by running the test and adapting
        REQUIRE(vDist < 2e-3); // Determined by running the test and adapting

        for (auto& ref : sppReference)
        {
            auto pos = ref.fs.tellg();
            std::string line;
            std::getline(ref.fs, line);
            if (line.empty()) { continue; }

            std::vector<std::string> v = str::split(line, ",");
            REQUIRE(v.size() == OroliaRawLogging_COUNT);

            InsTime refRecvTime = InsTime(0, std::stoi(v[OroliaRawLogging_GPS_Week_Number]), std::stold(v[OroliaRawLogging_GPS_TOW]));
            REQUIRE(sppSol->insTime <= refRecvTime);

            if (sppSol->insTime == refRecvTime)
            {
                LOG_DEBUG("line: {}", line);
                LOG_DEBUG("[{}][{}-{}] Processing line {} in file", refRecvTime.toYMDHMS(), ref.satSigId, ref.code, ref.counter + 2);
                auto iter = std::find_if(sppSol->extData.begin(), sppSol->extData.end(), [&ref](const SppSolutionExtended::ExtendedData& extData) {
                    return extData.satSigId == ref.satSigId && extData.code == ref.code;
                });
                REQUIRE(iter != sppSol->extData.end()); // This means something was calculated for the satellite
                ref.counter++;

                const auto& calcData = (*sppSol)(ref.satSigId.freq, ref.satSigId.satNum, ref.code);

                if (calcData.skipped)
                {
                    continue;
                }

                Eigen::Vector3d e_refSatPos(std::stod(v[OroliaRawLogging_ECEF_X]), std::stod(v[OroliaRawLogging_ECEF_Y]), std::stod(v[OroliaRawLogging_ECEF_Z]));
                LOG_DEBUG("    calcData.e_satPos {} [m]", calcData.e_satPos.transpose());
                LOG_DEBUG("    e_refSatPos       {} [m]", e_refSatPos.transpose());
                LOG_DEBUG("      calcData.pos - e_refPos   = {}", (calcData.e_satPos - e_refSatPos).transpose());
                LOG_DEBUG("    | calcData.pos - e_refPos | = {} [m]", (calcData.e_satPos - e_refSatPos).norm());
                REQUIRE_THAT((calcData.e_satPos - e_refSatPos).norm(), Catch::Matchers::WithinAbs(0.0, 2e-4)); // Determined by running the test and adapting

                // e_satVel

                double refClkCorrection = std::stod(v[OroliaRawLogging_Clock_Correction]); // [s]
                LOG_DEBUG("    calcData.satClkBias {} [s]", calcData.satClkBias);
                LOG_DEBUG("    refClkCorrection    {} [s]", refClkCorrection);
                LOG_DEBUG("    clkBias - ref {} [s]", calcData.satClkBias - refClkCorrection);
                REQUIRE_THAT(calcData.satClkBias - refClkCorrection, Catch::Matchers::WithinAbs(0.0, 4e-15)); // Determined by running the test and adapting

                // satClkDrift

                double refSatElevation = std::stod(v[OroliaRawLogging_Receiver_Antenna_Elevation]); // Satellite’s elevation, in radians, from the receiver’s antenna position.
                LOG_DEBUG("    calcData.satElevation {} [deg]", rad2deg(calcData.satElevation));
                LOG_DEBUG("    refSatElevation       {} [deg]", rad2deg(refSatElevation));
                LOG_DEBUG("    calcData.satElevation - refSatElevation {} [°]", rad2deg(calcData.satElevation - refSatElevation));
                REQUIRE_THAT(rad2deg(calcData.satElevation - refSatElevation), Catch::Matchers::WithinAbs(0.0, 1e-7)); // Determined by running the test and adapting

                double refSatAzimuth = std::stod(v[OroliaRawLogging_Receiver_Antenna_Azimuth]); // Satellite’s azimuth, in radians, from the receiver’s antenna position.
                LOG_DEBUG("    calcData.satAzimuth {} [°]", rad2deg(calcData.satAzimuth));
                LOG_DEBUG("    refSatAzimuth       {} [°]", rad2deg(refSatAzimuth));
                LOG_DEBUG("    calcData.satAzimuth - refSatAzimuth {} [°]", rad2deg(calcData.satAzimuth - refSatAzimuth));
                REQUIRE_THAT(rad2deg(calcData.satAzimuth - refSatAzimuth), Catch::Matchers::WithinAbs(0.0, 1e-7)); // Determined by running the test and adapting

                if (calcData.elevationMaskTriggered)
                {
                    REQUIRE(std::isnan(calcData.pseudorangeRate));
                    REQUIRE(std::isnan(calcData.dpsr_I));
                    REQUIRE(std::isnan(calcData.dpsr_T));
                    REQUIRE(std::isnan(calcData.geometricDist));
                }
                else
                {
                    double refIonoCorrection = std::stod(v[OroliaRawLogging_Iono_Correction]); // Ionospheric corrections [m]
                    LOG_DEBUG("    calcData.dpsr_I   {} [m]", calcData.dpsr_I);
                    LOG_DEBUG("    refIonoCorrection {} [m]", refIonoCorrection);
                    LOG_DEBUG("    calcData.dpsr_I - refIonoCorrection = {} [m]", calcData.dpsr_I - refIonoCorrection);
                    REQUIRE(calcData.dpsr_I == refIonoCorrection);

                    double refTropoCorrection = std::stod(v[OroliaRawLogging_Tropo_Correction]); // Tropospheric corrections [m]
                    LOG_DEBUG("    calcData.dpsr_T    {} [m]", calcData.dpsr_T);
                    LOG_DEBUG("    refTropoCorrection {} [m]", refTropoCorrection);
                    LOG_DEBUG("    calcData.dpsr_T - refTropoCorrection = {} [m]", calcData.dpsr_T - refTropoCorrection);
                    REQUIRE(calcData.dpsr_T == refTropoCorrection);

                    double timeDiffRange_ref = (std::stod(v[OroliaRawLogging_Elapsed_Time]) - std::stod(v[OroliaRawLogging_PSR_satellite_time])) * 1e-3; // [s]
                    double timeDiffRecvTrans = static_cast<double>((sppSol->insTime - calcData.transmitTime).count());                                   // [s]
                    timeDiffRecvTrans += calcData.dpsr_I / InsConst::C;
                    timeDiffRecvTrans += calcData.dpsr_T / InsConst::C;
                    LOG_DEBUG("    timeDiffRecvTrans {} [s]", timeDiffRecvTrans);
                    LOG_DEBUG("    timeDiffRange_ref {} [s]", timeDiffRange_ref);
                    LOG_DEBUG("    timeDiffRecvTrans - timeDiffRange_ref {} [s]", timeDiffRecvTrans - timeDiffRange_ref);
                    REQUIRE_THAT(timeDiffRecvTrans - timeDiffRange_ref, Catch::Matchers::WithinAbs(0.0, 7e-4)); // Determined by running the test and adapting

                    double refGeometricDist = std::stod(v[OroliaRawLogging_Range]); // Geometrical distance [m] between the satellite’s and receiver’s antennas.
                    LOG_DEBUG("    calcData.geometricDist {} [m]", calcData.geometricDist);
                    LOG_DEBUG("    refGeometricDist       {} [m]", refGeometricDist);
                    LOG_DEBUG("    calcData.geometricDist - refGeometricDist = {} [m]", calcData.geometricDist - refGeometricDist);

                    REQUIRE_THAT(calcData.geometricDist - refGeometricDist, Catch::Matchers::WithinAbs(0.0, 33)); // Determined by running the test and adapting // TODO: this is very high
                }
            }
            else // if (sppSol->insTime < refRecvTime)
            {
                ref.fs.seekg(pos);
            }
        }
    });

    REQUIRE(testFlow("test/flow/Nodes/DataProcessor/GNSS/SinglePointPositioning.flow"));

    CHECK(messageCounter == 49);
    for (auto& ref : sppReference)
    {
        LOG_DEBUG("Checking if all messages from satellite {}-{} were read.", ref.satSigId, ref.code);
        REQUIRE(ref.counter == ref.dataCnt);
    }
}

} // namespace NAV::TESTS::SinglePointPositioningTests
