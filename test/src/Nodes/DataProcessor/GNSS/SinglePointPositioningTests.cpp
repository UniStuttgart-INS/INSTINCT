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
#undef protected
#undef private
#pragma GCC diagnostic pop

namespace NAV::TESTS::SinglePointPositioningTests
{

/// @brief SppReference for SPP calcualtion
struct SppReference
{
    /// @brief Constructor
    /// @param[in] code Signal code
    /// @param[in] satNum Satellite number (SVID)
    /// @param[in] path Path to the reference file
    SppReference(Code code, uint16_t satNum, const std::string& path)
        : satSigId(code, satNum), fs("test/data/GNSS/Orolia-Skydel_static_duration-4h_rate-5min_sys-GERCQIS_iono-none_tropo-none/sat_data/" + path, std::ios_base::binary)
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
    sppReference.emplace_back(Code::G1C, 1, "L1CA 01.csv");
    sppReference.emplace_back(Code::G1C, 3, "L1CA 03.csv");
    sppReference.emplace_back(Code::G1C, 6, "L1CA 06.csv");
    sppReference.emplace_back(Code::G1C, 7, "L1CA 07.csv");
    sppReference.emplace_back(Code::G1C, 8, "L1CA 08.csv");
    sppReference.emplace_back(Code::G1C, 9, "L1CA 09.csv");
    sppReference.emplace_back(Code::G1C, 11, "L1CA 11.csv");
    sppReference.emplace_back(Code::G1C, 13, "L1CA 13.csv");
    sppReference.emplace_back(Code::G1C, 14, "L1CA 14.csv");
    sppReference.emplace_back(Code::G1C, 17, "L1CA 17.csv");
    sppReference.emplace_back(Code::G1C, 19, "L1CA 19.csv");
    sppReference.emplace_back(Code::G1C, 21, "L1CA 21.csv");
    sppReference.emplace_back(Code::G1C, 24, "L1CA 24.csv");
    sppReference.emplace_back(Code::G1C, 30, "L1CA 30.csv");

    size_t messageCounter = 0; // Message Counter

    nm::RegisterWatcherCallbackToInputPin(72, [&](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
        messageCounter++;
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
                LOG_DEBUG("[{}][{}] Processing line {} in file", refRecvTime.toYMDHMS(), ref.satSigId, ref.counter + 2);
                REQUIRE(sppSol->hasSatelliteData(ref.satSigId)); // This means something was calculated for the satellite
                ref.counter++;

                const auto& calcData = (*sppSol)(ref.satSigId);

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
                    REQUIRE(!calcData.psrRateEst.has_value());
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
        LOG_DEBUG("Checking if all messages from satellite {} were read.", ref.satSigId);
        REQUIRE(ref.counter == ref.dataCnt);
    }
}

TEST_CASE("[SinglePointPositioning][flow] SPP with Spirent data (GPS L1 C/A - no Sat clk - no Iono - no Tropo)", "[SinglePointPositioning][flow]")
{
    auto logger = initializeTestLogger();

    nm::RegisterPreInitCallback([&]() {
        dynamic_cast<RinexObsFile*>(nm::FindNode(65))->_path = "GNSS/Spirent-SimGEN_RTK_static-rover_duration-10min_rate-1s_sys-GE_iono-none_tropo-none/Spirent_RINEX_MO_Base.obs";
        dynamic_cast<RinexNavFile*>(nm::FindNode(54))->_path = "GNSS/Spirent-SimGEN_RTK_static-rover_duration-10min_rate-1s_sys-GE_iono-none_tropo-none/Spirent_RINEX_GN.23N";
    });

    // ###########################################################################################################
    //                                       SinglePointPositioningSkydel.flow
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

    std::ifstream fs{ "test/data/GNSS/Spirent-SimGEN_RTK_static-rover_duration-10min_rate-1s_sys-GE_iono-none_tropo-none/sat_data_V1A1_Base.csv", std::ios_base::binary };
    REQUIRE(fs.good());

    std::string line;
    for (size_t i = 0; i < 5; i++) { std::getline(fs, line); } // Read header lines

    std::vector<SpirentAsciiSatelliteData> spirentSatelliteData;
    while (!fs.eof() && std::getline(fs, line) && !line.empty())
    {
        std::vector<std::string> v = str::split(line, ",");
        REQUIRE(v.size() == size_t(SpirentAsciiSatelliteData_COUNT));

        SatelliteSystem satSys;
        auto satNum = static_cast<uint16_t>(std::stoul(v[SpirentAsciiSatelliteData_Sat_PRN]));
        if (v[SpirentAsciiSatelliteData_Sat_type] == "GPS") { satSys = GPS; }
        else if (v[SpirentAsciiSatelliteData_Sat_type] == "GALILEO") { satSys = GAL; }
        else if (v[SpirentAsciiSatelliteData_Sat_type] == "GLONASS") { satSys = GLO; }
        else if (v[SpirentAsciiSatelliteData_Sat_type] == "BeiDou") { satSys = BDS; }
        else if (v[SpirentAsciiSatelliteData_Sat_type] == "Quasi-Zenith") { satSys = QZSS; }
        else if (v[SpirentAsciiSatelliteData_Sat_type] == "IRNSS") { satSys = IRNSS; }
        REQUIRE(satSys != SatSys_None);

        InsTime recvTime = InsTime{ 2023, 1, 8, 10, 0, std::stod(v[SpirentAsciiSatelliteData_Time_ms]) * 1e-3, GPST };

        // The Spirent reference frame is rotated by the signal transmit time
        auto rotateDataFrame = [&v](const Eigen::Vector3d& e_satPos) -> Eigen::Vector3d {
            auto dt = std::stod(v[SpirentAsciiSatelliteData_Range]) / InsConst::C;

            // see \cite SpringerHandbookGNSS2017 Springer Handbook GNSS ch. 21.2, eq. 21.18, p. 610
            return Eigen::AngleAxisd(InsConst::omega_ie * dt, Eigen::Vector3d::UnitZ()) * e_satPos;
        };

        spirentSatelliteData.push_back(SpirentAsciiSatelliteData{
            .recvTime = recvTime,
            .satId = SatId{ satSys, satNum },
            .Time_ms = std::stoul(v[SpirentAsciiSatelliteData_Time_ms]),
            .Channel = std::stoul(v[SpirentAsciiSatelliteData_Channel]),
            .Sat_type = v[SpirentAsciiSatelliteData_Sat_type],
            .Sat_ID = std::stoul(v[SpirentAsciiSatelliteData_Sat_ID]),
            .Sat_PRN = std::stoul(v[SpirentAsciiSatelliteData_Sat_PRN]),
            .Echo_Num = std::stoul(v[SpirentAsciiSatelliteData_Echo_Num]),
            .Sat_Pos = rotateDataFrame({ std::stod(v[SpirentAsciiSatelliteData_Sat_Pos_X]),
                                         std::stod(v[SpirentAsciiSatelliteData_Sat_Pos_Y]),
                                         std::stod(v[SpirentAsciiSatelliteData_Sat_Pos_Z]) }),
            .Sat_Vel = rotateDataFrame({ std::stod(v[SpirentAsciiSatelliteData_Sat_Vel_X]),
                                         std::stod(v[SpirentAsciiSatelliteData_Sat_Vel_Y]),
                                         std::stod(v[SpirentAsciiSatelliteData_Sat_Vel_Z]) }),
            .Azimuth = std::stod(v[SpirentAsciiSatelliteData_Azimuth]),
            .Elevation = std::stod(v[SpirentAsciiSatelliteData_Elevation]),
            .Range = std::stod(v[SpirentAsciiSatelliteData_Range]),
            .P_Range_Group_A = std::stod(v[SpirentAsciiSatelliteData_P_Range_Group_A]),
            .P_Range_Group_B = std::stod(v[SpirentAsciiSatelliteData_P_Range_Group_B]),
            .P_Range_Group_C = std::stod(v[SpirentAsciiSatelliteData_P_Range_Group_C]),
            .P_Range_Group_D = std::stod(v[SpirentAsciiSatelliteData_P_Range_Group_D]),
            .P_Range_Group_E = std::stod(v[SpirentAsciiSatelliteData_P_Range_Group_E]),
            .P_Range_Group_F = std::stod(v[SpirentAsciiSatelliteData_P_Range_Group_F]),
            .P_R_rate = std::stod(v[SpirentAsciiSatelliteData_P_R_rate]),
            .Iono_delay_Group_A = std::stod(v[SpirentAsciiSatelliteData_Iono_delay_Group_A]),
            .Iono_delay_Group_B = std::stod(v[SpirentAsciiSatelliteData_Iono_delay_Group_B]),
            .Iono_delay_Group_C = std::stod(v[SpirentAsciiSatelliteData_Iono_delay_Group_C]),
            .Iono_delay_Group_D = std::stod(v[SpirentAsciiSatelliteData_Iono_delay_Group_D]),
            .Iono_delay_Group_E = std::stod(v[SpirentAsciiSatelliteData_Iono_delay_Group_E]),
            .Iono_delay_Group_F = std::stod(v[SpirentAsciiSatelliteData_Iono_delay_Group_F]),
            .Tropo_delay = std::stod(v[SpirentAsciiSatelliteData_Tropo_delay]),
            .P_R_Error = std::stod(v[SpirentAsciiSatelliteData_P_R_Error]),
            .Signal_dB_Group_A = std::stod(v[SpirentAsciiSatelliteData_Signal_dB_Group_A]),
            .Signal_dB_Group_B = std::stod(v[SpirentAsciiSatelliteData_Signal_dB_Group_B]),
            .Signal_dB_Group_C = std::stod(v[SpirentAsciiSatelliteData_Signal_dB_Group_C]),
            .Signal_dB_Group_D = std::stod(v[SpirentAsciiSatelliteData_Signal_dB_Group_D]),
            .Signal_dB_Group_E = std::stod(v[SpirentAsciiSatelliteData_Signal_dB_Group_E]),
            .Signal_dB_Group_F = std::stod(v[SpirentAsciiSatelliteData_Signal_dB_Group_F]),
            .Ant_azimuth = std::stod(v[SpirentAsciiSatelliteData_Ant_azimuth]),
            .Ant_elevation = std::stod(v[SpirentAsciiSatelliteData_Ant_elevation]),
            .Range_rate = std::stod(v[SpirentAsciiSatelliteData_Range_rate]),
            .P_R_Error_rate = std::stod(v[SpirentAsciiSatelliteData_P_R_Error_rate]),
            .Doppler_shift_Group_A = std::stod(v[SpirentAsciiSatelliteData_Doppler_shift_Group_A]),
            .Doppler_shift_Group_B = std::stod(v[SpirentAsciiSatelliteData_Doppler_shift_Group_B]),
            .Doppler_shift_Group_C = std::stod(v[SpirentAsciiSatelliteData_Doppler_shift_Group_C]),
            .Doppler_shift_Group_D = std::stod(v[SpirentAsciiSatelliteData_Doppler_shift_Group_D]),
            .Doppler_shift_Group_E = std::stod(v[SpirentAsciiSatelliteData_Doppler_shift_Group_E]),
            .Doppler_shift_Group_F = std::stod(v[SpirentAsciiSatelliteData_Doppler_shift_Group_F]),
            .Delta_range_Group_A = std::stod(v[SpirentAsciiSatelliteData_Delta_range_Group_A]),
            .Delta_range_Group_B = std::stod(v[SpirentAsciiSatelliteData_Delta_range_Group_B]),
            .Delta_range_Group_C = std::stod(v[SpirentAsciiSatelliteData_Delta_range_Group_C]),
            .Delta_range_Group_D = std::stod(v[SpirentAsciiSatelliteData_Delta_range_Group_D]),
            .Delta_range_Group_E = std::stod(v[SpirentAsciiSatelliteData_Delta_range_Group_E]),
            .Delta_range_Group_F = std::stod(v[SpirentAsciiSatelliteData_Delta_range_Group_F]),
            .Sat_Acc_X = std::stod(v[SpirentAsciiSatelliteData_Sat_Acc_X]),
            .Sat_Acc_Y = std::stod(v[SpirentAsciiSatelliteData_Sat_Acc_Y]),
            .Sat_Acc_Z = std::stod(v[SpirentAsciiSatelliteData_Sat_Acc_Z]),

        });
    }
    REQUIRE(spirentSatelliteData.size() == 11400);

    size_t messageCounter = 0; // Message Counter
    nm::RegisterWatcherCallbackToInputPin(72, [&](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) {
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

        if (messageCounter == 1) { return; } // First message not included in the sat_data files
        for (const auto& sppSatData : sppSol->satData)
        {
            auto ref = std::find_if(spirentSatelliteData.begin(), spirentSatelliteData.end(), [&sppSatData, &sppSol](const SpirentAsciiSatelliteData& spirentSatData) {
                return spirentSatData.recvTime == sppSol->insTime
                       && spirentSatData.satId == sppSatData.satSigId.toSatId();
            });
            REQUIRE(ref != spirentSatelliteData.end());
            LOG_DEBUG("    {}:", sppSatData.satSigId.toSatId());

            LOG_DEBUG("        | pos - e_refPos | = {}", (sppSatData.e_satPos - ref->Sat_Pos).norm());
            REQUIRE_THAT((sppSatData.e_satPos - ref->Sat_Pos).norm(), Catch::Matchers::WithinAbs(0.0, 2.0e-4)); // Determined by running the test and adapting

            LOG_DEBUG("        | vel - e_refVel | = {}", (sppSatData.e_satVel - ref->Sat_Vel).norm());
            REQUIRE_THAT((sppSatData.e_satVel - ref->Sat_Vel).norm(), Catch::Matchers::WithinAbs(0.0, 9.2e-4)); // Determined by running the test and adapting

            LOG_DEBUG("        spp.satElevation - refElevation = {}°", rad2deg(sppSatData.satElevation - ref->Elevation));
            REQUIRE_THAT(rad2deg(sppSatData.satElevation - ref->Elevation), Catch::Matchers::WithinAbs(0.0, 3.2e-3)); // Determined by running the test and adapting
            LOG_DEBUG("        spp.satAzimuth - refAzimuth = {}°", rad2deg(sppSatData.satAzimuth - ref->Azimuth));
            REQUIRE_THAT(rad2deg(sppSatData.satAzimuth - ref->Azimuth), Catch::Matchers::WithinAbs(0.0, 3.5e-3)); // Determined by running the test and adapting

            LOG_DEBUG("        spp.dpsr_I = {}", sppSatData.dpsr_I);
            REQUIRE(sppSatData.dpsr_I == 0.0);
            LOG_DEBUG("        ref.dpsr_I = {}", ref->Iono_delay_Group_A);
            REQUIRE(ref->Iono_delay_Group_A == 0.0);
            LOG_DEBUG("        ref.dpsr_T = {}", sppSatData.dpsr_T);
            REQUIRE(sppSatData.dpsr_T == 0.0);
            LOG_DEBUG("        ref.dpsr_T = {}", ref->Tropo_delay);
            REQUIRE(ref->Tropo_delay == 0.0);

            LOG_DEBUG("        spp.satClkBias = {}", sppSatData.satClkBias);
            REQUIRE(sppSatData.satClkBias == 0.0);
            LOG_DEBUG("        spp.satClkDrift = {}", sppSatData.satClkDrift);
            REQUIRE(sppSatData.satClkDrift == 0.0);
            LOG_DEBUG("        spp.dpsr_clkISB = {}", sppSatData.dpsr_clkISB);
            REQUIRE(sppSatData.dpsr_clkISB == 0.0);

            LOG_DEBUG("        spp.psrEst - refPsr = {}", sppSatData.psrEst - ref->P_Range_Group_A);
            REQUIRE_THAT(sppSatData.psrEst - ref->P_Range_Group_A, Catch::Matchers::WithinAbs(0.0, 6.7e-4)); // Determined by running the test and adapting

            // Sagnac correction [m] - Springer Handbook ch. 19.1.1, eq. 19.7, p. 562
            double ref_dpsr_ie = 1.0 / InsConst::C * (e_refRecvPos - ref->Sat_Pos).dot(InsConst::e_omega_ie.cross(e_refRecvPos));
            LOG_DEBUG("        spp.dpsr_ie - ref.dpsr_ie = {}", sppSatData.dpsr_ie - ref_dpsr_ie);
            REQUIRE_THAT(sppSatData.dpsr_ie - ref_dpsr_ie, Catch::Matchers::WithinAbs(0.0, 6.2e-9)); // Determined by running the test and adapting

            // The Range is the same as the Pseudorange in the sat_data file, therefore the Sagnac correction is included in the Range
            REQUIRE(ref->Range == ref->P_Range_Group_A);

            LOG_DEBUG("        spp.geometricDist - (refRange - ref_dpsr_ie) = {}", sppSatData.geometricDist - (ref->Range - ref_dpsr_ie));
            REQUIRE_THAT(sppSatData.geometricDist - (ref->Range - ref_dpsr_ie), Catch::Matchers::WithinAbs(0.0, 1.9e-3)); // Determined by running the test and adapting
        }
    });

    REQUIRE(testFlow("test/flow/Nodes/DataProcessor/GNSS/SinglePointPositioning.flow"));

    CHECK(messageCounter == 601);
}

} // namespace NAV::TESTS::SinglePointPositioningTests
