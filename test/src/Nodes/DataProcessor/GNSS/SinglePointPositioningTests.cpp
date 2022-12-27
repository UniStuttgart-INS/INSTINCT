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
#include "util/StringUtil.hpp"

#include "NodeData/GNSS/SppSolution.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/Transformations/Units.hpp"

namespace NAV::TESTS::SinglePointPositioningTests
{

constexpr size_t MESSAGE_COUNT_EXPECTED = 240; ///< Amount of messages expected in the files
size_t messageCounter = 0;                     ///< Message Counter

const Eigen::Vector3d e_refRecvPos{ 4156963.433175, 671202.276327, 4774532.960571 }; ///< Receiver position simulated by skydel
Eigen::Vector3d lla_refRecvPos;

/// @brief SppReference for SPP calcualtion
struct SppReference
{
    /// @brief Constructor
    /// @param[in] freq GNSS frequency
    /// @param[in] satNum Satellite number (SVID)
    /// @param[in] code Signal code
    /// @param[in] path Path to the reference file
    SppReference(Frequency freq, uint16_t satNum, Code code, const std::string& path)
        : satSigId(freq, satNum), code(code), fs("test/data/Skydel-static_4h_1min-rate/" + path, std::ios_base::binary)
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

std::vector<SppReference> sppReference; ///< SppReference files

TEST_CASE("[SinglePointPositioning][flow] SPP with Skydel data (GPS L1 C/A - Klobuchar - Saastamoinen)", "[SinglePointPositioning][flow]")
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

    lla_refRecvPos = trafo::ecef2lla_WGS84(e_refRecvPos);
    LOG_DEBUG("lla_refRecvPos {}, {}, {}", rad2deg(lla_refRecvPos.x()), rad2deg(lla_refRecvPos.y()), lla_refRecvPos.z());

    sppReference.clear();
    sppReference.emplace_back(G01, 1, Code::G1C, "L1CA 01.csv");
    sppReference.emplace_back(G01, 3, Code::G1C, "L1CA 03.csv");
    sppReference.emplace_back(G01, 5, Code::G1C, "L1CA 05.csv");
    sppReference.emplace_back(G01, 7, Code::G1C, "L1CA 07.csv");
    sppReference.emplace_back(G01, 8, Code::G1C, "L1CA 08.csv");
    sppReference.emplace_back(G01, 10, Code::G1C, "L1CA 10.csv");
    sppReference.emplace_back(G01, 14, Code::G1C, "L1CA 14.csv");
    sppReference.emplace_back(G01, 16, Code::G1C, "L1CA 16.csv");
    sppReference.emplace_back(G01, 17, Code::G1C, "L1CA 17.csv");
    sppReference.emplace_back(G01, 18, Code::G1C, "L1CA 18.csv");
    sppReference.emplace_back(G01, 20, Code::G1C, "L1CA 20.csv");
    sppReference.emplace_back(G01, 21, Code::G1C, "L1CA 21.csv");
    sppReference.emplace_back(G01, 22, Code::G1C, "L1CA 22.csv");
    sppReference.emplace_back(G01, 23, Code::G1C, "L1CA 23.csv");
    sppReference.emplace_back(G01, 26, Code::G1C, "L1CA 26.csv");
    sppReference.emplace_back(G01, 27, Code::G1C, "L1CA 27.csv");
    sppReference.emplace_back(G01, 28, Code::G1C, "L1CA 28.csv");
    sppReference.emplace_back(G01, 32, Code::G1C, "L1CA 32.csv");

    nm::RegisterWatcherCallbackToInputPin(72, [](const Node* /* node */, const InputPin::NodeDataQueue& queue, size_t /* pinIdx */) { // SppSolutionExtended
        messageCounter++;
        auto sppSol = std::dynamic_pointer_cast<const NAV::SppSolutionExtended>(queue.front());

        for (auto& ref : sppReference)
        {
            auto pos = ref.fs.tellg();
            std::string line;
            std::getline(ref.fs, line);
            if (line.empty())
            {
                continue;
            }

            //  0 | Elapsed Time (ms)                                       | The elapsed time of the simulation in milliseconds.
            //  1 | ECEF X (m)                                              | ECEF coordinates (meters) of the origin of the transmitted signal (satellite’s antenna phase center plus errors).
            //  2 | ECEF Y (m)                                              | ECEF coordinates (meters) of the origin of the transmitted signal (satellite’s antenna phase center plus errors).
            //  3 | ECEF Z (m)                                              | ECEF coordinates (meters) of the origin of the transmitted signal (satellite’s antenna phase center plus errors).
            //  4 | ECEF Error X (m)                                        | ECEF coordinates errors (offset in meters) from the satellite’s antenna phase center.
            //  5 | ECEF Error Y (m)                                        | ECEF coordinates errors (offset in meters) from the satellite’s antenna phase center.
            //  6 | ECEF Error Z (m)                                        | ECEF coordinates errors (offset in meters) from the satellite’s antenna phase center.
            //  7 | Body Azimuth (rad)                                      | Satellite’s azimuth, in radians, from the receiver’s body position relative to North.
            //  8 | Body Elevation (rad)                                    | Satellite elevation, in radians, from the receiver’s body position relative to the horizon.
            //  9 | Range (m)                                               | Geometrical distance, in meters, between the satellite’s and receiver’s antennas.
            // 10 | PSR (m)                                                 | Pseudorange, in meters, between the satellite’s and receiver’s antennas.
            // 11 | ADR                                                     | Accumulated Doppler range, in number of cycles, between the satellite and receiver.
            // 12 | Clock Correction (s)                                    | Satellite’s clock correction, in seconds.
            // 13 | Clock Noise (m)                                         | Additional clock error, in meters, not accounted for in navigation message.
            // 14 | Delta Af0 (s)                                           | Clock offset in seconds.
            // 15 | Delta Af1 (s/s)                                         | Clock drift in seconds per seconds.
            // 16 | Iono Correction (m)                                     | Ionospheric corrections, in meters.
            // 17 | Tropo Correction (m)                                    | Tropospheric corrections, in meters.
            // 18 | PSR Offset (m)                                          | Pseudorange offset, in meters.
            // 19 | Receiver Antenna Azimuth (rad)                          | Satellite’s azimuth, in radians, from the receiver’s antenna position.
            // 20 | Receiver Antenna Elevation (rad)                        | Satellite’s elevation, in radians, from the receiver’s antenna position.
            // 21 | Receiver Antenna Gain (dBi)                             | Receiver’s antenna gain, in dBi.
            // 22 | SV Antenna Azimuth (rad)                                | Receiver’s azimuth, in radians, from the satellite’s antenna position.
            // 23 | SV Antenna Elevation (rad)                              | Receiver’s elevation, in radians, from the satellite’s antenna position.
            // 24 | Relative Power Level (incl. Receiver Antenna gain) (dB) | Signal’s relative power level, in dB, corresponding to the sum of the global power offset, the user’s power offset, the receiver’s antenna gain and the satellite’s antenna gain.
            // 25 | Doppler Frequency (Hz)                                  | Doppler frequency, in Hertz, due to satellites' and receivers' antennas dynamics'.
            // 26 | PSR Change Rate (m/s)                                   | Pseudorange rate, in meters per second, due to satellites' and receivers' antennas dynamics'.
            // 27 | Receiver Carrier Phase Offset (rad)                     | Phase offset, in radians, caused by the receiver’s antenna phase pattern. This column does not appear in echo log files.
            // 28 | Satellite Carrier Phase Offset (rad)                    | Phase offset, in radians, caused by the satellite’s antenna phase pattern. This column does not appear in echo log files.
            // 29 | GPS TOW                                                 | GPS time of week, in seconds.
            // 30 | GPS Week Number                                         | GPS week number.
            // 31 | SBAS t0                                                 | SBAS time of the day, in seconds.
            // 32 | Calibration offset (m)                                  | Offset, in meters, applied to the signal during Wavefront simulation. Used to compensate for hardware differences between elements.
            // 33 | PSR satellite time (ms)                                 | The elapsed time of the simulation when the signal was emitted from the satellite, in milliseconds.
            std::vector<std::string> v = str::split(line, ",");

            InsTime refRecvTime = InsTime(0, std::stoi(v[30]), std::stold(v[29])); // GPS Week Number, GPS TOW [s]
            REQUIRE(sppSol->insTime <= refRecvTime);

            if (sppSol->insTime == refRecvTime)
            {
                LOG_DEBUG("line: {}", line);
                LOG_DEBUG("[{}][{}-{}] Processing line in file {}", refRecvTime.toYMDHMS(), ref.satSigId, ref.code, ref.counter + 2);
                auto iter = std::find_if(sppSol->extData.begin(), sppSol->extData.end(), [&ref](const SppSolutionExtended::ExtendedData& extData) {
                    return extData.satSigId == ref.satSigId && extData.code == ref.code;
                });
                REQUIRE(iter != sppSol->extData.end()); // This means something was calculated for the satellite
                ref.counter++;

                LOG_DEBUG("    e_refRecvPos         {} [m]", e_refRecvPos.transpose());
                LOG_DEBUG("    sppSol->e_position() {} [m]", sppSol->e_position().transpose());
                Eigen::Vector3d n_diff = trafo::n_Quat_e(lla_refRecvPos.x(), lla_refRecvPos.y()) * (sppSol->e_position() - e_refRecvPos);
                double hDist = n_diff.head<2>().norm();
                double vDist = n_diff.z();
                LOG_DEBUG("    hDist {} [m]", hDist);
                LOG_DEBUG("    vDist {} [m]", vDist);
                REQUIRE(hDist < 1.7);
                REQUIRE(std::abs(vDist) < 1.7); // TODO: This is way too large

                const auto& calcData = (*sppSol)(ref.satSigId.freq, ref.satSigId.satNum, ref.code);

                if (calcData.skipped)
                {
                    continue;
                }

                Eigen::Vector3d e_refSatPos(std::stod(v[1]), std::stod(v[2]), std::stod(v[3]));
                LOG_DEBUG("    calcData.e_satPos {} [m]", calcData.e_satPos.transpose());
                LOG_DEBUG("    e_refSatPos       {} [m]", e_refSatPos.transpose());
                LOG_DEBUG("    | calcData.pos - e_refPos | = {} [m]", (calcData.e_satPos - e_refSatPos).norm());
                REQUIRE_THAT(calcData.e_satPos - e_refSatPos, Catch::Matchers::WithinAbs(Eigen::Vector3d{ 0.0, 0.0, 0.0 }, 3e-4));

                // e_satVel

                double refClkCorrection = std::stod(v[12]); // Satellite’s clock correction [s]
                LOG_DEBUG("    calcData.satClkBias {} [s]", calcData.satClkBias);
                LOG_DEBUG("    refClkCorrection    {} [s]", refClkCorrection);
                LOG_DEBUG("    clkBias - ref {} [s]", calcData.satClkBias - refClkCorrection);
                REQUIRE_THAT(calcData.satClkBias - refClkCorrection, Catch::Matchers::WithinAbs(0.0, 4e-15));

                // satClkDrift

                double refSatElevation = std::stod(v[20]); // Satellite’s elevation, in radians, from the receiver’s antenna position.
                LOG_DEBUG("    calcData.satElevation {} [deg]", rad2deg(calcData.satElevation));
                LOG_DEBUG("    refSatElevation       {} [deg]", rad2deg(refSatElevation));
                LOG_DEBUG("    calcData.satElevation - refSatElevation {} [°]", rad2deg(calcData.satElevation - refSatElevation));
                REQUIRE_THAT(rad2deg(calcData.satElevation - refSatElevation), Catch::Matchers::WithinAbs(0.0, 3e-3));

                double refSatAzimuth = std::stod(v[19]); // Satellite’s azimuth, in radians, from the receiver’s antenna position.
                LOG_DEBUG("    calcData.satAzimuth {} [°]", rad2deg(calcData.satAzimuth));
                LOG_DEBUG("    refSatAzimuth       {} [°]", rad2deg(refSatAzimuth));
                LOG_DEBUG("    calcData.satAzimuth - refSatAzimuth {} [°]", rad2deg(calcData.satAzimuth - refSatAzimuth));
                REQUIRE_THAT(rad2deg(calcData.satAzimuth - refSatAzimuth), Catch::Matchers::WithinAbs(0.0, 3e-3));

                if (calcData.elevationMaskTriggered)
                {
                    REQUIRE(std::isnan(calcData.pseudorangeRate));
                    REQUIRE(std::isnan(calcData.dpsr_I));
                    REQUIRE(std::isnan(calcData.dpsr_T));
                    REQUIRE(std::isnan(calcData.geometricDist));
                }
                else
                {
                    double timeDiffRange_ref = (std::stod(v[0]) - std::stod(v[33])) * 1e-3;                            // [s]
                    double timeDiffRecvTrans = static_cast<double>((sppSol->insTime - calcData.transmitTime).count()); // [s]
                    timeDiffRecvTrans += calcData.dpsr_I / InsConst::C;
                    timeDiffRecvTrans += calcData.dpsr_T / InsConst::C; // TODO: Do we need to add the values here?
                    LOG_DEBUG("    timeDiffRecvTrans {} [s]", timeDiffRecvTrans);
                    LOG_DEBUG("    timeDiffRange_ref {} [s]", timeDiffRange_ref);
                    LOG_DEBUG("    timeDiffRecvTrans - timeDiffRange_ref {} [s]", timeDiffRecvTrans - timeDiffRange_ref);
                    // TODO: This is not sufficient yet
                    REQUIRE_THAT(timeDiffRecvTrans - timeDiffRange_ref, Catch::Matchers::WithinAbs(0.0, 1e-3));

                    // pseudorangeRate

                    double refIonoCorrection = std::stod(v[16]); // Ionospheric corrections [m]
                    LOG_DEBUG("    calcData.dpsr_I   {} [m]", calcData.dpsr_I);
                    LOG_DEBUG("    refIonoCorrection {} [m]", refIonoCorrection);
                    LOG_DEBUG("    calcData.dpsr_I - refIonoCorrection = {} [m]", calcData.dpsr_I - refIonoCorrection);
                    REQUIRE_THAT(calcData.dpsr_I - refIonoCorrection, Catch::Matchers::WithinAbs(0.0, 4e-2));

                    double refTropoCorrection = std::stod(v[17]); // Tropospheric corrections [m]
                    LOG_DEBUG("    calcData.dpsr_T    {} [m]", calcData.dpsr_T);
                    LOG_DEBUG("    refTropoCorrection {} [m]", refTropoCorrection);
                    LOG_DEBUG("    calcData.dpsr_T - refTropoCorrection = {} [m]", calcData.dpsr_T - refTropoCorrection);
                    REQUIRE_THAT(calcData.dpsr_T - refTropoCorrection, Catch::Matchers::WithinAbs(0.0, 5e-1));

                    double refGeometricDist = std::stod(v[9]); // Geometrical distance [m] between the satellite’s and receiver’s antennas.
                    LOG_DEBUG("    calcData.geometricDist {} [m]", calcData.geometricDist);
                    LOG_DEBUG("    refGeometricDist       {} [m]", refGeometricDist);
                    LOG_DEBUG("    calcData.geometricDist - refGeometricDist = {} [m]", calcData.geometricDist - refGeometricDist);
                    // TODO: this is very high
                    REQUIRE_THAT(calcData.geometricDist - refGeometricDist, Catch::Matchers::WithinAbs(0.0, 180));
                }
            }
            else // if (sppSol->insTime < refRecvTime)
            {
                ref.fs.seekg(pos);
            }
        }
    });

    messageCounter = 0;

    REQUIRE(testFlow("test/flow/Nodes/DataProcessor/GNSS/SPP-Skydel.flow"));

    CHECK(messageCounter == MESSAGE_COUNT_EXPECTED);
    for (auto& ref : sppReference)
    {
        LOG_DEBUG("Checking if all messages from satellite {}-{} were read.", ref.satSigId, ref.code);
        REQUIRE(ref.counter == ref.dataCnt);
    }
}

} // namespace NAV::TESTS::SinglePointPositioningTests
