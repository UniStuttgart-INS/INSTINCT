// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"

#include "Navigation/Constants.hpp"
#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"
#include "Logger.hpp"
#include "util/StringUtil.hpp"

#include <fstream>

namespace NAV::TESTS::EphemerisTests
{

// TEST_CASE("[Ephemeris] GLO Ephemeris calc orbit Skydel data", "[Ephemeris]")
// {
//     auto logger = initializeTestLogger();

//     // R01 2022 06 01 13 45 00 8.010864260000E-05 0.000000000000E+00 3.078000000000E+05
//     //     -7.702946198884E+02-3.037421299509E+00 0.000000000000E+00 0.000000000000E+00
//     //      1.119107394905E+04-7.764658397071E-01 0.000000000000E+00 1.000000000000E+00
//     //      2.291341259651E+04 2.787256343960E-01 0.000000000000E+00 0.000000000000E+00
//     GLONASSEphemeris eph;
//     // ------------------------------------ SV / EPOCH / SV CLK --------------------------------------
//     eph.toc = InsTime(2022, 6, 1, 13, 45, 0, GLNT);
//     eph.tau_n = 8.010864260000E-05;
//     eph.gamma_n = 0.000000000000E+00;
//     // ------------------------------------ BROADCAST ORBIT - 1 --------------------------------------
//     eph.pos.x() = -7.702946198884E+02 * 1e3;
//     eph.vel.x() = -3.037421299509E+00 * 1e3;
//     eph.accelLuniSolar.x() = 0.000000000000E+00 * 1e3;
//     // ------------------------------------ BROADCAST ORBIT - 2 --------------------------------------
//     eph.pos.y() = 1.119107394905E+04 * 1e3;
//     eph.vel.y() = -7.764658397071E-01 * 1e3;
//     eph.accelLuniSolar.y() = 0.000000000000E+00 * 1e3;
//     eph.frequencyNumber = 1;
//     // ------------------------------------ BROADCAST ORBIT - 3 --------------------------------------
//     eph.pos.z() = 2.291341259651E+04 * 1e3;
//     eph.vel.z() = 2.787256343960E-01 * 1e3;
//     eph.accelLuniSolar.z() = 0.000000000000E+00 * 1e3;

//     std::ifstream fs{ "test/data/NavMsg-Skydel/G1 01.csv" };
//     REQUIRE(fs.good());

//     std::string line;
//     std::getline(fs, line); // Header line

//     //  0 | Elapsed Time (ms)                                       | The elapsed time of the simulation in milliseconds.
//     //  1 | ECEF X (m)                                              | ECEF coordinates (meters) of the origin of the transmitted signal (satellite’s antenna phase center plus errors).
//     //  2 | ECEF Y (m)                                              | ECEF coordinates (meters) of the origin of the transmitted signal (satellite’s antenna phase center plus errors).
//     //  3 | ECEF Z (m)                                              | ECEF coordinates (meters) of the origin of the transmitted signal (satellite’s antenna phase center plus errors).
//     //  4 | ECEF Error X (m)                                        | ECEF coordinates errors (offset in meters) from the satellite’s antenna phase center.
//     //  5 | ECEF Error Y (m)                                        | ECEF coordinates errors (offset in meters) from the satellite’s antenna phase center.
//     //  6 | ECEF Error Z (m)                                        | ECEF coordinates errors (offset in meters) from the satellite’s antenna phase center.
//     //  7 | Body Azimuth (rad)                                      | Satellite’s azimuth, in radians, from the receiver’s body position relative to North.
//     //  8 | Body Elevation (rad)                                    | Satellite elevation, in radians, from the receiver’s body position relative to the horizon.
//     //  9 | Range (m)                                               | Geometrical distance, in meters, between the satellite’s and receiver’s antennas.
//     // 10 | PSR (m)                                                 | Pseudorange, in meters, between the satellite’s and receiver’s antennas.
//     // 11 | ADR                                                     | Accumulated Doppler range, in number of cycles, between the satellite and receiver.
//     // 12 | Clock Correction (s)                                    | Satellite’s clock correction, in seconds.
//     // 13 | Clock Noise (m)                                         | Additional clock error, in meters, not accounted for in navigation message.
//     // 14 | Delta Af0 (s)                                           | Clock offset in seconds.
//     // 15 | Delta Af1 (s/s)                                         | Clock drift in seconds per seconds.
//     // 16 | Iono Correction (m)                                     | Ionospheric corrections, in meters.
//     // 17 | Tropo Correction (m)                                    | Tropospheric corrections, in meters.
//     // 18 | PSR Offset (m)                                          | Pseudorange offset, in meters.
//     // 19 | Receiver Antenna Azimuth (rad)                          | Satellite’s azimuth, in radians, from the receiver’s antenna position.
//     // 20 | Receiver Antenna Elevation (rad)                        | Satellite’s elevation, in radians, from the receiver’s antenna position.
//     // 21 | Receiver Antenna Gain (dBi)                             | Receiver’s antenna gain, in dBi.
//     // 22 | SV Antenna Azimuth (rad)                                | Receiver’s azimuth, in radians, from the satellite’s antenna position.
//     // 23 | SV Antenna Elevation (rad)                              | Receiver’s elevation, in radians, from the satellite’s antenna position.
//     // 24 | Relative Power Level (incl. Receiver Antenna gain) (dB) | Signal’s relative power level, in dB, corresponding to the sum of the global power offset, the user’s power offset, the receiver’s antenna gain and the satellite’s antenna gain.
//     // 25 | Doppler Frequency (Hz)                                  | Doppler frequency, in Hertz, due to satellites' and receivers' antennas dynamics'.
//     // 26 | PSR Change Rate (m/s)                                   | Pseudorange rate, in meters per second, due to satellites' and receivers' antennas dynamics'.
//     // 27 | Receiver Carrier Phase Offset (rad)                     | Phase offset, in radians, caused by the receiver’s antenna phase pattern. This column does not appear in echo log files.
//     // 28 | Satellite Carrier Phase Offset (rad)                    | Phase offset, in radians, caused by the satellite’s antenna phase pattern. This column does not appear in echo log files.
//     // 29 | GPS TOW                                                 | GPS time of week, in seconds.
//     // 30 | GPS Week Number                                         | GPS week number.
//     // 31 | SBAS t0                                                 | SBAS time of the day, in seconds.
//     // 32 | Calibration offset (m)                                  | Offset, in meters, applied to the signal during Wavefront simulation. Used to compensate for hardware differences between elements.
//     // 33 | PSR satellite time (ms)                                 | The elapsed time of the simulation when the signal was emitted from the satellite, in milliseconds.

//     while (!fs.eof() && std::getline(fs, line) && !line.empty())
//     {
//         std::vector<std::string> v = str::split(line, ",");

//         double timeDiffRecvTrans_ref = (std::stod(v[0]) - std::stod(v[33])) * 1e-3; // [s] = 'Elapsed Time' - 'PSR satellite time'
//         double distance = timeDiffRecvTrans_ref * InsConst::C;
//         InsTime recvTime(0, std::stoi(v[30]), std::stold(v[29])); // GPS Week Number, GPS TOW [s]

//         double toe = static_cast<double>((recvTime - eph.toc).count());
//         if (std::abs(toe) > 5 * InsTimeUtil::SECONDS_PER_MINUTE + 1)
//         {
//             continue;
//         }
//         LOG_DATA("toe {}", toe);

//         double refClkCorrection = std::stod(v[12]); // Satellite’s clock correction [s]
//         LOG_DATA("    clkBias          {}", satClk.bias);
//         LOG_DATA("    refClkCorrection {}", refClkCorrection);
//         LOG_DATA("    clkBias - ref {}", satClk.bias - refClkCorrection);
//         REQUIRE_THAT(clkBias, Catch::Matchers::WithinAbs(refClkCorrection, 3e-9);

//         Eigen::Vector3d pos;
//         double clkBias{};
//         InsTime transmitTime;
//         eph.calcSatellitePosVelAccelClk(recvTime, distance, GLO, &clkBias, nullptr, &pos, nullptr, nullptr, &transmitTime);

//         Eigen::Vector3d e_refPos(std::stod(v[1]), std::stod(v[2]), std::stod(v[3]));
//         LOG_DATA("    e_refPos {}", e_refPos.transpose());
//         LOG_DATA("    pos        {}", pos.transpose());
//         LOG_DATA("    | pos - e_refPos | = {}", (pos - e_refPos).norm());

//         REQUIRE_THAT(pos, Catch::Matchers::WithinAbs(e_refPos, 1e-4));
//     }
// }

} // namespace NAV::TESTS::EphemerisTests