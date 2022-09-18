// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <catch2/catch.hpp>
#include "EigenApprox.hpp"

#include "Navigation/Constants.hpp"
#include "Navigation/GNSS/Functions.hpp"
#include "Navigation/GNSS/NavigationMessage/Ephemeris.hpp"
#include "util/Logger.hpp"
#include "util/StringUtil.hpp"

#include <fstream>

namespace NAV::TEST::EphemerisTests
{

TEST_CASE("[Ephemeris] GPS Ephemeris calc orbit Skydel data", "[Ephemeris]")
{
    Logger consoleSink;

    // G01 2022 06 01 14 00 00 6.486773490906E-04-1.170974428533E-11 0.000000000000E+00
    //      2.000000000000E+00-1.421562500000E+02 3.988023260033E-09-4.307765361283E-01
    //     -7.430091500282E-06 1.102848351002E-02 4.675239324570E-06 5.153680524826E+03
    //      3.096000000000E+05 2.104789018631E-07 2.491970850983E+00 1.713633537292E-07
    //      9.847163497771E-01 2.991875000000E+02 8.641417858768E-01-7.969260523117E-09
    //     -4.482329564161E-10 1.000000000000E+00 2.212000000000E+03 1.000000000000E+00
    //      2.000000000000E+00 0.000000000000E+00 5.122274160385E-09 2.000000000000E+00
    //      9.999000000000E+08 4.000000000000E+00
    GPSEphemeris eph;
    // -------------------------------------- SV / EPOCH / SV CLK ----------------------------------------
    eph.toc = InsTime(2022, 6, 1, 14, 0, 0, GPST);
    eph.a = { 6.486773490906e-04, -1.170974428533e-11, 0.000000000000e+00 };
    // ------------------------------------ BROADCAST ORBIT - 1 --------------------------------------
    eph.Crs = -1.421562500000e+02;
    eph.delta_n = 3.988023260033e-09;
    eph.M_0 = -4.307765361283e-01;
    // ------------------------------------ BROADCAST ORBIT - 2 --------------------------------------
    eph.Cuc = -7.430091500282e-06;
    eph.e = 1.102848351002e-02;
    eph.Cus = 4.675239324570e-06;
    eph.sqrt_A = 5.153680524826e+03;
    // ------------------------------------ BROADCAST ORBIT - 3 --------------------------------------
    eph.toe = InsTime(0, 2212, 3.096000000000e+05);
    eph.Cic = 2.104789018631e-07;
    eph.Omega_0 = 2.491970850983e+00;
    eph.Cis = 1.713633537292e-07;
    // ------------------------------------ BROADCAST ORBIT - 4 --------------------------------------
    eph.i_0 = 9.847163497771e-01;
    eph.Crc = 2.991875000000e+02;
    eph.omega = 8.641417858768e-01;
    eph.Omega_dot = -7.969260523117e-09;
    // ------------------------------------ BROADCAST ORBIT - 5 --------------------------------------
    eph.i_dot = -4.482329564161E-10;
    // ------------------------------------ BROADCAST ORBIT - 6 --------------------------------------
    eph.T_GD = 5.122274160385E-09;

    std::ifstream fs{ "test/data/Skydel-static_4h_1min-rate/L1C 01.csv", std::ios_base::binary };
    REQUIRE(fs.good());

    std::string line;
    std::getline(fs, line); // Header line

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

    while (!fs.eof() && std::getline(fs, line) && !line.empty())
    {
        std::vector<std::string> v = str::split(line, ",");
        REQUIRE(v.size() == 34);

        double timeDiffRecvTrans_ref = (std::stod(v[0]) - std::stod(v[33])) * 1e-3; // [s] = 'Elapsed Time' - 'PSR satellite time'
        double distance = timeDiffRecvTrans_ref * InsConst::C;
        InsTime recvTime(0, std::stoi(v[30]), std::stold(v[29])); // GPS Week Number, GPS TOW [s]

        double toe = static_cast<double>((recvTime - eph.toc).count());
        if (std::abs(toe) > 5 * InsTimeUtil::SECONDS_PER_MINUTE + 1)
        {
            continue;
        }
        LOG_DATA("eph.toc {} - {}", eph.toc.toGPSweekTow(), eph.toc.toYMDHMS(GPST));
        LOG_DATA("recvTime {} - {}", recvTime.toGPSweekTow(), recvTime.toYMDHMS(GPST));
        LOG_DATA("toe {}", toe);

        auto satClk = eph.calcSatelliteClockCorrections(recvTime, distance, G01);

        double clkCorrection_ref = std::stod(v[12]); // Satellite’s clock correction [s]
        LOG_DATA("    clkBias           {}", satClk.bias);
        LOG_DATA("    clkCorrection_ref {}", clkCorrection_ref);
        LOG_DATA("    clkBias - ref {}", satClk.bias - clkCorrection_ref);
        REQUIRE(satClk.bias == Approx(clkCorrection_ref).margin(3e-15).epsilon(0));

        Eigen::Vector3d pos;
        eph.calcSatellitePosVelAccel(satClk.transmitTime, GPS, &pos, nullptr, nullptr);

        Eigen::Vector3d e_refPos(std::stod(v[1]), std::stod(v[2]), std::stod(v[3]));
        LOG_DATA("    e_refPos {}", e_refPos.transpose());
        LOG_DATA("    pos        {}", pos.transpose());
        LOG_DATA("    | pos - e_refPos | = {}", (pos - e_refPos).norm());
        REQUIRE(pos == EigApprox(e_refPos).margin(2e-4).epsilon(0));
    }
}

TEST_CASE("[Ephemeris] GAL Ephemeris calc orbit Skydel data", "[Ephemeris]")
{
    Logger consoleSink;

    // E02 2022 06 01 14 00 00 2.283233505978E-04 2.629008122312E-12 0.000000000000E+00
    //      1.400000000000E+01 1.748750000000E+02 2.609037248302E-09 2.580226855533E+00
    //      8.022412657738E-06 2.002019900829E-04 1.190602779388E-05 5.440609741211E+03
    //      3.096000000000E+05 6.332993507385E-08 2.910557133736E-01-1.303851604462E-08
    //      9.781193191757E-01 9.403125000000E+01-2.278966187518E-01-5.099855286586E-09
    //     -2.610823036973E-10 3.000000000000E+00 2.212000000000E+03 0.000000000000E+00
    //      3.120000000000E+00 0.000000000000E+00-1.164153218269E-09-2.095475792885E-09
    //      9.999000000000E+08 0.000000000000E+00 0.000000000000E+00 0.000000000000E+00
    GPSEphemeris eph;
    // ------------------------------------ SV / EPOCH / SV CLK --------------------------------------
    eph.toc = InsTime(2022, 6, 1, 14, 00, 0, GST);
    eph.a = { 2.283233505978e-04, 2.629008122312e-12, 0.000000000000e+00 };
    // ------------------------------------ BROADCAST ORBIT - 1 --------------------------------------
    eph.Crs = 1.748750000000e+02;
    eph.delta_n = 2.609037248302e-09;
    eph.M_0 = 2.580226855533e+00;
    // ------------------------------------ BROADCAST ORBIT - 2 --------------------------------------
    eph.Cuc = 8.022412657738e-06;
    eph.e = 2.002019900829e-04;
    eph.Cus = 1.190602779388e-05;
    eph.sqrt_A = 5.440609741211e+03;
    // ------------------------------------ BROADCAST ORBIT - 3 --------------------------------------
    eph.toe = InsTime(0, 2212, 3.096000000000e+05);
    eph.Cic = 6.332993507385e-08;
    eph.Omega_0 = 2.910557133736E-01;
    eph.Cis = -1.303851604462e-08;
    // ------------------------------------ BROADCAST ORBIT - 4 --------------------------------------
    eph.i_0 = 9.781193191757E-01;
    eph.Crc = 9.403125000000e+01;
    eph.omega = -2.278966187518e-01;
    eph.Omega_dot = -5.099855286586e-09;
    // ------------------------------------ BROADCAST ORBIT - 5 --------------------------------------
    eph.i_dot = -2.610823036973e-10;
    // ------------------------------------ BROADCAST ORBIT - 6 --------------------------------------
    eph.BGD_E1_E5a = -1.164153218269E-09;
    eph.BGD_E1_E5b = -2.095475792885E-09;

    std::ifstream fs{ "test/data/Skydel-static_4h_1min-rate/E1 02.csv" };
    REQUIRE(fs.good());

    std::string line;
    std::getline(fs, line); // Header line

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

    while (!fs.eof() && std::getline(fs, line) && !line.empty())
    {
        std::vector<std::string> v = str::split(line, ",");

        double timeDiffRecvTrans_ref = (std::stod(v[0]) - std::stod(v[33])) * 1e-3; // [s] = 'Elapsed Time' - 'PSR satellite time'
        double distance = timeDiffRecvTrans_ref * InsConst::C;
        InsTime recvTime(0, std::stoi(v[30]), std::stold(v[29])); // GPS Week Number, GPS TOW [s]

        double toe = static_cast<double>((recvTime - eph.toc).count());
        if (std::abs(toe) > 5 * InsTimeUtil::SECONDS_PER_MINUTE + 1)
        {
            continue;
        }
        LOG_DATA("toe {}", toe);

        auto satClk = eph.calcSatelliteClockCorrections(recvTime, distance, E05);

        double refClkCorrection = std::stod(v[12]); // Satellite’s clock correction [s]
        LOG_DATA("    clkBias          {}", satClk.bias);
        LOG_DATA("    refClkCorrection {}", refClkCorrection);
        LOG_DATA("    clkBias - ref {}", satClk.bias - refClkCorrection);
        REQUIRE(satClk.bias == Approx(refClkCorrection).margin(8e-12).epsilon(0));

        Eigen::Vector3d pos;
        eph.calcSatellitePosVelAccel(satClk.transmitTime, GPS, &pos, nullptr, nullptr); // FIXME: Skydel uses GPS µ to generate the GAL ephemeris. Replace the test data if this is fixed and replace the SatSys here

        Eigen::Vector3d e_refPos(std::stod(v[1]), std::stod(v[2]), std::stod(v[3]));
        LOG_DATA("    e_refPos {}", e_refPos.transpose());
        LOG_DATA("    pos        {}", pos.transpose());
        LOG_DATA("    | pos - e_refPos | = {}", (pos - e_refPos).norm());
        REQUIRE(pos == EigApprox(e_refPos).margin(1e-4).epsilon(0));
    }
}

// TEST_CASE("[Ephemeris] GLO Ephemeris calc orbit Skydel data", "[Ephemeris]")
// {
//     Logger consoleSink;

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
//         REQUIRE(clkBias == Approx(refClkCorrection).margin(3e-9).epsilon(0));

//         Eigen::Vector3d pos;
//         double clkBias{};
//         InsTime transmitTime;
//         eph.calcSatellitePosVelAccelClk(recvTime, distance, GLO, &clkBias, nullptr, &pos, nullptr, nullptr, &transmitTime);

//         Eigen::Vector3d e_refPos(std::stod(v[1]), std::stod(v[2]), std::stod(v[3]));
//         LOG_DATA("    e_refPos {}", e_refPos.transpose());
//         LOG_DATA("    pos        {}", pos.transpose());
//         LOG_DATA("    | pos - e_refPos | = {}", (pos - e_refPos).norm());

//         REQUIRE(pos == EigApprox(e_refPos).margin(1e-4).epsilon(0));
//     }
// }

} // namespace NAV::TEST::EphemerisTests