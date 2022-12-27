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
#include "Navigation/GNSS/Satellite/Ephemeris/GPSEphemeris.hpp"
#include "Logger.hpp"
#include "util/StringUtil.hpp"

#include <fstream>

namespace NAV::TESTS::EphemerisTests
{

TEST_CASE("[Ephemeris] GPS Ephemeris calc orbit Skydel data", "[Ephemeris]")
{
    auto logger = initializeTestLogger();

    // G01 2022 06 01 14 00 00 6.486773490906E-04-1.170974428533E-11 0.000000000000E+00
    //      2.000000000000E+00-1.421562500000E+02 3.988023260033E-09-4.307765361283E-01
    //     -7.430091500282E-06 1.102848351002E-02 4.675239324570E-06 5.153680524826E+03
    //      3.096000000000E+05 2.104789018631E-07 2.491970850983E+00 1.713633537292E-07
    //      9.847163497771E-01 2.991875000000E+02 8.641417858768E-01-7.969260523117E-09
    //     -4.482329564161E-10 1.000000000000E+00 2.212000000000E+03 1.000000000000E+00
    //      2.000000000000E+00 0.000000000000E+00 5.122274160385E-09 2.000000000000E+00
    //      9.999000000000E+08 4.000000000000E+00
    GPSEphemeris eph(InsTime(2022, 6, 1, 14, 0, 0, GPST),                             // toc
                     InsTime(0, 2212, 3.096000000000e+05),                            // toe
                     0,                                                               // IODE
                     0,                                                               // IODC
                     { 6.486773490906e-04, -1.170974428533e-11, 0.000000000000e+00 }, // a
                     5.153680524826e+03,                                              // sqrt_A
                     1.102848351002e-02,                                              // e
                     9.847163497771e-01,                                              // i_0
                     2.491970850983e+00,                                              // Omega_0
                     8.641417858768e-01,                                              // omega
                     -4.307765361283e-01,                                             // M_0
                     3.988023260033e-09,                                              // delta_n
                     -7.969260523117e-09,                                             // Omega_dot
                     -4.482329564161E-10,                                             // i_dot
                     4.675239324570e-06,                                              // Cus
                     -7.430091500282e-06,                                             // Cuc
                     1.713633537292e-07,                                              // Cis
                     2.104789018631e-07,                                              // Cic
                     -1.421562500000e+02,                                             // Crs
                     2.991875000000e+02,                                              // Crc
                     0,                                                               // svAccuracy
                     0,                                                               // svHealth
                     0,                                                               // L2ChannelCodes
                     false,                                                           // L2DataFlagPCode
                     5.122274160385E-09,                                              // T_GD
                     0);                                                              // fitInterval

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

        auto satClk = eph.calcClockCorrections(recvTime, distance, G01);

        double clkCorrection_ref = std::stod(v[12]); // Satellite’s clock correction [s]
        LOG_DATA("    clkBias           {}", satClk.bias);
        LOG_DATA("    clkCorrection_ref {}", clkCorrection_ref);
        LOG_DATA("    clkBias - ref {}", satClk.bias - clkCorrection_ref);
        REQUIRE_THAT(satClk.bias, Catch::Matchers::WithinAbs(clkCorrection_ref, 3e-15));

        auto pos = eph.calcSatellitePos(satClk.transmitTime);

        Eigen::Vector3d e_refPos(std::stod(v[1]), std::stod(v[2]), std::stod(v[3]));
        LOG_DATA("    e_refPos {}", e_refPos.transpose());
        LOG_DATA("    pos        {}", pos.e_pos.transpose());
        LOG_DATA("    | pos - e_refPos | = {}", (pos.e_pos - e_refPos).norm());
        REQUIRE_THAT(pos.e_pos, Catch::Matchers::WithinAbs(e_refPos, 2e-4));
    }
}

} // namespace NAV::TESTS::EphemerisTests