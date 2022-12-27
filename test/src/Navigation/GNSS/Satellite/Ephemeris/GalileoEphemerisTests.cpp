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
#include "Navigation/GNSS/Satellite/Ephemeris/GalileoEphemeris.hpp"
#include "Logger.hpp"
#include "util/StringUtil.hpp"

#include <fstream>

namespace NAV::TESTS::EphemerisTests
{

TEST_CASE("[Ephemeris] GAL Ephemeris calc orbit Skydel data", "[Ephemeris]")
{
    auto logger = initializeTestLogger();

    // E02 2022 06 01 14 00 00 2.283233505978E-04 2.629008122312E-12 0.000000000000E+00
    //      1.400000000000E+01 1.748750000000E+02 2.609037248302E-09 2.580226855533E+00
    //      8.022412657738E-06 2.002019900829E-04 1.190602779388E-05 5.440609741211E+03
    //      3.096000000000E+05 6.332993507385E-08 2.910557133736E-01-1.303851604462E-08
    //      9.781193191757E-01 9.403125000000E+01-2.278966187518E-01-5.099855286586E-09
    //     -2.610823036973E-10 3.000000000000E+00 2.212000000000E+03 0.000000000000E+00
    //      3.120000000000E+00 0.000000000000E+00-1.164153218269E-09-2.095475792885E-09
    //      9.999000000000E+08 0.000000000000E+00 0.000000000000E+00 0.000000000000E+00
    GalileoEphemeris eph(InsTime(2022, 6, 1, 14, 00, 0, GST),                            // toc
                         InsTime(0, 2212, 3.096000000000e+05),                           // toe
                         14,                                                             // IODnav
                         { 2.283233505978e-04, 2.629008122312e-12, 0.000000000000e+00 }, // a
                         5.440609741211e+03,                                             // sqrt_A
                         2.002019900829e-04,                                             // e
                         9.781193191757E-01,                                             // i_0
                         2.910557133736E-01,                                             // Omega_0
                         -2.278966187518e-01,                                            // omega
                         2.580226855533e+00,                                             // M_0
                         2.609037248302e-09,                                             // delta_n
                         -5.099855286586e-09,                                            // Omega_dot
                         -2.610823036973e-10,                                            // i_dot
                         1.190602779388e-05,                                             // Cus
                         8.022412657738e-06,                                             // Cuc
                         -1.303851604462e-08,                                            // Cis
                         6.332993507385e-08,                                             // Cic
                         1.748750000000e+02,                                             // Crs
                         9.403125000000e+01,                                             // Crc
                         3,                                                              // dataSource
                         3.12,                                                           // signalAccuracy
                         {},                                                             // svHealth
                         -1.164153218269E-09,                                            // BGD_E1_E5a
                         -2.095475792885E-09                                             // BGD_E1_E5b
    );

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

        double timeDiff = static_cast<double>((recvTime - eph.toc).count());
        if (std::abs(timeDiff) > 5 * InsTimeUtil::SECONDS_PER_MINUTE + 1)
        {
            continue;
        }
        LOG_DATA("timeDiff {} [s]", timeDiff);

        auto satClk = eph.calcClockCorrections(recvTime, distance, E05);

        double refClkCorrection = std::stod(v[12]); // Satellite’s clock correction [s]
        LOG_DATA("    clkBias          {}", satClk.bias);
        LOG_DATA("    refClkCorrection {}", refClkCorrection);
        LOG_DATA("    clkBias - ref {}", satClk.bias - refClkCorrection);
        REQUIRE_THAT(satClk.bias, Catch::Matchers::WithinAbs(refClkCorrection, 8e-12));

        auto pos = eph.calcSatellitePos(satClk.transmitTime); // FIXME: Skydel uses GPS µ to generate the GAL ephemeris. Replace the test data if this is fixed and replace the SatSys here

        Eigen::Vector3d e_refPos(std::stod(v[1]), std::stod(v[2]), std::stod(v[3]));
        LOG_DATA("    e_refPos {}", e_refPos.transpose());
        LOG_DATA("    pos        {}", pos.e_pos.transpose());
        LOG_DATA("    | pos - e_refPos | = {}", (pos.e_pos - e_refPos).norm());
        REQUIRE_THAT(pos.e_pos, Catch::Matchers::WithinAbs(e_refPos, 1e-4));
    }
}

} // namespace NAV::TESTS::EphemerisTests