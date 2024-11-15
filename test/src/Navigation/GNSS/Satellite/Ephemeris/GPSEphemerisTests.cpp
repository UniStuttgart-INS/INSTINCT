// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GPSEphemerisTests.cpp
/// @brief GPS Ephemersi tests
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-27

#include "Common.hpp"

#include "Navigation/GNSS/Satellite/Ephemeris/GPSEphemeris.hpp"

namespace NAV::TESTS::EphemerisTests
{

TEST_CASE("[Ephemeris] GPS Ephemeris calc orbit (BRDC_20230080000)", "[Ephemeris]")
{
    // G01 - Taken from real data
    // NOLINTBEGIN
    GPSEphemeris eph(2023, 1, 8, 12, 0, 0, 2.270475961268e-04, -4.774847184308e-12, 0.000000000000e+00,
                     1.800000000000e+01, 4.412500000000e+01, 4.154815921903e-09, 9.534843171347e-02,
                     2.287328243256e-06, 1.217866723891e-02, 9.965151548386e-07, 5.153653379440e+03,
                     4.320000000000e+04, -6.891787052155e-08, -1.509394590195e+00, 1.434236764908e-07,
                     9.889891589796e-01, 3.767500000000e+02, 9.377162063410e-01, -8.364991292606e-09,
                     1.185763677531e-10, 1.000000000000e+00, 2.244000000000e+03, 0.000000000000e+00,
                     2.000000000000e+00, 0.000000000000e+00, 4.656612873077e-09, 1.800000000000e+01,
                     3.601800000000e+04, 4.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);
    // NOLINTEND

    // https://igs.org/products/
    // | Broadcast         | Accuracy     |
    // |     - orbits      | ~100 cm      |
    // |     - Sat. clocks | ~5   ns RMS  |
    // |                   | ~2.5 ns SDev |

    Margin margin; // Determined by running the test and adapting
    margin.clock = 2.4e-8;
    margin.pos = 1.6e0;

    testBrdcEphemerisData({ GPS, 1 }, eph, "test/data/GNSS/BRDC_20230080000/COD0OPSFIN_20230080000_01D_05M_ORB.SP3", margin);
}

TEST_CASE("[Ephemeris] GPS Ephemeris calc orbit (Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none)", "[Ephemeris][flow]")
{
    // Margins determined by running the test and adapting (Skydel file has only 16 digits after comma)
    Margin marginL1{ .clock = 3.7e-15, .pos = 1.9e-4, .vel = 0, .accel = 0 };
    Margin marginL2{ .clock = 4.8e-15, .pos = 1.7e-4, .vel = 0, .accel = 0 };
    Margin marginL5{ .clock = 1.3e-8, .pos = 1.9e-4, .vel = 0, .accel = 0 }; // Skydel applies T_GD to the satellite clock, which is wrong according to IS-GPS-705J GPS ICD L5, ch. 20.3.3.3.2.1, p.78

    std::string folder = "test/data/GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/sat_data/";
    std::vector<std::tuple<SatSigId, std::string, Margin>> files = {
        { SatSigId(Code::G1X, 1), folder + "L1C 01.csv", marginL1 },
        { SatSigId(Code::G1X, 3), folder + "L1C 03.csv", marginL1 },
        { SatSigId(Code::G1X, 6), folder + "L1C 06.csv", marginL1 },
        { SatSigId(Code::G1X, 7), folder + "L1C 07.csv", marginL1 },
        { SatSigId(Code::G1X, 8), folder + "L1C 08.csv", marginL1 },
        { SatSigId(Code::G1X, 9), folder + "L1C 09.csv", marginL1 },
        { SatSigId(Code::G1X, 11), folder + "L1C 11.csv", marginL1 },
        { SatSigId(Code::G1X, 13), folder + "L1C 13.csv", marginL1 },
        { SatSigId(Code::G1X, 14), folder + "L1C 14.csv", marginL1 },
        { SatSigId(Code::G1X, 17), folder + "L1C 17.csv", marginL1 },
        { SatSigId(Code::G1X, 19), folder + "L1C 19.csv", marginL1 },
        { SatSigId(Code::G1X, 21), folder + "L1C 21.csv", marginL1 },
        { SatSigId(Code::G1X, 24), folder + "L1C 24.csv", marginL1 },
        { SatSigId(Code::G1X, 30), folder + "L1C 30.csv", marginL1 },
        { SatSigId(Code::G1C, 1), folder + "L1CA 01.csv", marginL1 },
        { SatSigId(Code::G1C, 3), folder + "L1CA 03.csv", marginL1 },
        { SatSigId(Code::G1C, 6), folder + "L1CA 06.csv", marginL1 },
        { SatSigId(Code::G1C, 7), folder + "L1CA 07.csv", marginL1 },
        { SatSigId(Code::G1C, 8), folder + "L1CA 08.csv", marginL1 },
        { SatSigId(Code::G1C, 9), folder + "L1CA 09.csv", marginL1 },
        { SatSigId(Code::G1C, 11), folder + "L1CA 11.csv", marginL1 },
        { SatSigId(Code::G1C, 13), folder + "L1CA 13.csv", marginL1 },
        { SatSigId(Code::G1C, 14), folder + "L1CA 14.csv", marginL1 },
        { SatSigId(Code::G1C, 17), folder + "L1CA 17.csv", marginL1 },
        { SatSigId(Code::G1C, 19), folder + "L1CA 19.csv", marginL1 },
        { SatSigId(Code::G1C, 21), folder + "L1CA 21.csv", marginL1 },
        { SatSigId(Code::G1C, 24), folder + "L1CA 24.csv", marginL1 },
        { SatSigId(Code::G1C, 30), folder + "L1CA 30.csv", marginL1 },
        { SatSigId(Code::G1P, 1), folder + "L1P 01.csv", marginL1 },
        { SatSigId(Code::G1P, 3), folder + "L1P 03.csv", marginL1 },
        { SatSigId(Code::G1P, 6), folder + "L1P 06.csv", marginL1 },
        { SatSigId(Code::G1P, 7), folder + "L1P 07.csv", marginL1 },
        { SatSigId(Code::G1P, 8), folder + "L1P 08.csv", marginL1 },
        { SatSigId(Code::G1P, 9), folder + "L1P 09.csv", marginL1 },
        { SatSigId(Code::G1P, 11), folder + "L1P 11.csv", marginL1 },
        { SatSigId(Code::G1P, 13), folder + "L1P 13.csv", marginL1 },
        { SatSigId(Code::G1P, 14), folder + "L1P 14.csv", marginL1 },
        { SatSigId(Code::G1P, 17), folder + "L1P 17.csv", marginL1 },
        { SatSigId(Code::G1P, 19), folder + "L1P 19.csv", marginL1 },
        { SatSigId(Code::G1P, 21), folder + "L1P 21.csv", marginL1 },
        { SatSigId(Code::G1P, 24), folder + "L1P 24.csv", marginL1 },
        { SatSigId(Code::G1P, 30), folder + "L1P 30.csv", marginL1 },
        { SatSigId(Code::G2C, 1), folder + "L2C 01.csv", marginL2 },
        { SatSigId(Code::G2C, 3), folder + "L2C 03.csv", marginL2 },
        { SatSigId(Code::G2C, 6), folder + "L2C 06.csv", marginL2 },
        { SatSigId(Code::G2C, 7), folder + "L2C 07.csv", marginL2 },
        { SatSigId(Code::G2C, 8), folder + "L2C 08.csv", marginL2 },
        { SatSigId(Code::G2C, 9), folder + "L2C 09.csv", marginL2 },
        { SatSigId(Code::G2C, 11), folder + "L2C 11.csv", marginL2 },
        { SatSigId(Code::G2C, 13), folder + "L2C 13.csv", marginL2 },
        { SatSigId(Code::G2C, 14), folder + "L2C 14.csv", marginL2 },
        { SatSigId(Code::G2C, 17), folder + "L2C 17.csv", marginL2 },
        { SatSigId(Code::G2C, 19), folder + "L2C 19.csv", marginL2 },
        { SatSigId(Code::G2C, 21), folder + "L2C 21.csv", marginL2 },
        { SatSigId(Code::G2C, 24), folder + "L2C 24.csv", marginL2 },
        { SatSigId(Code::G2C, 30), folder + "L2C 30.csv", marginL2 },
        { SatSigId(Code::G2P, 1), folder + "L2P 01.csv", marginL2 },
        { SatSigId(Code::G2P, 3), folder + "L2P 03.csv", marginL2 },
        { SatSigId(Code::G2P, 6), folder + "L2P 06.csv", marginL2 },
        { SatSigId(Code::G2P, 7), folder + "L2P 07.csv", marginL2 },
        { SatSigId(Code::G2P, 8), folder + "L2P 08.csv", marginL2 },
        { SatSigId(Code::G2P, 9), folder + "L2P 09.csv", marginL2 },
        { SatSigId(Code::G2P, 11), folder + "L2P 11.csv", marginL2 },
        { SatSigId(Code::G2P, 13), folder + "L2P 13.csv", marginL2 },
        { SatSigId(Code::G2P, 14), folder + "L2P 14.csv", marginL2 },
        { SatSigId(Code::G2P, 17), folder + "L2P 17.csv", marginL2 },
        { SatSigId(Code::G2P, 19), folder + "L2P 19.csv", marginL2 },
        { SatSigId(Code::G2P, 21), folder + "L2P 21.csv", marginL2 },
        { SatSigId(Code::G2P, 24), folder + "L2P 24.csv", marginL2 },
        { SatSigId(Code::G2P, 30), folder + "L2P 30.csv", marginL2 },
        { SatSigId(Code::G5X, 1), folder + "L5 01.csv", marginL5 },
        { SatSigId(Code::G5X, 3), folder + "L5 03.csv", marginL5 },
        { SatSigId(Code::G5X, 6), folder + "L5 06.csv", marginL5 },
        { SatSigId(Code::G5X, 7), folder + "L5 07.csv", marginL5 },
        { SatSigId(Code::G5X, 8), folder + "L5 08.csv", marginL5 },
        { SatSigId(Code::G5X, 9), folder + "L5 09.csv", marginL5 },
        { SatSigId(Code::G5X, 11), folder + "L5 11.csv", marginL5 },
        { SatSigId(Code::G5X, 13), folder + "L5 13.csv", marginL5 },
        { SatSigId(Code::G5X, 14), folder + "L5 14.csv", marginL5 },
        { SatSigId(Code::G5X, 17), folder + "L5 17.csv", marginL5 },
        { SatSigId(Code::G5X, 19), folder + "L5 19.csv", marginL5 },
        { SatSigId(Code::G5X, 21), folder + "L5 21.csv", marginL5 },
        { SatSigId(Code::G5X, 24), folder + "L5 24.csv", marginL5 },
        { SatSigId(Code::G5X, 30), folder + "L5 30.csv", marginL5 },
    };

    testNavFile(Skydel, "GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/SkydelRINEX_S_20238959_7200S_GN.rnx", files);
}

TEST_CASE("[Ephemeris] GPS Ephemeris calc orbit (Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-Klob_tropo-Saast)", "[Ephemeris][flow]")
{
    // Margins determined by running the test and adapting (Skydel file has only 16 digits after comma)
    Margin marginL1{ .clock = 3.7e-15, .pos = 2.9e-4, .vel = 0, .accel = 0 };
    Margin marginL2{ .clock = 4.8e-15, .pos = 3.7e-4, .vel = 0, .accel = 0 };
    Margin marginL5{ .clock = 1.3e-8, .pos = 3.9e-4, .vel = 0, .accel = 0 }; // Skydel applies T_GD to the satellite clock, which is wrong according to IS-GPS-705J GPS ICD L5, ch. 20.3.3.3.2.1, p.78

    std::string folder = "test/data/GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-Klob_tropo-Saast/sat_data/";
    std::vector<std::tuple<SatSigId, std::string, Margin>> files = {
        { SatSigId(Code::G1X, 1), folder + "L1C 01.csv", marginL1 },
        { SatSigId(Code::G1X, 3), folder + "L1C 03.csv", marginL1 },
        { SatSigId(Code::G1X, 6), folder + "L1C 06.csv", marginL1 },
        { SatSigId(Code::G1X, 7), folder + "L1C 07.csv", marginL1 },
        { SatSigId(Code::G1X, 8), folder + "L1C 08.csv", marginL1 },
        { SatSigId(Code::G1X, 9), folder + "L1C 09.csv", marginL1 },
        { SatSigId(Code::G1X, 11), folder + "L1C 11.csv", marginL1 },
        { SatSigId(Code::G1X, 13), folder + "L1C 13.csv", marginL1 },
        { SatSigId(Code::G1X, 14), folder + "L1C 14.csv", marginL1 },
        { SatSigId(Code::G1X, 17), folder + "L1C 17.csv", marginL1 },
        { SatSigId(Code::G1X, 19), folder + "L1C 19.csv", marginL1 },
        { SatSigId(Code::G1X, 21), folder + "L1C 21.csv", marginL1 },
        { SatSigId(Code::G1X, 24), folder + "L1C 24.csv", marginL1 },
        { SatSigId(Code::G1X, 30), folder + "L1C 30.csv", marginL1 },
        { SatSigId(Code::G1C, 1), folder + "L1CA 01.csv", marginL1 },
        { SatSigId(Code::G1C, 3), folder + "L1CA 03.csv", marginL1 },
        { SatSigId(Code::G1C, 6), folder + "L1CA 06.csv", marginL1 },
        { SatSigId(Code::G1C, 7), folder + "L1CA 07.csv", marginL1 },
        { SatSigId(Code::G1C, 8), folder + "L1CA 08.csv", marginL1 },
        { SatSigId(Code::G1C, 9), folder + "L1CA 09.csv", marginL1 },
        { SatSigId(Code::G1C, 11), folder + "L1CA 11.csv", marginL1 },
        { SatSigId(Code::G1C, 13), folder + "L1CA 13.csv", marginL1 },
        { SatSigId(Code::G1C, 14), folder + "L1CA 14.csv", marginL1 },
        { SatSigId(Code::G1C, 17), folder + "L1CA 17.csv", marginL1 },
        { SatSigId(Code::G1C, 19), folder + "L1CA 19.csv", marginL1 },
        { SatSigId(Code::G1C, 21), folder + "L1CA 21.csv", marginL1 },
        { SatSigId(Code::G1C, 24), folder + "L1CA 24.csv", marginL1 },
        { SatSigId(Code::G1C, 30), folder + "L1CA 30.csv", marginL1 },
        { SatSigId(Code::G1P, 1), folder + "L1P 01.csv", marginL1 },
        { SatSigId(Code::G1P, 3), folder + "L1P 03.csv", marginL1 },
        { SatSigId(Code::G1P, 6), folder + "L1P 06.csv", marginL1 },
        { SatSigId(Code::G1P, 7), folder + "L1P 07.csv", marginL1 },
        { SatSigId(Code::G1P, 8), folder + "L1P 08.csv", marginL1 },
        { SatSigId(Code::G1P, 9), folder + "L1P 09.csv", marginL1 },
        { SatSigId(Code::G1P, 11), folder + "L1P 11.csv", marginL1 },
        { SatSigId(Code::G1P, 13), folder + "L1P 13.csv", marginL1 },
        { SatSigId(Code::G1P, 14), folder + "L1P 14.csv", marginL1 },
        { SatSigId(Code::G1P, 17), folder + "L1P 17.csv", marginL1 },
        { SatSigId(Code::G1P, 19), folder + "L1P 19.csv", marginL1 },
        { SatSigId(Code::G1P, 21), folder + "L1P 21.csv", marginL1 },
        { SatSigId(Code::G1P, 24), folder + "L1P 24.csv", marginL1 },
        { SatSigId(Code::G1P, 30), folder + "L1P 30.csv", marginL1 },
        { SatSigId(Code::G2C, 1), folder + "L2C 01.csv", marginL2 },
        { SatSigId(Code::G2C, 3), folder + "L2C 03.csv", marginL2 },
        { SatSigId(Code::G2C, 6), folder + "L2C 06.csv", marginL2 },
        { SatSigId(Code::G2C, 7), folder + "L2C 07.csv", marginL2 },
        { SatSigId(Code::G2C, 8), folder + "L2C 08.csv", marginL2 },
        { SatSigId(Code::G2C, 9), folder + "L2C 09.csv", marginL2 },
        { SatSigId(Code::G2C, 11), folder + "L2C 11.csv", marginL2 },
        { SatSigId(Code::G2C, 13), folder + "L2C 13.csv", marginL2 },
        { SatSigId(Code::G2C, 14), folder + "L2C 14.csv", marginL2 },
        { SatSigId(Code::G2C, 17), folder + "L2C 17.csv", marginL2 },
        { SatSigId(Code::G2C, 19), folder + "L2C 19.csv", marginL2 },
        { SatSigId(Code::G2C, 21), folder + "L2C 21.csv", marginL2 },
        { SatSigId(Code::G2C, 24), folder + "L2C 24.csv", marginL2 },
        { SatSigId(Code::G2C, 30), folder + "L2C 30.csv", marginL2 },
        { SatSigId(Code::G2P, 1), folder + "L2P 01.csv", marginL2 },
        { SatSigId(Code::G2P, 3), folder + "L2P 03.csv", marginL2 },
        { SatSigId(Code::G2P, 6), folder + "L2P 06.csv", marginL2 },
        { SatSigId(Code::G2P, 7), folder + "L2P 07.csv", marginL2 },
        { SatSigId(Code::G2P, 8), folder + "L2P 08.csv", marginL2 },
        { SatSigId(Code::G2P, 9), folder + "L2P 09.csv", marginL2 },
        { SatSigId(Code::G2P, 11), folder + "L2P 11.csv", marginL2 },
        { SatSigId(Code::G2P, 13), folder + "L2P 13.csv", marginL2 },
        { SatSigId(Code::G2P, 14), folder + "L2P 14.csv", marginL2 },
        { SatSigId(Code::G2P, 17), folder + "L2P 17.csv", marginL2 },
        { SatSigId(Code::G2P, 19), folder + "L2P 19.csv", marginL2 },
        { SatSigId(Code::G2P, 21), folder + "L2P 21.csv", marginL2 },
        { SatSigId(Code::G2P, 24), folder + "L2P 24.csv", marginL2 },
        { SatSigId(Code::G2P, 30), folder + "L2P 30.csv", marginL2 },
        { SatSigId(Code::G5X, 1), folder + "L5 01.csv", marginL5 },
        { SatSigId(Code::G5X, 3), folder + "L5 03.csv", marginL5 },
        { SatSigId(Code::G5X, 6), folder + "L5 06.csv", marginL5 },
        { SatSigId(Code::G5X, 7), folder + "L5 07.csv", marginL5 },
        { SatSigId(Code::G5X, 8), folder + "L5 08.csv", marginL5 },
        { SatSigId(Code::G5X, 9), folder + "L5 09.csv", marginL5 },
        { SatSigId(Code::G5X, 11), folder + "L5 11.csv", marginL5 },
        { SatSigId(Code::G5X, 13), folder + "L5 13.csv", marginL5 },
        { SatSigId(Code::G5X, 14), folder + "L5 14.csv", marginL5 },
        { SatSigId(Code::G5X, 17), folder + "L5 17.csv", marginL5 },
        { SatSigId(Code::G5X, 19), folder + "L5 19.csv", marginL5 },
        { SatSigId(Code::G5X, 21), folder + "L5 21.csv", marginL5 },
        { SatSigId(Code::G5X, 24), folder + "L5 24.csv", marginL5 },
        { SatSigId(Code::G5X, 30), folder + "L5 30.csv", marginL5 },
    };

    testNavFile(Skydel, "GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/SkydelRINEX_S_20238959_7200S_GN.rnx", files);
}

TEST_CASE("[Ephemeris] GPS Ephemeris calc orbit (Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none)", "[Ephemeris][flow]")
{
    // Margins determined by running the test and adapting
    Margin marginL1{ .clock = 0, .pos = 9.1e-5, .vel = 8.4e-4, .accel = 8.1e-1 };
    Margin marginL2{ .clock = 0, .pos = 9.1e-5, .vel = 8.4e-4, .accel = 8.1e-1 };
    Margin marginL5{ .clock = 0, .pos = 1.1e-4, .vel = 8.4e-4, .accel = 8.1e-1 };

    std::string path = "test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none/sat_data_V1A1.csv";
    std::vector<std::tuple<SatSigId, std::string, Margin>> files = {
        { SatSigId(Code::G1C, 1), path, marginL1 },
        { SatSigId(Code::G1C, 3), path, marginL1 },
        { SatSigId(Code::G1C, 6), path, marginL1 },
        { SatSigId(Code::G1C, 7), path, marginL1 },
        { SatSigId(Code::G1C, 8), path, marginL1 },
        { SatSigId(Code::G1C, 9), path, marginL1 },
        { SatSigId(Code::G1C, 11), path, marginL1 },
        { SatSigId(Code::G1C, 13), path, marginL1 },
        { SatSigId(Code::G1C, 14), path, marginL1 },
        { SatSigId(Code::G1C, 17), path, marginL1 },
        { SatSigId(Code::G1C, 19), path, marginL1 },
        { SatSigId(Code::G1C, 20), path, marginL1 },
        { SatSigId(Code::G1C, 21), path, marginL1 },
        { SatSigId(Code::G1C, 24), path, marginL1 },
        { SatSigId(Code::G1C, 30), path, marginL1 },
        { SatSigId(Code::G2C, 1), path, marginL2 },
        { SatSigId(Code::G2C, 3), path, marginL2 },
        { SatSigId(Code::G2C, 6), path, marginL2 },
        { SatSigId(Code::G2C, 7), path, marginL2 },
        { SatSigId(Code::G2C, 8), path, marginL2 },
        { SatSigId(Code::G2C, 9), path, marginL2 },
        { SatSigId(Code::G2C, 11), path, marginL2 },
        { SatSigId(Code::G2C, 13), path, marginL2 },
        { SatSigId(Code::G2C, 14), path, marginL2 },
        { SatSigId(Code::G2C, 17), path, marginL2 },
        { SatSigId(Code::G2C, 19), path, marginL2 },
        { SatSigId(Code::G2C, 20), path, marginL2 },
        { SatSigId(Code::G2C, 21), path, marginL2 },
        { SatSigId(Code::G2C, 24), path, marginL2 },
        { SatSigId(Code::G2C, 30), path, marginL2 },
        { SatSigId(Code::G5X, 1), path, marginL5 },
        { SatSigId(Code::G5X, 3), path, marginL5 },
        { SatSigId(Code::G5X, 6), path, marginL5 },
        { SatSigId(Code::G5X, 7), path, marginL5 },
        { SatSigId(Code::G5X, 8), path, marginL5 },
        { SatSigId(Code::G5X, 9), path, marginL5 },
        { SatSigId(Code::G5X, 11), path, marginL5 },
        { SatSigId(Code::G5X, 13), path, marginL5 },
        { SatSigId(Code::G5X, 14), path, marginL5 },
        { SatSigId(Code::G5X, 17), path, marginL5 },
        { SatSigId(Code::G5X, 19), path, marginL5 },
        { SatSigId(Code::G5X, 20), path, marginL5 },
        { SatSigId(Code::G5X, 21), path, marginL5 },
        { SatSigId(Code::G5X, 24), path, marginL5 },
        { SatSigId(Code::G5X, 30), path, marginL5 },
    };

    testNavFile(Spirent, "GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Spirent_RINEX_GN.23N", files);
}

TEST_CASE("[Ephemeris] GPS Ephemeris calc orbit (Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-Klob_tropo-Saast)", "[Ephemeris][flow]")
{
    // Margins determined by running the test and adapting
    Margin marginL1{ .clock = 0, .pos = 9.1e-5, .vel = 8.4e-4, .accel = 8.1e-1 };
    Margin marginL2{ .clock = 0, .pos = 2.4e-4, .vel = 8.4e-4, .accel = 8.1e-1 };
    Margin marginL5{ .clock = 0, .pos = 2.9e-4, .vel = 8.4e-4, .accel = 8.1e-1 };

    std::string path = "test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-Klob_tropo-Saast/sat_data_V1A1.csv";
    std::vector<std::tuple<SatSigId, std::string, Margin>> files = {
        { SatSigId(Code::G1C, 1), path, marginL1 },
        { SatSigId(Code::G1C, 3), path, marginL1 },
        { SatSigId(Code::G1C, 6), path, marginL1 },
        { SatSigId(Code::G1C, 7), path, marginL1 },
        { SatSigId(Code::G1C, 8), path, marginL1 },
        { SatSigId(Code::G1C, 9), path, marginL1 },
        { SatSigId(Code::G1C, 11), path, marginL1 },
        { SatSigId(Code::G1C, 13), path, marginL1 },
        { SatSigId(Code::G1C, 14), path, marginL1 },
        { SatSigId(Code::G1C, 17), path, marginL1 },
        { SatSigId(Code::G1C, 19), path, marginL1 },
        { SatSigId(Code::G1C, 20), path, marginL1 },
        { SatSigId(Code::G1C, 21), path, marginL1 },
        { SatSigId(Code::G1C, 24), path, marginL1 },
        { SatSigId(Code::G1C, 30), path, marginL1 },
        { SatSigId(Code::G2C, 1), path, marginL2 },
        { SatSigId(Code::G2C, 3), path, marginL2 },
        { SatSigId(Code::G2C, 6), path, marginL2 },
        { SatSigId(Code::G2C, 7), path, marginL2 },
        { SatSigId(Code::G2C, 8), path, marginL2 },
        { SatSigId(Code::G2C, 9), path, marginL2 },
        { SatSigId(Code::G2C, 11), path, marginL2 },
        { SatSigId(Code::G2C, 13), path, marginL2 },
        { SatSigId(Code::G2C, 14), path, marginL2 },
        { SatSigId(Code::G2C, 17), path, marginL2 },
        { SatSigId(Code::G2C, 19), path, marginL2 },
        { SatSigId(Code::G2C, 20), path, marginL2 },
        { SatSigId(Code::G2C, 21), path, marginL2 },
        { SatSigId(Code::G2C, 24), path, marginL2 },
        { SatSigId(Code::G2C, 30), path, marginL2 },
        { SatSigId(Code::G5X, 1), path, marginL5 },
        { SatSigId(Code::G5X, 3), path, marginL5 },
        { SatSigId(Code::G5X, 6), path, marginL5 },
        { SatSigId(Code::G5X, 7), path, marginL5 },
        { SatSigId(Code::G5X, 8), path, marginL5 },
        { SatSigId(Code::G5X, 9), path, marginL5 },
        { SatSigId(Code::G5X, 11), path, marginL5 },
        { SatSigId(Code::G5X, 13), path, marginL5 },
        { SatSigId(Code::G5X, 14), path, marginL5 },
        { SatSigId(Code::G5X, 17), path, marginL5 },
        { SatSigId(Code::G5X, 19), path, marginL5 },
        { SatSigId(Code::G5X, 20), path, marginL5 },
        { SatSigId(Code::G5X, 21), path, marginL5 },
        { SatSigId(Code::G5X, 24), path, marginL5 },
        { SatSigId(Code::G5X, 30), path, marginL5 },
    };

    testNavFile(Spirent, "GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Spirent_RINEX_GN.23N", files);
}

} // namespace NAV::TESTS::EphemerisTests