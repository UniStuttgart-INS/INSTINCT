// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Common.hpp"

#include "Navigation/GNSS/Satellite/Ephemeris/GalileoEphemeris.hpp"

namespace NAV::TESTS::EphemerisTests
{

TEST_CASE("[Ephemeris] GAL Ephemeris calc orbit (BRDC_20230080000)", "[Ephemeris]")
{
    // E24 - Taken from real data
    // NOLINTBEGIN
    GalileoEphemeris eph(2023, 1, 8, 12, 0, 0, -1.046533754561e-03, -2.094679985021e-11, 0.000000000000e+00,
                         7.200000000000e+01, -1.225000000000e+01, 3.048698419098e-09, 8.358485318292e-01,
                         -5.848705768585e-07, 7.337066344917e-04, 8.532777428627e-06, 5.440620456696e+03,
                         4.320000000000e+04, -3.166496753693e-08, 2.602299169600e+00, -5.587935447693e-08,
                         9.686712412968e-01, 1.626875000000e+02, 6.866771071778e-01, -5.322721712724e-09,
                         -7.236015694813e-10, 5.160000000000e+02, 2.244000000000e+03, 0.000000000000e+00,
                         3.120000000000e+00, 0.000000000000e+00, -3.259629011154e-09, -4.190951585770e-09,
                         4.386400000000e+04, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);
    // NOLINTEND

    // https://igs.org/products/
    // | Broadcast         | Accuracy     |
    // |     - orbits      | ~100 cm      |
    // |     - Sat. clocks | ~5   ns RMS  |
    // |                   | ~2.5 ns SDev |

    Margin margin; // Determined by running the test and adapting
    margin.clock = 9.4e-9;
    margin.pos = 8.4e-1;

    testBrdcEphemerisData({ GAL, 24 }, eph, "test/data/GNSS/BRDC_20230080000/COD0OPSFIN_20230080000_01D_05M_ORB.SP3", margin);
}

TEST_CASE("[Ephemeris] GAL Ephemeris calc orbit (Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none)", "[Ephemeris][flow]")
{
    // Margins determined by running the test and adapting (Skydel file has only 16 digits after comma)
    Margin marginE1{ .clock = 6.4e-14, .pos = 2.7e-5, .vel = 0, .accel = 0 };
    Margin marginE5a{ .clock = 6.4e-14, .pos = 3.9e-5, .vel = 0, .accel = 0 };
    Margin marginE5b{ .clock = 6.4e-14, .pos = 2.7e-5, .vel = 0, .accel = 0 };

    std::string folder = "test/data/GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/sat_data/";
    std::vector<std::tuple<SatSigId, std::string, Margin>> files = {
        { SatSigId(Code::E1X, 1), folder + "E1 01.csv", marginE1 },
        { SatSigId(Code::E1X, 4), folder + "E1 04.csv", marginE1 },
        { SatSigId(Code::E1X, 5), folder + "E1 05.csv", marginE1 },
        { SatSigId(Code::E1X, 9), folder + "E1 09.csv", marginE1 },
        { SatSigId(Code::E1X, 10), folder + "E1 10.csv", marginE1 },
        { SatSigId(Code::E1X, 11), folder + "E1 11.csv", marginE1 },
        { SatSigId(Code::E1X, 12), folder + "E1 12.csv", marginE1 },
        { SatSigId(Code::E1X, 14), folder + "E1 14.csv", marginE1 },
        { SatSigId(Code::E1X, 24), folder + "E1 24.csv", marginE1 },
        { SatSigId(Code::E1X, 25), folder + "E1 25.csv", marginE1 },
        { SatSigId(Code::E1X, 26), folder + "E1 26.csv", marginE1 },
        { SatSigId(Code::E1X, 31), folder + "E1 31.csv", marginE1 },
        { SatSigId(Code::E1X, 33), folder + "E1 33.csv", marginE1 },
        { SatSigId(Code::E5X, 1), folder + "E5a 01.csv", marginE5a },
        { SatSigId(Code::E5X, 4), folder + "E5a 04.csv", marginE5a },
        { SatSigId(Code::E5X, 5), folder + "E5a 05.csv", marginE5a },
        { SatSigId(Code::E5X, 9), folder + "E5a 09.csv", marginE5a },
        { SatSigId(Code::E5X, 10), folder + "E5a 10.csv", marginE5a },
        { SatSigId(Code::E5X, 11), folder + "E5a 11.csv", marginE5a },
        { SatSigId(Code::E5X, 12), folder + "E5a 12.csv", marginE5a },
        { SatSigId(Code::E5X, 14), folder + "E5a 14.csv", marginE5a },
        { SatSigId(Code::E5X, 24), folder + "E5a 24.csv", marginE5a },
        { SatSigId(Code::E5X, 25), folder + "E5a 25.csv", marginE5a },
        { SatSigId(Code::E5X, 26), folder + "E5a 26.csv", marginE5a },
        { SatSigId(Code::E5X, 31), folder + "E5a 31.csv", marginE5a },
        { SatSigId(Code::E5X, 33), folder + "E5a 33.csv", marginE5a },
        { SatSigId(Code::E7X, 1), folder + "E5b 01.csv", marginE5b },
        { SatSigId(Code::E7X, 4), folder + "E5b 04.csv", marginE5b },
        { SatSigId(Code::E7X, 5), folder + "E5b 05.csv", marginE5b },
        { SatSigId(Code::E7X, 9), folder + "E5b 09.csv", marginE5b },
        { SatSigId(Code::E7X, 10), folder + "E5b 10.csv", marginE5b },
        { SatSigId(Code::E7X, 11), folder + "E5b 11.csv", marginE5b },
        { SatSigId(Code::E7X, 12), folder + "E5b 12.csv", marginE5b },
        { SatSigId(Code::E7X, 14), folder + "E5b 14.csv", marginE5b },
        { SatSigId(Code::E7X, 24), folder + "E5b 24.csv", marginE5b },
        { SatSigId(Code::E7X, 25), folder + "E5b 25.csv", marginE5b },
        { SatSigId(Code::E7X, 26), folder + "E5b 26.csv", marginE5b },
        { SatSigId(Code::E7X, 31), folder + "E5b 31.csv", marginE5b },
        { SatSigId(Code::E7X, 33), folder + "E5b 33.csv", marginE5b },
    };

    testNavFile(Skydel, "GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/SkydelRINEX_S_20238959_600S_EN.rnx", files);
}

TEST_CASE("[Ephemeris] GAL Ephemeris calc orbit (Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none)", "[Ephemeris][flow]")
{
    // Margins determined by running the test and adapting
    Margin marginE1{ .clock = 0, .pos = 8.6e-5, .vel = 8.4e-4, .accel = 8.1e-1 };
    Margin marginE5{ .clock = 0, .pos = 8.5e-5, .vel = 8.4e-4, .accel = 8.1e-1 };

    std::string path = "test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none/sat_data_V1A1.csv";
    std::vector<std::tuple<SatSigId, std::string, Margin>> files = {
        { SatSigId(Code::E1X, 1), path, marginE1 },
        { SatSigId(Code::E1X, 4), path, marginE1 },
        { SatSigId(Code::E1X, 5), path, marginE1 },
        { SatSigId(Code::E1X, 9), path, marginE1 },
        { SatSigId(Code::E1X, 10), path, marginE1 },
        { SatSigId(Code::E1X, 11), path, marginE1 },
        { SatSigId(Code::E1X, 12), path, marginE1 },
        { SatSigId(Code::E1X, 14), path, marginE1 },
        { SatSigId(Code::E1X, 24), path, marginE1 },
        { SatSigId(Code::E1X, 25), path, marginE1 },
        { SatSigId(Code::E1X, 26), path, marginE1 },
        { SatSigId(Code::E1X, 31), path, marginE1 },
        { SatSigId(Code::E1X, 33), path, marginE1 },
        { SatSigId(Code::E1X, 36), path, marginE1 },
        { SatSigId(Code::E5X, 1), path, marginE5 },
        { SatSigId(Code::E5X, 4), path, marginE5 },
        { SatSigId(Code::E5X, 5), path, marginE5 },
        { SatSigId(Code::E5X, 9), path, marginE5 },
        { SatSigId(Code::E5X, 10), path, marginE5 },
        { SatSigId(Code::E5X, 11), path, marginE5 },
        { SatSigId(Code::E5X, 12), path, marginE5 },
        { SatSigId(Code::E5X, 14), path, marginE5 },
        { SatSigId(Code::E5X, 24), path, marginE5 },
        { SatSigId(Code::E5X, 25), path, marginE5 },
        { SatSigId(Code::E5X, 26), path, marginE5 },
        { SatSigId(Code::E5X, 31), path, marginE5 },
        { SatSigId(Code::E5X, 33), path, marginE5 },
        { SatSigId(Code::E5X, 36), path, marginE5 },
        // E6 values for pseudorange are empty in the 'sat_data_V1A1.csv' file
    };

    testNavFile(Spirent, "GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Spirent_RINEX_EN.23L", files);
}

} // namespace NAV::TESTS::EphemerisTests