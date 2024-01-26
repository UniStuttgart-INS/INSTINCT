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
    GalileoEphemeris eph(2023, 1, 8, 12, 0, 0, -1.046533754561e-03, -2.094679985021e-11, 0.000000000000e+00,
                         7.200000000000e+01, -1.225000000000e+01, 3.048698419098e-09, 8.358485318292e-01,
                         -5.848705768585e-07, 7.337066344917e-04, 8.532777428627e-06, 5.440620456696e+03,
                         4.320000000000e+04, -3.166496753693e-08, 2.602299169600e+00, -5.587935447693e-08,
                         9.686712412968e-01, 1.626875000000e+02, 6.866771071778e-01, -5.322721712724e-09,
                         -7.236015694813e-10, 5.160000000000e+02, 2.244000000000e+03, 0.000000000000e+00,
                         3.120000000000e+00, 0.000000000000e+00, -3.259629011154e-09, -4.190951585770e-09,
                         4.386400000000e+04, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);

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

TEST_CASE("[Ephemeris] GAL Ephemeris calc orbit (Orolia Skydel data)", "[Ephemeris]")
{
    // E13 - Generated from Orolia Skydel
    GalileoEphemeris eph(2023, 1, 8, 12, 0, 0, -1.047604542918e-03, -2.098943241435e-11, 0.000000000000e+00,
                         1.500000000000e+01, -1.715625000000e+01, 3.218348342841e-09, 8.358664211813e-01,
                         -8.828938007355e-07, 7.335821865126e-04, 7.871538400650e-06, 5.440620594025e+03,
                         4.320000000000e+04, -1.862645149231e-09, 2.602301116676e+00, -9.313225746155e-09,
                         9.686375123090e-01, 1.779687500000e+02, 6.866562298739e-01, -5.443083869148e-09,
                         -5.553802766749e-10, 3.000000000000e+00, 2.244000000000e+03, 0.000000000000e+00,
                         3.120000000000e+00, 0.000000000000e+00, -2.095475792885e-09, 0.000000000000e+00,
                         9.999000000000e+08, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);

    Margin margin;          // Determined by running the test and adapting
    margin.clock = 2.3e-14; // Orolia file has only 16 digits after comma
    margin.pos = 9.0e-6;

    testEphemerisData({ GAL, 24 }, eph,
                      "test/data/GNSS/Orolia-Skydel_static_duration-4h_rate-5min_sys-GERCQIS_iono-none_tropo-none/sat_data/E1 24.csv",
                      E01, Skydel, margin);
}

TEST_CASE("[Ephemeris] GAL Ephemeris calc orbit (Spirent SimGEN data)", "[Ephemeris]")
{
    // E24 - Exported from the Spirent SimGEN GUI
    GalileoEphemeris eph(2023, 1, 8, 12, 0, 0, -1.046534081865e-03, -2.098943241435e-11, 0.000000000000e+00,
                         2.000000000000e+00, -1.715625000000e+01, 3.218348342841e-09, 8.358664211813e-01,
                         -8.828938007355e-07, 7.335821865126e-04, 7.871538400650e-06, 5.440620594025e+03,
                         4.320000000000e+04, -1.862645149231e-09, 2.602301116676e+00, -9.313225746155e-09,
                         9.686658367031e-01, 1.779687500000e+02, 6.866562298739e-01, -5.443083869148e-09,
                         -5.553802766749e-10, 2.630000000000e+02, 2.244000000000e+03, 0.000000000000e+00,
                         2.500000000000e+01, 0.000000000000e+00, -2.095475792885e-09, -2.363722364516e-09,
                         4.320000000000e+04, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);

    Margin margin; // Determined by running the test and adapting
    margin.pos = 5.8e-5;
    margin.vel = 6.8e-4;
    margin.accel = 6.4e-1;

    testEphemerisData({ GAL, 24 }, eph,
                      "test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQ_iono-none_tropo-none/sat_data_V1A1.csv",
                      E01, Spirent, margin);
}

} // namespace NAV::TESTS::EphemerisTests