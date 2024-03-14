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
    GPSEphemeris eph(2023, 1, 8, 12, 0, 0, 2.270475961268e-04, -4.774847184308e-12, 0.000000000000e+00,
                     1.800000000000e+01, 4.412500000000e+01, 4.154815921903e-09, 9.534843171347e-02,
                     2.287328243256e-06, 1.217866723891e-02, 9.965151548386e-07, 5.153653379440e+03,
                     4.320000000000e+04, -6.891787052155e-08, -1.509394590195e+00, 1.434236764908e-07,
                     9.889891589796e-01, 3.767500000000e+02, 9.377162063410e-01, -8.364991292606e-09,
                     1.185763677531e-10, 1.000000000000e+00, 2.244000000000e+03, 0.000000000000e+00,
                     2.000000000000e+00, 0.000000000000e+00, 4.656612873077e-09, 1.800000000000e+01,
                     3.601800000000e+04, 4.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);

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

TEST_CASE("[Ephemeris] GPS L1 Ephemeris calc orbit (Orolia Skydel data)", "[Ephemeris]")
{
    // G01 - Generated from Orolia Skydel
    GPSEphemeris eph(2023, 1, 8, 12, 0, 0, 2.268395255669e-04, -4.774847184308e-12, 0.000000000000e+00,
                     3.000000000000e+00, 4.684375000000e+01, 4.114099940205e-09, 9.534739229069e-02,
                     2.561137080193e-06, 1.217794988770e-02, 1.572072505951e-06, 5.153651281357e+03,
                     4.320000000000e+04, 9.685754776001e-08, -1.509389413407e+00, 2.831220626831e-07,
                     9.889944452341e-01, 3.624375000000e+02, 9.377089341753e-01, -8.193912737927e-09,
                     2.285809498855e-11, 1.000000000000e+00, 2.244000000000e+03, 1.000000000000e+00,
                     2.000000000000e+00, 0.000000000000e+00, 4.656612873077e-09, 3.000000000000e+00,
                     9.999000000000e+08, 4.000000000000e+00);

    Margin margin;          // Determined by running the test and adapting
    margin.clock = 1.2e-15; // Orolia file has only 16 digits after comma
    margin.pos = 5.3e-5;

    testEphemerisData({ GPS, 1 }, eph,
                      "test/data/GNSS/Orolia-Skydel_static_duration-4h_rate-5min_sys-GERCQIS_iono-none_tropo-none/sat_data/L1CA 01.csv",
                      G01, Skydel, margin);
}

TEST_CASE("[Ephemeris] GPS L2 Ephemeris calc orbit (Orolia Skydel data)", "[Ephemeris]")
{
    // G01 - Generated from Orolia Skydel
    GPSEphemeris eph(2023, 1, 8, 12, 0, 0, 2.268395255669e-04, -4.774847184308e-12, 0.000000000000e+00,
                     3.000000000000e+00, 4.684375000000e+01, 4.114099940205e-09, 9.534739229069e-02,
                     2.561137080193e-06, 1.217794988770e-02, 1.572072505951e-06, 5.153651281357e+03,
                     4.320000000000e+04, 9.685754776001e-08, -1.509389413407e+00, 2.831220626831e-07,
                     9.889944452341e-01, 3.624375000000e+02, 9.377089341753e-01, -8.193912737927e-09,
                     2.285809498855e-11, 1.000000000000e+00, 2.244000000000e+03, 1.000000000000e+00,
                     2.000000000000e+00, 0.000000000000e+00, 4.656612873077e-09, 3.000000000000e+00,
                     9.999000000000e+08, 4.000000000000e+00);

    Margin margin;          // Determined by running the test and adapting
    margin.clock = 1.4e-15; // Orolia file has only 16 digits after comma
    margin.pos = 6.3e-5;

    testEphemerisData({ GPS, 1 }, eph,
                      "test/data/GNSS/Orolia-Skydel_static_duration-4h_rate-5min_sys-GERCQIS_iono-none_tropo-none/sat_data/L2C 01.csv",
                      G02, Skydel, margin);
}

TEST_CASE("[Ephemeris] GPS L5 Ephemeris calc orbit (Orolia Skydel data)", "[Ephemeris]")
{
    // G01 - Generated from Orolia Skydel
    GPSEphemeris eph(2023, 1, 8, 12, 0, 0, 2.268395255669e-04, -4.774847184308e-12, 0.000000000000e+00,
                     3.000000000000e+00, 4.684375000000e+01, 4.114099940205e-09, 9.534739229069e-02,
                     2.561137080193e-06, 1.217794988770e-02, 1.572072505951e-06, 5.153651281357e+03,
                     4.320000000000e+04, 9.685754776001e-08, -1.509389413407e+00, 2.831220626831e-07,
                     9.889944452341e-01, 3.624375000000e+02, 9.377089341753e-01, -8.193912737927e-09,
                     2.285809498855e-11, 1.000000000000e+00, 2.244000000000e+03, 1.000000000000e+00,
                     2.000000000000e+00, 0.000000000000e+00, 4.656612873077e-09, 3.000000000000e+00,
                     9.999000000000e+08, 4.000000000000e+00);

    Margin margin;          // Determined by running the test and adapting
    margin.clock = 1.8e-15; // Orolia file has only 16 digits after comma
    margin.pos = 6.3e-5;

    testEphemerisData({ GPS, 1 }, eph,
                      "test/data/GNSS/Orolia-Skydel_static_duration-4h_rate-5min_sys-GERCQIS_iono-none_tropo-none/sat_data/L5 01.csv",
                      G05, Skydel, margin);
}

TEST_CASE("[Ephemeris] GPS L1 Ephemeris calc orbit (Spirent SimGEN data)", "[Ephemeris]")
{
    // G01 - Exported from the Spirent SimGEN GUI
    GPSEphemeris eph(2023, 1, 8, 12, 0, 0, 2.270457989652e-04, -4.774847184308e-12, 0.000000000000e+00,
                     4.800000000000e+01, 4.684375000000e+01, 4.114099940205e-09, 9.534739229067e-02,
                     2.561137080193e-06, 1.217794988770e-02, 1.572072505951e-06, 5.153651281357e+03,
                     4.320000000000e+04, 9.685754776001e-08, -1.509389413407e+00, 2.831220626831e-07,
                     9.889934577644e-01, 3.624375000000e+02, 9.377089341753e-01, -8.193912737927e-09,
                     2.285809498855e-11, 1.000000000000e+00, 2.244000000000e+03, 0.000000000000e+00,
                     1.200000000000e+00, 0.000000000000e+00, 4.656612873077e-09, 4.800000000000e+01,
                     4.320000000000e+04, 4.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);

    Margin margin; // Determined by running the test and adapting
    margin.pos = 7.2e-5;
    margin.vel = 7.8e-4;
    margin.accel = 1.8e-1;

    testEphemerisData({ GPS, 1 }, eph,
                      "test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQ_iono-none_tropo-none/sat_data_V1A1.csv",
                      G01, Spirent, margin);
}

TEST_CASE("[Ephemeris] GPS L2 Ephemeris calc orbit (Spirent SimGEN data)", "[Ephemeris]")
{
    // G01 - Exported from the Spirent SimGEN GUI
    GPSEphemeris eph(2023, 1, 8, 12, 0, 0, 2.270457989652e-04, -4.774847184308e-12, 0.000000000000e+00,
                     4.800000000000e+01, 4.684375000000e+01, 4.114099940205e-09, 9.534739229067e-02,
                     2.561137080193e-06, 1.217794988770e-02, 1.572072505951e-06, 5.153651281357e+03,
                     4.320000000000e+04, 9.685754776001e-08, -1.509389413407e+00, 2.831220626831e-07,
                     9.889934577644e-01, 3.624375000000e+02, 9.377089341753e-01, -8.193912737927e-09,
                     2.285809498855e-11, 1.000000000000e+00, 2.244000000000e+03, 0.000000000000e+00,
                     1.200000000000e+00, 0.000000000000e+00, 4.656612873077e-09, 4.800000000000e+01,
                     4.320000000000e+04, 4.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);

    Margin margin; // Determined by running the test and adapting
    margin.pos = 7.2e-5;
    margin.vel = 7.8e-4;
    margin.accel = 1.8e-1;

    testEphemerisData({ GPS, 1 }, eph,
                      "test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQ_iono-none_tropo-none/sat_data_V1A1.csv",
                      G02, Spirent, margin);
}

TEST_CASE("[Ephemeris] GPS L5 Ephemeris calc orbit (Spirent SimGEN data)", "[Ephemeris]")
{
    // G01 - Exported from the Spirent SimGEN GUI
    GPSEphemeris eph(2023, 1, 8, 12, 0, 0, 2.270457989652e-04, -4.774847184308e-12, 0.000000000000e+00,
                     4.800000000000e+01, 4.684375000000e+01, 4.114099940205e-09, 9.534739229067e-02,
                     2.561137080193e-06, 1.217794988770e-02, 1.572072505951e-06, 5.153651281357e+03,
                     4.320000000000e+04, 9.685754776001e-08, -1.509389413407e+00, 2.831220626831e-07,
                     9.889934577644e-01, 3.624375000000e+02, 9.377089341753e-01, -8.193912737927e-09,
                     2.285809498855e-11, 1.000000000000e+00, 2.244000000000e+03, 0.000000000000e+00,
                     1.200000000000e+00, 0.000000000000e+00, 4.656612873077e-09, 4.800000000000e+01,
                     4.320000000000e+04, 4.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);

    Margin margin; // Determined by running the test and adapting
    margin.pos = 8.6e-5;
    margin.vel = 7.8e-4;
    margin.accel = 1.8e-1;

    testEphemerisData({ GPS, 1 }, eph,
                      "test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQ_iono-none_tropo-none/sat_data_V1A1.csv",
                      G05, Spirent, margin);
}

} // namespace NAV::TESTS::EphemerisTests