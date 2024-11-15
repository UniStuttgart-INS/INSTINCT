// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Common.hpp"

#include "Navigation/GNSS/Satellite/Ephemeris/QZSSEphemeris.hpp"

namespace NAV::TESTS::EphemerisTests
{

TEST_CASE("[Ephemeris] QZSS Ephemeris calc orbit (BRDC_20230080000) QZO Satellite", "[Ephemeris]")
{
    // J02 - Taken from real data
    // NOLINTBEGIN
    QZSSEphemeris eph(2023, 1, 8, 12, 0, 0, -5.154870450497e-07, -1.136868377216e-13, 0.000000000000e+00,
                      4.500000000000e+01, -9.359375000000e+01, 1.787574459651e-09, -1.781683475617e+00,
                      -4.431232810020e-06, 7.510575617198e-02, 1.722574234009e-05, 6.493197826385e+03,
                      4.320000000000e+04, 1.095235347748e-06, 2.689490712426e+00, -1.784414052963e-06,
                      7.232893311953e-01, -4.045625000000e+02, -1.578422486558e+00, -1.087188142893e-09,
                      -8.221771041194e-10, 2.000000000000e+00, 2.244000000000e+03, 1.000000000000e+00,
                      2.800000000000e+00, 0.000000000000e+00, 4.656612873077e-10, 8.130000000000e+02,
                      3.960600000000e+04, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);
    // NOLINTEND

    // https://igs.org/products/
    // | Broadcast         | Accuracy     |
    // |     - orbits      | ~100 cm      |
    // |     - Sat. clocks | ~5   ns RMS  |
    // |                   | ~2.5 ns SDev |

    Margin margin; // Determined by running the test and adapting
    margin.clock = 2.2e-07;
    margin.pos = 1.85; // TODO too high (mainly in x)

    testBrdcEphemerisData({ QZSS, 2 }, eph, "test/data/GNSS/BRDC_20230080000/GFZ0MGXRAP_20230080000_01D_05M_ORB.SP3", margin);
}

TEST_CASE("[Ephemeris] QZSS Ephemeris calc orbit (BRDC_20230080000) GEO Satellite", "[Ephemeris]")
{
    // J03 - Taken from real data
    // NOLINTBEGIN
    QZSSEphemeris eph(2023, 1, 8, 12, 0, 0, -7.729977369308e-08, 1.136868377216e-13, 0.000000000000e+00,
                      4.500000000000e+01, 1.023968750000e+03, 1.978653847446e-09, 2.715028968426e+00,
                      3.195181488991e-05, 7.530548574869e-02, -1.731514930725e-05, 6.492851787567e+03,
                      4.320000000000e+04, -1.063570380211e-06, -1.880581743881e+00, -9.760260581970e-07,
                      7.103337594066e-01, 7.236562500000e+02, -1.593040018134e+00, -2.168304604304e-09,
                      -2.928693420408e-10, 2.000000000000e+00, 2.244000000000e+03, 1.000000000000e+00,
                      2.800000000000e+00, 0.000000000000e+00, -4.656612873077e-10, 8.130000000000e+02,
                      3.960600000000e+04, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);
    // NOLINTEND

    // https://igs.org/products/
    // | Broadcast         | Accuracy     |
    // |     - orbits      | ~100 cm      |
    // |     - Sat. clocks | ~5   ns RMS  |
    // |                   | ~2.5 ns SDev |

    Margin margin; // Determined by running the test and adapting
    margin.clock = 1.3e-07;
    margin.pos = 1.72; // TODO too high (mainly in x)

    testBrdcEphemerisData({ QZSS, 3 }, eph, "test/data/GNSS/BRDC_20230080000/GFZ0MGXRAP_20230080000_01D_05M_ORB.SP3", margin);
}

TEST_CASE("[Ephemeris] QZSS Ephemeris calc orbit (Skydel data) QZO Satellite", "[Ephemeris]")
{
    // J02 - Generated from Skydel
    // file: test/data/GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/SkydelRINEX_S_20238959_3600S_JN.rnx
    // NOLINTBEGIN
    QZSSEphemeris eph(2023, 1, 8, 12, 0, 0, -5.200308805797E-07, -1.136868377216E-13, 0.000000000000E+00,
                      2.000000000000E+00, -2.813750000000E+02, 1.406129999530E-09, -1.781581877224E+00,
                      -8.951872587204E-06, 7.510803255718E-02, 1.034326851368E-05, 6.493260515213E+03,
                      4.320000000000E+04, 1.206994056702E-06, 2.689429669524E+00, -2.117827534676E-06,
                      7.232395089143E-01, -1.172500000000E+02, -1.578430481405E+00, -1.332555506286E-09,
                      -6.757424330990E-10, 2.000000000000E+00, 2.244000000000E+03, 1.000000000000E+00,
                      2.800000000000E+00, 0.000000000000E+00, 4.656612873077E-10, 2.000000000000E+00,
                      9.999000000000E+08, 0.000000000000E+00);
    // NOLINTEND

    Margin margin;        // Determined by running the test and adapting
    margin.clock = 2e-17; // file has only 16 digits after comma
    margin.pos = 4.7e-04;

    testEphemerisData({ QZSS, 2 }, eph,
                      "test/data/GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/sat_data/QZSSL1CA 02.csv",
                      J01, Skydel, margin);
}

TEST_CASE("[Ephemeris] QZSS Ephemeris calc orbit (Skydel data) GEO Satellite", "[Ephemeris]")
{
    // J03 - Generated from Skydel
    // file: test/data/GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/SkydelRINEX_S_20238959_3600S_JN.rnx
    // NOLINTBEGIN
    QZSSEphemeris eph(2023, 1, 8, 12, 0, 0, -7.869675755501e-08, 0.000000000000e+00, 0.000000000000e+00,
                      2.000000000000e+00, 4.391875000000e+02, 1.447917454431e-09, 2.714520003919e+00,
                      1.654401421547e-05, 7.526671467349e-02, -8.506700396538e-06, 6.492790670395e+03,
                      4.320000000000e+04, -2.335757017136e-06, -1.880630227061e+00, 3.058463335037e-06,
                      7.103428219327e-01, 4.215000000000e+02, -1.592629635966e+00, -1.165405686682e-09,
                      -9.450393646828e-10, 2.000000000000e+00, 2.244000000000e+03, 1.000000000000e+00,
                      2.800000000000e+00, 0.000000000000e+00, -4.656612873077e-10, 2.000000000000e+00,
                      9.999000000000e+08, 0.000000000000e+00);
    // NOLINTEND

    Margin margin;          // Determined by running the test and adapting
    margin.clock = 4.2e-17; // file has only 16 digits after comma
    margin.pos = 1.8e-04;

    testEphemerisData({ QZSS, 3 }, eph,
                      "test/data/GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/sat_data/QZSSL1CA 03.csv",
                      J01, Skydel, margin);
}

TEST_CASE("[Ephemeris] QZSS Ephemeris calc orbit (Spirent SimGEN data) QZO Satellite", "[Ephemeris]")
{
    // J02 - Exported from the Spirent SimGEN GUI
    // file: test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQ/Iono-none_tropo-none/Spirent_RINEX_CN.23C
    // NOLINTBEGIN
    QZSSEphemeris eph(2023, 1, 8, 12, 0, 0, -5.147103365743e-07, -1.136868377216e-13, 0.000000000000e+00,
                      8.570000000000e+02, -2.813750000000e+02, 1.406129999530e-09, -1.781581877224e+00,
                      -8.951872587204e-06, 7.510803255718e-02, 1.034326851368e-05, 6.493260515213e+03,
                      4.320000000000e+04, 1.206994056702e-06, 2.689429669524e+00, -2.117827534676e-06,
                      7.232711336602e-01, -1.172500000000e+02, -1.578430481405e+00, -1.332555506286e-09,
                      -6.757424330990e-10, 0.000000000000e+00, 2.244000000000e+03, 0.000000000000e+00,
                      2.900000000000e+00, 0.000000000000e+00, 0.000000000000e+00, 8.570000000000e+02,
                      4.320000000000e+04, 4.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);
    // NOLINTEND

    Margin margin; // Determined by running the test and adapting
    margin.pos = 8.1e-05;
    margin.vel = 6.7e-04;
    margin.accel = 1.8e-01;

    testEphemerisData({ QZSS, 2 }, eph,
                      "test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none/sat_data_V1A1.csv",
                      J01, Spirent, margin);
}

TEST_CASE("[Ephemeris] QZSS Ephemeris calc orbit (Spirent SimGEN data) GEO Satellite", "[Ephemeris]")
{
    // J03 - Exported from the Spirent SimGEN GUI
    // file: test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQ/Iono-none_tropo-none/Spirent_RINEX_CN.23C
    // NOLINTBEGIN
    QZSSEphemeris eph(2023, 1, 8, 12, 0, 0, -7.869675755501e-08, 0.000000000000e+00, 0.000000000000e+00,
                      8.570000000000e+02, 4.391875000000e+02, 1.447917454431e-09, 2.714520003919e+00,
                      1.654401421547e-05, 7.526671467349e-02, -8.506700396538e-06, 6.492790670395e+03,
                      4.320000000000e+04, -2.335757017136e-06, -1.880630227061e+00, 3.058463335037e-06,
                      7.103870497749e-01, 4.215000000000e+02, -1.592629635966e+00, -1.165405686682e-09,
                      -9.450393646828e-10, 0.000000000000e+00, 2.244000000000e+03, 0.000000000000e+00,
                      2.900000000000e+00, 0.000000000000e+00, 0.000000000000e+00, 8.570000000000e+02,
                      4.320000000000e+04, 4.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);
    // NOLINTEND

    Margin margin; // Determined by running the test and adapting
    margin.pos = 6.6e-05;
    margin.vel = 7.4e-04;
    margin.accel = 0.16;

    testEphemerisData({ QZSS, 3 }, eph,
                      "test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none/sat_data_V1A1.csv",
                      J01, Spirent, margin);
}

} // namespace NAV::TESTS::EphemerisTests