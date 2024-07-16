// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file BDSEphemerisTests.cpp
/// @brief BDS Ephemeris tests
/// @author P. Peitschat (Hiwi)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-10-19

#include "Common.hpp"

#include "Navigation/GNSS/Satellite/Ephemeris/BDSEphemeris.hpp"

namespace NAV::TESTS::EphemerisTests
{

TEST_CASE("[Ephemeris] BDS Ephemeris calc orbit (BRDC_20230080000) GEO Satellite", "[Ephemeris]")
{
    // C01 - Taken from real data
    BDSEphemeris eph(/*1,*/ 2023, 1, 8, 12, 0, 0, 9.214587043971e-04, -3.444355911597e-12, 0.000000000000e+00,
                     1.000000000000e+00, -5.703437500000e+02, -1.417916204758e-09, -2.600407961772e+00,
                     -1.850631088018e-05, 5.964186275378e-04, 1.725181937218e-05, 6.493409469604e+03,
                     4.320000000000e+04, -1.606531441212e-07, -9.470894048181e-02, 1.862645149231e-08,
                     1.091456290507e-01, -5.247343750000e+02, 2.084960396501e+00, 2.475460255713e-09,
                     8.411064640318e-10, 0.000000000000e+00, 8.880000000000e+02, 0.000000000000e+00,
                     2.000000000000e+00, 0.000000000000e+00, -4.700000000000e-09, -1.000000000000e-08,
                     4.320000000000e+04, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);

    // https://igs.org/products/
    // | Broadcast         | Accuracy     |
    // |     - orbits      | ~100 cm      |
    // |     - Sat. clocks | ~5   ns RMS  |
    // |                   | ~2.5 ns SDev |

    Margin margin; // Determined by running the test and adapting
    margin.clock = 1.3e-08;
    margin.pos = 16; // TODO too high (mainly in x and y)

    testBrdcEphemerisData({ BDS, 1 }, eph, "test/data/GNSS/BRDC_20230080000/GFZ0MGXRAP_20230080000_01D_05M_ORB.SP3", margin);
}

TEST_CASE("[Ephemeris] BDS Ephemeris calc orbit (BRDC_20230080000) MEO/IGSO Satellite", "[Ephemeris]")
{
    // C06 - Taken from real data
    BDSEphemeris eph(/*6,*/ 2023, 1, 8, 12, 0, 0, 2.072051865980e-04, -3.074873689002e-12, 0.000000000000e+00,
                     1.000000000000e+00, 3.406250000000e+01, 1.519349001270e-09, 6.931342476407e-01,
                     1.237727701664e-06, 3.453022218309e-03, 1.035863533616e-05, 6.492955289841e+03,
                     4.320000000000e+04, 1.047737896442e-07, 1.170523532409e+00, -1.457519829273e-07,
                     9.452982942340e-01, -8.842187500000e+01, -3.136736826182e+00, -1.807218135032e-09,
                     9.136094840736e-10, 0.000000000000e+00, 8.880000000000e+02, 0.000000000000e+00,
                     2.000000000000e+00, 0.000000000000e+00, 8.600000000000e-09, -1.800000000000e-09,
                     4.320000000000e+04, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);

    // https://igs.org/products/
    // | Broadcast         | Accuracy     |
    // |     - orbits      | ~100 cm      |
    // |     - Sat. clocks | ~5   ns RMS  |
    // |                   | ~2.5 ns SDev |

    Margin margin; // Determined by running the test and adapting
    margin.clock = 1.8e-08;
    margin.pos = 1.59; // Too high

    testBrdcEphemerisData({ BDS, 6 }, eph, "test/data/GNSS/BRDC_20230080000/GFZ0MGXRAP_20230080000_01D_05M_ORB.SP3", margin);
}

TEST_CASE("[Ephemeris] BDS Ephemeris calc orbit (Skydel data) GEO Satellite", "[Ephemeris]")
{
    // C01 - Generated from Skydel
    // file: test/data/GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/SkydelRINEX_S_20238959_3600S_CN.rnx
    BDSEphemeris eph(/*1,*/ 2023, 1, 8, 12, 0, 0, 9.214587043971e-04, -3.444355911597e-12, 0.000000000000e+00,
                     0.000000000000e+00, -5.703437500000e+02, -1.417916204758e-09, -2.600407961772e+00,
                     -1.850631088018e-05, 5.964186275378e-04, 1.725181937218e-05, 6.493409469604e+03,
                     4.320000000000e+04, -1.606531441212e-07, -9.470894048181e-02, 1.862645149231e-08,
                     1.091456290507e-01, -5.247343750000e+02, 2.084960396501e+00, 2.475460255713e-09,
                     8.411064640318e-10, 0.000000000000e+00, 8.880000000000e+02, 0.000000000000e+00,
                     2.000000000000e+00, 0.000000000000e+00, -4.700000000000e-09, -1.000000000000e-08,
                     9.999000000000e+08, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);

    Margin margin;        // Determined by running the test and adapting
    margin.clock = 5e-09; // file has only 16 digits after comma
    margin.pos = 8.3e-02;

    testEphemerisData({ BDS, 1 }, eph,
                      "test/data/GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/sat_data/B1 01.csv",
                      B01, Skydel, margin);
}

TEST_CASE("[Ephemeris] BDS Ephemeris calc orbit (Skydel data) MEO/IGSO Satellite", "[Ephemeris]")
{
    // C06 - Generated from Skydel
    // file: test/data/GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/SkydelRINEX_S_20238959_3600S_CN.rnx
    BDSEphemeris eph(/*6,*/ 2023, 1, 8, 11, 0, 0, 2.073161779492e-04, -3.033129303276e-12, 0.000000000000e+00,
                     0.000000000000e+00, 1.189687500000e+02, 1.575065607805e-09, 4.278749074939e-01,
                     3.844499588013e-06, 3.454052493908e-03, 8.593313395977e-06, 6.492945194244e+03,
                     3.960000000000e+04, -7.916241884232e-08, 1.170527646781e+00, 1.089647412300e-07,
                     9.452644419439e-01, -2.242187500000e+01, -3.134031840914e+00, -1.890435887100e-09,
                     1.040043321979e-09, 0.000000000000e+00, 8.880000000000e+02, 0.000000000000e+00,
                     2.000000000000e+00, 0.000000000000e+00, 8.600000000000e-09, -1.800000000000e-09,
                     9.999000000000e+08, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);

    Margin margin;          // Determined by running the test and adapting
    margin.clock = 8.7e-09; // file has only 16 digits after comma
    margin.pos = 2.3e-05;

    testEphemerisData({ BDS, 6 }, eph,
                      "test/data/GNSS/Skydel_static_duration-4h_rate-5min_sys-GERCQIS/Iono-none_tropo-none/sat_data/B1 06.csv",
                      B01, Skydel, margin);
}

// TODO: not working
// TEST_CASE("[Ephemeris] BDS Ephemeris calc orbit (Spirent SimGEN data) GEO Satellite", "[Ephemeris]")
// {
//     // C03 - Exported from the Spirent SimGEN GUI
//     // file: test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQ/Iono-none_tropo-none/Spirent_RINEX_CN.23C
//     BDSEphemeris eph(2023, 1, 8, 12, 0, 0, 4.635087699967e-05, 5.134914715654e-11, 0.000000000000e+00,
//                      0.000000000000e+00, -4.329218750000e+02, 5.353080120131e-09, -2.712002342380e-01,
//                      -1.422502100468e-05, 7.832465926185e-04, 1.935660839081e-05, 6.493571424484e+03,
//                      4.320000000000e+04, 1.532025635242e-07, 3.117327342700e+00, 2.086162567139e-07,
//                      5.891477258533e-02, -5.948750000000e+02, 2.231594947116e+00, -4.226247468742e-09,
//                      -6.650277010731e-10, 0.000000000000e+00, 8.880000000000e+02, 0.000000000000e+00,
//                      1.200000000000e+00, 0.000000000000e+00, 2.300000000000e-09, 0.000000000000e+00,
//                      4.320000000000e+04, 4.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);

//     // C03 from Skydel
//     // BDSEphemeris eph(2023, 1, 8, 12, 0, 0, 4.635390359908e-05, 5.122746671304e-11, 0.000000000000e+00,
//     //                  0.000000000000e+00, -5.872812500000e+02, -9.432535760119e-10, -2.728725985133e-01,
//     //                  -1.913774758577e-05, 7.755858823657e-04, 1.467764377594e-05, 6.493572343826e+03,
//     //                  4.320000000000e+04, -1.606531441212e-07, -1.669895635113e-01, 1.611188054085e-07,
//     //                  1.145925602115e-01, -4.436406250000e+02, -7.670210318362e-01, 2.070443385135e-09,
//     //                  7.364592479123e-10, 0.000000000000e+00, 8.880000000000e+02, 0.000000000000e+00,
//     //                  2.000000000000e+00, 0.000000000000e+00, 2.300000000000e-09, -8.100000000000e-09,
//     //                  9.999000000000e+08, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);

//     Margin margin; // Determined by running the test and adapting
//     margin.pos = 1;
//     margin.vel = 1;
//     margin.accel = 1;

//     testEphemerisData({ BDS, 3 }, eph,
//                       "test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none/sat_data_V1A1.csv",
//                       B01, Spirent, margin);
// }

TEST_CASE("[Ephemeris] BDS Ephemeris calc orbit (Spirent SimGEN data) MEO/IGSO Satellite", "[Ephemeris]")
{
    // C28 - Exported from the Spirent SimGEN GUI
    // file: test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQ/Iono-none_tropo-none/Spirent_RINEX_CN.23C
    BDSEphemeris eph(/*28, */ 2023, 1, 8, 12, 0, 0, 5.650872060415e-05, 4.426681243785e-12, 0.000000000000e+00,
                     0.000000000000e+00, 1.461093750000e+02, 3.719440643918e-09, -3.037317177979e+00,
                     7.318798452616e-06, 2.175933914259e-04, 5.401205271482e-06, 5.282637744904e+03,
                     4.320000000000e+04, 1.862645149231e-09, -2.123098532827e+00, -5.122274160385e-08,
                     9.638845403744e-01, 2.496093750000e+02, -1.617035009667e+00, -6.857071338831e-09,
                     1.467918287546e-10, 0.000000000000e+00, 8.880000000000e+02, 0.000000000000e+00,
                     1.200000000000e+00, 0.000000000000e+00, -4.000000000000e-09, 0.000000000000e+00,
                     4.320000000000e+04, 4.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00);

    Margin margin; // Determined by running the test and adapting
    margin.pos = 8.2e-05;
    margin.vel = 6.9e-04;
    margin.accel = 4.7e-01;

    testEphemerisData({ BDS, 28 }, eph,
                      "test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none/sat_data_V1A1.csv",
                      B01, Spirent, margin);
}

} // namespace NAV::TESTS::EphemerisTests