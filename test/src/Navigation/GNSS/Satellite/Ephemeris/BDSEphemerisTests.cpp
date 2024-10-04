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
    BDSEphemeris eph(1, 2023, 1, 8, 12, 0, 0, 9.214587043971e-04, -3.444355911597e-12, 0.000000000000e+00,
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
    BDSEphemeris eph(6, 2023, 1, 8, 12, 0, 0, 2.072051865980e-04, -3.074873689002e-12, 0.000000000000e+00,
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
    BDSEphemeris eph(1, 2023, 1, 8, 12, 0, 0, 9.214587043971e-04, -3.444355911597e-12, 0.000000000000e+00,
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
    BDSEphemeris eph(6, 2023, 1, 8, 11, 0, 0, 2.073161779492e-04, -3.033129303276e-12, 0.000000000000e+00,
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

TEST_CASE("[Ephemeris] BDS Ephemeris calc orbit (Spirent SimGEN data) MEO/IGSO Satellite", "[Ephemeris]")
{
    // C28 - Exported from the Spirent SimGEN GUI
    // file: test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQ/Iono-none_tropo-none/Spirent_RINEX_CN.23C
    BDSEphemeris eph(28, 2023, 1, 8, 12, 0, 0, 5.650872060415e-05, 4.426681243785e-12, 0.000000000000e+00,
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

TEST_CASE("[Ephemeris] BDS Ephemeris calc orbit (Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none)", "[Ephemeris][flow]")
{
    // Margins determined by running the test and adapting
    Margin margin_GroupA_B1I{ .clock = 0, .pos = 8.6e-5, .vel = 8.4e-4, .accel = 8.1e-1 };
    Margin margin_GroupB_B2I{ .clock = 0, .pos = 2.0e-4, .vel = 8.4e-4, .accel = 8.1e-1 };
    Margin margin_GroupC_B2A{ .clock = 0, .pos = 8.6e-5, .vel = 8.4e-4, .accel = 8.1e-1 };
    Margin margin_GroupD_B1C{ .clock = 0, .pos = 8.6e-5, .vel = 8.4e-4, .accel = 8.1e-1 };
    Margin margin_GroupE_B3I{ .clock = 0, .pos = 8.6e-5, .vel = 8.4e-4, .accel = 8.1e-1 };
    Margin margin_GroupF_B2b{ .clock = 0, .pos = 8.6e-5, .vel = 8.4e-4, .accel = 8.1e-1 };

    std::string path = "test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Iono-none_tropo-none/sat_data_V1A1.csv";
    std::vector<std::tuple<SatSigId, std::string, Margin>> files = {
        // { SatSigId(Code::B2I, 1), path, margin_GroupA_B1I }, // GEO Satellite
        // { SatSigId(Code::B2I, 2), path, margin_GroupA_B1I }, // GEO Satellite
        // { SatSigId(Code::B2I, 3), path, margin_GroupA_B1I }, // GEO Satellite
        // { SatSigId(Code::B2I, 4), path, margin_GroupA_B1I }, // GEO Satellite
        // { SatSigId(Code::B2I, 5), path, margin_GroupA_B1I }, // GEO Satellite
        { SatSigId(Code::B2I, 6), path, margin_GroupA_B1I },
        { SatSigId(Code::B2I, 7), path, margin_GroupA_B1I },
        { SatSigId(Code::B2I, 8), path, margin_GroupA_B1I },
        { SatSigId(Code::B2I, 9), path, margin_GroupA_B1I },
        { SatSigId(Code::B2I, 10), path, margin_GroupA_B1I },
        { SatSigId(Code::B2I, 13), path, margin_GroupA_B1I },
        { SatSigId(Code::B2I, 16), path, margin_GroupA_B1I },
        { SatSigId(Code::B2I, 20), path, margin_GroupA_B1I },
        { SatSigId(Code::B2I, 23), path, margin_GroupA_B1I },
        { SatSigId(Code::B2I, 27), path, margin_GroupA_B1I },
        { SatSigId(Code::B2I, 28), path, margin_GroupA_B1I },
        { SatSigId(Code::B2I, 29), path, margin_GroupA_B1I },
        { SatSigId(Code::B2I, 30), path, margin_GroupA_B1I },

        // { SatSigId(Code::B7I, 1), path, margin_GroupB_B2I }, // GEO Satellite
        // { SatSigId(Code::B7I, 2), path, margin_GroupB_B2I }, // GEO Satellite
        // { SatSigId(Code::B7I, 3), path, margin_GroupB_B2I }, // GEO Satellite
        // { SatSigId(Code::B7I, 4), path, margin_GroupB_B2I }, // GEO Satellite
        // { SatSigId(Code::B7I, 5), path, margin_GroupB_B2I }, // GEO Satellite
        { SatSigId(Code::B7I, 6), path, margin_GroupB_B2I },
        { SatSigId(Code::B7I, 7), path, margin_GroupB_B2I },
        { SatSigId(Code::B7I, 8), path, margin_GroupB_B2I },
        { SatSigId(Code::B7I, 9), path, margin_GroupB_B2I },
        { SatSigId(Code::B7I, 10), path, margin_GroupB_B2I },
        { SatSigId(Code::B7I, 13), path, margin_GroupB_B2I },
        { SatSigId(Code::B7I, 16), path, margin_GroupB_B2I },
        { SatSigId(Code::B7I, 20), path, margin_GroupB_B2I },
        { SatSigId(Code::B7I, 23), path, margin_GroupB_B2I },
        { SatSigId(Code::B7I, 27), path, margin_GroupB_B2I },
        { SatSigId(Code::B7I, 28), path, margin_GroupB_B2I },
        { SatSigId(Code::B7I, 29), path, margin_GroupB_B2I },
        { SatSigId(Code::B7I, 30), path, margin_GroupB_B2I },

        // Not sure if code correct
        // { SatSigId(Code::B5X, 1), path, margin_GroupC_B2A }, // GEO Satellite
        // { SatSigId(Code::B5X, 2), path, margin_GroupC_B2A }, // GEO Satellite
        // { SatSigId(Code::B5X, 3), path, margin_GroupC_B2A }, // GEO Satellite
        // { SatSigId(Code::B5X, 4), path, margin_GroupC_B2A }, // GEO Satellite
        // { SatSigId(Code::B5X, 5), path, margin_GroupC_B2A }, // GEO Satellite
        { SatSigId(Code::B5X, 6), path, margin_GroupC_B2A },
        { SatSigId(Code::B5X, 7), path, margin_GroupC_B2A },
        { SatSigId(Code::B5X, 8), path, margin_GroupC_B2A },
        { SatSigId(Code::B5X, 9), path, margin_GroupC_B2A },
        { SatSigId(Code::B5X, 10), path, margin_GroupC_B2A },
        { SatSigId(Code::B5X, 13), path, margin_GroupC_B2A },
        { SatSigId(Code::B5X, 16), path, margin_GroupC_B2A },
        { SatSigId(Code::B5X, 20), path, margin_GroupC_B2A },
        { SatSigId(Code::B5X, 23), path, margin_GroupC_B2A },
        { SatSigId(Code::B5X, 27), path, margin_GroupC_B2A },
        { SatSigId(Code::B5X, 28), path, margin_GroupC_B2A },
        { SatSigId(Code::B5X, 29), path, margin_GroupC_B2A },
        { SatSigId(Code::B5X, 30), path, margin_GroupC_B2A },

        // Not sure if code correct
        // { SatSigId(Code::B1X, 1), path, margin_GroupD_B1C }, // GEO Satellite
        // { SatSigId(Code::B1X, 2), path, margin_GroupD_B1C }, // GEO Satellite
        // { SatSigId(Code::B1X, 3), path, margin_GroupD_B1C }, // GEO Satellite
        // { SatSigId(Code::B1X, 4), path, margin_GroupD_B1C }, // GEO Satellite
        // { SatSigId(Code::B1X, 5), path, margin_GroupD_B1C }, // GEO Satellite
        { SatSigId(Code::B1X, 6), path, margin_GroupD_B1C },
        { SatSigId(Code::B1X, 7), path, margin_GroupD_B1C },
        { SatSigId(Code::B1X, 8), path, margin_GroupD_B1C },
        { SatSigId(Code::B1X, 9), path, margin_GroupD_B1C },
        { SatSigId(Code::B1X, 10), path, margin_GroupD_B1C },
        { SatSigId(Code::B1X, 13), path, margin_GroupD_B1C },
        { SatSigId(Code::B1X, 16), path, margin_GroupD_B1C },
        { SatSigId(Code::B1X, 20), path, margin_GroupD_B1C },
        { SatSigId(Code::B1X, 23), path, margin_GroupD_B1C },
        { SatSigId(Code::B1X, 27), path, margin_GroupD_B1C },
        { SatSigId(Code::B1X, 28), path, margin_GroupD_B1C },
        { SatSigId(Code::B1X, 29), path, margin_GroupD_B1C },
        { SatSigId(Code::B1X, 30), path, margin_GroupD_B1C },

        // { SatSigId(Code::B6I, 1), path, margin_GroupE_B3I }, // GEO Satellite
        // { SatSigId(Code::B6I, 2), path, margin_GroupE_B3I }, // GEO Satellite
        // { SatSigId(Code::B6I, 3), path, margin_GroupE_B3I }, // GEO Satellite
        // { SatSigId(Code::B6I, 4), path, margin_GroupE_B3I }, // GEO Satellite
        // { SatSigId(Code::B6I, 5), path, margin_GroupE_B3I }, // GEO Satellite
        { SatSigId(Code::B6I, 6), path, margin_GroupE_B3I },
        { SatSigId(Code::B6I, 7), path, margin_GroupE_B3I },
        { SatSigId(Code::B6I, 8), path, margin_GroupE_B3I },
        { SatSigId(Code::B6I, 9), path, margin_GroupE_B3I },
        { SatSigId(Code::B6I, 10), path, margin_GroupE_B3I },
        { SatSigId(Code::B6I, 13), path, margin_GroupE_B3I },
        { SatSigId(Code::B6I, 16), path, margin_GroupE_B3I },
        { SatSigId(Code::B6I, 20), path, margin_GroupE_B3I },
        { SatSigId(Code::B6I, 23), path, margin_GroupE_B3I },
        { SatSigId(Code::B6I, 27), path, margin_GroupE_B3I },
        { SatSigId(Code::B6I, 28), path, margin_GroupE_B3I },
        { SatSigId(Code::B6I, 29), path, margin_GroupE_B3I },
        { SatSigId(Code::B6I, 30), path, margin_GroupE_B3I },

        // Not sure if code correct
        // { SatSigId(Code::B8X, 1), path, margin_GroupF_B2b }, // GEO Satellite
        // { SatSigId(Code::B8X, 2), path, margin_GroupF_B2b }, // GEO Satellite
        // { SatSigId(Code::B8X, 3), path, margin_GroupF_B2b }, // GEO Satellite
        // { SatSigId(Code::B8X, 4), path, margin_GroupF_B2b }, // GEO Satellite
        // { SatSigId(Code::B8X, 5), path, margin_GroupF_B2b }, // GEO Satellite
        { SatSigId(Code::B8X, 6), path, margin_GroupF_B2b },
        { SatSigId(Code::B8X, 7), path, margin_GroupF_B2b },
        { SatSigId(Code::B8X, 8), path, margin_GroupF_B2b },
        { SatSigId(Code::B8X, 9), path, margin_GroupF_B2b },
        { SatSigId(Code::B8X, 10), path, margin_GroupF_B2b },
        { SatSigId(Code::B8X, 13), path, margin_GroupF_B2b },
        { SatSigId(Code::B8X, 16), path, margin_GroupF_B2b },
        { SatSigId(Code::B8X, 20), path, margin_GroupF_B2b },
        { SatSigId(Code::B8X, 23), path, margin_GroupF_B2b },
        { SatSigId(Code::B8X, 27), path, margin_GroupF_B2b },
        { SatSigId(Code::B8X, 28), path, margin_GroupF_B2b },
        { SatSigId(Code::B8X, 29), path, margin_GroupF_B2b },
        { SatSigId(Code::B8X, 30), path, margin_GroupF_B2b },
    };

    testNavFile(Spirent, "GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQI/Spirent_RINEX_CN.23C", files);
}

} // namespace NAV::TESTS::EphemerisTests