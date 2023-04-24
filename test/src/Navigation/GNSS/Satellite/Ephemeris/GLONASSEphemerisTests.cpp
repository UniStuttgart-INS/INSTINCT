// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Common.hpp"

#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"

namespace NAV::TESTS::EphemerisTests
{

TEST_CASE("[Ephemeris] GLO Ephemeris calc orbit (ICD-GLONASS-5.1)", "[Ephemeris]")
{
    auto logger = initializeTestLogger();

    GLONASSEphemeris eph(2007, 11, 15, 6, 15, 0, 0.0, 0.0, 0.0,
                         -14081.752701, -1.02576358, 0.0, 0.0,
                         18358.958252, 1.08672147, 0.0, 0.0,
                         10861.302124, -3.15732343, 0.0, 0.0);
    LOG_TRACE("  eph.toc = {} ({})", eph.toc.toYMDHMS(), eph.toc.toGPSweekTow());

    InsTime transmitTime = InsTime{ 2007, 11, 15, 6, 30, 0.0 };
    LOG_TRACE("    transmitTime = {} ({})", transmitTime.toYMDHMS(), transmitTime.toGPSweekTow());

    auto posVel = eph.calcSatellitePosVel(transmitTime);

    Eigen::Vector3d e_refPos(-14836.563872e3, 19249.935476e3, 7924.017196e3);

    LOG_TRACE("    e_refPos {}", e_refPos.transpose());
    LOG_TRACE("    pos      {}", posVel.e_pos.transpose());
    LOG_TRACE("      pos - e_refPos   = {}", (posVel.e_pos - e_refPos).transpose());
    LOG_TRACE("    | pos - e_refPos | = {}", (posVel.e_pos - e_refPos).norm());
    REQUIRE_THAT((posVel.e_pos - e_refPos).norm(), Catch::Matchers::WithinAbs(0.0, 1.34)); // Value is high for a theoretical calculation

    Eigen::Vector3d e_refVel(-0.65397782e3, 0.88262958e3, -3.49667707e3);
    LOG_TRACE("    vel      {}", posVel.e_vel.transpose());
    LOG_TRACE("    e_refVel {}", e_refVel.transpose());
    LOG_TRACE("      vel - e_refVel   = {}", (posVel.e_vel - e_refVel).transpose());
    LOG_TRACE("    | vel - e_refVel | = {}", (posVel.e_vel - e_refVel).norm());
    REQUIRE_THAT((posVel.e_vel - e_refVel).norm(), Catch::Matchers::WithinAbs(0.0, 138.0)); // This value is really high, probably the ICD values are wrong
}

TEST_CASE("[Ephemeris] GLO Ephemeris calc orbit (ICD-GLONASS-CDMA-1.0)", "[Ephemeris]")
{
    auto logger = initializeTestLogger();

    GLONASSEphemeris eph(2012, 9, 7, 0, 0, 11700, 0.0, 0.0, 0.0,
                         7003.008789, 0.7835417, 0.0, 0.0,
                         -12206.626953, 2.8042530, 1.7e-9, 0.0,
                         21280.765625, 1.3525150, -5.41e-9, 0.0);
    LOG_TRACE("  eph.toc = {} ({})", eph.toc.toYMDHMS(), eph.toc.toGPSweekTow());

    InsTime transmitTime = InsTime{ 2012, 9, 7, 0, 0, 12300 };
    LOG_TRACE("    transmitTime = {} ({})", transmitTime.toYMDHMS(), transmitTime.toGPSweekTow());

    auto posVel = eph.calcSatellitePosVel(transmitTime);

    Eigen::Vector3d e_refPos(7523.174853e3, -10506.962176e3, 21999.239866e3);
    LOG_TRACE("    e_refPos {}", e_refPos.transpose());
    LOG_TRACE("    pos      {}", posVel.e_pos.transpose());
    LOG_TRACE("      pos - e_refPos   = {}", (posVel.e_pos - e_refPos).transpose());
    LOG_TRACE("    | pos - e_refPos | = {}", (posVel.e_pos - e_refPos).norm());
    REQUIRE_THAT((posVel.e_pos - e_refPos).norm(), Catch::Matchers::WithinAbs(0.0, 1.03)); // Value is high for a theoretical calculation

    Eigen::Vector3d e_refVel(0.95012609e3, 2.85568710e3, 1.04068137e3);
    LOG_TRACE("    vel      {}", posVel.e_vel.transpose());
    LOG_TRACE("    e_refVel {}", e_refVel.transpose());
    LOG_TRACE("      vel - e_refVel   = {}", (posVel.e_vel - e_refVel).transpose());
    LOG_TRACE("    | vel - e_refVel | = {}", (posVel.e_vel - e_refVel).norm());
    REQUIRE_THAT((posVel.e_vel - e_refVel).norm(), Catch::Matchers::WithinAbs(0.0, 3.5e-3));
}

TEST_CASE("[Ephemeris] GLO Ephemeris calc orbit (BRDC_20230080000)", "[Ephemeris]")
{
    // R13 - Taken from real data
    GLONASSEphemeris eph(2023, 1, 8, 11, 45, 0, -3.075692802668e-05, 0.000000000000e+00, 4.143000000000e+04,
                         -3.571277832031e+03, -1.539720535278e+00, -9.313225746155e-10, 0.000000000000e+00,
                         1.760026269531e+04, 1.948561668396e+00, -1.862645149231e-09, -2.000000000000e+00,
                         1.809717138672e+04, -2.195667266846e+00, -9.313225746155e-10, 0.000000000000e+00,
                         0.0, 0.0, 0.0, 0.0, -1.8626451492e-09); // GLUT TIME SYSTEM CORR

    // https://igs.org/products/
    // | Broadcast         | Accuracy     |
    // |     - orbits      | ~100 cm      |
    // |     - Sat. clocks | ~5   ns RMS  |
    // |                   | ~2.5 ns SDev |

    Margin margin; // Determined by running the test and adapting
    margin.clock = 2.9e-8;
    margin.pos = 4.5;

    testBrdcEphemerisData({ GLO, 13 }, eph, "test/data/GNSS/BRDC_20230080000/COD0OPSFIN_20230080000_01D_05M_ORB.SP3", margin);
}

TEST_CASE("[Ephemeris] GLO Ephemeris calc orbit (Orolia Skydel data)", "[Ephemeris]")
{
    // R13 - Generated from Orolia Skydel
    GLONASSEphemeris eph(2023, 1, 8, 11, 45, 0, -3.075692802668e-05, 0.000000000000e+00, 4.143000000000e+04,
                         -3.571277832031e+03, -1.539720535278e+00, -9.313225746155e-10, 0.000000000000e+00,
                         1.760026269531e+04, 1.948561668396e+00, -1.862645149231e-09, -2.000000000000e+00,
                         1.809717138672e+04, -2.195667266846e+00, -9.313225746155e-10, 0.000000000000e+00,
                         0.0, 0.0, 0.0, 0.0, -1.8626451492e-09); // GLUT TIME SYSTEM CORRR

    Margin margin;          // Determined by running the test and adapting
    margin.clock = 4.8e-10; // Orolia file has only 16 digits after comma // TODO: This value is too high
    margin.pos = 1.0;       // TODO: This value is too high for simulated values

    testEphemerisData({ GLO, 13 }, eph,
                      "test/data/GNSS/Orolia-Skydel_static_duration-4h_rate-5min_sys-GERCQIS_iono-none_tropo-none/sat_data/G1 13.csv",
                      R01, Skydel, margin);
}

TEST_CASE("[Ephemeris] GLO Ephemeris calc orbit (Spirent SimGEN data)", "[Ephemeris]")
{
    // R13 - Exported from the Spirent SimGEN GUI
    GLONASSEphemeris eph(2023, 1, 8, 11, 45, 0, -3.077182918787e-05, 0.000000000000e+00, 4.140000000000e+04,
                         -3.571975579373e+03, -1.538787529527e+00, 0.000000000000e+00, 0.000000000000e+00,
                         1.760483613624e+04, 1.948528507402e+00, 0.000000000000e+00, -2.000000000000e+00,
                         1.809266202785e+04, -2.196646735467e+00, 0.000000000000e+00, 0.000000000000e+00,
                         0.0, 0.0, 0.0, 0.0, 5.5879354477e-09); // GLUT TIME SYSTEM CORR

    Margin margin;     // Determined by running the test and adapting
    margin.pos = 2500; // TODO: Values are way too high
    margin.vel = 0.7;
    margin.accel = 5.3e-5;

    testEphemerisData({ GLO, 13 }, eph,
                      "test/data/GNSS/Spirent-SimGEN_static_duration-4h_rate-5min_sys-GERCQ_iono-none_tropo-none/sat_data_V1A1.csv",
                      R01, Spirent, margin);
}

} // namespace NAV::TESTS::EphemerisTests