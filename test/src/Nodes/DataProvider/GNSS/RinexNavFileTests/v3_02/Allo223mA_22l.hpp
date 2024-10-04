// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Allo223mA_22l.hpp
/// @brief Test Data for the file Allo223mA.22l
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-16

#pragma once

// This is a small hack, which lets us change private/protected parameters
#if defined(__clang__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "NodeData/GNSS/GnssNavInfo.hpp"
#undef protected
#undef private
#if defined(__clang__)
    #pragma GCC diagnostic pop
#endif

#include "Navigation/GNSS/Satellite/Ephemeris/GalileoEphemeris.hpp"

namespace NAV::TESTS::RinexNavFileTests::v3_02
{
/// @brief Test Data for the file Allo223mA.22l
const GnssNavInfo gnssNavInfo_Allo223mA_22l = {
    .satelliteSystems = { GAL },
    .ionosphericCorrections = { {
        { .satSys = GAL, .alphaBeta = IonosphericCorrections::Alpha, .data = { 0.9600e+02, 0.2891e+00, 0.1132e-01 } },
    } },
    .timeSysCorr = {
        { { GST, UTC }, { -0.1862645149e-08, 0.888178420e-15 } },
        { { GST, GPST }, { 0.1600710675e-08, 0.355271368e-14 } },
    },
    .m_satellites = {
        { { GAL, 12 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<GalileoEphemeris>(2022, 8, 11, 11, 10, 0, -0.109432556201e-03, -0.200373051484e-10, 0.000000000000e+00, //
                                                                  0.300000000000e+01, 0.152093750000e+03, 0.296619498250e-08, 0.594633784838e+00,       //
                                                                  0.720284879208e-05, 0.326088978909e-03, 0.126659870148e-06, 0.544061335754e+04,       //
                                                                  0.385800000000e+06, -0.167638063431e-07, 0.112395726561e+01, 0.149011611938e-06,      //
                                                                  0.994849589787e+00, 0.352406250000e+03, 0.101746623307e+01, -0.586381568003e-08,      //
                                                                  0.383944564261e-09, 0.513000000000e+03, 0.222200000000e+04, 0.000000000000e+00,       //
                                                                  0.312000000000e+01, 0.000000000000e+00, -0.102445483208e-07, -0.100117176771e-07,     //
                                                                  0.386485000000e+06, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00),      //
                           },
                       } },
        { { GAL, 4 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GalileoEphemeris>(2022, 8, 11, 11, 10, 0, -0.109109678306e-02, -0.828492829896e-11, 0.000000000000e+00, //
                                                                 0.300000000000e+01, -0.114000000000e+03, 0.270439836333e-08, 0.779660129094e+00,      //
                                                                 -0.545009970665e-05, 0.174158252776e-03, 0.133309513330e-04, 0.544064266777e+04,      //
                                                                 0.385800000000e+06, 0.912696123123e-07, -0.306056236727e+01, -0.223517417908e-07,     //
                                                                 0.957548474720e+00, 0.524375000000e+02, -0.127987640721e+01, -0.527771983822e-08,     //
                                                                 0.257153568621e-09, 0.516000000000e+03, 0.222200000000e+04, 0.000000000000e+00,       //
                                                                 0.312000000000e+01, 0.000000000000e+00, -0.279396772385e-08, -0.302679836750e-08,     //
                                                                 0.386464000000e+06, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00),      //
                          },
                      } },
        { { GAL, 10 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<GalileoEphemeris>(2022, 8, 11, 11, 50, 0, -0.506716956987e-03, -0.262900812231e-11, 0.000000000000e+00, //
                                                                  0.700000000000e+01, 0.152312500000e+03, 0.292583615853e-08, -0.529350941203e+00,      //
                                                                  0.723265111446e-05, 0.465923687443e-03, 0.381842255592e-06, 0.544060746765e+04,       //
                                                                  0.388200000000e+06, 0.130385160446e-07, 0.111965804342e+01, 0.149011611938e-06,       //
                                                                  0.997858687928e+00, 0.348312500000e+03, 0.204698977990e+01, -0.585167231707e-08,      //
                                                                  0.400016662300e-09, 0.513000000000e+03, 0.222200000000e+04, 0.000000000000e+00,       //
                                                                  0.360000000000e+01, 0.455000000000e+03, 0.465661287308e-09, 0.116415321827e-08,       //
                                                                  0.391965000000e+06, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00),      //
                           },
                       } },
    },
};

} // namespace NAV::TESTS::RinexNavFileTests::v3_02