// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file brdc0990_22g.hpp
/// @brief Test Data for the file brdc0990.22g
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-23

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

#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"

namespace NAV::TESTS::RinexNavFileTests::v2_01
{
/// @brief Test Data for the file brdc0990.22g
const GnssNavInfo gnssNavInfo_brdc0990_22g = {
    .satelliteSystems = { GLO },
    .ionosphericCorrections = {},
    .timeSysCorr = {
        { { GLNT, UTC }, { -0.931322574616e-09, 0.0 } },
    },
    .m_satellites = {
        { { GLO, 1 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GLONASSEphemeris>(2022, 4, 9, 0, 15, 0.0, 0.765733420849e-05, 0.000000000000e+00, 0.510000000000e+03, //
                                                                 -0.227034033203e+04, 0.303863525391e+00, 0.000000000000e+00, 0.000000000000e+00,    //
                                                                 -0.211325244141e+05, -0.190967178345e+01, -0.000000000000e+00, 0.100000000000e+01,  //
                                                                 -0.140861977539e+05, 0.281576919556e+01, 0.279396772385e-08, 0.000000000000e+00),   //
                              std::make_shared<GLONASSEphemeris>(2022, 4, 9, 0, 45, 0.0, 0.765733420849e-05, 0.000000000000e+00, 0.288000000000e+04, //
                                                                 -0.205547851562e+04, -0.342092514038e-01, 0.000000000000e+00, 0.000000000000e+00,   //
                                                                 -0.239387368164e+05, -0.117769908905e+01, -0.931322574616e-09, 0.100000000000e+01,  //
                                                                 -0.853815625000e+04, 0.330864620209e+01, 0.279396772385e-08, 0.000000000000e+00),   //
                          },
                      } },
        { { GLO, 10 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<GLONASSEphemeris>(2022, 4, 9, 0, 15, 0.0, -0.813333317637e-04, -0.000000000000e+00, 0.000000000000e+00, //
                                                                  0.138154658203e+05, 0.218341255188e+01, 0.000000000000e+00, 0.000000000000e+00,       //
                                                                  -0.116574462891e+04, 0.195369625092e+01, 0.186264514923e-08, -0.700000000000e+01,     //
                                                                  -0.214054721680e+05, 0.131031036377e+01, 0.931322574616e-09, 0.000000000000e+00),     //
                           },
                       } },
        { { GLO, 11 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<GLONASSEphemeris>(2022, 4, 9, 0, 45, 0.0, 0.416506081820e-04, -0.909494701773e-12, 0.180000000000e+04, //
                                                                  0.317718212891e+04, 0.253574752808e+01, 0.000000000000e+00, 0.000000000000e+00,      //
                                                                  -0.109698769531e+05, 0.182526683807e+01, 0.931322574616e-09, 0.000000000000e+00,     //
                                                                  -0.227999091797e+05, -0.525754928589e+00, 0.186264514923e-08, 0.000000000000e+00),   //
                           },
                       } },
    },
};

} // namespace NAV::TESTS::RinexNavFileTests::v2_01