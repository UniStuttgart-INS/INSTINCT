// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Allo223mA_22g.hpp
/// @brief Test Data for the file Allo223mA.22g
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-19

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

namespace NAV::TESTS::RinexNavFileTests::v2_11
{
/// @brief Test Data for the file Allo223mA.22g
const GnssNavInfo gnssNavInfo_Allo223mA_22g = {
    .satelliteSystems = { GLO },
    .ionosphericCorrections = {},
    .timeSysCorr = {
        { { GLNT, UTC }, { 0.139698386192e-08, 0.0 } },
    },
    .m_satellites = {
        { { GLO, 24 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<GLONASSEphemeris>(2022, 8, 11, 11, 45, 0.0, 0.108333304524e-03, 0.181898940355e-11, 0.431700000000e+05, //
                                                                  0.172943281250e+05, 0.181065464020e+01, 0.558793544769e-08, 0.000000000000e+00,       //
                                                                  0.550966552734e+04, 0.165955257416e+01, 0.000000000000e+00, 0.200000000000e+01,       //
                                                                  0.179224531250e+05, -0.226067447662e+01, 0.931322574615e-09, 0.000000000000e+00),     //
                           },
                       } },
        { { GLO, 2 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GLONASSEphemeris>(2022, 8, 11, 11, 45, 0.0, 0.526603311300e-03, 0.909494701773e-12, 0.431700000000e+05, //
                                                                 0.248142031250e+05, -0.453983306885e+00, 0.558793544769e-08, 0.000000000000e+00,      //
                                                                 -0.521476562500e+04, -0.665988922119e-01, 0.000000000000e+00, -0.400000000000e+01,    //
                                                                 0.311843701172e+04, 0.354068946838e+01, 0.279396772385e-08, 0.000000000000e+00),      //
                          },
                      } },
        { { GLO, 17 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<GLONASSEphemeris>(2022, 8, 11, 11, 45, 0.0, 0.551442615688e-03, 0.272848410532e-11, 0.431700000000e+05, //
                                                                  0.561331103516e+04, 0.223770999908e+01, 0.372529029846e-08, 0.000000000000e+00,       //
                                                                  -0.921201269531e+04, 0.224292469025e+01, 0.931322574615e-09, 0.400000000000e+01,      //
                                                                  0.231198164063e+05, 0.346097946167e+00, -0.931322574615e-09, 0.000000000000e+00),     //
                           },
                       } },
    },
};

} // namespace NAV::TESTS::RinexNavFileTests::v2_11