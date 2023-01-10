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
#pragma GCC diagnostic push
#if defined(__clang__)
    #pragma GCC diagnostic ignored "-Wkeyword-macro"
    #pragma GCC diagnostic ignored "-Wmacro-redefined"
#endif
#define protected public
#define private public
#include "NodeData/GNSS/GnssNavInfo.hpp"
#undef protected
#undef private
#pragma GCC diagnostic pop

#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"

namespace NAV::TESTS::RinexNavFileTests::v3_04
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
                               std::make_shared<GLONASSEphemeris>(2022, 8, 11, 11, 45, 0, 0.108333304524e-03, 0.181898940355e-11, 0.388770000000e+06, //
                                                                  0.172943281250e+05, 0.181065464020e+01, 0.558793544769e-08, 0.000000000000e+00,     //
                                                                  0.550966552734e+04, 0.165955257416e+01, 0.000000000000e+00, 0.200000000000e+01,     //
                                                                  0.179224531250e+05, -0.226067447662e+01, 0.931322574615e-09, 0.000000000000e+00),   //
                               std::make_shared<GLONASSEphemeris>(2022, 8, 11, 12, 15, 0, 0.108337029815e-03, 0.181898940355e-11, 0.388800000000e+06, //
                                                                  0.203368964844e+05, 0.152326011658e+01, 0.558793544769e-08, 0.000000000000e+00,     //
                                                                  0.789213378906e+04, 0.986822128296e+00, -0.186264514923e-08, 0.200000000000e+01,    //
                                                                  0.132128681641e+05, -0.293827724457e+01, 0.931322574615e-09, 0.000000000000e+00),   //
                           },
                       } },
        { { GLO, 2 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GLONASSEphemeris>(2022, 8, 11, 11, 45, 0, 0.526603311300e-03, 0.909494701773e-12, 0.388770000000e+06, //
                                                                 0.248142031250e+05, -0.453983306885e+00, 0.558793544769e-08, 0.000000000000e+00,    //
                                                                 -0.521476562500e+04, -0.665988922119e-01, 0.000000000000e+00, -0.400000000000e+01,  //
                                                                 0.311843701172e+04, 0.354068946838e+01, 0.279396772385e-08, 0.000000000000e+00),    //
                          },
                      } },
        { { GLO, 1 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GLONASSEphemeris>(2022, 8, 11, 12, 15, 0, 0.129472464323e-04, 0.000000000000e+00, 0.388800000000e+06, //
                                                                 0.110683095703e+05, -0.276156330109e+01, 0.465661287308e-08, 0.000000000000e+00,    //
                                                                 0.655451269531e+04, 0.121209335327e+01, -0.931322574615e-09, 0.100000000000e+01,    //
                                                                 0.220235732422e+05, 0.102819824219e+01, 0.000000000000e+00, 0.000000000000e+00),    //
                          },
                      } },
    },
};

} // namespace NAV::TESTS::RinexNavFileTests::v3_04