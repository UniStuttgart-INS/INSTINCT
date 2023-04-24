// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file INSA11DEU_R_20223182100_01H_01S_EN_RNX.hpp
/// @brief Test Data for the file INSA11DEU_R_20223182100_01H_01S_EN.rnx
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-12-16

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

#include "Navigation/GNSS/Satellite/Ephemeris/GalileoEphemeris.hpp"

namespace NAV::TESTS::RinexNavFileTests::v3_02
{
/// @brief Test Data for the file INSA11DEU_R_20223182100_01H_01S_EN.rnx
const GnssNavInfo gnssNavInfo_INSA11DEU_R_20223182100_01H_01S_EN_RNX = {
    .satelliteSystems = { GAL },
    .ionosphericCorrections = { {
        { .satSys = GAL, .alphaBeta = IonosphericCorrections::Alpha, .data = { .1145e+03, -.6016e+00, .2533e-02, .0000e+00 } },
    } },
    .timeSysCorr = {
        { { GST, UTC }, { -.9313225746e-09, .888178420e-15 } },
        { { GST, GPST }, { -.1193257049e-08, -.666133815e-14 } },
    },
    .m_satellites = {
        { { GAL, 5 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GalileoEphemeris>(2022, 11, 14, 20, 30, 0, -.124636688270e-03, .366640051652e-11, .000000000000e+00, //
                                                                 .110000000000e+02, -.470937500000e+02, .316191742084e-08, -.407095693627e-01,      //
                                                                 -.226683914661e-05, .164534430951e-03, .877305865288e-05, .544062120819e+04,       //
                                                                 .160200000000e+06, .223517417908e-07, .149016691660e+01, -.428408384323e-07,       //
                                                                 .956948766083e+00, .147875000000e+03, .184043411793e+01, -.545236997024e-08,       //
                                                                 .313941648358e-09, .513000000000e+03, .223600000000e+04, .000000000000e+00,        //
                                                                 .312000000000e+01, .000000000000e+00, .302679836750e-08, .349245965481e-08,        //
                                                                 .160885000000e+06, .000000000000e+00, .000000000000e+00, .000000000000e+00),       //
                              std::make_shared<GalileoEphemeris>(2022, 11, 14, 21, 40, 0, -.124620506540e-03, .369482222595e-11, .000000000000e+00, //
                                                                 .180000000000e+02, -.565937500000e+02, .311798701953e-08, .473069240867e+00,       //
                                                                 -.262819230556e-05, .165554578416e-03, .848993659019e-05, .544062428093e+04,       //
                                                                 .164400000000e+06, .242143869400e-07, .149014418285e+01, -.558793544769e-08,       //
                                                                 .956949992008e+00, .155218750000e+03, .184735144707e+01, -.547737101163e-08,       //
                                                                 .301798285396e-09, .513000000000e+03, .223600000000e+04, .000000000000e+00,        //
                                                                 .312000000000e+01, .000000000000e+00, .302679836750e-08, .349245965481e-08,        //
                                                                 .165115000000e+06, .000000000000e+00, .000000000000e+00, .000000000000e+00),       //
                          },
                      } },
        { { GAL, 33 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<GalileoEphemeris>(2022, 11, 14, 20, 30, 0, -.455487170257e-03, -.852651282912e-13, .000000000000e+00, //
                                                                  .110000000000e+02, .142500000000e+02, .257939315636e-08, .167521105140e+01,         //
                                                                  .607222318649e-06, .219886656851e-03, .246427953243e-05, .544060475922e+04,         //
                                                                  .160200000000e+06, -.279396772385e-07, -.608799009940e+00, .130385160446e-07,       //
                                                                  .998844201710e+00, .304437500000e+03, -.167249202943e+01, -.557166065346e-08,       //
                                                                  .225009372544e-10, .513000000000e+03, .223600000000e+04, .000000000000e+00,         //
                                                                  .312000000000e+01, .000000000000e+00, -.395812094212e-08, -.442378222942e-08,       //
                                                                  .160885000000e+06, .000000000000e+00, .000000000000e+00, .000000000000e+00),        //
                               std::make_shared<GalileoEphemeris>(2022, 11, 14, 21, 40, 0, -.455487635918e-03, -.994759830064e-13, .000000000000e+00, //
                                                                  .180000000000e+02, .195000000000e+02, .256939273981e-08, .219390240491e+01,         //
                                                                  .862404704094e-06, .219382112846e-03, .245682895184e-05, .544060609627e+04,         //
                                                                  .164400000000e+06, -.409781932831e-07, -.608822422481e+00, -.130385160446e-07,      //
                                                                  .998844311429e+00, .304781250000e+03, -.167048546028e+01, -.557773233494e-08,       //
                                                                  .192865176466e-10, .513000000000e+03, .223600000000e+04, .000000000000e+00,         //
                                                                  .312000000000e+01, .000000000000e+00, -.395812094212e-08, -.442378222942e-08,       //
                                                                  .165085000000e+06, .000000000000e+00, .000000000000e+00, .000000000000e+00),        //
                           },
                       } },
        { { GAL, 26 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<GalileoEphemeris>(2022, 11, 14, 21, 40, 0, -.184547633398e-02, -.439825953436e-10, .000000000000e+00, //
                                                                  .180000000000e+02, .141875000000e+02, .269761236638e-08, -.270162077856e+01,        //
                                                                  .685453414917e-06, .302730477415e-03, .177323818207e-05, .544060042763e+04,         //
                                                                  .164400000000e+06, -.558793544769e-07, -.606283343826e+00, -.707805156708e-07,      //
                                                                  .992149057642e+00, .319937500000e+03, -.231370534097e+01, -.561023368875e-08,       //
                                                                  -.178578867098e-11, .513000000000e+03, .223600000000e+04, .000000000000e+00,        //
                                                                  .312000000000e+01, .000000000000e+00, -.419095158577e-08, -.465661287308e-08,       //
                                                                  .165085000000e+06, .000000000000e+00, .000000000000e+00, .000000000000e+00),        //
                           },
                       } },
    },
};

} // namespace NAV::TESTS::RinexNavFileTests::v3_02