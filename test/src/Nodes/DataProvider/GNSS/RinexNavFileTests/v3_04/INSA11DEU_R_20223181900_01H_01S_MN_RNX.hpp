// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file INSA11DEU_R_20223181900_01H_01S_MN_RNX.hpp
/// @brief Test Data for the file INSA11DEU_R_20223181900_01H_01S_MN.rnx
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

#include "Navigation/GNSS/Satellite/Ephemeris/GPSEphemeris.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GalileoEphemeris.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/GLONASSEphemeris.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/BDSEphemeris.hpp"

namespace NAV::TESTS::RinexNavFileTests::v3_04
{
/// @brief Test Data for the file INSA11DEU_R_20223181900_01H_01S_MN.rnx
const GnssNavInfo gnssNavInfo_INSA11DEU_R_20223181900_01H_01S_MN_RNX = {
    .satelliteSystems = { GPS | GAL | GLO | BDS },
    .ionosphericCorrections = { {
        { .satSys = GPS, .alphaBeta = IonosphericCorrections::Alpha, .data = { .2515e-07, -.7451e-08, -.1192e-06, .1788e-06 } },
        { .satSys = GPS, .alphaBeta = IonosphericCorrections::Beta, .data = { .1331e+06, -.8192e+05, .6554e+05, -.3932e+06 } },
        { .satSys = GAL, .alphaBeta = IonosphericCorrections::Alpha, .data = { .1145e+03, -.6016e+00, .2533e-02, .0000e+00 } },
        { .satSys = BDS, .alphaBeta = IonosphericCorrections::Alpha, .data = { .2515e-07, .1937e-06, -.1550e-05, .2265e-05 } },
        { .satSys = BDS, .alphaBeta = IonosphericCorrections::Beta, .data = { .1454e+06, -.6226e+06, .2097e+07, -.5243e+06 } },
    } },
    .timeSysCorr = {
        { { GPST, UTC }, { .3725290298e-08, .115463195e-13 } },
        { { GST, UTC }, { -.9313225746e-09, .888178420e-15 } },
        { { BDT, UTC }, { -.2793967724e-08, -.195399252e-13 } },
        { { GST, GPST }, { -.1193257049e-08, -.666133815e-14 } },
    },
    .m_satellites = {
        { { GLO, 18 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<GLONASSEphemeris>(2022, 11, 14, 18, 45, 0, .519901514053e-04, .181898940355e-11, .154770000000e+06, //
                                                                  .560945361328e+04, -.265369510651e+01, -.186264514923e-08, .000000000000e+00,     //
                                                                  .127987656250e+05, -.123060035706e+01, .931322574615e-09, -.300000000000e+01,     //
                                                                  .213490947266e+05, .144019031525e+01, -.931322574615e-09, .000000000000e+00),     //
                           },
                       } },
        { { GPS, 32 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<GPSEphemeris>(2022, 11, 14, 20, 0, 0, -.296252779663e-03, -.126192389871e-10, .000000000000e+00, //
                                                              .520000000000e+02, -.325312500000e+02, .462376402690e-08, -.990179826541e+00,      //
                                                              -.157766044140e-05, .620053743478e-02, .826455652714e-05, .515367015839e+04,       //
                                                              .158400000000e+06, .968575477600e-07, .156173969794e+01, .109896063805e-06,        //
                                                              .958439399145e+00, .218718750000e+03, -.229821643394e+01, -.796961768085e-08,      //
                                                              .278940190407e-09, .100000000000e+01, .223600000000e+04, .000000000000e+00,        //
                                                              .200000000000e+01, .000000000000e+00, .465661287308e-09, .520000000000e+02,        //
                                                              .151218000000e+06, .400000000000e+01),                                             //
                           },
                       } },
        { { BDS, 46 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<BDSEphemeris>(46, 2022, 11, 14, 18, 0, 0, -.129309482872e-04, -.238742359215e-11, .000000000000e+00, //
                                                              .100000000000e+01, .248593750000e+02, .419124601079e-08, .240455317689e+01,            //
                                                              .118836760521e-05, .868363771588e-03, .719958916306e-05, .528261955261e+04,            //
                                                              .151200000000e+06, -.610016286373e-07, .305309685200e+01, .104308128357e-06,           //
                                                              .952371054031e+00, .209531250000e+03, .292177670716e-01, -.684564229134e-08,           //
                                                              -.228938107620e-09, .000000000000e+00, .880000000000e+03, .000000000000e+00,           //
                                                              .200000000000e+01, .000000000000e+00, .188999997874e-07, .189000000000e-07,            //
                                                              .151200000000e+06, .100000000000e+01),                                                 //
                           },
                       } },
        { { BDS, 36 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<BDSEphemeris>(36, 2022, 11, 14, 18, 0, 0, -.841412344016e-03, .290620860710e-10, .000000000000e+00, //
                                                              .100000000000e+01, .221562500000e+02, .431696553323e-08, .290629546728e+01,           //
                                                              .113528221846e-05, .679084099829e-03, .707898288965e-05, .528261561584e+04,           //
                                                              .151200000000e+06, .558793544769e-07, .305010345682e+01, .661239027977e-07,           //
                                                              .948412871694e+00, .209562500000e+03, -.121791834188e+01, -.691635952271e-08,         //
                                                              -.215008955986e-09, .000000000000e+00, .880000000000e+03, .000000000000e+00,          //
                                                              .200000000000e+01, .000000000000e+00, -.206999999364e-07, -.207000000000e-07,         //
                                                              .151200000000e+06, .100000000000e+01),                                                //
                           },
                       } },
        { { GLO, 9 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GLONASSEphemeris>(2022, 11, 14, 18, 45, 0, .875927507877e-04, .272848410532e-11, .154770000000e+06, //
                                                                 .113761904297e+05, .502119064331e+00, -.931322574615e-09, .000000000000e+00,      //
                                                                 -.403252929687e+03, .308004093170e+01, .931322574615e-09, -.200000000000e+01,     //
                                                                 .228420986328e+05, -.202746391296e+00, -.186264514923e-08, .000000000000e+00),    //
                              std::make_shared<GLONASSEphemeris>(2022, 11, 14, 19, 15, 0, .875983387232e-04, .181898940355e-11, .154800000000e+06, //
                                                                 .126376831055e+05, .880850791931e+00, -.931322574615e-09, .000000000000e+00,      //
                                                                 .494732031250e+04, .281835746765e+01, .931322574615e-09, -.200000000000e+01,      //
                                                                 .215999653320e+05, -.116867733002e+01, -.931322574615e-09, .000000000000e+00),    //
                          },
                      } },
        { { GPS, 2 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GPSEphemeris>(2022, 11, 14, 20, 0, 0, -.637842342257e-03, .170530256582e-11, .000000000000e+00, //
                                                             .750000000000e+02, -.906250000000e+00, .437946813671e-08, .138757332166e+01,      //
                                                             -.456348061562e-06, .199980732286e-01, .371411442757e-05, .515369688797e+04,      //
                                                             .158400000000e+06, .137835741043e-06, -.603558306340e+00, -.465661287308e-06,     //
                                                             .966908607876e+00, .309250000000e+03, -.136330628441e+01, -.817069748520e-08,     //
                                                             .407159816984e-10, .100000000000e+01, .223600000000e+04, .000000000000e+00,       //
                                                             .200000000000e+01, .000000000000e+00, -.176951289177e-07, .750000000000e+02,      //
                                                             .154758000000e+06, .400000000000e+01),                                            //
                          },
                      } },
        { { GAL, 21 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<GalileoEphemeris>(2022, 11, 14, 18, 20, 0, -.494624779094e-03, -.211741735257e-11, .000000000000e+00, //
                                                                  .126000000000e+03, .570000000000e+02, .301369696115e-08, .236341164000e+01,         //
                                                                  .266171991825e-05, .164222321473e-03, .810995697975e-05, .544062031364e+04,         //
                                                                  .152400000000e+06, -.298023223877e-07, -.269567917482e+01, .223517417908e-07,       //
                                                                  .971661875949e+00, .171375000000e+03, -.494903202499e+00, -.539093883996e-08,       //
                                                                  -.383587406527e-09, .513000000000e+03, .223600000000e+04, .000000000000e+00,        //
                                                                  .360000000000e+01, .000000000000e+00, .209547579288e-08, .209547579288e-08,         //
                                                                  .153085000000e+06),                                                                 //
                           },
                       } },
        { { GAL, 9 }, Satellite{
                          .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                              std::make_shared<GalileoEphemeris>(2022, 11, 14, 18, 40, 0, -.729114806745e-03, -.134292577059e-10, .000000000000e+00, //
                                                                 .000000000000e+00, -.470937500000e+02, .311584407313e-08, .102047271698e+01,        //
                                                                 -.222399830818e-05, .246543670073e-03, .887736678123e-05, .544060820198e+04,        //
                                                                 .153600000000e+06, .745058059692e-08, .148773533463e+01, -.372529029846e-07,        //
                                                                 .962891513828e+00, .149437500000e+03, .746967327399e+00, -.540272504518e-08,        //
                                                                 .326799326789e-09, .513000000000e+03, .223600000000e+04, .000000000000e+00,         //
                                                                 .312000000000e+01, .000000000000e+00, .139698386192e-08, .162981450558e-08,         //
                                                                 .154285000000e+06),                                                                 //
                          },
                      } },
    },
};

} // namespace NAV::TESTS::RinexNavFileTests::v3_04