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

namespace NAV::TESTS::RinexNavFileTests::v3_04
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
                               std::make_shared<GalileoEphemeris>(2022, 8, 11, 12, 10, 0, -0.109504500870e-03, -0.200373051484e-10, 0.000000000000e+00, //
                                                                  0.900000000000e+01, 0.168468750000e+03, 0.298941023522e-08, 0.104741317557e+01,       //
                                                                  0.792369246483e-05, 0.325802364387e-03, 0.264495611191e-06, 0.544060910225e+04,       //
                                                                  0.389400000000e+06, -0.875443220139e-07, 0.112393556760e+01, 0.540167093277e-07,      //
                                                                  0.994851118537e+00, 0.349281250000e+03, 0.101099767015e+01, -0.579702718374e-08,      //
                                                                  0.434660962517e-09, 0.513000000000e+03, 0.222200000000e+04, 0.000000000000e+00,       //
                                                                  0.312000000000e+01, 0.000000000000e+00, -0.102445483208e-07, -0.100117176771e-07,     //
                                                                  0.390175000000e+06, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00),      //
                           },
                       } },
        { { GAL, 25 }, Satellite{
                           .m_navigationData /* std::vector<std::shared_ptr<SatNavData>> */ = {
                               std::make_shared<GalileoEphemeris>(2022, 8, 11, 11, 0, 0, -0.584173365496e-03, -0.126476606965e-11, 0.000000000000e+00, //
                                                                  0.200000000000e+01, -0.361562500000e+02, 0.322799160166e-08, -0.274072844385e+01,    //
                                                                  -0.165775418282e-05, 0.351418275386e-03, 0.733882188797e-05, 0.544060129356e+04,     //
                                                                  0.385200000000e+06, 0.298023223877e-07, -0.963250625106e+00, 0.502914190292e-07,     //
                                                                  0.971961899966e+00, 0.184406250000e+03, -0.150865615157e+01, -0.547987111577e-08,    //
                                                                  -0.606096674931e-09, 0.513000000000e+03, 0.222200000000e+04, 0.000000000000e+00,     //
                                                                  0.312000000000e+01, 0.000000000000e+00, 0.372529029846e-08, 0.419095158577e-08,      //
                                                                  0.385885000000e+06, 0.000000000000e+00, 0.000000000000e+00, 0.000000000000e+00),     //
                           },
                       } },
    },
};

} // namespace NAV::TESTS::RinexNavFileTests::v3_04