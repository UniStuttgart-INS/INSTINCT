// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file reach-m2-01_22O.hpp
/// @brief Test data for the reach-m2-01_raw_202211021639_test.22O
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2023-01-11

#pragma once

#include "NodeData/GNSS/GnssObs.hpp"

namespace NAV::TESTS::RinexObsFileTests::v3_03
{
/// @brief Test Data for the file reach-m2-01_raw_202211021639_test.22O
const std::vector<GnssObs> gnssObs_reach_m2_01_22O = {
    { /* .insTime = */ InsTime{ 2022, 11, 2, 16, 39, 59.6920000, GPST },
      /* .data = */ std::vector<GnssObs::ObservationData>{
          GnssObs::ObservationData{ /* .satSigId =     */ { G01, 1 },
                                    /* .code =         */ Code::G1C,
                                    /* .pseudorange =  */ { .value = 22876259.395, .SSI = 1 },
                                    /* .carrierPhase = */ { .value = 120215551.306, .SSI = 3, .LLI = 0 },
                                    /* .doppler =      */ 562.461,
                                    /* .CN0 =          */ 36.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { G02, 1 },
                                    /* .code =         */ Code::G2X,
                                    /* .pseudorange =  */ { .value = 22876256.297, .SSI = 2 },
                                    /* .carrierPhase = */ { .value = 93674506.094, .SSI = 4, .LLI = 0 },
                                    /* .doppler =      */ 438.697,
                                    /* .CN0 =          */ 34.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { G01, 13 },
                                    /* .code =         */ Code::G1C,
                                    /* .pseudorange =  */ { .value = 19627552.460, .SSI = 1 },
                                    /* .carrierPhase = */ { .value = 103143484.484, .SSI = 1, .LLI = 0 },
                                    /* .doppler =      */ -3581.727,
                                    /* .CN0 =          */ 46.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { R01, 1 },
                                    /* .code =         */ Code::R1C,
                                    /* .pseudorange =  */ { .value = 18660381.325, .SSI = 1 },
                                    /* .carrierPhase = */ { .value = 99750420.719, .SSI = 1, .LLI = 0 },
                                    /* .doppler =      */ -4129.152,
                                    /* .CN0 =          */ 46.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { R02, 1 },
                                    /* .code =         */ Code::R2C,
                                    /* .pseudorange =  */ { .value = 18660383.569, .SSI = 1 },
                                    /* .carrierPhase = */ { .value = 77583668.925, .SSI = 2, .LLI = 0 },
                                    /* .doppler =      */ -3211.629,
                                    /* .CN0 =          */ 42.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { R01, 10 },
                                    /* .code =         */ Code::R1C,
                                    /* .pseudorange =  */ { .value = 21657654.213, .SSI = 1 },
                                    /* .carrierPhase = */ { .value = 115447492.344, .SSI = 2, .LLI = 0 },
                                    /* .doppler =      */ -4203.686,
                                    /* .CN0 =          */ 41.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { R01, 13 },
                                    /* .code =         */ Code::R1C,
                                    /* .pseudorange =  */ { .value = 21689411.389, .SSI = 4 },
                                    /* .carrierPhase = */ { .value = 115820280.545, .SSI = 5, .LLI = 0 },
                                    /* .doppler =      */ 3020.874,
                                    /* .CN0 =          */ 32.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { R02, 13 },
                                    /* .code =         */ Code::R2C,
                                    /* .pseudorange =  */ { .value = 21689421.083, .SSI = 4 },
                                    /* .carrierPhase = */ { .value = 0, .SSI = 0, .LLI = 0 }, // '.value = 0' since 'carrierPhase = NaN', but pseudorange exists. This happens if the CN0 is so small that the PLL could not lock, even if the DLL has locked (= pseudorange available). The observation is still valid.
                                    /* .doppler =      */ 2349.882,
                                    /* .CN0 =          */ 26.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { E01, 3 },
                                    /* .code =         */ Code::E1X,
                                    /* .pseudorange =  */ { .value = 22094318.881, .SSI = 1 },
                                    /* .carrierPhase = */ { .value = 116106429.676, .SSI = 1, .LLI = 0 },
                                    /* .doppler =      */ -2446.293,
                                    /* .CN0 =          */ 46.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { E07, 3 },
                                    /* .code =         */ Code::E7X,
                                    /* .pseudorange =  */ { .value = 22094320.071, .SSI = 1 },
                                    /* .carrierPhase = */ { .value = 88964669.099, .SSI = 1, .LLI = 0 },
                                    /* .doppler =      */ -1874.433,
                                    /* .CN0 =          */ 50.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { E01, 5 },
                                    /* .code =         */ Code::E1X,
                                    /* .pseudorange =  */ { .value = 25830241.475, .SSI = 1 },
                                    /* .carrierPhase = */ { .value = 135738837.274, .SSI = 3, .LLI = 0 },
                                    /* .doppler =      */ -3677.055,
                                    /* .CN0 =          */ 37.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { B07, 7 },
                                    /* .code =         */ Code::B7I,
                                    /* .pseudorange =  */ { .value = 37737385.511, .SSI = 1 },
                                    /* .carrierPhase = */ { .value = 151952809.467, .SSI = 2, .LLI = 0 },
                                    /* .doppler =      */ -1170.329,
                                    /* .CN0 =          */ 40.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { B02, 10 },
                                    /* .code =         */ Code::B2I,
                                    /* .pseudorange =  */ { .value = 36759786.025, .SSI = 1 },
                                    /* .carrierPhase = */ { .value = 191417848.535, .SSI = 2, .LLI = 0 },
                                    /* .doppler =      */ -1192.425,
                                    /* .CN0 =          */ 42.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { B07, 10 },
                                    /* .code =         */ Code::B7I,
                                    /* .pseudorange =  */ { .value = 36759779.463, .SSI = 1 },
                                    /* .carrierPhase = */ { .value = 148016392.042, .SSI = 2, .LLI = 0 },
                                    /* .doppler =      */ -921.977,
                                    /* .CN0 =          */ 40.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { B02, 23 },
                                    /* .code =         */ Code::B2I,
                                    /* .pseudorange =  */ { .value = 19672914.865, .SSI = 1 },
                                    /* .carrierPhase = */ { .value = 102442028.933, .SSI = 1, .LLI = 0 },
                                    /* .doppler =      */ 96.813,
                                    /* .CN0 =          */ 50.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { S01, 23 },
                                    /* .code =         */ Code::S1C,
                                    /* .pseudorange =  */ { .value = 36325088.073, .SSI = 1 },
                                    /* .carrierPhase = */ { .value = 190889609.539, .SSI = 1, .LLI = 0 },
                                    /* .doppler =      */ -535.350,
                                    /* .CN0 =          */ 44.000 },
      } },
};

} // namespace NAV::TESTS::RinexObsFileTests::v3_03