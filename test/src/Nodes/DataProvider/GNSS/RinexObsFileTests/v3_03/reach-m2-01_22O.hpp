// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file reach-m2-01_22O.hpp
/// @brief Test data for the reach-m2-01_raw.22O
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2023-01-11

#pragma once

#include "NodeData/GNSS/GnssObs.hpp"

using Pseudorange = NAV::GnssObs::ObservationData::Pseudorange;
using CarrierPhase = NAV::GnssObs::ObservationData::CarrierPhase;

namespace NAV::TESTS::RinexObsFileTests::v3_03
{
/// @brief Test Data for the file reach-m2-01_raw.22O
const std::vector<GnssObs> gnssObs_reach_m2_01_22O = {
    { /* .insTime = */ InsTime{ 2022, 11, 2, 16, 39, 59.6920000, GPST },
      /* .data = */ std::vector<GnssObs::ObservationData>{
          GnssObs::ObservationData{ /* .satSigId =     */ { Code::G1C, 1 },
                                    /* .pseudorange =  */ Pseudorange{ .value = 22876259.395, .SSI = 1 },
                                    /* .carrierPhase = */ CarrierPhase{ .value = 120215551.306, .SSI = 3, .LLI = 0 },
                                    /* .doppler =      */ 562.461,
                                    /* .CN0 =          */ 36.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { Code::G2X, 1 },
                                    /* .pseudorange =  */ Pseudorange{ .value = 22876256.297, .SSI = 2 },
                                    /* .carrierPhase = */ CarrierPhase{ .value = 93674506.094, .SSI = 4, .LLI = 0 },
                                    /* .doppler =      */ 438.697,
                                    /* .CN0 =          */ 34.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { Code::G1C, 13 },
                                    /* .pseudorange =  */ Pseudorange{ .value = 19627552.460, .SSI = 1 },
                                    /* .carrierPhase = */ CarrierPhase{ .value = 103143484.484, .SSI = 1, .LLI = 0 },
                                    /* .doppler =      */ -3581.727,
                                    /* .CN0 =          */ 46.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { Code::R1C, 1 },
                                    /* .pseudorange =  */ Pseudorange{ .value = 18660381.325, .SSI = 1 },
                                    /* .carrierPhase = */ CarrierPhase{ .value = 99750420.719, .SSI = 1, .LLI = 0 },
                                    /* .doppler =      */ -4129.152,
                                    /* .CN0 =          */ 46.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { Code::R2C, 1 },
                                    /* .pseudorange =  */ Pseudorange{ .value = 18660383.569, .SSI = 1 },
                                    /* .carrierPhase = */ CarrierPhase{ .value = 77583668.925, .SSI = 2, .LLI = 0 },
                                    /* .doppler =      */ -3211.629,
                                    /* .CN0 =          */ 42.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { Code::R1C, 10 },
                                    /* .pseudorange =  */ Pseudorange{ .value = 21657654.213, .SSI = 1 },
                                    /* .carrierPhase = */ CarrierPhase{ .value = 115447492.344, .SSI = 2, .LLI = 0 },
                                    /* .doppler =      */ -4203.686,
                                    /* .CN0 =          */ 41.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { Code::R1C, 13 },
                                    /* .pseudorange =  */ Pseudorange{ .value = 21689411.389, .SSI = 4 },
                                    /* .carrierPhase = */ CarrierPhase{ .value = 115820280.545, .SSI = 5, .LLI = 0 },
                                    /* .doppler =      */ 3020.874,
                                    /* .CN0 =          */ 32.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { Code::R2C, 13 },
                                    /* .pseudorange =  */ Pseudorange{ .value = 21689421.083, .SSI = 4 },
                                    /* .carrierPhase = */ std::nullopt, // no carrierPhase, but pseudorange exists. This happens if the CN0 is so small that the PLL could not lock, even if the DLL has locked (= pseudorange available). The observation is still valid.
                                    /* .doppler =      */ 2349.882,
                                    /* .CN0 =          */ 26.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { Code::E1X, 3 },
                                    /* .pseudorange =  */ Pseudorange{ .value = 22094318.881, .SSI = 1 },
                                    /* .carrierPhase = */ CarrierPhase{ .value = 116106429.676, .SSI = 1, .LLI = 0 },
                                    /* .doppler =      */ -2446.293,
                                    /* .CN0 =          */ 46.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { Code::E7X, 3 },
                                    /* .pseudorange =  */ Pseudorange{ .value = 22094320.071, .SSI = 1 },
                                    /* .carrierPhase = */ CarrierPhase{ .value = 88964669.099, .SSI = 1, .LLI = 0 },
                                    /* .doppler =      */ -1874.433,
                                    /* .CN0 =          */ 50.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { Code::E1X, 5 },
                                    /* .pseudorange =  */ Pseudorange{ .value = 25830241.475, .SSI = 1 },
                                    /* .carrierPhase = */ CarrierPhase{ .value = 135738837.274, .SSI = 3, .LLI = 0 },
                                    /* .doppler =      */ -3677.055,
                                    /* .CN0 =          */ 37.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { Code::B7I, 7 },
                                    /* .pseudorange =  */ Pseudorange{ .value = 37737385.511, .SSI = 1 },
                                    /* .carrierPhase = */ CarrierPhase{ .value = 151952809.467, .SSI = 2, .LLI = 0 },
                                    /* .doppler =      */ -1170.329,
                                    /* .CN0 =          */ 40.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { Code::B2I, 10 },
                                    /* .pseudorange =  */ Pseudorange{ .value = 36759786.025, .SSI = 1 },
                                    /* .carrierPhase = */ CarrierPhase{ .value = 191417848.535, .SSI = 2, .LLI = 0 },
                                    /* .doppler =      */ -1192.425,
                                    /* .CN0 =          */ 42.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { Code::B7I, 10 },
                                    /* .pseudorange =  */ Pseudorange{ .value = 36759779.463, .SSI = 1 },
                                    /* .carrierPhase = */ CarrierPhase{ .value = 148016392.042, .SSI = 2, .LLI = 0 },
                                    /* .doppler =      */ -921.977,
                                    /* .CN0 =          */ 40.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { Code::B2I, 23 },
                                    /* .pseudorange =  */ Pseudorange{ .value = 19672914.865, .SSI = 1 },
                                    /* .carrierPhase = */ CarrierPhase{ .value = 102442028.933, .SSI = 1, .LLI = 0 },
                                    /* .doppler =      */ 96.813,
                                    /* .CN0 =          */ 50.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { Code::S1C, 23 },
                                    /* .pseudorange =  */ Pseudorange{ .value = 36325088.073, .SSI = 1 },
                                    /* .carrierPhase = */ CarrierPhase{ .value = 190889609.539, .SSI = 1, .LLI = 0 },
                                    /* .doppler =      */ -535.350,
                                    /* .CN0 =          */ 44.000 },
      },
      /* ._satData = */ std::vector<GnssObs::SatelliteData>{
          { SatId(GPS, 1), G01 | G02 },
          { SatId(GPS, 13), G01 },
          { SatId(GLO, 1), R01 | R02 },
          { SatId(GLO, 10), R01 },
          { SatId(GLO, 13), R01 | R02 },
          { SatId(GAL, 3), E01 | E07 },
          { SatId(GAL, 5), E01 },
          { SatId(BDS, 7), B07 },
          { SatId(BDS, 10), B02 | B07 },
          { SatId(BDS, 23), B02 },
          { SatId(SBAS, 23), S01 },
      } },
};

} // namespace NAV::TESTS::RinexObsFileTests::v3_03