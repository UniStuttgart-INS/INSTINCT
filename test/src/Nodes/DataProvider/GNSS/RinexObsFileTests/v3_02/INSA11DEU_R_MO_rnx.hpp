// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file INSA11DEU_R_MO_rnx.hpp
/// @brief Test data for the INSA11DEU_R_MO.rnx
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2023-01-12

#pragma once

#include "NodeData/GNSS/GnssObs.hpp"

namespace NAV::TESTS::RinexObsFileTests::v3_02
{
/// @brief Test Data for the file INSA11DEU_R_MO.rnx
const std::vector<GnssObs> gnssObs_INSA11DEU_R_MO_rnx = {
    { /* .insTime = */ InsTime{ 2022, 3, 1, 23, 0, 0.0000000, GPST }, // TODO: If 'RCV CLOCK OFFS APPL' == 1 (=yes), subtract 'recClkOffset' here (see Rinex v3.02, p. 25, A9 and A12)
      /* .data = */ std::vector<GnssObs::ObservationData>{
          GnssObs::ObservationData{ /* .satSigId =     */ { B02, 43 }, // careful here: In v3.02 'B02' (e.g. C2I) does not exist. However, v3.03 mentions that both, C1x and C2x should be accepted and treated as C2x --> B02
                                    /* .code =         */ Code::B2I,
                                    /* .pseudorange =  */ { .value = 22349867.305, .SSI = 8 },
                                    /* .carrierPhase = */ { .value = 116381665.390, .SSI = 8, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 48.900 },
          GnssObs::ObservationData{ /* .satSigId =     */ { B06, 43 },
                                    /* .code =         */ Code::B6I,
                                    /* .pseudorange =  */ { .value = 22349855.652, .SSI = 8 },
                                    /* .carrierPhase = */ { .value = 94569609.783, .SSI = 8, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 50.800 },
          GnssObs::ObservationData{ /* .satSigId =     */ { B02, 9 },
                                    /* .code =         */ Code::B2I,
                                    /* .pseudorange =  */ { .value = 38706674.828, .SSI = 7 },
                                    /* .carrierPhase = */ { .value = 201555842.344, .SSI = 7, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 42.300 },
          GnssObs::ObservationData{ /* .satSigId =     */ { B07, 9 },
                                    /* .code =         */ Code::B7I,
                                    /* .pseudorange =  */ { .value = 38706664.113, .SSI = 7 },
                                    /* .carrierPhase = */ { .value = 155855727.777, .SSI = 7, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 43.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { B06, 9 },
                                    /* .code =         */ Code::B6I,
                                    /* .pseudorange =  */ { .value = 38706659.184, .SSI = 6 },
                                    /* .carrierPhase = */ { .value = 163780574.733, .SSI = 6, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 41.700 },
          GnssObs::ObservationData{ /* .satSigId =     */ { R01, 5 },
                                    /* .code =         */ Code::R1C,
                                    /* .pseudorange =  */ { .value = 22420402.273, .SSI = 7 },
                                    /* .carrierPhase = */ { .value = 119849929.587, .SSI = 7, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 47.300 },
          GnssObs::ObservationData{ /* .satSigId =     */ { R02, 5 },
                                    /* .code =         */ Code::R2C,
                                    /* .pseudorange =  */ { .value = 22420404.453, .SSI = 7 },
                                    /* .carrierPhase = */ { .value = 93216638.634, .SSI = 7, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 42.400 },
          GnssObs::ObservationData{ /* .satSigId =     */ { G01, 1 },
                                    /* .code =         */ Code::G1C,
                                    /* .pseudorange =  */ { .value = 20046235.867, .SSI = 7 },
                                    /* .carrierPhase = */ { .value = 105343635.532, .SSI = 7, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 47.600 },
          GnssObs::ObservationData{ /* .satSigId =     */ { G02, 1 },
                                    /* .code =         */ Code::G2X,
                                    /* .pseudorange =  */ { .value = 20046242.508, .SSI = 8 },
                                    /* .carrierPhase = */ { .value = 82086056.934, .SSI = 8, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 50.800 },
          GnssObs::ObservationData{ /* .satSigId =     */ { G05, 1 },
                                    /* .code =         */ Code::G5X,
                                    /* .pseudorange =  */ { .value = 20046238.297, .SSI = 9 },
                                    /* .carrierPhase = */ { .value = 78665786.929, .SSI = 9, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 55.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { G01, 31 },
                                    /* .code =         */ Code::G1C,
                                    /* .pseudorange =  */ { .value = 24668780.836, .SSI = 5 },
                                    /* .carrierPhase = */ { .value = 129635384.615, .SSI = 5, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 35.700 },
          GnssObs::ObservationData{ /* .satSigId =     */ { G02, 31 },
                                    /* .code =         */ Code::G2X,
                                    /* .pseudorange =  */ { .value = 24668785.090, .SSI = 6 },
                                    /* .carrierPhase = */ { .value = 101014573.592, .SSI = 6, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 37.100 },
          GnssObs::ObservationData{ /* .satSigId =     */ { G01, 14 },
                                    /* .code =         */ Code::G1C,
                                    /* .pseudorange =  */ { .value = 24706684.281, .SSI = 6 },
                                    /* .carrierPhase = */ { .value = 129834420.530, .SSI = 6, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 38.100 },
          GnssObs::ObservationData{ /* .satSigId =     */ { G01, 14 },
                                    /* .code =         */ Code::G1X,
                                    /* .pseudorange =  */ { .value = 24706681.297, .SSI = 6 },
                                    /* .carrierPhase = */ { .value = 129834513.541, .SSI = 6, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 39.800 },
          GnssObs::ObservationData{ /* .satSigId =     */ { G02, 14 },
                                    /* .code =         */ Code::G2X,
                                    /* .pseudorange =  */ { .value = 24706686.559, .SSI = 6 },
                                    /* .carrierPhase = */ { .value = 101169767.146, .SSI = 6, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 39.100 },
          GnssObs::ObservationData{ /* .satSigId =     */ { G05, 14 },
                                    /* .code =         */ Code::G5X,
                                    /* .pseudorange =  */ { .value = 24706689.195, .SSI = 7 },
                                    /* .carrierPhase = */ { .value = 96954357.058, .SSI = 7, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 43.400 },
          GnssObs::ObservationData{ /* .satSigId =     */ { G01, 19 },
                                    /* .code =         */ Code::G1C,
                                    /* .pseudorange =  */ { .value = 24299006.547, .SSI = 6 },
                                    /* .carrierPhase = */ { .value = 127692003.035, .SSI = 6, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 39.500 },
          GnssObs::ObservationData{ /* .satSigId =     */ { G02, 19 },
                                    /* .code =         */ Code::G2W,
                                    /* .pseudorange =  */ { .value = 24299008.266, .SSI = 4 },
                                    /* .carrierPhase = */ { .value = 99500376.608, .SSI = 4, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 24.000 },
          GnssObs::ObservationData{ /* .satSigId =     */ { G01, 17 },
                                    /* .code =         */ Code::G1C,
                                    /* .pseudorange =  */ { .value = 22919820.070, .SSI = 7 },
                                    /* .carrierPhase = */ { .value = 120444400.361, .SSI = 7, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 45.100 },
          GnssObs::ObservationData{ /* .satSigId =     */ { G02, 17 },
                                    /* .code =         */ Code::G2X,
                                    /* .pseudorange =  */ { .value = 22919824.242, .SSI = 7 },
                                    /* .carrierPhase = */ { .value = 93852872.558, .SSI = 7, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 43.800 },
          GnssObs::ObservationData{ /* .satSigId =     */ { R01, 6 },
                                    /* .code =         */ Code::R1C,
                                    /* .pseudorange =  */ { .value = 19287914.984, .SSI = 6 },
                                    /* .carrierPhase = */ { .value = 102924048.466, .SSI = 6, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 39.200 },
          GnssObs::ObservationData{ /* .satSigId =     */ { R01, 21 },
                                    /* .code =         */ Code::R1C,
                                    /* .pseudorange =  */ { .value = 19448257.852, .SSI = 8 },
                                    /* .carrierPhase = */ { .value = 104071566.414, .SSI = 8, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 50.800 },
          GnssObs::ObservationData{ /* .satSigId =     */ { R02, 21 },
                                    /* .code =         */ Code::R2C,
                                    /* .pseudorange =  */ { .value = 19448258.457, .SSI = 8 },
                                    /* .carrierPhase = */ { .value = 80944613.870, .SSI = 8, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 48.400 },
          GnssObs::ObservationData{ /* .satSigId =     */ { R03, 21 },
                                    /* .code =         */ Code::R3X,
                                    /* .pseudorange =  */ { .value = 19448249.238, .SSI = 8 },
                                    /* .carrierPhase = */ { .value = 77978305.818, .SSI = 8, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 50.500 },
          GnssObs::ObservationData{ /* .satSigId =     */ { E01, 33 },
                                    /* .code =         */ Code::E1X,
                                    /* .pseudorange =  */ { .value = 23604423.273, .SSI = 7 },
                                    /* .carrierPhase = */ { .value = 124042147.331, .SSI = 7, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 47.800 },
          GnssObs::ObservationData{ /* .satSigId =     */ { E05, 33 },
                                    /* .code =         */ Code::E5X,
                                    /* .pseudorange =  */ { .value = 23604423.359, .SSI = 8 },
                                    /* .carrierPhase = */ { .value = 92628911.505, .SSI = 8, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 52.300 },
          GnssObs::ObservationData{ /* .satSigId =     */ { E07, 33 },
                                    /* .code =         */ Code::E7X,
                                    /* .pseudorange =  */ { .value = 23604420.621, .SSI = 8 },
                                    /* .carrierPhase = */ { .value = 95045301.171, .SSI = 8, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 52.200 },
          GnssObs::ObservationData{ /* .satSigId =     */ { E08, 33 },
                                    /* .code =         */ Code::E8X,
                                    /* .pseudorange =  */ { .value = 23604423.922, .SSI = 9 },
                                    /* .carrierPhase = */ { .value = 93837118.636, .SSI = 9, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 55.500 },
          GnssObs::ObservationData{ /* .satSigId =     */ { E06, 33 },
                                    /* .code =         */ Code::E6X,
                                    /* .pseudorange =  */ { .value = 23604418.109, .SSI = 9 },
                                    /* .carrierPhase = */ { .value = 100683563.381, .SSI = 9, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 54.100 },
          GnssObs::ObservationData{ /* .satSigId =     */ { S01, 23 },
                                    /* .code =         */ Code::S1C,
                                    /* .pseudorange =  */ { .value = 38737434.219, .SSI = 7 },
                                    /* .carrierPhase = */ { .value = 203567072.311, .SSI = 7, .LLI = 0 },
                                    /* .doppler =      */ 0.0, // observations contain no doppler
                                    /* .CN0 =          */ 44.800 },

      } },
};

} // namespace NAV::TESTS::RinexObsFileTests::v3_02