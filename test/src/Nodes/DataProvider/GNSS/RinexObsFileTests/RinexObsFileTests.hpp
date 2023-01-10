// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RinexObsFileTests.hpp
/// @brief Data definitions for the RinexObsFileTests
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2022-11-15

#pragma once

namespace NAV::TESTS::RinexObsFileTests
{

constexpr double Gps_LeapSec = 18;

enum RinexTimeRef : size_t
{
    RINEX_Year,
    RINEX_Month,
    RINEX_Day,
    RINEX_Hour,
    RINEX_Minute,
    RINEX_Second,
    RINEX_EpochFlag,
    RINEX_NumSats,
    RINEX_reserved,     // a one-digit flag that is not specified in the standard
    RINEX_RcvClkOffset, // optional
};

enum RinexRef : size_t
{
    RINEX_SatNum,
    RINEX_Obs_Pseudorange,
    RINEX_SSI_Pseudorange,
    RINEX_Obs_CarrierPhase,
    RINEX_SSI_CarrierPhase,
    RINEX_Obs_Doppler,
    RINEX_Obs_SigStrength,
};

} // namespace NAV::TESTS::RinexObsFileTests