// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GPT2Coeffs.hpp
/// @brief GPT2 Coefficients
/// @author Rui Wang (rui.wang@ins.uni-stuttgart.de)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-02-21
/// @note See https://vmf.geo.tuwien.ac.at/codes/ for data sources.

#pragma once

#include <array>

namespace NAV::internal
{

/// @brief Parameters of the GP2 grid in in 1 degree x 1 degree resolution
struct GPT2Data
{
    double lat{};        ///< latitude
    double lon{};        ///< longitude
    double p_A0{};       ///< pressure: mean value
    double p_A1{};       ///< pressure: cosine amplitude for the annual variation
    double p_B1{};       ///< pressure: sine amplitude for the annual variation
    double p_A2{};       ///< pressure: cosine amplitude for the semi-annual variation
    double p_B2{};       ///< pressure: sine amplitude for the semi-annual variation
    double T_A0{};       ///< temperature: mean value
    double T_A1{};       ///< temperature: cosine amplitude for the annual variation
    double T_B1{};       ///< temperature: sine amplitude for the annual variation
    double T_A2{};       ///< temperature: cosine amplitude for the semi-annual variation
    double T_B2{};       ///< temperature: sine amplitude for the semi-annual variation
    double Q_A0{};       ///< specific humidity: mean value
    double Q_A1{};       ///< specific humidity: cosine amplitude for the annual variation
    double Q_B1{};       ///< specific humidity: sine amplitude for the annual variation
    double Q_A2{};       ///< specific humidity: cosine amplitude for the semi-annual variation
    double Q_B2{};       ///< specific humidity: sine amplitude for the semi-annual variation
    double dT_A0{};      ///< temperature lapse rate: mean value
    double dT_A1{};      ///< temperature lapse rate: cosine amplitude for the annual variation
    double dT_B1{};      ///< temperature lapse rate: sine amplitude for the annual variation
    double dT_A2{};      ///< temperature lapse rate: cosine amplitude for the semi-annual variation
    double dT_B2{};      ///< temperature lapse rate: sine amplitude for the semi-annual variation
    double undulation{}; ///< geoid undulation in m
    double Hs{};         ///< orthometric grid height in m
    double h_A0{};       ///< hydrostatic mapping function coefficient: mean value
    double h_A1{};       ///< hydrostatic mapping function coefficient: cosine amplitude for the annual variation
    double h_B1{};       ///< hydrostatic mapping function coefficient: sine amplitude for the annual variation
    double h_A2{};       ///< hydrostatic mapping function coefficient: cosine amplitude for the semi-annual variation
    double h_B2{};       ///< hydrostatic mapping function coefficient: sine amplitude for the semi-annual variation
    double w_A0{};       ///< wet mapping function coefficient: mean value
    double w_A1{};       ///< wet mapping function coefficient: cosine amplitude for the annual variation
    double w_B1{};       ///< wet mapping function coefficient: sine amplitude for the annual variation
    double w_A2{};       ///< wet mapping function coefficient: cosine amplitude for the semi-annual variation
    double w_B2{};       ///< wet mapping function coefficient: sine amplitude for the semi-annual variation
    double la_A0{};      ///< water vapour decrease factor: mean value
    double la_A1{};      ///< water vapour decrease factor: cosine amplitude for the annual variation
    double la_B1{};      ///< water vapour decrease factor: sine amplitude for the annual variation
    double la_A2{};      ///< water vapour decrease factor: cosine amplitude for the semi-annual variation
    double la_B2{};      ///< water vapour decrease factor: sine amplitude for the semi-annual variation
    double Tm_A0{};      ///< weighted mean temperature: mean value
    double Tm_A1{};      ///< weighted mean temperature: cosine amplitude for the annual variation
    double Tm_B1{};      ///< weighted mean temperature: sine amplitude for the annual variation
    double Tm_A2{};      ///< weighted mean temperature: cosine amplitude for the semi-annual variation
    double Tm_B2{};      ///< weighted mean temperature: sine amplitude for the semi-annual variation
};

/// @brief GPT2 grid
extern std::array<GPT2Data, 64800> GPT2_grid;

} // namespace NAV::internal
