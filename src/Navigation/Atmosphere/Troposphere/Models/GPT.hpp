// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GPT.hpp
/// @brief GPT2/3 (Global Pressure and Temperature) models
/// @author Rui Wang (rui.wang@ins.uni-stuttgart.de)
/// @date 2020-11-25
/// @note See https://vmf.geo.tuwien.ac.at/codes/ for code sources in matlab.

#pragma once
#include "Eigen/Dense"
#include "internal/GPT2Coeffs.hpp"
#include "internal/GPT3Coeffs.hpp"

namespace NAV
{
/// GPT2/3 output parameters
struct GPToutput
{
    double p{};    ///< p pressure in hPa (vector of length nstat)
    double T{};    ///< temperature in degrees Celsius (vector of length nstat)
    double dT{};   ///< temperature lapse rate in degrees per km (vector of length nstat)
    double Tm{};   ///< mean temperature of the water vapor in degrees Kelvin (vector of length nstat)
    double e{};    ///< water vapor pressure in hPa (vector of length nstat)
    double ah{};   ///< hydrostatic mapping function coefficient at zero height (VMF1) (vector of length nstat)
    double aw{};   ///< wet mapping function coefficient (VMF1) (vector of length nstat)
    double la{};   ///< water vapor decrease factor (vector of length nstat)
    double undu{}; ///< geoid undulation in m (vector of length nstat)
    // more outputs for GPT3
    double Gn_h{}; ///< hydrostatic north gradient in m
    double Ge_h{}; ///< hydrostatic east gradient in m
    double Gn_w{}; ///< wet north gradient in m
    double Ge_w{}; ///< wet east gradient in m
};

/// @brief Determine pressure, temperature, temperature lapse rate, mean temperature of the water vapor,
///        water vapor pressure, hydrostatic and wet mapping function coefficients ah and aw, water vapour decrease
///        factor and geoid undulation for specific sites near the Earth surface, based on a 1 x 1 degree GPT2 grid
/// @param[in] mjd modified Julian date
/// @param[in] lla_pos [𝜙, λ, h]^T Geodetic latitude, longitude and height in [rad, rad, m]
/// @param[in] GPT2_grid GPT2 grid in 1 degree x 1 degree resolution
/// @param[in, out] gpt2outputs GPT2 outputs
/// @note See \cite bohm2015development
void GPT2_param(const double& mjd, const Eigen::Vector3d& lla_pos, const std::array<internal::GPT2Data, 64800>& GPT2_grid,
                GPToutput& gpt2outputs);

/// @brief Determine pressure, temperature, temperature lapse rate,
///        mean temperature of the water vapor, water vapour pressure, hydrostatic
///        and wet mapping function coefficients ah and aw, water vapour decrease
///        factor, geoid undulation and empirical tropospheric gradients for
///        specific sites near the earth's surface, based on a 1 x 1 degree GPT3 grid
/// @param[in] mjd modified Julian date
/// @param[in] lla_pos [𝜙, λ, h]^T Geodetic latitude, longitude and height in [rad, rad, m]
/// @param[in] GPT3_grid GPT3 grid in 1 degree x 1 degree resolution
/// @param[in, out] gpt3outputs GPT3 outputs
/// @note See \cite Landskron2018
void GPT3_param(const double& mjd, const Eigen::Vector3d& lla_pos, const std::array<internal::GPT3Data, 64800>& GPT3_grid,
                GPToutput& gpt3outputs);

/// @brief To calculate the day of year
/// @param[in] mjd the Modified Julien Date
/// @return the day of year
double mjd2doy(const double& mjd);

/// @brief Change the sign of x according to the value of y
/// @param[in] x input value
/// @param[in] y input value
/// @return -x or +x
double sign(const double& x, const double& y);

static constexpr double gm = 9.80665;     ///< mean gravity in m/s**2
static constexpr double dMtr = 28.965e-3; ///< molar mass of dry air in kg/mol
static constexpr double Rg = 8.3143;      ///< universal gas constant in J/K/mol

/// @brief This subroutine determines the zenith wet delay based on the
///        equation 22 by Aske and Nordius (1987)
/// @param[in] e water vapor pressure in hPa
/// @param[in] Tm mean temperature in Kelvin
/// @param[in] la water vapor lapse rate (see definition in Askne and Nordius 1987)
/// @return zwd: zenith wet delay in meter
/// @note See \cite askne1987estimation
double asknewet(const double& e, const double& Tm, const double& la);

} // namespace NAV