// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SaastamoinenTests.cpp
/// @brief Saastamoinen Tests
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-10-17

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"
#include "Logger.hpp"

#include "Navigation/Atmosphere/Troposphere/Models/Saastamoinen.hpp"
#include "Navigation/Atmosphere/Troposphere/MappingFunctions/Cosecant.hpp"
#include "Navigation/Atmosphere/Pressure/Pressure.hpp"
#include "Navigation/Atmosphere/Temperature/Temperature.hpp"
#include "Navigation/Atmosphere/WaterVapor/WaterVapor.hpp"
#include "Navigation/Transformations/Units.hpp"
#include <catch2/matchers/catch_matchers.hpp>

namespace NAV::TESTS::Saastamoinen
{
namespace
{
/* troposphere model -----------------------------------------------------------
 * compute tropospheric delay by standard atmosphere and saastamoinen model
 * args   : gtime_t time     I   time
 *          double *pos      I   receiver position {lat,lon,h} (rad,m)
 *          double *azel     I   azimuth/elevation angle {az,el} (rad)
 *          double humi      I   relative humidity
 * return : tropospheric delay (m)
 *-----------------------------------------------------------------------------*/
double tropmodel(const double* pos, const double* azel, double humi)
{
    const double temp0 = 15.0;                /* temperature at sea level */
    double hgt, pres, temp, e, z, trph, trpw; // NOLINT

    if (pos[2] < -100.0 || 1E4 < pos[2] || azel[1] <= 0) { return 0.0; };

    /* standard atmosphere */
    hgt = pos[2] < 0.0 ? 0.0 : pos[2];

    pres = 1013.25 * pow(1.0 - 2.2557E-5 * hgt, 5.2568);
    temp = temp0 - 6.5E-3 * hgt + 273.16;
    e = 6.108 * humi * exp((17.15 * temp - 4684.0) / (temp - 38.45));

    /* saastamoinen model */
    z = M_PI / 2.0 - azel[1];
    trph = 0.0022768 * pres / (1.0 - 0.00266 * cos(2.0 * pos[0]) - 0.00028 * hgt / 1E3) / cos(z);
    trpw = 0.002277 * (1255.0 / temp + 0.05) * e / cos(z);
    return trph + trpw;
}
} // namespace

TEST_CASE("[Saastamoinen] Compare with Rtklib code", "[Saastamoinen]")
{
    auto logger = initializeTestLogger();

    Eigen::Vector3d lla_pos(deg2rad(30.0), deg2rad(0.0), 100.0);
    double elevation = deg2rad(30.0);
    double humidity = 0.7;

    double p = calcTotalPressure(lla_pos(2), PressureModel::ISA);
    double T = calcAbsoluteTemperature(lla_pos(2), TemperatureModel::ISA);
    double e = calcWaterVaporPartialPressure(T, humidity, WaterVaporModel::ISA);

    double ZHD = calcZHD_Saastamoinen(lla_pos, p);
    double ZWD = calcZWD_Saastamoinen(T, e);
    double mf = calcTropoMapFunc_cosecant(elevation);
    double tropo_delay = ZHD * mf + ZWD * mf;

    std::array<double, 2> azel{ 0.0, elevation };
    double rtklib_tropo_delay = tropmodel(lla_pos.data(), azel.data(), humidity);

    REQUIRE_THAT(tropo_delay, Catch::Matchers::WithinAbs(rtklib_tropo_delay, 3e-4));
    // Accuracy limited as Rtklib converts temperature with 273.16 from °C to Kelvin
}

} // namespace NAV::TESTS::Saastamoinen