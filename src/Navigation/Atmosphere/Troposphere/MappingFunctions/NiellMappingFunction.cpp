// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file NiellMappingFunction.cpp
/// @brief Niell Mapping Function (NMF)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-10-17
/// @note See https://gssc.esa.int/navipedia/index.php/Mapping_of_Niell
/// @note See \cite Niell1996 Niell1996

#include "NiellMappingFunction.hpp"
#include "Navigation/Time/InsTime.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "Navigation/Math/Math.hpp"
#include "util/Logger.hpp"

namespace NAV::internal::NMF
{

constexpr std::array<double, 5> latitude{ deg2rad(15.0), deg2rad(30.0), deg2rad(45.0), deg2rad(60.0), deg2rad(75.0) };
constexpr double a_ht = 2.53e-5;
constexpr double b_ht = 5.49e-3;
constexpr double c_ht = 1.14e-3;

namespace
{

std::array<double, 3> interpolateAverage(double lat)
{
    std::array<double, 5> a{ 1.2769934e-3, 1.2683230e-3, 1.2465397e-3, 1.2196049e-3, 1.2045996e-3 };
    std::array<double, 5> b{ 2.9153695e-3, 2.9152299e-3, 2.9288445e-3, 2.9022565e-3, 2.9024912e-3 };
    std::array<double, 5> c{ 62.610505e-3, 62.837393e-3, 63.721774e-3, 63.824265e-3, 64.258455e-3 };

    auto ls = math::lerpSearch(latitude, lat);
    ls.t = std::clamp(ls.t, 0.0, 1.0);

    return { std::lerp(a.at(ls.l), a.at(ls.u), ls.t),
             std::lerp(b.at(ls.l), b.at(ls.u), ls.t),
             std::lerp(c.at(ls.l), c.at(ls.u), ls.t) };
}

std::array<double, 3> interpolateAmplitude(double lat)
{
    std::array<double, 5> a{ 0.0000000e-0, 1.2709626e-5, 2.6523662e-5, 3.4000452e-5, 4.1202191e-5 };
    std::array<double, 5> b{ 0.0000000e-0, 2.1414979e-5, 3.0160779e-5, 7.2562722e-5, 11.723375e-5 };
    std::array<double, 5> c{ 0.0000000e-0, 9.0128400e-5, 4.3497037e-5, 84.795348e-5, 170.37206e-5 };

    auto ls = math::lerpSearch(latitude, lat);
    ls.t = std::clamp(ls.t, 0.0, 1.0);

    return { std::lerp(a.at(ls.l), a.at(ls.u), ls.t),
             std::lerp(b.at(ls.l), b.at(ls.u), ls.t),
             std::lerp(c.at(ls.l), c.at(ls.u), ls.t) };
}

std::array<double, 3> interpolateWet(double lat)
{
    std::array<double, 5> a{ 5.8021897e-4, 5.6794847e-4, 5.8118019e-4, 5.9727542e-4, 6.1641693e-4 };
    std::array<double, 5> b{ 1.4275268e-3, 1.5138625e-3, 1.4572752e-3, 1.5007428e-3, 1.7599082e-3 };
    std::array<double, 5> c{ 4.3472961e-2, 4.6729510e-2, 4.3908931e-2, 4.4626982e-2, 5.4736038e-2 };

    auto ls = math::lerpSearch(latitude, lat);
    ls.t = std::clamp(ls.t, 0.0, 1.0);

    return { std::lerp(a.at(ls.l), a.at(ls.u), ls.t),
             std::lerp(b.at(ls.l), b.at(ls.u), ls.t),
             std::lerp(c.at(ls.l), c.at(ls.u), ls.t) };
}

// mapping normalised to unity at zenith [Marini, 1972]
double mappingNormalizedAtZenith(double elevation, double a, double b, double c)
{
    auto sinE = std::sin(elevation);
    return (1.0 + a / (1.0 + b / (1.0 + c)))
           / (sinE + a / (sinE + b / (sinE + c)));
}

} // namespace

} // namespace NAV::internal::NMF

double NAV::calcTropoMapFunc_NMFH(const InsTime& epoch, const Eigen::Vector3d& lla_pos, double elevation)
{
    using namespace internal::NMF; // NOLINT(google-build-using-namespace)

    auto yDoySod = epoch.toYDoySod();
    double doy = yDoySod.doy + static_cast<double>(yDoySod.sod) / InsTimeUtil::SECONDS_PER_DAY;

    // reference day is 28 January
    double year = (doy - 28.0) / 365.25 + (lla_pos(0) < 0.0 ? 0.5 : 0.0); // Add half a year for southern latitudes
    double cosY = std::cos(2 * M_PI * year);

    double lat = std::abs(lla_pos(0));

    auto avg = interpolateAverage(lat);
    auto amp = interpolateAmplitude(lat);
    double a_d = avg[0] - amp[0] * cosY;
    double b_d = avg[1] - amp[1] * cosY;
    double c_d = avg[2] - amp[2] * cosY;

    double dm = (1.0 / std::sin(elevation) - mappingNormalizedAtZenith(elevation, a_ht, b_ht, c_ht)) * lla_pos(2) / 1e3;
    // ellipsoidal height instead of height above sea level
    return mappingNormalizedAtZenith(elevation, a_d, b_d, c_d) + dm;
}

double NAV::calcTropoMapFunc_NMFW(const Eigen::Vector3d& lla_pos, double elevation)
{
    using namespace internal::NMF; // NOLINT(google-build-using-namespace)

    auto [a_w, b_w, c_w] = interpolateWet(std::abs(lla_pos(0)));
    return mappingNormalizedAtZenith(elevation, a_w, b_w, c_w);
}