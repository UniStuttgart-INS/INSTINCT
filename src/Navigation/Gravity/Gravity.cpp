// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Gravity.hpp"

#include <cmath>
#include <optional>
#include <string>
#include <fstream>
#include <streambuf>

#include <Eigen/Core>
#include <imgui.h>

#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "util/Logger.hpp"
#include "Navigation/Constants.hpp"
#include "internal/AssociatedLegendre.hpp"
#include "internal/egm96Coeffs.hpp"

namespace NAV
{

const char* to_string(GravitationModel gravitationModel)
{
    switch (gravitationModel)
    {
    case GravitationModel::None:
        return "None";
    case GravitationModel::WGS84:
        return "WGS84";
    case GravitationModel::WGS84_Skydel:
        return "WGS84 (Skydel Constants)";
    case GravitationModel::Somigliana:
        return "Somigliana";
    case GravitationModel::EGM96:
        return "EGM96";
    case GravitationModel::COUNT:
        return "";
    }
    return "";
}

bool ComboGravitationModel(const char* label, GravitationModel& gravitationModel)
{
    bool clicked = false;
    if (ImGui::BeginCombo(label, NAV::to_string(gravitationModel)))
    {
        for (size_t i = 0; i < static_cast<size_t>(GravitationModel::COUNT); i++)
        {
            const bool is_selected = (static_cast<size_t>(gravitationModel) == i);
            if (ImGui::Selectable(NAV::to_string(static_cast<GravitationModel>(i)), is_selected))
            {
                gravitationModel = static_cast<GravitationModel>(i);
                clicked = true;
            }

            // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
            if (is_selected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }

        ImGui::EndCombo();
    }
    return clicked;
}

Eigen::Vector3d n_calcGravitation(const Eigen::Vector3d& lla_position, GravitationModel gravitationModel)
{
    const double& latitude = lla_position(0);
    const double& altitude = lla_position(2);

    if (gravitationModel == GravitationModel::WGS84)
    {
        return n_calcGravitation_WGS84(latitude, altitude);
    }
    if (gravitationModel == GravitationModel::WGS84_Skydel) // TODO: This function becomes obsolete, once the ImuStream is deactivated due to the 'InstinctDataStream'
    {
        return n_calcGravitation_WGS84_Skydel(latitude, altitude);
    }
    if (gravitationModel == GravitationModel::Somigliana)
    {
        return n_calcGravitation_SomiglianaAltitude(latitude, altitude);
    }
    if (gravitationModel == GravitationModel::EGM96)
    {
        return n_calcGravitation_EGM96(lla_position);
    }
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d n_calcGravitation_SomiglianaAltitude(const double& latitude, const double& altitude)
{
    // eq 6.16 has a fault in the denominator, it should be a sin^2(latitude)
    double g_0 = 9.7803253359 * (1.0 + 1.931853e-3 * std::pow(std::sin(latitude), 2))
                 / std::sqrt(1.0 - InsConst::WGS84::e_squared * std::pow(std::sin(latitude), 2));

    // Altitude compensation (Matlab example from Chapter 6_GNSS_INS_1 - glocal.m)
    double k = 1
               - (2 * altitude / InsConst::WGS84::a)
                     * (1 + InsConst::WGS84::f
                        + (std::pow(InsConst::omega_ie * InsConst::WGS84::a, 2))
                              * (InsConst::WGS84::b / InsConst::WGS84::MU))
               + 3 * std::pow(altitude / InsConst::WGS84::a, 2);

    return { 0.0, 0.0, k * g_0 };
}

Eigen::Vector3d n_calcGravitation_WGS84_Skydel(const double& latitude, const double& altitude)
{
    // geocentric latitude determination from geographic latitude
    double latitudeGeocentric = std::atan((std::pow(InsConst::WGS84::b, 2.0) / std::pow(InsConst::WGS84::a, 2.0)) * std::tan(latitude));
    // effective radius determination, i.e. earth radius on WGS84 spheroid plus local altitude --> possible error!! altitude in lla should be added rather than subtracted!
    double radiusSpheroid = InsConst::WGS84::a * (1.0 - InsConst::WGS84::f * std::pow(std::sin(latitudeGeocentric), 2.0)) - altitude;

    // Derivation of gravity, i.e. gravitational potential derived after effective radius
    double gravitationMagnitude = InsConst::WGS84::MU * std::pow(radiusSpheroid, -2.0)
                                  - 3 * InsConst::WGS84::MU * InsConst::WGS84::J2 * std::pow(InsConst::WGS84::a, 2.0) * 0.5 * std::pow(radiusSpheroid, -4.0) * (3 * std::pow(std::sin(latitudeGeocentric), 2.0) - 1)
                                  - std::pow(InsConst::omega_ie_Skydel, 2.0) * radiusSpheroid * std::pow(std::cos(latitudeGeocentric), 2.0);

    return { 0, 0, gravitationMagnitude };
}

Eigen::Vector3d n_calcGravitation_WGS84(const double& latitude, const double& altitude)
{
    // Geocentric latitude determination from geographic latitude
    double latitudeGeocentric = std::atan((std::pow(InsConst::WGS84::b, 2.0) / std::pow(InsConst::WGS84::a, 2.0)) * std::tan(latitude));
    // Radius of spheroid determination
    double radiusSpheroid = InsConst::WGS84::a * (1.0 - InsConst::WGS84::f * std::pow(std::sin(latitudeGeocentric), 2.0)) + altitude;

    // Magnitude of the gravity, i.e. without orientation
    double gravitationMagnitude = InsConst::WGS84::MU * std::pow(radiusSpheroid, -2.0)
                                  - 3 * InsConst::WGS84::MU * InsConst::WGS84::J2 * std::pow(InsConst::WGS84::a, 2.0) * 0.5 * std::pow(radiusSpheroid, -4.0) * (3 * std::pow(std::sin(latitudeGeocentric), 2.0) - 1)
                                  - std::pow(InsConst::omega_ie, 2.0) * radiusSpheroid * std::pow(std::cos(latitudeGeocentric), 2.0);

    return { 0, 0, gravitationMagnitude };
}

Eigen::Vector3d n_calcGravitation_EGM96(const Eigen::Vector3d& lla_position, size_t ndegree)
{
    using internal::egm96Coeffs;
    using internal::associatedLegendre;

    Eigen::Vector3d e_position = trafo::lla2ecef_WGS84(lla_position);

    // Geocentric latitude determination from Groves (2013) - eq. (2.114)
    double latitudeGeocentric = std::atan(e_position(2) / std::sqrt(e_position(0) * e_position(0) + e_position(1) * e_position(1)));

    // Spherical coordinates
    double radius = std::sqrt(e_position(0) * e_position(0) + e_position(1) * e_position(1) + e_position(2) * e_position(2));
    double elevation = M_PI_2 - latitudeGeocentric; // [rad]
    double azimuth = lla_position(1);               // [rad]

    // Gravitation vector in local-navigation frame coordinates in [m/s^2]
    Eigen::Vector3d n_gravitation = Eigen::Vector3d::Zero();

    double Pnm = 0;
    double Pnmd = 0;

    auto coeffsRows = egm96Coeffs.size();

    // Associated Legendre Polynomial Coefficients 'P' and their derivatives 'Pd'
    auto [P, Pd] = associatedLegendre(elevation, ndegree);

    for (size_t i = 0; i < coeffsRows; i++) // NOLINT(clang-analyzer-core.UndefinedBinaryOperatorResult) // FIXME: Wrong error message about Eigen (error: The left operand of '*' is a garbage value)
    {
        // Retrieving EGM96 coefficients
        auto n = static_cast<int>(egm96Coeffs.at(i).at(0)); // Degree of the Associated Legendre Polynomial
        auto m = static_cast<int>(egm96Coeffs.at(i).at(1)); // Order of the Associated Legendre Polynomial
        auto C = egm96Coeffs.at(i).at(2);
        auto S = egm96Coeffs.at(i).at(3);

        if (static_cast<size_t>(n) == ndegree + 1)
        {
            // Ending of the for-loop once the iterated 'n' becomes larger than the user-defined 'ndegree'
            i = coeffsRows;
        }
        else
        {
            // Retrieving the parameters of the associated Legendre Polynomials
            Pnm = P(n, m);
            Pnmd = Pd(n, m);

            auto nd = static_cast<double>(n);
            auto md = static_cast<double>(m);

            // Gravity vector from differentiation of the gravity potential in spherical coordinates (see 'GUT User Guide' eq. 7.4.2)
            n_gravitation(0) += std::pow((InsConst::WGS84::a / radius), nd) * (C * std::cos(md * azimuth) + S * std::sin(md * azimuth)) * Pnmd;
            n_gravitation(1) += std::pow((InsConst::WGS84::a / radius), nd) * md * (C * std::sin(md * azimuth) - S * std::cos(md * azimuth)) * Pnm;
            n_gravitation(2) += (nd + 1.0) * std::pow((InsConst::WGS84::a / radius), nd) * (C * std::cos(md * azimuth) + S * std::sin(md * azimuth)) * Pnm;
        }
    }

    return { -InsConst::WGS84::MU / (radius * radius) * n_gravitation(0),
             (1.0 / std::sin(elevation)) * (-InsConst::WGS84::MU / (radius * radius)) * n_gravitation(1),
             InsConst::WGS84::MU / (radius * radius) * (1.0 + n_gravitation(2)) };
}

} // namespace NAV