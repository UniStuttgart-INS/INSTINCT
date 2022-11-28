// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file PosVelAttTests.cpp
/// @brief PosVelAtt NodeData related tests
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-03-13

#include <catch2/catch.hpp>
#include "EigenApprox.hpp"

#include "NodeData/State/PosVelAtt.hpp"
#include "fmt/core.h"
#include "Navigation/Transformations/Units.hpp"

#include "util/Logger.hpp"

namespace NAV::TEST::PosVelAttTests
{

TEST_CASE("[PosVelAtt] Position Functions", "[PosVelAtt]")
{
    Logger consoleSink;

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    Eigen::Vector3d lla_position{ deg2rad(48.78081), deg2rad(9.172012), 254 };
    Eigen::Vector3d e_position = Eigen::Vector3d(4157.128, 671.224, 4774.723) * 1000;

    Pos state;
    state.setPosition_e(e_position);

    CHECK(state.e_position() == e_position);
    CHECK(state.lla_position() == Eigen::Vector3d{ state.latitude(), state.longitude(), state.altitude() });
    CHECK(state.latitude() == Approx(lla_position(0)));
    CHECK(state.longitude() == Approx(lla_position(1)));
    CHECK(state.altitude() == Approx(lla_position(2)).margin(0.3));

    CHECK(state.e_Quat_n() == EigApproxQ(trafo::e_Quat_n(lla_position(0), lla_position(1))));
    CHECK(state.n_Quat_e() == state.e_Quat_n().conjugate());

    // ###########################################################################################################

    state.setPosition_lla(lla_position);

    CHECK(state.e_position() == EigApprox(e_position));
    CHECK(state.lla_position() == lla_position);
}

TEST_CASE("[PosVelAtt] Velocity Functions", "[PosVelAtt]")
{
    Logger consoleSink;

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    Eigen::Vector3d lla_position{ deg2rad(48.78081), deg2rad(9.172012), 254 };
    Eigen::Vector3d e_vel{ 30, -25.5, 4774.723 };
    Eigen::Vector3d n_vel = trafo::n_Quat_e(lla_position(0), lla_position(1)) * e_vel;

    PosVel state;
    state.setPosition_lla(lla_position);
    state.setVelocity_e(e_vel);

    CHECK(state.lla_position() == lla_position);
    CHECK(state.e_velocity() == e_vel);
    CHECK(state.n_velocity() == EigApprox(n_vel));
    CHECK(state.e_velocity().norm() == Approx(state.n_velocity().norm()));

    state.setVelocity_n(n_vel);
    CHECK(state.lla_position() == lla_position);
    CHECK(state.e_velocity() == EigApprox(e_vel));
    CHECK(state.n_velocity() == n_vel);
    CHECK(state.e_velocity().norm() == Approx(state.n_velocity().norm()));
}

TEST_CASE("[PosVelAtt] Attitude Functions", "[PosVelAtt]")
{
    Logger consoleSink;

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    Eigen::Vector3d lla_position{ deg2rad(48.78081), deg2rad(9.172012), 254 };
    Eigen::Vector3d e_vel{ 30, -25.5, 4774.723 };
    Eigen::Vector3d n_vel = trafo::n_Quat_e(lla_position(0), lla_position(1)) * e_vel;
    double roll = deg2rad(5);
    double pitch = deg2rad(-30);
    double yaw = deg2rad(66);

    PosVelAtt state;
    state.setState_e(trafo::lla2ecef_WGS84(lla_position), e_vel, trafo::e_Quat_n(lla_position(0), lla_position(1)) * trafo::n_Quat_b(roll, pitch, yaw));

    CHECK(state.lla_position() == EigApprox(lla_position));
    CHECK(state.e_velocity() == EigApprox(e_vel));
    CHECK(state.n_Quat_b() == EigApproxQ(trafo::n_Quat_b(roll, pitch, yaw)));
    CHECK(state.b_Quat_n() == EigApproxQ(trafo::b_Quat_n(roll, pitch, yaw)));
    CHECK(state.e_Quat_b() == EigApproxQ(trafo::e_Quat_n(lla_position(0), lla_position(1)) * trafo::n_Quat_b(roll, pitch, yaw)));
    CHECK(state.b_Quat_e() == EigApproxQ(trafo::b_Quat_n(roll, pitch, yaw) * trafo::n_Quat_e(lla_position(0), lla_position(1))));

    state.setState_n(lla_position, n_vel, trafo::n_Quat_b(roll, pitch, yaw));
    CHECK(state.n_velocity() == EigApprox(n_vel));
    CHECK(state.n_Quat_b() == EigApproxQ(trafo::n_Quat_b(roll, pitch, yaw)));
    CHECK(state.b_Quat_n() == EigApproxQ(trafo::b_Quat_n(roll, pitch, yaw)));
    CHECK(state.e_Quat_b() == EigApproxQ(trafo::e_Quat_n(lla_position(0), lla_position(1)) * trafo::n_Quat_b(roll, pitch, yaw)));
    CHECK(state.b_Quat_e() == EigApproxQ(trafo::b_Quat_n(roll, pitch, yaw) * trafo::n_Quat_e(lla_position(0), lla_position(1))));
}

TEST_CASE("[PosVelAtt] Attitude RollPitchYaw", "[PosVelAtt]")
{
    Logger consoleSink;

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    PosVelAtt state;
    state.setPosition_lla({ deg2rad(48.78081), deg2rad(9.172012), 254 });

    double delta = deg2rad(5);
    // (-pi:pi] x (-pi/2:pi/2] x (-pi:pi]
    for (double expectedRoll = -M_PI + delta; expectedRoll <= M_PI - delta + std::numeric_limits<float>::epsilon(); expectedRoll += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
    {
        for (double expectedPitch = -M_PI / 2.0 + delta; expectedPitch <= M_PI / 2.0 - delta + std::numeric_limits<float>::epsilon(); expectedPitch += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
        {
            for (double expectedYaw = -M_PI + delta; expectedYaw <= M_PI - delta + std::numeric_limits<float>::epsilon(); expectedYaw += delta) // NOLINT(clang-analyzer-security.FloatLoopCounter,cert-flp30-c)
            {
                state.setAttitude_n_Quat_b(trafo::n_Quat_b(expectedRoll, expectedPitch, expectedYaw));
                auto actualRollPitchYaw = rad2deg(state.rollPitchYaw());
                REQUIRE(rad2deg(Eigen::Vector3d{ expectedRoll, expectedPitch, expectedYaw }) == EigApprox(actualRollPitchYaw).margin(1e-8).epsilon(0));
            }
        }
    }
}

} // namespace NAV::TEST::PosVelAttTests
