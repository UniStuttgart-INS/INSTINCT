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

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"

#include "NodeData/State/PosVelAtt.hpp"
#include "fmt/core.h"
#include "Navigation/Transformations/Units.hpp"

#include "Logger.hpp"

namespace NAV::TESTS::PosVelAttTests
{

TEST_CASE("[PosVelAtt] Position Functions", "[PosVelAtt]")
{
    auto logger = initializeTestLogger();

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    Eigen::Vector3d lla_position{ deg2rad(48.78081), deg2rad(9.172012), 254 };
    Eigen::Vector3d e_position = Eigen::Vector3d(4157.128, 671.224, 4774.723) * 1000;

    Pos state;
    state.setPosition_e(e_position);

    CHECK(state.e_position() == e_position);
    CHECK(state.lla_position() == Eigen::Vector3d(state.latitude(), state.longitude(), state.altitude()));
    CHECK_THAT(state.latitude() - lla_position(0), Catch::Matchers::WithinAbs(0, 9e-9));
    CHECK_THAT(state.longitude() - lla_position(1), Catch::Matchers::WithinAbs(0, 6e-9));
    CHECK_THAT(state.altitude() - lla_position(2), Catch::Matchers::WithinAbs(0, 0.3));

    CHECK_THAT(state.e_Quat_n(), Catch::Matchers::WithinAbs(trafo::e_Quat_n(lla_position(0), lla_position(1)), 8e-9));
    CHECK(state.n_Quat_e() == state.e_Quat_n().conjugate());

    // ###########################################################################################################

    state.setPosition_lla(lla_position);

    CHECK_THAT(state.e_position() - e_position, Catch::Matchers::WithinAbs(Eigen::Vector3d::Zero(), 0.3));
    CHECK(state.lla_position() == lla_position);
}

TEST_CASE("[PosVelAtt] Velocity Functions", "[PosVelAtt]")
{
    auto logger = initializeTestLogger();

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
    CHECK_THAT(state.n_velocity() - n_vel, Catch::Matchers::WithinAbs(Eigen::Vector3d::Zero(), 2.0e-12));
    CHECK_THAT(state.e_velocity().norm() - state.n_velocity().norm(), Catch::Matchers::WithinAbs(0, 1e-11));

    state.setVelocity_n(n_vel);
    CHECK(state.lla_position() == lla_position);
    CHECK_THAT(state.e_velocity() - e_vel, Catch::Matchers::WithinAbs(Eigen::Vector3d::Zero(), 5.5e-12));
    CHECK(state.n_velocity() == n_vel);
    LOG_DATA("{}", state.e_velocity().norm() - state.n_velocity().norm());
    CHECK_THAT(state.e_velocity().norm() - state.n_velocity().norm(), Catch::Matchers::WithinAbs(0, 3.7e-12));
}

TEST_CASE("[PosVelAtt] Attitude Functions", "[PosVelAtt]")
{
    auto logger = initializeTestLogger();

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

    CHECK_THAT((state.lla_position() - lla_position).head<2>(), Catch::Matchers::WithinAbs(Eigen::Vector2d::Zero(), EPSILON));
    CHECK_THAT((state.lla_position() - lla_position)(2), Catch::Matchers::WithinAbs(0, 1.3e-9));
    CHECK_THAT(state.e_velocity(), Catch::Matchers::WithinAbs(e_vel, EPSILON));
    CHECK_THAT(state.n_Quat_b(), Catch::Matchers::WithinAbs(trafo::n_Quat_b(roll, pitch, yaw), EPSILON));
    CHECK_THAT(state.b_Quat_n(), Catch::Matchers::WithinAbs(trafo::b_Quat_n(roll, pitch, yaw), EPSILON));
    CHECK_THAT(state.e_Quat_b(), Catch::Matchers::WithinAbs(trafo::e_Quat_n(lla_position(0), lla_position(1)) * trafo::n_Quat_b(roll, pitch, yaw), EPSILON));
    CHECK_THAT(state.b_Quat_e(), Catch::Matchers::WithinAbs(trafo::b_Quat_n(roll, pitch, yaw) * trafo::n_Quat_e(lla_position(0), lla_position(1)), EPSILON));

    state.setState_n(lla_position, n_vel, trafo::n_Quat_b(roll, pitch, yaw));
    CHECK_THAT(state.n_velocity(), Catch::Matchers::WithinAbs(n_vel, EPSILON));
    CHECK_THAT(state.n_Quat_b(), Catch::Matchers::WithinAbs(trafo::n_Quat_b(roll, pitch, yaw), EPSILON));
    CHECK_THAT(state.b_Quat_n(), Catch::Matchers::WithinAbs(trafo::b_Quat_n(roll, pitch, yaw), EPSILON));
    CHECK_THAT(state.e_Quat_b(), Catch::Matchers::WithinAbs(trafo::e_Quat_n(lla_position(0), lla_position(1)) * trafo::n_Quat_b(roll, pitch, yaw), EPSILON));
    CHECK_THAT(state.b_Quat_e(), Catch::Matchers::WithinAbs(trafo::b_Quat_n(roll, pitch, yaw) * trafo::n_Quat_e(lla_position(0), lla_position(1)), EPSILON));
}

TEST_CASE("[PosVelAtt] Attitude RollPitchYaw", "[PosVelAtt]")
{
    auto logger = initializeTestLogger();

    // Stuttgart, Breitscheidstraße 2
    // https://www.koordinaten-umrechner.de/decimal/48.780810,9.172012?karte=OpenStreetMap&zoom=19
    PosVelAtt state;
    state.setPosition_lla(Eigen::Vector3d{ deg2rad(48.78081), deg2rad(9.172012), 254 });

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
                REQUIRE_THAT(rad2deg(Eigen::Vector3d{ expectedRoll, expectedPitch, expectedYaw }), Catch::Matchers::WithinAbs(actualRollPitchYaw, 1e-8));
            }
        }
    }
}

TEST_CASE("[PosVelAtt] State setStateAndStdDev_n", "[PosVelAtt]")
{
    auto logger = initializeTestLogger();

    Eigen::Vector3d lla_position{ deg2rad(0.0), deg2rad(0.0), 0.0 };
    Eigen::Matrix3d n_positionCovarianceMatrix = Eigen::Matrix3d::Zero();
    n_positionCovarianceMatrix.diagonal() << 1, 4, 9;
    Eigen::Vector3d n_velocity = Eigen::Vector3d::Zero();
    Eigen::Matrix3d n_velocityCovarianceMatrix = Eigen::Matrix3d::Zero();
    n_velocityCovarianceMatrix.diagonal() << 1, 4, 9;

    PosVelAtt state;
    state.setStateAndStdDev_n(lla_position, n_positionCovarianceMatrix,
                              n_velocity, n_velocityCovarianceMatrix,
                              trafo::n_Quat_b(0.0, 0.0, 0.0));

    REQUIRE_THAT(state.e_positionStdev().value().get(), Catch::Matchers::WithinAbs(Eigen::Vector3d(3, 2, 1), 1e-8));
    REQUIRE_THAT(state.e_velocityStdev().value().get(), Catch::Matchers::WithinAbs(Eigen::Vector3d(3, 2, 1), 1e-8));
}

} // namespace NAV::TESTS::PosVelAttTests
