// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <catch2/catch.hpp>

#include "Navigation/Transformations/Units.hpp"
#include "Logger.hpp"

namespace NAV::TESTS::CoordinateFramesTests
{

TEST_CASE("[Units] Degree to radian conversion", "[Units]")
{
    auto logger = initializeTestLogger();

    double rad_90 = deg2rad(90);
    double rad_180 = deg2rad(180.0);
    double rad_360 = deg2rad(360.0F);

    REQUIRE(rad_90 == M_PI_2);
    REQUIRE(rad_180 == M_PI);
    REQUIRE(rad_360 == M_PI * 2.0);

    Eigen::Vector3d rad3 = deg2rad(Eigen::Vector3d{ 90, 180, 360 });

    REQUIRE(rad3.x() == M_PI_2);
    REQUIRE(rad3.y() == M_PI);
    REQUIRE(rad3.z() == M_PI * 2.0);
}

TEST_CASE("[Units] Degree to radian conversion constexpr", "[Units]")
{
    auto logger = initializeTestLogger();

    constexpr double rad_90 = deg2rad(90);
    constexpr double rad_180 = deg2rad(180.0);
    constexpr double rad_360 = deg2rad(360.0F);

    STATIC_REQUIRE(rad_90 == M_PI_2);
    STATIC_REQUIRE(rad_180 == M_PI);
    STATIC_REQUIRE(rad_360 == M_PI * 2.0);
}

TEST_CASE("[Units] Radian to degree conversion", "[Units]")
{
    auto logger = initializeTestLogger();

    double deg_90 = rad2deg(M_PI_2);
    double deg_180 = rad2deg(M_PI);
    double deg_360 = rad2deg(M_PI * 2.0);

    REQUIRE(deg_90 == 90);
    REQUIRE(deg_180 == 180);
    REQUIRE(deg_360 == 360);

    Eigen::Vector3d deg3 = rad2deg(Eigen::Vector3d{ M_PI_2, M_PI, M_PI * 2.0 });

    REQUIRE(deg3.x() == 90);
    REQUIRE(deg3.y() == 180);
    REQUIRE(deg3.z() == 360);
}

TEST_CASE("[Units] Radian to degree conversion constexpr", "[Units]")
{
    auto logger = initializeTestLogger();

    constexpr double deg_90 = rad2deg(M_PI_2);
    constexpr double deg_180 = rad2deg(M_PI);
    constexpr double deg_360 = rad2deg(M_PI * 2.0);

    STATIC_REQUIRE(deg_90 == 90.0);
    STATIC_REQUIRE(deg_180 == 180.0);
    STATIC_REQUIRE(deg_360 == 360.0);
}

} // namespace NAV::TESTS::CoordinateFramesTests