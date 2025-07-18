// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ValidationTests.cpp
/// @brief Ambiguity Validation Tests
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-10-06

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"
#include "Logger.hpp"

#include <cmath>
#include "Navigation/Math/Math.hpp"
#include "Navigation/GNSS/Ambiguity/internal/Validate.hpp"

namespace NAV::TESTS::AmbiguityTests
{

TEST_CASE("[Ambiguity] Difference Test", "[Ambiguity]")
{
    auto logger = initializeTestLogger();

    Eigen::Matrix3d Qa; // Example taken from 'de Jonge 1996, eq. 3.37'
    Qa << 6.290, 5.978, 0.544,
        5.978, 6.292, 2.340,
        0.544, 2.340, 6.288;
    Eigen::Vector3d aFloat(5.45, 3.10, 2.97);

    Eigen::Vector3d aFix1(5, 3, 4);
    Eigen::Vector3d aFix2(6, 4, 4);

    double sqNorm1 = 0.218331095336938;
    double sqNorm2 = 0.307272575790267;

    double criticalValue = 0.08; // Has to empirically determined

    double sqNorm1_calc = math::squaredNormVectorMatrix(aFloat - aFix1, Qa);
    double sqNorm2_calc = math::squaredNormVectorMatrix(aFloat - aFix2, Qa);

    REQUIRE_THAT(sqNorm1, Catch::Matchers::WithinAbs(sqNorm1_calc, 1e-10));
    REQUIRE_THAT(sqNorm2, Catch::Matchers::WithinAbs(sqNorm2_calc, 1e-10));

    LOG_DEBUG("sqNorm2 - sqNorm1 = {}", sqNorm2 - sqNorm1);

    REQUIRE(Ambiguity::differenceTest(aFix1, aFix2, aFloat, Qa, criticalValue));
    REQUIRE(Ambiguity::differenceTest(sqNorm1, sqNorm2, criticalValue));
}

TEST_CASE("[Ambiguity] Ratio Test Critical Value", "[Ambiguity]")
{
    auto logger = initializeTestLogger();

    Eigen::Matrix3d Qa; // Example taken from 'de Jonge 1996, eq. 3.37'
    Qa << 6.290, 5.978, 0.544,
        5.978, 6.292, 2.340,
        0.544, 2.340, 6.288;
    Eigen::Vector3d aFloat(5.45, 3.10, 2.97);

    Eigen::Vector3d aFix1(5, 3, 4);
    Eigen::Vector3d aFix2(6, 4, 4);

    double sqNorm1 = 0.218331095336938;
    double sqNorm2 = 0.307272575790267;

    double criticalValue = 0.72;

    LOG_DEBUG("sqNorm1 / sqNorm2 = {}", sqNorm1 / sqNorm2);

    REQUIRE(Ambiguity::ratioTest(aFix1, aFix2, aFloat, Qa, criticalValue));
    REQUIRE(Ambiguity::ratioTest(sqNorm1, sqNorm2, criticalValue));
}

TEST_CASE("[Ambiguity] Projection Test", "[Ambiguity]")
{
    auto logger = initializeTestLogger();

    Eigen::Matrix3d Qa; // Example taken from 'de Jonge 1996, eq. 3.37'
    Qa << 6.290, 5.978, 0.544,
        5.978, 6.292, 2.340,
        0.544, 2.340, 6.288;
    Eigen::Vector3d aFloat(5.45, 3.10, 2.97);

    Eigen::Vector3d aFix1(5, 3, 4);
    Eigen::Vector3d aFix2(6, 4, 4);

    double criticalValue = 0.31;

    [[maybe_unused]] double proj = (aFix2 - aFix1).transpose() * Qa.inverse() * (aFloat - aFix1);
    LOG_DEBUG("Projection: {}", std::abs(proj / math::squaredNormVectorMatrix(aFix2 - aFix1, Qa)));

    REQUIRE(Ambiguity::projectorTest(aFix1, aFix2, aFloat, Qa, criticalValue));
}

} // namespace NAV::TESTS::AmbiguityTests