// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file MathTests.cpp
/// @brief Math related tests
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-02-15

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"

#include "Logger.hpp"
#include "Navigation/Math/Math.hpp"

namespace NAV::TESTS
{

TEST_CASE("[Math] Catch Matcher significant digits", "[Math]")
{
    auto logger = initializeTestLogger();

    double val1 = 0.4657e-08;
    double val2 = 0.46574932423e-08;
    LOG_DEBUG("{:.4f} == {:.4f}", val1, math::roundSignificantDigits(val2, 4));
    REQUIRE(math::roundSignificantDigits(val1, 4) == math::roundSignificantDigits(val2, 4));
    REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 4));

    val1 = std::stod("0.4657e-08");
    LOG_DEBUG("{:.4f} == {:.4f}", val1, math::roundSignificantDigits(val2, 4));
    REQUIRE(math::roundSignificantDigits(val1, 4) == math::roundSignificantDigits(val2, 4));
    REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 4));

    val1 = std::stod("0.1490e-07");
    val2 = 0.149000000002e-07;
    LOG_DEBUG("{:.4f} == {:.4f}", val1, math::roundSignificantDigits(val2, 4));
    REQUIRE(math::roundSignificantDigits(val1, 4) == math::roundSignificantDigits(val2, 4));
    REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 4));

    val1 = std::stod("0.2892e+00");
    val2 = 0.28915;
    LOG_DEBUG("{:.4f} == {:.4f}", val1, math::roundSignificantDigits(val2, 4));
    REQUIRE(math::roundSignificantDigits(val1, 4) == math::roundSignificantDigits(val2, 4));
    REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 4));

    val1 = std::stod("0.2892e+00");
    val2 = 0.28914;
    LOG_DEBUG("{:.4f} == {:.4f}", val1, math::roundSignificantDigits(val2, 4));
    REQUIRE(math::roundSignificantDigits(val1, 4) != math::roundSignificantDigits(val2, 4));
    REQUIRE_THAT(val1, !Catch::Matchers::EqualsSigDigits(val2, 4));

    val1 = std::stod("-0.2892e+00");
    val2 = -0.28915;
    LOG_DEBUG("{:.4f} == {:.4f}", val1, math::roundSignificantDigits(val2, 4));
    REQUIRE(math::roundSignificantDigits(val1, 4) == math::roundSignificantDigits(val2, 4));
    REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 4));

    val1 = std::stod("0.2891232e+10");
    val2 = 0.28912321334e10;
    LOG_DEBUG("{:.7f} == {:.7f}", val1, math::roundSignificantDigits(val2, 7));
    REQUIRE(math::roundSignificantDigits(val1, 7) == math::roundSignificantDigits(val2, 7));
    REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 7));

    val1 = std::stod("10.23");
    val2 = 10.2345;
    LOG_DEBUG("{:.4f} == {:.4f}", val1, math::roundSignificantDigits(val2, 4));
    REQUIRE(math::roundSignificantDigits(val1, 4) == math::roundSignificantDigits(val2, 4));
    REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 4));

    val1 = std::stod("0.123");
    val2 = 0.12345;
    LOG_DEBUG("{:.3f} == {:.3f}", val1, math::roundSignificantDigits(val2, 3));
    REQUIRE(math::roundSignificantDigits(val1, 3) == math::roundSignificantDigits(val2, 3));
    REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 3));

    val1 = std::stod("0.0123");
    val2 = 0.012345;
    LOG_DEBUG("{:.3f} == {:.3f}", val1, math::roundSignificantDigits(val2, 3));
    REQUIRE(math::roundSignificantDigits(val1, 3) == math::roundSignificantDigits(val2, 3));
    REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 3));

    val1 = std::stod("-0.1490e-07");
    val2 = -0.149000000002e-07;
    LOG_DEBUG("{:.4f} == {:.4f}", val1, math::roundSignificantDigits(val2, 4));
    REQUIRE(math::roundSignificantDigits(val1, 4) == math::roundSignificantDigits(val2, 4));
    REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 4));

    val1 = 0.0;
    REQUIRE(val1 == math::roundSignificantDigits(val1, 2));
    REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val1, 2));

    val1 = 1e-19;
    REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val1, 19));

    val1 = 1.85041689389;
    val2 = 1.850416893885315;
    LOG_DEBUG("{:.12f} == {:.12f}", val1, math::roundSignificantDigits(val2, 12));
    REQUIRE(math::roundSignificantDigits(val1, 12) == math::roundSignificantDigits(val2, 12));
    REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 12));

    val1 = -1.85041689389;
    val2 = -1.850416893885315;
    LOG_DEBUG("{:.12f} == {:.12f}", val1, math::roundSignificantDigits(val2, 12));
    REQUIRE(math::roundSignificantDigits(val1, 12) == math::roundSignificantDigits(val2, 12));
    REQUIRE_THAT(val1, Catch::Matchers::EqualsSigDigits(val2, 12));
}

TEST_CASE("[Math] Catch Matcher significant digits container", "[Math]")
{
    auto logger = initializeTestLogger();

    std::array<double, 6> arr1 = { 0.4657e-08, 0.1490e-07, 0.2892e+00, -0.2892e+00, 10.23, -0.1490e-07 };
    std::array<double, 6> arr2 = { 0.46574932423e-08, 0.149000000002e-07, 0.28915, -0.28915, 10.2345, -0.149000000002e-07 };
    LOG_DEBUG("Array:\n{}\n    ==\n{}", joinToString(arr1, ", ", ":.5e"), joinToString(arr2, ", ", ":.5e"));
    REQUIRE_THAT(arr1, Catch::Matchers::EqualsSigDigitsContainer(arr2, 4));

    std::vector<double> vec1;
    std::vector<double> vec2;
    std::ranges::copy(arr1, std::back_inserter(vec1));
    std::ranges::copy(arr2, std::back_inserter(vec2));
    LOG_DEBUG("Vector:\n{}\n    ==\n{}", joinToString(vec1, ", ", ":.5e"), joinToString(vec2, ", ", ":.5e"));
    REQUIRE_THAT(vec1, Catch::Matchers::EqualsSigDigitsContainer(vec2, 4));
}

} // namespace NAV::TESTS