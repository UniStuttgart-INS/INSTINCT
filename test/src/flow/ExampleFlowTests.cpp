// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ExampleFlowTests.cpp
/// @brief Run all the example flows without testing boundaries, just checking if they still load and initialize
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-08-08

#include <catch2/catch_test_macros.hpp>

#include "FlowTester.hpp"

#include "Logger.hpp"

namespace NAV::TESTS::ExampleFlowTests
{

TEST_CASE("[ExampleFlow] Test _ImuFusion.flow", "[ExampleFlow][flow]")
{
    auto logger = initializeTestLogger();
    REQUIRE(testFlow("flow/_ImuFusion.flow", false));
}

TEST_CASE("[ExampleFlow] Test _InsGnss-LCKF.flow", "[ExampleFlow][flow]")
{
    auto logger = initializeTestLogger();
    REQUIRE(testFlow("flow/_InsGnss-LCKF.flow", false));
}

TEST_CASE("[ExampleFlow] Test _SPP.flow", "[ExampleFlow][flow]")
{
    auto logger = initializeTestLogger();
    REQUIRE(testFlow("flow/_SPP.flow", false));
}

} // namespace NAV::TESTS::ExampleFlowTests
