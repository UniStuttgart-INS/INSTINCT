// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <catch2/catch_test_macros.hpp>

#include "Logger.hpp"
#include "Navigation/Math/ManagedKalmanFilter.hpp"

namespace NAV::TESTS
{

TEST_CASE("[Math] Managed Kalman Filter", "[Math]")
{
    auto logger = initializeTestLogger();

    ManagedKalmanFilter<int, std::string> mkf;

    REQUIRE(mkf.x.rows() == 2);
}

} // namespace NAV::TESTS