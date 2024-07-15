// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file PolynomialCycleSlipDetectorTests.cpp
/// @brief Cycle Slip Detector Tests
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-10-30

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"
#include "Logger.hpp"

#include <random>

#include "Navigation/GNSS/Ambiguity/CycleSlipDetector/PolynomialCycleSlipDetector.hpp"

namespace NAV::TESTS
{

TEST_CASE("[PolynomialCycleSlipDetector] PolynomialCycleSlipDetector", "[PolynomialCycleSlipDetector]")
{
    auto logger = initializeTestLogger();

    const double threshold = 0.1;
    const size_t windowSize = 5;
    PolynomialCycleSlipDetector<std::pair<SatSigId, SatSigId>> detector(windowSize, 2);
    InsTime startTime(2020, 1, 1, 0, 0, 0);

    SatSigId primary{ Code::G1C, 1 };
    SatSigId secondary{ Code::G2C, 1 };

    std::random_device r;
    std::default_random_engine en(r());
    std::normal_distribution<double> dist(0, threshold / 2.0);

    for (size_t i = 0; i < windowSize; i++)
    {
        InsTime t = startTime + std::chrono::seconds(i);
        double x = std::min(std::max(dist(en), threshold / 2.0), -threshold / 2.0);

        auto result = detector.checkForCycleSlip({ primary, secondary }, t, x, threshold);
        REQUIRE(result == PolynomialCycleSlipDetectorResult::LessDataThanWindowSize);
    }

    InsTime t = startTime + std::chrono::seconds(windowSize);
    {
        auto result = detector.checkForCycleSlip({ primary, secondary }, t, threshold / 2.0 - 1e-4, threshold);
        REQUIRE(result == PolynomialCycleSlipDetectorResult::NoCycleSlip);
    }
    t += std::chrono::seconds(windowSize);
    {
        auto result = detector.checkForCycleSlip({ primary, secondary }, t, threshold / 2.0 + 1e-4, threshold);
        REQUIRE(result == PolynomialCycleSlipDetectorResult::CycleSlip);
    }
}

} // namespace NAV::TESTS