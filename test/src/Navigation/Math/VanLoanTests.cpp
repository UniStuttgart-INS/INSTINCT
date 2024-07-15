// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SortTests.cpp
/// @brief Test for sorting algorithms
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-15

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"

#include "Logger.hpp"
#include "Navigation/Math/VanLoan.hpp"
#include "util/Eigen.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"

namespace NAV::TESTS
{

TEST_CASE("[VanLoan] Assemble from subparts", "[VanLoan]")
{
    auto logger = initializeTestLogger();

    double dt = 1.0;
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(10, 10);
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(10, 10);
    G.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero();
    G.block<3, 3>(3, 3) = trafo::e_Quat_n(deg2rad(20), deg2rad(30)).toRotationMatrix();
    Eigen::MatrixXd W = 1e-6 * Eigen::MatrixXd::Identity(10, 10);
    W.block<6, 6>(0, 0).diagonal() << Eigen::Vector3d::Zero(), 1e-2 * Eigen::Vector3d::Ones();

    LOG_DEBUG("F = \n{}", F);
    LOG_DEBUG("G = \n{}", G);
    LOG_DEBUG("W = \n{}", W);
    auto [Phi, Q] = calcPhiAndQWithVanLoanMethod(F, G, W, dt);

    LOG_DEBUG("Phi = \n{}", Phi);
    LOG_DEBUG("Q = \n{}", Q);
    REQUIRE(Phi == Eigen::MatrixXd::Identity(10, 10) + F * dt);

    auto [Phi_pv, Q_pv] = calcPhiAndQWithVanLoanMethod(F.topLeftCorner<6, 6>(), G.topLeftCorner<6, 6>(), W.topLeftCorner<6, 6>(), dt);
    LOG_DEBUG("Phi(p,v) = \n{}", Phi_pv);
    LOG_DEBUG("Q(p,v) = \n{}", Q_pv);

    REQUIRE_THAT(Eigen::MatrixXd(Phi.topLeftCorner<6, 6>() - Phi_pv), Catch::Matchers::WithinAbs(Eigen::MatrixXd::Zero(6, 6), 1e-12));
    REQUIRE_THAT(Eigen::MatrixXd(Q.topLeftCorner<6, 6>() - Q_pv), Catch::Matchers::WithinAbs(Eigen::MatrixXd::Zero(6, 6), 1e-12));
}

} // namespace NAV::TESTS