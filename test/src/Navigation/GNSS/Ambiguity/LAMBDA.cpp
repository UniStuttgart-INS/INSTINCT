// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file LAMBDA.cpp
/// @brief SatelliteIdentifier Tests
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-04

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"
#include "Logger.hpp"

#include <util/Eigen.hpp>
#include "Navigation/Math/Math.hpp"
#include "Navigation/GNSS/Ambiguity/LAMBDA.hpp"

namespace NAV::TESTS::AmbiguityTests
{

TEST_CASE("[Ambiguity] LDL Decomposition", "[Ambiguity]")
{
    auto logger = initializeTestLogger();

    constexpr size_t m = 30;
    constexpr size_t n = 3;
    // using Matrix = Eigen::Matrix<double, m, m>;
    using Matrix = Eigen::MatrixXd;

    Matrix A = Matrix::Random(m, n);
    Matrix Q = (A.transpose() * A).inverse();

    // Q << 2, -0.5, 0,
    //     -0.5, 2, -1,
    //     0, -1, 2;

    LOG_INFO("A = \n{}", A);
    LOG_INFO("Q = \n{}\n", Q);

    Eigen::LLT<Matrix> lltOfQ(Q);             // See https://eigen.tuxfamily.org/dox/classEigen_1_1LLT.html
    REQUIRE(lltOfQ.info() == Eigen::Success); // Success if computation was successful, NumericalIssue if the matrix.appears not to be positive definite.

    {
        const auto start{ std::chrono::steady_clock::now() };
        const auto [L, D] = math::LtDLdecomp_outerProduct(Q);
        const auto end{ std::chrono::steady_clock::now() };
        LOG_INFO("LtDLdecomp_outerProduct (FMFAC5): L^T * D * L");
        LOG_INFO("Elapsed time: {}", std::chrono::duration<double>(end - start).count());
        // LOG_INFO("L = \n{}", L);
        // LOG_INFO("D = {}", D.diagonal().transpose());
        Matrix LTDL_minus_Q = L.transpose() * D * L - Q;
        // LOG_INFO("L^T * D * L - Q = \n{}\n", LTDL_minus_Q);
        REQUIRE_THAT(LTDL_minus_Q, Catch::Matchers::WithinAbs(Matrix::Zero(n, n), 1e-10));
    }
    // LtDLdecomp_choleskyFact is faster than LtDLdecomp_outerProduct
    {
        const auto start{ std::chrono::steady_clock::now() };
        const auto [L, D] = math::LtDLdecomp_choleskyFact(Q);
        const auto end{ std::chrono::steady_clock::now() };
        LOG_INFO("LtDLdecomp_choleskyFact (FMFAC6): L^T * D * L");
        LOG_INFO("Elapsed time: {}", std::chrono::duration<double>(end - start).count());
        LOG_INFO("L = \n{}", L);
        LOG_INFO("D = {}", D.diagonal().transpose());
        Matrix LTDL_minus_Q = L.transpose() * D * L - Q;
        LOG_INFO("L^T * D * L - Q = \n{}\n", LTDL_minus_Q);
        REQUIRE_THAT(LTDL_minus_Q, Catch::Matchers::WithinAbs(Matrix::Zero(n, n), 1e-10));
    }
    // The Eigen LDLT apparently is not working for random Q matrices. So it should not be used
    {
        const auto start{ std::chrono::steady_clock::now() };
        Eigen::LDLT<Matrix> ldltOfQ(Q); // See https://eigen.tuxfamily.org/dox/classEigen_1_1LDLT.html
        const auto end{ std::chrono::steady_clock::now() };
        REQUIRE(ldltOfQ.info() == Eigen::Success); // Success if computation was successful, NumericalIssue if the factorization failed because of a zero pivot.
        Eigen::MatrixXd Leigen = ldltOfQ.matrixL();
        Eigen::VectorXd Deigen = ldltOfQ.vectorD();
        LOG_INFO("Eigen: L * D * L^T");
        LOG_INFO("Elapsed time: {}", std::chrono::duration<double>(end - start).count());
        LOG_INFO("L = \n{}", Leigen);
        LOG_INFO("D = {}", Deigen.transpose());
        Matrix Dmatrix = Eigen::DiagonalMatrix<double, m>(Deigen);
        [[maybe_unused]] Matrix LDLT_minus_Q = Leigen * Dmatrix * Leigen.transpose() - Q;
        LOG_INFO("L * D * L^T - Q = \n{}", LDLT_minus_Q);
        // REQUIRE_THAT(LDLT_minus_Q, Catch::Matchers::WithinAbs(Matrix::Zero(n, n), 1e-10)); // This fails
    }
}

} // namespace NAV::TESTS::AmbiguityTests