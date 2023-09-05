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
#include "Navigation/GNSS/Ambiguity/LAMBDA.hpp"

namespace NAV::TESTS::AmbiguityTests
{

/// @brief Find LtDL-decompostion of Q-matrix
/// @param[in] Q Symmetric n by n matrix to be factored
/// @return L - n by n factor matrix (strict lower triangular)
/// @return D - Diagonal n-vector
/// @note Code from
///         File.....: ldldecom
///         Date.....: 19-MAY-1999
///         Author...: Peter Joosten
///                    Mathematical Geodesy and Positioning
///                    Delft University of Technology
template<typename Derived>
std::pair<Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>,
          Eigen::Vector<typename Derived::Scalar, Derived::RowsAtCompileTime>>
    LtDLdecomp(const Eigen::MatrixBase<Derived>& Q)
{
    using Eigen::seq;

    auto n = Q.rows();
    Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> Qahat = Q;
    Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> L;
    Eigen::Vector<typename Derived::Scalar, Derived::RowsAtCompileTime> D;

    if constexpr (Derived::RowsAtCompileTime == Eigen::Dynamic)
    {
        L = Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(n, n);
        D = Eigen::Vector<typename Derived::Scalar, Eigen::Dynamic>::Zero(n);
    }
    else
    {
        L = Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>::Zero();
        D = Eigen::Vector<typename Derived::Scalar, Derived::RowsAtCompileTime>::Zero();
    }

    for (Eigen::Index i = n - 1; i >= 0; i--)
    {
        D(i) = Qahat(i, i);
        L(i, seq(0, i)) = Qahat(i, seq(0, i)) / std::sqrt(Qahat(i, i));

        for (Eigen::Index j = 0; j <= i - 1; j++)
        {
            Qahat(j, seq(0, j)) = Qahat(j, seq(0, j)) - L(i, seq(0, j)) * L(i, j);
        }
        L(i, seq(0, i)) = L(i, seq(0, i)) / L(i, i);
    }

    return { L, D };
}

TEST_CASE("[Ambiguity] LDL Decomposition", "[Ambiguity]")
{
    auto logger = initializeTestLogger();

    // erzeuge die eine (mxn m>>n, also zum bsp. 40 x 4) matrix A, dann bau dir daraus deine Q matrix als Q = inv(A'*A)

    constexpr size_t m = 3;
    constexpr size_t n = 3;
    using Matrix = Eigen::Matrix<double, m, m>;

    Eigen::Matrix<double, m, n> A = Eigen::Matrix<double, m, n>::Random();
    Matrix Qahat = A * A.transpose();
    Qahat += m * Matrix::Identity();

    // Qahat << 2, -0.5, 0,
    //     -0.5, 2, -1,
    //     0, -1, 2;

    LOG_INFO("A = \n{}", A);
    LOG_INFO("Q = \n{}\n", Qahat);

    Eigen::LLT<Matrix> lltOfQahat(Qahat);         // See https://eigen.tuxfamily.org/dox/classEigen_1_1LLT.html
    REQUIRE(lltOfQahat.info() == Eigen::Success); // Success if computation was successful, NumericalIssue if the matrix.appears not to be positive definite.

    auto start{ std::chrono::steady_clock::now() };
    auto [L, D] = LtDLdecomp(Qahat);
    auto end{ std::chrono::steady_clock::now() };
    LOG_INFO("Matlab (TU Delft): L^T * D * L");
    LOG_INFO("Elapsed time: {}", std::chrono::duration<double>(end - start).count());
    LOG_INFO("L = \n{}", L);
    LOG_INFO("D = {}", D.transpose());
    Matrix LTDL_minus_Q = L.transpose() * Eigen::DiagonalMatrix<double, m>(D) * L - Qahat;
    LOG_INFO("L^T * D * L - Q = \n{}\n", LTDL_minus_Q);
    REQUIRE_THAT(LTDL_minus_Q, Catch::Matchers::WithinAbs(Matrix::Zero(), 1e-10));

    // The Eigen LDLT apparently is not working for random Q matrices. So it should not be used

    start = std::chrono::steady_clock::now();
    Eigen::LDLT<Matrix> ldltOfQahat(Qahat); // See https://eigen.tuxfamily.org/dox/classEigen_1_1LDLT.html
    end = std::chrono::steady_clock::now();
    REQUIRE(ldltOfQahat.info() == Eigen::Success); // Success if computation was successful, NumericalIssue if the factorization failed because of a zero pivot.
    Eigen::MatrixXd Leigen = ldltOfQahat.matrixL();
    Eigen::VectorXd Deigen = ldltOfQahat.vectorD();
    LOG_INFO("Eigen: L * D * L^T");
    LOG_INFO("Elapsed time: {}", std::chrono::duration<double>(end - start).count());
    LOG_INFO("L = \n{}", Leigen);
    LOG_INFO("D = {}", Deigen.transpose());
    [[maybe_unused]] Matrix LDLT_minus_Q = Leigen * Eigen::DiagonalMatrix<double, m>(Deigen) * Leigen.transpose() - Qahat;
    LOG_INFO("L * D * L^T - Q = \n{}", LDLT_minus_Q);
    // REQUIRE_THAT(LDLT_minus_Q, Catch::Matchers::WithinAbs(Matrix::Zero(), 1e-10));
}

} // namespace NAV::TESTS::AmbiguityTests