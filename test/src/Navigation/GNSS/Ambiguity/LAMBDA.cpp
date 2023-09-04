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

    Eigen::Matrix3d Qahat;
    Qahat << 2, -0.5, 0,
        -0.5, 2, -1,
        0, -1, 2;

    Eigen::LLT<Eigen::Matrix3d> lltOfQahat(Qahat); // See https://eigen.tuxfamily.org/dox/classEigen_1_1LLT.html
    REQUIRE(lltOfQahat.info() == Eigen::Success);  // Success if computation was successful, NumericalIssue if the matrix.appears not to be positive definite.

    auto start{ std::chrono::steady_clock::now() };
    auto [L, D] = LtDLdecomp(Qahat);
    auto end{ std::chrono::steady_clock::now() };
    LOG_INFO("Matlab (TU Delft): L^T * D * L");
    LOG_INFO("Elapsed time: {}", std::chrono::duration<double>(end - start).count());
    LOG_INFO("L = \n{}", L);
    LOG_INFO("D = {}\n", D.transpose());
    REQUIRE(L.transpose() * Eigen::DiagonalMatrix<double, 3>(D) * L == Qahat);

    start = std::chrono::steady_clock::now();
    Eigen::LDLT<Eigen::Matrix3d> ldltOfQahat(Qahat); // See https://eigen.tuxfamily.org/dox/classEigen_1_1LDLT.html
    end = std::chrono::steady_clock::now();
    REQUIRE(ldltOfQahat.info() == Eigen::Success); // Success if computation was successful, NumericalIssue if the factorization failed because of a zero pivot.
    Eigen::MatrixXd Leigen = ldltOfQahat.matrixL();
    Eigen::VectorXd Deigen = ldltOfQahat.vectorD();
    LOG_INFO("Eigen: L * D * L^T");
    LOG_INFO("Elapsed time: {}", std::chrono::duration<double>(end - start).count());
    LOG_INFO("L = \n{}", Leigen);
    LOG_INFO("D = {}", Deigen.transpose());
    REQUIRE(Leigen * Eigen::DiagonalMatrix<double, 3>(Deigen) * Leigen.transpose() == Qahat);
}

} // namespace NAV::TESTS::AmbiguityTests