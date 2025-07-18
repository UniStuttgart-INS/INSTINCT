// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file DecorrelationTests.cpp
/// @brief Ambiguity Decorrelation Tests
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-04

#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"
#include "Logger.hpp"

#include "util/Eigen.hpp"
#include "Navigation/Math/Math.hpp"
#include "Navigation/GNSS/Ambiguity/internal/Decorrelation.hpp"

namespace NAV::TESTS::AmbiguityTests
{

namespace
{
/* LD factorization (Q=L'*diag(D)*L) -----------------------------------------*/
template<typename Derived>
std::optional<std::pair<Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>,
                        Eigen::Vector<typename Derived::Scalar, Derived::RowsAtCompileTime>>>
    LtDLdecomp_Rtklib(const Eigen::MatrixBase<Derived>& Q)
{
    auto n = static_cast<int>(Q.rows());
    double a = 0.0;
    typename Derived::PlainObject A = Q;

    typename Derived::PlainObject L = Q.template triangularView<Eigen::Lower>();

    Eigen::Vector<typename Derived::Scalar, Derived::RowsAtCompileTime> D;
    if constexpr (Derived::RowsAtCompileTime == Eigen::Dynamic) { D.setZero(n); }
    else { D.setZero(); }

    for (int i = n - 1; i >= 0; i--)
    {
        D(i) = A(i, i);
        if (D(i) <= 0.0) { return {}; }
        a = std::sqrt(D(i));
        for (int j = 0; j <= i; j++)
        {
            L(i, j) = A(i, j) / a;
        }
        for (int j = 0; j <= i - 1; j++)
        {
            for (int k = 0; k <= j; k++)
            {
                A(j, k) -= L(i, k) * L(i, j);
            }
        }
        for (int j = 0; j <= i; j++)
        {
            L(i, j) /= L(i, i);
        }
    }

    return { std::make_pair(L, D) };
}
} // namespace

TEST_CASE("[Ambiguity] LDL Decomposition", "[Ambiguity]")
{
    auto logger = initializeTestLogger();

    constexpr size_t m = 30;
    constexpr size_t n = 3;
    // using Matrix = Eigen::Matrix<double, n, n>;
    using Matrix = Eigen::MatrixXd;

    const Matrix A = Matrix::Random(m, n);
    const Matrix Q = (A.transpose() * A).inverse();

    // Q << 6.290, 5.978, 0.544, // Example taken from 'de Jonge 1996, eq. 3.37'
    //     5.978, 6.292, 2.340,
    //     0.544, 2.340, 6.288;

    LOG_INFO("A = \n{}", A);
    LOG_INFO("Q = \n{}\n", Q);

    Eigen::LLT<Matrix> lltOfQ(Q);             // See https://eigen.tuxfamily.org/dox/classEigen_1_1LLT.html
    REQUIRE(lltOfQ.info() == Eigen::Success); // Success if computation was successful, NumericalIssue if the matrix.appears not to be positive definite.

    {
        [[maybe_unused]] const auto start{ std::chrono::steady_clock::now() };
        const auto [L, Dvec] = math::LtDLdecomp_outerProduct(Q).value();
        [[maybe_unused]] const auto end{ std::chrono::steady_clock::now() };
        LOG_WARN("LtDLdecomp_outerProduct (FMFAC5): L^T * D * L");
        LOG_INFO("Elapsed time: {}", std::chrono::duration<double>(end - start).count());
        Matrix D = Eigen::DiagonalMatrix<double, n>(Dvec);
        LOG_INFO("L = \n{}", L);
        LOG_INFO("D = \n{}", D);
        Matrix LTDL_minus_Q = L.transpose() * D * L - Q;
        LOG_INFO("L^T * D * L - Q = \n{}\n", LTDL_minus_Q);
        REQUIRE_THAT(LTDL_minus_Q, Catch::Matchers::WithinAbs(Matrix::Zero(n, n), 1e-10));
    }
    // LtDLdecomp_choleskyFact is faster than LtDLdecomp_outerProduct
    {
        [[maybe_unused]] const auto start{ std::chrono::steady_clock::now() };
        const auto [L, Dvec] = math::LtDLdecomp_choleskyFact(Q).value();
        [[maybe_unused]] const auto end{ std::chrono::steady_clock::now() };
        LOG_WARN("LtDLdecomp_choleskyFact (FMFAC6): L^T * D * L");
        LOG_INFO("Elapsed time: {}", std::chrono::duration<double>(end - start).count());
        Matrix D = Eigen::DiagonalMatrix<double, n>(Dvec);
        LOG_INFO("L = \n{}", L);
        LOG_INFO("D = \n{}", D);
        Matrix LTDL_minus_Q = L.transpose() * D * L - Q;
        LOG_INFO("L^T * D * L - Q = \n{}\n", LTDL_minus_Q);
        REQUIRE_THAT(LTDL_minus_Q, Catch::Matchers::WithinAbs(Matrix::Zero(n, n), 1e-10));
    }
    // Rtklib variant
    {
        [[maybe_unused]] const auto start{ std::chrono::steady_clock::now() };
        auto ld = LtDLdecomp_Rtklib(Q);
        REQUIRE(ld.has_value());
        const auto [L, Dvec] = ld.value();
        [[maybe_unused]] const auto end{ std::chrono::steady_clock::now() };
        LOG_WARN("LD (Rtklib): L^T * D * L");
        LOG_INFO("Elapsed time: {}", std::chrono::duration<double>(end - start).count());
        Matrix D = Eigen::DiagonalMatrix<double, n>(Dvec);
        LOG_INFO("L = \n{}", L);
        LOG_INFO("D = \n{}", D);
        Matrix LTDL_minus_Q = L.transpose() * D * L - Q;
        LOG_INFO("L^T * D * L - Q = \n{}\n", LTDL_minus_Q);
        REQUIRE_THAT(LTDL_minus_Q, Catch::Matchers::WithinAbs(Matrix::Zero(n, n), 1e-10));
    }
    // Eigen is even faster for big matrices (but makes different decomposition)
    {
        [[maybe_unused]] const auto start{ std::chrono::steady_clock::now() };
        Eigen::LDLT<Matrix> ldltOfQ(Q); // See https://eigen.tuxfamily.org/dox/classEigen_1_1LDLT.html Eigen calculates P^T * L * D * L^T * P
        [[maybe_unused]] const auto end{ std::chrono::steady_clock::now() };
        REQUIRE(ldltOfQ.info() == Eigen::Success); // Success if computation was successful, NumericalIssue if the factorization failed because of a zero pivot.
        LOG_WARN("Eigen: P^T * L * D * L^T * P");
        LOG_INFO("Elapsed time: {}", std::chrono::duration<double>(end - start).count());
        auto P = Eigen::PermutationMatrix<n>(ldltOfQ.transpositionsP());
        Matrix L = ldltOfQ.matrixL();
        Matrix D = Eigen::DiagonalMatrix<double, n>(ldltOfQ.vectorD());
        LOG_INFO("L = \n{}", L);
        LOG_INFO("P^T * L = \n{}", P.transpose() * L);
        LOG_INFO("D = \n{}", D);
        Matrix PTLDLTP_minus_Q = P.transpose() * L * D * L.transpose() * P - Q;
        LOG_INFO("P^T * L * D * L^T * P - Q = \n{}\n", PTLDLTP_minus_Q);
        REQUIRE_THAT(PTLDLTP_minus_Q, Catch::Matchers::WithinAbs(Matrix::Zero(n, n), 1e-10));
    }
}

TEST_CASE("[Ambiguity] Decorrelate Ambiguities", "[Ambiguity]")
{
    auto logger = initializeTestLogger();

    constexpr size_t n = 3;
    using Matrix = Eigen::Matrix<double, n, n>;

    Matrix Qa; // Example taken from 'de Jonge 1996, eq. 3.37'
    Qa << 6.290, 5.978, 0.544,
        5.978, 6.292, 2.340,
        0.544, 2.340, 6.288;

    Eigen::LLT<Matrix> lltOfQ(Qa);            // See https://eigen.tuxfamily.org/dox/classEigen_1_1LLT.html
    REQUIRE(lltOfQ.info() == Eigen::Success); // Success if computation was successful, NumericalIssue if the matrix.appears not to be positive definite.

    Eigen::Vector3d a(5.45, 3.10, 2.97);
    LOG_INFO("a = {}", a.transpose());

    [[maybe_unused]] auto [Qz, Z, L, D, z] = Ambiguity::decorrelate_ztrafo(a, Qa).value();

    LOG_INFO("Qz = \n{}", Qz);
    LOG_INFO("Z = \n{}", Z);
    LOG_INFO("L = \n{}", L);
    LOG_INFO("D = {}", D.diagonal().transpose());
    LOG_INFO("Z^T * a = {}", (Z.transpose() * a).transpose());
    LOG_INFO("z       = {}", z.transpose());

    Matrix Qz_expected;
    // Qz_expected << 0.626, 0.230, 0.082, // This is the result given by 'de Jonge 1996, eq. 3.40', but they say they reordered it
    //     0.230, 4.476, 0.334,
    //     0.082, 0.334, 1.146;
    Qz_expected << 4.476, 0.334, 0.230,
        0.334, 1.146, 0.082,
        0.230, 0.082, 0.626;
    REQUIRE_THAT(Qz - Qz_expected, Catch::Matchers::WithinAbs(Matrix::Zero(n, n), 1e-10));

    Matrix Z_expected;
    // Z_expected << 1, -1, 0, // This is the result given by 'de Jonge 1996, eq. 3.39', but they say they reordered it
    //     -2, 3, -1,
    //     3, -3, 1;
    Z_expected << -2, 3, 1,
        3, -3, -1,
        -1, 1, 0;
    REQUIRE(Z == Z_expected);

    Matrix Dmat = Eigen::DiagonalMatrix<double, n>(D);
    Matrix LTDL = L.transpose() * Dmat * L;
    LOG_INFO("L^T D L = \n{}", LTDL);
    REQUIRE_THAT(LTDL - Qz, Catch::Matchers::WithinAbs(Matrix::Zero(n, n), 1e-10));

    Eigen::Vector3d z_expected(-4.57, 10.02, 2.35);
    // z_expected(2.35, -4.57, 10.02) // This is the result given by 'de Jonge 1996, eq. 4.32', reordered as before
    REQUIRE_THAT(z - z_expected, Catch::Matchers::WithinAbs(Eigen::Vector3d::Zero(), 1e-10));
}

TEST_CASE("[Ambiguity] Decorrelate Ambiguities Performance", "[Ambiguity]")
{
    auto logger = initializeTestLogger();

    constexpr size_t m = 30;
    constexpr size_t n = 30;
    // using Matrix = Eigen::Matrix<double, n, n>;
    using Matrix = Eigen::MatrixXd;
    using Vector = Eigen::VectorXd;

    Matrix A = Matrix::Random(m, n);
    Matrix Q = (A.transpose() * A).inverse();
    Vector a = Vector::Random(n);

    // Q << 6.290, 5.978, 0.544, // Example taken from 'de Jonge 1996, eq. 3.37'
    //     5.978, 6.292, 2.340,
    //     0.544, 2.340, 6.288;

    // LOG_INFO("A = \n{}", A);
    // LOG_INFO("Q = \n{}\n", Q);
    // LOG_INFO("a = \n{}", a);

    Eigen::LLT<Matrix> lltOfQ(Q);             // See https://eigen.tuxfamily.org/dox/classEigen_1_1LLT.html
    REQUIRE(lltOfQ.info() == Eigen::Success); // Success if computation was successful, NumericalIssue if the matrix.appears not to be positive definite.

    size_t ITERATIONS = 1000;
    [[maybe_unused]] double ztrafo = 0.0;
    for (size_t i = 0; i < ITERATIONS; i++)
    {
        [[maybe_unused]] const auto start{ std::chrono::steady_clock::now() };
        const auto [Qz, Z, L, D, z] = Ambiguity::decorrelate_ztrafo(a, Q).value();
        [[maybe_unused]] const auto end{ std::chrono::steady_clock::now() };
        ztrafo += std::chrono::duration<double>(end - start).count();

        REQUIRE_THAT(Qz, Catch::Matchers::WithinAbs(Qz, 1e-10));
        REQUIRE_THAT(Z, Catch::Matchers::WithinAbs(Z, 1e-10));
        REQUIRE_THAT(L, Catch::Matchers::WithinAbs(L, 1e-10));
        REQUIRE_THAT(D, Catch::Matchers::WithinAbs(D, 1e-10));
        REQUIRE_THAT(z, Catch::Matchers::WithinAbs(z, 1e-10));
    }
    LOG_INFO("decorrelate_ztrafo: Elapsed time: {}", ztrafo / static_cast<double>(ITERATIONS));
}

} // namespace NAV::TESTS::AmbiguityTests