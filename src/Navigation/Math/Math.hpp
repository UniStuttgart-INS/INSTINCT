// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Math.hpp
/// @brief Simple Math functions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-09-23

#pragma once

#include "util/Assert.h"
#include <cstdint>
#include <Eigen/Core>

namespace NAV::math
{

/// @brief Calculates the factorial of an unsigned integer
/// @param[in] n Unsigned integer
/// @return The factorial of 'n'
uint64_t factorial(uint64_t n);

/// @brief Calculates the skew symmetric matrix of the given vector.
///        This is needed to perform the cross product with a scalar product operation
/// @tparam Derived Derived Eigen Type
/// @param[in] a The vector
/// @return Skew symmetric matrix
/// @note See Groves (2013) equation (2.50)
template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetricMatrix(const Eigen::MatrixBase<Derived>& a)
{
    INS_ASSERT_USER_ERROR(a.cols() == 1, "Given Eigen Object must be a vector");
    INS_ASSERT_USER_ERROR(a.rows() == 3, "Given Vector must have 3 Rows");

    Eigen::Matrix<typename Derived::Scalar, 3, 3> skewMat;
    skewMat << 0, -a(2), a(1),
        a(2), 0, -a(0),
        -a(1), a(0), 0;

    return skewMat;
}

/// @brief Calculates the square of a skew symmetric matrix of the given vector.
/// @tparam Derived Derived Eigen Type
/// @param[in] a The vector
/// @return Square of skew symmetric matrix
template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetricMatrixSquared(const Eigen::MatrixBase<Derived>& a)
{
    INS_ASSERT_USER_ERROR(a.cols() == 1, "Given Eigen Object must be a vector");
    INS_ASSERT_USER_ERROR(a.rows() == 3, "Given Vector must have 3 Rows");

    Eigen::Matrix<typename Derived::Scalar, 3, 3> skewMat2;
    skewMat2 << std::pow(a(2), 2) + std::pow(a(1), 2), a(0) * a(1), a(0) * a(2),
        a(0) * a(1), std::pow(a(2), 2) + std::pow(a(0), 2), a(1) * a(2),
        a(0) * a(2), a(1) * a(2), std::pow(a(0), 2) + std::pow(a(1), 2);

    return skewMat2;
}

/// @brief Calculates the secant of a value (sec(x) = csc(pi/2 - x) = 1 / cos(x))
template<typename T,
         typename = std::enable_if_t<std::is_floating_point_v<T>>>
T sec(const T& x)
{
    return 1.0 / std::cos(x);
}

/// @brief Calculates the cosecant of a value (csc(x) = sec(pi/2 - x) = 1 / sin(x))
template<typename T,
         typename = std::enable_if_t<std::is_floating_point_v<T>>>
T csc(const T& x)
{
    return 1.0 / std::sin(x);
}

/// @brief Returns the sign of the given value
/// @param[in] val Value to get the sign from
/// @return Sign of the given value
template<typename T>
int sgn(const T& val)
{
    return (T(0) < val) - (val < T(0));
}

/// @brief Find (L^T D L)-decomposition of Q-matrix via outer product method
/// @param[in] Qmatrix Symmetric positive definite matrix to be factored
/// @return L - Factor matrix (strict lower triangular)
/// @return D - Diagonal matrix
/// @note See 1996 P. de Jonge and C. Tiberius - The LAMBDA method for integer ambiguity estimation: implementation aspects, algorithm FMFAC5
/// @attention Consider using NAV::math::LtDLdecomp_choleskyFact() because it is faster by up to a factor 10
template<typename Derived>
std::pair<Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>,
          Eigen::DiagonalMatrix<typename Derived::Scalar, Derived::RowsAtCompileTime>>
    LtDLdecomp_outerProduct(const Eigen::MatrixBase<Derived>& Qmatrix)
{
    using Eigen::seq;

    auto n = Qmatrix.rows();
    Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> Q = Qmatrix.template triangularView<Eigen::Lower>();
    Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> L;
    Eigen::DiagonalMatrix<typename Derived::Scalar, Derived::RowsAtCompileTime> D;

    if constexpr (Derived::RowsAtCompileTime == Eigen::Dynamic)
    {
        L = Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(n, n);
        D.setZero(n);
    }
    else
    {
        L = Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>::Zero();
        D.setZero(n);
    }

    for (Eigen::Index i = n - 1; i >= 0; i--)
    {
        D.diagonal()(i) = Q(i, i);
        L(i, seq(0, i)) = Q(i, seq(0, i)) / std::sqrt(Q(i, i));

        for (Eigen::Index j = 0; j <= i - 1; j++)
        {
            Q(j, seq(0, j)) -= L(i, seq(0, j)) * L(i, j);
        }
        L(i, seq(0, i)) /= L(i, i);
    }

    return { L, D };
}

/// @brief Find (L^T D L)-decomposition of Q-matrix via a backward Cholesky factorization in a bordering method formulation
/// @param[in] Q Symmetric positive definite matrix to be factored
/// @return L - Factor matrix (strict lower triangular)
/// @return D - Diagonal matrix
/// @note See 1996 P. de Jonge and C. Tiberius - The LAMBDA method for integer ambiguity estimation: implementation aspects, algorithm FMFAC6
template<typename Derived>
std::pair<Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>,
          Eigen::DiagonalMatrix<typename Derived::Scalar, Derived::RowsAtCompileTime>>
    LtDLdecomp_choleskyFact(const Eigen::MatrixBase<Derived>& Q)
{
    using Eigen::seq;

    auto n = Q.rows();
    Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime> L = Q.template triangularView<Eigen::Lower>();
    Eigen::DiagonalMatrix<typename Derived::Scalar, Derived::RowsAtCompileTime> D;
    double cmin = 1;

    if constexpr (Derived::RowsAtCompileTime == Eigen::Dynamic) { D.setZero(n); }
    else { D.setZero(n); }

    for (Eigen::Index j = n - 1; j >= 0; j--)
    {
        for (Eigen::Index i = n - 1; i >= j + 1; i--)
        {
            L(i, j) = (L(i, j) - L(seq(i + 1, n - 1), j).dot(L(seq(i + 1, n - 1), i))) / L(i, i);
        }
        double t = L(j, j) - L(seq(j + 1, n - 1), j).dot(L(seq(j + 1, n - 1), j));
        double c = t / L(j, j);
        if (c < cmin)
        {
            cmin = c;
        }
        L(j, j) = std::sqrt(t);
    }
    for (Eigen::Index i = 0; i < n; i++)
    {
        L.row(i).leftCols(i) /= L(i, i);
        D.diagonal()(i) = std::pow(L(i, i), 2.0);
        L(i, i) = 1;
    }

    return { L, D };
}

} // namespace NAV::math
