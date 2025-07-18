// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Decorrelation.hpp
/// @brief Ambiguity decorrelation algorithms
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-04

#pragma once

#include <tuple>

#include "Navigation/Math/Math.hpp"
#include "util/Eigen.hpp"
#include "util/Logger.hpp"
#include "util/Assert.h"
#include <Eigen/src/Core/ArithmeticSequence.h>
#include <Eigen/src/Core/MatrixBase.h>
#include <Eigen/src/Core/util/Meta.h>

namespace NAV::Ambiguity
{

namespace internal
{

/// @brief \cite chang2005 Chang 2005, Integer Gauss Transformations algorithm
/// @param[in, out] L (L^T * D * L) decomposition of Q_z
/// @param[in] i Row index
/// @param[in] j Col index
/// @param[in, out] a Float ambiguity vector [cycles]
/// @param[in, out] Z Decorrelation transformation matrix
/// @param[in] n Dimension
template<typename DerivedL, typename DerivedZ, typename DerivedA>
void gauss(Eigen::MatrixBase<DerivedL>& L, Eigen::Index i, Eigen::Index j, Eigen::MatrixBase<DerivedA>& a, Eigen::MatrixBase<DerivedZ>& Z, Eigen::Index n)
{
    using Eigen::seq;

    auto mu = std::round(L(i, j));
    if (mu != 0)
    {
        L(seq(i, n - 1), j) -= mu * L(seq(i, n - 1), i);
        Z(seq(0, n - 1), j) -= mu * Z(seq(0, n - 1), i);
        a(j) -= mu * a(i);
    }
}

/// @brief \cite chang2005 Chang 2005, Permutations algorithm
/// @param[in, out] L (L^T * D * L) decomposition of Q_z
/// @param[in, out] D (L^T * D * L) decomposition of Q_z
/// @param[in] k Index
/// @param[in] delta Delta parameter
/// @param[in, out] a Float ambiguity vector [cycles]
/// @param[in, out] Z Z trafo
/// @param[in] n Dimension
template<typename DerivedL, typename DerivedD, typename DerivedA, typename DerivedZ>
void permute(Eigen::MatrixBase<DerivedL>& L, Eigen::MatrixBase<DerivedD>& D, Eigen::Index k, double delta, Eigen::MatrixBase<DerivedA>& a, Eigen::MatrixBase<DerivedZ>& Z, Eigen::Index n)
{
    using Eigen::seq;

    auto eta = D(k) / delta;
    auto lambda = D(k + 1) * L(k + 1, k) / delta;
    D(k) = eta * D(k + 1);
    D(k + 1) = delta;

    L(seq(k, k + 1), seq(0, k - 1)) = (Eigen::MatrixXd(2, 2) << -L(k + 1, k), 1,
                                       eta, lambda)
                                          .finished()
                                      * L(seq(k, k + 1), seq(0, k - 1));
    L(k + 1, k) = lambda;
    L(seq(k + 2, n - 1), k).swap(L(seq(k + 2, n - 1), k + 1));
    Z.col(k).swap(Z.col(k + 1));
    std::swap(a(k), a(k + 1));
}

} // namespace internal

/// @brief Decorrelates the ambiguities
/// @param[in] a Float ambiguity vector [cycles]
/// @param[in] Q Variance/covariance matrix of the ambiguities
/// @return [Qz, Z, L, D, z] L, D are a (L^T * D * L) decomposition of Q_z
/// @note See \cite chang2005 Chang 2005, Reduction algorithm
template<typename DerivedA, typename DerivedQ>
[[nodiscard]] std::optional<std::tuple<typename DerivedQ::PlainObject,  // Qz
                                       typename DerivedQ::PlainObject,  // Z
                                       typename DerivedQ::PlainObject,  // L
                                       typename DerivedA::PlainObject,  // D
                                       typename DerivedA::PlainObject>> // z
    decorrelate_ztrafo(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedQ>& Q)
{
    static_assert(DerivedA::ColsAtCompileTime == Eigen::Dynamic || DerivedA::ColsAtCompileTime == 1);
    INS_ASSERT_USER_ERROR(a.cols() == 1, "Parameter 'a' has to be a vector");

    using Eigen::seq;

    auto n = a.rows(); // Amount of ambiguities
    auto ltdl_decomp = math::LtDLdecomp_choleskyFact(Q);
    if (!ltdl_decomp) { return {}; }
    auto [L, D] = *ltdl_decomp;

    typename DerivedQ::PlainObject Z;
    if constexpr (DerivedQ::RowsAtCompileTime == Eigen::Dynamic) { Z.setIdentity(n, n); }
    else { Z.setIdentity(); }

    typename DerivedA::PlainObject z = a;

    auto k = n - 2;
    auto k1 = n - 2;

    while (k >= 0)
    {
        if (k <= k1)
        {
            for (auto i = k + 1; i < n; i++)
            {
                internal::gauss(L, i, k, z, Z, n);
            }
        }
        auto delta = D(k) + std::pow(L(k + 1, k), 2) * D(k + 1);
        if (delta + 1e-6 < D(k + 1))
        {
            internal::permute(L, D, k, delta, z, Z, n);
            k1 = k;
            k = n - 2;
        }
        else
        {
            k--;
        }
    }

    typename DerivedQ::PlainObject Qz = Z.transpose() * Q * Z;

    return std::make_tuple(Qz, Z, L, D, z);
}

} // namespace NAV::Ambiguity