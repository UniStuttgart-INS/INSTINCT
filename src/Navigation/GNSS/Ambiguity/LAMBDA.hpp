// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file LAMBDA.hpp
/// @brief Functions related to the LAMBDA algorithm
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-09-04

#pragma once

#include <tuple>

#include "Navigation/Math/Math.hpp"
#include "util/Eigen.hpp"
#include "util/Assert.h"

namespace NAV::Ambiguity::LAMBDA
{

/// @brief Decorrelates the ambiguities
///
/// The returned Z matrix is defined as
/// \anchor eq-Amb-z \f{equation}{ \label{eq:eq-Amb-z}
///   z = Z^T a,\quad \hat{z} = Z^T \hat{a}
/// \f}
/// \anchor eq-Amb-Q \f{equation}{ \label{eq:eq-Amb-Q}
///   Q_{\hat{z}} = Z^T Q_{\hat{a}} Z
/// \f}
///
/// @param[in] a Ambiguity vector [cycles]
/// @param[in] Q Variance/covariance matrix of the ambiguities
/// @return [Qz, Z, L, D, z]
/// @note See \cite deJonge1996 de Jonge 1996, Algorithm SRC1 and ZTRAN
template<typename DerivedA, typename DerivedQ>
[[nodiscard]] std::tuple<typename DerivedQ::PlainObject,                                        // Qz
                         typename DerivedQ::PlainObject,                                        // Z
                         typename DerivedQ::PlainObject,                                        // L
                         Eigen::Vector<typename DerivedQ::Scalar, DerivedQ::RowsAtCompileTime>, // D
                         typename DerivedA::PlainObject>                                        // z
    decorrelate(const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedQ>& Q)
{
    static_assert(DerivedA::ColsAtCompileTime == Eigen::Dynamic || DerivedA::ColsAtCompileTime == 1);
    INS_ASSERT_USER_ERROR(a.cols() == 1, "Parameter 'a' has to be a vector");

    using Eigen::seq;

    auto n = a.rows(); // Amount of ambiguities
    auto [L, D] = math::LtDLdecomp_choleskyFact(Q);

    typename DerivedQ::PlainObject Z;
    if constexpr (DerivedQ::RowsAtCompileTime == Eigen::Dynamic) { Z.setIdentity(n, n); }
    else { Z.setIdentity(); }

    typename DerivedA::PlainObject z = a;

    auto i1 = n - 2; // keeps track what the last column is that fulfills the criterion dn <= ... <= d1 (most precise ambiguity at position n)

    for (auto i = n - 2; i >= 0; i--)
    {
        if (i <= i1)
        {
            // Apply algorithm ZTRAN to column i
            for (auto j = i + 1; j < n; j++)
            {
                auto mu = std::round(L(j, i));
                if (mu != 0)
                {
                    L(seq(j, n - 1), i) -= mu * L(seq(j, n - 1), j);
                    Z(seq(0, n - 1), i) -= mu * Z(seq(0, n - 1), j);
                    // Z(seq(0, n - 1), j) += mu * Z(seq(0, n - 1), i); // Z^-T can be directly computed with this
                    z(i) -= mu * z(j);
                }
            }
        }

        double delta = D(i) + std::pow(L(i + 1, i), 2) * D(i + 1);
        if (delta < D(i + 1))
        {
            auto lambda3 = D(i + 1) * L(i + 1, i) / delta;
            auto eta = D(i) / delta;
            D(i) = eta * D(i + 1);
            D(i + 1) = delta;

            // for (int j = 0; j <= i - 1; j++)
            auto lambda1 = L(i, seq(0, i - 1));
            auto lambda2 = L(i + 1, seq(0, i - 1));
            auto L_ij = (-L(i + 1, i) * lambda1 + lambda2).eval();
            L(i + 1, seq(0, i - 1)) = eta * lambda1 + lambda3 * lambda2;
            L(i, seq(0, i - 1)) = L_ij;

            L(i + 1, i) = lambda3;
            L(seq(i + 2, n - 1), i).swap(L(seq(i + 2, n - 1), i + 1));
            Z(seq(0, n - 1), i).swap(Z(seq(0, n - 1), i + 1));
            std::swap(z(i), z(i + 1));

            i1 = i;
            i = n - 1; // Restart the for loop
        }
    }

    typename DerivedQ::PlainObject Qz = Z.transpose() * Q * Z;

    return { Qz, Z, L, D, z };
}

} // namespace NAV::Ambiguity::LAMBDA