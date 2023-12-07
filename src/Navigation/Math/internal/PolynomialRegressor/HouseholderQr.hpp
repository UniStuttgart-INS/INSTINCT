// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file HouseholderQr.hpp
/// @brief Least Squares Curve Fit
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-10-26

#pragma once

#include <cstddef>
#include <vector>
#include <cmath>
#include "util/Assert.h"
#include "util/Eigen.hpp"

namespace NAV
{

/// @brief Householder QR decomposition Curve Fitting
/// @tparam Scalar Data type to store
template<typename Scalar>
class HouseholderQr
{
  public:
    /// @brief Constructor
    HouseholderQr() = delete;

    /// @brief Calculates the polynomial coefficients in order a0 + a1 * x + a2 * x^2 + ...
    /// @param[in] x X data to fit
    /// @param[in] y Y data to fit
    /// @param[in] polynomialDegree Polynomial degree to fit
    template<typename DerivedX, typename DerivedY>
    [[nodiscard]] static Eigen::VectorX<Scalar> calcCoefficients(const Eigen::MatrixBase<DerivedX>& x, const Eigen::MatrixBase<DerivedY>& y, size_t polynomialDegree = 2)
    {
        auto effectiveDegree = static_cast<Eigen::Index>(std::min(polynomialDegree, static_cast<size_t>(x.rows()) - 1));

        auto n = x.size();
        Eigen::MatrixX<Scalar> H = Eigen::MatrixX<Scalar>(n, effectiveDegree + 1);
        Eigen::ArrayX<Scalar> xpow = Eigen::ArrayX<Scalar>::Ones(x.rows());
        for (int i = 0; i < effectiveDegree + 1; i++)
        {
            H.col(i) = xpow.matrix();
            xpow *= x.array();
        }

        return H.householderQr().solve(y);
    }
};

} // namespace NAV