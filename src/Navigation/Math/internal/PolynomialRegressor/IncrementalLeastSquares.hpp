// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file IncrementalLeastSquares.hpp
/// @brief Incremental Least Squares Curve Fit
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-10-26
/// @note See https://blog.demofox.org/2016/12/22/incremental-least-squares-curve-fitting/

#pragma once

#include <cstddef>
#include <vector>
#include <cmath>
#include "util/Assert.h"
#include "util/Eigen.hpp"

namespace NAV
{

/// @brief Incremental Least Squares Curve Fitting
/// @tparam Scalar Data type to store
template<typename Scalar>
class IncrementalLeastSquares
{
  public:
    /// @brief Constructor
    /// @param[in] polynomialDegree Degree of the polynomial to fit
    explicit IncrementalLeastSquares(size_t polynomialDegree)
    {
        setPolynomialDegree(polynomialDegree);
    }

    /// @brief Set the Polynomial Degree
    /// @param[in] polynomialDegree Degree of the polynomial to fit
    void setPolynomialDegree(size_t polynomialDegree)
    {
        _polyDegree = polynomialDegree;
        _summedPowersX = Eigen::VectorX<Scalar>::Zero((static_cast<Eigen::Index>(polynomialDegree) + 1) * 2 - 1);
        _summedPowersXTimesY = Eigen::VectorX<Scalar>::Zero(static_cast<Eigen::Index>(polynomialDegree) + 1);
    }

    /// @brief Add a data point to the polynomial
    /// @param[in] x X Value
    /// @param[in] y Y Value
    void addDataPoint(const Scalar& x, const Scalar& y)
    {
        auto xpow = static_cast<Scalar>(1.0);
        for (auto& sum : _summedPowersX)
        {
            sum += xpow;
            xpow *= x;
        }

        xpow = static_cast<Scalar>(1.0);
        for (auto& sum : _summedPowersXTimesY)
        {
            sum += xpow * y;
            xpow *= x;
        }
    }

    /// @brief Removes a data point from the polynomial fit
    /// @param[in] x X Value
    /// @param[in] y Y Value
    void removeDataPoint(const Scalar& x, const Scalar& y)
    {
        auto xpow = static_cast<Scalar>(1.0);
        for (auto& sum : _summedPowersX)
        {
            sum -= xpow;
            xpow *= x;
        }

        xpow = static_cast<Scalar>(1.0);
        for (auto& sum : _summedPowersXTimesY)
        {
            sum -= xpow * y;
            xpow *= x;
        }
    }

    /// @brief Reset the saved data
    void reset()
    {
        _summedPowersX.setZero();
        _summedPowersXTimesY.setZero();
    }

    /// @brief Calculates the polynomial coefficients in order a0 + a1 * x + a2 * x^2 + ...
    [[nodiscard]] Eigen::VectorX<Scalar> calcCoefficients() const
    {
        if (_summedPowersX(0) == 0) { return {}; }

        auto effectiveDegree = static_cast<Eigen::Index>(std::min(_polyDegree, static_cast<size_t>(_summedPowersX(0)) - 1));

        // A^T * A matrix
        Eigen::MatrixXd ATA(effectiveDegree + 1, effectiveDegree + 1);
        for (Eigen::Index i = 0; i < effectiveDegree + 1; ++i)
        {
            ATA.row(i) = _summedPowersX.segment(i, effectiveDegree + 1);
        }

        return ATA.inverse().topRows(effectiveDegree + 1) * _summedPowersXTimesY.head(effectiveDegree + 1);
    }

  private:
    size_t _polyDegree = 2;                      ///< Polynomial degree to fit
    Eigen::VectorX<Scalar> _summedPowersX;       ///< Sum{x^2}. (DEGREE + 1) * 2 - 1 values
    Eigen::VectorX<Scalar> _summedPowersXTimesY; ///< Sum{x^2 * y}. DEGREE + 1 values
};

} // namespace NAV