// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Polynomial.hpp
/// @brief Polynomial
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-10-23

#pragma once

#include <cmath>
#include "util/Eigen.hpp"

namespace NAV
{

/// @brief Polynomial
template<typename Scalar>
class Polynomial
{
  public:
    /// @brief Constructor
    /// @param coefficients Polynomial coefficients in order a0 + a1 * x + a2 * x^2 + ...
    template<typename Derived>
    Polynomial(const Eigen::DenseBase<Derived>& coefficients) // NOLINT(google-explicit-constructor, hicpp-explicit-conversions)
        : _coefficients(coefficients)
    {}

    /// @brief Calculates the polynomial value at given x
    /// @param x X value
    [[nodiscard]] Scalar f(Scalar x) const
    {
        Scalar value = 0;
        auto xpow = static_cast<Scalar>(1.0);
        for (const auto& coeff : _coefficients)
        {
            value += coeff * xpow;
            xpow *= x;
        }
        return value;
    }

    /// @brief Calculates the derivative of the polynomial
    [[nodiscard]] Polynomial<Scalar> getDerivative() const
    {
        int degree = _coefficients.size() - 1;
        Eigen::VectorX<Scalar> derivative = Eigen::VectorX<Scalar>::Zero(degree);
        for (int i = 0; i < degree; i++)
        {
            derivative(i) = (i + 1) * _coefficients(i + 1);
        }
        return Polynomial<Scalar>(derivative);
    }

    /// @brief Returns the coefficients
    [[nodiscard]] const Eigen::VectorX<Scalar>& coeffs() const
    {
        return _coefficients;
    }

    /// @brief Format the polynomial as a string
    /// @param[in] fmt Format string for the polynomial coefficients
    /// @param[in] var Variable name to use
    [[nodiscard]] std::string toString(fmt::format_string<double> fmt = "{:.1e}", const char* var = "x") const
    {
        std::string out;
        for (int i = 0; i < _coefficients.rows(); i++)
        {
            bool positive = _coefficients(i) >= 0;
            std::string a = fmt::format(fmt, std::abs(_coefficients(i)));
            if (i == 0)
            {
                out += fmt::format("{}{}", positive ? "" : "-", a);
            }
            else
            {
                out += fmt::format(" {} {}{}{}",
                                   positive ? "+" : "-",
                                   a,
                                   var,
                                   i > 1 ? fmt::format("^{}", i) : "");
            }
        }
        return out;
    }

  private:
    /// @brief Polynomial coefficients in order a0 + a1 * x + a2 * x^2 + ...
    Eigen::VectorX<Scalar> _coefficients;
};

} // namespace NAV

#ifndef DOXYGEN_IGNORE

/// @brief Formatter for Polynomial<Scalar>
template<typename Scalar>
struct fmt::formatter<NAV::Polynomial<Scalar>>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    static constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin())
    {
        return ctx.begin();
    }

    /// @brief Defines how to format Polynomial structs
    /// @param[in] poly Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::Polynomial<Scalar>& poly, FormatContext& ctx) const -> decltype(ctx.out())
    {
        return fmt::format_to(ctx.out(), "{}", poly.toString());
    }
};

#endif