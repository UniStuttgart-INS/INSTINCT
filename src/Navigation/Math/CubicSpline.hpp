// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file CubicSpline.hpp
/// @brief Cubic Spline class
/// @author T. Hobiger (thomas.hobiger@ins.uni-stuttgart.de)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-06-06

#pragma once

#include <cstdio>
#include <cassert>
#include <cmath>
#include <vector>
#include <algorithm>
#include <sstream>
#include <string>
#include <iostream>

namespace NAV
{

/// Cubic Spline class
class CubicSpline
{
  public:
    /// @brief Boundary conditions for the spline end-points
    struct BoundaryCondition
    {
        /// @brief Boundary type
        enum BoundaryType
        {
            FirstDerivative = 1,  ///< First derivative has to match a certain value
            SecondDerivative = 2, ///< Second derivative has to match a certain value
            NotAKnot = 3,         ///< not-a-knot: At the first and last interior break, even the third derivative is continuous (up to round-off error).
        };
        BoundaryType type = SecondDerivative; ///< Type of the boundary condition
        double value{ 0.0 };                  ///< Value of the boundary condition
    };

    /// @brief Default Constructor
    CubicSpline() = default;

    /// @brief Constructor
    /// @param[in] X List of x coordinates for the spline points/knots
    /// @param[in] Y List of y coordinates for the spline points/knots
    /// @param[in] leftBoundaryCondition Boundary condition for the start knot
    /// @param[in] rightBoundaryCondition Boundary condition for the end knot
    CubicSpline(const std::vector<double>& X, const std::vector<double>& Y,
                BoundaryCondition leftBoundaryCondition = { BoundaryCondition::SecondDerivative, 0.0 },
                BoundaryCondition rightBoundaryCondition = { BoundaryCondition::SecondDerivative, 0.0 });

    /// @brief Set the boundaries conditions. Has to be called before setPoints
    /// @param[in] leftBoundaryCondition Boundary condition for the start knot
    /// @param[in] rightBoundaryCondition Boundary condition for the end knot
    void setBoundaries(BoundaryCondition leftBoundaryCondition, BoundaryCondition rightBoundaryCondition);

    /// @brief Set the points/knots of the spline and calculate the spline coefficients
    /// @param[in] x List of x coordinates of the points
    /// @param[in] y List of y coordinates of the points
    void setPoints(const std::vector<double>& x, const std::vector<double>& y);

    /// @brief Interpolates or extrapolates a value on the spline
    /// @param[in] x X coordinate to inter-/extrapolate the value for
    /// @return The y coordinate
    [[nodiscard]] double operator()(double x) const;

    /// @brief Calculates the derivative of the spline
    /// @param[in] order Order of the derivative to calculate (<= 3)
    /// @param[in] x X coordinate to calculate the derivative for
    /// @return The derivative of y up to the given order
    [[nodiscard]] double derivative(size_t order, double x) const;

  private:
    std::vector<double> vals_x; ///< x coordinates of the knots
    std::vector<double> vals_y; ///< y coordinates of the knots
    std::vector<double> coef_b; ///< Spline coefficients b
    std::vector<double> coef_c; ///< Spline coefficients c
    std::vector<double> coef_d; ///< Spline coefficients d
    double coef_c0{ 0.0 };      ///< Spline coefficient c0 for left extrapolation

    BoundaryCondition boundaryConditionLeft;  ///< Boundary condition for the left knot
    BoundaryCondition boundaryConditionRight; ///< Boundary condition for the right knot

    /// @brief Finds the closest index so that vals_x[idx] <= x (return 0 if x < vals_x[0])
    /// @param[in] x X coordinate to search the closest knot to
    /// @return Index of a knot closest to the x coordinate given
    [[nodiscard]] size_t findClosestIdx(double x) const;

    /// Sparse matrix whose non-zero entries are confined to a diagonal band, comprising the main diagonal and zero or more diagonals on either side.
    class BandMatrix
    {
      public:
        /// @brief Constructor
        /// @param[in] dim Dimension of the matrix
        /// @param[in] nUpper Amount of upper diagonals
        /// @param[in] nLower Amount of lower diagonals
        BandMatrix(size_t dim, size_t nUpper, size_t nLower);

        /// @brief Access operator i ∈ [i=0,...,dim()-1]
        double& operator()(size_t i, size_t j);
        /// @brief Access operator i ∈ [i=0,...,dim()-1]
        [[nodiscard]] const double& operator()(size_t i, size_t j) const;

        /// Solves the linear equations Ax = b for x by obtaining the LU decomposition A = LU and solving
        ///   - Ly = b for y
        ///   - Ux = y for x
        [[nodiscard]] std::vector<double> lu_solve(const std::vector<double>& b, bool is_lu_decomposed = false);

      private:
        /// Returns the matrix dimension
        [[nodiscard]] size_t dim() const;
        /// Returns the dimension of the upper band
        [[nodiscard]] size_t dimUpperBand() const;
        /// Returns the dimension of the lower band
        [[nodiscard]] size_t dimLowerBand() const;

        /// @brief Calculate the LU decomposition
        void lu_decompose();

        /// Solves the equation Lx = b for x
        [[nodiscard]] std::vector<double> l_solve(const std::vector<double>& b) const;
        /// Solves the equation Ux = b for x
        [[nodiscard]] std::vector<double> u_solve(const std::vector<double>& b) const;

        std::vector<std::vector<double>> upperBand; ///< upper diagonal band
        std::vector<std::vector<double>> lowerBand; ///< lower diagonal band
    };
};

} // namespace NAV