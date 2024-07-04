// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file QuadraticBsplines.hpp
/// @brief Constructs four overlapping qaudratic B-splines
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2024-03-28

#pragma once

#include "util/Eigen.hpp"

#include <array>

namespace NAV::BsplineKF
{
/// @brief Set the points/knots of the four B-splines
/// @param[in] ti time i to evaluate the spline at in [s]
/// @param[in] splineSpacing time difference between one and another B-spline in [s]
/// @return Vector that contains the values of each quadratic B-spline at time ti, from the stacked B-spline
std::array<double, 3> quadraticBsplines(const double& ti, const double& splineSpacing = 1.0);
} // namespace NAV::BsplineKF