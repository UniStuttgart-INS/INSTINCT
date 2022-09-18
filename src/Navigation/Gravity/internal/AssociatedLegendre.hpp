// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file AssociatedLegendre.hpp
/// @brief Associated Legendre Polynomials for EGM96
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2021-05-21

#pragma once

#include "util/Eigen.hpp"

namespace NAV::internal
{
/// @brief Calculates the associated Legendre Polynomial coefficients necessary for the EGM96
/// @param[in] theta Elevation angle (spherical coordinates) [rad]
/// @param[in] ndegree Degree of associated Legendre polynomials
/// @return Matrix of associated Legendre polynomial coefficients P and its derivative Pd
///
/// @note See 'GUT User Guide' (2018) chapter 4.2, equations (4.2.2), (4.2.3) and (4.2.6)
[[nodiscard]] std::pair<Eigen::MatrixXd, Eigen::MatrixXd> associatedLegendre(double theta, size_t ndegree = 10);

} // namespace NAV::internal
