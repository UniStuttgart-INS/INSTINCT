/// @file AssociatedLegendre.hpp
/// @brief Legendre Polynomials for EGM
/// @author M. Maier (maier@ins.uni-stuttgart.de)
/// @date 2021-05-21

#pragma once

#include "util/Eigen.hpp"

namespace NAV::utilGravity
{
/// @brief Calculates the associated Legendre Polynomials necessary for the EGM96
/// @param[in] degreeN of Legendre polynomial
/// @param[in] x data points
/// @return Associated Legendre Polynomial Parameters as P2 << P, Pd (derivative)
///
// /// @note See S. Gleason (2009) - GNSS Applications and Methods (Chapter 6.2.3.2 - eq. 6.16)
[[nodiscard]] std::pair<Eigen::MatrixXd, Eigen::MatrixXd> associatedLegendre(int degreeN, double x);

} // namespace NAV::utilGravity
