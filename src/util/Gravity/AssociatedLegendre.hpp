/// @file AssociatedLegendre.hpp
/// @brief Legendre Polynomials for EGM
/// @author M. Maier (maier@ins.uni-stuttgart.de)
/// @date 2021-05-21

#pragma once

#include "util/Eigen.hpp"

namespace NAV::util::gravity
{
/// @brief Calculates the associated Legendre Polynomials necessary for the EGM96
/// @param[in] degreeN of Legendre polynomial
/// @param[in] x sampling points of the polynomials
/// @return Associated Legendre Polynomial Parameters as P2 << P, Pd (derivative)
///
/// @note See https://github.com/lukasbystricky/SpaceSimulator/blob/master/Utilities/Math/associated_legendre.m (last accessed on June 24th, 2021)
[[nodiscard]] std::pair<Eigen::MatrixXd, Eigen::MatrixXd> associatedLegendre(int degreeN, double x);

} // namespace NAV::util::gravity
