/// @file AssociatedLegendre.hpp
/// @brief Legendre Polynomials for EGM
/// @author M. Maier (maier@ins.uni-stuttgart.de)
/// @date 2021-05-21

#pragma once

namespace NAV::legendre
{
/// @brief Calculates the associated Legendre Polynomials necessary for the EGM96
// /// @param[in] latitude Latitude where to calculate the gravity for
// /// @param[in] altitude Altitude where to calculate the gravity for
/// @return Associated Legendre Polynomial Parameters
///
// /// @note See S. Gleason (2009) - GNSS Applications and Methods (Chapter 6.2.3.2 - eq. 6.16)
[[nodiscard]] double associatedLegendre(int N, double x);

} // namespace NAV::legendre
