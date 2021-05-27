/// @file ReadAscii2Matrix.hpp
/// @brief Read function for EGM96 coefficients
/// @author M. Maier (maier@ins.uni-stuttgart.de)
/// @date 2021-05-27

#pragma once

#include "util/Eigen.hpp"

namespace NAV::utilGravity
{
/// @brief Read function for EGM96 coefficients
/// @param[in] coeffsFile Ascii file of the EGM96 coefficients
/// @return 'coeff' MatrixXd of the EGM96 coefficients
///
// /// @note See S. Gleason (2009) - GNSS Applications and Methods (Chapter 6.2.3.2 - eq. 6.16)
[[nodiscard]] Eigen::MatrixXd readAscii2Matrix();

} // namespace NAV::utilGravity
