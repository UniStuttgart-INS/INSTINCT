/// @file readAscii2Matrix.hpp
/// @brief Read function for EGM96 coefficients
/// @author M. Maier (maier@ins.uni-stuttgart.de)
/// @date 2021-05-27

#pragma once

#include "util/Eigen.hpp"

namespace NAV::gravity::internal
{
/// @brief Read function for EGM96 coefficients
/// @return 'coeffs' MatrixXd of the EGM96 coefficients
[[nodiscard]] Eigen::MatrixXd readAscii2Matrix();
} // namespace NAV::gravity::internal
