/// @file readAscii2Matrix.hpp
/// @brief Read function for EGM96 coefficients
/// @author M. Maier (maier@ins.uni-stuttgart.de)
/// @date 2021-05-27

#pragma once

#include "util/Eigen.hpp"

namespace NAV::internal
{
/// @brief Read function for EGM96 coefficients
/// @return 'coeffs' MatrixXd of the EGM96 coefficients
/// @deprecated This function is not used anymore, as the coefficients are stored in the source code
[[nodiscard]] [[deprecated]] Eigen::MatrixXd readAscii2Matrix();
} // namespace NAV::internal
