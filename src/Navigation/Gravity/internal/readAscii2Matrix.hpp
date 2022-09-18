// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

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
