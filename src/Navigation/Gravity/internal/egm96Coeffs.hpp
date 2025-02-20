// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @brief EGM96 coefficients
/// @author M. Maier (maier@ins.uni-stuttgart.de)
/// @author T. Topp (topp@ins.uni-stuttgart.de)

#pragma once

#include <array>

namespace NAV::internal
{

/// @brief EGM96 coefficients
extern std::array<std::array<double, 6>, 65338> egm96Coeffs;

} // namespace NAV::internal