// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file ViennaMappingFunction.hpp
/// @brief Gridded Vienna Mapping Function
/// @author Rui Wang (rui.wang@ins.uni-stuttgart.de)
/// @date 2023-02-22

#pragma once

namespace NAV
{
/// @brief mapping functions with height correction
/// @param[in] ah hydrostatic coefficient a
/// @param[in] dmjd modified julian date
/// @param[in] dlat ellipsoidal latitude in radians
/// @param[in] ht ellipsoidal height in meter
/// @param[in] zd zenith distance in radians
/// @return vmf1h: hydrostatic mapping function
double vmf1h(const double& ah, const double& dmjd,
             const double& dlat, const double& ht, const double& zd);

/// @brief mapping functions with height correction
/// @param[in] aw wet coefficient a
/// @param[in] zd zenith distance in radians
/// @return vmf1w: wet mapping function
double vmf1w(const double& aw, const double& zd);

} // namespace NAV