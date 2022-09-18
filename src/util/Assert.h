// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Assert.h
/// @brief Assertion helpers
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-04-22

#pragma once

#include <cassert>

/// Assert function wrapper
#define INS_ASSERT(_EXPR) assert(_EXPR)
/// Assert function with message
#define INS_ASSERT_USER_ERROR(_EXP, _MSG) INS_ASSERT((_EXP) && _MSG)
