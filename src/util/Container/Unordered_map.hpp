// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Unordered_map.hpp
/// @brief Unordered map wrapper
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-07-28

#pragma once

#ifdef protected
    #define protectedTmp
    #undef protected
    #undef private
#endif
#include <ankerl/unordered_dense.h>
#ifdef protectedTmp
    #define protected public
    #define private public
    #undef protectedTmp
#endif

/// @brief Unordered map type
/// @tparam Key Key
/// @tparam T Value
template<
    class Key,
    class T>
using unordered_map = ankerl::unordered_dense::map<Key, T>;
