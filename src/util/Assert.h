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
