/// @file Debug.hpp
/// @brief Debug Utilities for the Project
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-05-12

#pragma once

#include <cassert>

#define ASSERT1(cond) assert(cond)
#define ASSERT2(cond, message) assert(message&& cond)

#define GET_ASSERT_MACRO(_1, _2, NAME, ...) NAME
#define ASSERT(...)                                 \
    GET_ASSERT_MACRO(__VA_ARGS__, ASSERT2, ASSERT1) \
    (__VA_ARGS__)

#ifndef NDEBUG
    #ifdef _MSC_VER
        #define DEBUG_BREAK __debugbreak()
    #else
        #define DEBUG_BREAK __builtin_trap()
    #endif
#endif
