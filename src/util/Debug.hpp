/// @file Debug.hpp
/// @brief Debug Utilities for the Project
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-05-12

#pragma once

#include <cassert>

/// @brief Assert the condition in debug builds
#define ASSERT1(cond) assert(cond)
/// @brief Assert the condition in debug builds and shows the message if it fails
#define ASSERT2(cond, message) assert(message&& cond)

/// @brief Helper Macro to get the Assert macro
#define GET_ASSERT_MACRO(_1, _2, NAME, ...) NAME
/// @brief Assert the condition in debug builds and shows the message on failure if provided
#define ASSERT(...)                                 \
    GET_ASSERT_MACRO(__VA_ARGS__, ASSERT2, ASSERT1) \
    (__VA_ARGS__)

#ifndef NDEBUG
    #ifdef _MSC_VER
        /// @brief Sets a debug breakpoint
        #define DEBUG_BREAK __debugbreak()
    #else
        /// @brief Sets a debug breakpoint
        #define DEBUG_BREAK __builtin_trap()
    #endif
#endif
