/**
 * @file Common.hpp
 * @brief Common things for the Project
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-10
 */

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

namespace NAV
{
/// Enum providing return values indicating the type of failure
enum NavStatus
{
    NAV_OK,                          ///< Everything OK
    NAV_ERROR,                       ///< A general fatal problem occurred
    NAV_ERROR_COULD_NOT_CONNECT,     ///< Could not connect to a sensor
    NAV_ERROR_CONFIGURATION_FAULT,   ///< A faulty configuration was specified
    NAV_ERROR_COULD_NOT_OPEN_FILE,   ///< Could not open a file
    NAV_ERROR_NOT_INITIALIZED,       ///< Fatal feature requested without initializing first
    NAV_WARNING_ALREADY_INITIALIZED, ///< Already initialized
    NAV_WARNING_NOT_INITIALIZED,     ///< Non fatal feature requested without initializing first
};

} // namespace NAV