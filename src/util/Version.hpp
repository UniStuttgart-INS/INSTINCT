/**
 * @file Version.hpp
 * @brief Provides the version of the project
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-10
 */

#pragma once

#include <string>

/// Major Version of the Project (maximum 2 digits)
constexpr unsigned int PROJECT_VER_MAJOR = 0;
/// Minor Version of the Project (maximum 2 digits)
constexpr unsigned int PROJECT_VER_MINOR = 1;
/// Patch Version of the Project (maximum 2 digits)
constexpr unsigned int PROJECT_VER_PATCH = 0;
/// Project Version Integer
constexpr unsigned int PROJECT_VERSION()
{
    constexpr unsigned int MAJOR_FACTOR = 10000;
    constexpr unsigned int MINOR_FACTOR = 100;
    return PROJECT_VER_MAJOR * MAJOR_FACTOR + PROJECT_VER_MINOR * MINOR_FACTOR + PROJECT_VER_PATCH;
}
/// Project Version String in the form "major.minor.patch"
#define PROJECT_VERSION_STRING std::to_string(PROJECT_VER_MAJOR) + std::string(".") + std::to_string(PROJECT_VER_MINOR) + std::string(".") + std::to_string(PROJECT_VER_PATCH)