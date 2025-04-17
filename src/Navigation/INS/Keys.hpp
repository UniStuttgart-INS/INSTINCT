// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Keys.hpp
/// @brief Inertial Navigation System Keys
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2025-03-12

#pragma once

#include <array>
#include <cmath>
#include <iostream>
#include <fmt/format.h>

namespace NAV::Keys
{

/// Keys used in the model
enum InertialNavigationKey : uint8_t
{
    AccelBiasX, ///< Accelerometer bias X [m/s^2]
    AccelBiasY, ///< Accelerometer bias Y [m/s^2]
    AccelBiasZ, ///< Accelerometer bias Z [m/s^2]
    GyroBiasX,  ///< Gyroscope bias X [rad/s]
    GyroBiasY,  ///< Gyroscope bias Y [rad/s]
    GyroBiasZ,  ///< Gyroscope bias Z [rad/s]

    AccelScaleFactorX, ///< Accelerometer scale factor X [-]
    AccelScaleFactorY, ///< Accelerometer scale factor Y [-]
    AccelScaleFactorZ, ///< Accelerometer scale factor Z [-]
    GyroScaleFactorX,  ///< Gyroscope scale factor X [-]
    GyroScaleFactorY,  ///< Gyroscope scale factor Y [-]
    GyroScaleFactorZ,  ///< Gyroscope scale factor Z [-]

    AccelMisalignmentQ1, ///< Accelerometer misalignment quaternion x: Coefficient of i
    AccelMisalignmentQ2, ///< Accelerometer misalignment quaternion y: Coefficient of j
    AccelMisalignmentQ3, ///< Accelerometer misalignment quaternion z: Coefficient of k
    AccelMisalignmentQ0, ///< Accelerometer misalignment quaternion w: Real (scalar) part of the Quaternion
    GyroMisalignmentQ1,  ///< Gyroscope misalignment quaternion x: Coefficient of i
    GyroMisalignmentQ2,  ///< Gyroscope misalignment quaternion y: Coefficient of j
    GyroMisalignmentQ3,  ///< Gyroscope misalignment quaternion z: Coefficient of k
    GyroMisalignmentQ0,  ///< Gyroscope misalignment quaternion w: Real (scalar) part of the Quaternion

    InertialNavigationKey_COUNT, ///< Count
};

/// @brief All accelerometer bias keys
template<typename StateKeyType>
constexpr std::array<StateKeyType, 3> AccelBias = { Keys::AccelBiasX, Keys::AccelBiasY, Keys::AccelBiasZ };
/// @brief All gyroscope bias keys
template<typename StateKeyType>
constexpr std::array<StateKeyType, 3> GyroBias = { Keys::GyroBiasX, Keys::GyroBiasY, Keys::GyroBiasZ };

/// @brief All accelerometer scale factor keys
template<typename StateKeyType>
constexpr std::array<StateKeyType, 3> AccelScaleFactor = { Keys::AccelScaleFactorX, Keys::AccelScaleFactorY, Keys::AccelScaleFactorZ };
/// @brief All gyroscope scale factor keys
template<typename StateKeyType>
constexpr std::array<StateKeyType, 3> GyroScaleFactor = { Keys::GyroScaleFactorX, Keys::GyroScaleFactorY, Keys::GyroScaleFactorZ };

/// @brief All accelerometer misalignment quaternion keys
template<typename StateKeyType>
constexpr std::array<StateKeyType, 4> AccelMisalignment = { Keys::AccelMisalignmentQ1, Keys::AccelMisalignmentQ2, Keys::AccelMisalignmentQ3, Keys::AccelMisalignmentQ0 };
/// @brief All gyroscope misalignment quaternion keys
template<typename StateKeyType>
constexpr std::array<StateKeyType, 4> GyroMisalignment = { Keys::GyroMisalignmentQ1, Keys::GyroMisalignmentQ2, Keys::GyroMisalignmentQ3, Keys::GyroMisalignmentQ0 };

} // namespace NAV::Keys

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::Keys::InertialNavigationKey& obj);

#ifndef DOXYGEN_IGNORE

/// @brief Formatter
template<>
struct fmt::formatter<NAV::Keys::InertialNavigationKey> : fmt::formatter<const char*>
{
    /// @brief Defines how to format structs
    /// @param[in] state Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::Keys::InertialNavigationKey& state, FormatContext& ctx) const
    {
        using namespace NAV::Keys; // NOLINT(google-build-using-namespace)

        switch (state)
        {
        case AccelBiasX:
            return fmt::formatter<const char*>::format("AccelBiasX", ctx);
        case AccelBiasY:
            return fmt::formatter<const char*>::format("AccelBiasY", ctx);
        case AccelBiasZ:
            return fmt::formatter<const char*>::format("AccelBiasZ", ctx);
        case GyroBiasX:
            return fmt::formatter<const char*>::format("GyroBiasX", ctx);
        case GyroBiasY:
            return fmt::formatter<const char*>::format("GyroBiasY", ctx);
        case GyroBiasZ:
            return fmt::formatter<const char*>::format("GyroBiasZ", ctx);
        case AccelScaleFactorX:
            return fmt::formatter<const char*>::format("AccelScaleFactorX", ctx);
        case AccelScaleFactorY:
            return fmt::formatter<const char*>::format("AccelScaleFactorY", ctx);
        case AccelScaleFactorZ:
            return fmt::formatter<const char*>::format("AccelScaleFactorZ", ctx);
        case GyroScaleFactorX:
            return fmt::formatter<const char*>::format("GyroScaleFactorX", ctx);
        case GyroScaleFactorY:
            return fmt::formatter<const char*>::format("GyroScaleFactorY", ctx);
        case GyroScaleFactorZ:
            return fmt::formatter<const char*>::format("GyroScaleFactorZ", ctx);
        case AccelMisalignmentQ1:
            return fmt::formatter<const char*>::format("AccelMisalignmentQ1", ctx);
        case AccelMisalignmentQ2:
            return fmt::formatter<const char*>::format("AccelMisalignmentQ2", ctx);
        case AccelMisalignmentQ3:
            return fmt::formatter<const char*>::format("AccelMisalignmentQ3", ctx);
        case AccelMisalignmentQ0:
            return fmt::formatter<const char*>::format("AccelMisalignmentQ0", ctx);
        case GyroMisalignmentQ1:
            return fmt::formatter<const char*>::format("GyroMisalignmentQ1", ctx);
        case GyroMisalignmentQ2:
            return fmt::formatter<const char*>::format("GyroMisalignmentQ2", ctx);
        case GyroMisalignmentQ3:
            return fmt::formatter<const char*>::format("GyroMisalignmentQ3", ctx);
        case GyroMisalignmentQ0:
            return fmt::formatter<const char*>::format("GyroMisalignmentQ0", ctx);
        case InertialNavigationKey_COUNT:
            return fmt::formatter<const char*>::format("InertialNavigationKey_COUNT", ctx);
        }

        return fmt::formatter<const char*>::format("ERROR", ctx);
    }
};

#endif