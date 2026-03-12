// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Keys.hpp
/// @brief Barometer Keys
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2026-03-12

#pragma once

#include <array>
#include <cstdint>
#include <iostream>
#include <fmt/format.h>

namespace NAV
{

namespace Keys
{

enum BarometerModelKey : uint8_t
{
    HeightBias,  ///< Baro Height Bias
    HeightScale, ///< Baro Height Scale

    BarometerModelKey_COUNT, ///< Count
};

/// @brief All Barometer keys
template<typename StateKeyType>
constexpr std::array<StateKeyType, 2> Baro = { HeightBias, HeightScale };

} // namespace Keys

namespace MeasKeys
{

/// @brief Baro height difference measurement
struct BaroHeightDiff
{
    /// @brief Equal comparison operator
    bool operator==(const BaroHeightDiff& /* rhs */) const = default;
};

} // namespace MeasKeys

} // namespace NAV

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::Keys::BarometerModelKey& obj);

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::MeasKeys::BaroHeightDiff& obj);

#ifndef DOXYGEN_IGNORE

/// @brief Formatter
template<>
struct fmt::formatter<NAV::Keys::BarometerModelKey> : fmt::formatter<const char*>
{
    /// @brief Defines how to format structs
    /// @param[in] state Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::Keys::BarometerModelKey& state, FormatContext& ctx) const
    {
        using namespace NAV::Keys; // NOLINT(google-build-using-namespace)

        switch (state)
        {
        case HeightBias:
            return fmt::formatter<const char*>::format("HeightBias", ctx);
        case HeightScale:
            return fmt::formatter<const char*>::format("HeightScale", ctx);
        case BarometerModelKey_COUNT:
            return fmt::formatter<const char*>::format("BarometerModelKey_COUNT", ctx);
        }

        return fmt::formatter<const char*>::format("ERROR", ctx);
    }
};

/// @brief Formatter
template<>
struct fmt::formatter<NAV::MeasKeys::BaroHeightDiff> : fmt::formatter<std::string>
{
    /// @brief Defines how to format structs
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::MeasKeys::BaroHeightDiff& /* baroHgtDiff */, FormatContext& ctx) const
    {
        return fmt::formatter<std::string>::format("BaroHeightDiff", ctx);
    }
};

#endif