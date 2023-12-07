// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SatelliteSystem.hpp
/// @brief GNSS Satellite System
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-04-22

#pragma once

#include <string>
#include <vector>
#include <fmt/format.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace

#include "Navigation/Time/TimeSystem.hpp"

namespace NAV
{

/// @brief Satellite System enumeration
enum SatelliteSystem_ : uint64_t
{ // clang-format off
    SatSys_None = 0x0000'0000'0000'0000, ///< No Satellite system
    GPS         = 0x0000'0000'0000'00FF, ///< Global Positioning System
    GAL         = 0x0000'0000'0000'FF00, ///< Galileo
    GLO         = 0x0000'0000'00FF'0000, ///< Globalnaja nawigazionnaja sputnikowaja sistema (GLONASS)
    BDS         = 0x0000'0000'FF00'0000, ///< Beidou
    QZSS        = 0x0000'00FF'0000'0000, ///< Quasi-Zenith Satellite System
    IRNSS       = 0x0000'FF00'0000'0000, ///< Indian Regional Navigation Satellite System
    SBAS        = 0x00FF'0000'0000'0000, ///< Satellite Based Augmentation System
}; // clang-format on

/// @brief Satellite System type
struct SatelliteSystem
{
    /// @brief Default Constructor
    constexpr SatelliteSystem() = default;

    /// @brief Implicit Constructor from Value type
    /// @param[in] type Value type to construct from
    constexpr SatelliteSystem(SatelliteSystem_ type) // NOLINT(hicpp-explicit-conversions, google-explicit-constructor)
        : value(type)
    {}

    /// @brief Construct new object from std::string
    /// @param[in] typeString String representation of the satellite system
    static SatelliteSystem fromString(const std::string& typeString);

    /// @brief Construct new object from char
    /// @param[in] typeChar Character representation of the satellite system
    static SatelliteSystem fromChar(char typeChar);

    /// @brief Constructs a new object from continuous enumeration
    /// @param[in] enumeration Continuous enumeration of the satellite system
    static SatelliteSystem fromEnum(size_t enumeration);

    /// @brief Assignment operator from Value type
    /// @param[in] v Value type to construct from
    /// @return The Type type from the value type
    constexpr SatelliteSystem& operator=(SatelliteSystem_ v)
    {
        value = v;
        return *this;
    }

    friend constexpr bool operator==(const SatelliteSystem& lhs, const SatelliteSystem& rhs);

    /// @brief Less than comparison (needed for map)
    /// @param[in] rhs Right hand side of the operator
    /// @return True if lhs < rhs
    constexpr bool operator<(const SatelliteSystem& rhs) const { return value < rhs.value; }

    /// @brief Allow switch(SatelliteSystem_(type)) and comparisons
    constexpr explicit operator SatelliteSystem_() const { return value; }
    /// @brief Prevent usage: if(...)
    constexpr explicit operator bool() = delete;

    /// @brief int conversion operator
    /// @return A int representation of the type
    constexpr explicit operator uint64_t() const { return uint64_t(value); }

    /// @brief std::string conversion operator
    /// @return A std::string representation of the type
    explicit operator std::string() const;

    /// @brief char conversion operator
    /// @return A char representation of the type
    explicit operator char() const;

    /// @brief Get the Time System of the specified Satellite System
    /// @param[in] satSys Satellite System to get the time system for
    static TimeSystem GetTimeSystemForSatelliteSystem(SatelliteSystem satSys);

    /// @brief Get the Time System of this Satellite System
    [[nodiscard]] TimeSystem getTimeSystem() const;

    /// @brief Get a list of satellites in the constellation
    /// @param[in] satSys Satellite System to get the list for
    static std::vector<uint16_t> GetSatellitesForSatelliteSystem(SatelliteSystem satSys);

    /// @brief Get a list of satellites in the constellation
    [[nodiscard]] std::vector<uint16_t> getSatellites() const;

    /// @brief Get the continuous enumeration of the specified Satellite System
    /// @param[in] satSys Satellite System to get the continuous enumeration for
    static size_t ToEnumeration(SatelliteSystem satSys);

    /// @brief Returns a continuous enumeration of the object
    [[nodiscard]] size_t toEnumeration() const;

    /// @brief Get a vector representation of the specified Satellite Systems
    /// @param[in] satSys Satellite System to get the vector for
    static std::vector<SatelliteSystem> ToVector(SatelliteSystem satSys);

    /// @brief Get a vector representation of the specified Satellite Systems
    [[nodiscard]] std::vector<SatelliteSystem> toVector() const;

    /// @brief Returns a list with all possible satellite systems
    static std::vector<SatelliteSystem> GetAll();

  private:
    /// @brief Internal value
    SatelliteSystem_ value = SatelliteSystem_::SatSys_None;
};

// #########################################################################################################################################

/// @brief Converts the provided link into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const SatelliteSystem& data);
/// @brief Converts the provided json object into a link object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, SatelliteSystem& data);

// #########################################################################################################################################

/// @brief Allows combining flags of the SatelliteSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr SatelliteSystem_ operator|(SatelliteSystem_ lhs, SatelliteSystem_ rhs)
{
    return SatelliteSystem_(uint64_t(lhs) | uint64_t(rhs));
}
/// @brief Allows combining flags of the SatelliteSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr SatelliteSystem_ operator|(SatelliteSystem lhs, SatelliteSystem rhs)
{
    return SatelliteSystem_(lhs) | SatelliteSystem_(rhs);
}
/// @brief Allows combining flags of the SatelliteSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr SatelliteSystem_ operator|(SatelliteSystem_ lhs, SatelliteSystem rhs)
{
    return lhs | SatelliteSystem_(rhs);
}
/// @brief Allows combining flags of the SatelliteSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr SatelliteSystem_ operator|(SatelliteSystem lhs, SatelliteSystem_ rhs)
{
    return SatelliteSystem_(lhs) | rhs;
}

/// @brief Allows combining flags of the SatelliteSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr SatelliteSystem_& operator|=(SatelliteSystem_& lhs, const SatelliteSystem_& rhs)
{
    lhs = lhs | rhs;
    return lhs;
}
/// @brief Allows combining flags of the SatelliteSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr SatelliteSystem& operator|=(SatelliteSystem& lhs, const SatelliteSystem& rhs)
{
    lhs = lhs | rhs;
    return lhs;
}
/// @brief Allows combining flags of the SatelliteSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr SatelliteSystem_& operator|=(SatelliteSystem_& lhs, const SatelliteSystem& rhs)
{
    lhs = lhs | SatelliteSystem_(rhs);
    return lhs;
}
/// @brief Allows combining flags of the SatelliteSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr SatelliteSystem& operator|=(SatelliteSystem& lhs, const SatelliteSystem_& rhs)
{
    lhs = lhs | rhs;
    return lhs;
}

/// @brief Allows combining flags of the SatelliteSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr SatelliteSystem_ operator&(SatelliteSystem_ lhs, SatelliteSystem_ rhs)
{
    return SatelliteSystem_(uint64_t(lhs) & uint64_t(rhs));
}
/// @brief Allows combining flags of the SatelliteSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr SatelliteSystem operator&(SatelliteSystem lhs, SatelliteSystem rhs)
{
    return SatelliteSystem_(lhs) & SatelliteSystem_(rhs);
}
/// @brief Allows combining flags of the SatelliteSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr SatelliteSystem_ operator&(SatelliteSystem lhs, SatelliteSystem_ rhs)
{
    return SatelliteSystem_(lhs) & rhs;
}
/// @brief Allows combining flags of the SatelliteSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr SatelliteSystem_ operator&(SatelliteSystem_ lhs, SatelliteSystem rhs)
{
    return lhs & SatelliteSystem_(rhs);
}

/// @brief Allows combining flags of the SatelliteSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr SatelliteSystem_& operator&=(SatelliteSystem_& lhs, const SatelliteSystem_& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}
/// @brief Allows combining flags of the SatelliteSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr SatelliteSystem& operator&=(SatelliteSystem& lhs, const SatelliteSystem& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}
/// @brief Allows combining flags of the SatelliteSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr SatelliteSystem_& operator&=(SatelliteSystem_& lhs, const SatelliteSystem& rhs)
{
    lhs = lhs & SatelliteSystem_(rhs);
    return lhs;
}
/// @brief Allows combining flags of the SatelliteSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr SatelliteSystem& operator&=(SatelliteSystem& lhs, const SatelliteSystem_& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}

/// @brief Allows negating flags of the SatelliteSystem enum.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary NEGed value.
constexpr SatelliteSystem_ operator~(SatelliteSystem_ rhs)
{
    return SatelliteSystem_(~uint64_t(rhs));
}
/// @brief Allows negating flags of the SatelliteSystem enum.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary NEGed value.
constexpr SatelliteSystem_ operator~(SatelliteSystem rhs)
{
    return SatelliteSystem_(~uint64_t(rhs));
}

// #########################################################################################################################################

/// @brief Equal compares values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator==(const SatelliteSystem& lhs, const SatelliteSystem& rhs) { return lhs.value == rhs.value; }
/// @brief Equal compares values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator==(const SatelliteSystem& lhs, const SatelliteSystem_& rhs) { return lhs == SatelliteSystem(rhs); }
/// @brief Equal compares values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator==(const SatelliteSystem_& lhs, const SatelliteSystem& rhs) { return SatelliteSystem(lhs) == rhs; }

/// @brief Inequal compares values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator!=(const SatelliteSystem& lhs, const SatelliteSystem& rhs) { return !(lhs == rhs); }
/// @brief Inequal compares values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator!=(const SatelliteSystem& lhs, const SatelliteSystem_& rhs) { return !(lhs == rhs); }
/// @brief Inequal compares values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator!=(const SatelliteSystem_& lhs, const SatelliteSystem& rhs) { return !(lhs == rhs); }

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] satSys Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const SatelliteSystem& satSys);

/// All Systems
constexpr SatelliteSystem_ SatSys_All = GPS | GAL | GLO | BDS | QZSS | IRNSS | SBAS;

} // namespace NAV

namespace std
{
/// @brief Hash function for SatelliteSystem (needed for unordered_map)
template<>
struct hash<NAV::SatelliteSystem_>
{
    /// @brief Hash function for SatelliteSystem
    /// @param[in] satSys Satellite system
    std::size_t operator()(const NAV::SatelliteSystem_& satSys) const
    {
        using namespace NAV; // NOLINT(google-build-using-namespace)
        switch (satSys)
        {
        case SatSys_None:
            return 1;
        case GPS:
            return 100;
        case GAL:
            return 200;
        case GLO:
            return 300;
        case BDS:
            return 400;
        case QZSS:
            return 500;
        case IRNSS:
            return 600;
        case SBAS:
            return 700;
        }
        return 0;
    }
};
/// @brief Hash function for SatelliteSystem (needed for unordered_map)
template<>
struct hash<NAV::SatelliteSystem>
{
    /// @brief Hash function for SatelliteSystem
    /// @param[in] satSys Satellite system
    std::size_t operator()(const NAV::SatelliteSystem& satSys) const
    {
        return std::hash<NAV::SatelliteSystem_>()(NAV::SatelliteSystem_(satSys));
    }
};
} // namespace std

#ifndef DOXYGEN_IGNORE

/// @brief Formatter for SatelliteSystem
template<>
struct fmt::formatter<NAV::SatelliteSystem>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format SatelliteSystem structs
    /// @param[in] satSys Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::SatelliteSystem& satSys, FormatContext& ctx)
    {
        return fmt::format_to(ctx.out(), "{0}", std::string(satSys));
    }
};

#endif