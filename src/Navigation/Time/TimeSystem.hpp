// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file TimeSystem.hpp
/// @brief Time System defintions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-04-26

#pragma once

#include <string>

namespace NAV
{
/// @brief List of all time systems
enum TimeSystem_
{
    TimeSys_None = 0x00, ///< No Time system
    UTC = 0x01,          ///< Coordinated Universal Time
    GPST = 0x02,         ///< GPS Time
    GLNT = 0x04,         ///< GLONASS Time (GLONASST)
    GST = 0x08,          ///< Galileo System Time
    BDT = 0x10,          ///< BeiDou Time
    QZSST = 0x20,        ///< Quasi-Zenith Satellite System Time
    IRNSST = 0x40,       ///< Indian Regional Navigation Satellite System Time
};

/// @brief Time System defintions
class TimeSystem
{
  public:
    /// @brief Default Constructor
    constexpr TimeSystem() = default;

    /// @brief Implicit Constructor from Value type
    /// @param[in] type Value type to construct from
    constexpr TimeSystem(TimeSystem_ type) // NOLINT(hicpp-explicit-conversions, google-explicit-constructor)
        : value(type)
    {}

    /// @brief Construct new object from std::string
    /// @param[in] typeString String representation of the TimeSystem
    static TimeSystem fromString(const std::string& typeString)
    {
        if (typeString == "UTC")
        {
            return UTC;
        }
        if (typeString == "GPST" || typeString == "GPS")
        {
            return GPST;
        }
        if (typeString == "GLNT" || typeString == "GLO")
        {
            return GLNT;
        }
        if (typeString == "GST" || typeString == "GAL")
        {
            return GST;
        }
        if (typeString == "BDT")
        {
            return BDT;
        }
        if (typeString == "QZSST" || typeString == "QZS")
        {
            return QZSST;
        }
        if (typeString == "IRNSST" || typeString == "IRN")
        {
            return IRNSST;
        }

        return TimeSys_None;
    }

    /// @brief Assignment operator from Value type
    /// @param[in] v Value type to construct from
    /// @return The Type type from the value type
    constexpr TimeSystem& operator=(TimeSystem_ v)
    {
        value = v;
        return *this;
    }

    friend constexpr bool operator==(const TimeSystem& lhs, const TimeSystem& rhs);

    /// @brief Allow switch(TimeSystem_(type)) and comparisons
    constexpr explicit operator TimeSystem_() const { return value; }
    /// @brief Prevent usage: if(...)
    constexpr explicit operator bool() = delete;

    /// @brief int conversion operator
    /// @return A int representation of the type
    constexpr explicit operator int() const { return int(value); }

    /// @brief std::string conversion operator
    /// @return A std::string representation of the type
    explicit operator std::string() const
    {
        switch (value)
        {
        case UTC:
            return "UTC";
        case GPST:
            return "GPST";
        case GLNT:
            return "GLNT";
        case GST:
            return "GST";
        case BDT:
            return "BDT";
        case QZSST:
            return "QZSST";
        case IRNSST:
            return "IRNSST";
        case TimeSys_None:
            return "None";
        }

        return "None";
    }

  private:
    /// @brief Internal value
    TimeSystem_ value = TimeSystem_::TimeSys_None;
};

/// @brief Allows combining flags of the TimeSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr TimeSystem_ operator|(TimeSystem_ lhs, TimeSystem_ rhs)
{
    return TimeSystem_(int(lhs) | int(rhs));
}
/// @brief Allows combining flags of the TimeSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr TimeSystem operator|(TimeSystem lhs, TimeSystem rhs)
{
    return { TimeSystem_(lhs) | TimeSystem_(rhs) };
}
/// @brief Allows combining flags of the TimeSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr TimeSystem operator|(TimeSystem_ lhs, TimeSystem rhs)
{
    return { lhs | TimeSystem_(rhs) };
}
/// @brief Allows combining flags of the TimeSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr TimeSystem operator|(TimeSystem lhs, TimeSystem_ rhs)
{
    return { TimeSystem_(lhs) | rhs };
}

/// @brief Allows combining flags of the TimeSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr TimeSystem_& operator|=(TimeSystem_& lhs, const TimeSystem_& rhs)
{
    lhs = lhs | rhs;
    return lhs;
}
/// @brief Allows combining flags of the TimeSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr TimeSystem& operator|=(TimeSystem& lhs, const TimeSystem& rhs)
{
    lhs = lhs | rhs;
    return lhs;
}
/// @brief Allows combining flags of the TimeSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr TimeSystem_& operator|=(TimeSystem_& lhs, const TimeSystem& rhs)
{
    lhs = lhs | TimeSystem_(rhs);
    return lhs;
}
/// @brief Allows combining flags of the TimeSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr TimeSystem& operator|=(TimeSystem& lhs, const TimeSystem_& rhs)
{
    lhs = lhs | rhs;
    return lhs;
}

/// @brief Allows combining flags of the TimeSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr TimeSystem_ operator&(TimeSystem_ lhs, TimeSystem_ rhs)
{
    return TimeSystem_(int(lhs) & int(rhs));
}
/// @brief Allows combining flags of the TimeSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr TimeSystem operator&(TimeSystem lhs, TimeSystem rhs)
{
    return { TimeSystem_(lhs) & TimeSystem_(rhs) };
}
/// @brief Allows combining flags of the TimeSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr TimeSystem operator&(TimeSystem lhs, TimeSystem_ rhs)
{
    return { TimeSystem_(lhs) & rhs };
}
/// @brief Allows combining flags of the TimeSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr TimeSystem operator&(TimeSystem_ lhs, TimeSystem rhs)
{
    return { lhs & TimeSystem_(rhs) };
}

/// @brief Allows combining flags of the TimeSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr TimeSystem_& operator&=(TimeSystem_& lhs, const TimeSystem_& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}
/// @brief Allows combining flags of the TimeSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr TimeSystem& operator&=(TimeSystem& lhs, const TimeSystem& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}
/// @brief Allows combining flags of the TimeSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr TimeSystem_& operator&=(TimeSystem_& lhs, const TimeSystem& rhs)
{
    lhs = lhs & TimeSystem_(rhs);
    return lhs;
}
/// @brief Allows combining flags of the TimeSystem enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr TimeSystem& operator&=(TimeSystem& lhs, const TimeSystem_& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}

/// @brief Allows negating flags of the TimeSystem enum.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary NEGed value.
constexpr TimeSystem operator~(TimeSystem_ rhs)
{
    return { TimeSystem_(~int(rhs)) };
}
/// @brief Allows negating flags of the TimeSystem enum.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary NEGed value.
constexpr TimeSystem operator~(TimeSystem rhs)
{
    return { TimeSystem_(~int(rhs)) };
}

/// @brief Equal compares values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator==(const TimeSystem& lhs, const TimeSystem& rhs) { return lhs.value == rhs.value; }
/// @brief Equal compares values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator==(const TimeSystem& lhs, const TimeSystem_& rhs) { return lhs == TimeSystem(rhs); }
/// @brief Equal compares values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator==(const TimeSystem_& lhs, const TimeSystem& rhs) { return TimeSystem(lhs) == rhs; }

/// @brief Inequal compares values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator!=(const TimeSystem& lhs, const TimeSystem& rhs) { return !(lhs == rhs); }
/// @brief Inequal compares values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator!=(const TimeSystem& lhs, const TimeSystem_& rhs) { return !(lhs == rhs); }
/// @brief Inequal compares values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator!=(const TimeSystem_& lhs, const TimeSystem& rhs) { return !(lhs == rhs); }

} // namespace NAV
