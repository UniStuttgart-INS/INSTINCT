// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Frequency.hpp
/// @brief Frequency definition for different satellite systems
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-04-26

#pragma once

#include <string>
#include <fmt/format.h>

#include "Navigation/GNSS/Core/SatelliteSystem.hpp"

namespace NAV
{

/// Enumerate for GNSS frequencies
enum Frequency_ : uint64_t
{ // clang-format off
    Freq_None = 0x0000'0000'0000'0000, ///< None
    G01       = 0x0000'0000'0000'0001, ///< GPS L1 (1575.42 MHz).
    G02       = 0x0000'0000'0000'0002, ///< GPS L2 (1227.6 MHz).
    G05       = 0x0000'0000'0000'0004, ///< GPS L5 (1176.45 MHz).
    E01       = 0x0000'0000'0000'0100, ///< Galileo, "E1" (1575.42 MHz).
    E05       = 0x0000'0000'0000'0200, ///< Galileo E5a (1176.45 MHz).
    E06       = 0x0000'0000'0000'0400, ///< Galileo E6 (1278.75 MHz).
    E07       = 0x0000'0000'0000'0800, ///< Galileo E5b (1207.14 MHz).
    E08       = 0x0000'0000'0000'1000, ///< Galileo E5 (E5a + E5b) (1191.795MHz).
    R01       = 0x0000'0000'0001'0000, ///< GLONASS, "G1" (1602 MHZ).
    R02       = 0x0000'0000'0002'0000, ///< GLONASS, "G2" (1246 MHz).
    R03       = 0x0000'0000'0004'0000, ///< GLONASS, "G3" (1202.025 MHz).
    R04       = 0x0000'0000'0008'0000, ///< GLONASS, "G1a" (1600.995 MHZ).
    R06       = 0x0000'0000'0010'0000, ///< GLONASS, "G2a" (1248.06 MHz).
    B01       = 0x0000'0000'0100'0000, ///< Beidou B1 (1575.42 MHz).
    B02       = 0x0000'0000'0200'0000, ///< Beidou B1-2 (1561.098 MHz).
    B05       = 0x0000'0000'0400'0000, ///< Beidou B2a (1176.45 MHz).
    B06       = 0x0000'0000'0800'0000, ///< Beidou B3 (1268.52 MHz).
    B07       = 0x0000'0000'1000'0000, ///< Beidou B2b (1207.14 MHz).
    B08       = 0x0000'0000'2000'0000, ///< Beidou B2 (B2a + B2b) (1191.795MHz).
    J01       = 0x0000'0001'0000'0000, ///< QZSS L1 (1575.42 MHz).
    J02       = 0x0000'0002'0000'0000, ///< QZSS L2 (1227.6 MHz).
    J05       = 0x0000'0004'0000'0000, ///< QZSS L5 (1176.45 MHz).
    J06       = 0x0000'0008'0000'0000, ///< QZSS L6 / LEX (1278.75 MHz).
    I05       = 0x0000'0100'0000'0000, ///< IRNSS L5 (1176.45 MHz).
    I09       = 0x0000'0200'0000'0000, ///< IRNSS S (2492.028 MHz).
    S01       = 0x0001'0000'0000'0000, ///< SBAS L1 (1575.42 MHz).
    S05       = 0x0002'0000'0000'0000, ///< SBAS L5 (1176.45 MHz).
}; // clang-format on

/// @brief Frequency definition for different satellite systems
class Frequency
{
  public:
    /// @brief Default Constructor
    constexpr Frequency() = default;

    /// @brief Implicit Constructor from Value type
    /// @param[in] type Value type to construct from
    constexpr Frequency(Frequency_ type) // NOLINT(hicpp-explicit-conversions, google-explicit-constructor)
        : value(type)
    {}

    /// @brief Construct new object from std::string
    /// @param[in] typeString String representation of the frequency
    static Frequency fromString(const std::string& typeString);

    /// @brief Assignment operator from Value type
    /// @param[in] v Value type to construct from
    /// @return The Type type from the value type
    constexpr Frequency& operator=(Frequency_ v)
    {
        value = v;
        return *this;
    }

    friend constexpr bool operator==(const Frequency& lhs, const Frequency& rhs);

    /// @brief Less than comparison (needed for map)
    /// @param[in] rhs Right hand side of the operator
    /// @return True if lhs < rhs
    constexpr bool operator<(const Frequency& rhs) const { return value < rhs.value; }

    /// @brief Allow switch(Frequency_(type)) and comparisons
    constexpr explicit operator Frequency_() const { return value; }
    /// @brief Prevent usage: if(...)
    constexpr explicit operator bool() = delete;

    /// @brief int conversion operator
    /// @return A int representation of the type
    constexpr explicit operator uint64_t() const { return uint64_t(value); }

    /// @brief std::string conversion operator
    /// @return A std::string representation of the type
    explicit operator std::string() const;

    /// @brief Get the Time System of the specified Frequency
    /// @param[in] freq Frequency to get the satellite system for
    static SatelliteSystem GetSatelliteSystemForFrequency(Frequency freq);

    /// @brief Get the satellite system for which this frequency is defined
    [[nodiscard]] SatelliteSystem getSatSys() const
    {
        return GetSatelliteSystemForFrequency(value);
    }

    /// @brief Get the frequency in [Hz]
    /// @param[in] freq Frequency to get the value for
    /// @param[in] num Frequency number. Only used for GLONASS G1 and G2
    static double GetFrequency(Frequency freq, int8_t num);

    /// @brief Get the frequency in [Hz]
    /// @param[in] num Frequency number. Only used for GLONASS G1 and G2
    [[nodiscard]] double getFrequency(int8_t num) const
    {
        return GetFrequency(value, num);
    }

    /// @brief Returns the L1 Frequency for each constellation
    /// @param[in] freq Frequency to get the value for
    static Frequency GetL1(Frequency freq);

    /// @brief Returns the L1 Frequency for each constellation
    [[nodiscard]] Frequency getL1() const
    {
        return GetL1(value);
    }

    /// Counts the amount of frequencies contained
    [[nodiscard]] size_t count() const;

    /// @brief Returns a list with all possible frequencies
    constexpr static std::array<Frequency, 27> GetAll()
    {
        return { G01, G02, G05,
                 E01, E05, E06, E07, E08,
                 R01, R02, R03, R04, R06,
                 B01, B02, B05, B06, B07, B08,
                 J01, J02, J05, J06,
                 I05, I09,
                 S01, S05 };
    }

  private:
    /// @brief Internal value
    Frequency_ value = Frequency_::Freq_None;
};

// #########################################################################################################################################

/// @brief Converts the provided link into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const Frequency& data);
/// @brief Converts the provided json object into a link object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, Frequency& data);

// #########################################################################################################################################

/// @brief Allows combining flags of the Frequency enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr Frequency_ operator|(Frequency_ lhs, Frequency_ rhs)
{
    return Frequency_(uint64_t(lhs) | uint64_t(rhs));
}
/// @brief Allows combining flags of the Frequency enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr Frequency_ operator|(Frequency lhs, Frequency rhs)
{
    return Frequency_(lhs) | Frequency_(rhs);
}
/// @brief Allows combining flags of the Frequency enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr Frequency_ operator|(Frequency_ lhs, Frequency rhs)
{
    return lhs | Frequency_(rhs);
}
/// @brief Allows combining flags of the Frequency enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr Frequency_ operator|(Frequency lhs, Frequency_ rhs)
{
    return Frequency_(lhs) | rhs;
}

/// @brief Allows combining flags of the Frequency enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr Frequency_& operator|=(Frequency_& lhs, const Frequency_& rhs)
{
    lhs = lhs | rhs;
    return lhs;
}
/// @brief Allows combining flags of the Frequency enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr Frequency& operator|=(Frequency& lhs, const Frequency& rhs)
{
    lhs = lhs | rhs;
    return lhs;
}
/// @brief Allows combining flags of the Frequency enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr Frequency_& operator|=(Frequency_& lhs, const Frequency& rhs)
{
    lhs = lhs | Frequency_(rhs);
    return lhs;
}
/// @brief Allows combining flags of the Frequency enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ORed value.
constexpr Frequency& operator|=(Frequency& lhs, const Frequency_& rhs)
{
    lhs = lhs | rhs;
    return lhs;
}

/// @brief Allows combining flags of the Frequency enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency_ operator&(Frequency_ lhs, Frequency_ rhs)
{
    return Frequency_(uint64_t(lhs) & uint64_t(rhs));
}
/// @brief Allows combining flags of the Frequency enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency_ operator&(Frequency lhs, Frequency rhs)
{
    return Frequency_(lhs) & Frequency_(rhs);
}
/// @brief Allows combining flags of the Frequency enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency_ operator&(Frequency lhs, Frequency_ rhs)
{
    return Frequency_(lhs) & rhs;
}
/// @brief Allows combining flags of the Frequency enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency_ operator&(Frequency_ lhs, Frequency rhs)
{
    return lhs & Frequency_(rhs);
}

/// @brief Allows combining flags of the Frequency enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency_& operator&=(Frequency_& lhs, const Frequency_& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}
/// @brief Allows combining flags of the Frequency enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency& operator&=(Frequency& lhs, const Frequency& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}
/// @brief Allows combining flags of the Frequency enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency_& operator&=(Frequency_& lhs, const Frequency& rhs)
{
    lhs = lhs & Frequency_(rhs);
    return lhs;
}
/// @brief Allows combining flags of the Frequency enum.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency& operator&=(Frequency& lhs, const Frequency_& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}

/// @brief Allows negating flags of the Frequency enum.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary NEGed value.
constexpr Frequency_ operator~(Frequency_ rhs)
{
    return Frequency_(~uint64_t(rhs));
}
/// @brief Allows negating flags of the Frequency enum.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary NEGed value.
constexpr Frequency_ operator~(Frequency rhs)
{
    return Frequency_(~uint64_t(rhs));
}

/// @brief Equal compares values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator==(const Frequency& lhs, const Frequency& rhs) { return lhs.value == rhs.value; }
/// @brief Equal compares values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator==(const Frequency& lhs, const Frequency_& rhs) { return lhs == Frequency(rhs); }
/// @brief Equal compares values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator==(const Frequency_& lhs, const Frequency& rhs) { return Frequency(lhs) == rhs; }

/// @brief Inequal compares values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator!=(const Frequency& lhs, const Frequency& rhs) { return !(lhs == rhs); }
/// @brief Inequal compares values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator!=(const Frequency& lhs, const Frequency_& rhs) { return !(lhs == rhs); }
/// @brief Inequal compares values
/// @param[in] lhs Left-hand side of the operator
/// @param[in] rhs Right-hand side of the operator
/// @return Whether the comparison was successful
constexpr bool operator!=(const Frequency_& lhs, const Frequency& rhs) { return !(lhs == rhs); }

// #########################################################################################################################################

/// @brief Allows filtering Frequency with SatelliteSystem.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency_ operator&(Frequency_ lhs, SatelliteSystem_ rhs)
{
    return Frequency_(uint64_t(lhs) & uint64_t(rhs));
}
/// @brief Allows filtering Frequency with SatelliteSystem.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency_ operator&(SatelliteSystem_ lhs, Frequency_ rhs)
{
    return Frequency_(uint64_t(lhs) & uint64_t(rhs));
}
/// @brief Allows filtering Frequency with SatelliteSystem.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency_& operator&=(Frequency_& lhs, const SatelliteSystem_& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}

/// @brief Allows filtering Frequency with SatelliteSystem.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency_ operator&(Frequency_ lhs, SatelliteSystem rhs)
{
    return Frequency_(uint64_t(lhs) & uint64_t(rhs));
}
/// @brief Allows filtering Frequency with SatelliteSystem.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency_ operator&(SatelliteSystem lhs, Frequency_ rhs)
{
    return Frequency_(uint64_t(lhs) & uint64_t(rhs));
}
/// @brief Allows filtering Frequency with SatelliteSystem.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency_& operator&=(Frequency_& lhs, const SatelliteSystem& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}

/// @brief Allows filtering Frequency with SatelliteSystem.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency_ operator&(Frequency lhs, SatelliteSystem_ rhs)
{
    return Frequency_(uint64_t(lhs) & uint64_t(rhs));
}
/// @brief Allows filtering Frequency with SatelliteSystem.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency_ operator&(SatelliteSystem_ lhs, Frequency rhs)
{
    return Frequency_(uint64_t(lhs) & uint64_t(rhs));
}
/// @brief Allows filtering Frequency with SatelliteSystem.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency& operator&=(Frequency& lhs, const SatelliteSystem_& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}

/// @brief Allows filtering Frequency with SatelliteSystem.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency_ operator&(Frequency lhs, SatelliteSystem rhs)
{
    return Frequency_(uint64_t(lhs) & uint64_t(rhs));
}
/// @brief Allows filtering Frequency with SatelliteSystem.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency_ operator&(SatelliteSystem lhs, Frequency rhs)
{
    return Frequency_(uint64_t(lhs) & uint64_t(rhs));
}
/// @brief Allows filtering Frequency with SatelliteSystem.
/// @param[in] lhs Left-hand side enum value.
/// @param[in] rhs Right-hand side enum value.
/// @return The binary ANDed value.
constexpr Frequency& operator&=(Frequency& lhs, const SatelliteSystem& rhs)
{
    lhs = lhs & rhs;
    return lhs;
}

/// All Frequencies
constexpr Frequency_ Freq_All = G01 | G02 | G05
                                | E01 | E05 | E06 | E07 | E08
                                | R01 | R02 | R03 | R04 | R06
                                | B01 | B02 | B05 | B06 | B07 | B08
                                | J01 | J02 | J05 | J06
                                | I05 | I09
                                | S01 | S05;

/// @brief Shows a ComboBox to select GNSS frequencies
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in, out] frequency Reference to the frequency object to select
/// @param[in] singleSelect If true, only one code can be selected at a time
bool ShowFrequencySelector(const char* label, Frequency& frequency, bool singleSelect = false);

} // namespace NAV

namespace std
{
/// @brief Hash function for Frequency (needed for unordered_map)
template<>
struct hash<NAV::Frequency_>
{
    /// @brief Hash function for Frequency
    /// @param[in] f Satellite frequency
    /// @return Has value for the frequency
    std::size_t operator()(const NAV::Frequency_& f) const
    {
        using namespace NAV; // NOLINT(google-build-using-namespace)
        switch (f)
        {
        case Freq_None:
            return 2;
        case G01:
            return 101;
        case G02:
            return 102;
        case G05:
            return 103;
        case E01:
            return 201;
        case E05:
            return 202;
        case E06:
            return 203;
        case E07:
            return 204;
        case E08:
            return 205;
        case R01:
            return 301;
        case R02:
            return 302;
        case R03:
            return 303;
        case R04:
            return 304;
        case R06:
            return 305;
        case B01:
            return 401;
        case B02:
            return 402;
        case B05:
            return 403;
        case B06:
            return 404;
        case B07:
            return 405;
        case B08:
            return 406;
        case J01:
            return 501;
        case J02:
            return 502;
        case J05:
            return 503;
        case J06:
            return 504;
        case I05:
            return 601;
        case I09:
            return 602;
        case S01:
            return 701;
        case S05:
            return 702;
        }

        return 0;
    }
};
/// @brief Hash function for Frequency (needed for unordered_map)
template<>
struct hash<NAV::Frequency>
{
    /// @brief Hash function for Frequency
    /// @param[in] f Satellite frequency
    /// @return Has value for the frequency
    std::size_t operator()(const NAV::Frequency& f) const
    {
        return std::hash<NAV::Frequency_>()(NAV::Frequency_(f));
    }
};
} // namespace std

#ifndef DOXYGEN_IGNORE

/// @brief Formatter for Frequency
template<>
struct fmt::formatter<NAV::Frequency> : fmt::formatter<std::string>
{
    /// @brief Defines how to format Frequency structs
    /// @param[in] freq Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::Frequency& freq, FormatContext& ctx)
    {
        return fmt::formatter<std::string>::format(std::string(freq), ctx);
    }
};

#endif