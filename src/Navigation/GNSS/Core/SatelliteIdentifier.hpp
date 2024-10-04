// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file SatelliteIdentifier.hpp
/// @brief Structs identifying a unique satellite
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-04-29

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace
#include <fmt/format.h>

#include "SatelliteSystem.hpp"
#include "Frequency.hpp"
#include "Code.hpp"

#include "util/Container/STL.hpp"

namespace NAV
{

/// @brief Identifies a satellite (satellite system and number)
struct SatId
{
    /// @brief Constructor
    /// @param[in] satSys Satellite system
    /// @param[in] satNum Number of the satellite
    SatId(SatelliteSystem satSys, uint16_t satNum)
        : satSys(satSys), satNum(satNum) {}

    /// Default constructor
    SatId() = default;

    SatelliteSystem satSys = SatSys_None; ///< Satellite system (GPS, GLONASS, GALILEO, QZSS, BDS, IRNSS, SBAS)
    uint16_t satNum = 0;                  ///< Number of the satellite

    /// @brief Equal comparison (needed for unordered_map)
    /// @param[in] rhs Right hand side of the operator
    /// @return True if the elements are equal
    constexpr bool operator==(const SatId& rhs) const { return satSys == rhs.satSys && satNum == rhs.satNum; }

    /// @brief Less than comparison (needed for map)
    /// @param[in] rhs Right hand side of the operator
    /// @return True if lhs < rhs
    constexpr bool operator<(const SatId& rhs) const
    {
        return satSys == rhs.satSys ? satNum < rhs.satNum
                                    : satSys < rhs.satSys;
    }

    /// Checks if the satellite is geostationary
    [[nodiscard]] bool isGeo() const;
};

/// @brief Identifies a satellite signal (satellite frequency and number)
struct SatSigId
{
    /// @brief Constructor
    /// @param[in] code Signal code
    /// @param[in] satNum Number of the satellite
    SatSigId(Code code, uint16_t satNum)
        : code(code), satNum(satNum) {}

    /// Default constructor
    SatSigId() = default;

    Code code = Code::None; ///< Code
    uint16_t satNum = 0;    ///< Number of the satellite

    /// @brief Equal comparison (needed for unordered_map)
    /// @param[in] rhs Right hand side of the operator
    /// @return True if the elements are equal
    bool operator==(const SatSigId& rhs) const { return code == rhs.code && satNum == rhs.satNum; }

    /// @brief Less than comparison (needed for map)
    /// @param[in] rhs Right hand side of the operator
    /// @return True if lhs < rhs
    bool operator<(const SatSigId& rhs) const
    {
        if (toSatId().satSys == rhs.toSatId().satSys)
        {
            if (satNum == rhs.satNum)
            {
                return Code::Set(code) < Code::Set(rhs.code);
            }
            return satNum < rhs.satNum;
        }
        return toSatId().satSys < rhs.toSatId().satSys;
    }

    /// @brief Returns a satellite identifier for the satellite signal
    [[nodiscard]] SatId toSatId() const
    {
        return { code.getFrequency().getSatSys(), satNum };
    }

    /// @brief Returns the frequency of the satellite signal
    [[nodiscard]] Frequency freq() const
    {
        return code.getFrequency();
    }
};

/// @brief Less than comparison from string representation
/// @param[in] lhs Left hand side of the operator
/// @param[in] rhs Right hand side of the operator
/// @return True if lhs < rhs
bool lessCompareSatSigId(const std::string& lhs, const std::string& rhs);

/// @brief Converts the provided link into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Data to convert into json
void to_json(json& j, const SatId& data);
/// @brief Converts the provided json object into a link object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, SatId& data);

/// @brief Converts the provided link into a json object
/// @param[out] j Json object which gets filled with the info
/// @param[in] data Object to convert into json
void to_json(json& j, const SatSigId& data);
/// @brief Converts the provided json object into a link object
/// @param[in] j Json object with the needed values
/// @param[out] data Object to fill from the json
void from_json(const json& j, SatSigId& data);

/// @brief Shows a ComboBox to select satellites
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in, out] satellites Reference to the SatId vector to select
/// @param[in] filterSys Enable/Disable GUI elements according to this filter
/// @param[in] displayOnlyNumber Display only the number, not the system
bool ShowSatelliteSelector(const char* label, std::vector<SatId>& satellites, SatelliteSystem filterSys = SatSys_All, bool displayOnlyNumber = false);

/// @brief Shows a ComboBox to select a single satellite
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in, out] satellite Reference to the SatId to select
/// @param[in] filterSys Enable/Disable GUI elements according to this filter
/// @param[in] displayOnlyNumber Display only the number, not the system
bool ShowSatelliteSelector(const char* label, SatId& satellite, SatelliteSystem filterSys = SatSys_All, bool displayOnlyNumber = false);

} // namespace NAV

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::SatId& obj);

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::SatSigId& obj);

#ifndef DOXYGEN_IGNORE

/// @brief Formatter for SatId
template<>
struct fmt::formatter<NAV::SatId>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    static constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin())
    {
        return ctx.begin();
    }

    /// @brief Defines how to format SatId structs
    /// @param[in] satId Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::SatId& satId, FormatContext& ctx) const -> decltype(ctx.out())
    {
        return fmt::format_to(ctx.out(), "{0}{1:02d}", char(satId.satSys), satId.satNum);
    }
};

/// @brief Formatter for SatSigId
template<>
struct fmt::formatter<NAV::SatSigId>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    static constexpr auto parse(format_parse_context& ctx) -> decltype(ctx.begin())
    {
        return ctx.begin();
    }

    /// @brief Defines how to format SatSigId structs
    /// @param[in] satSigId Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::SatSigId& satSigId, FormatContext& ctx) const
    {
        return fmt::format_to(ctx.out(), "{0}-{1:02d}", satSigId.code, satSigId.satNum);
    }
};

#endif

namespace std
{
/// @brief Hash function for SatId (needed for unordered_map)
template<>
struct hash<NAV::SatId>
{
    /// @brief Hash function for SatId
    /// @param[in] f Satellite identifier
    std::size_t operator()(const NAV::SatId& f) const
    {
        auto hash1 = std::hash<NAV::SatelliteSystem_>{}(NAV::SatelliteSystem_(f.satSys));
        auto hash2 = static_cast<size_t>(f.satNum);

        return hash1 | (hash2 << 10);
    }
};
/// @brief Hash function for SatSigId (needed for unordered_map)
template<>
struct hash<NAV::SatSigId>
{
    /// @brief Hash function for SatSigId
    /// @param[in] f Satellite signal identifier
    std::size_t operator()(const NAV::SatSigId& f) const
    {
        auto hash1 = static_cast<size_t>(f.code.getEnumValue());
        auto hash2 = static_cast<size_t>(f.satNum);

        return hash1 | (hash2 << 8);
    }
};
} // namespace std