/// @file SatelliteIdentifier.hpp
/// @brief Structs identifying a unique satellite
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-04-29

#pragma once

#include <vector>
#include <nlohmann/json.hpp>
using json = nlohmann::json; ///< json namespace
#include <fmt/format.h>

#include "SatelliteSystem.hpp"
#include "Frequency.hpp"

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
    bool operator==(const SatId& rhs) const { return satSys == rhs.satSys && satNum == rhs.satNum; }

    /// @brief Less than comparison (needed for map)
    /// @param[in] rhs Right hand side of the operator
    /// @return True if lhs < rhs
    bool operator<(const SatId& rhs) const
    {
        return satSys == rhs.satSys ? satNum < rhs.satNum
                                    : satSys < rhs.satSys;
    }
};

/// @brief Identifies a satellite signal (satellite frequency and number)
struct SatSigId
{
    /// @brief Constructor
    /// @param[in] freq Signal frequency
    /// @param[in] satNum Number of the satellite
    SatSigId(Frequency freq, uint16_t satNum)
        : freq(freq), satNum(satNum) {}

    /// Default constructor
    SatSigId() = default;

    Frequency freq = Freq_None; ///< Signal frequency
    uint16_t satNum = 0;        ///< Number of the satellite

    /// @brief Equal comparison (needed for unordered_map)
    /// @param[in] rhs Right hand side of the operator
    /// @return True if the elements are equal
    bool operator==(const SatSigId& rhs) const { return freq == rhs.freq && satNum == rhs.satNum; }

    /// @brief Less than comparison (needed for map)
    /// @param[in] rhs Right hand side of the operator
    /// @return True if lhs < rhs
    bool operator<(const SatSigId& rhs) const
    {
        return freq == rhs.freq ? satNum < rhs.satNum
                                : freq < rhs.freq;
    }

    /// @brief Returns a satellite identifier for the satellite signal
    [[nodiscard]] SatId toSatId() const
    {
        return { freq.getSatSys(), satNum };
    }
};

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
/// @param[in, out] satellites Reference to the satId vector to select
bool ShowSatelliteSelector(const char* label, std::vector<SatId>& satellites);

} // namespace NAV

#ifndef DOXYGEN_IGNORE

/// @brief Formatter for SatId
template<>
struct fmt::formatter<NAV::SatId>
{
    /// @brief Parse function to make the struct formattable
    /// @param[in] ctx Parser context
    /// @return Beginning of the context
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format SatId structs
    /// @param[in] satId Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::SatId& satId, FormatContext& ctx)
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
    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    /// @brief Defines how to format SatSigId structs
    /// @param[in] satSigId Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::SatSigId& satSigId, FormatContext& ctx)
    {
        return fmt::format_to(ctx.out(), "{0}-{1:02d}", std::string(satSigId.freq), satSigId.satNum);
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
    /// @return Has value for the satellite identifier
    std::size_t operator()(const NAV::SatId& f) const
    {
        return std::hash<NAV::SatelliteSystem_>{}(NAV::SatelliteSystem_(f.satSys)) ^ (std::hash<decltype(f.satNum)>()(f.satNum) << 1);
    }
};
/// @brief Hash function for SatSigId (needed for unordered_map)
template<>
struct hash<NAV::SatSigId>
{
    /// @brief Hash function for SatSigId
    /// @param[in] f Satellite identifier
    /// @return Has value for the satellite identifier
    std::size_t operator()(const NAV::SatSigId& f) const
    {
        return std::hash<NAV::Frequency_>{}(NAV::Frequency_(f.freq)) ^ (std::hash<decltype(f.satNum)>()(f.satNum) << 1);
    }
};
} // namespace std