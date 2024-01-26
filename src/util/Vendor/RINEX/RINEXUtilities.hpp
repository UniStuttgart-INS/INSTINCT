// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RINEXUtilities.hpp
/// @brief Functions to work with RINEX.
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author N. Stahl (HiWi: Moved from RinexObsFile.hpp)
/// @date 18.12.2023

#pragma once

#include <map>
#include <unordered_set>

#include "Navigation/GNSS/Core/Code.hpp"
#include "Navigation/GNSS/Core/SatelliteIdentifier.hpp"
#include "Navigation/Time/TimeSystem.hpp"
#include "NodeData/GNSS/GnssObs.hpp"
#include "util/Eigen.hpp"
#include "util//Json.hpp"

namespace NAV
{
namespace vendor::RINEX
{

/// @brief Observation types of the 'SYS / # / OBS TYPES' header
enum class ObsType
{
    Error, ///< Error Type
    C,     ///< Code / Pseudorange
    L,     ///< Phase
    D,     ///< Doppler
    S,     ///< Raw signal strength(carrier to noise ratio)
    I,     ///< Ionosphere phase delay
    X,     ///< Receiver channel numbers
};

/// @brief Description of the observations from the 'SYS / # / OBS TYPES' header
struct ObservationDescription
{
    /// RINEX observation type
    ///
    /// C = Code / Pseudorange [m]
    /// L = Phase [full cycles]
    /// D = Doppler [Hz]
    /// S = Raw signal strength (carrier to noise ratio) [unit receiver dependent]
    /// I = Ionosphere phase delay [full cycles]
    /// X = Receiver channel numbers
    ObsType type{ ObsType::Error };

    /// GNSS Code
    Code code;
};

/// Rinex Observation File Header information
struct ObsHeader
{
    double version = 3.04;             ///< Format version
    std::string runBy;                 ///< Name of agency creating current file [A20]
    std::vector<std::string> comments; ///< Comment line(s) [A60] (Optional)
    std::string markerName;            ///< Name of antenna marker [A60]
    std::string markerNumber;          ///< Number of antenna marker [A20] (Optional)

    /// Available marker types
    enum class MarkerTypes
    {
        GEODETIC,      ///< Earth-fixed, high-precision monument
        NON_GEODETIC,  ///< Earth-fixed, low-precision monument
        NON_PHYSICAL,  ///< Generated from network processing
        SPACEBORNE,    ///< Orbiting space vehicle
        GROUND_CRAFT,  ///< Mobile terrestrial vehicle
        WATER_CRAFT,   ///< Mobile water craft
        AIRBORNE,      ///< Aircraft, balloon, etc.
        FIXED_BUOY,    ///< "Fixed" on water surface
        FLOATING_BUOY, ///< Floating on water surface
        FLOATING_ICE,  ///< Floating ice sheet, etc.
        GLACIER,       ///< "Fixed" on a glacier
        BALLISTIC,     ///< Rockets, shells, etc
        ANIMAL,        ///< Animal carrying a receiver
        HUMAN,         ///< Human being
        USER_DEFINED,  ///< Users may define other project-dependent keywords
        COUNT,         ///< Amount of items in the enum
    };
    MarkerTypes markerType = MarkerTypes::GEODETIC;              ///< Type of the marker
    std::string markerTypeUser;                                  ///< User-defined Marker Type [A20]
    std::string observer;                                        ///< Name of observer [A20]
    std::string agency;                                          ///< Name of agency [A40]
    std::string receiverNumber;                                  ///< Receiver number [A20]
    std::string receiverType;                                    ///< Receiver type [A20]
    std::string receiverVersion;                                 ///< Receiver version (e.g. Internal Software Version) [A20]
    std::string antennaNumber;                                   ///< Antenna number [A20]
    std::string antennaType;                                     ///< Antenna type [A20]
    bool approxPositionEnabled = true;                           ///< Whether the approx position should be written
    Eigen::Vector3d approxPositionXYZ = Eigen::Vector3d::Zero(); ///< Geocentric approximate marker position (Units: Meters, System: ITRS recommended) Optional for moving platforms [F14.4]
    std::array<double, 3> antennaDeltaHeightEastNorth{};         ///< Antenna height and horizontal eccentricity of ARP relative to the marker (east/north) in (meters) [F14.4]

    // ------------------------------- Dynamic data to clear and not to save ---------------------------------

    SatelliteSystem satSys = SatSys_None;                                          ///< Satellite System
    std::map<SatelliteSystem, std::vector<ObservationDescription>> systemObsTypes; ///< Observation types for each satellite system
    double interval = 1e9;                                                         ///< Observation interval in seconds
    InsTime timeFirstObs;                                                          ///< Time of first observation record
    InsTime timeLastObs;                                                           ///< Time of last observation record
    std::unordered_set<SatId> satellites;                                          ///< Satellites observed
    TimeSystem timeSys = TimeSys_None;                                             ///< Time system used

    /// @brief Reset all dynamic data
    void reset();

    /// @brief Adds the obs type if not existing already
    /// @param code Signal Code
    /// @param type RINEX obs type
    /// @return True if the type was not present yet
    bool addObsType(const Code& code, ObsType type);

    /// @brief Generates the RINEX observation header
    [[nodiscard]] std::string generateHeader() const;
    /// @brief Generates the Header line 'RINEX VERSION / TYPE'
    [[nodiscard]] std::string headerLineRinexVersionType() const;
    /// @brief Generates the Header line 'PGM / RUN BY / DATE'
    [[nodiscard]] std::string headerLinePgmRunByDate() const;
    /// @brief Generates the Header line 'COMMENT'
    [[nodiscard]] std::string headerLineComments() const;
    /// @brief Generates the Header line 'MARKER NAME'
    [[nodiscard]] std::string headerLineMarkerName() const;
    /// @brief Generates the Header line 'MARKER NUMBER'
    [[nodiscard]] std::string headerLineMarkerNumber() const;
    /// @brief Generates the Header line 'MARKER TYPE'
    [[nodiscard]] std::string headerLineMarkerType() const;
    /// @brief Generates the Header line 'OBSERVER / AGENCY'
    [[nodiscard]] std::string headerLineObserverAgency() const;
    /// @brief Generates the Header line 'REC # / TYPE / VERS'
    [[nodiscard]] std::string headerLineRecNumTypeVers() const;
    /// @brief Generates the Header line 'ANT # / TYPE'
    [[nodiscard]] std::string headerLineAntNumType() const;
    /// @brief Generates the Header line 'APPROX POSITION XYZ'
    [[nodiscard]] std::string headerLineApproxPositionXYZ() const;
    /// @brief Generates the Header line 'ANTENNA: DELTA H/E/N'
    [[nodiscard]] std::string headerLineAntennaDeltaHEN() const;
    /// @brief Generates the Header line 'SYS / # / OBS TYPES'
    [[nodiscard]] std::string headerLineSysNumObsType() const;
    /// @brief Generates the Header line 'SIGNAL STRENGTH UNIT'
    [[nodiscard]] static std::string headerLineSignalStrengthUnit();
    /// @brief Generates the Header line 'INTERVAL'
    [[nodiscard]] std::string headerLineInterval() const;
    /// @brief Generates the Header line 'TIME OF FIRST OBS'
    [[nodiscard]] std::string headerLineTimeOfFirstObs() const;
    /// @brief Generates the Header line 'TIME OF LAST OBS'
    [[nodiscard]] std::string headerLineTimeOfLastObs() const;
    /// @brief Generates the Header line 'SYS / PHASE SHIFT'
    [[nodiscard]] static std::string headerLineSysPhaseShift();
    /// @brief Generates the Header line 'GLONASS COD/PHS/BIS'
    [[nodiscard]] static std::string headerLineGlonassCodPhsBis();
    /// @brief Generates the Header line 'LEAP SECONDS'
    [[nodiscard]] std::string headerLineLeapSeconds() const;
    /// @brief Generates the Header line '# OF SATELLITES'
    [[nodiscard]] std::string headerLineNumSatellites() const;
    /// @brief Generates the Header line 'END OF HEADER'
    [[nodiscard]] static std::string headerLineEndOfHeader();

    /// @brief Generates the epoch line
    /// @param[in] epochTime Time of the GNSS epoch
    [[nodiscard]] std::string epochRecordLine(const InsTime& epochTime) const;
};

/// @brief Converts the provided struct into a json object
/// @param[out] j Return Json object
/// @param[in] obj Object to convert
void to_json(json& j, const ObsHeader& obj);
/// @brief Converts the provided json object into a struct
/// @param[in] j Json object with the color values
/// @param[out] obj Object to return
void from_json(const json& j, ObsHeader& obj);

/// @brief Converts a character to an ObsType
/// @param[in] c Character for the ObsType
ObsType obsTypeFromChar(char c);

/// @brief Converts an ObsType to char
/// @param[in] type ObsType to convert
char obsTypeToChar(ObsType type);

/// @brief Converts the satellite system(s) to 3 characters representation of the time system
/// @param[in] timeSys Time system
std::string timeSystemString(TimeSystem timeSys);

/// @brief Converts the satellite system(s) to the time system
/// @param[in] satSys Satellite System
TimeSystem timeSystem(SatelliteSystem satSys);

/// @brief Get the Frequency from the provided satellite system and band in the 'SYS / # / OBS TYPES' header
/// @param[in] satSys Satellite System
/// @param[in] band Band (1...9, 0)
[[nodiscard]] Frequency getFrequencyFromBand(SatelliteSystem satSys, int band);

} // namespace vendor::RINEX

/// @brief Converts the enum to a string
/// @param[in] markerType Enum value to convert into text
/// @return String representation of the enum
const char* to_string(vendor::RINEX::ObsHeader::MarkerTypes markerType);
/// @brief Converts the enum to a string tooltip
/// @param[in] markerType Enum value to convert into text
/// @return Tooltip for the enum
const char* tooltip(vendor::RINEX::ObsHeader::MarkerTypes markerType);

} // namespace NAV