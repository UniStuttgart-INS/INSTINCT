// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RinexNavFile.hpp
/// @brief File reader for RINEX Navigation messages
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-04-21

#pragma once

#include <set>

#include "internal/Node/Node.hpp"
#include "Nodes/DataProvider/Protocol/FileReader.hpp"

#include "NodeData/GNSS/GnssNavInfo.hpp"

namespace NAV
{
/// File reader Node for RINEX Navigation messages
class RinexNavFile : public Node, public FileReader
{
  public:
    /// @brief Default constructor
    RinexNavFile();
    /// @brief Destructor
    ~RinexNavFile() override;
    /// @brief Copy constructor
    RinexNavFile(const RinexNavFile&) = delete;
    /// @brief Move constructor
    RinexNavFile(RinexNavFile&&) = delete;
    /// @brief Copy assignment operator
    RinexNavFile& operator=(const RinexNavFile&) = delete;
    /// @brief Move assignment operator
    RinexNavFile& operator=(RinexNavFile&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

    /// @brief Resets the node. Moves the read cursor to the start
    bool resetNode() override;

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_GNSS_NAV_INFO = 0; ///< @brief Object (GnssNavInfo)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Determines the type of the file
    /// @return The File Type
    [[nodiscard]] FileType determineFileType() override;

    /// @brief Read the header of the file with correct version
    /// @param[in] version RINEX version
    void executeHeaderParser(double version);

    /// @brief Aborts RINEX file reading and deinitializes node
    /// @return True if deinitialization successful
    inline auto abortReading()
    {
        LOG_ERROR("{}: The file '{}' is corrupt in line {}.", nameId(), _path, getCurrentLineNumber());
        _gnssNavInfo.reset();
        doDeinitialize();
    };

    /// @brief Read the messages of the file with correct version
    /// @param[in] version RINEX version
    void executeOrbitParser(double version);

    /// @brief Parses RINEX version 2.* headers
    void parseHeader2();

    /// @brief Parses RINEX version 3.* headers
    void parseHeader3();

    /// @brief Parses RINEX version 4.00 headers
    void parseHeader4();

    /// @brief Parses RINEX version 2.* messages
    void parseOrbit2();

    /// @brief Parses RINEX version 3.* messages
    void parseOrbit3();

    /// @brief Parses RINEX version 4.00 messages
    void parseOrbit4();

    /// @brief Parses ephemeris message since version 3
    /// @param[in] line string
    /// @param[in] satSys Satellite System
    /// @param[in] satNum uint8_t Satellite Number
    /// @return False if message should be skipped
    bool parseEphemeris(std::string& line, SatelliteSystem satSys, uint8_t satNum);

    /// @brief Read the Header of the file
    void readHeader() override;

    /// @brief Read the orbit information
    void readOrbits();

    /// @brief Supported RINEX versions
    static inline const std::set<double> _supportedVersions = { 4.00, 3.05, 3.04, 3.03, 3.02, 2.11, 2.10, 2.01 };

    /// @brief RINEX navigation message types enumeration with continuous range
    enum class NavMsgType
    {
        EPH,    ///< Ephemeris
        STO,    ///< System Time Offset
        EOP,    ///< Earth Orientation Parameter
        ION,    ///< Ionosphere
        UNKNOWN ///< Unknown message type
    };

    /// @brief Converts RINEX navigation message string to enum type
    /// @param[in] type string
    /// @return navMsgType enum
    static inline NavMsgType getNavMsgType(const std::string& type)
    {
        NavMsgType navMsgType = NavMsgType::UNKNOWN;
        if (type == "EPH")
        {
            navMsgType = NavMsgType::EPH;
        }
        else if (type == "STO")
        {
            navMsgType = NavMsgType::STO;
        }
        else if (type == "EOP")
        {
            navMsgType = NavMsgType::EOP;
        }
        else if (type == "ION")
        {
            navMsgType = NavMsgType::ION;
        }
        return navMsgType;
    }

    /// @brief Data object to share over the output pin
    GnssNavInfo _gnssNavInfo;

    /// @brief Version of the RINEX file
    double _version = 0.0;
};

} // namespace NAV
