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
    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Determines the type of the file
    /// @return The File Type
    [[nodiscard]] FileType determineFileType() override;

    /// @brief Read the Header of the file
    void readHeader() override;

    /// @brief Read the orbit information
    void readOrbits();

    /// @brief Supported RINEX versions
    static inline const std::set<double> _supportedVersions = { 3.05, 3.04, 3.03, 3.02, 2.11, 2.10, 2.01 }; // TODO version 4.00

    /// @brief Data object to share over the output pin
    GnssNavInfo _gnssNavInfo;

    /// @brief Version of the RINEX file
    double _version = 0.0;
};

} // namespace NAV
