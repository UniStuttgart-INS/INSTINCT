// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file UbloxFile.hpp
/// @brief File Reader for ubx files
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-16

#pragma once

#include "internal/Node/Node.hpp"
#include "Nodes/DataProvider/Protocol/FileReader.hpp"

#include "util/Vendor/Ublox/UbloxUartSensor.hpp"

namespace NAV
{
/// File Reader for Ublox log files
class UbloxFile : public Node, public FileReader
{
  public:
    /// @brief Default constructor
    UbloxFile();
    /// @brief Destructor
    ~UbloxFile() override;
    /// @brief Copy constructor
    UbloxFile(const UbloxFile&) = delete;
    /// @brief Move constructor
    UbloxFile(UbloxFile&&) = delete;
    /// @brief Copy assignment operator
    UbloxFile& operator=(const UbloxFile&) = delete;
    /// @brief Move assignment operator
    UbloxFile& operator=(UbloxFile&&) = delete;

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
    constexpr static size_t OUTPUT_PORT_INDEX_UBLOX_OBS = 0; ///< @brief Flow (UbloxObs)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Polls data from the file
    /// @param[in] peek Specifies if the data should be peeked (without moving the read cursor) or read
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<const NodeData> pollData(bool peek = false);

    /// @brief Determines the type of the file
    /// @return The File Type
    [[nodiscard]] FileType determineFileType() override;

    /// Sensor Object
    vendor::ublox::UbloxUartSensor _sensor;
};

} // namespace NAV
