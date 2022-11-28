// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file KvhFile.hpp
/// @brief File Reader for Kvh log files
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-16

#pragma once

#include "Nodes/DataProvider/IMU/Imu.hpp"
#include "Nodes/DataProvider/Protocol/FileReader.hpp"

#include "util/Vendor/KVH/KvhUartSensor.hpp"

namespace NAV
{
/// File Reader for Kvh log files
class KvhFile : public Imu, public FileReader
{
  public:
    /// @brief Default constructor
    KvhFile();
    /// @brief Destructor
    ~KvhFile() override;
    /// @brief Copy constructor
    KvhFile(const KvhFile&) = delete;
    /// @brief Move constructor
    KvhFile(KvhFile&&) = delete;
    /// @brief Copy assignment operator
    KvhFile& operator=(const KvhFile&) = delete;
    /// @brief Move assignment operator
    KvhFile& operator=(KvhFile&&) = delete;

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
    constexpr static size_t OUTPUT_PORT_INDEX_KVH_OBS = 0;        ///< @brief Flow (KvhObs)
    constexpr static size_t OUTPUT_PORT_INDEX_HEADER_COLUMNS = 1; ///< @brief Object (std::vector<std::string>)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Polls data from the file
    /// @return The read observation
    [[nodiscard]] std::shared_ptr<const NodeData> pollData();

    /// @brief Determines the type of the file
    /// @return The File Type
    [[nodiscard]] FileType determineFileType() override;

    /// Sensor Object
    vendor::kvh::KvhUartSensor _sensor;

    /// Previous Sequence number to check for order errors
    uint8_t _prevSequenceNumber = UINT8_MAX;
};

} // namespace NAV
