// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RinexObsLogger.hpp
/// @brief Logger for GnssObs to RINEX observation files
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-01-18

#pragma once

#include <set>

#include "internal/Node/Node.hpp"

#include "Nodes/DataLogger/Protocol/FileWriter.hpp"

#include "util/Vendor/RINEX/RINEXUtilities.hpp"

namespace NAV
{

/// Data Logger for GnssObs to RINEX observation files
class RinexObsLogger : public Node, public FileWriter
{
  public:
    /// @brief Default constructor
    RinexObsLogger();
    /// @brief Destructor
    ~RinexObsLogger() override;
    /// @brief Copy constructor
    RinexObsLogger(const RinexObsLogger&) = delete;
    /// @brief Move constructor
    RinexObsLogger(RinexObsLogger&&) = delete;
    /// @brief Copy assignment operator
    RinexObsLogger& operator=(const RinexObsLogger&) = delete;
    /// @brief Move assignment operator
    RinexObsLogger& operator=(RinexObsLogger&&) = delete;

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

    /// @brief Function called by the flow executer after finishing to flush out remaining data
    void flush() override;

  private:
    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Update the file header and eventually rewrite the file
    /// @param[in] oldTimeSys Old time system before this epoch
    void updateFileHeader(TimeSystem oldTimeSys);

    /// @brief Write Observation to the file
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void writeObservation(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Supported RINEX versions
    static inline const std::set<double> _supportedVersions = { 3.04 };

    /// Header information
    vendor::RINEX::ObsHeader _header;
};

} // namespace NAV
