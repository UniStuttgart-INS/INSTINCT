// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file CsvLogger.hpp
/// @brief Data Logger for CSV files
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-06-03

#pragma once

#include "internal/Node/Node.hpp"
#include "Nodes/DataLogger/Protocol/FileWriter.hpp"
#include "util/Logger/CommonLog.hpp"

namespace NAV
{
class NodeData;

/// Data Logger for PosVelAtt observations
class CsvLogger : public Node, public FileWriter, public CommonLog
{
  public:
    /// @brief Default constructor
    CsvLogger();
    /// @brief Destructor
    ~CsvLogger() override;
    /// @brief Copy constructor
    CsvLogger(const CsvLogger&) = delete;
    /// @brief Move constructor
    CsvLogger(CsvLogger&&) = delete;
    /// @brief Copy assignment operator
    CsvLogger& operator=(const CsvLogger&) = delete;
    /// @brief Move assignment operator
    CsvLogger& operator=(CsvLogger&&) = delete;

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

    /// @brief Write Observation to the file
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void writeObservation(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// Flag whether the header was written already
    bool _headerWritten = false;
};

} // namespace NAV
