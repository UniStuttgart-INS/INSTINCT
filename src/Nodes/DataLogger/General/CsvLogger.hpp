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

    /// @brief Called when a new link is to be established
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    /// @return True if link is allowed, false if link is rejected
    bool onCreateLink(OutputPin& startPin, InputPin& endPin) override;

  private:
    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Writes the header
    /// @param[in] obs Observation containing data
    void writeHeader(const std::shared_ptr<const NodeData>& obs);

    /// @brief Rewrites the data file with a new size
    /// @param oldSize Old dynamic size
    /// @param newSize New dynamic size
    /// @param obs Observation containing data
    void rewriteData(size_t oldSize, size_t newSize, const std::shared_ptr<const NodeData>& obs);

    /// @brief Write Observation to the file
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void writeObservation(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// Type last connected
    std::string _lastConnectedType;

    /// Flag whether the header was written already
    bool _headerWritten = false;

    /// Dynamic Header
    std::vector<std::string> _dynamicHeader;

    /// Header which should be logged
    std::vector<std::pair<std::string, bool>> _headerLogging;
    /// Regex to search for when selecting
    std::string _headerLoggingRegex;
    /// Default for new headers
    bool _headerLoggingDefault = true;
    /// Sort headers in the GUI
    bool _headerLoggingSortGui = false;
};

} // namespace NAV
