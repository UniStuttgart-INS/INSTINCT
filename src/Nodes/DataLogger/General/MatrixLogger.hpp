// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file MatrixLogger.hpp
/// @brief Data Logger for matrices
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-06-27

#pragma once

#include "internal/Node/Node.hpp"
#include "Nodes/DataLogger/Protocol/FileWriter.hpp"
#include "Nodes/DataLogger/Protocol/CommonLog.hpp"

namespace NAV
{
class NodeData;

/// Data Logger for PosVelAtt observations
class MatrixLogger : public Node, public FileWriter, public CommonLog
{
  public:
    /// @brief Default constructor
    MatrixLogger();
    /// @brief Destructor
    ~MatrixLogger() override;
    /// @brief Copy constructor
    MatrixLogger(const MatrixLogger&) = delete;
    /// @brief Move constructor
    MatrixLogger(MatrixLogger&&) = delete;
    /// @brief Copy assignment operator
    MatrixLogger& operator=(const MatrixLogger&) = delete;
    /// @brief Move assignment operator
    MatrixLogger& operator=(MatrixLogger&&) = delete;

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
    constexpr static size_t INPUT_PORT_INDEX_MATRIX = 0; ///< @brief Matrix

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Write a matrix to the file
    /// @param[in] insTime Time the data was received
    /// @param[in] pinIdx Index of the pin the data is received on
    void writeMatrix(const InsTime& insTime, size_t pinIdx);

    /// Flag whether the header was written
    bool _headerWritten = false;
};

} // namespace NAV
