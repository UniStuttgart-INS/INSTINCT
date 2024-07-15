// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file KmlLogger.hpp
/// @brief Data Logger for Pos data as KML (Keyhole Markup Language) files (input for Google Earth)
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-06-04

#pragma once

#include <memory>
#include "internal/Node/Node.hpp"
#include "internal/gui/widgets/DynamicInputPins.hpp"
#include "Nodes/DataLogger/Protocol/FileWriter.hpp"
#include <Eigen/src/Core/Matrix.h>

namespace NAV
{
class NodeData;

/// Data Logger for Pos data as KML files (input for Google Earth)
class KmlLogger : public Node, public FileWriter
{
  public:
    /// @brief Default constructor
    KmlLogger();
    /// @brief Destructor
    ~KmlLogger() override;
    /// @brief Copy constructor
    KmlLogger(const KmlLogger&) = delete;
    /// @brief Move constructor
    KmlLogger(KmlLogger&&) = delete;
    /// @brief Copy assignment operator
    KmlLogger& operator=(const KmlLogger&) = delete;
    /// @brief Move assignment operator
    KmlLogger& operator=(KmlLogger&&) = delete;

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

    /// @brief Function to call to add a new pin
    /// @param[in, out] node Pointer to this node
    static void pinAddCallback(Node* node);
    /// @brief Function to call to delete a pin
    /// @param[in, out] node Pointer to this node
    /// @param[in] pinIdx Input pin index to delete
    static void pinDeleteCallback(Node* node, size_t pinIdx);

    /// One data set of positions for each pin in Latitude [deg], Longitude [deg], Height above Mean Sea Level [m]
    std::vector<std::vector<Eigen::Vector3d>> _positionData;

    /// @brief Write Observation to the file
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void writeObservation(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief Dynamic input pins
    /// @attention This should always be the last variable in the header, because it accesses others through the function callbacks
    gui::widgets::DynamicInputPins _dynamicInputPins{ 0, this, pinAddCallback, pinDeleteCallback, 1 };
};

} // namespace NAV
