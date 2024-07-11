// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file DynamicInputPins.hpp
/// @brief Inputs pins which can be added dynamically
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-12-21

#pragma once

#include <vector>
#include <string>
#include <functional>

#include "internal/Node/Node.hpp"
#include "internal/Node/Pin.hpp"

#include "util/Json.hpp"

namespace NAV::gui::widgets
{
/// @brief Inputs pins which can be added dynamically
struct DynamicInputPins
{
    /// Information to create extra columns
    struct ExtraColumn
    {
        /// Column header text
        std::string header;
        /// Function to create the column content. Argument is the pin index. Returns true if changes occurred.
        std::function<bool(size_t)> content;
    };

    /// @brief Constructor
    /// @param[in] firstDynamicPinIndex First pin index which is dynamic
    /// @param[in, out] node Pointer to the calling node (needs to be valid only during construction)
    /// @param[in] pinAddCallback Function to call to add a new pin
    /// @param[in] pinDeleteCallback Function to call to delete a pin
    /// @param[in] defaultInputPins Default value for the input pins
    DynamicInputPins(size_t firstDynamicPinIndex,
                     Node* node,
                     std::function<void(Node*)> pinAddCallback,
                     std::function<void(Node*, size_t)> pinDeleteCallback,
                     size_t defaultInputPins = 0);

    /// @brief Shows the GUI input to select the options
    /// @param[in] id Unique id for ImGui.
    /// @param[in, out] inputPins Input Pins of the node
    /// @param[in, out] node Pointer to the calling node
    /// @param[in] extraColumns Extra columns to display in the table
    /// @return True when changes occurred
    bool ShowGuiWidgets(size_t id, std::vector<InputPin>& inputPins, Node* node, const std::vector<ExtraColumn>& extraColumns = {});

    /// @brief Get the number Of dynamic pins
    [[nodiscard]] size_t getNumberOfDynamicPins() const;

    /// @brief Adds a pin and call the pinAddCallback
    /// @param[in, out] node Pointer to the calling node
    void addPin(Node* node);

    /// @brief Set the First Dynamic Pin Idx
    /// @param[in] firstDynamicPinIndex First pin index which is dynamic
    void setFirstDynamicPinIdx(size_t firstDynamicPinIndex) { _firstDynamicPinIdx = firstDynamicPinIndex; }

    /// @brief Get the First Dynamic Pin Idx
    [[nodiscard]] size_t getFirstDynamicPinIdx() const { return _firstDynamicPinIdx; }

  private:
    /// @brief Index of the Pin currently being dragged
    int _dragAndDropPinIndex = -1;
    /// @brief First pin index which is dynamic
    size_t _firstDynamicPinIdx = 0;
    /// @brief Number of dynamic input pins
    size_t _nDynamicInputPins = 0;
    /// @brief Function to call to add a new pin
    std::function<void(Node*)> _pinAddCallback;
    /// @brief Function to call to delete a pin. Argument is the pin index.
    std::function<void(Node*, size_t)> _pinDeleteCallback;

    friend void to_json(json& j, const DynamicInputPins& obj);
    friend void from_json(const json& j, DynamicInputPins& obj, Node* node);
};

/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
void to_json(json& j, const DynamicInputPins& obj);
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
/// @param[in, out] node Pointer to the node calling this
void from_json(const json& j, DynamicInputPins& obj, Node* node);

} // namespace NAV::gui::widgets
