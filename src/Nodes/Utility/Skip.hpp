// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Skip.hpp
/// @brief Limits measurement data from any source to a user-defined Skip
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2022-10-27

#pragma once

#include <array>
#include "internal/Node/Node.hpp"

#include "Navigation/Time/InsTime.hpp"

#include "internal/gui/widgets/TimeEdit.hpp"
#include "util/Eigen.hpp"

namespace NAV
{
/// Limits measurement data from any source to a user-defined Skip
class Skip : public Node
{
  public:
    /// @brief Default constructor
    Skip();
    /// @brief Destructor
    ~Skip() override;
    /// @brief Copy Constructor
    Skip(const Skip&) = delete;
    /// @brief Move Constructor
    Skip(Skip&&) = delete;
    /// @brief Copy assignment operator
    Skip& operator=(const Skip&) = delete;
    /// @brief Move assignment operator
    Skip& operator=(Skip&&) = delete;

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

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_FLOW = 0; ///< @brief Flow
    constexpr static size_t INPUT_PORT_INDEX_FLOW = 0;  ///< @brief Flow

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Called when a new link was established
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    void afterCreateLink(OutputPin& startPin, InputPin& endPin) override;

    /// @brief Called when a link was deleted
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    void afterDeleteLink(OutputPin& startPin, InputPin& endPin) override;

    /// @brief Callback when receiving data on a port
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receiveObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// Retain every Nth message
    size_t _retainEvery = 10;

    /// Current retain counter
    size_t _retainCounter = 0;
};
} // namespace NAV
