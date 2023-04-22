// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Delay.hpp
/// @brief Delay Node
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-02-01

#pragma once

#include "internal/Node/Node.hpp"

#include "util/Eigen.hpp"

#include <deque>

namespace NAV::experimental
{
/// @brief Delays messages by buffering them
class Delay : public Node
{
  public:
    /// @brief Default constructor
    Delay();
    /// @brief Destructor
    ~Delay() override;
    /// @brief Copy constructor
    Delay(const Delay&) = delete;
    /// @brief Move constructor
    Delay(Delay&&) = delete;
    /// @brief Copy assignment operator
    Delay& operator=(const Delay&) = delete;
    /// @brief Move assignment operator
    Delay& operator=(Delay&&) = delete;

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

    /// @brief Called when a new link is to be established
    /// @param[in] startPin Pin where the link starts
    /// @param[in] endPin Pin where the link ends
    /// @return True if link is allowed, false if link is rejected
    bool onCreateLink(OutputPin& startPin, InputPin& endPin) override;

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_FLOW = 0; ///< @brief Flow
    constexpr static size_t INPUT_PORT_INDEX_FLOW = 0;  ///< @brief Flow

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Delays the observation
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void delayObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// @brief The amount to delay messages for
    int _delayLength = 1;

    /// @brief Buffer to delay data
    std::deque<std::shared_ptr<const NodeData>> _buffer;
};

} // namespace NAV::experimental
