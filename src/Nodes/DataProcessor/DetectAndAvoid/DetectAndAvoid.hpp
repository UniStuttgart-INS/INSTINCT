// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file DetectAndAvoid.hpp
/// @brief Detect-And-Avoid class
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2024-10-25

#pragma once

#include <cstddef>
#include "internal/Node/Node.hpp"
#include "NodeData/State/PosVelAtt.hpp"

namespace NAV
{
/// @brief Detect-And-Avoid class
class DetectAndAvoid : public Node
{
  public:
    /// @brief Default constructor
    DetectAndAvoid();
    /// @brief Destructor
    ~DetectAndAvoid() override;
    /// @brief Copy constructor
    DetectAndAvoid(const DetectAndAvoid&) = delete;
    /// @brief Move constructor
    DetectAndAvoid(DetectAndAvoid&&) = delete;
    /// @brief Copy assignment operator
    DetectAndAvoid& operator=(const DetectAndAvoid&) = delete;
    /// @brief Move assignment operator
    DetectAndAvoid& operator=(DetectAndAvoid&&) = delete;
    /// @brief String representation of the class type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the class type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the class category
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
    constexpr static size_t INPUT_PORT_INDEX_POS_VEL_ATT = 0; ///< @brief Flow (PosVelAtt)
    constexpr static size_t OUTPUT_PORT_INDEX_SOLUTION = 0;   ///< @brief Flow (DaaSolution)

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Invoke the callback with a PosVelAtt solution
    /// @param[in] posVelAtt PosVelAtt solution
    void invokeCallbackWithPosVelAtt(const PosVelAtt& posVelAtt);

    /// @brief Receive Function for the PosVelAtt observation
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvPosVelAtt(InputPin::NodeDataQueue& queue, size_t pinIdx);
};
} // namespace NAV