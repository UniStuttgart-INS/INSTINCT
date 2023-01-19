// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file TightlyCoupledKF.hpp
/// @brief Kalman Filter class for the tightly coupled INS/GNSS integration
/// @author M. Maier (marcel.maier@ins.uni-stuttgart.de)
/// @date 2023-01-18

#pragma once

#include "internal/Node/Node.hpp"
#include "Navigation/Time/InsTime.hpp"
#include "NodeData/State/InertialNavSol.hpp"

#include "Navigation/Math/KalmanFilter.hpp"

namespace NAV
{
/// @brief Tightly-coupled Kalman Filter for INS/GNSS integration
class TightlyCoupledKF : public Node
{
  public:
    /// @brief Default constructor
    TightlyCoupledKF();
    /// @brief Destructor
    ~TightlyCoupledKF() override;
    /// @brief Copy constructor
    TightlyCoupledKF(const TightlyCoupledKF&) = delete;
    /// @brief Move constructor
    TightlyCoupledKF(TightlyCoupledKF&&) = delete;
    /// @brief Copy assignment operator
    TightlyCoupledKF& operator=(const TightlyCoupledKF&) = delete;
    /// @brief Move assignment operator
    TightlyCoupledKF& operator=(TightlyCoupledKF&&) = delete;
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
    /// @param j Json object with the node state
    void restore(const json& j) override;

  private:
    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Receive Function for the inertial navigation solution
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void recvInertialNavigationSolution(InputPin::NodeDataQueue& queue, size_t pinIdx);
};

} // namespace NAV