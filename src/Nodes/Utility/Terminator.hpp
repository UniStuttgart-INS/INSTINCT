// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Terminator.hpp
/// @brief Terminator for open signals. Mainly used for test flows
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-01-11

#pragma once

#include <array>
#include "internal/Node/Node.hpp"

#include "Navigation/Time/InsTime.hpp"

#include "internal/gui/widgets/TimeEdit.hpp"
#include "util/Eigen.hpp"

namespace NAV
{
/// Terminator for open signals. Mainly used for test flows
class Terminator : public Node
{
  public:
    /// @brief Default constructor
    Terminator();
    /// @brief Destructor
    ~Terminator() override;
    /// @brief Copy Constructor
    Terminator(const Terminator&) = delete;
    /// @brief Move Constructor
    Terminator(Terminator&&) = delete;
    /// @brief Copy assignment operator
    Terminator& operator=(const Terminator&) = delete;
    /// @brief Move assignment operator
    Terminator& operator=(Terminator&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

  private:
    constexpr static size_t INPUT_PORT_INDEX_FLOW = 0; ///< @brief Flow

    /// @brief Callback when receiving data on a port
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receiveObs(InputPin::NodeDataQueue& queue, size_t pinIdx);
};
} // namespace NAV
