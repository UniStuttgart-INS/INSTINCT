// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GroupBox.hpp
/// @brief Group Box
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-12-28

#pragma once

#include "internal/Node/Node.hpp"

namespace NAV
{
/// Group Box
class GroupBox : public Node
{
  public:
    /// @brief Default constructor
    GroupBox();
    /// @brief Destructor
    ~GroupBox() override;
    /// @brief Copy constructor
    GroupBox(const GroupBox&) = delete;
    /// @brief Move constructor
    GroupBox(GroupBox&&) = delete;
    /// @brief Copy assignment operator
    GroupBox& operator=(const GroupBox&) = delete;
    /// @brief Move assignment operator
    GroupBox& operator=(GroupBox&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;
};

} // namespace NAV
