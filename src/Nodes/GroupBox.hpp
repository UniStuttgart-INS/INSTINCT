/// @file Groupbox.hpp
/// @brief Group Box
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-12-28

#pragma once

#include "Node.hpp"

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

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set hasConfig to true in the constructor of the node
    void config() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;
};

} // namespace NAV
