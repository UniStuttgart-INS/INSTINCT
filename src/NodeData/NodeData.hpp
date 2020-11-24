/// @file NodeData.hpp
/// @brief Abstract NodeData Class
/// @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
/// @date 2020-04-16

#pragma once

#include <string_view>
#include <vector>

namespace NAV
{
class NodeData
{
  public:
    /// @brief Default constructor
    NodeData() = default;
    /// @brief Destructor
    virtual ~NodeData() = default;
    /// @brief Copy constructor
    NodeData(const NodeData&) = delete;
    /// @brief Move constructor
    NodeData(NodeData&&) = delete;
    /// @brief Copy assignment operator
    NodeData& operator=(const NodeData&) = delete;
    /// @brief Move assignment operator
    NodeData& operator=(NodeData&&) = delete;

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static constexpr std::string_view type() { return ""; }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string_view> parentTypes() { return {}; }
};

} // namespace NAV
