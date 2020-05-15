/**
 * @file NodeData.hpp
 * @brief Abstract NodeData Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-04-16
 */

#pragma once

#include <string_view>
#include <vector>

namespace NAV
{
class NodeData
{
  public:
    NodeData() = default;                          ///< Constructor
    virtual ~NodeData() = default;                 ///< Destructor
    NodeData(const NodeData&) = delete;            ///< Copy constructor
    NodeData(NodeData&&) = delete;                 ///< Move constructor
    NodeData& operator=(const NodeData&) = delete; ///< Copy assignment operator
    NodeData& operator=(NodeData&&) = delete;      ///< Move assignment operator

    /**
     * @brief Returns the type of the data class
     * 
     * @retval constexpr std::string_view The data type
     */
    [[nodiscard]] virtual constexpr std::string_view type() const = 0;

    /**
     * @brief Returns the parent types of the data class
     * 
     * @retval std::vector<std::string_view> The parent data types
     */
    [[nodiscard]] virtual std::vector<std::string_view> parentTypes() const = 0;
};

} // namespace NAV
