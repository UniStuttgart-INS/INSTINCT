/**
 * @file InsObs.hpp
 * @brief Parent Class for all Observations
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-11
 */

#pragma once

#include <optional>

#include "NodeData.hpp"

#include "util/notidy/InsTime.hpp"

namespace NAV
{
/// Parent storage Class for all Observations
class InsObs : public NodeData
{
  public:
    InsObs() = default;                        ///< Constructor
    ~InsObs() override = default;              ///< Destructor
    InsObs(const InsObs&) = delete;            ///< Copy constructor
    InsObs(InsObs&&) = delete;                 ///< Move constructor
    InsObs& operator=(const InsObs&) = delete; ///< Copy assignment operator
    InsObs& operator=(InsObs&&) = delete;      ///< Move assignment operator

    /**
     * @brief Returns the type of the data class
     * 
     * @retval constexpr std::string_view The data type
     */
    [[nodiscard]] constexpr std::string_view type() const override
    {
        return std::string_view("InsObs");
    }

    /**
     * @brief Returns the parent types of the data class
     * 
     * @retval std::vector<std::string_view> The parent data types
     */
    [[nodiscard]] std::vector<std::string_view> parentTypes() const override
    {
        return {};
    }

    /// Time at which the message was received
    std::optional<InsTime> insTime;
};

} // namespace NAV
