/**
 * @file GnssObs.hpp
 * @brief Abstract GNSS Observation Class
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-19
 */

#pragma once

#include "NodeData/InsObs.hpp"

namespace NAV
{
/// Abstract GNSS Observation Class
class GnssObs : public InsObs
{
  public:
    GnssObs() = default;                         ///< Constructor
    ~GnssObs() override = default;               ///< Destructor
    GnssObs(const GnssObs&) = delete;            ///< Copy constructor
    GnssObs(GnssObs&&) = delete;                 ///< Move constructor
    GnssObs& operator=(const GnssObs&) = delete; ///< Copy assignment operator
    GnssObs& operator=(GnssObs&&) = delete;      ///< Move assignment operator

    /**
     * @brief Returns the type of the data class
     * 
     * @retval constexpr std::string_view The data type
     */
    [[nodiscard]] constexpr std::string_view type() const override
    {
        return std::string_view("GnssObs");
    }

    /**
     * @brief Returns the parent types of the data class
     * 
     * @retval std::vector<std::string_view> The parent data types
     */
    [[nodiscard]] std::vector<std::string_view> parentTypes() const override
    {
        std::vector<std::string_view> parents{ "InsObs" };
        return parents;
    }
};
} // namespace NAV
