/**
 * @file InsObs.hpp
 * @brief Parent Class for all Observations
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-11
 */

#pragma once

#include <optional>
#include <string>
#include "NodeData.hpp"

namespace NAV
{
/// Parent storage Class for all Observations
class InsObs : public NodeData
{
  public:
    static constexpr const char* gpsTimeOfWeekDescription = "Seconds since week start";
    static constexpr const char* gpsWeekDescription = "Weeks since 6-Jan-1980";

    /**
     * @brief Seconds since week start in GPS time (without leap seconds)
     * @deprecated This Timestamp should be replaced by an own InsTime implementation
     */
    std::optional<double> gpsTimeOfWeek;

    /**
     * @brief Weeks since 6-Jan-1980 in GPS time (without leap seconds)
     * @deprecated This Timestamp should be replaced by an own InsTime implementation
     */
    std::optional<uint16_t> gpsWeek;
};

} // namespace NAV
