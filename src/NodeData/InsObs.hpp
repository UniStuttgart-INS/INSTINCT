/**
 * @file InsObs.hpp
 * @brief Parent Class for all Observations
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-11
 */

#pragma once

#include <optional>

namespace NAV
{
/// Parent storage Class for all Observations
class InsObs
{
  public:
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
