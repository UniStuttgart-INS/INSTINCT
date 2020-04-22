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

#include "util/InsTime.hpp"

namespace NAV
{
/// Parent storage Class for all Observations
class InsObs : public NodeData
{
  public:
    /// Time at which the message was received
    std::optional<InsTime> insTime;
};

} // namespace NAV
