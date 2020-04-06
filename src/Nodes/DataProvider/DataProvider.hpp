/**
 * @file DataProvider.hpp
 * @brief Abstract Class for Data Providers
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-11
 */

#pragma once

#include "NodeData/InsObs.hpp"

#include <memory>

namespace NAV
{
/// Abstract Class for Data Providers
class DataProvider
{
  public:
    /**
     * @brief Poll the next/current observation
     * 
     * @attention Has to call the observationReceivedCallback handlers
     * @retval std::shared_ptr<InsObs> The polled observation
     */
    virtual std::shared_ptr<InsObs> pollObservation() = 0;

    /// Default Destructor
    virtual ~DataProvider(){};

  protected:
    /// Default Constructor
    DataProvider() {}
};

} // namespace NAV
