/**
 * @file Sleep.hpp
 * @brief Class to catch system signals and sleep
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-10
 */

#pragma once

#include <cstddef>

namespace NAV
{
/// @brief Class to catch system signals and sleep
class Sleep
{
  public:
    /// @brief Wait the thread till sigusr signal is send
    static void waitForSignal();

    /**
     * @brief Wait the thread till time passes
     * 
     * @param[in] seconds Time to sleep in seconds
     */
    static void countDownSeconds(size_t seconds);

  private:
    /// @brief Constructor is defined private, so creating an instance of this object is imposible
    Sleep(){};

    /// @brief Handler for Signal interrupts
    static void handler(int /* */);
};

} // namespace NAV
