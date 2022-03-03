/// @file Sleep.hpp
/// @brief Class to catch system signals and sleep
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-03-10

#pragma once

#include <cstddef>

namespace NAV::Sleep
{
/// @brief Wait the thread till sigusr signal is send
void waitForSignal(bool showText = false);

/// @brief Wait the thread till time passes
/// @param[in] seconds Time to sleep in seconds
void countDownSeconds(size_t seconds);

} // namespace NAV::Sleep
