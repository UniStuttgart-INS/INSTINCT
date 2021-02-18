/// @file TimeBase.hpp
/// @brief Keeps track of the current real/simulation time
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-01-28

#pragma once

#include "util/InsTime.hpp"

namespace NAV::util::time
{
enum class Mode
{
    REAL_TIME,       //< Computer clock will be added to last time update
    POST_PROCESSING, //< Time will be set by FlowExecutor only
};

/// @brief Set the time mode
/// @param[in] mode Real time or postprocessing
void SetMode(Mode mode);

/// @brief Get the time mode
Mode GetMode();

/// @brief Get the current time.
/// @return Pointer to the current time or nullptr if it is not known yet.
InsTime GetCurrentTime();

/// @brief Set the current time object
/// @param[in] insTime The new current time
void SetCurrentTime(const InsTime& insTime);

/// @brief Clears the current time object
void ClearCurrentTime();

} // namespace NAV::util::time
