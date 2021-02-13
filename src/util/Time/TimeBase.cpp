#include "TimeBase.hpp"

#include <chrono>

#include "util/Logger.hpp"

/* -------------------------------------------------------------------------------------------------------- */
/*                                              Private Members                                             */
/* -------------------------------------------------------------------------------------------------------- */

NAV::util::time::Mode timeMode = NAV::util::time::Mode::REAL_TIME;
NAV::InsTime currentTime;

std::chrono::steady_clock::time_point currentTimeComputer;

/* -------------------------------------------------------------------------------------------------------- */
/*                                       Private Function Declarations                                      */
/* -------------------------------------------------------------------------------------------------------- */

namespace NAV::util::time
{
} // namespace NAV::util::time

/* -------------------------------------------------------------------------------------------------------- */
/*                                               Public Members                                             */
/* -------------------------------------------------------------------------------------------------------- */

namespace NAV::util::time
{
} // namespace NAV::util::time

/* -------------------------------------------------------------------------------------------------------- */
/*                                           Function Definitions                                           */
/* -------------------------------------------------------------------------------------------------------- */

void NAV::util::time::SetMode(NAV::util::time::Mode mode)
{
    timeMode = mode;
}

NAV::InsTime NAV::util::time::GetCurrentTime()
{
    if (timeMode == Mode::POST_PROCESSING || currentTime.empty())
    {
        return currentTime;
    }
    // (timeMode == Mode::REAL_TIME)
    auto elapsed = std::chrono::steady_clock::now() - currentTimeComputer;
    return currentTime + elapsed;
}

void NAV::util::time::SetCurrentTime(const NAV::InsTime& insTime)
{
    if (auto currentExactTime = GetCurrentTime();
        insTime < currentExactTime)
    {
        LOG_DATA("Not updating current Time [{} {:.6f}] to [{} {:.6f}], because the new time is earlier.",
                 currentExactTime.toGPSweekTow().gpsWeek, currentExactTime.toGPSweekTow().tow,
                 insTime.toGPSweekTow().gpsWeek, insTime.toGPSweekTow().tow);
    }
    else if (insTime >= currentExactTime)
    {
        if (timeMode == Mode::REAL_TIME)
        {
            LOG_INFO("Updating current Time [{} {:.6f}] to [{} {:.6f}]",
                     currentExactTime.toGPSweekTow().gpsWeek, currentExactTime.toGPSweekTow().tow,
                     insTime.toGPSweekTow().gpsWeek, insTime.toGPSweekTow().tow);
        }
        currentTimeComputer = std::chrono::steady_clock::now();
        currentTime = insTime;
    }
}

void NAV::util::time::ClearCurrentTime()
{
    currentTime = InsTime();
}