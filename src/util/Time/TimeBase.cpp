// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "TimeBase.hpp"

#include <chrono>
#include <ctime>

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

NAV::util::time::Mode NAV::util::time::GetMode()
{
    return timeMode;
}

void NAV::util::time::SetMode(NAV::util::time::Mode mode)
{
    timeMode = mode;
}

NAV::InsTime NAV::util::time::GetCurrentInsTime()
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
    if (auto currentExactTime = GetCurrentInsTime();
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
            LOG_DATA("Updating current Time [{}] to [{} ]", currentExactTime, insTime);
        }
        currentTimeComputer = std::chrono::steady_clock::now();
        currentTime = insTime;
        LOG_DATA("Updating current Time [{}] to [{} ]", currentExactTime, insTime);
    }
}

void NAV::util::time::SetCurrentTimeToComputerTime()
{
    std::time_t t = std::time(nullptr);
    std::tm* now = std::localtime(&t); // NOLINT(concurrency-mt-unsafe)

    currentTimeComputer = std::chrono::steady_clock::now();
    currentTime = InsTime{ static_cast<uint16_t>(now->tm_year + 1900), static_cast<uint16_t>(now->tm_mon), static_cast<uint16_t>(now->tm_mday),
                           static_cast<uint16_t>(now->tm_hour), static_cast<uint16_t>(now->tm_min), static_cast<long double>(now->tm_sec) };
}

void NAV::util::time::ClearCurrentTime()
{
    currentTime.reset();
}