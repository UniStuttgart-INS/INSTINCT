// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Sleep.hpp"
#include <mutex>
#include <condition_variable>
#include "util/Logger.hpp"

#if !__linux__ && !__APPLE__ && !__CYGWIN__ && !__QNXNTO__
    #include <chrono>
    #include <thread>
#else
    #include <unistd.h>
#endif
#include <csignal>

namespace
{
/// Flag for interrupt check
bool usr_interrupt = false;
/// Condition variable for waiting
std::condition_variable cv;
/// Mutex for condition variabel and interrupt flag access
std::mutex cv_m;

void handler(int signum)
{
#if !_WIN32
    switch (signum)
    {
    case SIGUSR1:
        LOG_DEBUG("SIGUSR1 caught");
        break;
    case SIGINT:
        LOG_DEBUG("SIGINT caught");
        break;
    case SIGTERM:
        LOG_DEBUG("SIGTERM caught");
        break;
    default:
        LOG_DEBUG("Unexpected signal caught: {}", signum);
    }
#else
    LOG_DEBUG("Signal caught");
#endif
    {
        std::lock_guard lk(cv_m);
        usr_interrupt = true;
    }
    cv.notify_one();
}
} // namespace

void NAV::Sleep::waitForSignal(bool showText)
{
    LOG_TRACE("called");

#if !_WIN32
    usr_interrupt = false;
    static_cast<void>(signal(SIGUSR1, handler));
    static_cast<void>(signal(SIGINT, handler));
    static_cast<void>(signal(SIGTERM, handler));

    if (showText)
    {
        LOG_INFO("Programm waits for one of the signals: -SIGUSR1 / -SIGINT (Ctrl + c) / -SIGTERM");
    }

    std::unique_lock lk(cv_m);
    cv.wait(lk, [] { return usr_interrupt; });
    LOG_DEBUG("Continuing");

    static_cast<void>(signal(SIGUSR1, SIG_DFL));
    static_cast<void>(signal(SIGINT, SIG_DFL));
    static_cast<void>(signal(SIGTERM, SIG_DFL));
#else
    LOG_ERROR("Waiting for Sigterm is not supported in Windows");
#endif
}

void NAV::Sleep::countDownSeconds(size_t seconds)
{
    LOG_TRACE("called with seconds={}", seconds);

    static_cast<void>(signal(SIGINT, handler));
    static_cast<void>(signal(SIGTERM, handler));

    for (size_t i = 0; i < seconds && !usr_interrupt; i++)
    {
        LOG_INFO("{} seconds till program finishes", seconds - i);

        // Use of system sleep better here, as it interrupts on signal
#if __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
        sleep(1); // NOLINT(concurrency-mt-unsafe) // error: function is not thread safe
#else
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
#endif
    }

    static_cast<void>(signal(SIGINT, SIG_DFL));
    static_cast<void>(signal(SIGTERM, SIG_DFL));
}