#include "Sleep.hpp"
#include "util/Logger.hpp"

#if !__linux__ && !__APPLE__ && !__CYGWIN__ && !__QNXNTO__
    #include <chrono>
    #include <thread>
#else
    #include <unistd.h>
#endif
#include <csignal>

/// Flag for interrupt check
static volatile sig_atomic_t usr_interrupt = 0;

void handler(int /* */)
{
    LOG_DEBUG("Signal caught");
    usr_interrupt = 1;
}

void NAV::Sleep::waitForSignal(bool showText)
{
    LOG_TRACE("called");

#if !_WIN32
    usr_interrupt = 0;
    sigset_t mask;
    sigset_t oldmask;
    // Set up the mask of signals to temporarily block.
    sigemptyset(&mask);
    sigaddset(&mask, SIGUSR1);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);
    static_cast<void>(signal(SIGUSR1, handler));
    static_cast<void>(signal(SIGINT, handler));
    static_cast<void>(signal(SIGTERM, handler));

    if (showText)
    {
        LOG_INFO("Programm waits for one of the signals: -SIGUSR1 / -SIGINT (Ctrl + c) / -SIGTERM");
    }

    // Wait for a signal to arrive.
    sigprocmask(SIG_BLOCK, &mask, &oldmask); // NOLINT(concurrency-mt-unsafe) // error: function is not thread safe
    while (!usr_interrupt)
    {
        sigsuspend(&oldmask); // NOLINT(concurrency-mt-unsafe) // error: function is not thread safe
    }
    sigprocmask(SIG_UNBLOCK, &mask, nullptr); // NOLINT(concurrency-mt-unsafe) // error: function is not thread safe

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