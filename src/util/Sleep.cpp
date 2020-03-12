#include "Sleep.hpp"

/// Flag for interrupt check
static volatile sig_atomic_t usr_interrupt = 0;

void NAV::Sleep::handler(int /* */)
{
    LOG_DEBUG("Signal caught");
    usr_interrupt = 1;
}

void NAV::Sleep::waitForSignal()
{
    usr_interrupt = 0;
    sigset_t mask, oldmask;
    /* Set up the mask of signals to temporarily block. */
    sigemptyset(&mask);
    sigaddset(&mask, SIGUSR1);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);
    signal(SIGUSR1, handler);
    signal(SIGINT, handler);
    signal(SIGTERM, handler);

    LOG_INFO("Programm waits for one of the signals: -SIGUSR1 / -SIGINT (Ctrl + c) / -SIGTERM");

    /* Wait for a signal to arrive. */
    sigprocmask(SIG_BLOCK, &mask, &oldmask);
    while (!usr_interrupt)
        sigsuspend(&oldmask);
    sigprocmask(SIG_UNBLOCK, &mask, NULL);
}

void NAV::Sleep::countDownSeconds(size_t seconds)
{
    for (size_t i = 0; i < seconds; i++)
    {
        LOG_INFO("{} seconds till program finishes", seconds - i);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}