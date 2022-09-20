// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "timestamp.hpp"

#ifndef HAS_UARTSENSOR_LIBRARY

    #if _WIN32
        #include <Windows.h>
    #elif __linux__ || __CYGWIN__ || __QNXNTO__
        #include <ctime>
        #include <sys/time.h>
    #elif __APPLE__
        #include <sys/time.h>
        #include <mach/clock.h>
        #include <mach/mach.h>
    #else
        #error "Unknown System"
    #endif

    #include <stdexcept>

namespace uart::xplat
{
TimeStamp::TimeStamp(int64_t sec, uint64_t usec)
    : _sec(sec), _usec(usec) {}

TimeStamp TimeStamp::get()
{
    #if _WIN32

    // HACK: Just returning an empty TimeStamp for now.
    return TimeStamp();

    #elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__

    struct timeval tv
    {};

    gettimeofday(&tv, nullptr);

    return TimeStamp(tv.tv_sec, static_cast<uint64_t>(tv.tv_usec));

    #else
        #error "Unknown System"
    #endif
}

} // namespace uart::xplat

#endif