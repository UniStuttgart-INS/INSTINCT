#include "ub/xplat/ubtime.hpp"

#include <time.h>
#include <sys/time.h>
#include <cstdint>
#include <stdexcept>

using namespace std;

namespace ub::xplat
{
TimeStamp::TimeStamp()
    : _sec(0), _usec(0) {}

TimeStamp::TimeStamp(int64_t sec, uint64_t usec)
    : _sec(sec), _usec(usec) {}

TimeStamp TimeStamp::get()
{
    struct timeval tv;

    gettimeofday(&tv, NULL);

    return TimeStamp(tv.tv_sec, tv.tv_usec);
}

struct Stopwatch::Impl
{
    double _clockStart;

    Impl()
        : _clockStart(-1)
    {
        // Start the stopwatch.
        reset();
    }

    void reset()
    {
        struct timespec time;
        int error;

        error = clock_gettime(CLOCK_MONOTONIC, &time);

        if (error)
            throw std::runtime_error("Could not get time");

        _clockStart = (time.tv_sec * 1000.0) + (time.tv_nsec / 1000000.0);
    }

    float elapsedMs()
    {
        struct timespec time;
        int error;

        if (_clockStart < 0)
            throw std::logic_error("Clock not started");

        error = clock_gettime(CLOCK_MONOTONIC, &time);

        if (error)
            throw std::runtime_error("Could not get time");

        return (time.tv_sec * 1000.0) + (time.tv_nsec / 1000000.0) - _clockStart;
    }
};

Stopwatch::Stopwatch()
    : _pi(new Impl())
{
}

Stopwatch::~Stopwatch()
{
    delete _pi;
}

void Stopwatch::reset()
{
    _pi->reset();
}

float Stopwatch::elapsedMs()
{
    return _pi->elapsedMs();
}

} // namespace ub::xplat
