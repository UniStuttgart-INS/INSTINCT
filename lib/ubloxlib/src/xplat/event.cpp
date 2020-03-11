#include "ub/xplat/event.hpp"

#include <stdexcept>
#include <errno.h>
#include <pthread.h>
#include <time.h>

using namespace std;

namespace ub::xplat
{
struct Event::Impl
{
    pthread_mutex_t Mutex;
    pthread_cond_t Condition;
    bool IsTriggered;

    Impl()
        : IsTriggered(false)
    {
        pthread_mutex_init(&Mutex, NULL);

        pthread_cond_init(&Condition, NULL);
    }
};

Event::Event()
    : _pi(new Impl())
{
}

Event::~Event()
{
    delete _pi;
}

void Event::wait()
{
    pthread_mutex_lock(&_pi->Mutex);

    int errorCode = pthread_cond_wait(
        &_pi->Condition,
        &_pi->Mutex);

    pthread_mutex_unlock(&_pi->Mutex);

    if (errorCode == 0)
        return;

    throw std::runtime_error("Wait error");
}

Event::WaitResult Event::waitUs(uint32_t timeoutInMicroSec)
{
    pthread_mutex_lock(&_pi->Mutex);

    timespec now;
    clock_gettime(CLOCK_REALTIME, &now);

    uint32_t numOfSecs = timeoutInMicroSec / 1000000;
    uint32_t numOfNanoseconds = (timeoutInMicroSec % 1000000) * 1000;

    now.tv_sec += numOfSecs;
    now.tv_nsec += numOfNanoseconds;

    if (now.tv_nsec > 1000000000)
    {
        now.tv_nsec %= 1000000000;
        now.tv_sec++;
    }

    int errorCode = pthread_cond_timedwait(
        &_pi->Condition,
        &_pi->Mutex,
        &now);

    pthread_mutex_unlock(&_pi->Mutex);

    if (errorCode == 0)
        return WAIT_SIGNALED;

    if (errorCode == ETIMEDOUT)
        return WAIT_TIMEDOUT;

    throw std::runtime_error("Wait error");
}

Event::WaitResult Event::waitMs(uint32_t timeoutInMs)
{
    return waitUs(timeoutInMs * 1000);

    throw std::runtime_error("Wait error");
}

void Event::signal()
{
    pthread_mutex_lock(&_pi->Mutex);

    _pi->IsTriggered = true;

    pthread_cond_signal(&_pi->Condition);

    pthread_mutex_unlock(&_pi->Mutex);
}

} // namespace ub::xplat