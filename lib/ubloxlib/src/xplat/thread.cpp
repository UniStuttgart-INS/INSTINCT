#include "ub/xplat/thread.hpp"

#include <pthread.h>
#include <unistd.h>
#include <stdexcept>

using namespace std;

namespace ub::xplat
{
struct Thread::Impl
{
    pthread_t ThreadHandle;
    void* Data;

    Thread::ThreadStartRoutine StartRoutine;

    Impl()
        : ThreadHandle(0),
          Data(NULL),
          StartRoutine(NULL)
    {
    }

    static void* StartRoutineWrapper(void* data)
    {
        Impl* impl = (Impl*)data;

        impl->StartRoutine(impl->Data);

        return NULL;
    }
};

Thread::Thread(
    ThreadStartRoutine startRoutine)
    : _pimpl(new Thread::Impl())
{
    _pimpl->StartRoutine = startRoutine;
}

Thread::~Thread()
{
    delete _pimpl;
}

Thread* Thread::startNew(ThreadStartRoutine startRoutine, void* routineData)
{
    Thread* newThread = new Thread(startRoutine);

    newThread->start(routineData);

    return newThread;
}

void Thread::start(void* routineData)
{
    if (_pimpl->StartRoutine != NULL)
    {
        _pimpl->Data = routineData;

        int errorCode = pthread_create(
            &_pimpl->ThreadHandle,
            NULL,
            Impl::StartRoutineWrapper,
            _pimpl);

        if (errorCode != 0)
            throw std::runtime_error("Could not start thread");
    }
    else
    {
        throw std::logic_error("Not implemented");
    }
}

void Thread::join()
{
    if (pthread_join(
            _pimpl->ThreadHandle,
            NULL))
        throw std::runtime_error("Could not join");
}

void Thread::sleepSec(uint32_t numOfSecsToSleep)
{
    sleep(numOfSecsToSleep);
}

void Thread::sleepMs(uint32_t numOfMsToSleep)
{
    usleep(numOfMsToSleep * 1000);
}

void Thread::sleepUs(uint32_t numOfUsToSleep)
{
    usleep(numOfUsToSleep);
}

void Thread::sleepNs(uint32_t /*numOfNsToSleep*/)
{
    throw std::logic_error("Not implemented");
}

} // namespace ub::xplat