#include "uart/xplat/thread.hpp"

#include <stdexcept>

#if _WIN32
    #include <Windows.h>
#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
    #include <pthread.h>
    #include <unistd.h>
#else
    #error "Unknown System"
#endif

namespace uart::xplat
{
struct Thread::Impl
{
#if _WIN32
    HANDLE ThreadHandle{ nullptr };
#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
    pthread_t ThreadHandle{ 0 };
    void* Data{ nullptr };
#else
    #error "Unknown System"
#endif

    Thread::ThreadStartRoutine StartRoutine{ nullptr };

    Impl() = default;

#if __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__

    static void* StartRoutineWrapper(void* data)
    {
        Impl* impl = static_cast<Impl*>(data);

        impl->StartRoutine(impl->Data);

        return nullptr;
    }

#endif
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
    auto* newThread = new Thread(startRoutine); // NOLINT

    newThread->start(routineData);

    return newThread;
}

void Thread::start(void* routineData)
{
    if (_pimpl->StartRoutine != nullptr)
    {
#if _WIN32

        _pimpl->ThreadHandle = CreateThread(
            nullptr,
            0,
            (LPTHREAD_START_ROUTINE)_pimpl->StartRoutine,
            routineData,
            0,
            nullptr);

        if (_pimpl->ThreadHandle == nullptr)
        {
            throw unknown_error();
        }

#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__

        _pimpl->Data = routineData;

        int errorCode = pthread_create(
            &_pimpl->ThreadHandle,
            nullptr,
            Impl::StartRoutineWrapper,
            _pimpl);

        if (errorCode != 0)
        {
            throw std::runtime_error("Could not start thread");
        }

#else
    #error "Unknown System"
#endif
    }
    else
    {
        throw std::logic_error("Not implemented");
    }
}

void Thread::join()
{
#if _WIN32

    WaitForSingleObject(
        _pimpl->ThreadHandle,
        INFINITE);

#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__

    if (pthread_join(_pimpl->ThreadHandle, nullptr))
    {
        throw std::runtime_error("Could not join");
    }

#else
    #error "Unknown System"
#endif
}

void Thread::sleepSec(uint32_t numOfSecsToSleep)
{
#if _WIN32
    Sleep(numOfSecsToSleep * 1000);
#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
    sleep(numOfSecsToSleep);
#else
    #error "Unknown System"
#endif
}

void Thread::sleepMs(uint32_t numOfMsToSleep)
{
#if _WIN32
    Sleep(numOfMsToSleep);
#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
    usleep(numOfMsToSleep * 1000);
#else
    #error "Unknown System"
#endif
}

void Thread::sleepUs(uint32_t numOfUsToSleep)
{
#if _WIN32
    throw not_implemented();
#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
    usleep(numOfUsToSleep);
#else
    #error "Unknown System"
#endif
}

void Thread::sleepNs(uint32_t /*numOfNsToSleep*/)
{
#if _WIN32
    throw std::logic_error("Not implemented");
#elif __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
    throw std::logic_error("Not implemented");
#endif
}

} // namespace uart::xplat