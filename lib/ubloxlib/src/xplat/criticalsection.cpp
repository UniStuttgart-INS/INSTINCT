#include "ub/xplat/criticalsection.hpp"

#include <pthread.h>

namespace ub::xplat
{
struct CriticalSection::Impl
{
    pthread_mutex_t CriticalSection;

    Impl()
    {
    }
};

CriticalSection::CriticalSection()
    : _pi(new Impl())
{
    pthread_mutex_init(&_pi->CriticalSection, NULL);
}

CriticalSection::~CriticalSection()
{
    pthread_mutex_destroy(&_pi->CriticalSection);

    delete _pi;
}

void CriticalSection::enter()
{
    pthread_mutex_lock(&_pi->CriticalSection);
}

void CriticalSection::leave()
{
    pthread_mutex_unlock(&_pi->CriticalSection);
}

} // namespace ub::xplat
