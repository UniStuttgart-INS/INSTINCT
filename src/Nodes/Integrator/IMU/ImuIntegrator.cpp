#include "ImuIntegrator.hpp"

#include "util/Logger.hpp"

NAV::ImuIntegrator::ImuIntegrator(std::string name)
    : Integrator(name) {}

NAV::ImuIntegrator::~ImuIntegrator()
{
    deinitialize();
}

NAV::NavStatus NAV::ImuIntegrator::initialize()
{
    LOG_TRACE("called for {}", name);

    if (initialized)
    {
        LOG_WARN("{} already initialized!!!", name);
        return NavStatus::NAV_WARNING_ALREADY_INITIALIZED;
    }

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::ImuIntegrator::deinitialize()
{
    LOG_TRACE("called for {}", name);

    if (initialized)
    {
        initialized = false;
        LOG_DEBUG("{} successfully deinitialized", name);
        return NavStatus::NAV_OK;
    }

    return NAV_WARNING_NOT_INITIALIZED;
}