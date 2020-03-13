#include "DataProvider.hpp"
#include "util/Logger.hpp"

NAV::DataProvider::DataProvider(const std::string name)
    : name(name) {}

NAV::DataProvider::~DataProvider() {}

bool NAV::DataProvider::isInitialized()
{
    LOG_TRACE("called for {} with value {}", name, initialized);
    return initialized;
}

std::string NAV::DataProvider::getName()
{
    LOG_TRACE("called for {}", name);
    return name;
}

NAV::NavStatus NAV::DataProvider::addObservationReceivedCallback(std::function<NAV::NavStatus(std::shared_ptr<NAV::InsObs>, std::shared_ptr<void>)> callback, std::shared_ptr<void> userData)
{
    LOG_TRACE("called for {}", name);

    observationReceivedCallbacks.push_back(callback);
    observationReceivedUserData.push_back(userData);

    return NAV_OK;
}

NAV::NavStatus NAV::DataProvider::addObservationReceivedCallback(std::function<NAV::NavStatus(std::shared_ptr<NAV::InsObs>, std::shared_ptr<void>)> callback, std::shared_ptr<void> userData, size_t index)
{
    LOG_TRACE("called for {})", name);

    auto itc = observationReceivedCallbacks.begin();
    auto itu = observationReceivedUserData.begin();
    for (size_t i = 0; i < observationReceivedCallbacks.size(); itc++, itu++, i++)
    {
        if (i == index)
        {
            observationReceivedCallbacks.insert(itc, callback);
            observationReceivedUserData.insert(itu, userData);
            return NAV_OK;
        }
    }

    LOG_WARN("Tried to insert a callback at index={}, but there are only {} callbacks registered so far. Adding it now to the end of the list", index, observationReceivedCallbacks.size());

    return addObservationReceivedCallback(callback, userData);
}

NAV::NavStatus NAV::DataProvider::removeObservationReceivedCallback()
{
    LOG_TRACE("called for {}", name);

    if (observationReceivedCallbacks.size() > 0)
    {
        observationReceivedCallbacks.pop_back();
        observationReceivedUserData.pop_back();
        return NAV_OK;
    }
    else
    {
        LOG_WARN("Could not remove the last callback because no callbacks registered.");
        return NAV_ERROR;
    }
}

NAV::NavStatus NAV::DataProvider::removeObservationReceivedCallback(size_t index)
{
    LOG_TRACE("called for {} and index {}", name, index);

    auto itc = observationReceivedCallbacks.begin();
    auto itu = observationReceivedUserData.begin();
    for (size_t i = 0; i < observationReceivedCallbacks.size(); itc++, itu++, i++)
    {
        if (i == index)
        {
            observationReceivedCallbacks.erase(itc);
            observationReceivedUserData.erase(itu);
            return NAV_OK;
        }
    }

    LOG_ERROR("Could not remove the callback at index={}. There are {} callbacks registered.", index, observationReceivedCallbacks.size());
    return NAV_ERROR;
}

NAV::NavStatus NAV::DataProvider::removeAllObservationReceivedCallbacks()
{
    LOG_TRACE("called for {}", name);

    // Clear does not erase the memory the pointers point at
    // This is not a problem howeever, as we use shared_ptr
    observationReceivedCallbacks.clear();
    observationReceivedUserData.clear();
    return NAV_OK;
}

NAV::NavStatus NAV::DataProvider::observationReceived(std::shared_ptr<InsObs> obs)
{
    LOG_TRACE("called for {} and callbacks are enabled={}", name, callbacksEnabled);

    if (!callbacksEnabled)
        return NAV_OK;

    for (size_t i = 0; i < observationReceivedCallbacks.size(); i++)
    {
        if (NavStatus result = observationReceivedCallbacks[i](obs, observationReceivedUserData[i]);
            result != NAV_OK)
        {
            LOG_ERROR("An error (NavStatus: {}) occurred when calling the observationReceivedCallback {}", result, i);
            return result;
        }
    }
    return NAV_OK;
}