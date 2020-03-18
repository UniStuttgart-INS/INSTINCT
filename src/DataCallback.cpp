#include "DataCallback.hpp"

#include "util/Logger.hpp"

NAV::DataCallback::DataCallback() {}

NAV::DataCallback::~DataCallback() {}

NAV::NavStatus NAV::DataCallback::addCallback(std::function<NAV::NavStatus(std::shared_ptr<void>, std::shared_ptr<void>)> callback, std::shared_ptr<void> userData)
{
    LOG_TRACE("called");

    _callbacks.push_back(callback);
    _userData.push_back(userData);

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::DataCallback::addCallback(std::function<NAV::NavStatus(std::shared_ptr<void>, std::shared_ptr<void>)> callback, std::shared_ptr<void> userData, size_t index)
{
    LOG_TRACE("called");

    auto itc = _callbacks.begin();
    auto itu = _userData.begin();
    for (size_t i = 0; i < _callbacks.size(); itc++, itu++, i++)
    {
        if (i == index)
        {
            _callbacks.insert(itc, callback);
            _userData.insert(itu, userData);
            return NavStatus::NAV_OK;
        }
    }

    LOG_WARN("Tried to insert a callback at index={}, but there are only {} callbacks registered so far. Adding it now to the end of the list", index, _callbacks.size());

    return addCallback(callback, userData);
}

NAV::NavStatus NAV::DataCallback::removeCallback()
{
    LOG_TRACE("called");

    if (_callbacks.size() > 0)
    {
        _callbacks.pop_back();
        _userData.pop_back();
        return NavStatus::NAV_OK;
    }
    else
    {
        LOG_WARN("Could not remove the last callback because no callbacks registered.");
        return NavStatus::NAV_ERROR;
    }
}

NAV::NavStatus NAV::DataCallback::removeCallback(size_t index)
{
    LOG_TRACE("called and index {}", index);

    auto itc = _callbacks.begin();
    auto itu = _userData.begin();
    for (size_t i = 0; i < _callbacks.size(); itc++, itu++, i++)
    {
        if (i == index)
        {
            _callbacks.erase(itc);
            _userData.erase(itu);
            return NavStatus::NAV_OK;
        }
    }

    LOG_ERROR("Could not remove the callback at index={}. There are {} callbacks registered.", index, _callbacks.size());
    return NavStatus::NAV_ERROR;
}

NAV::NavStatus NAV::DataCallback::removeAllCallbacks()
{
    LOG_TRACE("called");

    // Clear does not erase the memory the pointers point at
    // This is not a problem howeever, as we use shared_ptr
    _callbacks.clear();
    _userData.clear();
    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::DataCallback::invokeCallbacks(std::shared_ptr<void> obs)
{
    LOG_TRACE("called and callbacks are enabled={}", callbacksEnabled);

    if (!callbacksEnabled)
        return NAV_OK;

    for (size_t i = 0; i < _callbacks.size(); i++)
    {
        if (NavStatus result = _callbacks[i](obs, _userData[i]);
            result != NAV_OK)
        {
            LOG_ERROR("An error (NavStatus: {}) occurred when calling the observationReceivedCallback {}", result, i);
            return result;
        }
    }
    return NavStatus::NAV_OK;
}