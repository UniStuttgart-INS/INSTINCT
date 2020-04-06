#include "DataCallback.hpp"

#include "util/Logger.hpp"

NAV::DataCallback::DataCallback() {}

NAV::DataCallback::~DataCallback() {}

NAV::NavStatus NAV::DataCallback::addCallback(std::function<NAV::NavStatus(std::shared_ptr<void>, std::shared_ptr<void>)> callback, std::shared_ptr<void> userData)
{
    LOG_TRACE("called");

    _callbacks.push_back({ callback, userData });

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::DataCallback::addCallback(std::function<NAV::NavStatus(std::shared_ptr<void>, std::shared_ptr<void>)> callback, std::shared_ptr<void> userData, size_t index)
{
    LOG_TRACE("called");

    auto it = _callbacks.begin();
    for (size_t i = 0; i < _callbacks.size(); it++, i++)
    {
        if (i == index)
        {
            _callbacks.insert(it, { callback, userData });
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
    LOG_TRACE("called for index {}", index);

    auto it = _callbacks.begin();
    for (size_t i = 0; i < _callbacks.size(); it++, i++)
    {
        if (i == index)
        {
            _callbacks.erase(it);
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

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::DataCallback::invokeCallbacks(std::shared_ptr<void> data)
{
    LOG_TRACE("called and callbacks are enabled={}", callbacksEnabled);

    if (!callbacksEnabled)
        return NAV_OK;

    for (size_t i = 0; i < _callbacks.size(); i++)
    {
        if (NavStatus result = _callbacks.at(i).callback(data, _callbacks.at(i).data);
            result != NAV_OK)
        {
            LOG_ERROR("An error (NavStatus: {}) occurred when calling the callback {}", result, i);
            return result;
        }
    }

    return NavStatus::NAV_OK;
}