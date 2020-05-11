#include "DataCallback.hpp"

#include "util/Logger.hpp"

NAV::NavStatus NAV::DataCallback::addCallback(size_t port, std::function<NAV::NavStatus(std::shared_ptr<NAV::NodeData>, std::shared_ptr<NAV::Node>)> callback, std::shared_ptr<NAV::Node> userData)
{
    LOG_TRACE("called");

    if (!_callbacks.count(port))
        _callbacks.insert({ port, {} });

    _callbacks.at(port).push_back({ callback, userData });

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::DataCallback::addCallback(size_t port, std::function<NAV::NavStatus(std::shared_ptr<NAV::NodeData>, std::shared_ptr<NAV::Node>)> callback, std::shared_ptr<NAV::Node> userData, size_t index)
{
    LOG_TRACE("called");

    if (!_callbacks.count(port))
        _callbacks.insert({ port, {} });

    auto it = _callbacks.at(port).begin();
    for (size_t i = 0; i < _callbacks.at(port).size(); it++, i++)
    {
        if (i == index)
        {
            _callbacks.at(port).insert(it, { callback, userData });
            return NavStatus::NAV_OK;
        }
    }

    LOG_WARN("Tried to insert a callback at index={}, but there are only {} callbacks registered so far. Adding it now to the end of the list", index, _callbacks.size());

    return addCallback(port, callback, userData);
}

NAV::NavStatus NAV::DataCallback::removeCallback(size_t port)
{
    LOG_TRACE("called");

    if (_callbacks.count(port))
    {
        if (_callbacks.at(port).size() > 0)
        {
            _callbacks.at(port).pop_back();
            return NavStatus::NAV_OK;
        }
        else
        {
            LOG_WARN("Could not remove the last callback because no callbacks registered.");
            return NavStatus::NAV_ERROR;
        }
    }
    else
    {
        LOG_WARN("Could not remove the last callback because port does not exist.");
        return NavStatus::NAV_ERROR;
    }
}

NAV::NavStatus NAV::DataCallback::removeCallback(size_t port, size_t index)
{
    LOG_TRACE("called for index {}", index);

    if (!_callbacks.count(port))
        _callbacks.insert({ port, {} });

    auto it = _callbacks.at(port).begin();
    for (size_t i = 0; i < _callbacks.at(port).size(); it++, i++)
    {
        if (i == index)
        {
            _callbacks.at(port).erase(it);
            return NavStatus::NAV_OK;
        }
    }

    LOG_ERROR("Could not remove the callback at index={}. There are {} callbacks registered.", index, _callbacks.at(port).size());
    return NavStatus::NAV_ERROR;
}

NAV::NavStatus NAV::DataCallback::removeAllCallbacks(size_t port)
{
    LOG_TRACE("called");

    // Clear does not erase the memory the pointers point at
    // This is not a problem howeever, as we use shared_ptr
    if (_callbacks.count(port))
        _callbacks.at(port).clear();

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::DataCallback::removeAllCallbacks()
{
    LOG_TRACE("called");

    // Clear does not erase the memory the pointers point at
    // This is not a problem howeever, as we use shared_ptr
    for (size_t i = 0; i < _callbacks.size(); i++)
        _callbacks.at(i).clear();

    _callbacks.clear();

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::DataCallback::invokeCallbacks(size_t port, std::shared_ptr<NAV::NodeData> data)
{
    LOG_TRACE("called and callbacks are enabled={}", callbacksEnabled);

    if (!callbacksEnabled)
        return NAV_OK;

    if (_callbacks.count(port))
    {
        for (size_t i = 0; i < _callbacks.at(port).size(); i++)
        {
            if (NavStatus result = _callbacks.at(port).at(i).callback(data, _callbacks.at(port).at(i).data);
                result != NAV_OK)
            {
                LOG_ERROR("An error (NavStatus: {}) occurred when calling the callback {}", result, i);
                return result;
            }
        }
    }

    return NavStatus::NAV_OK;
}