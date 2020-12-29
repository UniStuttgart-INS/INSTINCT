#include "Node.hpp"

void NAV::Node::initialize() {}

void NAV::Node::deinitialize() {}

void NAV::Node::invokeCallbacks(size_t portIndex, const std::shared_ptr<NAV::NodeData>& data)
{
    if (callbacksEnabled)
    {
        for (auto& [node, callback] : outputPins.at(portIndex).callbacks)
        {
            std::invoke(callback, node, data);
        }
    }
}