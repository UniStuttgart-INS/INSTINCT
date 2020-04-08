#include "NodeInterface.hpp"

size_t NAV::NodeInterface::getCallbackPort(std::string interfaceType, std::string messageType, bool inPort)
{
    if (inPort)
    {
        auto& port = nodeInterfaces.find(interfaceType)->second.in;
        for (size_t i = 0; i < port.size(); i++)
            if (port.at(i).type == messageType)
                return i;
    }
    else
    {
        auto& port = nodeInterfaces.find(interfaceType)->second.out;
        for (size_t i = 0; i < port.size(); i++)
            if (port.at(i) == messageType)
                return i;
    }
    return 1000;
}

bool NAV::NodeInterface::isTypeOrBase(std::string targetType, std::string messageType)
{
    if (targetType == messageType)
        return true;

    if (inheritance.count(messageType))
    {
        auto& parents = inheritance.find(messageType)->second;
        for (size_t i = 0; i < parents.size(); i++)
        {
            if (isTypeOrBase(targetType, parents.at(i)))
                return true;
        }
    }
    return false;
}