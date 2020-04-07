#include "NodeCreator.hpp"

#include "util/Logger.hpp"

#include "ub/protocol/types.hpp"

/// Comparision operator between Input Port Type and Output Node Data Type
bool operator==(const NAV::InputPort& lhs, const std::string& rhs)
{
    return (lhs.type == rhs);
}
/// Comparision operator between Output Node Data Type and Input Port Type
bool operator==(const std::string& lhs, const NAV::InputPort& rhs)
{
    return (lhs == rhs.type);
}

size_t NAV::NodeCreator::getCallbackPort(std::string interfaceType, std::string messageType, bool inPort)
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

bool NAV::NodeCreator::isTypeOrBase(std::string targetType, std::string messageType)
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

NAV::NavStatus NAV::NodeCreator::createNodes(NAV::Config* pConfig)
{
    LOG_INFO("Creating {} Node{}", pConfig->nodes.size(), pConfig->nodes.size() > 1 ? "s" : "");
    for (auto& node : pConfig->nodes)
    {
        if (nodeInterfaces.count(node.type))
            node.node = nodeInterfaces.find(node.type)->second.constructor(node.name, node.options);
        else
        {
            LOG_CRITICAL("Node {} - {} has unknown type", node.type, node.name);
            return NavStatus::NAV_ERROR;
        }

        // Initialize the Node
        if (node.node == nullptr || node.node->initialize() != NAV::NavStatus::NAV_OK)
        {
            LOG_CRITICAL("Node {} - {} could not be created", node.type, node.name);
            return NavStatus::NAV_ERROR;
        }
        else
            LOG_INFO("{}═⇒ {} ({}) created", node.name == pConfig->nodes.back().name ? "╚" : "╠", node.name, node.type);
    }

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::NodeCreator::createLinks(NAV::Config* pConfig)
{
    LOG_INFO("Creating {} Node Link{}", pConfig->nodeLinks.size(), pConfig->nodeLinks.size() > 1 ? "s" : "");
    for (auto& nodeLink : pConfig->nodeLinks)
    {
        // Find source and target
        NAV::Config::NodeConfig *sourceNode = nullptr, *targetNode = nullptr;
        for (size_t i = 0; i < pConfig->nodes.size(); i++)
        {
            if (pConfig->nodes[i].name == nodeLink.source)
                sourceNode = &pConfig->nodes[i];
            else if (pConfig->nodes[i].name == nodeLink.target)
                targetNode = &pConfig->nodes[i];
        }
        if (!sourceNode || !targetNode)
        {
            LOG_CRITICAL("Node Link {} ⇒ {} could not be created because {} could not be found",
                         nodeLink.source, nodeLink.target,
                         targetNode ? nodeLink.source : (sourceNode ? nodeLink.target : "source and target"));
            return NavStatus::NAV_ERROR;
        }

        // Search matching interfaces
        if (!nodeInterfaces.count(sourceNode->type) || !nodeInterfaces.count(targetNode->type))
        {
            LOG_CRITICAL("Data Link {} ⇒ {} could not be created because type {} is not supported by any node interface.",
                         nodeLink.source, nodeLink.target, !nodeInterfaces.count(sourceNode->type) ? sourceNode->type : targetNode->type);
            return NavStatus::NAV_ERROR;
        }
        auto& sourceInterface = nodeInterfaces.find(sourceNode->type)->second;
        auto& targetInterface = nodeInterfaces.find(targetNode->type)->second;

        bool linkEstablished = false;
        for (size_t i = 0; i < sourceInterface.out.size(); i++)
        {
            // Check if source interface provides message type
            if (sourceInterface.out[i] == nodeLink.type)
            {
                for (size_t j = 0; j < targetInterface.in.size(); j++)
                {
                    // Check if target interface provides message type or any of its base classes
                    if (isTypeOrBase(targetInterface.in[j].type, nodeLink.type))
                    {
                        // Set up callback
                        sourceNode->node->addCallback(i, targetInterface.in[j].callback, targetNode->node);
                        sourceNode->node->callbacksEnabled = true;
                        linkEstablished = true;
                        break;
                    }
                }
                break;
            }
        }

        if (linkEstablished)
        {
            LOG_INFO("{}═⇒ {} ═({})⇒ {} created", (nodeLink.source == pConfig->nodeLinks.back().source && nodeLink.target == pConfig->nodeLinks.back().target) ? "╚" : "╠",
                     nodeLink.source, nodeLink.type, nodeLink.target);
        }
        else
        {
            LOG_CRITICAL("Data Link {} ⇒ {} could not be created because link generation for types {} ═({})⇒ {} is not supported.",
                         nodeLink.source, nodeLink.target, sourceNode->type, nodeLink.type, targetNode->type);
            return NavStatus::NAV_ERROR;
        }
    }

    return NavStatus::NAV_OK;
}