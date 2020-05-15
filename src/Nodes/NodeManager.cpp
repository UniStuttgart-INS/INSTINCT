#include "NodeManager.hpp"

#include "util/ConfigManager.hpp"
#include "util/Logger.hpp"

NAV::Node::NodeContext NAV::NodeManager::appContext = NAV::Node::NodeContext::ALL;

void NAV::NodeManager::processConfigFile()
{
    LOG_TRACE("called");

    if (ConfigManager::HasKey("node"))
    {
        std::vector<std::string> names;
        for (const std::string& line : ConfigManager::Get<std::vector<std::string>>("node", {}))
        {
            NodeConfig config;

            std::stringstream lineStream(line);
            std::string cell;
            // Split line at comma
            while (std::getline(lineStream, cell, ','))
            {
                // Remove any trailing non text characters
                cell.erase(std::find_if(cell.begin(), cell.end(),
                                        std::ptr_fun<int, int>(std::iscntrl)),
                           cell.end());
                // Remove whitespaces
                cell.erase(cell.begin(), std::find_if(cell.begin(), cell.end(), [](int ch) { return !std::isspace(ch); }));
                cell.erase(std::find_if(cell.rbegin(), cell.rend(), [](int ch) { return !std::isspace(ch); }).base(), cell.end());

                if (config.type.empty())
                {
                    config.type = cell;
                }
                else if (config.name.empty())
                {
                    config.name = cell;
                }
                else
                {
                    config.options.push_back(cell);
                }
            }

            nodeConfigs.push_back(config);
            names.push_back(config.name);

            LOG_DEBUG("Option-node: {}, {}, {}", config.type, config.name, fmt::join(config.options, ", "));
        }
        // Check if duplicate names
        auto it = std::unique(names.begin(), names.end());
        if (it != names.end())
        {
            LOG_CRITICAL("Node names must be unique: '{}'", fmt::join(names, ", "));
        }
    }
    else
    {
        LOG_CRITICAL("No Nodes found");
    }

    if (ConfigManager::HasKey("link"))
    {
        for (const std::string& line : ConfigManager::Get<std::vector<std::string>>("link", {}))
        {
            NodeLink link;

            std::stringstream lineStream(line);
            std::string cell;
            // Split line at comma
            while (std::getline(lineStream, cell, ','))
            {
                // Remove any trailing non text characters
                cell.erase(std::find_if(cell.begin(), cell.end(),
                                        std::ptr_fun<int, int>(std::iscntrl)),
                           cell.end());
                // Remove whitespaces
                cell.erase(cell.begin(), std::find_if(cell.begin(), cell.end(), [](int ch) { return !std::isspace(ch); }));
                cell.erase(std::find_if(cell.rbegin(), cell.rend(), [](int ch) { return !std::isspace(ch); }).base(), cell.end());

                if (link.source.empty())
                {
                    link.source = cell;
                }
                else if (link.type.empty())
                {
                    link.type = cell;
                }
                else if (link.target.empty())
                {
                    link.target = cell;
                }
            }

            nodeLinks.push_back(link);

            LOG_DEBUG("Option-link: {} == {} ==> {}", link.source, link.type, link.target);
        }
    }
    else
    {
        LOG_CRITICAL("No Node Links found");
    }
}

void NAV::NodeManager::initializeNodes()
{
    LOG_TRACE("called");

    for (auto& config : nodeConfigs)
    {
        auto iter = _registeredNodes.find(config.type);
        if (iter == _registeredNodes.end())
        {
            LOG_CRITICAL("Node {} with type '{}' is not registered with the application", config.name, config.type);
        }

        if (appContext == Node::NodeContext::ALL && iter->second.constructorEmpty()->context() != Node::NodeContext::ALL)
        {
            appContext = iter->second.constructorEmpty()->context();
        }

        if (iter->second.constructorEmpty()->context() != appContext && iter->second.constructorEmpty()->context() != Node::NodeContext::ALL)
        {
            LOG_CRITICAL("Node {} - {} is of type {} but previous nodes are of type {}.", config.type, config.name,
                         iter->second.constructorEmpty()->context() == Node::NodeContext::REAL_TIME ? "Real-Time" : "Post Processing",
                         appContext == Node::NodeContext::REAL_TIME ? "Real-Time" : "Post Processing");
        }

        _nodes.emplace_back(iter->second.constructor(config.name, config.options));
    }
}

void NAV::NodeManager::linkNodes()
{
    LOG_TRACE("called");

    for (const auto& link : nodeLinks)
    {
        bool sourceNodeFound = false;
        for (auto& sourceNode : _nodes)
        {
            // Find source node
            if (sourceNode->getName() == link.source)
            {
                sourceNodeFound = true;

                bool targetNodeFound = false;
                for (auto& targetNode : _nodes)
                {
                    // Find target node
                    if (targetNode->getName() == link.target)
                    {
                        targetNodeFound = true;

                        bool sourceNodeHasOutputType = false;
                        for (uint8_t i = 0; i < sourceNode->nPorts(Node::PortType::Out); i++)
                        {
                            if (sourceNode->dataType(Node::PortType::Out, i) == link.type)
                            {
                                sourceNodeHasOutputType = true;

                                bool targetNodeHasInputType = false;
                                for (uint8_t j = 0; j < targetNode->nPorts(Node::PortType::In); j++)
                                {
                                    if (targetNode->dataType(Node::PortType::In, j) == link.type)
                                    {
                                        targetNodeHasInputType = true;

                                        // At this point both nodes were found and they have the link data type

                                        // Check if the NodeData type is registered and add the callback
                                        auto iter = _registeredNodeDataTypes.find(link.type);
                                        if (iter == _registeredNodeDataTypes.end())
                                        {
                                            LOG_CRITICAL("Requested NodeLink with type '{}' is not registered with the application", link.type);
                                        }
                                        iter->second.addCallback(sourceNode, targetNode, j);

                                        break;
                                    }
                                }
                                if (!targetNodeHasInputType)
                                {
                                    LOG_CRITICAL("Data Link {} ⇒ {} could not be created because the data type {} is not supported by the target node",
                                                 link.source, link.target, link.type);
                                }

                                break;
                            }
                        }
                        if (!sourceNodeHasOutputType)
                        {
                            LOG_CRITICAL("Data Link {} ⇒ {} could not be created because the data type {} is not supported by the source node",
                                         link.source, link.target, link.type);
                        }

                        break;
                    }
                }
                if (!targetNodeFound)
                {
                    LOG_CRITICAL("Node Link {} ⇒ {} could not be created because the target node could not be found",
                                 link.source, link.target);
                }

                break;
            }
        }
        if (!sourceNodeFound)
        {
            LOG_CRITICAL("Node Link {} ⇒ {} could not be created because the source node could not be found",
                         link.source, link.target);
        }
    }
}

void NAV::NodeManager::enableAllCallbacks()
{
    for (auto& node : _nodes)
    {
        node->callbacksEnabled = true;
    }
}

void NAV::NodeManager::disableAllCallbacks()
{
    for (auto& node : _nodes)
    {
        node->callbacksEnabled = false;
    }
}

const std::vector<std::shared_ptr<NAV::Node>>& NAV::NodeManager::nodes() const
{
    return _nodes;
}

const std::map<std::string_view, NAV::NodeManager::NodeInfo>& NAV::NodeManager::registeredNodeTypes() const
{
    return _registeredNodes;
}

const std::map<std::string_view, NAV::NodeManager::NodeDataInfo>& NAV::NodeManager::registeredNodeDataTypes() const
{
    return _registeredNodeDataTypes;
}