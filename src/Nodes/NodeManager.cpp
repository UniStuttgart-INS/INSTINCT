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
        for (std::string line : ConfigManager::Get<std::vector<std::string>>("node", {}))
        {
            NodeConfig config;

            std::string delimiter = " _,_ ";
            // Split line at comma
            while (true)
            {
                std::string cell;
                size_t pos = line.find(delimiter);
                if (pos != std::string::npos)
                {
                    cell = line.substr(0, pos);
                    line = line.substr(pos + delimiter.length());
                }
                else if (!line.empty())
                {
                    cell = line;
                    line.clear();
                }
                else
                {
                    break;
                }

                // Remove any trailing non text characters
                cell.erase(std::find_if(cell.begin(), cell.end(), [](int ch) { return std::iscntrl(ch); }), cell.end());
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
                    size_t splitPos = cell.find("\" = \"");
                    std::string key = cell.substr(1, splitPos - 1);
                    std::string value = cell.substr(splitPos + 5, cell.length() - splitPos - 6);

                    // Replace Hash sign
                    while (true)
                    {
                        std::string searchString = "[hash]";
                        std::string replaceString = "#";
                        size_t start_pos = value.find(searchString);
                        if (start_pos == std::string::npos)
                        {
                            break;
                        }
                        value.replace(start_pos, searchString.length(), replaceString);
                    }
                    // Replace newline
                    while (true)
                    {
                        std::string searchString = "\\n";
                        std::string replaceString = "\n";
                        size_t start_pos = value.find(searchString);
                        if (start_pos == std::string::npos)
                        {
                            break;
                        }
                        value.replace(start_pos, searchString.length(), replaceString);
                    }

                    config.options[key] = value;
                }
            }

            nodeConfigs.push_back(config);
            names.push_back(config.name);

            std::vector<std::string> v;
            for (const auto& [key, value] : config.options)
            {
                std::string concat = key;
                concat.append(" = ");
                concat.append(value);
                v.push_back(concat);
            }
            LOG_DEBUG("Option-node: {}, {}, {}", config.type, config.name, fmt::join(v, ", "));
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
                cell.erase(std::find_if(cell.begin(), cell.end(), [](int ch) { return std::iscntrl(ch); }), cell.end());
                // Remove whitespaces
                cell.erase(cell.begin(), std::find_if(cell.begin(), cell.end(), [](int ch) { return !std::isspace(ch); }));
                cell.erase(std::find_if(cell.rbegin(), cell.rend(), [](int ch) { return !std::isspace(ch); }).base(), cell.end());

                if (link.source.empty())
                {
                    link.source = cell;
                }
                else if (link.sourcePortIndex == UINT8_MAX)
                {
                    link.sourcePortIndex = static_cast<uint8_t>(std::stoul(cell));
                }
                else if (link.targetPortIndex == UINT8_MAX)
                {
                    link.targetPortIndex = static_cast<uint8_t>(std::stoul(cell));
                }
                else if (link.target.empty())
                {
                    link.target = cell;
                }
            }

            nodeLinks.push_back(link);

            LOG_DEBUG("Option-link: {} [{}] ==> [{}] {}", link.source, link.sourcePortIndex, link.targetPortIndex, link.target);
        }
    }
    else
    {
        LOG_CRITICAL("No Node Links found");
    }
}

void NAV::NodeManager::constructNodes()
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

        // Construct the node
        _nodes.emplace_back(iter->second.constructor(config.name, config.options));
    }
}

void NAV::NodeManager::initializeNodes()
{
    LOG_TRACE("called");

    for (auto& node : _nodes)
    {
        // Call the initialize function
        node->initialize();
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

                        if (sourceNode->nPorts(Node::PortType::Out) <= link.sourcePortIndex)
                        {
                            LOG_CRITICAL("Data Link {} [{}] ⇒ [{}] {} could not be created because the source node only has {} output ports",
                                         link.source, link.sourcePortIndex, link.targetPortIndex, link.target, sourceNode->nPorts(Node::PortType::Out));
                        }

                        if (targetNode->nPorts(Node::PortType::In) <= link.targetPortIndex)
                        {
                            LOG_CRITICAL("Data Link {} [{}] ⇒ [{}] {} could not be created because the target node only has {} input ports",
                                         link.source, link.sourcePortIndex, link.targetPortIndex, link.target, targetNode->nPorts(Node::PortType::In));
                        }

                        // At this point both nodes were found and they have the link data type

                        // Check if the NodeData type is registered and add the callback
                        auto portType = sourceNode->dataType(Node::PortType::Out, link.sourcePortIndex).first;
                        auto iter = _registeredNodeDataTypes.find(portType);
                        if (iter == _registeredNodeDataTypes.end())
                        {
                            LOG_CRITICAL("Requested NodeLink with type '{}' is not registered with the application", portType);
                        }
                        iter->second.addCallback(sourceNode, targetNode, link.targetPortIndex);
                        targetNode->incomingLinks.emplace(link.targetPortIndex, std::make_pair(sourceNode, link.sourcePortIndex));

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

void NAV::NodeManager::deleteAllNodesExcept(const std::string_view& type)
{
    for (auto& node : _nodes)
    {
        if (node && node->type() != type)
        {
            node->incomingLinks.clear();
            node->removeAllCallbacks();
            // Call the deinitialize function
            node->deinitialize();
            // Remove last reference and there call the destructor
            node = nullptr;
        }
    }
}

void NAV::NodeManager::deleteAllNodes()
{
    for (auto& node : _nodes)
    {
        if (node)
        {
            node->incomingLinks.clear();
            node->removeAllCallbacks();
            node = nullptr;
        }
    }
    _nodes.clear();
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