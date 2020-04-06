#include "NodeCreator.hpp"

#include "util/Logger.hpp"

#include "Nodes/DataProvider/GNSS/Sensors/UbloxSensor.hpp"
#include "Nodes/DataProvider/IMU/Sensors/VectorNavSensor.hpp"
#include "Nodes/DataProvider/IMU/FileReader/VectorNavFile.hpp"

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

NAV::NavStatus NAV::NodeCreator::createNodes(NAV::Config* pConfig)
{
    LOG_INFO("Creating {} Node{}", pConfig->nodes.size(), pConfig->nodes.size() > 1 ? "s" : "");
    for (auto& node : pConfig->nodes)
    {
        // Handle the program options to create a node here
        if (node.type == "VectorNavSensor")
        {
            NAV::VectorNavSensor::Config config;

            if (node.options.size() >= 1)
                config.outputFrequency = static_cast<uint16_t>(std::stoul(node.options.at(0)));
            if (node.options.size() >= 2)
                config.sensorPort = node.options.at(1);
            if (node.options.size() >= 3)
                config.sensorBaudrate = static_cast<NAV::UartSensor::Baudrate>(std::stoul(node.options.at(2)));

            node.node = std::make_shared<NAV::VectorNavSensor>(node.name, config);
        }
        else if (node.type == "VectorNavFile")
        {
            NAV::VectorNavFile::Config config;
            std::string path;

            if (node.options.size() >= 1)
                path = node.options.at(0);

            node.node = std::make_shared<NAV::VectorNavFile>(node.name, path, config);
        }
        else if (node.type == "UbloxSensor")
        {
            NAV::UbloxSensor::Config config;

            if (node.options.size() >= 1)
                config.outputFrequency = static_cast<uint16_t>(std::stoul(node.options.at(0)));
            if (node.options.size() >= 2)
                config.sensorPort = node.options.at(1);
            if (node.options.size() >= 3)
                config.sensorBaudrate = static_cast<NAV::UartSensor::Baudrate>(std::stoul(node.options.at(2)));

            node.node = std::make_shared<NAV::UbloxSensor>(node.name, config);
        }
        else if (node.type == "VectorNavDataLogger")
        {
            std::string path;
            bool isBinary = false;

            if (node.options.size() >= 1)
                path = node.options.at(0);
            if (node.options.size() >= 2)
            {
                if (node.options.at(1) == "ascii")
                    isBinary = false;
                else if (node.options.at(1) == "binary")
                    isBinary = true;
                else
                    LOG_WARN("Node {} - {} has unknown file type {}. Using ascii instead", node.type, node.name, node.options.at(1));
            }

            node.node = std::make_shared<NAV::VectorNavDataLogger>(node.name, path, isBinary);
        }
        else if (node.type == "UbloxDataLogger")
        {
            std::string path;
            bool isBinary = true;

            if (node.options.size() >= 1)
                path = node.options.at(0);
            if (node.options.size() >= 2)
            {
                if (node.options.at(1) == "ascii")
                    isBinary = false;
                else if (node.options.at(1) == "binary")
                    isBinary = true;
                else
                    LOG_WARN("Node {} - {} has unknown file type {}. Using binary instead", node.type, node.name, node.options.at(1));
            }

            node.node = std::make_shared<NAV::UbloxDataLogger>(node.name, path, isBinary);
        }
        else if (node.type == "UbloxSyncSignal")
        {
            //SensorPort           type msgClass msgId
            std::string port;
            ub::protocol::uart::UbxClass ubxClass;
            uint8_t ubxMsgId;

            if (node.options.size() >= 1)
                port = node.options.at(0);
            if (node.options.size() >= 4)
            {
                if (node.options.at(1) == "UBX")
                {
                    ubxClass = ub::protocol::uart::getMsgClassFromString(node.options.at(2));
                    ubxMsgId = ub::protocol::uart::getMsgIdFromString(ubxClass, node.options.at(3));

                    node.node = std::make_shared<NAV::UbloxSyncSignal>(node.name, port, ubxClass, ubxMsgId);
                }
                else
                {
                    LOG_CRITICAL("Node {} - {} has unknown type {}", node.type, node.name, node.options.at(1));
                    return NavStatus::NAV_ERROR;
                }
            }
            else
            {
                LOG_CRITICAL("Node {} - {} has not enough options", node.type, node.name);
                return NavStatus::NAV_ERROR;
            }
        }
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
        const NAV::NodeInterface *sourceInterface = nullptr, *targetInterface = nullptr;
        for (size_t i = 0; i < nodeInterfaces.size(); i++)
        {
            if (sourceNode->type == nodeInterfaces[i].type)
                sourceInterface = &nodeInterfaces[i];
            else if (targetNode->type == nodeInterfaces[i].type)
                targetInterface = &nodeInterfaces[i];
        }
        if (!sourceInterface || !targetInterface)
        {
            LOG_CRITICAL("Data Link {} ⇒ {} could not be created because link generation for types {} ═({})⇒ {} is not supported by any node interface.",
                         nodeLink.source, nodeLink.target, sourceNode->type, nodeLink.type, targetNode->type);
            return NavStatus::NAV_ERROR;
        }

        bool linkEstablished = false;
        for (size_t i = 0; i < sourceInterface->out.size(); i++)
        {
            // Check if source interface provides message type
            if (sourceInterface->out[i] == nodeLink.type)
            {
                for (size_t j = 0; j < targetInterface->in.size(); j++)
                {
                    // Check if target interface provides message type
                    if (targetInterface->in[j].type == nodeLink.type)
                    {
                        // Set up callback
                        sourceNode->node->addCallback(targetInterface->in[j].callback, targetNode->node);
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