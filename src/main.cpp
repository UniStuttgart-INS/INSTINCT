/** @mainpage NavSoS Documentation
 *
 *  @section sec1 Introduction
 *  This software provides real-time and post processing functionality for navigational tasks. It can read from sensors and fuse together the data. It can fuse GNSS data with IMU data and do advanced functions like RTK, RAIM, ...
 *
 *  @section sec4 Code Elements
 *      - @link src/main.cpp Main File @endlink
 *      - @link src/DataProvider/DataProvider.hpp Data Provider Class @endlink
 *          - @link src/DataProvider/IMU/Imu.hpp IMU Data Provider Class @endlink
 *          - @link src/DataProvider/GNSS/Gnss.hpp GNSS Data Provider Class @endlink
 */

/**
 * @file main.cpp
 * @brief Main entry point for the program
 * @author T. Topp (thomas.topp@nav.uni-stuttgart.de)
 * @date 2020-03-12
 */

#include <chrono>
#include <thread>
#include <iostream>

#include "util/Logger.hpp"
#include "util/Version.hpp"
#include "util/Sleep.hpp"
#include "util/ConfigManager.hpp"

#include "Nodes/NodeManager.hpp"
#include "NodeRegistry.hpp"

#include "NodeData/InsObs.hpp"

int main(int argc, const char* argv[])
{
    // Config Manager object
    NAV::ConfigManager configManager;

    if (argc == 2)
    {
        // User requested the version of the program
        if (!strcmp(argv[1], "--version") || !strcmp(argv[1], "-v"))
        {
            std::cout << PROJECT_VERSION_STRING << '\n';
            return EXIT_SUCCESS;
        }

        // User requested the help text of the program
        if (!strcmp(argv[1], "--help") || !strcmp(argv[1], "-h"))
        {
            std::cout << "NavSoS " << PROJECT_VERSION_STRING << " - Navigation Software Stuttgart\n\n"
                      << NAV::ConfigManager::GetProgramOptions() << '\n';
            return EXIT_SUCCESS;
        }
    }

    try
    {
        Logger logger("logs/navsos.log");

        // Program configuration
        NAV::ConfigManager::FetchConfigs(argc, argv);

        // Create a Node Manager
        NAV::NodeManager nodeManager;

        // Register all Node Types which are available to the program
        NAV::NodeRegistry::registerNodeTypes(nodeManager);

        // Register all Node Data Types which are available to the program
        NAV::NodeRegistry::registerNodeDataTypes(nodeManager);

        // Processes all nodes which are specified in the config file
        nodeManager.processConfigFile();

        // Call constructors of all nodes from the config file
        nodeManager.initializeNodes();

        // Establish data links between the nodes
        nodeManager.linkNodes();

        // Enable the callbacks for all nodes
        nodeManager.enableAllCallbacks();

        // Read data files
        if (NAV::NodeManager::appContext == NAV::Node::NodeContext::POST_PROCESSING)
        {
            std::map<NAV::InsTime, std::pair<std::shared_ptr<NAV::Node>, uint8_t>> events;
            // Get first event of all nodes
            for (const auto& node : nodeManager.nodes())
            {
                for (uint8_t portIndex = 0; portIndex < node->nPorts(NAV::Node::PortType::Out); portIndex++)
                {
                    auto nextUpdateTime = std::static_pointer_cast<NAV::InsObs>(node->requestOutputDataPeek(portIndex));

                    if (nextUpdateTime)
                    {
                        if (nextUpdateTime->insTime.has_value())
                        {
                            events.insert(std::make_pair(nextUpdateTime->insTime.value(), std::make_pair(node, portIndex)));
                        }
                        else
                        {
                            LOG_ERROR("Node {} does provide data but without InsTime value.", node->getName());
                        }
                    }
                }
            }

            std::map<NAV::InsTime, std::pair<std::shared_ptr<NAV::Node>, uint8_t>>::iterator it;
            while (it = events.begin(), it != events.end())
            {
                auto& node = it->second.first;
                auto& portIndex = it->second.second;
                if (node->requestOutputData(portIndex) == nullptr)
                {
                    LOG_ERROR("{} - {} could not poll its observation despite being able to peek it.", node->type(), node->getName());
                }

                auto nextUpdateTime = std::static_pointer_cast<NAV::InsObs>(node->requestOutputDataPeek(portIndex));
                if (nextUpdateTime)
                {
                    if (nextUpdateTime->insTime.has_value())
                    {
                        events.insert(std::make_pair(nextUpdateTime->insTime.value(), it->second));
                    }
                    else
                    {
                        LOG_WARN("Node {} does provide data but without InsTime value.", node->getName());
                    }
                }

                events.erase(it);
            }
        }
        // Wait and receive data packages in other threads
        else
        {
            if (NAV::ConfigManager::Get<bool>("sigterm", false))
            {
                NAV::Sleep::waitForSignal();
            }
            else
            {
                NAV::Sleep::countDownSeconds(NAV::ConfigManager::Get<size_t>("duration", 5));
            }
        }

        // Stop all callbacks
        nodeManager.disableAllCallbacks();

        // Update all GnuPlot Windows and wait for them to open
        for (const auto& node : nodeManager.nodes())
        {
            if (node->type().find("GnuPlot") != std::string_view::npos)
            {
                // std::static_pointer_cast<NAV::GnuPlot>(node)->update();
                while (system("xwininfo -name \"Gnuplot window 0\" > /dev/null 2>&1")) // NOLINT
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        }

        if (!system("xwininfo -name \"Gnuplot window 0\" > /dev/null 2>&1")) // NOLINT
        {
            LOG_INFO("Waiting for all Gnuplot windows to close");
        }

        // Wait if any GnuPlot Window is open. GnuPlot becomes unresponsive when parent dies
        while (!system("xwininfo -name \"Gnuplot window 0\" > /dev/null 2>&1")) // NOLINT
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        return EXIT_SUCCESS;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Critical Event occurred: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
}