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

#include <iostream>

#include "util/Logger.hpp"
#include "util/Version.hpp"
#include "util/Sleep.hpp"
#include "util/ConfigManager.hpp"

#include "Nodes/NodeManager.hpp"
#include "NodeRegistry.hpp"

#include "Nodes/GnuPlot/GnuPlot.hpp"

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
            LOG_INFO("Post Processing Mode");
            std::multimap<NAV::InsTime, std::pair<std::shared_ptr<NAV::Node>, uint8_t>> events;
            // Get first event of all nodes
            for (const auto& node : nodeManager.nodes())
            {
                for (uint8_t portIndex = 0; portIndex < node->nPorts(NAV::Node::PortType::Out); portIndex++)
                {
                    // Add next data event from the node
                    while (true)
                    {
                        // Check if data available
                        if (auto nextUpdateTime = std::static_pointer_cast<NAV::InsObs>(node->requestOutputDataPeek(portIndex)))
                        {
                            // Check if data has a time
                            if (nextUpdateTime->insTime.has_value())
                            {
                                events.insert(std::make_pair(nextUpdateTime->insTime.value(), std::make_pair(node, portIndex)));
                                LOG_INFO("Reading Data from {}", node->getName());
                                break;
                            }

                            // Remove data without calling the callback if no time stamp
                            // For post processing all data needs a time stamp
                            node->callbacksEnabled = false;
                            static_cast<void>(node->requestOutputData(portIndex));
                            node->callbacksEnabled = true;
                        }
                        else
                        {
                            break;
                        }
                    }
                    node->resetNode();
                }
            }

            std::multimap<NAV::InsTime, std::pair<std::shared_ptr<NAV::Node>, uint8_t>>::iterator it;
            while (it = events.begin(), it != events.end())
            {
                auto& node = it->second.first;
                auto& portIndex = it->second.second;
                if (node->requestOutputData(portIndex) == nullptr)
                {
                    LOG_ERROR("{} - {} could not poll its observation despite being able to peek it.", node->type(), node->getName());
                }

                // Add next data event from the node
                while (true)
                {
                    // Check if data available
                    if (auto nextUpdateTime = std::static_pointer_cast<NAV::InsObs>(node->requestOutputDataPeek(portIndex)))
                    {
                        // Check if data has a time
                        if (nextUpdateTime->insTime.has_value())
                        {
                            events.insert(std::make_pair(nextUpdateTime->insTime.value(), it->second));
                            break;
                        }

                        // Remove data without calling the callback if no time stamp
                        // For post processing all data needs a time stamp
                        node->callbacksEnabled = false;
                        static_cast<void>(node->requestOutputData(portIndex));
                        node->callbacksEnabled = true;
                    }
                    else
                    {
                        break;
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

        // Delete all Nodes to call the destructors
        nodeManager.deleteAllNodes();

        // Update all GnuPlot Windows and wait for them to open
        if (NAV::GnuPlot::update())
        {
            LOG_INFO("Programm finished and waits for Gnuplot windows to close...");
            NAV::Sleep::waitForSignal();
            system("pkill gnuplot_qt > /dev/null 2>&1"); // NOLINT
        }

        return EXIT_SUCCESS;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Critical Event occurred: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
}