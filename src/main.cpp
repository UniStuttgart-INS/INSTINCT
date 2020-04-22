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

#include "util/Common.hpp"
#include "util/Logger.hpp"
#include "util/Version.hpp"
#include "util/Sleep.hpp"
#include "util/Config.hpp"

#include "NodeInterface.hpp"
#include "Nodes/NodeCreator.hpp"

int main(int argc, const char** argv)
{
    if (NAV::Logger::initialize("logs/navsos.log") != NAV::NavStatus::NAV_OK)
        return EXIT_FAILURE;

    // Read Command Line Options
    NAV::Config* pConfig = NAV::Config::Get();
    if (NAV::NavStatus result = pConfig->AddOptions(argc, argv);
        result == NAV::NavStatus::NAV_REQUEST_EXIT)
        return EXIT_SUCCESS;
    else if (result != NAV::NavStatus::NAV_OK)
        LOG_CRITICAL("Critical problem during adding of program options");

    // Write the Log Header
    NAV::Logger::writeHeader();
    // Decode Options
    if (pConfig->DecodeOptions() != NAV::NavStatus::NAV_OK)
        LOG_CRITICAL("Critical problem during decoding of program options");

    // Create Nodes
    if (NAV::NodeCreator::createNodes(pConfig) != NAV::NavStatus::NAV_OK)
        LOG_CRITICAL("Critical problem during node creation");

    // Set up Node Links
    if (NAV::NodeCreator::createLinks(pConfig) != NAV::NavStatus::NAV_OK)
        LOG_CRITICAL("Critical problem during node link creation");

    // Read data files
    if (NAV::appContext == NAV::NodeInterface::NodeContext::POST_PROCESSING)
    {
        std::map<NAV::InsTime, size_t> events;
        // Get first event of all the file readers
        for (size_t i = 0; i < pConfig->nodes.size(); i++)
        {
            if (pConfig->nodes.at(i).node->isFileReader())
            {
                auto nextUpdateTime = pConfig->nodes.at(i).node->peekNextUpdateTime();
                if (nextUpdateTime.has_value())
                    events.insert(std::make_pair(nextUpdateTime.value(), i));
            }
        }

        std::map<NAV::InsTime, size_t>::iterator it;
        while (it = events.begin(), it != events.end())
        {
            if (pConfig->nodes.at(it->second).node->pollData() == nullptr)
                LOG_ERROR("{} - {} could not poll its observation despite being able to peek it.", pConfig->nodes.at(it->second).type, pConfig->nodes.at(it->second).name);

            auto nextUpdateTime = pConfig->nodes.at(it->second).node->peekNextUpdateTime();
            if (nextUpdateTime.has_value())
                events.insert(std::make_pair(nextUpdateTime.value(), it->second));

            events.erase(it);
        }
    }
    // Wait and receive data packages in other threads
    else
    {
        if (pConfig->GetSigterm())
            NAV::Sleep::waitForSignal();
        else
            NAV::Sleep::countDownSeconds(pConfig->GetProgExecTime());
    }

    // Stop all callbacks
    for (auto& node : pConfig->nodes)
    {
        node.node->callbacksEnabled = false;
        node.node->removeAllCallbacks();
    }

    // Update all GnuPlot Windows and wait for them to open
    for (auto& node : pConfig->nodes)
    {
        if (node.type.find("GnuPlot") != std::string::npos)
        {
            std::static_pointer_cast<NAV::GnuPlot>(node.node)->update();
            while (system("xwininfo -name \"Gnuplot window 0\" > /dev/null 2>&1"))
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        // Destruct all Nodes
        node.node = nullptr;
    }

    if (!system("xwininfo -name \"Gnuplot window 0\" > /dev/null 2>&1"))
        LOG_INFO("Waiting for all Gnuplot windows to close");

    // Wait if any GnuPlot Window is open. GnuPlot becomes unresponsive when parent dies
    while (!system("xwininfo -name \"Gnuplot window 0\" > /dev/null 2>&1"))
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    NAV::Logger::writeFooter();

    return EXIT_SUCCESS;
}