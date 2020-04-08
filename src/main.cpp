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

#include "util/Common.hpp"
#include "util/Logger.hpp"
#include "util/Version.hpp"
#include "util/Sleep.hpp"
#include "util/Config.hpp"

#include "Nodes/NodeCreator.hpp"

#include "Nodes/DataProvider/IMU/FileReader/VectorNavFile.hpp" // TODO: Remove when not needed anymore

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

    // Play all the data files
    for (auto& node : pConfig->nodes)
    {
        // TODO: Add DataManager, which polls all data files in correct order instead of just reading the files here
        if (node.type == "VectorNavFile")
        {
            while (std::static_pointer_cast<NAV::VectorNavFile>(node.node)->pollObservation() != nullptr)
                ;
        }
    }

    if (pConfig->GetSigterm())
        NAV::Sleep::waitForSignal();
    else
        NAV::Sleep::countDownSeconds(pConfig->GetProgExecTime());

    NAV::Logger::writeFooter();

    return EXIT_SUCCESS;
}