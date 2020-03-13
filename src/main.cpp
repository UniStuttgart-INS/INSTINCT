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

#include "DataProvider/IMU/Sensors/VectorNavSensor.hpp"

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
        return EXIT_FAILURE;

    // Write the Log Header
    NAV::Logger::writeHeader();

    if (pConfig->DecodeOptions() != NAV::NavStatus::NAV_OK)
        return EXIT_FAILURE;

    if (pConfig->isVN100Enabled())
    {
        // warning: C++ designated initializers only available with ‘-std=c++2a’ or ‘-std=gnu++2a’ [-Wpedantic]
        // TODO: Remove after changing to C++20
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
        if (NAV::VectorNavSensor vn100("VN-100",
                                       { .outputFrequency = 2,
                                         // If the sensor port is wrong, all available ports will be searched (takes ~5 seconds)
                                         .sensorPort = "/dev/ttyUSB_VN100_1" });
            vn100.initialize() == NAV::NavStatus::NAV_OK)
#pragma GCC diagnostic pop
        {
        }
    }

    if (pConfig->isVN110Enabled())
    {
        // warning: C++ designated initializers only available with ‘-std=c++2a’ or ‘-std=gnu++2a’ [-Wpedantic]
        // TODO: Remove after changing to C++20
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
        if (NAV::VectorNavSensor vn100("VN-110",
                                       { .outputFrequency = 2,
                                         // If the sensor port is wrong, all available ports will be searched (takes ~5 seconds)
                                         .sensorPort = "/dev/ttyUSB_VN110_1" });
            vn100.initialize() == NAV::NavStatus::NAV_OK)
#pragma GCC diagnostic pop
        {
        }
    }

    if (pConfig->isUbloxEnabled())
    {
        LOG_INFO("ublox enabled");
    }

    if (pConfig->GetSigterm())
        NAV::Sleep::waitForSignal();
    else
        NAV::Sleep::countDownSeconds(pConfig->GetProgExecTime());

    LOG_TRACE("Trace");
    LOG_DEBUG("Debug");
    LOG_INFO("Info");
    LOG_WARN("Warn");
    LOG_ERROR("Error");
    LOG_CRITICAL("Critical");

    NAV::Logger::writeFooter();
}