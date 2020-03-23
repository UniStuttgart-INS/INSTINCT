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

#include "DataCallback.hpp"

#include "DataProvider/IMU/Sensors/VectorNavSensor.hpp"
#include "DataProvider/IMU/FileReader/VectorNavFile.hpp"
#include "DataProcessor/DataLogger/IMU/VectorNavDataLogger.hpp"

#include "ub/protocol/types.hpp"
#include "DataProvider/GNSS/Sensors/UbloxSensor.hpp"
#include "DataProcessor/DataLogger/GNSS/UbloxDataLogger.hpp"
#include "DataProcessor/UsbSync/GNSS/UbloxSyncSignal.hpp"

void exitFailure()
{
    NAV::Logger::writeFooter();
    exit(EXIT_FAILURE);
}

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
    // Decode Options
    if (pConfig->DecodeOptions() != NAV::NavStatus::NAV_OK)
        exitFailure();

    // Create Data Providers which were specified in the configs
    LOG_INFO("Creating {} Data Provider{}", pConfig->dataProviders.size(), pConfig->dataProviders.size() > 1 ? "s" : "");
    for (auto& dataProvider : pConfig->dataProviders)
    {
        if (dataProvider.type == "VectorNavSensor")
        {
            NAV::VectorNavSensor::Config config;

            if (dataProvider.options.size() >= 1)
                config.outputFrequency = static_cast<uint16_t>(std::stoul(dataProvider.options.at(0)));
            if (dataProvider.options.size() >= 2)
                config.sensorPort = dataProvider.options.at(1);
            if (dataProvider.options.size() >= 3)
                config.sensorBaudrate = static_cast<NAV::UartSensor::Baudrate>(std::stoul(dataProvider.options.at(2)));

            dataProvider.provider = std::make_shared<NAV::VectorNavSensor>(dataProvider.name, config);
        }
        else if (dataProvider.type == "VectorNavFile")
        {
            NAV::VectorNavFile::Config config;
            std::string path;

            if (dataProvider.options.size() >= 1)
                path = dataProvider.options.at(0);

            dataProvider.provider = std::make_shared<NAV::VectorNavFile>(dataProvider.name, path, config);
        }
        else if (dataProvider.type == "UbloxSensor")
        {
            NAV::UbloxSensor::Config config;

            if (dataProvider.options.size() >= 1)
                config.outputFrequency = static_cast<uint16_t>(std::stoul(dataProvider.options.at(0)));
            if (dataProvider.options.size() >= 2)
                config.sensorPort = dataProvider.options.at(1);
            if (dataProvider.options.size() >= 3)
                config.sensorBaudrate = static_cast<NAV::UartSensor::Baudrate>(std::stoul(dataProvider.options.at(2)));

            dataProvider.provider = std::make_shared<NAV::UbloxSensor>(dataProvider.name, config);
        }
        else
        {
            LOG_CRITICAL("Data Provider {} - {} has unknown type", dataProvider.type, dataProvider.name);
            exitFailure();
        }

        // Check if there was a problem with creation and then initialize
        if (dataProvider.provider == nullptr || dataProvider.provider->initialize() != NAV::NavStatus::NAV_OK)
        {
            LOG_CRITICAL("Data Provider {} - {} could not be created", dataProvider.type, dataProvider.name);
            exitFailure();
        }
        else
            LOG_INFO("{}═⇒ {} ({}) created", dataProvider.name == pConfig->dataProviders.back().name ? "╚" : "╠", dataProvider.name, dataProvider.type);
    }

    // Create Data Processors
    LOG_INFO("Creating {} Data Processor{}", pConfig->dataProcessors.size(), pConfig->dataProcessors.size() > 1 ? "s" : "");
    for (auto& dataProcessor : pConfig->dataProcessors)
    {
        if (dataProcessor.type == "VectorNavDataLogger")
        {
            std::string path;
            bool isBinary = false;

            if (dataProcessor.options.size() >= 1)
                path = dataProcessor.options.at(0);
            if (dataProcessor.options.size() >= 2)
            {
                if (dataProcessor.options.at(1) == "ascii")
                    isBinary = false;
                else if (dataProcessor.options.at(1) == "binary")
                    isBinary = true;
                else
                    LOG_WARN("Data Processor {} - {} has unknown file type {}. Using ascii instead", dataProcessor.type, dataProcessor.name, dataProcessor.options.at(1));
            }

            dataProcessor.processor = std::make_shared<NAV::VectorNavDataLogger>(dataProcessor.name, path, isBinary);
        }
        else if (dataProcessor.type == "UbloxDataLogger")
        {
            std::string path;
            bool isBinary = true;

            if (dataProcessor.options.size() >= 1)
                path = dataProcessor.options.at(0);
            if (dataProcessor.options.size() >= 2)
            {
                if (dataProcessor.options.at(1) == "ascii")
                    isBinary = false;
                else if (dataProcessor.options.at(1) == "binary")
                    isBinary = true;
                else
                    LOG_WARN("Data Processor {} - {} has unknown file type {}. Using binary instead", dataProcessor.type, dataProcessor.name, dataProcessor.options.at(1));
            }

            dataProcessor.processor = std::make_shared<NAV::UbloxDataLogger>(dataProcessor.name, path, isBinary);
        }
        else if (dataProcessor.type == "UbloxSyncSignal")
        {
            //SensorPort           type msgClass msgId
            std::string port;
            ub::protocol::uart::UbxClass ubxClass;
            uint8_t ubxMsgId;

            if (dataProcessor.options.size() >= 1)
                port = dataProcessor.options.at(0);
            if (dataProcessor.options.size() >= 4)
            {
                if (dataProcessor.options.at(1) == "UBX")
                {
                    ubxClass = ub::protocol::uart::getMsgClassFromString(dataProcessor.options.at(2));
                    ubxMsgId = ub::protocol::uart::getMsgIdFromString(ubxClass, dataProcessor.options.at(3));

                    dataProcessor.processor = std::make_shared<NAV::UbloxSyncSignal>(dataProcessor.name, port, ubxClass, ubxMsgId);
                }
                else
                {
                    LOG_CRITICAL("Data Processor {} - {} has unknown type {}", dataProcessor.type, dataProcessor.name, dataProcessor.options.at(1));
                    exitFailure();
                }
            }
            else
            {
                LOG_CRITICAL("Data Processor {} - {} has not enough options", dataProcessor.type, dataProcessor.name);
                exitFailure();
            }
        }
        else
        {
            LOG_CRITICAL("Data Processor {} - {} has unknown type", dataProcessor.type, dataProcessor.name);
            exitFailure();
        }

        // Check if there was a problem with creation and then initialize
        if (dataProcessor.processor == nullptr || dataProcessor.processor->initialize() != NAV::NavStatus::NAV_OK)
        {
            LOG_CRITICAL("Data Processor {} - {} could not be created", dataProcessor.type, dataProcessor.name);
            exitFailure();
        }
        else
            LOG_INFO("{}═⇒ {} ({}) created", dataProcessor.name == pConfig->dataProcessors.back().name ? "╚" : "╠", dataProcessor.name, dataProcessor.type);
    }

    // Set up Data Links
    LOG_INFO("Creating {} Data Link{}", pConfig->dataLinks.size(), pConfig->dataLinks.size() > 1 ? "s" : "");
    for (auto& dataLink : pConfig->dataLinks)
    {
        bool linkEstablished = false;
        // Find the target
        std::vector<NAV::Config::DataProcessorConfig>::iterator target;
        for (target = pConfig->dataProcessors.begin(); target != pConfig->dataProcessors.end(); target++)
            if (target->name == dataLink.target)
                break;

        if (target == pConfig->dataProcessors.end())
        {
            LOG_CRITICAL("Data Link {} ⇒ {} could not be created because target could not be found", dataLink.source, dataLink.target);
            exitFailure();
        }

        // Find the source
        std::shared_ptr<NAV::DataCallback> source = nullptr;
        std::string sourceType;
        for (auto& dataProvider : pConfig->dataProviders)
        {
            if (dataProvider.name == dataLink.source)
            {
                source = dataProvider.provider;
                sourceType = dataProvider.type;
                break;
            }
        }
        if (source == nullptr)
        {
            for (auto& dataProcessor : pConfig->dataProcessors)
            {
                if (dataProcessor.name == dataLink.source)
                {
                    source = dataProcessor.processor;
                    break;
                }
            }
        }
        if (source == nullptr)
        {
            LOG_CRITICAL("Data Link {} ⇒ {} could not be created because source could not be found", dataLink.source, dataLink.target);
            exitFailure();
        }

        // Source and target were found
        if (sourceType == "VectorNavSensor" || sourceType == "VectorNavFile")
        {
            if (target->type == "VectorNavDataLogger")
            {
                source->addCallback(std::static_pointer_cast<NAV::VectorNavDataLogger>(target->processor)->writeObservation,
                                    std::static_pointer_cast<NAV::VectorNavDataLogger>(target->processor));
                source->callbacksEnabled = true;
                linkEstablished = true;
            }
        }
        else if (sourceType == "UbloxSensor")
        {
            if (target->type == "UbloxDataLogger")
            {
                source->addCallback(std::static_pointer_cast<NAV::UbloxDataLogger>(target->processor)->writeObservation,
                                    std::static_pointer_cast<NAV::UbloxDataLogger>(target->processor));
                source->callbacksEnabled = true;
                linkEstablished = true;
            }
            else if (target->type == "UbloxSyncSignal")
            {
                source->addCallback(std::static_pointer_cast<NAV::UbloxSyncSignal>(target->processor)->triggerSync,
                                    std::static_pointer_cast<NAV::UbloxSyncSignal>(target->processor));
                source->callbacksEnabled = true;
                linkEstablished = true;
            }
        }

        if (linkEstablished)
        {
            LOG_INFO("{}═⇒ {} ⇒ {} created", (dataLink.source == pConfig->dataLinks.back().source && dataLink.target == pConfig->dataLinks.back().target) ? "╚" : "╠",
                     dataLink.source, dataLink.target);
        }
        else
        {
            LOG_CRITICAL("Data Link {} ⇒ {} could not be created because link generation for types {} ⇒ {} is not supported.", dataLink.source, dataLink.target, sourceType, target->type);
            exitFailure();
        }
    }

    // Play all the data files
    for (auto& dataProvider : pConfig->dataProviders)
    {
        // TODO: Add DataManager, which polls all data files in correct order
        if (dataProvider.type == "VectorNavFile")
        {
            while (std::static_pointer_cast<NAV::VectorNavFile>(dataProvider.provider)->pollObservation() != nullptr)
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