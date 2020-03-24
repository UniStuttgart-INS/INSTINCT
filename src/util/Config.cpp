#include "Config.hpp"

#include <string>
#include <fstream>
#include <iostream>
#include <boost/program_options/parsers.hpp>

#include "Logger.hpp"

#include "DataProvider/IMU/Sensors/VectorNavSensor.hpp"
#include <boost/tokenizer.hpp>

namespace bpo = boost::program_options;

NAV::Config::Config() {}

NAV::Config::~Config() {}

NAV::Config* NAV::Config::Get()
{
    LOG_TRACE("called");

    static Config pToConfig;
    return &pToConfig;
}

NAV::NavStatus NAV::Config::AddOptions(const int argc, const char* argv[])
{
    LOG_TRACE("called with {} arguments", argc);
    for (int i = 0; i < argc; i++)
        LOG_TRACE("argument[{}] = '{}'", i, argv[i]);

    // clang-format off
    // See https://www.boost.org/doc/libs/1_72_0/doc/html/program_options.html
    program_options.add_options()
        ("config,f", bpo::value<std::vector<std::string>>()->multitoken(), "Read parameters from file")
        ("version,v", "Display the version number")
        ("help,h", "Display this help message")
        ("sigterm", bpo::value<bool>()->default_value(false),"Programm waits for -SIGUSR1 / -SIGINT / -SIGTERM")
        ("duration", bpo::value<size_t>()->default_value(10), "Program execution duration [sec]")
        ("dataProvider", bpo::value<std::vector<std::string>>()->multitoken(), "Data Provider registration.\nFormat: Type, Name, [Options...]")
        ("dataProcessor", bpo::value<std::vector<std::string>>()->multitoken(), "Data Processor registration.\nFormat: Type, Name, [Options...]")
        ("dataLink", bpo::value<std::vector<std::string>>()->multitoken(), "Data Links.\nFormat: From(Name), To(Name)")
    ;
    // clang-format on

    bpo::store(bpo::parse_command_line(argc, argv, program_options), vm);

    // if config file is available, the parameters from file will be added
    if (vm.count("config"))
    {
        for (std::string configFile : vm["config"].as<std::vector<std::string>>())
        {
            std::ifstream ifs{ configFile };
            if (ifs)
                bpo::store(bpo::parse_config_file(ifs, program_options), vm);
            else
            {
                LOG_CRITICAL("Could not open the config file: {}", configFile);
                return NavStatus::NAV_ERROR;
            }
        }
    }
    bpo::notify(vm);

    if (vm.count("help"))
    {
        std::cout << "NavSoS " << PROJECT_VERSION_STRING << " - Navigation Software Stuttgart" << std::endl
                  << std::endl
                  << program_options << std::endl;
        return NavStatus::NAV_REQUEST_EXIT;
    }

    if (vm.count("version"))
    {
        std::cout << PROJECT_VERSION_STRING << std::endl;
        return NavStatus::NAV_REQUEST_EXIT;
    }

    return NavStatus::NAV_OK;
}

NAV::NavStatus NAV::Config::DecodeOptions()
{
    LOG_TRACE("called");

    try
    {
        if (vm.count("config"))
            for (std::string configFile : vm["config"].as<std::vector<std::string>>())
                LOG_DEBUG("Read Option file '{}'", configFile);

        if (vm.count("sigterm"))
        {
            sigterm = vm["sigterm"].as<bool>();
            LOG_DEBUG("Option-sigterm = '{}'", sigterm);
        }
        if (vm.count("duration"))
        {
            progExecTime = vm["duration"].as<size_t>();
            LOG_DEBUG("Option-duration = '{}'", progExecTime);
        }

        std::vector<std::string> names;
        if (vm.count("dataProvider"))
        {
            for (std::string provider : vm["dataProvider"].as<std::vector<std::string>>())
            {
                DataProviderConfig config;

                std::stringstream lineStream(provider);
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
                        config.type = cell;
                    else if (config.name.empty())
                        config.name = cell;
                    else
                        config.options.push_back(cell);
                }

                config.provider = nullptr;
                dataProviders.push_back(config);
                names.push_back(config.name);

                LOG_DEBUG("Option-dataProvider = {}, {}, {}", config.type, config.name, fmt::join(config.options, ", "));
            }
        }
        if (vm.count("dataProcessor"))
        {
            for (std::string processor : vm["dataProcessor"].as<std::vector<std::string>>())
            {
                DataProcessorConfig config;

                std::stringstream lineStream(processor);
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
                        config.type = cell;
                    else if (config.name.empty())
                        config.name = cell;
                    else
                        config.options.push_back(cell);
                }

                config.processor = nullptr;
                dataProcessors.push_back(config);
                names.push_back(config.name);

                LOG_DEBUG("Option-dataProcessor = {}, {}, {}", config.type, config.name, fmt::join(config.options, ", "));
            }
        }
        if (vm.count("dataLink"))
        {
            for (std::string link : vm["dataLink"].as<std::vector<std::string>>())
            {
                DataLinkConfig config;

                std::stringstream lineStream(link);
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

                    if (config.source.empty())
                        config.source = cell;
                    else if (config.target.empty())
                        config.target = cell;
                }

                dataLinks.push_back(config);

                LOG_DEBUG("Option-dataLink = {} ==> {}", config.source, config.target);
            }
        }

        // Check if duplicate names
        auto it = std::unique(names.begin(), names.end());
        if (it != names.end())
        {
            LOG_CRITICAL("Data Provider and Processor names must be unique: '{}'", fmt::join(names, ", "));
            return NavStatus::NAV_ERROR_DUPLICATE_NAMES;
        }

        return NavStatus::NAV_OK;
    }
    catch (const boost::bad_any_cast& e)
    {
        LOG_CRITICAL("Error Decoding Options: '{}'", e.what());
        return NavStatus::NAV_ERROR;
    }
}

bool NAV::Config::GetSigterm()
{
    LOG_TRACE("called with sigterm={}", sigterm);
    return sigterm;
}

size_t NAV::Config::GetProgExecTime()
{
    LOG_TRACE("called with progExecTime={}", progExecTime);
    return progExecTime;
}