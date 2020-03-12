#include "Config.hpp"

#include <string>
#include <fstream>
#include <iostream>
#include <boost/program_options/parsers.hpp>

#include "Logger.hpp"

namespace bpo = boost::program_options;

NAV::Config::Config() {}

NAV::Config::~Config() {}

NAV::Config* NAV::Config::Get()
{
    static Config pToConfig;
    return &pToConfig;
}

NAV::NavStatus NAV::Config::AddOptions(const int argc, const char* argv[])
{
    // clang-format off
    program_options.add_options()
        ("config,-f", bpo::value<std::string>(), "Read parameters from file")
        ("version,v", "Display the version number")
        ("help,h", "Display this help message")
        ("sigterm", bpo::value<bool>()->default_value(false),"Programm waits for -SIGUSR1 / -SIGINT / -SIGTERM")
        ("duration", bpo::value<size_t>()->default_value(10), "Program execution duration [sec]")
    ;
    // clang-format on

    bpo::store(bpo::parse_command_line(argc, argv, program_options), vm);

    // if config file is available, the parameters from file will be added
    if (vm.count("config"))
    {
        std::ifstream ifs{ vm["config"].as<std::string>() };
        if (ifs)
            bpo::store(bpo::parse_config_file(ifs, program_options), vm);
        else
        {
            std::cout << "Could not open the config file: " << vm["config"].as<std::string>() << std::endl;
            return NAV_ERROR;
        }
    }
    bpo::notify(vm);

    if (vm.count("help"))
    {
        std::cout << "NavSoS " << PROJECT_VERSION_STRING << " - Navigation Software Stuttgart" << std::endl
                  << std::endl
                  << program_options << std::endl;
        return NAV_REQUEST_EXIT;
    }

    if (vm.count("version"))
    {
        std::cout << PROJECT_VERSION_STRING << std::endl;
        return NAV_REQUEST_EXIT;
    }

    return NAV_OK;
}

NAV::NavStatus NAV::Config::DecodeOptions()
{
    try
    {
        if (vm.count("sigterm"))
        {
            sigterm = vm["sigterm"].as<bool>();
            LOG_DEBUG("Option-sigterm: '{}'", sigterm);
        }
        if (vm.count("duration"))
        {
            progExecTime = vm["duration"].as<size_t>();
            LOG_DEBUG("Option-duration: '{}'", progExecTime);
        }
        return NAV_OK;
    }
    catch (const boost::bad_any_cast& e)
    {
        LOG_ERROR("Error Decoding Options: '{}'", e.what());
        return NAV_ERROR;
    }
}

bool NAV::Config::GetSigterm()
{
    return sigterm;
}

size_t NAV::Config::GetProgExecTime()
{
    return progExecTime;
}