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
    program_options.add_options()
        ("config,-f", bpo::value<std::string>(), "Read parameters from file")
        ("version,v", "Display the version number")
        ("help,h", "Display this help message")
        ("sigterm", bpo::value<bool>()->default_value(false),"Programm waits for -SIGUSR1 / -SIGINT / -SIGTERM")
        ("duration", bpo::value<size_t>()->default_value(10), "Program execution duration [sec]")
        ("vn100", bpo::value<bool>()->default_value(false), "Enables VN100 Logging")
        ("vn110", bpo::value<bool>()->default_value(false), "Enables VN110 Logging")
        ("ublox", bpo::value<bool>()->default_value(false), "Enables ublox Logging")
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
    LOG_TRACE("called");

    try
    {
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
        if (vm.count("vn100"))
        {
            vn100Enabled = vm["vn100"].as<bool>();
            LOG_DEBUG("Option-vn100 = '{}'", vn100Enabled);
        }
        if (vm.count("vn110"))
        {
            vn110Enabled = vm["vn110"].as<bool>();
            LOG_DEBUG("Option-vn110 = '{}'", vn110Enabled);
        }
        if (vm.count("ublox"))
        {
            ubloxEnabled = vm["ublox"].as<bool>();
            LOG_DEBUG("Option-ublox = '{}'", ubloxEnabled);
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
    LOG_TRACE("called with sigterm={}", sigterm);
    return sigterm;
}

size_t NAV::Config::GetProgExecTime()
{
    LOG_TRACE("called with progExecTime={}", progExecTime);
    return progExecTime;
}

bool NAV::Config::isVN100Enabled()
{
    LOG_TRACE("called with vn100Enabled={}", vn100Enabled);
    return vn100Enabled;
}
bool NAV::Config::isVN110Enabled()
{
    LOG_TRACE("called with vn110Enabled={}", vn110Enabled);
    return vn110Enabled;
}
bool NAV::Config::isUbloxEnabled()
{
    LOG_TRACE("called with ubloxEnabled={}", ubloxEnabled);
    return ubloxEnabled;
}