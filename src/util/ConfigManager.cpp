#include "ConfigManager.hpp"

#include <string>
#include <fstream>
#include <iostream>
#include <boost/program_options/parsers.hpp>

#include "Logger.hpp"

#include <boost/tokenizer.hpp>

namespace bpo = boost::program_options;

bpo::options_description NAV::ConfigManager::program_options{ "Allowed options" }; // NOLINT

boost::program_options::variables_map NAV::ConfigManager::vm; // NOLINT

NAV::ConfigManager::ConfigManager()
{
    LOG_TRACE("called");

    // clang-format off
    // See https://www.boost.org/doc/libs/1_72_0/doc/html/program_options.html
    program_options.add_options()
        ("config,f", bpo::value<std::vector<std::string>>()->multitoken(), "List of configuration files to read parameters from")
        ("version,v", "Display the version number")
        ("help,h", "Display this help message")
        ("sigterm", bpo::bool_switch()->default_value(false), "Programm waits for -SIGUSR1 / -SIGINT / -SIGTERM")
        ("duration", bpo::value<size_t>()->default_value(0), "Program execution duration [sec]")
        ("nogui", bpo::bool_switch()->default_value(false), "Launch without the gui")
        ("load,l", bpo::value<std::string>(), "Flow file to load" )
    ;
    // clang-format on
}

const boost::program_options::options_description& NAV::ConfigManager::GetProgramOptions()
{
    return program_options;
}

void NAV::ConfigManager::FetchConfigs(const int argc, const char* argv[]) // NOLINT
{
    LOG_TRACE("called with {} arguments", argc);

    for (int i = 0; i < argc; i++)
    {
        LOG_TRACE("argument[{}] = '{}'", i, argv[i]);
    }

    bpo::store(bpo::parse_command_line(argc, argv, program_options), vm);

    // if config file is available, the parameters from file will be added
    if (vm.count("config"))
    {
        for (const std::string& configFile : vm["config"].as<std::vector<std::string>>())
        {
            std::ifstream ifs{ configFile };
            if (ifs.good())
            {
                bpo::store(bpo::parse_config_file(ifs, program_options), vm);
            }
            else
            {
                LOG_ERROR("Could not open the config file: {}", configFile);
            }
        }
    }

    bpo::notify(vm);
}

bool NAV::ConfigManager::HasKey(const std::string& key)
{
    return vm.count(key);
}

std::vector<std::string> NAV::ConfigManager::GetKeys()
{
    std::vector<std::string> keys;

    for (auto& param : vm)
    {
        keys.push_back(param.first);
    }

    return keys;
}