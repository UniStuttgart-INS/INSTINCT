#include "ConfigManager.hpp"

#include <string>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <boost/program_options/parsers.hpp>

#include "internal/FlowManager.hpp"
#include "util/Logger.hpp"

#include <boost/tokenizer.hpp>

namespace bpo = boost::program_options;

/// Program option description
bpo::options_description program_options{ "Allowed options" }; // NOLINT

/// Map which stores all options
boost::program_options::variables_map NAV::ConfigManager::vm; // NOLINT

void NAV::ConfigManager::initialize()
{
    LOG_TRACE("called");

    if (program_options.options().empty())
    {
        // clang-format off
        // See https://www.boost.org/doc/libs/1_72_0/doc/html/program_options.html
        program_options.add_options()
            ("config",            bpo::value<std::vector<std::string>>()->multitoken(),             "List of configuration files to read parameters from"                                     )
            ("version,v",                                                                           "Display the version number"                                                              )
            ("help,h",                                                                              "Display this help message"                                                               )
            ("sigterm",           bpo::bool_switch()->default_value(false),                         "Programm waits for -SIGUSR1 / -SIGINT / -SIGTERM"                                        )
            ("duration",          bpo::value<size_t>()->default_value(0),                           "Program execution duration [sec]"                                                        )
            ("nogui",             bpo::bool_switch()->default_value(false),                         "Launch without the gui"                                                                  )
            ("noinit",            bpo::bool_switch()->default_value(false),                         "Do not initialize flows after loading them"                                              )
            ("load,l",            bpo::value<std::string>(),                                        "Flow file to load"                                                                       )
            ("rotate-output",     bpo::bool_switch()->default_value(false),                         "Create new folders for output files"                                                     )
            ("output-path,o",     bpo::value<std::string>()->default_value("logs"),                 "Directory path for logs and output files"                                                )
            ("input-path,i",      bpo::value<std::string>()->default_value("data"),                 "Directory path for searching input files"                                                )
            ("flow-path,f",       bpo::value<std::string>()->default_value("flow"),                 "Directory path for searching flow files"                                                 )
            ("implot-config",     bpo::value<std::string>()->default_value("config/implot.json"),   "Config file to read implot settings from"                                                )
            ("console-log-level", bpo::value<std::string>()->default_value("off"),                  "Log level on the console  (possible values: trace/debug/info/warning/error/critical/off" )
            ("file-log-level",    bpo::value<std::string>()->default_value("debug"),                "Log level to the log file (possible values: trace/debug/info/warning/error/critical/off" )
        ;
        // clang-format on
    }
}

void NAV::ConfigManager::deinitialize()
{
    vm.clear();
}

const boost::program_options::options_description& NAV::ConfigManager::GetProgramOptions()
{
    return program_options;
}

std::vector<std::string> NAV::ConfigManager::FetchConfigs(const int argc, const char* argv[]) // NOLINT
{
    bpo::store(bpo::parse_command_line(argc, argv, program_options), vm);

    std::vector<std::string> failedConfigFiles;

    // if config file is available, the parameters from file will be added
    if (vm.count("config"))
    {
        for (const std::string& configFile : vm["config"].as<std::vector<std::string>>())
        {
            std::filesystem::path filepath{ configFile };
            if (filepath.is_relative())
            {
                filepath = NAV::flow::GetProgramRootPath();
                filepath /= configFile;
            }

            std::ifstream ifs{ filepath };
            if (ifs.good())
            {
                bpo::store(bpo::parse_config_file(ifs, program_options), vm);
            }
            else
            {
                failedConfigFiles.push_back(configFile);
            }
        }
    }

    bpo::notify(vm);

    return failedConfigFiles;
}

void NAV::ConfigManager::CheckOptions(const int argc, [[maybe_unused]] const char* argv[]) // NOLINT
{
    LOG_DEBUG("{} arguments were provided over the command line", argc);

    if (vm["console-log-level"].as<std::string>() != "trace"
        && vm["console-log-level"].as<std::string>() != "debug"
        && vm["console-log-level"].as<std::string>() != "info"
        && vm["console-log-level"].as<std::string>() != "warning"
        && vm["console-log-level"].as<std::string>() != "error"
        && vm["console-log-level"].as<std::string>() != "critical"
        && vm["console-log-level"].as<std::string>() != "off")
    {
        LOG_CRITICAL("The command line argument 'console-log-level' has to be one of 'trace/debug/info/warning/error/critical/off' but the value '{}' was provided", vm["console-log-level"].as<std::string>());
    }
    if (vm["file-log-level"].as<std::string>() != "trace"
        && vm["file-log-level"].as<std::string>() != "debug"
        && vm["file-log-level"].as<std::string>() != "info"
        && vm["file-log-level"].as<std::string>() != "warning"
        && vm["file-log-level"].as<std::string>() != "error"
        && vm["file-log-level"].as<std::string>() != "critical"
        && vm["file-log-level"].as<std::string>() != "off")
    {
        LOG_CRITICAL("The command line argument 'file-log-level' has to be one of 'trace/debug/info/warning/error/critical/off' but the value '{}' was provided", vm["file-log-level"].as<std::string>());
    }

    for (int i = 0; i < argc; i++)
    {
        LOG_DEBUG("\targument[{}] = '{}'", i, argv[i]);
    }

    LOG_DEBUG("{} arguments are set in the allowed variable map", vm.size());

    for (const auto& value : vm)
    {
        if ([[maybe_unused]] const auto* v = boost::any_cast<size_t>(&value.second.value()))
        {
            LOG_DEBUG("\tvm[{}] = '{}'", value.first, *v);
        }
        else if ([[maybe_unused]] const auto* v = boost::any_cast<bool>(&value.second.value()))
        {
            LOG_DEBUG("\tvm[{}] = '{}'", value.first, *v);
        }
        else if ([[maybe_unused]] const auto* v = boost::any_cast<std::string>(&value.second.value()))
        {
            LOG_DEBUG("\tvm[{}] = '{}'", value.first, *v);
        }
        else if ([[maybe_unused]] const auto* v = boost::any_cast<std::vector<std::string>>(&value.second.value()))
        {
            LOG_DEBUG("\tvm[{}] = '{}'", value.first, fmt::join(v->begin(), v->end(), ", "));
        }
        else
        {
            LOG_ERROR("The Log option vm[{}] could not be casted. Please report this to the developers.", value.first);
        }
    }
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