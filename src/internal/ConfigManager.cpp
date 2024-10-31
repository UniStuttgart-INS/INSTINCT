// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ConfigManager.hpp"

#include <string>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <boost/program_options/parsers.hpp>
#include <implot.h>
#include <implot_internal.h>

#include "internal/FlowManager.hpp"
#include "util/Logger.hpp"

#include <boost/tokenizer.hpp>

#include "util/Json.hpp"
#include "util/Plot/Colormap.hpp"
#include "internal/gui/windows/Screenshotter.hpp"

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
            ("config",            bpo::value<std::vector<std::string>>()->multitoken(),             "List of configuration files to read parameters from"                                         )
            ("version,v",                                                                           "Display the version number"                                                                  )
            ("help,h",                                                                              "Display this help message"                                                                   )
            ("sigterm",           bpo::bool_switch()->default_value(false),                         "Programm waits for -SIGUSR1 / -SIGINT / -SIGTERM"                                            )
            ("duration",          bpo::value<size_t>()->default_value(0),                           "Program execution duration [sec]"                                                            )
            ("nogui",             bpo::bool_switch()->default_value(false),                         "Launch without the gui"                                                                      )
            ("noinit",            bpo::bool_switch()->default_value(false),                         "Do not initialize flows after loading them"                                                  )
            ("load,l",            bpo::value<std::string>(),                                        "Flow file to load"                                                                           )
            ("rotate-output",     bpo::bool_switch()->default_value(false),                         "Create new folders for output files"                                                         )
            ("output-path,o",     bpo::value<std::string>()->default_value("logs"),                 "Directory path for logs and output files"                                                    )
            ("input-path,i",      bpo::value<std::string>()->default_value("data"),                 "Directory path for searching input files"                                                    )
            ("flow-path,f",       bpo::value<std::string>()->default_value("flow"),                 "Directory path for searching flow files"                                                     )
            ("implot-config",     bpo::value<std::string>()->default_value("implot.json"),          "Config file to read implot settings from"                                                    )
            ("global-log-level",  bpo::value<std::string>()->default_value("trace"),                "Global log level of all sinks (possible values: trace/debug/info/warning/error/critical/off" )
            ("console-log-level", bpo::value<std::string>()->default_value("info"),                 "Log level on the console      (possible values: trace/debug/info/warning/error/critical/off" )
            ("file-log-level",    bpo::value<std::string>()->default_value("debug"),                "Log level to the log file     (possible values: trace/debug/info/warning/error/critical/off" )
            ("flush-log-level",    bpo::value<std::string>()->default_value("info"),               "Log level to flush on         (possible values: trace/debug/info/warning/error/critical/off" )
            ("log-filter",        bpo::value<std::string>(),                                        "Filter/Regex for log messages"                                                               )
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

    for (const char* logger : { "global-log-level", "console-log-level", "file-log-level", "flush-log-level" })
    {
        if (vm[logger].as<std::string>() != "trace"
            && vm[logger].as<std::string>() != "debug"
            && vm[logger].as<std::string>() != "info"
            && vm[logger].as<std::string>() != "warning"
            && vm[logger].as<std::string>() != "error"
            && vm[logger].as<std::string>() != "critical"
            && vm[logger].as<std::string>() != "off")
        {
            LOG_CRITICAL("The command line argument '{}' has to be one of 'trace/debug/info/warning/error/critical/off' but the value '{}' was provided", logger, vm[logger].as<std::string>());
        }
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
        else if ([[maybe_unused]] const auto* v = boost::any_cast<bool>(&value.second.value())) // NOLINT(bugprone-bool-pointer-implicit-conversion)
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

void NAV::ConfigManager::SaveGlobalSettings()
{
    // Save also global settings
    std::ofstream filestream(flow::GetConfigPath() / "globals.json");
    json j;
#ifdef IMGUI_IMPL_OPENGL_LOADER_GL3W
    j["plotScreenshotImPlotStyleFile"] = gui::windows::plotScreenshotImPlotStyleFile;
    j["copyScreenshotsToClipboard"] = gui::windows::copyScreenshotsToClipboard;
#endif
    j["colormaps"] = ColormapsGlobal;

    ImPlotContext& gp = *ImPlot::GetCurrentContext();
    j["selectedImPlotColormap"] = gp.Style.Colormap;

    constexpr int CMAP_USER_START = ImPlotColormap_Greys + 2;
    for (int i = CMAP_USER_START; i < gp.ColormapData.Count; ++i)
    {
        j["ImPlotColormaps"][static_cast<size_t>(i - CMAP_USER_START)]["name"] = gp.ColormapData.GetName(i);
        j["ImPlotColormaps"][static_cast<size_t>(i - CMAP_USER_START)]["qual"] = gp.ColormapData.IsQual(i);
        for (int c = 0; c < gp.ColormapData.GetKeyCount(i); ++c)
        {
            j["ImPlotColormaps"][static_cast<size_t>(i - CMAP_USER_START)]["colors"][static_cast<size_t>(c)] =
                ImGui::ColorConvertU32ToFloat4(gp.ColormapData.GetKeyColor(i, c));
        }
    }

    filestream << std::setw(4) << j << std::endl; // NOLINT(performance-avoid-endl)
}

void NAV::ConfigManager::LoadGlobalSettings()
{
    auto filepath = flow::GetConfigPath() / "globals.json";
    std::ifstream filestream(filepath);

    if (!filestream.good())
    {
        LOG_ERROR("Load Flow error: Could not open file: {}", filepath);
        return;
    }
    json j;
    filestream >> j;

    if (j.contains("colormaps"))
    {
        j.at("colormaps").get_to(ColormapsGlobal);
    }
#ifdef IMGUI_IMPL_OPENGL_LOADER_GL3W
    if (j.contains("plotScreenshotImPlotStyleFile"))
    {
        j.at("plotScreenshotImPlotStyleFile").get_to(gui::windows::plotScreenshotImPlotStyleFile);
    }
    if (j.contains("copyScreenshotsToClipboard"))
    {
        j.at("copyScreenshotsToClipboard").get_to(gui::windows::copyScreenshotsToClipboard);
    }
#endif
    if (j.contains("ImPlotColormaps"))
    {
        for (size_t i = 0; i < j["ImPlotColormaps"].size(); ++i)
        {
            ImVector<ImVec4> custom;
            for (const auto& c : j.at("ImPlotColormaps").at(i).at("colors"))
            {
                custom.push_back(c.get<ImVec4>());
            }

            ImPlot::AddColormap(j.at("ImPlotColormaps").at(i).at("name").get<std::string>().c_str(),
                                custom.Data, custom.Size, j.at("ImPlotColormaps").at(i).at("qual").get<bool>());
            ImPlot::BustItemCache();
        }
    }
    if (j.contains("selectedImPlotColormap"))
    {
        ImPlotContext& gp = *ImPlot::GetCurrentContext();
        gp.Style.Colormap = j.at("selectedImPlotColormap").get<int>();
        ImPlot::BustItemCache();
    }
}