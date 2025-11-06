// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "AppLogic.hpp"

// <boost/asio.hpp> needs to be included before <winsock.h> (even though not used in this file)
// https://stackoverflow.com/questions/9750344/boostasio-winsock-and-winsock-2-compatibility-issue
#ifdef _WIN32
    // Set the proper SDK version before including boost/Asio
    #include <SDKDDKVer.h>
    // Note boost/ASIO includes Windows.h.
    #include <boost/asio.hpp>
#endif //_WIN32

#include <filesystem>
#include <chrono>

#include "NodeRegistry.hpp"
#include "Navigation/GNSS/Positioning/AntexReader.hpp"
#include "internal/gui/NodeEditorApplication.hpp"
#include "internal/ConfigManager.hpp"
#include "internal/FlowManager.hpp"
#include "internal/FlowExecutor.hpp"

#include "util/Logger.hpp"
#include "util/Time/TimeBase.hpp"
#include "Sleep.hpp"

#ifdef TESTING
    #include "FlowTester.hpp"
#endif

#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2 * !!(condition)]))

int NAV::AppLogic::processCommandLineArguments(int argc, const char* argv[]) // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
{
    // Save the root path of the program
    flow::SetProgramRootPath(std::filesystem::current_path());

    // Program configuration
    auto failedConfigFiles = ConfigManager::FetchConfigs(argc, argv);

    // Sets the output path
    flow::SetOutputPath();

#ifndef TESTING
    // Initialize the logger
    Logger logger((flow::GetOutputPath() / "instinct.log").string());
#endif

    // Log all the options
    ConfigManager::CheckOptions(argc, argv);

    for ([[maybe_unused]] const auto& configFile : failedConfigFiles)
    {
        LOG_ERROR("Could not open the config file: {}", configFile);
    }

    // Register all Node Types which are available to the program
    NodeRegistry::RegisterNodeTypes();

    // Register all Node Data Types which are available to the program
    NodeRegistry::RegisterNodeDataTypes();

    AntexReader::Get().initialize();

    util::time::SetCurrentTimeToComputerTime();

    if (sizeof(long double) != 16)
    {
        LOG_WARN("You are running INSTINCT on a platform without quadruple-precision floating-point support. Functionality concerning time measurements and ranging could be affected by the precision loss.");
    }

    if (ConfigManager::Get<bool>("nogui"))
    {
        LOG_INFO("Starting in No-GUI Mode");

        if (ConfigManager::HasKey("load"))
        {
            gui::NodeEditorApplication::showFlowWhenInvokingCallbacks = false;
            gui::NodeEditorApplication::showFlowWhenNotifyingValueChange = false;

            bool loadSuccessful = false;
            try
            {
                loadSuccessful = flow::LoadFlow(ConfigManager::Get<std::string>("load", ""));
            }
            catch (...)
            {
                flow::DeleteAllNodes();
                LOG_ERROR("Loading flow file failed");
            }
            if (loadSuccessful)
            {
#ifdef TESTING
                flow::ApplyWatcherCallbacks();
#endif

                FlowExecutor::start();

                if (ConfigManager::Get<bool>("nogui")
                    && (ConfigManager::Get<bool>("sigterm") || ConfigManager::Get<size_t>("duration")))
                {
                    auto interruptThread = std::thread([]() {
                        if (ConfigManager::Get<bool>("nogui")
                            && ConfigManager::Get<bool>("sigterm"))
                        {
                            Sleep::waitForSignal(true);
                            FlowExecutor::stop();
                        }
                        else if (size_t duration = ConfigManager::Get<size_t>("duration");
                                 ConfigManager::Get<bool>("nogui") && duration)
                        {
                            Sleep::countDownSeconds(duration);
                            FlowExecutor::stop();
                        }
                    });
                    interruptThread.join();
                }
                else
                {
                    FlowExecutor::waitForFinish();
                }

#ifdef TESTING
                TESTS::runGeneralFlowCleanupChecks();
                flow::CallCleanupCallback();
#endif

                flow::DisableAllCallbacks();
                flow::DeleteAllNodes();
            }
            else
            {
                return EXIT_FAILURE;
            }
        }
        else
        {
            LOG_CRITICAL("When running in No-GUI Mode you have to specify a flow file to load (-l)");
        }
    }
    else
    {
        LOG_INFO("Starting the GUI");
        gui::NodeEditorApplication app("INSTINCT - INS Toolkit for Integrated Navigation Concepts and Training", "INSTINCT.ini", argc, argv);

        if (app.Create())
        {
            if (ConfigManager::HasKey("load"))
            {
                LOG_INFO("Loading flow file: {}", ConfigManager::Get<std::string>("load", ""));
                if (flow::LoadFlow(ConfigManager::Get<std::string>("load", "")))
                {
                    app.frameCountNavigate = ImGui::GetFrameCount();
                }
                else
                {
                    flow::DeleteAllNodes();
                    flow::DiscardChanges();
                    flow::SetCurrentFilename("");
                }
            }

            return app.Run();
        }

        LOG_CRITICAL("Could not create the window");
    }

    return EXIT_SUCCESS;
}