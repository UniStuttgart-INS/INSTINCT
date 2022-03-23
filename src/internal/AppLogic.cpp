#include "AppLogic.hpp"

#include <filesystem>
#include <chrono>

#include "NodeRegistry.hpp"
#include "internal/gui/NodeEditorApplication.hpp"
#include "internal/ConfigManager.hpp"
#include "internal/FlowManager.hpp"
#include "internal/FlowExecutor.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "util/Logger.hpp"
#include "Sleep.hpp"

#define BUILD_BUG_ON(condition) ((void)sizeof(char[1 - 2 * !!(condition)]))

int NAV::AppLogic::processCommandLineArguments(int argc, const char* argv[]) // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
{
    // Save the root path of the program
    NAV::flow::SetProgramRootPath(std::filesystem::current_path());

    // Program configuration
    auto failedConfigFiles = NAV::ConfigManager::FetchConfigs(argc, argv);

    // Sets the output path
    NAV::flow::SetOutputPath();

    // Initialize the logger
    Logger logger((NAV::flow::GetOutputPath() / "instinct.log").string());

    // Log all the options
    NAV::ConfigManager::LogOptions(argc, argv);

    for ([[maybe_unused]] const auto& configFile : failedConfigFiles)
    {
        LOG_ERROR("Could not open the config file: {}", configFile);
    }

    // Register all Node Types which are available to the program
    NAV::NodeRegistry::RegisterNodeTypes();

    // Register all Node Data Types which are available to the program
    NAV::NodeRegistry::RegisterNodeDataTypes();

    if (sizeof(long double) != 16)
    {
        LOG_WARN("You are running INSTINCT on a platform without quadruple-precision floating-point support. Functionality concerning time measurements and ranging could be affected by the precision loss.");
    }

    if (NAV::ConfigManager::Get<bool>("nogui"))
    {
        LOG_INFO("Starting in No-GUI Mode");

        if (NAV::ConfigManager::HasKey("load"))
        {
            nm::showFlowWhenInvokingCallbacks = false;
            nm::showFlowWhenNotifyingValueChange = false;

            bool loadSuccessful = false;
            try
            {
                LOG_INFO("Loading flow file: {}", NAV::ConfigManager::Get<std::string>("load", ""));
                loadSuccessful = NAV::flow::LoadFlow(NAV::ConfigManager::Get<std::string>("load", ""));
            }
            catch (...)
            {
                nm::DeleteAllNodes();
                LOG_ERROR("Loading flow file failed");
            }
            if (loadSuccessful)
            {
#ifdef TESTING
                nm::ApplyWatcherCallbacks();
#endif

                auto start = std::chrono::steady_clock::now();
                NAV::FlowExecutor::start();

                NAV::FlowExecutor::waitForFinish();

                if (NAV::ConfigManager::Get<bool>("nogui")
                    && NAV::ConfigManager::Get<bool>("sigterm"))
                {
                    NAV::Sleep::waitForSignal(true);
                }
                else if (size_t duration = NAV::ConfigManager::Get<size_t>("duration");
                         NAV::ConfigManager::Get<bool>("nogui") && duration)
                {
                    auto now = std::chrono::steady_clock::now();
                    std::chrono::duration<double> elapsed = now - start;
                    if (elapsed.count() < static_cast<double>(duration))
                    {
                        NAV::Sleep::countDownSeconds(duration - static_cast<size_t>(elapsed.count()));
                    }
                }

                nm::DisableAllCallbacks();
                nm::DeleteAllNodes();
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
        NAV::gui::NodeEditorApplication app("INSTINCT - INS Toolkit for Integrated Navigation Concepts and Training", "INSTINCT.ini", argc, argv);

        if (app.Create())
        {
            if (NAV::ConfigManager::HasKey("load"))
            {
                try
                {
                    LOG_INFO("Loading flow file: {}", NAV::ConfigManager::Get<std::string>("load", ""));
                    if (NAV::flow::LoadFlow(NAV::ConfigManager::Get<std::string>("load", "")))
                    {
                        app.frameCountNavigate = ImGui::GetFrameCount();
                    }
                }
                catch (...)
                {
                    nm::DeleteAllLinksAndNodes();
                    NAV::flow::DiscardChanges();
                    NAV::flow::SetCurrentFilename("");
                    LOG_ERROR("Loading flow file failed");
                }
            }

            return app.Run();
        }

        LOG_CRITICAL("Could not create the window");
    }

    return EXIT_SUCCESS;
}