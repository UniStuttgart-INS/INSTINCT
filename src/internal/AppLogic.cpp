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
#include "util/Sleep.hpp"

int NAV::AppLogic::processCommandLineArguments(int argc, const char* argv[]) // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
{
    // Save the root path of the program
    NAV::flow::SetProgramRootPath(std::filesystem::current_path().string());

    // Program configuration
    NAV::ConfigManager::FetchConfigs(argc, argv);

    // Register all Node Types which are available to the program
    NAV::NodeRegistry::RegisterNodeTypes();

    // Register all Node Data Types which are available to the program
    NAV::NodeRegistry::RegisterNodeDataTypes();

    if (NAV::ConfigManager::Get<bool>("nogui", false))
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

                if (NAV::ConfigManager::Get<bool>("nogui", false)
                    && NAV::ConfigManager::Get<bool>("sigterm", false))
                {
                    NAV::Sleep::waitForSignal(true);
                }
                else if (size_t duration = NAV::ConfigManager::Get<size_t>("duration", 0);
                         NAV::ConfigManager::Get<bool>("nogui", false) && duration)
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