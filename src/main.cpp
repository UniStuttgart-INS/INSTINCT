#include "gui/NodeEditorApplication.hpp"

#include <iostream>
#include <chrono>

#include "util/Logger.hpp"
#include "util/Version.hpp"
#include "util/ConfigManager.hpp"
#include "util/Sleep.hpp"

#include "NodeRegistry.hpp"
#include "internal/FlowManager.hpp"
#include "internal/FlowExecutor.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

int Main(int argc, const char* argv[]) // NOLINT(cppcoreguidelines-avoid-c-arrays,hicpp-avoid-c-arrays,modernize-avoid-c-arrays)
{
    // Config Manager object
    NAV::ConfigManager configManager;

    if (argc == 2)
    {
        // User requested the version of the program
        if (!strcmp(argv[1], "--version") || !strcmp(argv[1], "-v"))
        {
            std::cout << PROJECT_VERSION_STRING << '\n';
            return EXIT_SUCCESS;
        }

        // User requested the help text of the program
        if (!strcmp(argv[1], "--help") || !strcmp(argv[1], "-h"))
        {
            std::cout << "NavSoS " << PROJECT_VERSION_STRING << " - Navigation Software Stuttgart\n\n"
                      << NAV::ConfigManager::GetProgramOptions() << '\n';
            return EXIT_SUCCESS;
        }
    }

    try
    {
        Logger logger("logs/navsos.log");

        // Program configuration
        NAV::ConfigManager::FetchConfigs(argc, argv);

        // Register all Node Types which are available to the program
        NAV::NodeRegistry::registerNodeTypes();

        // Register all Node Data Types which are available to the program
        NAV::NodeRegistry::registerNodeDataTypes();

        if (NAV::ConfigManager::Get<bool>("nogui", false))
        {
            LOG_INFO("Starting in No-GUI Mode");

            if (NAV::ConfigManager::HasKey("load"))
            {
                nm::showFlowWhenInvokingCallbacks = false;

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
                    auto start = std::chrono::high_resolution_clock::now();
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
                        auto now = std::chrono::high_resolution_clock::now();
                        std::chrono::duration<double> elapsed = now - start;
                        if (elapsed.count() < static_cast<double>(duration))
                        {
                            NAV::Sleep::countDownSeconds(duration - static_cast<size_t>(elapsed.count()));
                        }
                    }

                    nm::DeleteAllNodes();
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
            NAV::gui::NodeEditorApplication app("NavSoS - Navigation Software Stuttgart (Institute of Navigation)", "NavSoS.ini", argc, argv);

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
                        nm::DeleteAllLinks();
                        nm::DeleteAllNodes();
                        NAV::flow::DiscardChanges();
                        NAV::flow::SetCurrentFilename("");
                        LOG_ERROR("Loading flow file failed");
                    }
                }

                return app.Run();
            }
        }

        return EXIT_SUCCESS;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Critical Event occurred: " << e.what() << '\n';
        return EXIT_FAILURE;
    }
}