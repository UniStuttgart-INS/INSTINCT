#include "gui/NodeEditorApplication.hpp"

#include <iostream>

#include "util/Logger.hpp"
#include "util/Version.hpp"
#include "util/ConfigManager.hpp"

#include "NodeRegistry.hpp"
#include "internal/FlowManager.hpp"
#include "internal/FlowExecutor.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

int Main(int argc, const char* argv[])
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
                    nm::DeleteAllLinks();
                    nm::DeleteAllNodes();
                    NAV::flow::DiscardChanges();
                    NAV::flow::SetCurrentFilename("");
                    LOG_ERROR("Loading flow file failed");
                }
                if (loadSuccessful)
                {
                    NAV::FlowExecutor flowExecutor;

                    flowExecutor.start();

                    flowExecutor.waitForFinish();
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