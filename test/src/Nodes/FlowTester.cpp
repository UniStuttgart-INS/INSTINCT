#include "FlowTester.hpp"

#include <filesystem>
#include <iostream>
#include <vector>

#include "util/Logger.hpp"

#include "internal/FlowExecutor.hpp"
#include "internal/FlowManager.hpp"
#include "NodeRegistry.hpp"
#include "internal/NodeManager.hpp"
#include "util/ConfigManager.hpp"
namespace nm = NAV::NodeManager;

void testFlow(const char* path)
{
    std::vector<const char*> argv = { "", "--nogui", "-l", path, nullptr };

    // Save the root path of the program
    NAV::flow::SetProgramRootPath(std::filesystem::current_path().string());

    // Program configuration
    NAV::ConfigManager configManager;
    NAV::ConfigManager::FetchConfigs(static_cast<int>(argv.size() - 1), argv.data());

    // Register all Node Types which are available to the program
    NAV::NodeRegistry::RegisterNodeTypes();

    // Register all Node Data Types which are available to the program
    NAV::NodeRegistry::RegisterNodeDataTypes();

    LOG_INFO("Starting in No-GUI Mode");

    nm::showFlowWhenInvokingCallbacks = false;
    nm::showFlowWhenNotifyingValueChange = false;

    bool loadSuccessful = false;
    try
    {
        LOG_INFO("Loading flow file: {}", NAV::ConfigManager::Get<std::string>("load", ""));
        loadSuccessful = NAV::flow::LoadFlow(NAV::ConfigManager::Get<std::string>("load", ""));
    }
    catch (const std::exception& e)
    {
        nm::DeleteAllNodes();
        LOG_ERROR("Loading flow file failed");
        throw e;
    }

    if (loadSuccessful)
    {
        LOG_INFO("Starting flow execution");
        NAV::FlowExecutor::start();

        NAV::FlowExecutor::waitForFinish();

        nm::DeleteAllNodes();
    }
}