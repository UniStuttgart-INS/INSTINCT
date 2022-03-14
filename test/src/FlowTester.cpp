#include "FlowTester.hpp"

#include <vector>

#include "internal/AppLogic.hpp"
#include "internal/ConfigManager.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

bool testFlow(const char* path)
{
    // Config Manager object
    NAV::ConfigManager::initialize();

    std::vector<const char*> argv = { "",
                                      "--nogui",
                                      "-l", path,
                                      "--input-path=test/data",
                                      "--output-path=test/logs", nullptr };

    int executionFailure = NAV::AppLogic::processCommandLineArguments(static_cast<int>(argv.size() - 1), argv.data());

    nm::ClearRegisteredCallbacks();

    NAV::ConfigManager::deinitialize();

    return !static_cast<bool>(executionFailure);
}