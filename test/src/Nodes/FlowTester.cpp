#include "FlowTester.hpp"

#include <vector>

#include "internal/AppLogic.hpp"

void testFlow(const char* path)
{
    std::vector<const char*> argv = { "", "--nogui", "-l", path, nullptr };

    NAV::AppLogic::processCommandLineArguments(static_cast<int>(argv.size() - 1), argv.data());
}