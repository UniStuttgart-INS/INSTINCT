// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "FlowTester.hpp"

#include <catch2/catch_test_macros.hpp>
#include <vector>
#include <string>

#include "internal/Node/Node.hpp"
#include "internal/AppLogic.hpp"
#include "internal/ConfigManager.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

namespace NAV::TESTS
{

std::vector<const char*> argv = { "",
                                  "--nogui",
                                  "-l", "path",
                                  "inputPath",
                                  "outputPath",
                                  "--console-log-level=trace", // trace/debug/info/warning/error/critical/off
                                  "--file-log-level=trace",    // trace/debug/info/warning/error/critical/off

                                  //   "--flush-log-level=trace",   // trace/debug/info/warning/error/critical/off
                                  //   "--log-filter=GnssObsComparisons|UbloxGnssObsConverterTests|UbloxGnssObsConverter.cpp",
                                  nullptr };

} // namespace NAV::TESTS

bool NAV::TESTS::testFlow(const char* path, bool useTestDirectories)
{
    // Config Manager object
    NAV::ConfigManager::initialize();

    std::string folder = useTestDirectories ? "test/" : "";
    std::string inputPath = "--input-path=" + folder + "data";
    std::string outputPath = "--output-path=" + folder + "logs";

    argv.at(3) = path;
    argv.at(4) = inputPath.c_str();
    argv.at(5) = outputPath.c_str();

    int executionFailure = NAV::AppLogic::processCommandLineArguments(static_cast<int>(argv.size() - 1), argv.data());

    nm::ClearRegisteredCallbacks();

    NAV::ConfigManager::deinitialize();

    return !static_cast<bool>(executionFailure);
}

void NAV::TESTS::runGeneralFlowCleanupChecks()
{
    for (const NAV::Node* node : nm::m_Nodes())
    {
        for (const auto& inputPin : node->inputPins)
        {
            REQUIRE(inputPin.queue.empty()); // No data left in the flow
        }
    }
}