// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "FlowTester.hpp"

#include <catch2/catch.hpp>
#include <vector>

#include "internal/Node/Node.hpp"
#include "internal/AppLogic.hpp"
#include "internal/ConfigManager.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

bool NAV::TESTS::testFlow(const char* path)
{
    // Config Manager object
    NAV::ConfigManager::initialize();

    std::vector<const char*> argv = { "",
                                      "--nogui",
                                      "-l", path,
                                      "--input-path=test/data",
                                      "--output-path=test/logs",
                                      "--console-log-level=trace", // trace/debug/info/warning/error/critical/off
                                      "--file-log-level=trace",    // trace/debug/info/warning/error/critical/off
                                      nullptr };

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