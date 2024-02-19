// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Logger.hpp"

#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include "FlowTester.hpp"

#include "internal/ConfigManager.hpp"
#include "util/StringUtil.hpp"

[[nodiscard]] Logger NAV::TESTS::initializeTestLogger()
{
    NAV::ConfigManager::initialize();
    NAV::ConfigManager::FetchConfigs(static_cast<int>(argv.size() - 1), argv.data());

    std::string testName = Catch::getResultCapture().getCurrentTestName();

    str::replaceAll(testName, " ", "_");
    // Linux
    str::replaceAll(testName, "/", "_"); // / (forward slash)
    // Windows
    str::replaceAll(testName, "<", "_");  // < (less than)
    str::replaceAll(testName, ">", "_");  // > (greater than)
    str::replaceAll(testName, ":", "_");  // : (colon - sometimes works, but is actually NTFS Alternate Data Streams)
    str::replaceAll(testName, "\"", "_"); // " (double quote)
    str::replaceAll(testName, "/", "_");  // / (forward slash)
    str::replaceAll(testName, "\\", "_"); // \ (backslash)
    str::replaceAll(testName, "|", "_");  // | (vertical bar or pipe)
    str::replaceAll(testName, "?", "_");  // ? (question mark)
    str::replaceAll(testName, "*", "_");  // * (asterisk)

    auto logpath = std::filesystem::path("test") / "logs" / (testName + ".log");

    Logger log(logpath.string());

    NAV::ConfigManager::deinitialize();
    return log;
}