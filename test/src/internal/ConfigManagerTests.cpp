#include <catch2/catch.hpp>
#include <fmt/core.h>
#include <filesystem>

#include "internal/ConfigManager.hpp"
#include "internal/FlowManager.hpp"

namespace NAV::TESTS
{

TEST_CASE("[ConfigManager] Fetch configs (long options)", "[ConfigManager]")
{
    NAV::ConfigManager::initialize();

    std::vector<const char*> argv = { "",
                                      "--config=BadConfig.ini",
                                      "--config=BadConfig2.ini",
                                      "--sigterm",
                                      "--duration=60",
                                      "--nogui",
                                      "--load=flow/Default.flow",
                                      "--output-path=test/logs/ConfigManager",
                                      "--input-path=test/data",
                                      "--flow-path=test/flow",
                                      nullptr };

    NAV::flow::SetProgramRootPath(std::filesystem::current_path());
    auto failedConfigFiles = NAV::ConfigManager::FetchConfigs(static_cast<int>(argv.size() - 1), argv.data());
    NAV::flow::SetOutputPath();

    REQUIRE(failedConfigFiles.size() == 2);
    REQUIRE(failedConfigFiles.front() == "BadConfig.ini");
    REQUIRE(failedConfigFiles.back() == "BadConfig2.ini");

    REQUIRE(ConfigManager::Get<bool>("sigterm", false) == true);
    REQUIRE(ConfigManager::Get<size_t>("duration", 0) == 60);
    REQUIRE(ConfigManager::Get<bool>("nogui", false) == true);
    REQUIRE(ConfigManager::Get<std::string>("load", "") == "flow/Default.flow");

    REQUIRE(ConfigManager::Get<std::string>("output-path", "") == "test/logs/ConfigManager");
    REQUIRE(NAV::flow::GetOutputPath() == std::filesystem::path{ NAV::flow::GetProgramRootPath() / "test" / "logs" / "ConfigManager" });

    REQUIRE(ConfigManager::Get<std::string>("input-path", "") == "test/data");
    REQUIRE(NAV::flow::GetInputPath() == std::filesystem::path{ NAV::flow::GetProgramRootPath() / "test" / "data" });

    REQUIRE(ConfigManager::Get<std::string>("flow-path", "") == "test/flow");
    REQUIRE(NAV::flow::GetFlowPath() == std::filesystem::path{ NAV::flow::GetProgramRootPath() / "test" / "flow" });

    NAV::ConfigManager::deinitialize();
}

TEST_CASE("[ConfigManager] Fetch configs (rotate outputs)", "[ConfigManager][Debug]")
{
    NAV::ConfigManager::initialize();

    std::vector<const char*> argv = { "",
                                      "--rotate-output",
                                      "--output-path=test/logs/ConfigManager",
                                      nullptr };

    NAV::flow::SetProgramRootPath(std::filesystem::current_path());
    NAV::ConfigManager::FetchConfigs(static_cast<int>(argv.size() - 1), argv.data());

    REQUIRE(ConfigManager::Get<std::string>("output-path", "") == "test/logs/ConfigManager");

    std::filesystem::remove_all(NAV::flow::GetProgramRootPath() / "test" / "logs" / "ConfigManager");

    for (size_t i = 0; i < 10; ++i)
    {
        NAV::flow::SetOutputPath();
        REQUIRE(NAV::flow::GetOutputPath() == std::filesystem::path{ NAV::flow::GetProgramRootPath() / "test" / "logs" / "ConfigManager" / fmt::format("{:04d}", i) });
        std::filesystem::create_directories(NAV::flow::GetOutputPath());
    }

    NAV::ConfigManager::deinitialize();
}

TEST_CASE("[ConfigManager] Fetch configs (short options)", "[ConfigManager]")
{
    NAV::ConfigManager::initialize();

    std::vector<const char*> argv = { "",
                                      "-l", "flow/Default.flow",
                                      "-o", "test/logs/ConfigManager",
                                      "-i", "test/data",
                                      "-f", "test/flow",
                                      nullptr };

    NAV::flow::SetProgramRootPath(std::filesystem::current_path());
    auto failedConfigFiles = NAV::ConfigManager::FetchConfigs(static_cast<int>(argv.size() - 1), argv.data());
    NAV::flow::SetOutputPath();

    REQUIRE(failedConfigFiles.empty());
    REQUIRE(ConfigManager::Get<std::string>("load", "") == "flow/Default.flow");

    REQUIRE(ConfigManager::Get<std::string>("output-path", "") == "test/logs/ConfigManager");
    REQUIRE(NAV::flow::GetOutputPath() == std::filesystem::path{ NAV::flow::GetProgramRootPath() / "test" / "logs" / "ConfigManager" });

    REQUIRE(ConfigManager::Get<std::string>("input-path", "") == "test/data");
    REQUIRE(NAV::flow::GetInputPath() == std::filesystem::path{ NAV::flow::GetProgramRootPath() / "test" / "data" });

    REQUIRE(ConfigManager::Get<std::string>("flow-path", "") == "test/flow");
    REQUIRE(NAV::flow::GetFlowPath() == std::filesystem::path{ NAV::flow::GetProgramRootPath() / "test" / "flow" });

    NAV::ConfigManager::deinitialize();
}

} // namespace NAV::TESTS
