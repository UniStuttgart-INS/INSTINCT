#include <catch2/catch.hpp>

#include "util/Logger.hpp"

namespace NAV::TEST
{
TEST_CASE("[Logger] Initialization", "[Logger]")
{
    Logger consoleSink;

    Logger consoleAndFileSink("/tmp/LoggerTest.log");
}

} // namespace NAV::TEST