#include <catch2/catch.hpp>

#include "util/Logger.hpp"

TEST_CASE("[Logger] Initialization", "[Logger]")
{
    Logger consoleSink;

    Logger consoleAndFileSink("/tmp/LoggerTest.log");
}
