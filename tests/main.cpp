#include <gtest/gtest.h>

#include "util/Logger.hpp"

// #include "util/Math/VectorTest.hpp"
// #include "util/Math/MatrixTest.hpp"
// #include "util/Math/QuaternionTest.hpp"

#include "DataLogger/VectorNavDataLoggerTest.hpp"

int main(int argc, char* argv[])
{
    if (NAV::Logger::initialize("logs/navsos-test.log"))
        return EXIT_FAILURE;

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
