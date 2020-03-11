#include <gtest/gtest.h>

#include "util/Math/VectorTest.hpp"
#include "util/Math/MatrixTest.hpp"
#include "util/Math/QuaternionTest.hpp"

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
