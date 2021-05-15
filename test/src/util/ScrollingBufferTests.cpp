#include <catch2/catch.hpp>

#include "util/ScrollingBuffer.hpp"
#include <iostream>

namespace NAV
{
TEST_CASE("[ScrollingBuffer] AddValue", "[ScrollingBuffer]")
{
    ScrollingBuffer<int> buffer1(5);
    std::cout << "Empty Buffer  : " << buffer1; // _, _, _, _, _,
    REQUIRE(buffer1.empty());
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.empty());
    for (int i = 0; i < 5; i++)
    {
        buffer1.AddValue(i);
    }
    std::cout << "Filled Buffer : " << buffer1; // 0, 1, 2, 3, 4,
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(!buffer1.empty());
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 4);
    for (int i = 5; i < 7; i++)
    {
        buffer1.AddValue(i);
    }
    std::cout << "Rotated Buffer: " << buffer1; // 5, 6, 2, 3, 4,
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.offset() == 2);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 6);
}

TEST_CASE("[ScrollingBuffer<double>] All Functions", "[ScrollingBuffer]")
{
    ScrollingBuffer<double> buffer1(5);
    std::cout << "Empty Buffer  : " << buffer1; // _, _, _, _, _,
    REQUIRE(buffer1.empty());
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.empty());
    for (int i = 0; i < 5; i++)
    {
        buffer1.AddValue(i);
    }
    std::cout << "Filled Buffer : " << buffer1; // 0, 1, 2, 3, 4,
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(!buffer1.empty());
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 4);
    for (int i = 5; i < 7; i++)
    {
        buffer1.AddValue(i);
    }
    std::cout << "Rotated Buffer: " << buffer1; // 5, 6, 2, 3, 4,
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.offset() == 2);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 6);

    buffer1.resize(3);
    std::cout << "Shrink     (3): " << buffer1; // 5, 6, 4,
    REQUIRE(buffer1.size() == 3);
    REQUIRE(buffer1.offset() == 2);
    REQUIRE(buffer1.front() == 4);
    REQUIRE(buffer1.back() == 6);

    const double* raw = buffer1.data();
    REQUIRE(raw[0] == 5);
    REQUIRE(raw[1] == 6);
    REQUIRE(raw[2] == 4);

    buffer1.clear();
    std::cout << "Clear      (3): " << buffer1; // _, _, _, _, _,
    REQUIRE(buffer1.empty());
    REQUIRE(buffer1.offset() == 0);
}

TEST_CASE("[ScrollingBuffer] Shrink unscrolled buffer", "[ScrollingBuffer]")
{
    ScrollingBuffer<int> buffer1(6);
    for (int i = 0; i < 4; i++)
    {
        buffer1.AddValue(i);
    }
    std::cout << "Buffer (6): " << buffer1; // 0, 1, 2, 3, _, _,
    REQUIRE(buffer1.size() == 4);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 3);

    buffer1.resize(5);
    std::cout << "Buffer (5): " << buffer1; // 0, 1, 2, 3, _,
    REQUIRE(buffer1.size() == 4);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 3);

    buffer1.resize(4);
    std::cout << "Shrink (4): " << buffer1; // 0, 1, 2, 3,
    REQUIRE(buffer1.size() == 4);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 3);

    buffer1.resize(2);
    std::cout << "Shrink (2): " << buffer1; // 2, 3,
    REQUIRE(buffer1.size() == 2);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 3);

    buffer1.AddValue(4);
    std::cout << "Add Value : " << buffer1; // 4, 3,
    REQUIRE(buffer1.size() == 2);
    REQUIRE(buffer1.offset() == 1);
    REQUIRE(buffer1.front() == 3);
    REQUIRE(buffer1.back() == 4);

    buffer1 = ScrollingBuffer<int>(5);
    for (int i = 0; i < 4; i++)
    {
        buffer1.AddValue(i);
    }
    std::cout << "Buffer (5): " << buffer1; // 0, 1, 2, 3, _,

    buffer1.resize(3);
    std::cout << "Shrink (3): " << buffer1; // 1, 2, 3,
    REQUIRE(buffer1.size() == 3);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 1);
    REQUIRE(buffer1.back() == 3);
}

TEST_CASE("[ScrollingBuffer] Grow unscrolled buffer", "[ScrollingBuffer]")
{
    ScrollingBuffer<int> buffer1(5);
    for (int i = 0; i < 4; i++)
    {
        buffer1.AddValue(i);
    }
    std::cout << "Buffer (5): " << buffer1; // 0, 1, 2, 3, _,

    buffer1.resize(7);
    std::cout << "Grow   (7): " << buffer1; // 0, 1, 2, 3, _, _, _,
    REQUIRE(buffer1.size() == 4);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 3);
}

TEST_CASE("[ScrollingBuffer] Shrink scrolled buffer", "[ScrollingBuffer]")
{
    ScrollingBuffer<int> buffer1(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.AddValue(i);
    }
    std::cout << "Buffer (5): " << buffer1; // 5, 6, 2, 3, 4,

    buffer1.resize(3);
    std::cout << "Shrink (3): " << buffer1; // 5, 6, 4,
    REQUIRE(buffer1.size() == 3);
    REQUIRE(buffer1.offset() == 2);
    REQUIRE(buffer1.front() == 4);
    REQUIRE(buffer1.back() == 6);

    buffer1.resize(1);
    std::cout << "Shrink (1): " << buffer1; // 6,
    REQUIRE(buffer1.size() == 1);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 6);
    REQUIRE(buffer1.back() == 6);
}

TEST_CASE("[ScrollingBuffer] Grow scrolled buffer", "[ScrollingBuffer]")
{
    ScrollingBuffer<int> buffer1(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.AddValue(i);
    }
    std::cout << "Buffer (5): " << buffer1; // 5, 6, 2, 3, 4,

    buffer1.resize(7);
    std::cout << "Grow   (7): " << buffer1; // 5, 6, _, _, 2, 3, 4,
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.offset() == 4);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 6);

    buffer1.resize(1);
    std::cout << "Shrink (1): " << buffer1; // 6,
    REQUIRE(buffer1.size() == 1);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 6);
    REQUIRE(buffer1.back() == 6);
}

TEST_CASE("[ScrollingBuffer] Raw data", "[ScrollingBuffer]")
{
    ScrollingBuffer<int> buffer1(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.AddValue(i);
    }
    std::cout << "Buffer (5): " << buffer1; // 5, 6, 2, 3, 4,

    const int* raw = buffer1.data();
    REQUIRE(raw[0] == 5);
    REQUIRE(raw[1] == 6);
    REQUIRE(raw[2] == 2);
    REQUIRE(raw[3] == 3);
    REQUIRE(raw[4] == 4);
}

TEST_CASE("[ScrollingBuffer] Infinite buffer", "[ScrollingBuffer]")
{
    ScrollingBuffer<int> buffer1(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.AddValue(i);
    }
    std::cout << "Buffer (5): " << buffer1; // 5, 6, 2, 3, 4,

    buffer1.resize(0);
    std::cout << "Infi   (0): " << buffer1; // 2, 3, 4, 5, 6,
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 6);

    for (int i = 7; i < 10; i++)
    {
        buffer1.AddValue(i);
    }
    std::cout << "Add Value : " << buffer1; // 2, 3, 4, 5, 6, 7, 8, 9,
    REQUIRE(buffer1.size() == 8);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 9);

    buffer1 = ScrollingBuffer<int>(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.AddValue(i);
    }
    std::cout << "Buffer (5): " << buffer1; // 5, 6, 2, 3, 4,

    buffer1.resize(7);
    std::cout << "Grow   (7): " << buffer1; // 5, 6, _, _, 2, 3, 4,

    buffer1.resize(0);
    std::cout << "Infi   (0): " << buffer1; // 2, 3, 4, 5, 6,
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 6);
}

TEST_CASE("[ScrollingBuffer] Clear", "[ScrollingBuffer]")
{
    ScrollingBuffer<int> buffer1(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.AddValue(i);
    }
    std::cout << "Buffer (5): " << buffer1; // 5, 6, 2, 3, 4,

    buffer1.resize(7);
    std::cout << "Grow   (7): " << buffer1; // 5, 6, _, _, 2, 3, 4,

    buffer1.clear();
    std::cout << "Clear  (7): " << buffer1; // _, _, _, _, _, _, _,
    REQUIRE(buffer1.empty());
    REQUIRE(buffer1.offset() == 0);
}

} // namespace NAV