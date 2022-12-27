// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <catch2/catch_test_macros.hpp>

#include "Logger.hpp"
#include "util/Container/ScrollingBuffer.hpp"
#include <iostream>
#include <sstream>

namespace NAV::TESTS
{
constexpr size_t PADDING = 2;
std::stringstream sstream;

TEST_CASE("[ScrollingBuffer] InitializerList", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<int> buffer1({});
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str().empty());
    std::cout << "Empty Buffer  : " << buffer1 << '\n';

    REQUIRE(buffer1.empty());
    REQUIRE(buffer1.capacity() == 0);
    REQUIRE(buffer1.offset() == 0);

    buffer1 = ScrollingBuffer<int>{ 0, 1, 2, 3, 4 };
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "0, 1, 2, 3, 4");
    std::cout << "Filled Buffer : " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(!buffer1.empty());
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 4);
    REQUIRE(buffer1.at(0) == 0);
    REQUIRE(buffer1.at(1) == 1);
    REQUIRE(buffer1.at(2) == 2);
    REQUIRE(buffer1.at(3) == 3);
    REQUIRE(buffer1.at(4) == 4);
}

TEST_CASE("[ScrollingBuffer] push_back", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<int> buffer1(5);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "_, _, _, _, _");
    std::cout << "Empty Buffer  : " << buffer1 << '\n';
    REQUIRE(buffer1.empty());
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == 0);

    for (int i = 0; i < 4; i++)
    {
        REQUIRE(buffer1.size() == static_cast<size_t>(i));
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "0, 1, 2, 3, _");
    std::cout << "Filled Buffer : " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 4);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(!buffer1.empty());
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 3);

    buffer1.push_back(4);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "0, 1, 2, 3, 4");
    std::cout << "Filled Buffer : " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(!buffer1.empty());
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 4);

    for (int i = 5; i < 7; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, 2, 3, 4");
    std::cout << "Rotated Buffer: " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == 2);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 6);
}

TEST_CASE("[ScrollingBuffer] push_back (padding)", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<int, PADDING> buffer1(5);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, _, _, _, _, _");
    std::cout << "Empty Buffer  : " << buffer1 << '\n';
    REQUIRE(buffer1.empty());
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == PADDING);

    for (int i = 0; i < 4; i++)
    {
        REQUIRE(buffer1.size() == static_cast<size_t>(i));
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 0, 1, 2, 3, _");
    std::cout << "Filled Buffer : " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 4);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(!buffer1.empty());
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 3);

    buffer1.push_back(4);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 0, 1, 2, 3, 4");
    std::cout << "Filled Buffer : " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(!buffer1.empty());
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 4);

    for (int i = 5; i < 7; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, X, X, 2, 3, 4");
    std::cout << "Rotated Buffer: " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == PADDING + 2);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 6);
}

TEST_CASE("[ScrollingBuffer<double>] All Functions", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<double> buffer1(5);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "_, _, _, _, _");
    std::cout << "Empty Buffer  : " << buffer1 << '\n';
    REQUIRE(buffer1.empty());
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.empty());

    for (int i = 0; i < 5; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "0.000000, 1.000000, 2.000000, 3.000000, 4.000000");
    std::cout << "Filled Buffer : " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(!buffer1.empty());
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 4);
    REQUIRE(buffer1.at(0) == 0);
    REQUIRE(buffer1.at(1) == 1);
    REQUIRE(buffer1.at(2) == 2);
    REQUIRE(buffer1.at(3) == 3);
    REQUIRE(buffer1.at(4) == 4);
    REQUIRE_THROWS_AS(buffer1.at(5), std::out_of_range);

    for (int i = 5; i < 7; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5.000000, 6.000000, 2.000000, 3.000000, 4.000000");
    std::cout << "Rotated Buffer: " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == 2);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 6);
    REQUIRE(buffer1.at(0) == 2);
    REQUIRE(buffer1.at(1) == 3);
    REQUIRE(buffer1.at(2) == 4);
    REQUIRE(buffer1.at(3) == 5);
    REQUIRE(buffer1.at(4) == 6);
    REQUIRE_THROWS_AS(buffer1.at(5), std::out_of_range);

    buffer1.resize(3);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5.000000, 6.000000, 4.000000");
    std::cout << "Shrink     (3): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 3);
    REQUIRE(buffer1.capacity() == 3);
    REQUIRE(buffer1.offset() == 2);
    REQUIRE(buffer1.front() == 4);
    REQUIRE(buffer1.back() == 6);
    REQUIRE(buffer1.at(0) == 4);
    REQUIRE(buffer1.at(1) == 5);
    REQUIRE(buffer1.at(2) == 6);
    REQUIRE_THROWS_AS(buffer1.at(3), std::out_of_range);

    const double* raw = buffer1.data();
    REQUIRE(raw[0] == 5);
    REQUIRE(raw[1] == 6);
    REQUIRE(raw[2] == 4);

    buffer1.clear();
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "_, _, _");
    std::cout << "Clear      (3): " << buffer1 << '\n';
    REQUIRE(buffer1.empty());
    REQUIRE(buffer1.capacity() == 3);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE_THROWS_AS(buffer1.at(0), std::out_of_range);
}

TEST_CASE("[ScrollingBuffer<double>] All Functions (padding)", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<double, PADDING> buffer1(5);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, _, _, _, _, _");
    std::cout << "Empty Buffer  : " << buffer1 << '\n';
    REQUIRE(buffer1.empty());
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(buffer1.empty());

    for (int i = 0; i < 5; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 0.000000, 1.000000, 2.000000, 3.000000, 4.000000");
    std::cout << "Filled Buffer : " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(!buffer1.empty());
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 4);
    REQUIRE(buffer1.at(0) == 0);
    REQUIRE(buffer1.at(1) == 1);
    REQUIRE(buffer1.at(2) == 2);
    REQUIRE(buffer1.at(3) == 3);
    REQUIRE(buffer1.at(4) == 4);
    REQUIRE_THROWS_AS(buffer1.at(5), std::out_of_range);

    for (int i = 5; i < 7; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5.000000, 6.000000, X, X, 2.000000, 3.000000, 4.000000");
    std::cout << "Rotated Buffer: " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == PADDING + 2);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 6);
    REQUIRE(buffer1.at(0) == 2);
    REQUIRE(buffer1.at(1) == 3);
    REQUIRE(buffer1.at(2) == 4);
    REQUIRE(buffer1.at(3) == 5);
    REQUIRE(buffer1.at(4) == 6);
    REQUIRE_THROWS_AS(buffer1.at(5), std::out_of_range);

    buffer1.resize(3);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5.000000, 6.000000, X, X, 4.000000");
    std::cout << "Shrink     (3): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 3);
    REQUIRE(buffer1.capacity() == 3);
    REQUIRE(buffer1.offset() == PADDING + 2);
    REQUIRE(buffer1.front() == 4);
    REQUIRE(buffer1.back() == 6);
    REQUIRE(buffer1.at(0) == 4);
    REQUIRE(buffer1.at(1) == 5);
    REQUIRE(buffer1.at(2) == 6);
    REQUIRE_THROWS_AS(buffer1.at(3), std::out_of_range);

    const double* raw = buffer1.data();
    REQUIRE(raw[0] == 5.0);
    REQUIRE(raw[1] == 6.0);
    REQUIRE(raw[2] == 2.0);
    REQUIRE(raw[3] == 3.0);
    REQUIRE(raw[4] == 4.0);

    buffer1.clear();
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, _, _, _");
    std::cout << "Clear      (3): " << buffer1 << '\n';
    REQUIRE(buffer1.empty());
    REQUIRE(buffer1.capacity() == 3);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE_THROWS_AS(buffer1.at(0), std::out_of_range);
}

TEST_CASE("[ScrollingBuffer] Shrink unscrolled buffer", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<int> buffer1(6);
    for (int i = 0; i < 4; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "0, 1, 2, 3, _, _");
    std::cout << "Buffer (6): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 4);
    REQUIRE(buffer1.capacity() == 6);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 3);

    buffer1.resize(5);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "0, 1, 2, 3, _");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 4);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 3);

    buffer1.resize(4);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "0, 1, 2, 3");
    std::cout << "Shrink (4): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 4);
    REQUIRE(buffer1.capacity() == 4);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 3);

    buffer1.resize(2);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "2, 3");
    std::cout << "Shrink (2): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 2);
    REQUIRE(buffer1.capacity() == 2);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 3);

    buffer1.push_back(4);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "4, 3");
    std::cout << "Add Value : " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 2);
    REQUIRE(buffer1.capacity() == 2);
    REQUIRE(buffer1.offset() == 1);
    REQUIRE(buffer1.front() == 3);
    REQUIRE(buffer1.back() == 4);

    buffer1 = ScrollingBuffer<int>(5);
    for (int i = 0; i < 4; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "0, 1, 2, 3, _");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.capacity() == 5);

    buffer1.resize(3);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "1, 2, 3");
    std::cout << "Shrink (3): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 3);
    REQUIRE(buffer1.capacity() == 3);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 1);
    REQUIRE(buffer1.back() == 3);
}

TEST_CASE("[ScrollingBuffer] Shrink unscrolled buffer (padding)", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<int, PADDING> buffer1(6);
    for (int i = 0; i < 4; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 0, 1, 2, 3, _, _");
    std::cout << "Buffer (6): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 4);
    REQUIRE(buffer1.capacity() == 6);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 3);

    buffer1.resize(5);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 0, 1, 2, 3, _");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 4);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 3);

    buffer1.resize(4);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 0, 1, 2, 3");
    std::cout << "Shrink (4): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 4);
    REQUIRE(buffer1.capacity() == 4);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 3);

    buffer1.resize(2);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 2, 3");
    std::cout << "Shrink (2): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 2);
    REQUIRE(buffer1.capacity() == 2);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 3);

    buffer1.push_back(4);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "4, X, X, 3");
    std::cout << "Add Value : " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 2);
    REQUIRE(buffer1.capacity() == 2);
    REQUIRE(buffer1.offset() == PADDING + 1);
    REQUIRE(buffer1.front() == 3);
    REQUIRE(buffer1.back() == 4);

    buffer1 = ScrollingBuffer<int, PADDING>(5);
    for (int i = 0; i < 4; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 0, 1, 2, 3, _");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.capacity() == 5);

    buffer1.resize(3);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 1, 2, 3");
    std::cout << "Shrink (3): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 3);
    REQUIRE(buffer1.capacity() == 3);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(buffer1.front() == 1);
    REQUIRE(buffer1.back() == 3);
}

TEST_CASE("[ScrollingBuffer] Grow unscrolled buffer", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<int> buffer1(5);
    for (int i = 0; i < 4; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "0, 1, 2, 3, _");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 4);
    REQUIRE(buffer1.capacity() == 5);

    buffer1.resize(7);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "0, 1, 2, 3, _, _, _");
    std::cout << "Grow   (7): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 4);
    REQUIRE(buffer1.capacity() == 7);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 3);
}

TEST_CASE("[ScrollingBuffer] Grow unscrolled buffer (padding)", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<int, PADDING> buffer1(5);
    for (int i = 0; i < 4; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 0, 1, 2, 3, _");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 4);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 3);

    buffer1.resize(7);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 0, 1, 2, 3, _, _, _");
    std::cout << "Grow   (7): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 4);
    REQUIRE(buffer1.capacity() == 7);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 3);
}

TEST_CASE("[ScrollingBuffer] Shrink scrolled buffer", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<int> buffer1(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, 2, 3, 4");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);

    buffer1.resize(3);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, 4");
    std::cout << "Shrink (3): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 3);
    REQUIRE(buffer1.capacity() == 3);
    REQUIRE(buffer1.offset() == 2);
    REQUIRE(buffer1.front() == 4);
    REQUIRE(buffer1.back() == 6);

    buffer1.resize(1);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "6");
    std::cout << "Shrink (1): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 1);
    REQUIRE(buffer1.capacity() == 1);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 6);
    REQUIRE(buffer1.back() == 6);
}

TEST_CASE("[ScrollingBuffer] Shrink scrolled buffer (padding)", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<int, PADDING> buffer1(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, X, X, 2, 3, 4");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);

    buffer1.resize(3);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, X, X, 4");
    std::cout << "Shrink (3): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 3);
    REQUIRE(buffer1.capacity() == 3);
    REQUIRE(buffer1.offset() == PADDING + 2);
    REQUIRE(buffer1.front() == 4);
    REQUIRE(buffer1.back() == 6);

    buffer1.resize(1);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "6, X, X");
    std::cout << "Shrink (1): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 1);
    REQUIRE(buffer1.capacity() == 1);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 6);
    REQUIRE(buffer1.back() == 6);

    buffer1.push_back(7);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, 7, X");
    std::cout << "Shrink (1): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 1);
    REQUIRE(buffer1.capacity() == 1);
    REQUIRE(buffer1.offset() == 1);
    REQUIRE(buffer1.front() == 7);
    REQUIRE(buffer1.back() == 7);

    buffer1 = ScrollingBuffer<int, PADDING>(5);
    for (int i = 0; i < 11; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, 6, 7, 8, 9, 10, X");
    std::cout << "Buffer (5): " << buffer1 << '\n';

    buffer1.resize(4);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 7, 8, 9, 10");
    std::cout << "Shrink (2): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 4);
    REQUIRE(buffer1.capacity() == 4);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(buffer1.front() == 7);
    REQUIRE(buffer1.back() == 10);
    const int* raw = buffer1.data();
    REQUIRE(raw[0] == 5);
    REQUIRE(raw[1] == 6);
    REQUIRE(raw[2] == 7);
    REQUIRE(raw[3] == 8);
    REQUIRE(raw[4] == 9);
    REQUIRE(raw[5] == 10);

    buffer1 = ScrollingBuffer<int, PADDING>(5);
    for (int i = 0; i < 11; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, 6, 7, 8, 9, 10, X");
    std::cout << "Buffer (5): " << buffer1 << '\n';

    buffer1.resize(2);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 9, 10");
    std::cout << "Shrink (2): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 2);
    REQUIRE(buffer1.capacity() == 2);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(buffer1.front() == 9);
    REQUIRE(buffer1.back() == 10);
    raw = buffer1.data();
    REQUIRE(raw[0] == 7);
    REQUIRE(raw[1] == 8);
    REQUIRE(raw[2] == 9);
    REQUIRE(raw[3] == 10);

    ScrollingBuffer<int, 7> buffer2(5);
    for (int i = 0; i < 11; i++)
    {
        buffer2.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer2;
    REQUIRE(sstream.str() == "X, 6, 7, 8, 9, 10, X, X, X, X, X, X");
    std::cout << "Buffer (5): " << buffer2 << '\n';

    buffer2.resize(2);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer2;
    REQUIRE(sstream.str() == "X, X, X, X, 9, 10, X, X, X");
    std::cout << "Shrink (2): " << buffer2 << '\n';
    REQUIRE(buffer2.size() == 2);
    REQUIRE(buffer2.capacity() == 2);
    REQUIRE(buffer2.offset() == 4);
    REQUIRE(buffer2.front() == 9);
    REQUIRE(buffer2.back() == 10);
    raw = buffer2.data();
    REQUIRE(raw[0] == 5);
    REQUIRE(raw[1] == 6);
    REQUIRE(raw[2] == 7);
    REQUIRE(raw[3] == 8);
    REQUIRE(raw[4] == 9);
    REQUIRE(raw[5] == 10);
    REQUIRE(raw[6] == 2);
    REQUIRE(raw[7] == 3);
    REQUIRE(raw[8] == 4);
}

TEST_CASE("[ScrollingBuffer] Grow scrolled buffer", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<int> buffer1(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, 2, 3, 4");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);

    buffer1.resize(7);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, _, _, 2, 3, 4");
    std::cout << "Grow   (7): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 7);
    REQUIRE(buffer1.offset() == 4);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 6);

    buffer1.push_back(7);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, 7, _, 2, 3, 4");
    std::cout << "Grow   (7): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 6);
    REQUIRE(buffer1.capacity() == 7);
    REQUIRE(buffer1.offset() == 4);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 7);

    buffer1.resize(1);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "7");
    std::cout << "Shrink (1): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 1);
    REQUIRE(buffer1.capacity() == 1);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 7);
    REQUIRE(buffer1.back() == 7);

    buffer1.push_back(8);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "8");
    std::cout << "Shrink (1): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 1);
    REQUIRE(buffer1.capacity() == 1);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 8);
    REQUIRE(buffer1.back() == 8);
}

TEST_CASE("[ScrollingBuffer] Grow scrolled buffer (padding)", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<int, PADDING> buffer1(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, X, X, 2, 3, 4");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == PADDING + 2);

    buffer1.resize(7);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, _, _, X, X, 2, 3, 4");
    std::cout << "Grow   (7): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 7);
    REQUIRE(buffer1.offset() == PADDING + 4);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 6);

    buffer1.push_back(7);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, 7, _, X, X, 2, 3, 4");
    std::cout << "Grow   (7): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 6);
    REQUIRE(buffer1.capacity() == 7);
    REQUIRE(buffer1.offset() == PADDING + 4);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 7);

    buffer1.resize(1);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "7, X, X");
    std::cout << "Shrink (1): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 1);
    REQUIRE(buffer1.capacity() == 1);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 7);
    REQUIRE(buffer1.back() == 7);

    buffer1.push_back(8);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, 8, X");
    std::cout << "Shrink (1): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 1);
    REQUIRE(buffer1.capacity() == 1);
    REQUIRE(buffer1.offset() == 1);
    REQUIRE(buffer1.front() == 8);
    REQUIRE(buffer1.back() == 8);

    buffer1 = ScrollingBuffer<int, PADDING>(5);
    for (int i = 0; i < 11; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, 6, 7, 8, 9, 10, X");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == 1);
    REQUIRE(buffer1.front() == 6);
    REQUIRE(buffer1.back() == 10);

    buffer1.resize(7);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, 6, 7, 8, 9, 10, _, _, X");
    std::cout << "Grow   (7): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 7);
    REQUIRE(buffer1.offset() == 1);
    REQUIRE(buffer1.front() == 6);
    REQUIRE(buffer1.back() == 10);
}

TEST_CASE("[ScrollingBuffer] Raw data", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<int> buffer1(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, 2, 3, 4");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);

    const int* raw = buffer1.data();
    REQUIRE(raw[0] == 5);
    REQUIRE(raw[1] == 6);
    REQUIRE(raw[2] == 2);
    REQUIRE(raw[3] == 3);
    REQUIRE(raw[4] == 4);
}

TEST_CASE("[ScrollingBuffer] Raw data (padding)", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<int, PADDING> buffer1(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, X, X, 2, 3, 4");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);

    const int* raw = buffer1.data();
    REQUIRE(raw[0] == 5);
    REQUIRE(raw[1] == 6);
    REQUIRE(raw[2] == 0);
    REQUIRE(raw[3] == 1);
    REQUIRE(raw[4] == 2);
    REQUIRE(raw[5] == 3);
    REQUIRE(raw[6] == 4);
}

TEST_CASE("[ScrollingBuffer] Infinite buffer", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<int> buffer1(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, 2, 3, 4");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);

    buffer1.resize(0);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "2, 3, 4, 5, 6");
    std::cout << "Infi   (0): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 0);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 6);

    for (int i = 7; i < 10; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "2, 3, 4, 5, 6, 7, 8, 9");
    std::cout << "Add Value : " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 8);
    REQUIRE(buffer1.capacity() == 0);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 9);

    buffer1 = ScrollingBuffer<int>(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, 2, 3, 4");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);

    buffer1.resize(7);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, _, _, 2, 3, 4");
    std::cout << "Grow   (7): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 7);

    buffer1.resize(0);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "2, 3, 4, 5, 6");
    std::cout << "Infi   (0): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 0);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 6);

    buffer1 = ScrollingBuffer<int>(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.push_back(i * (i % 2 ? 1 : -1));
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, -6, -2, 3, -4");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == 2);
    REQUIRE(buffer1.front() == -2);
    REQUIRE(buffer1.back() == -6);

    buffer1.resize(0);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "-2, 3, -4, 5, -6");
    std::cout << "Infi   (0): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 0);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == -2);
    REQUIRE(buffer1.back() == -6);

    buffer1 = ScrollingBuffer<int>(0);
    for (int i = 0; i < 8; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "0, 1, 2, 3, 4, 5, 6, 7");
    std::cout << "Buffer (0): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 8);
    REQUIRE(buffer1.capacity() == 0);
    REQUIRE(buffer1.offset() == 0);
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 7);
}

TEST_CASE("[ScrollingBuffer] Infinite buffer (padding)", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<int, PADDING> buffer1(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, X, X, 2, 3, 4");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);

    buffer1.resize(0);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 2, 3, 4, 5, 6");
    std::cout << "Infi   (0): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 0);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 6);

    for (int i = 7; i < 10; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 2, 3, 4, 5, 6, 7, 8, 9");
    std::cout << "Add Value : " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 8);
    REQUIRE(buffer1.capacity() == 0);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 9);

    buffer1 = ScrollingBuffer<int, PADDING>(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, X, X, 2, 3, 4");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);

    buffer1.resize(7);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, _, _, X, X, 2, 3, 4");
    std::cout << "Grow   (7): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 7);

    buffer1.resize(0);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 2, 3, 4, 5, 6");
    std::cout << "Infi   (0): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 0);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(buffer1.front() == 2);
    REQUIRE(buffer1.back() == 6);

    buffer1 = ScrollingBuffer<int, PADDING>(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.push_back(i * (i % 2 ? 1 : -1));
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, -6, X, X, -2, 3, -4");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == PADDING + 2);
    REQUIRE(buffer1.front() == -2);
    REQUIRE(buffer1.back() == -6);

    buffer1.resize(0);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, -2, 3, -4, 5, -6");
    std::cout << "Infi   (0): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 0);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(buffer1.front() == -2);
    REQUIRE(buffer1.back() == -6);

    buffer1 = ScrollingBuffer<int, PADDING>(0);
    for (int i = 0; i < 8; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 0, 1, 2, 3, 4, 5, 6, 7");
    std::cout << "Buffer (0): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 8);
    REQUIRE(buffer1.capacity() == 0);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 7);

    buffer1 = ScrollingBuffer<int, PADDING>(5);
    for (int i = 0; i < 11; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, 6, 7, 8, 9, 10, X");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);
    REQUIRE(buffer1.offset() == 1);
    REQUIRE(buffer1.front() == 6);
    REQUIRE(buffer1.back() == 10);

    buffer1.resize(0);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 6, 7, 8, 9, 10");
    std::cout << "Infi   (0): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 0);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(buffer1.front() == 6);
    REQUIRE(buffer1.back() == 10);

    buffer1 = ScrollingBuffer<int, PADDING>(5);
    for (int i = 0; i < 5; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 0, 1, 2, 3, 4");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);

    buffer1.resize(0);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, 0, 1, 2, 3, 4");
    std::cout << "Infi   (0): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 0);
    REQUIRE(buffer1.offset() == PADDING);
    REQUIRE(buffer1.front() == 0);
    REQUIRE(buffer1.back() == 4);
}

TEST_CASE("[ScrollingBuffer] Clear", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<int> buffer1(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, 2, 3, 4");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);

    buffer1.resize(7);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, _, _, 2, 3, 4");
    std::cout << "Grow   (7): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 7);

    buffer1.clear();
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "_, _, _, _, _, _, _");
    std::cout << "Clear  (7): " << buffer1 << '\n';
    REQUIRE(buffer1.capacity() == 7);
    REQUIRE(buffer1.empty());
    REQUIRE(buffer1.offset() == 0);
}

TEST_CASE("[ScrollingBuffer] Clear (padding)", "[ScrollingBuffer]")
{
    auto logger = initializeTestLogger();

    ScrollingBuffer<int, PADDING> buffer1(5);
    for (int i = 0; i < 7; i++)
    {
        buffer1.push_back(i);
    }
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, X, X, 2, 3, 4");
    std::cout << "Buffer (5): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 5);

    buffer1.resize(7);
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "5, 6, _, _, X, X, 2, 3, 4");
    std::cout << "Grow   (7): " << buffer1 << '\n';
    REQUIRE(buffer1.size() == 5);
    REQUIRE(buffer1.capacity() == 7);

    buffer1.clear();
    sstream.clear();
    sstream.str(std::string());
    sstream << buffer1;
    REQUIRE(sstream.str() == "X, X, _, _, _, _, _, _, _");
    std::cout << "Clear  (7): " << buffer1 << '\n';
    REQUIRE(buffer1.capacity() == 7);
    REQUIRE(buffer1.empty());
    REQUIRE(buffer1.offset() == PADDING);
}

} // namespace NAV::TESTS