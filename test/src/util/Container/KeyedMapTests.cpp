// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file KeyedMapTests.cpp
/// @brief UnitTests for the KeyedMap class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-10-08

#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include "CatchMatchers.hpp"

#include "Logger.hpp"
#include <unordered_map>
#include "util/Container/KeyedMap.hpp"
#include <catch2/matchers/catch_matchers.hpp>

namespace NAV::TESTS
{
namespace
{
template<bool unordered>
void runTest()
{
    auto logger = initializeTestLogger();

    std::vector<int> keys = { 1, 2 };
    std::unordered_map<int, double*> memory;

    KeyedMap<int, double, unordered> a;
    a.addKeys(keys);
    REQUIRE(a.size() == keys.size());
    REQUIRE(a.contains(keys));
    for (const auto& key : keys)
    {
        CAPTURE(key);
        REQUIRE(a.at(key) == 0.0);
        REQUIRE(a.contains(key));
        memory[key] = &a.at(key);
    }

    a.at(keys[0]) = 4.0;
    REQUIRE(a.at(keys[0]) == 4.0);
    REQUIRE(a.at(keys[1]) == 0.0);

    {
        int newKey = 4;
        a.addKey(newKey);
        REQUIRE(a.at(keys[0]) == 4.0);
        REQUIRE(a.at(keys[1]) == 0.0);
        REQUIRE(a.at(newKey) == 0.0);
        REQUIRE(memory.at(keys[0]) == &a.at(keys[0]));
        REQUIRE(memory.at(keys[1]) == &a.at(keys[1]));
        memory[newKey] = &a.at(newKey);
    }

    {
        std::vector<std::pair<int, double>> keyValues = { { 3, 2.5 }, { -1, -30.6 } };
        a.addKeys(keyValues);
        for (const auto& [key, value] : keyValues)
        {
            CAPTURE(key);
            REQUIRE(a.at(key) == value);
            memory[key] = &a.at(key);
        }
    }

    {
        std::vector<int> keys = { -5, -3 };
        std::vector<double> values = { 1, 2 };
        a.addKeys(keys, values);
        for (size_t i = 0; i < keys.size(); i++)
        {
            CAPTURE(keys.at(i));
            REQUIRE(a.at(keys.at(i)) == values.at(i));
            memory[keys.at(i)] = &a.at(keys.at(i));
        }
    }

    for (int i = 5; i < 20; i++)
    {
        CAPTURE(i);
        a.addKey(i, i);
        REQUIRE(a.at(i) == i);
        for (const auto& [key, address] : memory)
        {
            REQUIRE(address == &a.at(key));
        }
        memory[i] = &a.at(i);
    }

    a.clear();
    REQUIRE(a.size() == 0);
    REQUIRE(a.empty());
}
} // namespace

TEST_CASE("[KeyedMap] Test all functions (unordered)", "[KeyedMap]")
{
    runTest<true>();
}
TEST_CASE("[KeyedMap] Test all functions (ordered)", "[KeyedMap]")
{
    runTest<false>();
}

} // namespace NAV::TESTS