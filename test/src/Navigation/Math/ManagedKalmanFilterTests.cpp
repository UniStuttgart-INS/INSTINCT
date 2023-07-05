// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <catch2/catch_test_macros.hpp>

#include "Logger.hpp"
#include "util/Container/Pair.hpp"
#include "Navigation/Math/ManagedKalmanFilter.hpp"

#include <iostream>

// struct Position
// {};
// struct Velocity
// {};
// struct Ambiguity
// {
//     Ambiguity(size_t number) : number(number) {}
//     size_t number = 0;
// };

// struct Pseudorange
// {
//     size_t number = 0;
// };
// struct Carrierphase
// {
//     size_t number = 0;
// };

// namespace std
// {
// /// @brief Hash function (needed for unordered_map)
// template<>
// struct hash<Position>
// {
//     /// @brief Hash function
//     size_t operator()(const Position& /* p */) const { return 0; }
// };
// /// @brief Hash function (needed for unordered_map)
// template<>
// struct hash<Velocity>
// {
//     /// @brief Hash function
//     size_t operator()(const Velocity& /* v */) const { return 1; }
// };
// /// @brief Hash function (needed for unordered_map)
// template<>
// struct hash<Ambiguity>
// {
//     /// @brief Hash function
//     /// @param[in] a Ambiguity
//     size_t operator()(const Ambiguity& a) const { return 2 + a.number; }
// };
// /// @brief Hash function (needed for unordered_map)
// template<>
// struct hash<Pseudorange>
// {
//     /// @brief Hash function
//     /// @param[in] psr Pseudorange
//     size_t operator()(const Pseudorange& psr) const { return psr.number; }
// };
// /// @brief Hash function (needed for unordered_map)
// template<>
// struct hash<Carrierphase>
// {
//     /// @brief Hash function
//     /// @param[in] cp Carrierphase
//     size_t operator()(const Carrierphase& cp) const { return 1000 + cp.number; }
// };
// } // namespace std

namespace NAV::TESTS
{

TEST_CASE("[Math] Managed Kalman Filter", "[Math]")
{
    auto logger = initializeTestLogger();

    enum StateKeys
    {
        Position,
        Velocity,
        Ambiguity
    };
    enum MeasKeys
    {
        Psr,
        Carrier,
    };

    ManagedKalmanFilter<std::pair<StateKeys, size_t>, std::pair<MeasKeys, size_t>> mkf;

    mkf.addState(std::make_pair(Position, 0UL), 3);
    mkf.addState(std::make_pair(Velocity, 0UL), 3);
    mkf.addState(std::make_pair(Ambiguity, 1UL), 1);
    mkf.addState(std::make_pair(Ambiguity, 2UL), 3);

    // mkf.addState(Position{}, 3);
    // mkf.addState(Velocity{}, 3);
    // mkf.addState(Ambiguity{ 1 }, 1);
    // mkf.addState(Ambiguity{ 2 }, 1);

    mkf.x(std::make_pair(Position, 0UL)) = Eigen::Vector3d(1, 2, 3);
    // REQUIRE(mkf.x.rows() == 2);

    std::cout << mkf._x << std::endl;
    REQUIRE(false);
}

} // namespace NAV::TESTS