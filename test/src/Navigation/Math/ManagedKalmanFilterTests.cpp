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

struct Position
{
    constexpr bool operator==(const Position& /* rhs */) const { return true; }
};
struct Velocity
{
    constexpr bool operator==(const Velocity& /* rhs */) const { return true; }
};
struct Ambiguity
{
    constexpr Ambiguity(size_t number) : number(number) {}
    constexpr bool operator==(const Ambiguity& rhs) const { return number == rhs.number; }

    size_t number = 0;
};

struct Pseudorange
{
    size_t number = 0;
};
struct Carrierphase
{
    size_t number = 0;
};

namespace std
{
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<Position>
{
    /// @brief Hash function
    size_t operator()(const Position& /* p */) const { return 0; }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<Velocity>
{
    /// @brief Hash function
    size_t operator()(const Velocity& /* v */) const { return 1; }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<Ambiguity>
{
    /// @brief Hash function
    /// @param[in] a Ambiguity
    size_t operator()(const Ambiguity& a) const { return 2 + a.number; }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<Pseudorange>
{
    /// @brief Hash function
    /// @param[in] psr Pseudorange
    size_t operator()(const Pseudorange& psr) const { return psr.number; }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<Carrierphase>
{
    /// @brief Hash function
    /// @param[in] cp Carrierphase
    size_t operator()(const Carrierphase& cp) const { return 1000 + cp.number; }
};
} // namespace std

namespace NAV::TESTS
{

TEST_CASE("[Math] Managed Kalman Filter", "[Math][Debug]")
{
    auto logger = initializeTestLogger();

    using StateKeys = std::variant<Position, Velocity, Ambiguity>;
    using MeasKeys = std::variant<Pseudorange, Carrierphase>;
    ManagedKalmanFilter<StateKeys, MeasKeys> mkf;

    mkf.addState(Position{}, 3);
    mkf.addState(Velocity{}, 3);
    mkf.addState(Ambiguity{ 1 }, 1);
    mkf.addState(Ambiguity{ 2 }, 1);

    mkf.x(Position{}) = Eigen::Vector3d(1, 2, 3);
    mkf.x(Velocity{}) = Eigen::Vector3d(4, 5, 6);
    mkf.x(Ambiguity{ 1 }) = Eigen::VectorXd::Ones(1) * 7;
    mkf.x(Ambiguity{ 2 }) = Eigen::VectorXd::Ones(1) * 8;

    REQUIRE(mkf.x(Position{}) == Eigen::Vector3d(1, 2, 3));
    REQUIRE(mkf.x(Velocity{}) == Eigen::Vector3d(4, 5, 6));
    REQUIRE(mkf.x(Ambiguity{ 1 }) == Eigen::VectorXd::Ones(1) * 7);
    REQUIRE(mkf.x(Ambiguity{ 2 }) == Eigen::VectorXd::Ones(1) * 8);

    std::cout << mkf._x.transpose() << std::endl;
    REQUIRE(false);
}

} // namespace NAV::TESTS