// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <catch2/catch_test_macros.hpp>

#include "Logger.hpp"
#include "Navigation/Math/ManagedKalmanFilter.hpp"

#include <iostream>

namespace StateKey
{
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
    constexpr bool operator==(const Ambiguity& rhs) const { return number == rhs.number; }

    size_t number = 0;
};
} // namespace StateKey

namespace MeasurementKey
{
struct Pseudorange
{
    size_t number = 0;
};
struct Carrierphase
{
    size_t number = 0;
};
} // namespace MeasurementKey

namespace std
{
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<StateKey::Position>
{
    /// @brief Hash function
    size_t operator()(const StateKey::Position& /* p */) const { return 0; }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<StateKey::Velocity>
{
    /// @brief Hash function
    size_t operator()(const StateKey::Velocity& /* v */) const { return 1; }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<StateKey::Ambiguity>
{
    /// @brief Hash function
    /// @param[in] a Ambiguity
    size_t operator()(const StateKey::Ambiguity& a) const { return 2 + a.number; }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<MeasurementKey::Pseudorange>
{
    /// @brief Hash function
    /// @param[in] psr Pseudorange
    size_t operator()(const MeasurementKey::Pseudorange& psr) const { return psr.number; }
};
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<MeasurementKey::Carrierphase>
{
    /// @brief Hash function
    /// @param[in] cp Carrierphase
    size_t operator()(const MeasurementKey::Carrierphase& cp) const { return 1000 + cp.number; }
};
} // namespace std

namespace NAV::TESTS
{

TEST_CASE("[Math] Managed Kalman Filter", "[Math][Debug]")
{
    auto logger = initializeTestLogger();

    using StateKeys = std::variant<StateKey::Position, StateKey::Velocity, StateKey::Ambiguity>;
    using MeasKeys = std::variant<MeasurementKey::Pseudorange, MeasurementKey::Carrierphase>;
    ManagedKalmanFilter<StateKeys, MeasKeys> mkf;

    mkf.addState(StateKey::Position{}, 3);
    mkf.addState(StateKey::Velocity{}, 3);
    mkf.addState(StateKey::Ambiguity{ 1 }, 1);
    mkf.addState(StateKey::Ambiguity{ 2 }, 1);

    mkf.x(StateKey::Position{}) = Eigen::Vector3d(1, 2, 3);
    mkf.x(StateKey::Velocity{}) = Eigen::Vector3d(4, 5, 6);
    mkf.x(StateKey::Ambiguity{ 1 }) = Eigen::VectorXd::Ones(1) * 7;
    mkf.x(StateKey::Ambiguity{ 2 }) = Eigen::VectorXd::Ones(1) * 8;

    REQUIRE(mkf.x(StateKey::Position{}) == Eigen::Vector3d(1, 2, 3));
    REQUIRE(mkf.x(StateKey::Velocity{}) == Eigen::Vector3d(4, 5, 6));
    REQUIRE(mkf.x(StateKey::Ambiguity{ 1 }) == Eigen::VectorXd::Ones(1) * 7);
    REQUIRE(mkf.x(StateKey::Ambiguity{ 2 }) == Eigen::VectorXd::Ones(1) * 8);
}

} // namespace NAV::TESTS