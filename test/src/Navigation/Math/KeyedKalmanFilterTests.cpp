// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file KeyedKalmanFilterTests.cpp
/// @brief UnitTests for the KeyedKalmanFilter class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-07-11

#include <catch2/catch_test_macros.hpp>

#include "Logger.hpp"
#include "Navigation/Math/KeyedKalmanFilter.hpp"

#include <iostream>

namespace StateKey
{
enum States
{
    PosX,
    PosY,
    PosZ,
    VelX,
    VelY,
    VelZ,
    COUNT,
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
    constexpr bool operator==(const Pseudorange& rhs) const { return number == rhs.number; }
    size_t number = 0;
};
struct Carrierphase
{
    constexpr bool operator==(const Carrierphase& rhs) const { return number == rhs.number; }
    size_t number = 0;
};
} // namespace MeasurementKey

namespace std
{
/// @brief Hash function (needed for unordered_map)
template<>
struct hash<StateKey::Ambiguity>
{
    /// @brief Hash function
    /// @param[in] a Ambiguity
    size_t operator()(const StateKey::Ambiguity& a) const { return StateKey::States::COUNT + a.number; }
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

TEST_CASE("[KeyedKalmanFilter] Basic usage", "[KeyedKalmanFilter]")
{
    auto logger = initializeTestLogger();

    using StateKeys = std::variant<StateKey::States, StateKey::Ambiguity>;
    using MeasKeys = std::variant<MeasurementKey::Pseudorange, MeasurementKey::Carrierphase>;

    static const std::vector<StateKeys> Pos = { StateKey::PosX, StateKey::PosY, StateKey::PosZ };
    static const std::vector<StateKeys> Vel = { StateKey::VelX, StateKey::VelY, StateKey::VelZ };

    KeyedKalmanFilter<double, StateKeys, MeasKeys> kf({ StateKey::PosX, StateKey::PosY, StateKey::PosZ, StateKey::VelX, StateKey::VelY, StateKey::VelZ },
                                                      {});
    REQUIRE(kf.x(all) == Eigen::VectorXd::Zero(6));
    REQUIRE(kf.x(Pos) == Eigen::VectorXd::Zero(3));
    REQUIRE(kf.x(StateKey::PosX) == 0);
    REQUIRE(kf.P(all, all) == Eigen::MatrixXd::Zero(6, 6));
    REQUIRE(kf.P(Pos, Pos) == Eigen::MatrixXd::Zero(3, 3));
    REQUIRE(kf.P(Pos, Vel) == Eigen::MatrixXd::Zero(3, 3));
    REQUIRE(kf.P(StateKey::PosX, StateKey::PosX) == 0);
    REQUIRE(kf.P(StateKey::PosX, StateKey::VelX) == 0);
    REQUIRE(kf.Phi(all, all) == Eigen::MatrixXd::Zero(6, 6));
    REQUIRE(kf.Q(all, all) == Eigen::MatrixXd::Zero(6, 6));

    kf.setMeasurements({ MeasurementKey::Pseudorange{ 0 }, MeasurementKey::Pseudorange{ 1 },
                         MeasurementKey::Carrierphase{ 0 }, MeasurementKey::Carrierphase{ 1 } });
    REQUIRE(kf.z(all) == Eigen::VectorXd::Zero(4));
    REQUIRE(kf.H(all, all) == Eigen::MatrixXd::Zero(4, 6));
    REQUIRE(kf.R(all, all) == Eigen::MatrixXd::Zero(4, 4));
    REQUIRE(kf.S(all, all) == Eigen::MatrixXd::Zero(4, 4));
    REQUIRE(kf.K(all, all) == Eigen::MatrixXd::Zero(6, 4));

    kf.addState(StateKey::Ambiguity{ 0 });
    kf.addStates({ StateKey::Ambiguity{ 1 }, StateKey::Ambiguity{ 2 } });
    REQUIRE(kf.x(all) == Eigen::VectorXd::Zero(9));

    kf.x(all) = (Eigen::VectorXd(9) << 0, 1, 2, 3, 4, 5, 6, 7, 8).finished();
    kf.P(all, all) = Eigen::MatrixXd::Identity(9, 9);
    kf.Phi(all, all) = Eigen::MatrixXd::Identity(9, 9) * 2;
    kf.Q(all, all) = Eigen::MatrixXd::Ones(9, 9) * 0.1;

    kf.predict();

    REQUIRE(kf.x(all) == (Eigen::VectorXd(9) << 0, 1, 2, 3, 4, 5, 6, 7, 8).finished() * 2);
    REQUIRE(kf.P(all, all) == Eigen::MatrixXd::Identity(9, 9) * 4 + Eigen::MatrixXd::Ones(9, 9) * 0.1);

    kf.removeState(StateKey::Ambiguity{ 1 });
    REQUIRE(kf.x(all) == (Eigen::VectorXd(8) << 0, 1, 2, 3, 4, 5, 6, 8).finished() * 2);
    REQUIRE(kf.P(all, all).rows() == 8);
    REQUIRE(kf.Phi(all, all).rows() == 8);
    REQUIRE(kf.Q(all, all).rows() == 8);

    kf.removeStates(Vel);
    REQUIRE(kf.x(all) == (Eigen::VectorXd(5) << 0, 1, 2, 6, 8).finished() * 2);
    REQUIRE(kf.P(all, all).rows() == 5);
    REQUIRE(kf.Phi(all, all).rows() == 5);
    REQUIRE(kf.Q(all, all).rows() == 5);

    kf.correct();
    kf.correctWithMeasurementInnovation();
}

} // namespace NAV::TESTS