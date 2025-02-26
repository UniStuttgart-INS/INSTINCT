// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file PressureToHeight.hpp
/// @brief Pressure to height converter
/// @author T. Hobiger (hobiger@ins.uni-stuttgart.de)
/// @date 2025-02-10

#pragma once

#include "internal/Node/Node.hpp"
#include "util/Random/RandomNumberGenerator.hpp"
#include "Navigation/Constants.hpp"

namespace NAV
{
/// Convert RTKLib pos files into PosVel
class PressureToHeight : public Node
{
  public:
    /// @brief Default constructor
    PressureToHeight();
    /// @brief Destructor
    ~PressureToHeight() override;
    /// @brief Copy constructor
    PressureToHeight(const PressureToHeight&) = delete;
    /// @brief Move constructor
    PressureToHeight(PressureToHeight&&) = delete;
    /// @brief Copy assignment operator
    PressureToHeight& operator=(const PressureToHeight&) = delete;
    /// @brief Move assignment operator
    PressureToHeight& operator=(PressureToHeight&&) = delete;

    /// @brief String representation of the Class Type
    [[nodiscard]] static std::string typeStatic();

    /// @brief String representation of the Class Type
    [[nodiscard]] std::string type() const override;

    /// @brief String representation of the Class Category
    [[nodiscard]] static std::string category();

    /// @brief ImGui config window which is shown on double click
    /// @attention Don't forget to set _hasConfig to true in the constructor of the node
    void guiConfig() override;

    /// @brief Saves the node into a json object
    [[nodiscard]] json save() const override;

    /// @brief Restores the node from a json object
    /// @param[in] j Json object with the node state
    void restore(const json& j) override;

  private:
    constexpr static size_t OUTPUT_PORT_INDEX_BAROHEIGHT = 0; ///< @brief Flow
    constexpr static size_t INPUT_PORT_INDEX_POS = 0;         ///< @brief Flow

    /// @brief Initialize the node
    bool initialize() override;

    /// @brief Deinitialize the node
    void deinitialize() override;

    /// @brief Converts the RtklibPosObs into PosVel
    /// @param[in] queue Queue with all the received data messages
    /// @param[in] pinIdx Index of the pin the data is received on
    void receiveObs(InputPin::NodeDataQueue& queue, size_t pinIdx);

    /// Temperature at Sea level in deg K
    double _temp0 = 288.15;
    /// Pressure at Sea level in hPa
    double _pressure0 = 1013.25;
    /// Temperature lapse rate  in K / m
    double _lapserate = 0.00976;
    /// Geoid undulation in m
    double _geoidhgt = 0.0;
    /// Gravity in m / s²
    double _gravity = InsConst::G_NORM;

    /// save computations, by storing g * M / R0 / L;
    double _exponent = InsConst::G_NORM * InsConst::dMtr / InsConst::Rg / _lapserate;

    /// Initial position in LLA [deg, deg, m] for the calculation of the local gravity through EGM96
    Eigen::Vector3d _initPos{ 48.780509, 9.171712, 300.0 };

    /// @brief Available options for gravity input
    enum class GravityInput : uint8_t
    {
        Manual,   ///< Manual entry of the gravity's magnitude
        Position, ///< Entry of the position, gravity is then deducted from EGM96
        COUNT,    ///< Number of items in the enum
    };

    /// Default gravity input type
    GravityInput _gravityInput = GravityInput::Manual;

    /// @brief Converts the enum to a string
    /// @param[in] value Enum value to convert into text
    /// @return String representation of the enum
    friend constexpr const char* to_string(GravityInput value);
};

/// @brief Converts the enum to a string
/// @param[in] value Enum value to convert into text
/// @return String representation of the enum
constexpr const char* to_string(NAV::PressureToHeight::GravityInput value)
{
    switch (value)
    {
    case NAV::PressureToHeight::GravityInput::Manual:
        return "Manual";
    case NAV::PressureToHeight::GravityInput::Position:
        return "Position";
    case NAV::PressureToHeight::GravityInput::COUNT:
        return "";
    }
    return "";
};

} // namespace NAV
