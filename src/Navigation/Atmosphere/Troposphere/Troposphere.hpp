/// @file Troposphere.hpp
/// @brief Troposphere Models
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2022-05-26

#pragma once

#include <array>
#include <Eigen/Core>

#include "ZenithDelay.hpp"

namespace NAV
{

/// Available Troposphere Models
enum class TroposphereModel : int
{
    None,         ///< Troposphere Model turned off
    Saastamoinen, ///< Saastamoinen model
    COUNT,        ///< Amount of items in the enum
};

/// Available Mapping Functions
enum class MappingFunction : int
{
    None,     ///< Mapping Function turned off (= 1)
    Cosecant, ///< Cosecant of elevation
    COUNT,    ///< Amount of items in the enum
};

/// @brief Converts the enum to a string
/// @param[in] troposphereModel Enum value to convert into text
/// @return String representation of the enum
const char* to_string(TroposphereModel troposphereModel);

/// @brief Converts the enum to a string
/// @param[in] mappingFunction Enum value to convert into text
/// @return String representation of the enum
const char* to_string(MappingFunction mappingFunction);

/// @brief Shows a ComboBox to select the troposphere model
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in] troposphereModel Reference to the troposphere model to select
bool ComboTroposphereModel(const char* label, TroposphereModel& troposphereModel);

/// @brief Shows a ComboBox to select the mapping function
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in] mappingFunction Reference to the mapping function to select
bool ComboMappingFunction(const char* label, MappingFunction& mappingFunction);

/// @brief Calculates the tropospheric zenith hydrostatic and wet delays
/// @param[in] lla_pos [𝜙, λ, h]^T Geodetic latitude, longitude and height in [rad, rad, m]
/// @param[in] troposphereModel Troposphere model to use
/// @return Range correction for troposphere and stratosphere for radio ranging in [m]
ZenithDelay calcTroposphericRangeDelay(const Eigen::Vector3d& lla_pos, TroposphereModel troposphereModel = TroposphereModel::None);

/// @brief Calculates the troposphere mapping factor
/// @param[in] elevation Angle between the user and satellite [rad]
/// @param[in] mappingFunction Mapping function to use
/// @return Mapping factor for converting tropospheric zenith delay into slant delay
double calcTropoMapFunc(double elevation, MappingFunction mappingFunction = MappingFunction::None);

} // namespace NAV
