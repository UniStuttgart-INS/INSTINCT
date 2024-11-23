// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Troposphere.hpp
/// @brief Troposphere Models
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @author Rui Wang (rui.wang@ins.uni-stuttgart.de)
/// @date 2022-05-26

#pragma once

#include <array>
#include <Eigen/Core>

#include "ZenithDelay.hpp"
#include "Navigation/Time/InsTime.hpp"
#include "Navigation/Atmosphere/Pressure/Pressure.hpp"
#include "Navigation/Atmosphere/Temperature/Temperature.hpp"
#include "Navigation/Atmosphere/WaterVapor/WaterVapor.hpp"
#include "Navigation/Atmosphere/Troposphere/MappingFunctions/ViennaMappingFunction.hpp"

namespace NAV
{

/// @brief Atmospheric model selection for temperature, pressure and water vapor
struct AtmosphereModels
{
    PressureModel pressureModel = PressureModel::ISA;          ///< Pressure model
    TemperatureModel temperatureModel = TemperatureModel::ISA; ///< Temperature model
    WaterVaporModel waterVaporModel = WaterVaporModel::ISA;    ///< WaterVapor model
};

/// Available Troposphere delay models
enum class TroposphereModel : uint8_t
{
    None,         ///< Troposphere Model turned off
    Saastamoinen, ///< Saastamoinen model
    GPT2,         ///< GPT2
    GPT3,         ///< GPT3
    COUNT,        ///< Amount of items in the enum
};

/// Available Mapping Functions
enum class MappingFunction : uint8_t
{
    None,     ///< Mapping Function turned off (= 1)
    Cosecant, ///< Cosecant of elevation
    GMF,      ///< Global Mapping Function (GMF)
    NMF,      ///< Niell Mapping Function (NMF)
    VMF_GPT2, ///< Vienna Mapping Function based on the GPT2 grid
    VMF_GPT3, ///< Vienna Mapping Function based on the GPT3 grid
    COUNT,    ///< Amount of items in the enum
};

/// @brief Collection of troposphere model selections
struct TroposphereModelSelection
{
    /// Troposphere ZHD model, atmosphere models
    std::pair<TroposphereModel, AtmosphereModels> zhdModel = std::make_pair(TroposphereModel::Saastamoinen, AtmosphereModels{});
    /// Troposphere ZWD model, atmosphere models
    std::pair<TroposphereModel, AtmosphereModels> zwdModel = std::make_pair(TroposphereModel::Saastamoinen, AtmosphereModels{});

    /// Mapping function ZHD, atmosphere models
    std::pair<MappingFunction, AtmosphereModels> zhdMappingFunction = std::make_pair(MappingFunction::GMF, AtmosphereModels{});
    /// Mapping function ZWD, atmosphere models
    std::pair<MappingFunction, AtmosphereModels> zwdMappingFunction = std::make_pair(MappingFunction::GMF, AtmosphereModels{});
};

/// @brief Converts the enum to a string
/// @param[in] troposphereZhdModel Enum value to convert into text
/// @return String representation of the enum
const char* to_string(TroposphereModel troposphereZhdModel);

/// @brief Converts the enum to a string
/// @param[in] mappingFunction Enum value to convert into text
/// @return String representation of the enum
const char* to_string(MappingFunction mappingFunction);

/// @brief Shows a ComboBox and button for advanced configuration to select the troposphere models
/// @param[in] label Label to show beside the combo box. This has to be a unique id for ImGui.
/// @param[in] troposphereModelSelection Reference to the troposphere model to select
/// @param[in] width Width of the widget
bool ComboTroposphereModel(const char* label, TroposphereModelSelection& troposphereModelSelection, float width = 0.0F);

/// @brief Calculates the tropospheric zenith hydrostatic and wet delays and corresponding mapping factors
/// @param[in] insTime Time to calculate the values for
/// @param[in] lla_pos [𝜙, λ, h]^T Geodetic latitude, longitude and height in [rad, rad, m]
/// @param[in] elevation Satellite elevation [rad]
/// @param[in] azimuth Satellite azimuth [rad]
/// @param[in] troposphereModels Models to use for each calculation
/// @return ZHD, ZWD and mapping factors for ZHD and ZWD
ZenithDelay calcTroposphericDelayAndMapping(const InsTime& insTime, const Eigen::Vector3d& lla_pos, double elevation, double azimuth,
                                            const TroposphereModelSelection& troposphereModels);

/// @brief Calculates the tropospheric error variance
/// @param[in] dpsr_T Tropospheric propagation error [m]
/// @param[in] elevation Satellite elevation in [rad]
/// @return Variance of the error [m^2]
double tropoErrorVar(double dpsr_T, double elevation);

/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
void to_json(json& j, const AtmosphereModels& obj);
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
void from_json(const json& j, AtmosphereModels& obj);

/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
void to_json(json& j, const TroposphereModelSelection& obj);
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
void from_json(const json& j, TroposphereModelSelection& obj);

} // namespace NAV
