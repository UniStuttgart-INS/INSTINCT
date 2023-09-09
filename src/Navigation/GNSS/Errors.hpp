// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Errors.hpp
/// @brief Errors concerning GNSS observations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-06-09

#pragma once

#include <unordered_map>

#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "Navigation/GNSS/Core/Frequency.hpp"

namespace NAV
{

/// @brief Errors concerning GNSS observations
class GnssMeasurementErrorModel
{
  public:
    /// @brief Models
    enum Model : int
    {
        None,       ///< Measurement error model turned off
        RTKLIB,     ///< RTKLIB error model. See \cite RTKLIB RTKLIB ch. E.6, eq. E.6.24, p. 162
        Groves2013, ///< Measurement Error see Groves 2013 ch. 9.4.2.4, eq. 9.168, p. 422 (range acceleration is neglected)
        COUNT,      ///< Amount of items in the enum
    };
    /// @brief Model to use
    Model model = RTKLIB;

    /// @brief Calculates the measurement Error Variance for pseudorange observations
    /// @param[in] satSys Satellite System
    /// @param[in] freq Frequency
    /// @param[in] elevation Satellite Elevation in [rad]
    /// @return Variance of the measurement error [m^2]
    [[nodiscard]] double psrMeasErrorVar(const SatelliteSystem& satSys, const Frequency& freq, double elevation) const;

    /// @brief Calculates the measurement Error Variance for carrier-phase observations
    /// @param[in] satSys Satellite System
    /// @param[in] elevation Satellite Elevation in [rad]
    /// @return Variance of the measurement error [m^2]
    [[nodiscard]] double carrierMeasErrorVar(const SatelliteSystem& satSys, double elevation) const;

    /// @brief Returns the Code Bias Error Variance
    /// @return Variance of the code bias error [m^2]
    [[nodiscard]] double codeBiasErrorVar() const;

    /// @brief Returns the Pseudo-range rate Error Variance
    /// @param[in] freq Frequency the measurement originates from
    /// @param[in] num  Frequency number. Only used for GLONASS G1 and G2
    /// @return Variance of the Pseudo-range rate error [m^2/s^2]
    [[nodiscard]] double psrRateErrorVar(Frequency freq, int8_t num = -128) const;

    /// @brief Returns the Doppler Error Variance
    /// @return Variance of the Doppler error [Hz^2]
    [[nodiscard]] double dopplerErrorVar() const;

    /// @brief Calculates the measurement Error Variance for pseudorange observations for model of Groves 2013
    /// @param[in] elevation Satellite Elevation in [rad]
    /// @param[in] cn0 Carrier power to noise density
    /// @return Variance of the measurement error [m^2]
    [[nodiscard]] double psrMeasErrorVar(double elevation, double cn0) const;

    /// @brief Calculates the measurement Error Variance for doppler observations for model of Groves 2013
    /// @param[in] elevation Satellite Elevation in [rad]
    /// @param[in] cn0 Carrier power to noise density
    /// @return Variance of the measurement error [m^2]
    [[nodiscard]] double dopplerMeasErrorVar(double elevation, double cn0) const;

    /// @brief Shows a ComboBox to select the model
    /// @param[in] id Unique id for ImGui.
    /// @param[in] width Width of the widgets
    bool ShowGuiWidgets(const char* id, float width);

    /// @brief RTKLIB model parameters
    struct RtklibParameters
    {
        /// Code To Carrier Error ratio (L1, L2)
        std::array<double, 2> codeCarrierPhaseErrorRatio = { 100.0, 100.0 };
        /// Carrier-Phase Error Factor a+b [m] - Measurement error standard deviation
        std::array<double, 2> carrierPhaseErrorAB = { 0.003, 0.003 };
        /// Doppler Frequency error factor [Hz] - Measurement error standard deviation
        double dopplerFrequency = 1;
    };
    /// @brief Parameters for the RTKLIB model
    RtklibParameters rtklibParams;

    /// @brief Groves 2013 model parameters
    struct Groves2013Parameters
    {
        /// Pseudo-range error standard deviation in [m] (zenith and cn0-dependent)
        std::array<double, 2> sigmaRho = { 0.1, 0.1 };
        /// Pseudo-range rate error standard deviation in [m/s] (zenith and cn0-dependent)
        std::array<double, 2> sigmaR = { 0.1, 0.1 };
    };
    /// @brief Parameters for the model of Groves 2013
    Groves2013Parameters groves2013Params;

  private:
    /// @brief Calculates the measurement Error Variance for pseudorange or carrier-phase observations
    /// @param[in] satSys Satellite System
    /// @param[in] elevation Satellite Elevation in [rad]
    /// @return Variance of the measurement error [m^2]
    [[nodiscard]] double gnssMeasErrorVar(const SatelliteSystem& satSys, double elevation) const;
};

/// @brief Converts the enum to a string
/// @param[in] model Enum value to convert into text
/// @return String representation of the enum
const char* to_string(GnssMeasurementErrorModel::Model model);

/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
void to_json(json& j, const GnssMeasurementErrorModel& obj);
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
void from_json(const json& j, GnssMeasurementErrorModel& obj);

/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
void to_json(json& j, const GnssMeasurementErrorModel::RtklibParameters& obj);
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
void from_json(const json& j, GnssMeasurementErrorModel::RtklibParameters& obj);
/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
void to_json(json& j, const GnssMeasurementErrorModel::Groves2013Parameters& obj);
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
void from_json(const json& j, GnssMeasurementErrorModel::Groves2013Parameters& obj);

} // namespace NAV
