// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file MeasurementErrors.hpp
/// @brief Errors concerning GNSS observations
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-06-09
/// #### Model descriptions
/// @anchor GNSS-MeasErrorModel

#pragma once

#include <array>

#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "Navigation/GNSS/Core/Frequency.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "util/Container/Array.hpp"

namespace NAV
{

/// @brief Errors concerning GNSS observations
class GnssMeasurementErrorModel
{
  public:
    /// @brief Default constructor
    GnssMeasurementErrorModel();

    /// @brief Models
    enum Model : int
    {
        None,        ///< Measurement error model turned off
        SINE,        ///< Sine. See \cite Dach2015 Dach 2015
        SINE_OFFSET, ///< Sine with offset. See \cite Zhang2021 Zhang 2021 eq. 7, p. 3
        SINE_CN0,    ///< Sine and CN0 dependent. See \cite Groves2013 Groves, ch. 9.4.2.4, eq. 9.168, p. 422 (range acceleration is neglected)
        RTKLIB,      ///< RTKLIB error model. See \cite RTKLIB RTKLIB ch. E.6, eq. E.6.24, p. 162
        SINE_TYPE,   ///< Sine Type. See \cite KingBock2001 King and Bock 2001
        SINE_SQRT,   ///< Sine square-root. See \cite Kiliszek2022 Kiliszek 2022, table 2, p. 5
        EXPONENTIAL, ///< Exponential. See \cite EulerGoad1991 Euler and Goad 1991 / \cite Li2016 Li et al. 2016
        COSINE_TYPE, ///< Cosine Type. See \cite Hadas2020 Hadas 2020 eq. 14, p. 8
        COUNT,       ///< Amount of items in the enum
    };

    /// @brief Calculates the measurement Error Variance for pseudorange observations
    /// @param[in] satSys Satellite System
    /// @param[in] elevation Satellite Elevation in [rad]
    /// @param[in] cn0 Carrier-to-Noise density [dB-Hz]
    /// @return Variance of the measurement error [m^2]
    [[nodiscard]] double psrMeasErrorVar(const SatelliteSystem& satSys, double elevation, double cn0) const;

    /// @brief Calculates the measurement Error Variance for carrier-phase observations
    /// @param[in] satSys Satellite System
    /// @param[in] elevation Satellite Elevation in [rad]
    /// @param[in] cn0 Carrier-to-Noise density [dB-Hz]
    /// @return Variance of the measurement error [m^2]
    [[nodiscard]] double carrierMeasErrorVar(const SatelliteSystem& satSys, double elevation, double cn0) const;

    /// @brief Returns the Pseudo-range rate Error Variance
    /// @param[in] freq Frequency the measurement originates from
    /// @param[in] num  Frequency number. Only used for GLONASS G1 and G2
    /// @param[in] elevation Satellite Elevation in [rad]
    /// @param[in] cn0 Carrier-to-Noise density [dB-Hz]
    /// @return Variance of the Pseudo-range rate error [m^2/s^2]
    [[nodiscard]] double psrRateMeasErrorVar(const Frequency& freq, int8_t num, double elevation, double cn0) const;

    /// @brief Returns the Code Bias Error Variance
    /// @return Variance of the code bias error [m^2]
    [[nodiscard]] double codeBiasErrorVar() const;

    /// @brief Shows a GUI widgets
    /// @param[in] id Unique id for ImGui.
    /// @param[in] width Width of the widgets
    bool ShowGuiWidgets(const char* id, float width);

  private:
    /// @brief Model to use
    Model _model = RTKLIB;

    /// Carrier Measurement error standard deviation per Frequency [m]
    double _carrierStdDev = 0.001;
    /// Code/Pseudorange Measurement error standard deviation per Frequency [m]
    double _codeStdDev = 0.1;
    /// Doppler Frequency error factor [Hz] - Measurement error standard deviation
    double _dopplerStdDev = 1;

    /// @brief Carrier-to-Noise density [dB-Hz] to use in the plot
    double _plotCN0 = 30;

    // #######################################################################################################

    /// Model parameters for the 'sine' model
    struct ModelParametersSine
    {
        double a = 1.0; ///< Coefficient
    };
    /// Model parameters for the 'sine' model
    ModelParametersSine _modelParametersSine;

    /// Model parameters for the 'sine + offset' model
    struct ModelParametersSineOffset
    {
        double a = 0.5; ///< Coefficient
        double b = 0.5; ///< Coefficient
    };
    /// Model parameters for the 'sine + offset' model
    ModelParametersSineOffset _modelParametersSineOffset;

    /// Model parameters for the 'sine + CN0' model
    struct ModelParametersSineCN0
    {
        double a = 1.0; ///< Coefficient
        double b = 1.0; ///< Coefficient
        double c = 1.0; ///< Factor to weight the CN0 dependant part
    };
    /// Model parameters for the 'sine + CN0' model
    ModelParametersSineCN0 _modelParametersSineCN0;

    /// Model parameters for the 'RTKLIB' model
    struct ModelParametersRtklib
    {
        double a = 1.0; ///< Coefficient
        double b = 1.0; ///< Coefficient
    };
    /// Model parameters for the 'RTKLIB' model
    ModelParametersRtklib _modelParametersRtklib;

    /// Model parameters for the 'sine - type' model
    struct ModelParametersSineType
    {
        double a = 0.64; ///< Coefficient
        double b = 0.36; ///< Coefficient
    };
    /// Model parameters for the 'sine - type' model
    ModelParametersSineType _modelParametersSineType;

    /// Model parameters for the 'sine - sqrt' model
    struct ModelParametersSineSqrt
    {
        double a = 0.3; ///< Coefficient
        double b = 0.5; ///< Coefficient
    };
    /// Model parameters for the 'sine - sqrt' model
    ModelParametersSineSqrt _modelParametersSineSqrt;

    /// Model parameters for the 'exponential' model
    struct ModelParametersExponential
    {
        double a = 1.0; ///< Coefficient
        double b = 3.5; ///< Coefficient
        double e0 = 9;  ///< Coefficient [deg]
    };
    /// Model parameters for the 'exponential' model
    ModelParametersExponential _modelParametersExponential;

    /// Model parameters for the 'cosine - type' model
    struct ModelParametersCosineType
    {
        double a = 1.0; ///< Coefficient
        double b = 4.0; ///< Coefficient
        int n = 8;      ///< Coefficient
    };
    /// Model parameters for the 'cosine - type' model
    ModelParametersCosineType _modelParametersCosineType;

    // #######################################################################################################

    /// @brief Amount of samples for the plot
    static constexpr size_t PLOT_SAMPLES = 9001;
    /// @brief Elevation data for plotting [rad]
    static constexpr std::array<double, PLOT_SAMPLES> _elevation = genRangeArray<PLOT_SAMPLES>(0.0, deg2rad(0.01), deg2rad(90.001));
    /// @brief Elevation data for plotting [rad]
    static constexpr std::array<double, PLOT_SAMPLES> _elevation_deg = genRangeArray<PLOT_SAMPLES>(0.0, 0.01, 90.001);
    /// @brief Standard deviations for plotting
    std::vector<std::vector<double>> _stdDevCurvePlot{ Model::COUNT, std::vector<double>(PLOT_SAMPLES) };

    /// @brief Calculates the weighting function for the standard deviation
    /// @param[in] model Model to use
    /// @param[in] elevation Satellite Elevation in [rad]
    /// @param[in] cn0 Carrier-to-Noise density [dB-Hz]
    [[nodiscard]] double weightingFunction(Model model, double elevation, double cn0) const;

    /// @brief Returns an error factor for the variance depending on the satellite system
    /// @param satSys Satellite system
    [[nodiscard]] static double satSysErrorFactorVariance(const SatelliteSystem& satSys);

    /// @brief Updates the curve plot data for the given model
    /// @param model Error model to use
    void updateStdDevCurvePlot(Model model);

    friend void to_json(json& j, const GnssMeasurementErrorModel& obj);
    friend void from_json(const json& j, GnssMeasurementErrorModel& obj);

    friend void to_json(json& j, const ModelParametersSine& obj);
    friend void from_json(const json& j, ModelParametersSine& obj);

    friend void to_json(json& j, const ModelParametersSineOffset& obj);
    friend void from_json(const json& j, ModelParametersSineOffset& obj);

    friend void to_json(json& j, const ModelParametersSineCN0& obj);
    friend void from_json(const json& j, ModelParametersSineCN0& obj);

    friend void to_json(json& j, const ModelParametersRtklib& obj);
    friend void from_json(const json& j, ModelParametersRtklib& obj);

    friend void to_json(json& j, const ModelParametersSineType& obj);
    friend void from_json(const json& j, ModelParametersSineType& obj);

    friend void to_json(json& j, const ModelParametersSineSqrt& obj);
    friend void from_json(const json& j, ModelParametersSineSqrt& obj);

    friend void to_json(json& j, const ModelParametersExponential& obj);
    friend void from_json(const json& j, ModelParametersExponential& obj);

    friend void to_json(json& j, const ModelParametersCosineType& obj);
    friend void from_json(const json& j, ModelParametersCosineType& obj);
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
void to_json(json& j, const GnssMeasurementErrorModel::ModelParametersSine& obj);
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
void from_json(const json& j, GnssMeasurementErrorModel::ModelParametersSine& obj);

/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
void to_json(json& j, const GnssMeasurementErrorModel::ModelParametersSineOffset& obj);
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
void from_json(const json& j, GnssMeasurementErrorModel::ModelParametersSineOffset& obj);

/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
void to_json(json& j, const GnssMeasurementErrorModel::ModelParametersSineCN0& obj);
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
void from_json(const json& j, GnssMeasurementErrorModel::ModelParametersSineCN0& obj);

/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
void to_json(json& j, const GnssMeasurementErrorModel::ModelParametersRtklib& obj);
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
void from_json(const json& j, GnssMeasurementErrorModel::ModelParametersRtklib& obj);

/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
void to_json(json& j, const GnssMeasurementErrorModel::ModelParametersSineType& obj);
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
void from_json(const json& j, GnssMeasurementErrorModel::ModelParametersSineType& obj);

/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
void to_json(json& j, const GnssMeasurementErrorModel::ModelParametersSineSqrt& obj);
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
void from_json(const json& j, GnssMeasurementErrorModel::ModelParametersSineSqrt& obj);

/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
void to_json(json& j, const GnssMeasurementErrorModel::ModelParametersExponential& obj);
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
void from_json(const json& j, GnssMeasurementErrorModel::ModelParametersExponential& obj);

/// @brief Converts the provided object into json
/// @param[out] j Json object which gets filled with the info
/// @param[in] obj Object to convert into json
void to_json(json& j, const GnssMeasurementErrorModel::ModelParametersCosineType& obj);
/// @brief Converts the provided json object into a node object
/// @param[in] j Json object with the needed values
/// @param[out] obj Object to fill from the json
void from_json(const json& j, GnssMeasurementErrorModel::ModelParametersCosineType& obj);

} // namespace NAV
