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
/// #### Model descriptions
/// @anchor GNSS-MeasErrorModel
///
/// <b> rtklib </b>
/// \fl{equation,eq-MeasErrorModel-rtklib}
/// \begin{array}{rl}
/// \sigma_{pj}^2 &= \sigma_{pseudo}^2 + \sigma_{eph}^2  + \sigma_{ion}^2  + \sigma_{trop}^2  + \sigma_{bias}^2 \\
/// \sigma_{rj}^2 &= \sigma_{doppler}^2 + \sigma_{eph}^2 \\
/// \end{array}
/// \f}
/// - \f$ \sigma_{pseudo} \f$ pseudorange measurement error variance
///   \f$ \sigma_{pseudo}^2 = {F^{s}}^2 R_r^2 \left( a_{\sigma}^2 + \frac{b_{\sigma}^2}{\sin(el^s_r)} \right) \f$
///     - \f$ F^s \f$ satellite system error factor (1.5 for GLONASS, 1 for GPS / GALILEO / QZSS / BeiDou, 3.0 for SBAS)
///     - \f$ R_r \f$ code/carrier-phase error ratio (default 300)
///     - \f$ a_{\sigma} \f$, \f$ b_{\sigma} \f$ carrier‐phase error factor a and b (default both 0.003) in [m]
/// - \f$ \sigma_{\text{eph}} \f$ standard deviation of ephemeris and clock error in [m]
/// - \f$ \sigma_{\text{ion}} \f$ standard deviation of ionosphere correction model error in [m]
///
///   \f$ \sigma^2_{\text{ion}} = \frac{f_{L1}^2}{f^2} (I \cdot err_{\text{BRDCI}})^2 \f$
///     where \f$ err_{\text{BRDCI}} \f$ is the broadcast iono model error factor (0.5)
///
///     - \f$ \sigma_{\text{trop}} \f$ standard deviation of troposphere correction model error in [m]
///
///     \f$ \sigma^2_{\text{trop}} = \left(\frac{err_{\text{SAAS}}}{\sin(el) + 0.1}\right)^2 \f$
///     where \f$ err_{\text{SAAS}} \f$ is the saastamoinen model error (0.3) in [m]
///
/// - \f$ \sigma_{\text{bias}} \f$ standard deviation of code bias error in [m]
///
///   \f$ \sigma^2_{\text{bias}} = err_{\text{CBIAS}}^2 \f$
///     where \f$ err_{\text{CBIAS}} \f$ is the code bias error 0.3 m
///
/// - \f$ \sigma_{Doppler} \f$
///   \f$ \sigma_{Doppler}^2 = err_{f_\text{Doppler}}^2 \f$
///     where \f$ err_{f_\text{doppler}} \f$ is the Doppler Frequency error factor in [Hz] (1.0)
///
/// A weight matrix for Least squares estimation or a measurement noise covariance matrix for Kalman Filtering would contain
///
/// \fl{equation,eq-MeasErrorModel-rtklib-W}
/// \mathbf{W} = \mathbf{R} = \left[\begin{array}{cccc:cccc}
/// 1/\sigma_{p 1}^2 & 0 & \dots & 0 & 0 & 0 & \dots & 0 \\
/// 0 & 1/\sigma_{p 2}^2 & 0 & \dots & 0 & 0 & \dots & \dots \\
/// \vdots & \vdots & \ddots & \vdots & \vdots & \vdots & \ddots & \vdots \\
/// 0 & 0 & \dots & 1/\sigma_{p m}^2 & 0 & 0 & \dots & 0 \\
/// \hdashline
/// 0 & 0 & \dots & 0                 & 1/\sigma_{d 1}^2 & 0 & \dots & 0 \\
/// 0 & 0 & \dots & \dots             & 0 & 1/\sigma_{d 2}^2 & 0 & \dots \\
/// \vdots & \vdots & \ddots & \vdots & \vdots & \vdots & \ddots & \vdots \\
/// 0 & 0 & \dots & 0                 & 0 & 0 & \dots & 1/\sigma_{d m}^2 \\
/// \end{array}\right]
/// \f}
/// (\cite RTKLIB rtklib, eq. E6.23, p. 158)
///
/// <b> Groves </b>
/// \fl{equation,eq-MeasErrorModel-groves}
/// \begin{array}{rl}
/// \sigma_{pj}^2 &= \dfrac{1}{\sin^2({\varepsilon})} \left( \sigma_{pZ}^2 + \dfrac{\sigma_{pc}^2}{(c/n_0)_j} + \sigma_{pa}^2 \ddot{r}_{aj}^2 \right)\\
/// \sigma_{rj}^2 &= \dfrac{1}{\sin^2({\varepsilon})} \left( \sigma_{rZ}^2 + \dfrac{\sigma_{rc}^2}{(c/n_0)_j} + \sigma_{ra}^2 \ddot{r}_{aj}^2 \right)\\
/// \end{array}
/// \f}
/// - \f$ \varepsilon \f$ Satellite elevation in [rad]
/// - \f$ c/n_0 \f$ Carrier power to noise density
/// - coefficients \f$ \sigma_{pZ} \f$, \f$ \sigma_{pc} \f$, \f$ \sigma_{pa} \f$, \f$ \sigma_{rZ} \f$, \f$ \sigma_{rc} \f$ and \f$ \sigma_{ra} \f$ should be determined empirically
/// - \f$ \ddot{r}_{aj} \f$ Range acceleration [groves2013, Appendix G.4.1] (neglected)
///
/// A weight matrix for Least squares estimation or a measurement noise covariance matrix for Kalman Filtering would contain
///
/// \fl{equation,eq-MeasErrorModel-groves-W}
/// \mathbf{W} = \mathbf{R} = \left[\begin{array}{cccc:cccc}
/// \sigma_{p 1}^2 & 0 & \dots & 0 & 0 & 0 & \dots & 0 \\
/// 0 & \sigma_{p 2}^2 & 0 & \dots & 0 & 0 & \dots & \dots \\
/// \vdots & \vdots & \ddots & \vdots & \vdots & \vdots & \ddots & \vdots \\
/// 0 & 0 & \dots & \sigma_{p m}^2 & 0 & 0 & \dots & 0 \\
/// \hdashline
/// 0 & 0 & \dots & 0                 & \sigma_{d 1}^2 & 0 & \dots & 0 \\
/// 0 & 0 & \dots & \dots             & 0 & \sigma_{d 2}^2 & 0 & \dots \\
/// \vdots & \vdots & \ddots & \vdots & \vdots & \vdots & \ddots & \vdots \\
/// 0 & 0 & \dots & 0                 & 0 & 0 & \dots & \sigma_{d m}^2 \\
/// \end{array}\right]
/// \f}
/// (\cite Groves2013 Groves, ch. 9.4.2.2, eq. 9.168, p. 422)

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
        Groves2013, ///< Measurement Error. See \cite Groves2013 Groves, ch. 9.4.2.4, eq. 9.168, p. 422 (range acceleration is neglected)
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
