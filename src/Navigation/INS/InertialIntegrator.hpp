// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file InertialIntegrator.hpp
/// @brief Inertial Measurement Integrator
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-12-09

#pragma once

#include <functional>
#include <optional>
#include <memory>

#include "NodeData/IMU/ImuPos.hpp"
#include "NodeData/State/PosVelAtt.hpp"

#include "Navigation/Gravity/Gravity.hpp"
#include "Navigation/Math/NumericalIntegration.hpp"
#include "Navigation/Time/InsTime.hpp"
#include "Navigation/INS/Mechanization.hpp"

#include "util/Container/ScrollingBuffer.hpp"
#include "util/Eigen.hpp"
#include "util/Json.hpp"

namespace NAV
{

/// @brief Inertial Measurement Integrator
class InertialIntegrator
{
  public:
    /// Available Integration Algorithms
    enum class IntegrationAlgorithm
    {
        SingleStepRungeKutta1, ///< Runge-Kutta 1st order (explicit) / (Forward) Euler method
        SingleStepRungeKutta2, ///< Runge-Kutta 2nd order (explicit) / Explicit midpoint method
        SingleStepHeun2,       ///< Heun's method (2nd order) (explicit)
        SingleStepRungeKutta3, ///< Runge-Kutta 3rd order (explicit) / Simpson's rule
        SingleStepHeun3,       ///< Heun's method (3nd order) (explicit)
        SingleStepRungeKutta4, ///< Runge-Kutta 4th order (explicit)
        MultiStepRK3,          ///< Multistep Runge-Kutta 3rd order (explicit) / Simpson's rule (taking 2 old epochs into account)
        MultiStepRK4,          ///< Multistep Runge-Kutta 4th order (explicit) (taking 2 old epochs into account)
        COUNT,                 ///< Amount of available integration algorithms
    };

    /// @brief Inertial Measurement
    struct Measurement
    {
        double dt = 0.0;                ///< Time since previous observation
        Eigen::Vector3d p_acceleration; ///< Acceleration in platform frame coordinates in [m/s^2]
        Eigen::Vector3d p_angularRate;  ///< Angular rate in platform frame coordinates in [rad/s]

        Eigen::Vector3d p_biasAcceleration = Eigen::Vector3d::Zero(); ///< Acceleration bias in platform frame coordinates in [m/s^2]
        Eigen::Vector3d p_biasAngularRate = Eigen::Vector3d::Zero();  ///< Angular rate bias in platform frame coordinates in [rad/s]
    };

    /// @brief Clears all internal data
    void reset();

    /// @brief Checks if an initial position is set
    [[nodiscard]] bool hasInitialPosition() const;

    /// @brief Sets the initial state
    /// @param[in] state State to set
    void setInitialState(const PosVelAtt& state);

    /// @brief Pushes the state to the list of states
    /// @param[in] state State to set
    void setState(const PosVelAtt& state);

    /// @brief Sets the sensor biases total values
    /// @param[in] p_biasAcceleration Acceleration bias in platform frame coordinates in [m/s^2]
    /// @param[in] p_biasAngularRate Angular rate bias in platform frame coordinates in [rad/s]
    void setTotalSensorBiases(const Eigen::Vector3d& p_biasAcceleration, const Eigen::Vector3d& p_biasAngularRate);

    /// @brief Sets the sensor biases increment
    /// @param[in] p_deltaBiasAcceleration Acceleration bias increment in platform frame coordinates in [m/s^2]
    /// @param[in] p_deltaBiasAngularRate Angular rate bias increment in platform frame coordinates in [rad/s]
    void applySensorBiasesIncrements(const Eigen::Vector3d& p_deltaBiasAcceleration, const Eigen::Vector3d& p_deltaBiasAngularRate);

    /// @brief Apply the errors to the latest state
    /// @param[in] lla_positionError δ𝐩_n = [δ𝜙 δλ δ𝘩] The position error (latitude, longitude, altitude) in [rad, rad, m]
    /// @param[in] n_velocityError δ𝐯_n The velocity error in n frame coordinates in [m/s]
    /// @param[in] n_attitudeError_b δ𝛙_nb_n The attitude error in n frame coordinates in [rad]
    void applyStateErrors_n(const Eigen::Vector3d& lla_positionError, const Eigen::Vector3d& n_velocityError, const Eigen::Vector3d& n_attitudeError_b);

    /// @brief Apply the errors to the latest state
    /// @param[in] e_positionError δr_e The position error in e frame coordinates in [m]
    /// @param[in] e_velocityError δ𝐯_e The velocity error in e frame coordinates in [m/s]
    /// @param[in] e_attitudeError_b δ𝛙_eb_e The attitude error in e frame coordinates in [rad]
    void applyStateErrors_e(const Eigen::Vector3d& e_positionError, const Eigen::Vector3d& e_velocityError, const Eigen::Vector3d& e_attitudeError_b);

    /// Get the measurements buffer
    [[nodiscard]] const ScrollingBuffer<Measurement>& getMeasurements() const;

    /// Get the latest state if it exists
    [[nodiscard]] std::optional<std::reference_wrapper<const PosVelAtt>> getLatestState() const;

    /// @brief Return the last acceleration bias in platform frame coordinates in [m/s^2]
    [[nodiscard]] const Eigen::Vector3d& p_getLastAccelerationBias() const;

    /// @brief Return the last angular rate bias in platform frame coordinates in [rad/s]
    [[nodiscard]] const Eigen::Vector3d& p_getLastAngularRateBias() const;

    /// Available Integration Frames
    enum class IntegrationFrame : int
    {
        ECEF, ///< Earth-Centered Earth-Fixed frame
        NED,  ///< Local North-East-Down frame
    };

    /// @brief Returns the selected integration frame
    [[nodiscard]] IntegrationFrame getIntegrationFrame() const;

    /// Calculate the current acceleration, if measurements area available
    [[nodiscard]] std::optional<Eigen::Vector3d> p_calcCurrentAcceleration() const;

    /// Calculate the current angular rate, if measurements area available
    [[nodiscard]] std::optional<Eigen::Vector3d> p_calcCurrentAngularRate() const;

    /// @brief Calculates the inertial navigation solution
    /// @param[in] obsTime Time of the observation
    /// @param[in] p_acceleration Acceleration in platform frame coordinates in [m/s^2]
    /// @param[in] p_angularRate Angular rate in platform frame coordinates in [rad/s]
    /// @param[in] imuPos IMU platform frame position with respect to body frame
    /// @return The new state at the observation time
    std::shared_ptr<PosVelAtt> calcInertialSolution(const InsTime& obsTime, const Eigen::Vector3d& p_acceleration, const Eigen::Vector3d& p_angularRate, const ImuPos& imuPos);

    /// @brief Calculates the inertial navigation solution
    /// @param[in] obsTime Time of the observation
    /// @param[in] dt Delte Time over which the deltaVelocity and deltaTheta were measured
    /// @param[in] p_deltaVelocity Integrated Acceleration in platform frame coordinates in [m/s]
    /// @param[in] p_deltaTheta Integrated Angular rate in platform frame coordinates in [rad]
    /// @param[in] p_acceleration Acceleration in platform frame coordinates in [m/s^2]
    /// @param[in] p_angularRate Angular rate in platform frame coordinates in [rad/s]
    /// @param[in] imuPos IMU platform frame position with respect to body frame
    /// @return The new state at the observation time
    std::shared_ptr<PosVelAtt> calcInertialSolutionDelta(const InsTime& obsTime, const double& dt, Eigen::Vector3d p_deltaVelocity, Eigen::Vector3d p_deltaTheta,
                                                         Eigen::Vector3d p_acceleration, Eigen::Vector3d p_angularRate, const ImuPos& imuPos);

  private:
    /// @brief Calculates the inertial navigation solution
    /// @param[in] imuPos IMU platform frame position with respect to body frame
    /// @return The new state at the observation time
    std::shared_ptr<PosVelAtt> calcInertialSolutionFromMeasurementBuffer(const ImuPos& imuPos);

    /// @brief Resizes the measurement and state buffers depending on the integration algorithm
    void setBufferSizes();

    /// List of measurements. Length depends on algorithm used
    ScrollingBuffer<Measurement> _measurements = ScrollingBuffer<Measurement>(2);
    /// List of states. Length depends on algorithm used
    ScrollingBuffer<PosVelAtt> _states = ScrollingBuffer<PosVelAtt>(1);

    Eigen::Vector3d p_lastBiasAcceleration = Eigen::Vector3d::Zero(); ///< Acceleration bias in platform frame coordinates in [m/s^2]
    Eigen::Vector3d p_lastBiasAngularRate = Eigen::Vector3d::Zero();  ///< Angular rate bias in platform frame coordinates in [rad/s]

    // #########################################################################################################################################

    /// Frame to integrate the observations in
    IntegrationFrame _integrationFrame = IntegrationFrame::NED;

    /// Integration algorithm used for the update
    IntegrationAlgorithm _integrationAlgorithm = IntegrationAlgorithm::SingleStepRungeKutta3;

    // #########################################################################################################################################

    /// Settings for the models to use
    PosVelAttDerivativeConstants<double> _posVelAttDerivativeConstants;

    friend bool InertialIntegratorGui(const char* label, InertialIntegrator& integrator, float width);
    friend void to_json(json& j, const InertialIntegrator& data);
    friend void from_json(const json& j, InertialIntegrator& data);
};

/// @brief Shows a GUI for advanced configuration of the InertialIntegrator
/// @param[in] label Label to show. This has to be a unique id for ImGui.
/// @param[in] integrator Reference to the integrator to configure
/// @param[in] width Width of the widget
bool InertialIntegratorGui(const char* label, InertialIntegrator& integrator, float width = 250.0F);

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const InertialIntegrator& data);
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
void from_json(const json& j, InertialIntegrator& data);

/// @brief Converts the enum to a string
/// @param[in] algorithm Enum value to convert into text
/// @return String representation of the enum
const char* to_string(InertialIntegrator::IntegrationAlgorithm algorithm);

} // namespace NAV
