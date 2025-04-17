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
#include "Navigation/INS/LocalNavFrame/Mechanization.hpp"
#include "Navigation/INS/EcefFrame/Mechanization.hpp"

#include "util/Container/ScrollingBuffer.hpp"
#include "util/Eigen.hpp"
#include "util/Json.hpp"
#include "util/Logger.hpp"

namespace NAV
{

/// @brief Inertial Measurement Integrator
class InertialIntegrator
{
  public:
    /// Available Integration Algorithms
    enum class IntegrationAlgorithm : uint8_t
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

    /// Available Integration Frames
    enum class IntegrationFrame : uint8_t
    {
        ECEF, ///< Earth-Centered Earth-Fixed frame
        NED,  ///< Local North-East-Down frame
    };

    /// Inertial Measurement
    struct Measurement
    {
        bool averagedMeasurement = false; ///< Wether the acceleration is averaged over the last epoch
        double dt = 0.0;                  ///< Time since previous observation in [s]
        Eigen::Vector3d p_acceleration;   ///< Acceleration in platform frame coordinates in [m/s^2]
        Eigen::Vector3d p_angularRate;    ///< Angular rate in platform frame coordinates in [rad/s]
    };

    /// @brief Inertial state and measurements
    template<typename T>
    struct GenericState
    {
        InsTime epoch; ///< Epoch of this state

        Eigen::Vector3<T> position;    ///< IMU position (e_pos / lla_pos)
        Eigen::Vector3<T> velocity;    ///< IMU velocity (e_vel / n_vel)
        Eigen::Quaternion<T> attitude; ///< IMU attitude (e_Quat_b / n_Quat_b)

        Measurement m; ///< Inertial measurement

        Eigen::Vector3<T> p_biasAcceleration = Eigen::Vector3<T>::Zero(); ///< Acceleration bias in platform frame coordinates in [m/s^2]
        Eigen::Vector3<T> p_biasAngularRate = Eigen::Vector3<T>::Zero();  ///< Angular rate bias in platform frame coordinates in [rad/s]

        Eigen::Vector3<T> scaleFactorAccel = Eigen::Vector3<T>::Ones(); ///< Scale factor of the accelerometer [-]
        Eigen::Vector3<T> scaleFactorGyro = Eigen::Vector3<T>::Ones();  ///< Scale factor of the gyroscope [-]

        Eigen::Quaternion<T> misalignmentAccel = Eigen::Quaternion<T>::Identity(); ///< Misalignment of the accelerometer sensor axes
        Eigen::Quaternion<T> misalignmentGyro = Eigen::Quaternion<T>::Identity();  ///< Misalignment of the gyroscope sensor axes
    };

    /// Inertial state and measurements
    using State = GenericState<double>;

    /// @brief Default Constructor
    InertialIntegrator() = default;

    /// @brief Constructor
    /// @param integrationFrame Integration frame to lock to
    explicit InertialIntegrator(IntegrationFrame integrationFrame);

    /// @brief Clears all internal data
    void reset();

    /// @brief Checks if an initial position is set
    [[nodiscard]] bool hasInitialPosition() const;

    /// @brief Sets the initial state
    /// @param[in] state State to set
    /// @param[in] nameId NameId of the calling node for logging
    void setInitialState(const PosVelAtt& state, const char* nameId);

    /// @brief Pushes the state to the list of states
    /// @param[in] state State to set
    /// @param[in] nameId NameId of the calling node for logging
    void addState(const PosVelAtt& state, const char* nameId);

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

    /// Get the latest state if it exists
    [[nodiscard]] std::optional<std::reference_wrapper<const State>> getLatestState() const;

    /// @brief Return the last acceleration bias in platform frame coordinates in [m/s^2]
    [[nodiscard]] const Eigen::Vector3d& p_getLastAccelerationBias() const;

    /// @brief Return the last angular rate bias in platform frame coordinates in [rad/s]
    [[nodiscard]] const Eigen::Vector3d& p_getLastAngularRateBias() const;

    /// @brief Returns the selected integration frame
    [[nodiscard]] IntegrationFrame getIntegrationFrame() const;

    /// @brief Returns the selected compensation models
    [[nodiscard]] const PosVelAttDerivativeConstants& getConstants() const;

    /// Wether the measurements are accumulated values over the last epoch. (always true when using delta measurements, so GUI has no effect)
    [[nodiscard]] bool areAccelerationsAveragedMeasurements() const;

    /// Calculate the current acceleration, if measurements area available
    [[nodiscard]] std::optional<Eigen::Vector3d> p_calcCurrentAcceleration() const;

    /// Calculate the current angular rate, if measurements area available
    [[nodiscard]] std::optional<Eigen::Vector3d> p_calcCurrentAngularRate() const;

    /// @brief Calculates the inertial navigation solution
    /// @param[in] obsTime Time of the observation
    /// @param[in] p_acceleration Acceleration in platform frame coordinates in [m/s^2]
    /// @param[in] p_angularRate Angular rate in platform frame coordinates in [rad/s]
    /// @param[in] imuPos IMU platform frame position with respect to body frame
    /// @param[in] nameId NameId of the calling node for logging
    /// @return The new state at the observation time
    std::shared_ptr<PosVelAtt> calcInertialSolution(const InsTime& obsTime,
                                                    const Eigen::Vector3d& p_acceleration, const Eigen::Vector3d& p_angularRate,
                                                    const ImuPos& imuPos, const char* nameId);

    /// @brief Calculates the inertial navigation solution
    /// @param[in] obsTime Time of the observation
    /// @param[in] deltaTime Delta time over which the deltaVelocity and deltaTheta were measured in [s]
    /// @param[in] p_deltaVelocity Integrated acceleration in platform frame coordinates in [m/s]
    /// @param[in] p_deltaTheta Integrated angular rate in platform frame coordinates in [rad]
    /// @param[in] imuPos IMU platform frame position with respect to body frame
    /// @param[in] nameId NameId of the calling node for logging
    /// @return The new state at the observation time
    std::shared_ptr<PosVelAtt> calcInertialSolutionDelta(const InsTime& obsTime, double deltaTime,
                                                         const Eigen::Vector3d& p_deltaVelocity, const Eigen::Vector3d& p_deltaTheta,
                                                         const ImuPos& imuPos, const char* nameId);

    // Forward declaring external function
    friend const char* to_string(InertialIntegrator::IntegrationAlgorithm algorithm);
    friend const char* to_string(InertialIntegrator::IntegrationFrame frame);

    /// @brief Calculates the inertial solution going from state__t1 to state__t0 given that measurements are available for both states
    /// @param imuPos IMU mounting position connecting the platform to the body frame
    /// @param state__t0 State at the epoch to calculate (measurements only)
    /// @param state__t1 State at the previous epoch (state + measurements)
    /// @param nameId NameId of the calling node for logging
    /// @return Position, velocity and attitude from the integration step
    template<typename T>
    [[nodiscard]] Eigen::Vector<T, 10> calcInertialSolution(const ImuPos& imuPos,
                                                            const GenericState<T>& state__t0,
                                                            const GenericState<T>& state__t1,
                                                            [[maybe_unused]] const char* nameId) const
    {
        // #if LOG_LEVEL <= LOG_LEVEL_DATA
        //         auto printState = [](const GenericState<T>& state, const char* t, const char* nameId) {
        //             LOG_DATA("{} [{}]:\n"
        //                      " - {}:\n"
        //                      "    Position [{}, {}, {}], Velocity [{}, {}, {}], Attitude [{}x, {}y, {}z, {}w]\n"
        //                      "    dt = {:.5f}, p_accel [{}, {}, {}], p_angRate [{}, {}, {}]\n"
        //                      "    p_biasAccel [{}, {}, {}], p_biasAngRate [{}, {}, {}]\n"
        //                      "    p_scaleFacAccel [{}, {}, {}], p_scaleFacAngRate [{}, {}, {}]\n"
        //                      "    p_misAlignAccel [{}x, {}y, {}z, {}w], p_misAlignAngRate [{}x, {}y, {}z, {}w]",
        //                      nameId, t, state.epoch.toYMDHMS(GPST),
        //                      state.position(0), state.position(1), state.position(2),
        //                      state.velocity(0), state.velocity(1), state.velocity(2),
        //                      state.attitude.x(), state.attitude.y(), state.attitude.z(), state.attitude.w(),
        //                      state.m.dt, state.m.p_acceleration(0), state.m.p_acceleration(1), state.m.p_acceleration(2),
        //                      state.m.p_angularRate(0), state.m.p_angularRate(1), state.m.p_angularRate(2),
        //                      state.p_biasAcceleration(0), state.p_biasAcceleration(1), state.p_biasAcceleration(2),
        //                      state.p_biasAngularRate(0), state.p_biasAngularRate(1), state.p_biasAngularRate(2),
        //                      state.scaleFactorAccel(0), state.scaleFactorAccel(1), state.scaleFactorAccel(2),
        //                      state.scaleFactorGyro(0), state.scaleFactorGyro(1), state.scaleFactorGyro(2),
        //                      state.misalignmentAccel.x(), state.misalignmentAccel.y(), state.misalignmentAccel.z(), state.misalignmentAccel.w(),
        //                      state.misalignmentGyro.x(), state.misalignmentGyro.y(), state.misalignmentGyro.z(), state.misalignmentGyro.w());
        //         };
        //         LOG_DATA("{}: Frame {}, Algorithm {}", nameId, to_string(_integrationFrame), to_string(_integrationAlgorithm));
        //         printState(state__t0, "t0", nameId);
        //         printState(state__t1, "t1", nameId);
        // #endif

        Eigen::Vector<T, 10> y;
        //        0  1  2   3    4    5     6       7       8       9
        // NED  [ 𝜙, λ, h, v_N, v_E, v_D, n_q_bx, n_q_by, n_q_bz, n_q_bw]^T
        // ECEF [ x, y, z, v_x, v_y, v_z, e_q_bx, e_q_by, e_q_bz, e_q_bw]^T
        y.template segment<3>(0) = state__t1.position;
        y.template segment<3>(3) = state__t1.velocity;
        y.template segment<4>(6) = state__t1.attitude.coeffs();

        auto p_accel = [](const GenericState<T>& state) -> Eigen::Vector3<T> {
            return (state.misalignmentAccel * state.m.p_acceleration.template cast<T>()).cwiseProduct(state.scaleFactorAccel) + state.p_biasAcceleration;
        };
        auto p_gyro = [](const GenericState<T>& state) -> Eigen::Vector3<T> {
            return (state.misalignmentGyro * state.m.p_angularRate.template cast<T>()).cwiseProduct(state.scaleFactorGyro) + state.p_biasAngularRate;
        };

        // LOG_DATA("{}: imuPos.b_quat_p() [{}]", nameId, imuPos.b_quat_p());
        Eigen::Vector3<T> b_accel__t0 = imuPos.b_quat_p().cast<T>() * p_accel(state__t0);
        Eigen::Vector3<T> b_accel__t1 = imuPos.b_quat_p().cast<T>() * p_accel(state__t1);
        Eigen::Vector3<T> b_gyro__t0 = imuPos.b_quat_p().cast<T>() * p_gyro(state__t0);
        Eigen::Vector3<T> b_gyro__t1 = imuPos.b_quat_p().cast<T>() * p_gyro(state__t1);
        // LOG_DATA("{}: b_accel__t0 [{}, {}, {}], b_accel__t1 [{}, {}, {}]", nameId, b_accel__t0(0), b_accel__t0(1), b_accel__t0(2), b_accel__t1(0), b_accel__t1(1), b_accel__t1(2));
        // LOG_DATA("{}: b_gyro__t0 [{}, {}, {}], b_gyro__t1 [{}, {}, {}]", nameId, b_gyro__t0(0), b_gyro__t0(1), b_gyro__t0(2), b_gyro__t1(0), b_gyro__t1(1), b_gyro__t1(2));

        // LOG_DATA("{}: integrationAlgorithm = {}", nameId, to_string(_integrationAlgorithm));
        switch (_integrationAlgorithm)
        {
        case IntegrationAlgorithm::SingleStepRungeKutta1:
        {
            std::array<Eigen::Vector<T, 6>, 1> z;
            if (state__t0.m.averagedMeasurement) { z[0] << b_accel__t0, b_gyro__t0; }
            else { z[0] << b_accel__t1, b_gyro__t1; }
            // LOG_DATA("{}: z[0] = {}, {}, {}, {}, {}, {}", nameId, z[0](0), z[0](1), z[0](2), z[0](3), z[0](4), z[0](5));

            switch (_integrationFrame)
            {
            case IntegrationFrame::NED:
                y = RungeKutta1(y, z, state__t0.m.dt, n_calcPosVelAttDerivative<T>, _posVelAttDerivativeConstants);
                break;
            case IntegrationFrame::ECEF:
                y = RungeKutta1(y, z, state__t0.m.dt, e_calcPosVelAttDerivative<T>, _posVelAttDerivativeConstants);
                break;
            }
            break;
        }
        case IntegrationAlgorithm::SingleStepRungeKutta2:
        {
            std::array<Eigen::Vector<T, 6>, 2> z;
            if (state__t0.m.averagedMeasurement)
            {
                z[0] << b_accel__t0, b_gyro__t0;
                z[1] << b_accel__t0, b_gyro__t0;
            }
            else
            {
                z[0] << b_accel__t1, b_gyro__t1;
                z[1] << math::lerp(b_accel__t1, b_accel__t0, 0.5), math::lerp(b_gyro__t1, b_gyro__t0, 0.5);
            }
            // LOG_DATA("{}: z[0] = {}, {}, {}, {}, {}, {}", nameId, z[0](0), z[0](1), z[0](2), z[0](3), z[0](4), z[0](5));
            // LOG_DATA("{}: z[1] = {}, {}, {}, {}, {}, {}", nameId, z[1](0), z[1](1), z[1](2), z[1](3), z[1](4), z[1](5));
            switch (_integrationFrame)
            {
            case IntegrationFrame::NED:
                y = RungeKutta2(y, z, state__t0.m.dt, n_calcPosVelAttDerivative<T>, _posVelAttDerivativeConstants);
                break;
            case IntegrationFrame::ECEF:
                y = RungeKutta2(y, z, state__t0.m.dt, e_calcPosVelAttDerivative<T>, _posVelAttDerivativeConstants);
                break;
            }
            break;
        }
        case IntegrationAlgorithm::SingleStepHeun2:
        {
            std::array<Eigen::Vector<T, 6>, 2> z;
            if (state__t0.m.averagedMeasurement)
            {
                z[0] << b_accel__t0, b_gyro__t0;
                z[1] << b_accel__t0, b_gyro__t0;
            }
            else
            {
                z[0] << b_accel__t1, b_gyro__t1;
                z[1] << b_accel__t0, b_gyro__t0;
            }
            // LOG_DATA("{}: z[0] = {}, {}, {}, {}, {}, {}", nameId, z[0](0), z[0](1), z[0](2), z[0](3), z[0](4), z[0](5));
            // LOG_DATA("{}: z[1] = {}, {}, {}, {}, {}, {}", nameId, z[1](0), z[1](1), z[1](2), z[1](3), z[1](4), z[1](5));
            switch (_integrationFrame)
            {
            case IntegrationFrame::NED:
                y = Heun2(y, z, state__t0.m.dt, n_calcPosVelAttDerivative<T>, _posVelAttDerivativeConstants);
                break;
            case IntegrationFrame::ECEF:
                y = Heun2(y, z, state__t0.m.dt, e_calcPosVelAttDerivative<T>, _posVelAttDerivativeConstants);
                break;
            }
            break;
        }
        case IntegrationAlgorithm::SingleStepRungeKutta3:
        {
            std::array<Eigen::Vector<T, 6>, 3> z;
            if (state__t0.m.averagedMeasurement)
            {
                z[0] << b_accel__t0, b_gyro__t0;
                z[1] << b_accel__t0, b_gyro__t0;
                z[2] << b_accel__t0, b_gyro__t0;
            }
            else
            {
                z[0] << b_accel__t1, b_gyro__t1;
                z[1] << math::lerp(b_accel__t1, b_accel__t0, 0.5), math::lerp(b_gyro__t1, b_gyro__t0, 0.5);
                z[2] << b_accel__t0, b_gyro__t0;
            }
            // LOG_DATA("{}: z[0] = {}, {}, {}, {}, {}, {}", nameId, z[0](0), z[0](1), z[0](2), z[0](3), z[0](4), z[0](5));
            // LOG_DATA("{}: z[1] = {}, {}, {}, {}, {}, {}", nameId, z[1](0), z[1](1), z[1](2), z[1](3), z[1](4), z[1](5));
            // LOG_DATA("{}: z[2] = {}, {}, {}, {}, {}, {}", nameId, z[2](0), z[2](1), z[2](2), z[2](3), z[2](4), z[2](5));
            switch (_integrationFrame)
            {
            case IntegrationFrame::NED:
                y = RungeKutta3(y, z, state__t0.m.dt, n_calcPosVelAttDerivative<T>, _posVelAttDerivativeConstants);
                break;
            case IntegrationFrame::ECEF:
                y = RungeKutta3(y, z, state__t0.m.dt, e_calcPosVelAttDerivative<T>, _posVelAttDerivativeConstants);
                break;
            }
            break;
        }
        case IntegrationAlgorithm::SingleStepHeun3:
        {
            std::array<Eigen::Vector<T, 6>, 3> z;
            if (state__t0.m.averagedMeasurement)
            {
                z[0] << b_accel__t0, b_gyro__t0;
                z[1] << b_accel__t0, b_gyro__t0;
                z[2] << b_accel__t0, b_gyro__t0;
            }
            else
            {
                z[0] << b_accel__t1, b_gyro__t1;
                z[1] << math::lerp(b_accel__t1, b_accel__t0, 1.0 / 3.0), math::lerp(b_gyro__t1, b_gyro__t0, 1.0 / 3.0);
                z[2] << math::lerp(b_accel__t1, b_accel__t0, 2.0 / 3.0), math::lerp(b_gyro__t1, b_gyro__t0, 2.0 / 3.0);
            }
            // LOG_DATA("{}: z[0] = {}, {}, {}, {}, {}, {}", nameId, z[0](0), z[0](1), z[0](2), z[0](3), z[0](4), z[0](5));
            // LOG_DATA("{}: z[1] = {}, {}, {}, {}, {}, {}", nameId, z[1](0), z[1](1), z[1](2), z[1](3), z[1](4), z[1](5));
            // LOG_DATA("{}: z[2] = {}, {}, {}, {}, {}, {}", nameId, z[2](0), z[2](1), z[2](2), z[2](3), z[2](4), z[2](5));
            switch (_integrationFrame)
            {
            case IntegrationFrame::NED:
                y = RungeKutta3(y, z, state__t0.m.dt, n_calcPosVelAttDerivative<T>, _posVelAttDerivativeConstants);
                break;
            case IntegrationFrame::ECEF:
                y = RungeKutta3(y, z, state__t0.m.dt, e_calcPosVelAttDerivative<T>, _posVelAttDerivativeConstants);
                break;
            }
            break;
        }
        case IntegrationAlgorithm::SingleStepRungeKutta4:
        {
            std::array<Eigen::Vector<T, 6>, 4> z;
            if (state__t0.m.averagedMeasurement)
            {
                z[0] << b_accel__t0, b_gyro__t0;
                z[1] << b_accel__t0, b_gyro__t0;
                z[2] << b_accel__t0, b_gyro__t0;
                z[3] << b_accel__t0, b_gyro__t0;
            }
            else
            {
                z[0] << b_accel__t1, b_gyro__t1;
                z[1] << math::lerp(b_accel__t1, b_accel__t0, 0.5), math::lerp(b_gyro__t1, b_gyro__t0, 0.5);
                z[2] << math::lerp(b_accel__t1, b_accel__t0, 0.5), math::lerp(b_gyro__t1, b_gyro__t0, 0.5);
                z[3] << b_accel__t0, b_gyro__t0;
            }
            // LOG_DATA("{}: z[0] = {}, {}, {}, {}, {}, {}", nameId, z[0](0), z[0](1), z[0](2), z[0](3), z[0](4), z[0](5));
            // LOG_DATA("{}: z[1] = {}, {}, {}, {}, {}, {}", nameId, z[1](0), z[1](1), z[1](2), z[1](3), z[1](4), z[1](5));
            // LOG_DATA("{}: z[2] = {}, {}, {}, {}, {}, {}", nameId, z[2](0), z[2](1), z[2](2), z[2](3), z[2](4), z[2](5));
            // LOG_DATA("{}: z[3] = {}, {}, {}, {}, {}, {}", nameId, z[3](0), z[3](1), z[3](2), z[3](3), z[3](4), z[3](5));
            switch (_integrationFrame)
            {
            case IntegrationFrame::NED:
                y = RungeKutta4(y, z, state__t0.m.dt, n_calcPosVelAttDerivative<T>, _posVelAttDerivativeConstants);
                break;
            case IntegrationFrame::ECEF:
                y = RungeKutta4(y, z, state__t0.m.dt, e_calcPosVelAttDerivative<T>, _posVelAttDerivativeConstants);
                break;
            }
            break;
        }
        case IntegrationAlgorithm::MultiStepRK3:
        {
            LOG_CRITICAL("Not implemented yet");
            break;
        }
        case IntegrationAlgorithm::MultiStepRK4:
        {
            LOG_CRITICAL("Not implemented yet");
            break;
        }
        case IntegrationAlgorithm::COUNT:
        {
            LOG_CRITICAL("Unreachable");
            break;
        }
        }

        y.template segment<4>(6) = Eigen::Quaternion<T>(y.template segment<4>(6)).normalized().coeffs();

        return y;
    }

  private:
    /// @brief Adds the measurement to the tracking buffer
    /// @param[in] epoch Epoch of the measurement
    /// @param[in] dt Time between observation and last state in [s]
    /// @param[in] p_acceleration Acceleration in platform frame coordinates in [m/s^2]
    /// @param[in] p_angularRate Angular rate in platform frame coordinates in [rad/s]
    /// @param[in] nameId NameId of the calling node for logging
    void addMeasurement(const InsTime& epoch, double dt,
                        const Eigen::Vector3d& p_acceleration, const Eigen::Vector3d& p_angularRate, const char* nameId);

    /// @brief Adds the measurement to the tracking buffer
    /// @param[in] epoch Epoch of the measurement
    /// @param[in] dt Time between observation and last state in [s]
    /// @param[in] deltaTime Delta time over which the deltaVelocity and deltaTheta were measured in [s]
    /// @param[in] p_deltaVelocity Integrated acceleration in platform frame coordinates in [m/s]
    /// @param[in] p_deltaTheta Integrated angular rate in platform frame coordinates in [rad]
    /// @param[in] nameId NameId of the calling node for logging
    void addDeltaMeasurement(const InsTime& epoch, double dt, double deltaTime,
                             const Eigen::Vector3d& p_deltaVelocity, const Eigen::Vector3d& p_deltaTheta, const char* nameId);

    /// @brief Returns the last state as PosVelAtt
    [[nodiscard]] std::shared_ptr<PosVelAtt> lastStateAsPosVelAtt() const;

    /// @brief Calculates the inertial navigation solution
    /// @param[in] imuPos IMU platform frame position with respect to body frame
    /// @param[in] nameId NameId of the calling node for logging
    /// @return The new state at the observation time
    std::shared_ptr<PosVelAtt> calcInertialSolutionFromMeasurementBuffer(const ImuPos& imuPos, const char* nameId);

    /// @brief Resizes the measurement and state buffers depending on the integration algorithm
    void setBufferSizes();

    /// List of states. Length depends on algorithm used
    ScrollingBuffer<State> _states = ScrollingBuffer<State>(1);

    Eigen::Vector3d _p_lastBiasAcceleration = Eigen::Vector3d::Zero(); ///< Initial values for the acceleration bias [m/s^2]
    Eigen::Vector3d _p_lastBiasAngularRate = Eigen::Vector3d::Zero();  ///< Initial values for the angular rate bias [rad/s]

    // #########################################################################################################################################

    /// Frame to integrate the observations in
    IntegrationFrame _integrationFrame = IntegrationFrame::NED;

    /// Wether to lock the integration frame
    bool _lockIntegrationFrame = false;

    /// Integration algorithm used for the update
    IntegrationAlgorithm _integrationAlgorithm = IntegrationAlgorithm::SingleStepRungeKutta3;

    /// If true, then the measurements are accumulated values over the last epoch. (always true when using delta measurements, so GUI has no effect)
    bool _accelerationsAreAveragedMeasurements = true;

    // #########################################################################################################################################

    /// Settings for the models to use
    PosVelAttDerivativeConstants _posVelAttDerivativeConstants;

    friend bool InertialIntegratorGui(const char* label, InertialIntegrator& integrator, bool& preferAccelerationOverDeltaMeasurements, float width);
    friend void to_json(json& j, const InertialIntegrator& data);
    friend void from_json(const json& j, InertialIntegrator& data);
};

/// @brief Shows a GUI for advanced configuration of the InertialIntegrator
/// @param[in] label Label to show. This has to be a unique id for ImGui.
/// @param[in] integrator Reference to the integrator to configure
/// @param[in] preferAccelerationOverDeltaMeasurements Wether to prefer accelerations over delta measurements
/// @param[in] width Width of the widget
bool InertialIntegratorGui(const char* label, InertialIntegrator& integrator, bool& preferAccelerationOverDeltaMeasurements, float width = 250.0F);

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

/// @brief Converts the enum to a string
/// @param[in] frame Enum value to convert into text
/// @return String representation of the enum
const char* to_string(InertialIntegrator::IntegrationFrame frame);

} // namespace NAV
