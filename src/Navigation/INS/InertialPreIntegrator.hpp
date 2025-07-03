// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file InertialPreIntegrator.hpp
/// @brief Inertial Measurement Preintegrator
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2025-06-03

#pragma once

#include "Navigation/Math/Math.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "NodeData/IMU/ImuPos.hpp"

#include "Navigation/Gravity/Gravity.hpp"
#include "Navigation/Time/InsTime.hpp"

#include "util/Eigen.hpp"
#include "util/Logger.hpp"
#include "util/Json.hpp"

namespace NAV
{

/// @brief Inertial Measurement Integrator
class InertialPreIntegrator
{
  public:
    /// Available Integration Frames
    enum class IntegrationFrame : uint8_t
    {
        ECEF, ///< Earth-Centered Earth-Fixed frame
        NED,  ///< Local North-East-Down frame
    };

    /// IMU measurement
    struct ImuMeasurement
    {
        double dt = 0.0;                ///< Time since previous observation in [s]
        Eigen::Vector3d p_acceleration; ///< Acceleration in platform frame coordinates in [m/s^2]
        Eigen::Vector3d p_angularRate;  ///< Angular rate in platform frame coordinates in [rad/s]
    };

    /// IMU state
    template<typename T>
    struct ImuState
    {
        Eigen::Vector3<T> p_biasAcceleration = Eigen::Vector3<T>::Zero(); ///< Acceleration bias in platform frame coordinates in [m/s^2]
        Eigen::Vector3<T> p_biasAngularRate = Eigen::Vector3<T>::Zero();  ///< Angular rate bias in platform frame coordinates in [rad/s]

        // Eigen::Vector3<T> scaleFactorAccel = Eigen::Vector3<T>::Ones(); ///< Scale factor of the accelerometer [-]
        // Eigen::Vector3<T> scaleFactorGyro = Eigen::Vector3<T>::Ones();  ///< Scale factor of the gyroscope [-]

        // Eigen::Quaternion<T> misalignmentAccel = Eigen::Quaternion<T>::Identity(); ///< Misalignment of the accelerometer sensor axes
        // Eigen::Quaternion<T> misalignmentGyro = Eigen::Quaternion<T>::Identity();  ///< Misalignment of the gyroscope sensor axes

        /// @brief Equal comparison
        /// @param[in] rhs Right hand side of the operator
        /// @return True if the elements are equal
        constexpr bool operator==(const ImuState<T>& rhs) const
        {
            return p_biasAcceleration == rhs.p_biasAcceleration && p_biasAngularRate == rhs.p_biasAngularRate;
            //    && scaleFactorAccel == rhs.scaleFactorAccel && scaleFactorGyro == rhs.scaleFactorGyro
            //    && misalignmentAccel == rhs.misalignmentAccel && misalignmentGyro == rhs.misalignmentGyro;
        }
    };

    /// Inertial Measurement for a generic type
    template<typename T>
    struct GenericMeasurement
    {
        ImuMeasurement meas; ///< IMU measurement
        ImuState<T> imu;     ///< IMU state
    };

    /// Position, velocity and attitude state
    template<typename T>
    struct PVAState
    {
        Eigen::Vector3<T> position;    ///< IMU position (e_pos / lla_pos)
        Eigen::Vector3<T> velocity;    ///< IMU velocity (e_vel / n_vel)
        Eigen::Quaternion<T> attitude; ///< IMU attitude (e_Quat_b / n_Quat_b)
    };

    /// Inertial measurements
    using Measurement = GenericMeasurement<double>;

    /// @brief Default Constructor
    InertialPreIntegrator() = default;

    /// @brief Constructor
    /// @param integrationFrame Integration frame to lock to
    explicit InertialPreIntegrator(IntegrationFrame integrationFrame);

    /// @brief Clears all internal data
    void reset();

    /// @brief Set the IMU Noise Scale Matrix
    /// @param[in] W IMU Noise Scale Matrix
    void setImuNoiseScaleMatrix(const Eigen::Matrix6d& W);

    /// @brief Adds a inertial measurement to the relative motion increments
    /// @param[in] meas Inertial measurement
    /// @param[in] imuPos IMU mounting position connecting the platform to the body frame
    /// @param[in] nameId Name and id of the calling node for logging
    void addInertialMeasurement(const Measurement& meas, const ImuPos& imuPos, [[maybe_unused]] const std::string& nameId);

    /// @brief Calculates the state using the preintegrated relative motion increments
    /// @param[in] pvaOld Position, velocity and attitude at the start of the preintegration interval
    /// @param[in] imuNew The IMU state at the moment of evaluation
    /// @param[in] imuPos IMU mounting position connecting the platform to the body frame
    /// @param[in] nameId Name and id of the calling node for logging
    /// @return The position, velocity and attitude at the end of the preintegration interval
    template<typename T>
    PVAState<T> calcIntegratedState(const PVAState<T>& pvaOld, const ImuState<T>& imuNew, const ImuPos& imuPos, [[maybe_unused]] const std::string& nameId)
    {
        // LOG_DATA("{}: calcIntegratedState (dt = {})", nameId, dt);
        // LOG_DATA("{}:   pvaOld.attitude = {}", nameId, pvaOld.attitude);
        // LOG_DATA("{}:   pvaOld.velocity = {}", nameId, pvaOld.velocity.transpose());
        // LOG_DATA("{}:   pvaOld.position = {}", nameId, pvaOld.position.transpose());

        // LOG_DATA("{}:   attitudeIncrement = {}", nameId, _attitudeIncrement);
        // LOG_DATA("{}:   velocityIncrement = {}", nameId, _velocityIncrement.transpose());
        // LOG_DATA("{}:   positionIncrement = {}", nameId, _positionIncrement.transpose());

        Eigen::Vector3d deltaBiasAngularRate = imuPos.b_quat_p() * (imuNew.p_biasAngularRate - _imuState.p_biasAngularRate);
        Eigen::Vector3d deltaBiasAcceleration = imuPos.b_quat_p() * (imuNew.p_biasAcceleration - _imuState.p_biasAcceleration);

        Eigen::Quaterniond attitudeIncrement = _attitudeIncrement * math::expMapQuat(_pAtt_pOmega * deltaBiasAngularRate);
        Eigen::Vector3d velocityIncrement = _velocityIncrement + _pVel_pAccel * deltaBiasAcceleration + _pVel_pOmega * deltaBiasAngularRate;
        Eigen::Vector3d positionIncrement = _positionIncrement + _pPos_pAccel * deltaBiasAcceleration + _pPos_pOmega * deltaBiasAngularRate;

        auto lla_positionOld = trafo::ecef2lla_WGS84(pvaOld.position);
        Eigen::Vector3<T> gravitation = n_calcGravitation(lla_positionOld, _gravitationModel);
        if (_integrationFrame == IntegrationFrame::ECEF)
        {
            gravitation = trafo::e_Quat_n(lla_positionOld(0), lla_positionOld(1)) * gravitation;
        }
        // LOG_DATA("{}:   gravitation = {} (vel = {}, pos = {})", nameId, gravitation.transpose(), (gravitation * dt).transpose(), (0.5 * gravitation * dt * dt).transpose());

        PVAState<T> pvaNew;
        pvaNew.attitude = pvaOld.attitude * attitudeIncrement;
        pvaNew.velocity = pvaOld.velocity
                          + pvaOld.attitude * velocityIncrement
                          + gravitation * _timeIncrement;
        pvaNew.position = pvaOld.position
                          + pvaOld.velocity * _timeIncrement
                          + pvaOld.attitude * positionIncrement
                          + 0.5 * gravitation * _timeIncrement * _timeIncrement;

        return pvaNew;
    }

    /// @brief Get the Covariance Matrix of the preintegrated measurements (3x attitude, 3x velocity, 3x position)
    [[nodiscard]] const Eigen::Matrix9d& getCovMatrix() const;

    /// @brief Returns the selected integration frame
    [[nodiscard]] IntegrationFrame getIntegrationFrame() const;

  private:
    /// Frame to integrate the observations in
    IntegrationFrame _integrationFrame = IntegrationFrame::ECEF;

    /// Gravity Model to use
    GravitationModel _gravitationModel = GravitationModel::EGM96;

    /// Wether to lock the integration frame
    bool _lockIntegrationFrame = false;

    // #########################################################################################################################################

    double _timeIncrement = 0.0;                                            ///< Time accumulated for each increment
    Eigen::Quaterniond _attitudeIncrement = Eigen::Quaterniond::Identity(); ///< Relative motion increment for the attitude bi_q_bj
    Eigen::Vector3d _velocityIncrement = Eigen::Vector3d::Zero();           ///< Relative motion increment for the velocity
    Eigen::Vector3d _positionIncrement = Eigen::Vector3d::Zero();           ///< Relative motion increment for the position

    Eigen::Matrix9d _covMatrix = Eigen::Matrix9d::Zero(); ///< Incremental covariance matrix (3x attitude, 3x velocity, 3x position)

    Eigen::Matrix3d _pAtt_pOmega; ///< Partial derivative of the attitude increment after the angular rate bias
    Eigen::Matrix3d _pVel_pAccel; ///< Partial derivative of the velocity increment after the acceleration bias
    Eigen::Matrix3d _pVel_pOmega; ///< Partial derivative of the velocity increment after the angular rate bias
    Eigen::Matrix3d _pPos_pAccel; ///< Partial derivative of the position increment after the acceleration bias
    Eigen::Matrix3d _pPos_pOmega; ///< Partial derivative of the position increment after the angular rate bias

    ImuState<double> _imuState; ///< IMU state used to calculate the increments

    std::optional<Eigen::Matrix6d> _imuNoiseScale_W; ///< Noise scale matrix of the IMU

    // #########################################################################################################################################

    // Forward declaring external function
    friend bool InertialPreIntegratorGui(const char* label, InertialPreIntegrator& integrator, float width);
    friend void to_json(json& j, const InertialPreIntegrator& data);
    friend void from_json(const json& j, InertialPreIntegrator& data);
    friend const char* to_string(InertialPreIntegrator::IntegrationFrame frame);
};

/// @brief Shows a GUI for advanced configuration of the InertialPreIntegrator
/// @param[in] label Label to show. This has to be a unique id for ImGui.
/// @param[in] integrator Reference to the integrator to configure
/// @param[in] width Width of the widget
bool InertialPreIntegratorGui(const char* label, InertialPreIntegrator& integrator, float width = 250.0F);

/// @brief Write info to a json object
/// @param[out] j Json output
/// @param[in] data Object to read info from
void to_json(json& j, const InertialPreIntegrator& data);
/// @brief Read info from a json object
/// @param[in] j Json variable to read info from
/// @param[out] data Output object
void from_json(const json& j, InertialPreIntegrator& data);

/// @brief Converts the enum to a string
/// @param[in] frame Enum value to convert into text
/// @return String representation of the enum
const char* to_string(InertialPreIntegrator::IntegrationFrame frame);

} // namespace NAV
