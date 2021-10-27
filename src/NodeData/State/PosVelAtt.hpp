/// @file PosVelAtt.hpp
/// @brief Position, Velocity and Attitude Storage Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-08-21

#pragma once

#include "util/InsTransformations.hpp"

#include "util/Eigen.hpp"
#include "NodeData/State/PosVel.hpp"

namespace NAV
{
/// Position, Velocity and Attitude Storage Class
class PosVelAtt : public PosVel
{
  public:
    /// @brief Default constructor
    PosVelAtt() = default;
    /// @brief Destructor
    ~PosVelAtt() override = default;
    /// @brief Copy constructor
    PosVelAtt(const PosVelAtt&) = default;
    /// @brief Move constructor
    PosVelAtt(PosVelAtt&&) = default;
    /// @brief Copy assignment operator
    PosVelAtt& operator=(const PosVelAtt&) = default;
    /// @brief Move assignment operator
    PosVelAtt& operator=(PosVelAtt&&) = default;

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return std::string("PosVelAtt");
    }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { PosVel::type() };
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                           Rotation Quaternions                                           */
    /* -------------------------------------------------------------------------------------------------------- */

    /// @brief Returns the Quaternion from body to navigation frame (NED)
    /// @return The Quaternion for the rotation from body to navigation coordinates
    [[nodiscard]] const Eigen::Quaterniond& quaternion_nb() const
    {
        return q_nb;
    }

    /// @brief Returns the Quaternion from navigation to body frame (NED)
    /// @return The Quaternion for the rotation from navigation to body coordinates
    [[nodiscard]] Eigen::Quaterniond quaternion_bn() const
    {
        return quaternion_nb().conjugate();
    }

    /// @brief Returns the Quaternion from body to Earth-fixed frame
    /// @return The Quaternion for the rotation from body to earth coordinates
    [[nodiscard]] const Eigen::Quaterniond& quaternion_eb() const
    {
        return q_eb;
    }

    /// @brief Returns the Quaternion from Earth-fixed to body frame
    /// @return The Quaternion for the rotation from earth to body coordinates
    [[nodiscard]] Eigen::Quaterniond quaternion_be() const
    {
        return quaternion_eb().conjugate();
    }

    /// @brief Returns the Roll, Pitch and Yaw angles in [rad]
    /// @return [roll, pitch, yaw]^T
    [[nodiscard]] Eigen::Vector3d rollPitchYaw() const
    {
        // Eigen::Matrix3d DCMBodyToNED = quaternion_nb().toRotationMatrix();
        // Eigen::Vector3d EulerAngles = Eigen::Vector3d::Zero();

        // EulerAngles(1) = -asin(DCMBodyToNED(2, 0));
        // if (fabs(trafo::rad2deg(EulerAngles(1))) < 89.0)
        // {
        //     EulerAngles(0) = atan2((DCMBodyToNED(2, 1) / cos(EulerAngles(1))), (DCMBodyToNED(2, 2) / cos(EulerAngles(1))));

        //     EulerAngles(2) = atan2((DCMBodyToNED(1, 0) / cos(EulerAngles(1))), (DCMBodyToNED(0, 0) / cos(EulerAngles(1))));
        // }
        // else
        // {
        //     EulerAngles(0) = 0.0;
        //     EulerAngles(2) = 0.0;
        // }

        // return EulerAngles;
        return trafo::quat2eulerZYX(quaternion_nb());
    }

    // ###########################################################################################################
    //                                                  Setter
    // ###########################################################################################################

    /// @brief Set the Quaternion from body to earth frame
    /// @param[in] quat_eb Quaternion from body to earth frame
    void setAttitude_eb(const Eigen::Quaterniond& quat_eb)
    {
        q_eb = quat_eb;
        q_nb = quaternion_ne() * quat_eb;
    }

    /// @brief Set the Quaternion from body to navigation frame
    /// @param[in] quat_nb Quaternion from body to navigation frame
    void setAttitude_nb(const Eigen::Quaterniond& quat_nb)
    {
        q_eb = quaternion_en() * quat_nb;
        q_nb = quat_nb;
    }

    /// @brief Set the State
    /// @param[in] pos_ecef New Position in ECEF coordinates
    /// @param[in] vel_e The new velocity in the earth frame
    /// @param[in] quat_eb Quaternion from body to earth frame
    void setState_e(const Eigen::Vector3d& pos_ecef, const Eigen::Vector3d& vel_e, const Eigen::Quaterniond& quat_eb)
    {
        setPosition_e(pos_ecef);
        setVelocity_e(vel_e);
        setAttitude_eb(quat_eb);
    }

    /// @brief Set the State
    /// @param[in] pos_lla New Position in LatLonAlt coordinates
    /// @param[in] vel_n The new velocity in the NED frame
    /// @param[in] quat_nb Quaternion from body to navigation frame
    void setState_n(const Eigen::Vector3d& pos_lla, const Eigen::Vector3d& vel_n, const Eigen::Quaterniond& quat_nb)
    {
        setPosition_lla(pos_lla);
        setVelocity_n(vel_n);
        setAttitude_nb(quat_nb);
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                             Member variables                                             */
    /* -------------------------------------------------------------------------------------------------------- */

  private:
    /// Quaternion body to earth frame
    Eigen::Quaterniond q_eb{ 0, 0, 0, 0 };
    /// Quaternion body to navigation frame (roll, pitch, yaw)
    Eigen::Quaterniond q_nb{ 0, 0, 0, 0 };
};

} // namespace NAV
