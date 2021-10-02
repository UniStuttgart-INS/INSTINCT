/// @file PosVelAtt.hpp
/// @brief Position, Velocity and Attitude Storage Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-08-21

#pragma once

#include "util/InsTransformations.hpp"

#include "util/Eigen.hpp"
#include "NodeData/InsObs.hpp"

namespace NAV
{
/// Position, Velocity and Attitude Storage Class
class PosVelAtt : public InsObs
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
        return { InsObs::type() };
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

    /// @brief Returns the Quaternion from navigation to Earth-fixed frame
    /// @return The Quaternion for the rotation from navigation to earth coordinates
    [[nodiscard]] Eigen::Quaterniond quaternion_en() const
    {
        return trafo::quat_en(latitude(), longitude());
    }

    /// @brief Returns the Quaternion from Earth-fixed frame to navigation
    /// @return The Quaternion for the rotation from earth navigation coordinates
    [[nodiscard]] Eigen::Quaterniond quaternion_ne() const
    {
        return quaternion_en().conjugate();
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

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                                 Position                                                 */
    /* -------------------------------------------------------------------------------------------------------- */

    /// Returns the latitude 𝜙, longitude λ and altitude (height above ground) in [rad, rad, m]
    [[nodiscard]] const Eigen::Vector3d& latLonAlt() const { return p_lla; }

    /// Returns the latitude 𝜙 in [rad]
    [[nodiscard]] const double& latitude() const { return latLonAlt()(0); }

    /// Returns the longitude λ in [rad]
    [[nodiscard]] const double& longitude() const { return latLonAlt()(1); }

    /// Returns the altitude (height above ground) in [m]
    [[nodiscard]] const double& altitude() const { return latLonAlt()(2); }

    /// Returns the ECEF coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& position_ecef() const { return p_ecef; }

    /// Returns the local coordinates [m]
    [[nodiscard]] const Eigen::Vector3d& position_n() const { return p_n; }

    /// Returns the reference position in LatLon Alt for the local coordinates [rad, rad, m]
    [[nodiscard]] const Eigen::Vector3d& position_n_ref_lla() const { return p_n_ref_lla; }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                                 Velocity                                                 */
    /* -------------------------------------------------------------------------------------------------------- */

    /// Returns the velocity in [m/s], in earth coordinates
    [[nodiscard]] const Eigen::Vector3d& velocity_e() const { return v_e; }

    /// Returns the velocity in [m/s], in navigation coordinates
    [[nodiscard]] const Eigen::Vector3d& velocity_n() const { return v_n; }

    // ###########################################################################################################
    //                                                  Setter
    // ###########################################################################################################

    /// @brief Set the Position in ecef coordinates
    /// @param[in] pos_ecef New Position in ECEF coordinates
    /// @param[in] pos_n_ref_lla Reference position for the local n System
    void setPosition_e(const Eigen::Vector3d& pos_ecef, const Eigen::Vector3d& pos_n_ref_lla)
    {
        p_ecef = pos_ecef;
        p_lla = trafo::ecef2lla_WGS84(pos_ecef);

        p_n = trafo::ecef2ned(pos_ecef, pos_n_ref_lla);
        p_n_ref_lla = pos_n_ref_lla;
    }
    /// @brief Set the Position lla object
    /// @param[in] pos_lla New Position in LatLonAlt coordinates
    /// @param[in] pos_n_ref_lla Reference position for the local n System
    void setPosition_lla(const Eigen::Vector3d& pos_lla, const Eigen::Vector3d& pos_n_ref_lla)
    {
        p_ecef = trafo::lla2ecef_WGS84(pos_lla);
        p_lla = pos_lla;

        p_n = trafo::ecef2ned(p_ecef, pos_n_ref_lla);
        p_n_ref_lla = pos_n_ref_lla;
    }

    /// @brief Set the Position in local coordinates
    /// @param[in] pos_n New Position in local NED coordinates
    /// @param[in] pos_n_ref_lla Reference position for the local n System
    void setPosition_n(const Eigen::Vector3d& pos_n, const Eigen::Vector3d& pos_n_ref_lla)
    {
        p_ecef = trafo::ned2ecef(pos_n, pos_n_ref_lla);
        p_lla = trafo::ecef2lla_WGS84(p_ecef);

        p_n = pos_n;
        p_n_ref_lla = pos_n_ref_lla;
    }

    /// @brief Set the Velocity in the earth frame
    /// @param[in] vel_e The new velocity in the earth frame
    void setVelocity_e(const Eigen::Vector3d& vel_e)
    {
        v_e = vel_e;
        v_n = quaternion_ne() * vel_e;
    }

    /// @brief Set the Velocity in the NED frame
    /// @param[in] vel_n The new velocity in the NED frame
    void setVelocity_n(const Eigen::Vector3d& vel_n)
    {
        v_e = quaternion_en() * vel_n;
        v_n = vel_n;
    }

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
    /// @param[in] pos_n_ref_lla Reference position for the local n System
    void setState_e(const Eigen::Vector3d& pos_ecef, const Eigen::Vector3d& vel_e, const Eigen::Quaterniond& quat_eb, const Eigen::Vector3d& pos_n_ref_lla)
    {
        setPosition_e(pos_ecef, pos_n_ref_lla);
        setVelocity_e(vel_e);
        setAttitude_eb(quat_eb);
    }

    /// @brief Set the State
    /// @param[in] pos_n New Position in local NED coordinates
    /// @param[in] vel_n The new velocity in the NED frame
    /// @param[in] quat_nb Quaternion from body to navigation frame
    /// @param[in] pos_n_ref_lla Reference position for the local n System
    void setState_n(const Eigen::Vector3d& pos_n, const Eigen::Vector3d& vel_n, const Eigen::Quaterniond& quat_nb, const Eigen::Vector3d& pos_n_ref_lla)
    {
        setPosition_n(pos_n, pos_n_ref_lla);
        setVelocity_n(vel_n);
        setAttitude_nb(quat_nb);
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                             Member variables                                             */
    /* -------------------------------------------------------------------------------------------------------- */

  private:
    /// Position in ECEF coordinates [m]
    Eigen::Vector3d p_ecef{ 0, 0, 0 };
    /// Position in LatLonAlt coordinates [rad, rad, m]
    Eigen::Vector3d p_lla{ 0, 0, 0 };

    /// Position in local NED coordinates [m]
    Eigen::Vector3d p_n{ 0, 0, 0 };
    /// Reference Position for the local NED coordinates [rad, rad, m]
    Eigen::Vector3d p_n_ref_lla{ 0, 0, 0 };

    /// Velocity in earth coordinates [m/s]
    Eigen::Vector3d v_e{ std::nan(""), std::nan(""), std::nan("") };
    /// Velocity in navigation coordinates [m/s]
    Eigen::Vector3d v_n{ std::nan(""), std::nan(""), std::nan("") };

    /// Quaternion body to earth frame
    Eigen::Quaterniond q_eb{ 0, 0, 0, 0 };
    /// Quaternion body to navigation frame (roll, pitch, yaw)
    Eigen::Quaterniond q_nb{ 0, 0, 0, 0 };
};

} // namespace NAV
