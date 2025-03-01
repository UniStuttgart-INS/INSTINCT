// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file Pos.hpp
/// @brief Position Storage Class
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2021-10-27

#pragma once

#include <cstdint>
#include <Eigen/src/Core/MatrixBase.h>

#include "NodeData/NodeData.hpp"
#include "Navigation/GNSS/SystemModel/MotionModel.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "util/Logger/CommonLog.hpp"
#include "util/Container/KeyedMatrix.hpp"
#include "util/Assert.h"

namespace NAV
{
/// Position Storage Class
class Pos : public NodeData
{
  public:
    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] static std::string type()
    {
        return "Pos";
    }

    /// @brief Returns the type of the data class
    /// @return The data type
    [[nodiscard]] std::string getType() const override { return type(); }

    /// @brief Returns the parent types of the data class
    /// @return The parent data types
    [[nodiscard]] static std::vector<std::string> parentTypes()
    {
        return { NodeData::type() };
    }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] static std::vector<std::string> GetStaticDataDescriptors()
    {
        return {
            "Latitude [deg]",
            "Longitude [deg]",
            "Altitude [m]",
            "North/South [m]",
            "East/West [m]",
            "X-ECEF [m]",
            "Y-ECEF [m]",
            "Z-ECEF [m]",
            "X-ECEF StDev [m]",
            "Y-ECEF StDev [m]",
            "Z-ECEF StDev [m]",
            "XY-ECEF StDev [m]",
            "XZ-ECEF StDev [m]",
            "YZ-ECEF StDev [m]",
            "North StDev [m]",
            "East StDev [m]",
            "Down StDev [m]",
            "NE StDev [m]",
            "ND StDev [m]",
            "ED StDev [m]",
        };
    }

    /// @brief Get the amount of descriptors
    [[nodiscard]] static constexpr size_t GetStaticDescriptorCount() { return 20; }

    /// @brief Returns a vector of data descriptors
    [[nodiscard]] std::vector<std::string> staticDataDescriptors() const override { return GetStaticDataDescriptors(); }

    /// @brief Get the amount of descriptors
    [[nodiscard]] size_t staticDescriptorCount() const override { return GetStaticDescriptorCount(); }

    /// @brief Get the value at the index
    /// @param idx Index corresponding to data descriptor order
    /// @return Value if in the observation
    [[nodiscard]] std::optional<double> getValueAt(size_t idx) const override
    {
        INS_ASSERT(idx < GetStaticDescriptorCount());
        switch (idx)
        {
        case 0: // Latitude [deg]
            return rad2deg(latitude());
        case 1: // Longitude [deg]
            return rad2deg(longitude());
        case 2: // Altitude [m]
            return altitude();
        case 3: // North/South [m]
            return CommonLog::calcLocalPosition(lla_position()).northSouth;
        case 4: // East/West [m]
            return CommonLog::calcLocalPosition(lla_position()).eastWest;
        case 5: // X-ECEF [m]
            return e_position().x();
        case 6: // Y-ECEF [m]
            return e_position().y();
        case 7: // Z-ECEF [m]
            return e_position().z();
        case 8: // X-ECEF StDev [m]
            if (auto stDev = e_positionStdev()) { return stDev->x(); }
            break;
        case 9: // Y-ECEF StDev [m]
            if (auto stDev = e_positionStdev()) { return stDev->y(); }
            break;
        case 10: // Z-ECEF StDev [m]
            if (auto stDev = e_positionStdev()) { return stDev->z(); }
            break;
        case 11: // XY-ECEF StDev [m]
            if (_e_covarianceMatrix) { return std::sqrt(std::abs((*_e_covarianceMatrix)(Keys::PosX, Keys::PosY))); }
            break;
        case 12: // XZ-ECEF StDev [m]
            if (_e_covarianceMatrix) { return std::sqrt(std::abs((*_e_covarianceMatrix)(Keys::PosX, Keys::PosZ))); }
            break;
        case 13: // YZ-ECEF StDev [m]
            if (_e_covarianceMatrix) { return std::sqrt(std::abs((*_e_covarianceMatrix)(Keys::PosY, Keys::PosZ))); }
            break;
        case 14: // North StDev [m]
            if (auto stDev = n_positionStdev()) { return stDev->x(); }
            break;
        case 15: // East StDev [m]
            if (auto stDev = n_positionStdev()) { return stDev->y(); }
            break;
        case 16: // Down StDev [m]
            if (auto stDev = n_positionStdev()) { return stDev->z(); }
            break;
        case 17: // NE StDev [m]
            if (_n_covarianceMatrix) { return std::sqrt(std::abs((*_n_covarianceMatrix)(Keys::PosX, Keys::PosY))); }
            break;
        case 18: // ND StDev [m]
            if (_n_covarianceMatrix) { return std::sqrt(std::abs((*_n_covarianceMatrix)(Keys::PosX, Keys::PosZ))); }
            break;
        case 19: // ED StDev [m]
            if (_n_covarianceMatrix) { return std::sqrt(std::abs((*_n_covarianceMatrix)(Keys::PosY, Keys::PosZ))); }
            break;
        default:
            return std::nullopt;
        }
        return std::nullopt;
    }

    /// @brief Set the value at the index
    /// @param idx Index corresponding to data descriptor order
    /// @param value Value to set
    /// @return True if the value was updated
    [[nodiscard]] bool setValueAt(size_t idx, double value) override
    {
        INS_ASSERT(idx < GetStaticDescriptorCount());
        switch (idx)
        {
        case 0: // Latitude [deg]
            _lla_position(0) = deg2rad(value);
            _e_position = trafo::lla2ecef_WGS84(_lla_position);
            break;
        case 1: // Longitude [deg]
            _lla_position(1) = deg2rad(value);
            _e_position = trafo::lla2ecef_WGS84(_lla_position);
            break;
        case 2: // Altitude [m]
            _lla_position(2) = value;
            _e_position = trafo::lla2ecef_WGS84(_lla_position);
            break;
        case 3: // North/South [m]
            _e_position += e_Quat_n() * Eigen::Vector3d(value, 0.0, 0.0);
            _e_position = trafo::lla2ecef_WGS84(_lla_position);
            break;
        case 4: // East/West [m]
            _e_position += e_Quat_n() * Eigen::Vector3d(0.0, value, 0.0);
            _e_position = trafo::lla2ecef_WGS84(_lla_position);
            break;
        case 5: // X-ECEF [m]
            _e_position(0) = value;
            _lla_position = trafo::ecef2lla_WGS84(_e_position);
            break;
        case 6: // Y-ECEF [m]
            _e_position(1) = value;
            _lla_position = trafo::ecef2lla_WGS84(_e_position);
            break;
        case 7: // Z-ECEF [m]
            _e_position(2) = value;
            _lla_position = trafo::ecef2lla_WGS84(_e_position);
            break;
        case 8:  // X-ECEF StDev [m]
        case 9:  // Y-ECEF StDev [m]
        case 10: // Z-ECEF StDev [m]
        case 11: // XY-ECEF StDev [m]
        case 12: // XZ-ECEF StDev [m]
        case 13: // YZ-ECEF StDev [m]
        case 14: // North StDev [m]
        case 15: // East StDev [m]
        case 16: // Down StDev [m]
        case 17: // NE StDev [m]
        case 18: // ND StDev [m]
        case 19: // ED StDev [m]
        default:
            return false;
        }

        return true;
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                           Rotation Quaternions                                           */
    /* -------------------------------------------------------------------------------------------------------- */

    /// @brief Returns the Quaternion from navigation to Earth-fixed frame
    /// @return The Quaternion for the rotation from navigation to earth coordinates
    [[nodiscard]] Eigen::Quaterniond e_Quat_n() const
    {
        return trafo::e_Quat_n(latitude(), longitude());
    }

    /// @brief Returns the Quaternion from Earth-fixed frame to navigation
    /// @return The Quaternion for the rotation from earth navigation coordinates
    [[nodiscard]] Eigen::Quaterniond n_Quat_e() const
    {
        return e_Quat_n().conjugate();
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                                 Position                                                 */
    /* -------------------------------------------------------------------------------------------------------- */

    /// Returns the latitude 𝜙, longitude λ and altitude (height above ground) in [rad, rad, m]
    [[nodiscard]] const Eigen::Vector3d& lla_position() const { return _lla_position; }

    /// Returns the latitude 𝜙 in [rad]
    [[nodiscard]] const double& latitude() const { return lla_position()(0); }

    /// Returns the longitude λ in [rad]
    [[nodiscard]] const double& longitude() const { return lla_position()(1); }

    /// Returns the altitude (height above ground) in [m]
    [[nodiscard]] const double& altitude() const { return lla_position()(2); }

    /// Returns the  coordinates in [m]
    [[nodiscard]] const Eigen::Vector3d& e_position() const { return _e_position; }

    /// Returns the standard deviation of the position in ECEF frame coordinates in [m]
    [[nodiscard]] std::optional<Eigen::Vector3d> e_positionStdev() const
    {
        if (_e_covarianceMatrix)
        {
            return (*_e_covarianceMatrix)(Keys::Pos<Keys::MotionModelKey>, Keys::Pos<Keys::MotionModelKey>).diagonal().cwiseSqrt();
        }
        return std::nullopt;
    }

    /// Returns the standard deviation of the position in local navigation frame coordinates in [m]
    [[nodiscard]] std::optional<Eigen::Vector3d> n_positionStdev() const
    {
        if (_n_covarianceMatrix)
        {
            return (*_n_covarianceMatrix)(Keys::Pos<Keys::MotionModelKey>, Keys::Pos<Keys::MotionModelKey>).diagonal().cwiseSqrt();
        }
        return std::nullopt;
    }

    /// Returns the Covariance matrix in ECEF frame
    [[nodiscard]] std::optional<std::reference_wrapper<const KeyedMatrixXd<Keys::MotionModelKey, Keys::MotionModelKey>>> e_CovarianceMatrix() const { return _e_covarianceMatrix; }

    /// Returns the Covariance matrix in local navigation frame
    [[nodiscard]] std::optional<std::reference_wrapper<const KeyedMatrixXd<Keys::MotionModelKey, Keys::MotionModelKey>>> n_CovarianceMatrix() const { return _n_covarianceMatrix; }

    // ###########################################################################################################
    //                                                  Setter
    // ###########################################################################################################

    /// @brief Set the Position in  coordinates
    /// @param[in] e_position New Position in ECEF coordinates
    template<typename Derived>
    void setPosition_e(const Eigen::MatrixBase<Derived>& e_position)
    {
        _e_position = e_position;
        _lla_position = trafo::ecef2lla_WGS84(e_position);
    }

    /// @brief Set the Position lla object
    /// @param[in] lla_position New Position in LatLonAlt coordinates
    template<typename Derived>
    void setPosition_lla(const Eigen::MatrixBase<Derived>& lla_position)
    {
        _e_position = trafo::lla2ecef_WGS84(lla_position);
        _lla_position = lla_position;
    }

    /// @brief Set the position and the covariance matrix
    /// @param[in] e_position New Position in ECEF coordinates
    /// @param[in] e_covarianceMatrix 3x3 Pos Error variance
    template<typename DerivedP, typename Derived>
    void setPositionAndCov_e(const Eigen::MatrixBase<DerivedP>& e_position, const Eigen::MatrixBase<Derived>& e_covarianceMatrix)
    {
        setPosition_e(e_position);
        setPosCovarianceMatrix_e(e_covarianceMatrix);
    }

    /// @brief Set the position and the covariance matrix
    /// @param[in] lla_position New Position in LatLonAlt coordinates [rad, rad, m]
    /// @param[in] n_covarianceMatrix 3x3 Pos Error variance in NED frame [m]
    template<typename DerivedP, typename Derived>
    void setPositionAndCov_n(const Eigen::MatrixBase<DerivedP>& lla_position, const Eigen::MatrixBase<Derived>& n_covarianceMatrix)
    {
        setPosition_lla(lla_position);
        setPosCovarianceMatrix_n(n_covarianceMatrix);
    }

    /// @brief Set the Covariance matrix in ECEF coordinates
    /// @param[in] e_covarianceMatrix 3x3 Pos Error variance
    /// @attention Position has to be set before calling this
    template<typename Derived>
    void setPosCovarianceMatrix_e(const Eigen::MatrixBase<Derived>& e_covarianceMatrix)
    {
        INS_ASSERT_USER_ERROR(e_covarianceMatrix.rows() == 3, "This function needs a 3x3 matrix as input");
        INS_ASSERT_USER_ERROR(e_covarianceMatrix.cols() == 3, "This function needs a 3x3 matrix as input");

        _e_covarianceMatrix = KeyedMatrixXd<Keys::MotionModelKey, Keys::MotionModelKey>(
            e_covarianceMatrix, Keys::Pos<Keys::MotionModelKey>);

        Eigen::Matrix3d J = n_Quat_e().toRotationMatrix();

        _n_covarianceMatrix = KeyedMatrixXd<Keys::MotionModelKey, Keys::MotionModelKey>(
            J * e_covarianceMatrix * J.transpose(), Keys::Pos<Keys::MotionModelKey>);
    }

    /// @brief Set the Covariance matrix in NED coordinates
    /// @param[in] n_covarianceMatrix 3x3 Pos Error variance
    /// @attention Position has to be set before calling this
    template<typename Derived>
    void setPosCovarianceMatrix_n(const Eigen::MatrixBase<Derived>& n_covarianceMatrix)
    {
        INS_ASSERT_USER_ERROR(n_covarianceMatrix.rows() == 3, "This function needs a 3x3 matrix as input");
        INS_ASSERT_USER_ERROR(n_covarianceMatrix.cols() == 3, "This function needs a 3x3 matrix as input");

        _n_covarianceMatrix = KeyedMatrixXd<Keys::MotionModelKey, Keys::MotionModelKey>(
            n_covarianceMatrix, Keys::Pos<Keys::MotionModelKey>);

        Eigen::Matrix3d J = e_Quat_n().toRotationMatrix();

        _e_covarianceMatrix = KeyedMatrixXd<Keys::MotionModelKey, Keys::MotionModelKey>(
            J * n_covarianceMatrix * J.transpose(), Keys::Pos<Keys::MotionModelKey>);
    }

    /* -------------------------------------------------------------------------------------------------------- */
    /*                                             Member variables                                             */
    /* -------------------------------------------------------------------------------------------------------- */

  protected:
    /// Covariance matrix in ECEF coordinates
    std::optional<KeyedMatrixXd<Keys::MotionModelKey, Keys::MotionModelKey>> _e_covarianceMatrix;

    /// Covariance matrix in local navigation coordinates
    std::optional<KeyedMatrixXd<Keys::MotionModelKey, Keys::MotionModelKey>> _n_covarianceMatrix;

  private:
    /// Position in ECEF coordinates [m]
    Eigen::Vector3d _e_position{ std::nan(""), std::nan(""), std::nan("") };
    /// Position in LatLonAlt coordinates [rad, rad, m]
    Eigen::Vector3d _lla_position{ std::nan(""), std::nan(""), std::nan("") };
};

} // namespace NAV
