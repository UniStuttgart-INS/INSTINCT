// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file AttitudeModel.hpp
/// @brief Attitude System Model
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2025-03-07

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <imgui.h>
#include <Eigen/Core>

#include "Navigation/GNSS/SystemModel/Units.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Units.hpp"
#include "util/Container/KeyedMatrix.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "MotionModel.hpp"

namespace NAV
{

/// Attitude System Model
template<typename StateKeyType>
class AttitudeModel
{
  public:
    /// @brief Initializes the attitude model
    void initialize()
    {
        for (int i = 0; i < _gui_covarianceAngularVelocity.rows(); i++)
        {
            _covarianceAngularVelocity(i) = convertUnit(_gui_covarianceAngularVelocity(i), _gui_covarianceAngularVelocityUnit);
        }
    }

    /// @brief Updates the provided Phi, Q and G matrix
    /// @param[in, out] Phi State transition matrix
    /// @param[in, out] Q System/Process noise covariance matrix
    /// @param[in] dt Time step size in [s]
    /// @param[in] latitude Latitude [rad]
    /// @param[in] longitude Longitude [rad]
    /// @param[in] n_quat_b Quaternion body to navigation frame
    template<typename Scalar, int Size, typename Derived>
    void updatePhiAndQ(KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& Phi,
                       KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& Q,
                       double dt,
                       const double& latitude,
                       const double& longitude,
                       const Eigen::QuaternionBase<Derived>& n_quat_b)
    {
        Phi.template block<4>(Att, Att).setIdentity();
        Q.template block<4>(Att, Att) = calcProcessNoiseMatrix(dt, latitude, longitude, n_quat_b)(all, all);
    }

    /// @brief Calculates the state transition matrix (𝚽) and the process noise covariance matrix (𝐐)
    /// @param[in] dt Time step size in [s]
    /// @param[in] latitude Latitude [rad]
    /// @param[in] longitude Longitude [rad]
    /// @param[in] n_quat_b Quaternion body to navigation frame
    /// @return Phi and Q matrix
    template<typename Derived>
    [[nodiscard]] std::pair<KeyedMatrix4d<StateKeyType>, KeyedMatrix4d<StateKeyType>>
        calcPhiAndQ(double dt, const double& latitude, const double& longitude, const Eigen::QuaternionBase<Derived>& n_quat_b)
    {
        KeyedMatrix4d<StateKeyType> Phi(Eigen::Matrix4d::Zero(), Att, Att);
        KeyedMatrix4d<StateKeyType> Q(Eigen::Matrix4d::Zero(), Att, Att);
        updatePhiAndQ(Phi, Q, dt, latitude, longitude, n_quat_b);

        return { Phi, Q };
    }

    /// @brief Shows a GUI
    /// @param[in] itemWidth Width of the space for the config items
    /// @param[in] unitWidth Width of the units
    /// @param[in] id Unique id for ImGui
    /// @return True if something was changed
    bool ShowGui(float itemWidth, float unitWidth, const char* id)
    {
        bool changed = false;

        if (gui::widgets::InputDouble3LWithUnit(fmt::format("{} of the angular velocity due to user motion (Roll, Pitch, Yaw)##{}",
                                                            _gui_covarianceAngularVelocityUnit == Units::CovarianceAngularVelocityUnits::deg_sqrts
                                                                    || _gui_covarianceAngularVelocityUnit == Units::CovarianceAngularVelocityUnits::degsqrts_min
                                                                    || _gui_covarianceAngularVelocityUnit == Units::CovarianceAngularVelocityUnits::rad_sqrts
                                                                ? "StdDev"
                                                                : "Variance",
                                                            id)
                                                    .c_str(),
                                                itemWidth, unitWidth, _gui_covarianceAngularVelocity.data(), 0.0, std::numeric_limits<double>::max(),
                                                _gui_covarianceAngularVelocityUnit, MakeComboItems<Units::CovarianceAngularVelocityUnits>().c_str(),
                                                "%.1e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _gui_covarianceAngularVelocity changed to {}", id, _gui_covarianceAngularVelocity.transpose());
            LOG_DEBUG("{}: _gui_covarianceAngularVelocityUnit changed to {}", id, to_string(_gui_covarianceAngularVelocityUnit));
            changed = true;
        }

        return changed;
    }

  private:
    /// @brief All attitude keys
    const std::array<StateKeyType, 4>& Att = Keys::Att<StateKeyType>;

    /// @brief Calculates the process noise matrix Q
    /// @param[in] dt Time step [s]
    /// @param[in] latitude Latitude [rad]
    /// @param[in] longitude Longitude [rad]
    /// @param[in] n_quat_b Quaternion body to navigation frame
    template<typename Derived>
    [[nodiscard]] KeyedMatrix4d<StateKeyType>
        calcProcessNoiseMatrix(double dt, double latitude, double longitude, const Eigen::QuaternionBase<Derived>& n_quat_b) const
    {
        // Scaling matrix in n-frame
        Eigen::Matrix3d S_n = _covarianceAngularVelocity.asDiagonal();

        Eigen::Matrix4d n_covQuat_b = trafo::covRPY2quat(S_n, n_quat_b);
        Eigen::Matrix4d e_J_n = trafo::covQuat2QuatJacobian(trafo::e_Quat_n(latitude, longitude));

        // Scaling matrix in e-frame
        Eigen::Matrix4d S_e = e_J_n * n_covQuat_b * e_J_n.transpose();

        return { dt * S_e, Att, Att };
    }

    /// Gui selection for the Unit of the input covarianceAngularVelocity
    Units::CovarianceAngularVelocityUnits _gui_covarianceAngularVelocityUnit = Units::CovarianceAngularVelocityUnits::deg_sqrts;
    /// @brief GUI selection for the Standard deviation of the angular velocity due to user motion (Roll, Pitch, Yaw)
    Eigen::Vector3d _gui_covarianceAngularVelocity = { 100.0, 100.0, 100.0 } /* [ deg / √(s) ] */;

    /// @brief Covariance of the angular velocity due to user motion (Roll, Pitch, Yaw) [rad²/s]
    Eigen::Vector3d _covarianceAngularVelocity;

    /// @brief Converts the provided data into a json object
    /// @param[out] j Json object which gets filled with the info
    /// @param[in] data Data to convert into json
    friend void to_json(json& j, const AttitudeModel& data)
    {
        j = {
            { "covarianceAngularVelocityUnit", data._gui_covarianceAngularVelocityUnit },
            { "covarianceAngularVelocity", data._gui_covarianceAngularVelocity },
        };
    }
    /// @brief Converts the provided json object into the data object
    /// @param[in] j Json object with the needed values
    /// @param[out] data Object to fill from the json
    friend void from_json(const json& j, AttitudeModel& data)
    {
        if (j.contains("covarianceAngularVelocityUnit")) { j.at("covarianceAngularVelocityUnit").get_to(data._gui_covarianceAngularVelocityUnit); }
        if (j.contains("covarianceAngularVelocity")) { j.at("covarianceAngularVelocity").get_to(data._gui_covarianceAngularVelocity); }
    }
};

} // namespace NAV