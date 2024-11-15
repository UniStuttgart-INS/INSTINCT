// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file MotionModel.hpp
/// @brief Motion System Model
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2024-08-20

#pragma once

#include <array>
#include <cmath>
#include "internal/gui/widgets/HelpMarker.hpp"
#include <Eigen/Core>
#include <cstdint>
#include <imgui.h>

#include "Navigation/GNSS/SystemModel/Units.hpp"
#include "Navigation/Math/KalmanFilter.hpp"
#include "Navigation/Math/VanLoan.hpp"
#include "Navigation/GNSS/SystemModel/SystemModel.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Units.hpp"
#include "util/Container/KeyedMatrix.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"

namespace NAV
{

namespace Keys
{

/// Keys used in the model
enum MotionModelKey : uint8_t
{
    PosX,                 ///< Position ECEF_X [m]
    PosY,                 ///< Position ECEF_Y [m]
    PosZ,                 ///< Position ECEF_Z [m]
    VelX,                 ///< Velocity ECEF_X [m/s]
    VelY,                 ///< Velocity ECEF_Y [m/s]
    VelZ,                 ///< Velocity ECEF_Z [m/s]
    MotionModelKey_COUNT, ///< Count
};

/// @brief All position keys
template<typename StateKeyType>
const std::vector<StateKeyType> Pos = { Keys::PosX, Keys::PosY, Keys::PosZ };
/// @brief All velocity keys
template<typename StateKeyType>
const std::vector<StateKeyType> Vel = { Keys::VelX, Keys::VelY, Keys::VelZ };
/// @brief Vector with all position and velocity state keys
template<typename StateKeyType>
const std::vector<StateKeyType> PosVel = { Keys::PosX, Keys::PosY, Keys::PosZ,
                                           Keys::VelX, Keys::VelY, Keys::VelZ };

} // namespace Keys

/// Motion System Model
template<typename StateKeyType>
class MotionModel
{
  public:
    /// @brief Initializes the motion model
    /// @param[in, out] F System model matrix
    /// @param[in, out] W Noise scale matrix
    template<typename Scalar, int Size>
    void initialize(KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& F,
                    KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& W)
    {
        for (size_t i = 0; i < _gui_covarianceAccel.size(); i++)
        {
            // Covariance of the acceleration 𝜎_a due to user motion in horizontal and vertical component [m²/s³]
            _covarianceAccel.at(i) = convertUnit(_gui_covarianceAccel.at(i), _gui_covarianceAccelUnit);
        }

        F.template block<3>(Pos, Vel) = Eigen::Matrix3d::Identity();
        W.template block<3>(Vel, Vel) = Eigen::DiagonalMatrix<double, 3>(_covarianceAccel[0], _covarianceAccel[0], _covarianceAccel[1]);
    }

    /// @brief Updates the provided Phi, Q and G matrix
    /// @param[in, out] Phi State transition matrix
    /// @param[in, out] Q System/Process noise covariance matrix
    /// @param[in, out] G Noise input matrix
    /// @param[in] F System model matrix
    /// @param[in] W Noise scale matrix
    /// @param[in] dt Time step size in [s]
    /// @param[in] latitude Latitude [rad]
    /// @param[in] longitude Longitude [rad]
    /// @param[in] algorithm Algorithm to use for the calculation
    template<typename Scalar, int Size>
    void updatePhiAndQ(KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& Phi,
                       KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& Q,
                       KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& G,
                       const KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& F,
                       const KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& W,
                       double dt,
                       const double& latitude,
                       const double& longitude,
                       SystemModelCalcAlgorithm algorithm)
    {
        if (algorithm == SystemModelCalcAlgorithm::VanLoan)
        {
            G.template block<3>(Vel, Vel) = trafo::e_Quat_n(latitude, longitude).toRotationMatrix();
            auto [PhiMot, QMot] = NAV::calcPhiAndQWithVanLoanMethod(
                F.template block<6>(PosVel, PosVel),
                G.template block<6>(PosVel, PosVel),
                W.template block<6>(PosVel, PosVel),
                dt);
            Phi.template block<6>(PosVel, PosVel) = PhiMot;
            Q.template block<6>(PosVel, PosVel) = QMot;
        }
        else // QCalculationAlgorithm::Taylor1
        {
            Phi.template block<6>(PosVel, PosVel) = transitionMatrix_Phi_Taylor(F.template block<6>(PosVel, PosVel), dt, 1);
            Q.template block<6>(PosVel, PosVel) = calcProcessNoiseMatrixTaylor(dt, latitude, longitude)(all, all);
        }
    }

    /// @brief Calculates the state transition matrix (𝚽) and the process noise covariance matrix (𝐐)
    /// @param[in] dt Time step size in [s]
    /// @param[in] latitude Latitude [rad]
    /// @param[in] longitude Longitude [rad]
    /// @param[in] algorithm Algorithm to use for the calculation
    /// @return Phi and Q matrix
    [[nodiscard]] std::pair<KeyedMatrix6d<StateKeyType>, KeyedMatrix6d<StateKeyType>>
        calcPhiAndQ(double dt, const double& latitude, const double& longitude, SystemModelCalcAlgorithm algorithm)
    {
        KeyedMatrix6d<StateKeyType> F(Eigen::Matrix6d::Zero(), PosVel, PosVel);
        KeyedMatrix6d<StateKeyType> G(Eigen::Matrix6d::Zero(), PosVel, PosVel);
        KeyedMatrix6d<StateKeyType> W(Eigen::Matrix6d::Zero(), PosVel, PosVel);
        initialize(F, W);

        KeyedMatrix6d<StateKeyType> Phi(Eigen::Matrix6d::Zero(), PosVel, PosVel);
        KeyedMatrix6d<StateKeyType> Q(Eigen::Matrix6d::Zero(), PosVel, PosVel);
        updatePhiAndQ(Phi, Q, G, F, W, dt, latitude, longitude, algorithm);

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

        if (gui::widgets::InputDouble2WithUnit(fmt::format("{} of the acceleration due to user motion (Hor/Ver)##{}",
                                                           _gui_covarianceAccelUnit == Units::CovarianceAccelUnits::m_sqrts3
                                                               ? "StdDev"
                                                               : "Variance",
                                                           id)
                                                   .c_str(),
                                               itemWidth, unitWidth, _gui_covarianceAccel.data(), reinterpret_cast<int*>(&_gui_covarianceAccelUnit),
                                               MakeComboItems<Units::CovarianceAccelUnits>().c_str(), //"m/√(s^3)\0m^2/s^3\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _gui_covarianceAccel changed to {}", id, fmt::join(_gui_covarianceAccel.begin(), _gui_covarianceAccel.end(), ", "));
            LOG_DEBUG("{}: _gui_covarianceAccelUnit changed to {}", id, to_string(_gui_covarianceAccelUnit));
            changed = true;
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker(fmt::format("Suitable values for the horizontal acceleration are around\n"
                                             "- 1 {} for a pedestrian or ship,\n"
                                             "- {} {} for a car, and\n"
                                             "- {} {} for a military aircraft.\n"
                                             "The vertical acceleration PSD is usually smaller.",
                                             to_string(_gui_covarianceAccelUnit),
                                             _gui_covarianceAccelUnit == Units::CovarianceAccelUnits::m_sqrts3 ? 3 : 10,
                                             to_string(_gui_covarianceAccelUnit),
                                             _gui_covarianceAccelUnit == Units::CovarianceAccelUnits::m_sqrts3 ? 10 : 100,
                                             to_string(_gui_covarianceAccelUnit))
                                     .c_str());

        return changed;
    }

  private:
    /// @brief All position keys
    const std::vector<StateKeyType>& Pos = Keys::Pos<StateKeyType>;
    /// @brief All velocity keys
    const std::vector<StateKeyType>& Vel = Keys::Vel<StateKeyType>;
    /// @brief All position and velocity keys
    const std::vector<StateKeyType>& PosVel = Keys::PosVel<StateKeyType>;

    /// @brief Calculates the process noise matrix Q
    /// @param[in] dt Time step [s]
    /// @param[in] latitude Latitude [rad]
    /// @param[in] longitude Longitude [rad]
    /// @note See \cite Groves2013 Groves, ch. 9.4.2.2, eq. 9.152, p. 417
    [[nodiscard]] KeyedMatrix6d<StateKeyType>
        calcProcessNoiseMatrixTaylor(double dt, double latitude, double longitude) const
    {
        // Scaling matrix in n-frame
        Eigen::Matrix3d a_S_n = Eigen::DiagonalMatrix<double, 3>(_covarianceAccel[0],
                                                                 _covarianceAccel[0],
                                                                 _covarianceAccel[1]);
        // Scaling matrix in e-frame
        Eigen::Matrix3d a_S_e = trafo::e_Quat_n(latitude, longitude) * a_S_n * trafo::n_Quat_e(latitude, longitude);

        double dt2 = std::pow(dt, 2);
        double dt3 = dt2 * dt;

        KeyedMatrix6d<StateKeyType> Q(Eigen::Matrix6d::Zero(), PosVel, PosVel);

        // Groves ch. 9.4.2.2, eq. 9.152, p. 417
        Q.template block<3>(Pos, Pos) = dt3 / 3.0 * a_S_e;
        Q.template block<3>(Pos, Vel) = dt2 / 2.0 * a_S_e;
        Q.template block<3>(Vel, Pos) = Q.template block<3>(Pos, Vel).transpose();
        Q.template block<3>(Vel, Vel) = dt * a_S_e;

        return Q;
    }

    /// Gui selection for the Unit of the input covarianceAccel parameter for the StDev due to acceleration due to user motion
    Units::CovarianceAccelUnits _gui_covarianceAccelUnit = Units::CovarianceAccelUnits::m_sqrts3;
    /// @brief GUI selection for the Standard deviation of the acceleration 𝜎_a due to user motion in horizontal and vertical component
    /// @note See Groves (2013) eq. (9.156)
    std::array<double, 2> _gui_covarianceAccel = { 3.0, 1.5 } /* [ m / √(s^3) ] */;

    /// @brief Covariance of the acceleration 𝜎_a due to user motion in horizontal and vertical component [m²/s³]
    std::array<double, 2> _covarianceAccel = { 3.0, 1.5 };

    /// @brief Converts the provided data into a json object
    /// @param[out] j Json object which gets filled with the info
    /// @param[in] data Data to convert into json
    friend void to_json(json& j, const MotionModel& data)
    {
        j = {
            { "covarianceAccelUnit", data._gui_covarianceAccelUnit },
            { "covarianceAccel", data._gui_covarianceAccel },
        };
    }
    /// @brief Converts the provided json object into the data object
    /// @param[in] j Json object with the needed values
    /// @param[out] data Object to fill from the json
    friend void from_json(const json& j, MotionModel& data)
    {
        if (j.contains("covarianceAccelUnit")) { j.at("covarianceAccelUnit").get_to(data._gui_covarianceAccelUnit); }
        if (j.contains("covarianceAccel")) { j.at("covarianceAccel").get_to(data._gui_covarianceAccel); }
    }
};

} // namespace NAV

/// @brief Stream insertion operator overload
/// @param[in, out] os Output stream object to stream the time into
/// @param[in] obj Object to print
/// @return Returns the output stream object in order to chain stream insertions
std::ostream& operator<<(std::ostream& os, const NAV::Keys::MotionModelKey& obj);

#ifndef DOXYGEN_IGNORE

/// @brief Formatter
template<>
struct fmt::formatter<NAV::Keys::MotionModelKey> : fmt::formatter<const char*>
{
    /// @brief Defines how to format structs
    /// @param[in] state Struct to format
    /// @param[in, out] ctx Format context
    /// @return Output iterator
    template<typename FormatContext>
    auto format(const NAV::Keys::MotionModelKey& state, FormatContext& ctx) const
    {
        using namespace NAV::Keys; // NOLINT(google-build-using-namespace)

        switch (state)
        {
        case PosX:
            return fmt::formatter<const char*>::format("PosX", ctx);
        case PosY:
            return fmt::formatter<const char*>::format("PosY", ctx);
        case PosZ:
            return fmt::formatter<const char*>::format("PosZ", ctx);
        case VelX:
            return fmt::formatter<const char*>::format("VelX", ctx);
        case VelY:
            return fmt::formatter<const char*>::format("VelY", ctx);
        case VelZ:
            return fmt::formatter<const char*>::format("VelZ", ctx);
        case MotionModelKey_COUNT:
            return fmt::formatter<const char*>::format("MotionModelKey_COUNT", ctx);
        }

        return fmt::formatter<const char*>::format("ERROR", ctx);
    }
};

#endif