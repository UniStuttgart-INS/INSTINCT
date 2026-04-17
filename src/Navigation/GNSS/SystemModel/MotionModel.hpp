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
#include <Eigen/src/Geometry/Quaternion.h>
#include <fmt/ranges.h>

#include "Navigation/GNSS/SystemModel/Units.hpp"
#include "Navigation/Math/KalmanFilter.hpp"
#include "Navigation/Math/VanLoan.hpp"
#include "Navigation/GNSS/SystemModel/SystemModel.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Units.hpp"
#include <iterator>
#include <limits>
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
    AttQ1,                ///< x: Coefficient of i
    AttQ2,                ///< y: Coefficient of j
    AttQ3,                ///< z: Coefficient of k
    AttQ0,                ///< w: Real (scalar) part of the Quaternion
    AccelX,               ///< Acceleration X
    AccelY,               ///< Acceleration Y
    AccelZ,               ///< Acceleration Z
    AngularRateX,         ///< Angular rate X
    AngularRateY,         ///< Angular rate Y
    AngularRateZ,         ///< Angular rate Z
    MotionModelKey_COUNT, ///< Count
};

/// @brief All position keys
template<typename StateKeyType>
constexpr std::array<StateKeyType, 3> Pos = { Keys::PosX, Keys::PosY, Keys::PosZ };
/// @brief All velocity keys
template<typename StateKeyType>
constexpr std::array<StateKeyType, 3> Vel = { Keys::VelX, Keys::VelY, Keys::VelZ };
/// @brief All attitude keys
template<typename StateKeyType>
constexpr std::array<StateKeyType, 4> Att = { Keys::AttQ1, Keys::AttQ2, Keys::AttQ3, Keys::AttQ0 };
/// @brief 3 quaternion keys are used for the angle error
template<typename StateKeyType>
constexpr std::array<StateKeyType, 3> AngleErrorKeys = { Keys::AttQ1, Keys::AttQ2, Keys::AttQ3 };

/// @brief Vector with all position and velocity keys
template<typename StateKeyType>
constexpr std::array<StateKeyType, 6> PosVel = { Keys::PosX, Keys::PosY, Keys::PosZ,
                                                 Keys::VelX, Keys::VelY, Keys::VelZ };
/// @brief Vector with all position velocity and attitude keys
template<typename StateKeyType>
constexpr std::array<StateKeyType, 10> PosVelAtt = { Keys::PosX, Keys::PosY, Keys::PosZ,
                                                     Keys::VelX, Keys::VelY, Keys::VelZ,
                                                     Keys::AttQ1, Keys::AttQ2, Keys::AttQ3, Keys::AttQ0 };

/// @brief All acceleration keys
template<typename StateKeyType>
constexpr std::array<StateKeyType, 3> Accel = { Keys::AccelX, Keys::AccelY, Keys::AccelZ };
/// @brief All angular rate keys
template<typename StateKeyType>
constexpr std::array<StateKeyType, 3> AngularRate = { Keys::AngularRateX, Keys::AngularRateY, Keys::AngularRateZ };

/// @brief Vector with all position, velocity and acceleration keys
template<typename StateKeyType>
constexpr std::array<StateKeyType, 9> PosVelAccel = { Keys::PosX, Keys::PosY, Keys::PosZ,
                                                      Keys::VelX, Keys::VelY, Keys::VelZ,
                                                      Keys::AccelX, Keys::AccelY, Keys::AccelZ };

} // namespace Keys

/// Motion System Model
template<typename StateKeyType>
class MotionModel
{
  public:
    /// @brief Initializes the motion model
    void initialize()
    {
        for (size_t i = 0; i < _gui_covarianceAccel.size(); i++)
        {
            // Covariance of the acceleration 𝜎_a due to user motion in horizontal and vertical component [m²/s³]
            _covarianceAccel.at(i) = convertUnit(_gui_covarianceAccel.at(i), _gui_covarianceAccelUnit);
        }
        for (size_t i = 0; i < _gui_covarianceJerk.size(); i++)
        {
            // Covariance of the jerk 𝜎_j due to user motion in horizontal and vertical component [m^2/s^5]
            _covarianceJerk.at(i) = convertUnit(_gui_covarianceJerk.at(i), _gui_covarianceJerkUnit);
        }
    }

    /// @brief Initializes the motion model
    /// @param[in, out] F System model matrix
    /// @param[in, out] W Noise scale matrix
    template<typename Scalar, int Size>
    void initialize(KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& F,
                    KeyedMatrix<Scalar, StateKeyType, StateKeyType, Size, Size>& W)
    {
        initialize();

        F.template block<3>(Pos, Vel) = Eigen::Matrix3d::Identity();
        W.template block<3>(Vel, Vel) = Eigen::DiagonalMatrix<double, 3>(_covarianceAccel[0], _covarianceAccel[0], _covarianceAccel[1]);
        if (F.hasRows(Accel))
        {
            F.template block<3>(Vel, Accel) = Eigen::Matrix3d::Identity();
            W.template block<3>(Accel, Accel) = Eigen::DiagonalMatrix<double, 3>(_covarianceJerk[0], _covarianceJerk[0], _covarianceJerk[1]);
        }
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
            auto e_quat_n = trafo::e_Quat_n(latitude, longitude).toRotationMatrix();
            G.template block<3>(Vel, Vel) = e_quat_n;

            std::vector<StateKeyType> states;
            if (G.hasRows(Accel))
            {
                G.template block<3>(Accel, Accel) = e_quat_n;
                std::ranges::copy(PosVelAccel, std::back_inserter(states));
            }
            else
            {
                std::ranges::copy(PosVel, std::back_inserter(states));
            }
            auto [PhiMot, QMot] = NAV::calcPhiAndQWithVanLoanMethod(F(states, states), G(states, states), W(states, states), dt);
            Phi(states, states) = PhiMot;
            Q(states, states) = QMot;
        }
        else // QCalculationAlgorithm::Taylor1
        {
            if (G.hasRows(Accel))
            {
                Phi(PosVelAccel, PosVelAccel) = transitionMatrix_Phi_Taylor(F(PosVelAccel, PosVelAccel), dt, 1);
                Q(PosVelAccel, PosVelAccel) = calcProcessNoiseMatrixTaylorWithAccel(dt, latitude, longitude)(all, all);
            }
            else
            {
                Phi(PosVel, PosVel) = transitionMatrix_Phi_Taylor(F(PosVel, PosVel), dt, 1);
                Q(PosVel, PosVel) = calcProcessNoiseMatrixTaylor(dt, latitude, longitude)(all, all);
            }
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

    /// @brief Calculates the state transition matrix (𝚽) and the process noise covariance matrix (𝐐)
    /// @param[in] dt Time step size in [s]
    /// @param[in] latitude Latitude [rad]
    /// @param[in] longitude Longitude [rad]
    /// @param[in] algorithm Algorithm to use for the calculation
    /// @return Phi and Q matrix
    [[nodiscard]] std::pair<KeyedMatrix9d<StateKeyType>, KeyedMatrix9d<StateKeyType>>
        calcPhiAndQWithAccel(double dt, const double& latitude, const double& longitude, SystemModelCalcAlgorithm algorithm)
    {
        KeyedMatrix9d<StateKeyType> F(Eigen::Matrix9d::Zero(), PosVelAccel, PosVelAccel);
        KeyedMatrix9d<StateKeyType> G(Eigen::Matrix9d::Zero(), PosVelAccel, PosVelAccel);
        KeyedMatrix9d<StateKeyType> W(Eigen::Matrix9d::Zero(), PosVelAccel, PosVelAccel);
        initialize(F, W);

        KeyedMatrix9d<StateKeyType> Phi(Eigen::Matrix9d::Zero(), PosVelAccel, PosVelAccel);
        KeyedMatrix9d<StateKeyType> Q(Eigen::Matrix9d::Zero(), PosVelAccel, PosVelAccel);
        updatePhiAndQ(Phi, Q, G, F, W, dt, latitude, longitude, algorithm);

        return { Phi, Q };
    }

    /// @brief Shows a GUI
    /// @param[in] itemWidth Width of the space for the config items
    /// @param[in] unitWidth Width of the units
    /// @param[in] id Unique id for ImGui
    /// @param[in] withJerk Display Jerk options
    /// @return True if something was changed
    bool ShowGui(float itemWidth, float unitWidth, const char* id, bool withJerk = false)
    {
        bool changed = false;

        if (gui::widgets::InputDouble2LWithUnit(fmt::format("{} of the acceleration due to user motion (Hor/Ver)##{}",
                                                            _gui_covarianceAccelUnit == Units::CovarianceAccelUnits::m_sqrts3
                                                                ? "StdDev"
                                                                : "Variance",
                                                            id)
                                                    .c_str(),
                                                itemWidth, unitWidth, _gui_covarianceAccel.data(), 0.0, std::numeric_limits<double>::max(),
                                                _gui_covarianceAccelUnit,
                                                MakeComboItems<Units::CovarianceAccelUnits>().c_str(),
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

        if (withJerk
            && gui::widgets::InputDouble2LWithUnit(fmt::format("{} of the jerk due to user motion (Hor/Ver)##{}",
                                                               _gui_covarianceJerkUnit == Units::CovarianceJerkUnits::m_sqrts5
                                                                   ? "StdDev"
                                                                   : "Variance",
                                                               id)
                                                       .c_str(),
                                                   itemWidth, unitWidth, _gui_covarianceJerk.data(), 0.0, std::numeric_limits<double>::max(),
                                                   _gui_covarianceJerkUnit,
                                                   MakeComboItems<Units::CovarianceJerkUnits>().c_str(),
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: _gui_covarianceJerk changed to {}", id, fmt::join(_gui_covarianceAccel.begin(), _gui_covarianceJerk.end(), ", "));
            LOG_DEBUG("{}: _gui_covarianceJerkUnit changed to {}", id, to_string(_gui_covarianceJerkUnit));
            changed = true;
        }

        return changed;
    }

    /// @brief Whether jerk noise is added (and the acceleration is needed in the state)
    [[nodiscard]] bool hasJerkNoise() const
    {
        return std::ranges::all_of(_gui_covarianceJerk, [](double a) { return a > 1e-12; });
    }

  private:
    /// @brief All position keys
    const std::array<StateKeyType, 3>& Pos = Keys::Pos<StateKeyType>;
    /// @brief All velocity keys
    const std::array<StateKeyType, 3>& Vel = Keys::Vel<StateKeyType>;
    /// @brief All acceleration keys
    const std::array<StateKeyType, 3>& Accel = Keys::Accel<StateKeyType>;
    /// @brief All position and velocity keys
    const std::array<StateKeyType, 6>& PosVel = Keys::PosVel<StateKeyType>;
    /// @brief All position, velocity and acceleration keys
    const std::array<StateKeyType, 9>& PosVelAccel = Keys::PosVelAccel<StateKeyType>;

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

    /// @brief Calculates the process noise matrix Q
    /// @param[in] dt Time step [s]
    /// @param[in] latitude Latitude [rad]
    /// @param[in] longitude Longitude [rad]
    [[nodiscard]] KeyedMatrix9d<StateKeyType>
        calcProcessNoiseMatrixTaylorWithAccel(double dt, double latitude, double longitude) const
    {
        // Scaling matrix in n-frame
        Eigen::Matrix3d Sa_n = Eigen::DiagonalMatrix<double, 3>(_covarianceAccel[0],
                                                                _covarianceAccel[0],
                                                                _covarianceAccel[1]);

        Eigen::Quaterniond e_quat_n = trafo::e_Quat_n(latitude, longitude);
        Eigen::Quaterniond n_quat_e = e_quat_n.conjugate();

        // Scaling matrix in e-frame
        Eigen::Matrix3d Sa_e = e_quat_n * Sa_n * n_quat_e;

        // Scaling matrix in n-frame
        Eigen::Matrix3d Sj_n = Eigen::DiagonalMatrix<double, 3>(_covarianceJerk[0],
                                                                _covarianceJerk[0],
                                                                _covarianceJerk[1]);
        // Scaling matrix in e-frame
        Eigen::Matrix3d Sj_e = e_quat_n * Sj_n * n_quat_e;

        double dt2 = std::pow(dt, 2);
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        double dt5 = dt4 * dt;

        KeyedMatrix9d<StateKeyType> Q(Eigen::Matrix9d::Zero(), PosVelAccel, PosVelAccel);

        Q.template block<3>(Pos, Pos) = dt3 / 3.0 * Sa_e + dt5 / 20.0 * Sj_e;
        Q.template block<3>(Pos, Vel) = dt2 / 2.0 * Sa_e + dt4 / 8.0 * Sj_e;
        Q.template block<3>(Pos, Accel) = dt3 / 6.0 * Sj_e;
        Q.template block<3>(Vel, Pos) = Q.template block<3>(Pos, Vel).transpose();
        Q.template block<3>(Vel, Vel) = dt * Sa_e + dt3 / 3.0 * Sj_e;
        Q.template block<3>(Vel, Accel) = dt2 / 2.0 * Sj_e;
        Q.template block<3>(Accel, Pos) = Q.template block<3>(Pos, Accel).transpose();
        Q.template block<3>(Accel, Vel) = Q.template block<3>(Vel, Accel).transpose();
        Q.template block<3>(Accel, Accel) = dt * Sj_e;

        return Q;
    }

    /// Gui selection for the Unit of the input covarianceAccel parameter for the StDev due to acceleration due to user motion
    Units::CovarianceAccelUnits _gui_covarianceAccelUnit = Units::CovarianceAccelUnits::m_sqrts3;
    /// @brief GUI selection for the Standard deviation of the acceleration 𝜎_a due to user motion in horizontal and vertical component
    /// @note See Groves (2013) eq. (9.156)
    std::array<double, 2> _gui_covarianceAccel = { 3.0, 1.5 } /* [ m / √(s^3) ] */;

    /// @brief Covariance of the acceleration 𝜎_a due to user motion in horizontal and vertical component [m²/s³]
    std::array<double, 2> _covarianceAccel = { 3.0, 1.5 };

    /// Gui selection for the Unit of the input covarianceJerk parameter for the StDev due to jerk due to user motion
    Units::CovarianceJerkUnits _gui_covarianceJerkUnit = Units::CovarianceJerkUnits::m_sqrts5;
    /// @brief GUI selection for the Standard deviation of the jerk 𝜎_j due to user motion in horizontal and vertical component
    /// @note See Groves (2013) eq. (9.156)
    std::array<double, 2> _gui_covarianceJerk = { 0.0, 0.0 } /* [ m / √(s^5) ] */;

    /// @brief Covariance of the jerk 𝜎_j due to user motion in horizontal and vertical component [m^2/s^5]
    std::array<double, 2> _covarianceJerk = { 0.0, 0.0 };

    /// @brief Converts the provided data into a json object
    /// @param[out] j Json object which gets filled with the info
    /// @param[in] data Data to convert into json
    friend void to_json(json& j, const MotionModel& data)
    {
        j = {
            { "covarianceAccelUnit", data._gui_covarianceAccelUnit },
            { "covarianceAccel", data._gui_covarianceAccel },
            { "covarianceJerkUnit", data._gui_covarianceJerkUnit },
            { "covarianceJerk", data._gui_covarianceJerk },
        };
    }
    /// @brief Converts the provided json object into the data object
    /// @param[in] j Json object with the needed values
    /// @param[out] data Object to fill from the json
    friend void from_json(const json& j, MotionModel& data)
    {
        if (j.contains("covarianceAccelUnit")) { j.at("covarianceAccelUnit").get_to(data._gui_covarianceAccelUnit); }
        if (j.contains("covarianceAccel")) { j.at("covarianceAccel").get_to(data._gui_covarianceAccel); }
        if (j.contains("covarianceJerkUnit")) { j.at("covarianceJerkUnit").get_to(data._gui_covarianceJerkUnit); }
        if (j.contains("covarianceJerk")) { j.at("covarianceJerk").get_to(data._gui_covarianceJerk); }
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
        case AttQ1:
            return fmt::formatter<const char*>::format("AttQ1", ctx);
        case AttQ2:
            return fmt::formatter<const char*>::format("AttQ2", ctx);
        case AttQ3:
            return fmt::formatter<const char*>::format("AttQ3", ctx);
        case AttQ0:
            return fmt::formatter<const char*>::format("AttQ0", ctx);
        case AccelX:
            return fmt::formatter<const char*>::format("AccelX", ctx);
        case AccelY:
            return fmt::formatter<const char*>::format("AccelY", ctx);
        case AccelZ:
            return fmt::formatter<const char*>::format("AccelZ", ctx);
        case AngularRateX:
            return fmt::formatter<const char*>::format("AngularRateX", ctx);
        case AngularRateY:
            return fmt::formatter<const char*>::format("AngularRateY", ctx);
        case AngularRateZ:
            return fmt::formatter<const char*>::format("AngularRateZ", ctx);
        case MotionModelKey_COUNT:
            return fmt::formatter<const char*>::format("MotionModelKey_COUNT", ctx);
        }

        return fmt::formatter<const char*>::format("ERROR", ctx);
    }
};

#endif