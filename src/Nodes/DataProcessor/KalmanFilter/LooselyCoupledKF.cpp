// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "LooselyCoupledKF.hpp"

#include "util/Eigen.hpp"
#include <cmath>

#include <imgui_internal.h>
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

#include "internal/FlowManager.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "Navigation/Constants.hpp"
#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/INS/Functions.hpp"
#include "Navigation/INS/ProcessNoise.hpp"
#include "Navigation/INS/EcefFrame/ErrorEquations.hpp"
#include "Navigation/INS/LocalNavFrame/ErrorEquations.hpp"
#include "Navigation/Math/Math.hpp"
#include "Navigation/Math/VanLoan.hpp"
#include "Navigation/Gravity/Gravity.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "util/Logger.hpp"

/// @brief Scale factor to convert the attitude error
constexpr double SCALE_FACTOR_ATTITUDE = 180. / M_PI;
/// @brief Scale factor to convert the latitude and longitude error
constexpr double SCALE_FACTOR_LAT_LON = NAV::InsConst::pseudometre;
/// @brief Scale factor to convert the acceleration error
constexpr double SCALE_FACTOR_ACCELERATION = 1e3 / NAV::InsConst::G_NORM;
/// @brief Scale factor to convert the angular rate error
constexpr double SCALE_FACTOR_ANGULAR_RATE = 1e3;

NAV::LooselyCoupledKF::LooselyCoupledKF()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 822, 721 };

    nm::CreateInputPin(this, "InertialNavSol", Pin::Type::Flow, { NAV::InertialNavSol::type() }, &LooselyCoupledKF::recvInertialNavigationSolution, nullptr, 1);
    inputPins.back().neededForTemporalQueueCheck = false;
    nm::CreateInputPin(this, "GNSSNavigationSolution", Pin::Type::Flow, { NAV::PosVel::type() }, &LooselyCoupledKF::recvGNSSNavigationSolution,
                       [](const Node* node, const InputPin& inputPin) {
                           const auto* lckf = static_cast<const LooselyCoupledKF*>(node); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
                           return !inputPin.queue.empty() && lckf->_lastPredictRequestedTime < inputPin.queue.front()->insTime;
                       });
    inputPins.back().dropQueueIfNotFirable = false;
    nm::CreateOutputPin(this, "Errors", Pin::Type::Flow, { NAV::LcKfInsGnssErrors::type() });
    nm::CreateOutputPin(this, "Sync", Pin::Type::Flow, { NAV::NodeData::type() });
}

NAV::LooselyCoupledKF::~LooselyCoupledKF()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::LooselyCoupledKF::typeStatic()
{
    return "LooselyCoupledKF";
}

std::string NAV::LooselyCoupledKF::type() const
{
    return typeStatic();
}

std::string NAV::LooselyCoupledKF::category()
{
    return "Data Processor";
}

void NAV::LooselyCoupledKF::addKalmanMatricesPins()
{
    LOG_TRACE("{}: called", nameId());

    if (outputPins.size() == 2)
    {
        nm::CreateOutputPin(this, "x", Pin::Type::Matrix, { "Eigen::VectorXd" }, &_kalmanFilter.x(all));
        nm::CreateOutputPin(this, "P", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.P(all, all));
        nm::CreateOutputPin(this, "Phi", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.Phi(all, all));
        nm::CreateOutputPin(this, "Q", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.Q(all, all));
        nm::CreateOutputPin(this, "z", Pin::Type::Matrix, { "Eigen::VectorXd" }, &_kalmanFilter.z(all));
        nm::CreateOutputPin(this, "H", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.H(all, all));
        nm::CreateOutputPin(this, "R", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.R(all, all));
        nm::CreateOutputPin(this, "K", Pin::Type::Matrix, { "Eigen::MatrixXd" }, &_kalmanFilter.K(all, all));
    }
}

void NAV::LooselyCoupledKF::removeKalmanMatricesPins()
{
    LOG_TRACE("{}: called", nameId());
    while (outputPins.size() > 2)
    {
        nm::DeleteOutputPin(outputPins.back());
    }
}

void NAV::LooselyCoupledKF::guiConfig()
{
    float configWidth = 380.0F * gui::NodeEditorApplication::windowFontRatio();
    float unitWidth = 150.0F * gui::NodeEditorApplication::windowFontRatio();

    float taylorOrderWidth = 75.0F * gui::NodeEditorApplication::windowFontRatio();

    if (ImGui::Checkbox(fmt::format("Show Kalman Filter matrices as output pins##{}", size_t(id)).c_str(), &_showKalmanFilterOutputPins))
    {
        LOG_DEBUG("{}: showKalmanFilterOutputPins {}", nameId(), _showKalmanFilterOutputPins);
        if (_showKalmanFilterOutputPins)
        {
            addKalmanMatricesPins();
        }
        else
        {
            removeKalmanMatricesPins();
        }
        flow::ApplyChanges();
    }
    if (ImGui::Checkbox(fmt::format("Rank check for Kalman filter matrices##{}", size_t(id)).c_str(), &_checkKalmanMatricesRanks))
    {
        LOG_DEBUG("{}: checkKalmanMatricesRanks {}", nameId(), _checkKalmanMatricesRanks);
        flow::ApplyChanges();
    }

    ImGui::Separator();

    ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
    if (ImGui::Combo(fmt::format("Frame##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_frame), "ECEF\0NED\0\0"))
    {
        LOG_DEBUG("{}: Frame changed to {}", nameId(), _frame == Frame::NED ? "NED" : "ECEF");
        flow::ApplyChanges();
    }

    if (_phiCalculationAlgorithm == PhiCalculationAlgorithm::Taylor)
    {
        ImGui::SetNextItemWidth(configWidth - taylorOrderWidth);
    }
    else
    {
        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
    }
    if (ImGui::Combo(fmt::format("##Phi calculation algorithm {}", size_t(id)).c_str(), reinterpret_cast<int*>(&_phiCalculationAlgorithm), "Van Loan\0Taylor\0\0"))
    {
        LOG_DEBUG("{}: Phi calculation algorithm changed to {}", nameId(), fmt::underlying(_phiCalculationAlgorithm));
        flow::ApplyChanges();
    }

    if (_phiCalculationAlgorithm == PhiCalculationAlgorithm::Taylor)
    {
        ImGui::SameLine();
        ImGui::SetNextItemWidth(taylorOrderWidth);
        if (ImGui::InputIntL(fmt::format("##Phi calculation Taylor Order {}", size_t(id)).c_str(), &_phiCalculationTaylorOrder, 1, 9))
        {
            LOG_DEBUG("{}: Phi calculation  Taylor Order changed to {}", nameId(), _phiCalculationTaylorOrder);
            flow::ApplyChanges();
        }
    }
    ImGui::SameLine();
    ImGui::Text("Phi calculation algorithm%s", _phiCalculationAlgorithm == PhiCalculationAlgorithm::Taylor ? " (up to order)" : "");

    ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
    if (ImGui::Combo(fmt::format("Q calculation algorithm##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_qCalculationAlgorithm), "Van Loan\0Taylor 1st Order (Groves 2013)\0\0"))
    {
        LOG_DEBUG("{}: Q calculation algorithm changed to {}", nameId(), fmt::underlying(_qCalculationAlgorithm));
        flow::ApplyChanges();
    }

    ImGui::Separator();

    // ###########################################################################################################
    //                                Q - System/Process noise covariance matrix
    // ###########################################################################################################

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("Q - System/Process noise covariance matrix##{}", size_t(id)).c_str()))
    {
        // --------------------------------------------- Accelerometer -----------------------------------------------
        if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
        {
            ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
            if (ImGui::Combo(fmt::format("Random Process Accelerometer##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_randomProcessAccel), "Random Walk\0"
                                                                                                                                                "Gauss-Markov 1st Order\0\0"))
            {
                LOG_DEBUG("{}: randomProcessAccel changed to {}", nameId(), fmt::underlying(_randomProcessAccel));
                flow::ApplyChanges();
            }
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Standard deviation of the noise on the\naccelerometer specific-force measurements##{}", size_t(id)).c_str(),
                                               configWidth, unitWidth, _stdev_ra.data(), reinterpret_cast<int*>(&_stdevAccelNoiseUnits), "mg/√(Hz)\0m/s^2/√(Hz)\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: stdev_ra changed to {}", nameId(), _stdev_ra.transpose());
            LOG_DEBUG("{}: stdevAccelNoiseUnits changed to {}", nameId(), fmt::underlying(_stdevAccelNoiseUnits));
            flow::ApplyChanges();
        }
        if (gui::widgets::InputDouble3WithUnit(fmt::format("Standard deviation σ of the accel {}##{}",
                                                           _qCalculationAlgorithm == QCalculationAlgorithm::Taylor1
                                                               ? "dynamic bias, in σ²/τ"
                                                               : (_randomProcessAccel == RandomProcess::RandomWalk ? "bias noise" : "bias noise, in √(2σ²β)"),
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _stdev_bad.data(), reinterpret_cast<int*>(&_stdevAccelBiasUnits), "µg\0m/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: stdev_bad changed to {}", nameId(), _stdev_bad.transpose());
            LOG_DEBUG("{}: stdevAccelBiasUnits changed to {}", nameId(), fmt::underlying(_stdevAccelBiasUnits));
            flow::ApplyChanges();
        }

        if (_randomProcessAccel == RandomProcess::GaussMarkov1 || _qCalculationAlgorithm == QCalculationAlgorithm::Taylor1)
        {
            ImGui::SetNextItemWidth(configWidth - unitWidth);
            if (ImGui::InputDouble3L(fmt::format("##Correlation length τ of the accel dynamic bias {}", size_t(id)).c_str(), _tau_bad.data(), 0., std::numeric_limits<double>::max(), "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: tau_bad changed to {}", nameId(), _tau_bad);
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            int unitCorrelationLength = 0;
            ImGui::SetNextItemWidth(unitWidth);
            ImGui::Combo(fmt::format("##Correlation length of the accel dynamic bias unit {}", size_t(id)).c_str(), &unitCorrelationLength, "s\0\0");
            ImGui::SameLine();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
            if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
            {
                ImGui::TextUnformatted("Correlation length τ of the accel bias noise");
            }
            else if (_qCalculationAlgorithm == QCalculationAlgorithm::Taylor1)
            {
                ImGui::TextUnformatted("Correlation length τ of the accel dynamic bias");
            }
        }

        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 20.F);

        // ----------------------------------------------- Gyroscope -------------------------------------------------

        if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
        {
            ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
            if (ImGui::Combo(fmt::format("Random Process Gyroscope##{}", size_t(id)).c_str(), reinterpret_cast<int*>(&_randomProcessGyro), "Random Walk\0"
                                                                                                                                           "Gauss-Markov 1st Order\0\0"))
            {
                LOG_DEBUG("{}: randomProcessGyro changed to {}", nameId(), fmt::underlying(_randomProcessGyro));
                flow::ApplyChanges();
            }
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Standard deviation of the noise on\nthe gyro angular-rate measurements##{}", size_t(id)).c_str(),
                                               configWidth, unitWidth, _stdev_rg.data(), reinterpret_cast<int*>(&_stdevGyroNoiseUnits), "deg/hr/√(Hz)\0rad/s/√(Hz)\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: stdev_rg changed to {}", nameId(), _stdev_rg.transpose());
            LOG_DEBUG("{}: stdevGyroNoiseUnits changed to {}", nameId(), fmt::underlying(_stdevGyroNoiseUnits));
            flow::ApplyChanges();
        }
        if (gui::widgets::InputDouble3WithUnit(fmt::format("Standard deviation σ of the gyro {}##{}",
                                                           _qCalculationAlgorithm == QCalculationAlgorithm::Taylor1
                                                               ? "dynamic bias, in σ²/τ"
                                                               : (_randomProcessGyro == RandomProcess::RandomWalk ? "bias noise" : "bias noise, in √(2σ²β)"),
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _stdev_bgd.data(), reinterpret_cast<int*>(&_stdevGyroBiasUnits), "°/h\0rad/s\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: stdev_bgd changed to {}", nameId(), _stdev_bgd.transpose());
            LOG_DEBUG("{}: stdevGyroBiasUnits changed to {}", nameId(), fmt::underlying(_stdevGyroBiasUnits));
            flow::ApplyChanges();
        }

        if (_randomProcessGyro == RandomProcess::GaussMarkov1 || _qCalculationAlgorithm == QCalculationAlgorithm::Taylor1)
        {
            ImGui::SetNextItemWidth(configWidth - unitWidth);
            if (ImGui::InputDouble3L(fmt::format("##Correlation length τ of the gyro dynamic bias {}", size_t(id)).c_str(), _tau_bgd.data(), 0., std::numeric_limits<double>::max(), "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: tau_bgd changed to {}", nameId(), _tau_bgd);
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            int unitCorrelationLength = 0;
            ImGui::SetNextItemWidth(unitWidth);
            ImGui::Combo(fmt::format("##Correlation length of the gyro dynamic bias unit {}", size_t(id)).c_str(), &unitCorrelationLength, "s\0\0");
            ImGui::SameLine();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
            if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
            {
                ImGui::TextUnformatted("Correlation length τ of the gyro bias noise");
            }
            else if (_qCalculationAlgorithm == QCalculationAlgorithm::Taylor1)
            {
                ImGui::TextUnformatted("Correlation length τ of the gyro dynamic bias");
            }
        }

        ImGui::TreePop();
    }

    // ###########################################################################################################
    //                                        Measurement Uncertainties 𝐑
    // ###########################################################################################################

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("R - Measurement noise covariance matrix##{}", size_t(id)).c_str()))
    {
        if (gui::widgets::InputDouble3WithUnit(fmt::format("{} of the GNSS position measurements##{}",
                                                           _gnssMeasurementUncertaintyPositionUnit == GnssMeasurementUncertaintyPositionUnit::rad2_rad2_m2
                                                                   || _gnssMeasurementUncertaintyPositionUnit == GnssMeasurementUncertaintyPositionUnit::meter2
                                                               ? "Variance"
                                                               : "Standard deviation",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _gnssMeasurementUncertaintyPosition.data(), reinterpret_cast<int*>(&_gnssMeasurementUncertaintyPositionUnit), "rad^2, rad^2, m^2\0"
                                                                                                                                                                                     "rad, rad, m\0"
                                                                                                                                                                                     "m^2, m^2, m^2\0"
                                                                                                                                                                                     "m, m, m\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: gnssMeasurementUncertaintyPosition changed to {}", nameId(), _gnssMeasurementUncertaintyPosition.transpose());
            LOG_DEBUG("{}: gnssMeasurementUncertaintyPositionUnit changed to {}", nameId(), fmt::underlying(_gnssMeasurementUncertaintyPositionUnit));
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("{} of the GNSS velocity measurements##{}", _gnssMeasurementUncertaintyVelocityUnit == GnssMeasurementUncertaintyVelocityUnit::m2_s2 ? "Variance" : "Standard deviation",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _gnssMeasurementUncertaintyVelocity.data(), reinterpret_cast<int*>(&_gnssMeasurementUncertaintyVelocityUnit), "m^2/s^2\0"
                                                                                                                                                                                     "m/s\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: gnssMeasurementUncertaintyVelocity changed to {}", nameId(), _gnssMeasurementUncertaintyVelocity);
            LOG_DEBUG("{}: gnssMeasurementUncertaintyVelocityUnit changed to {}", nameId(), fmt::underlying(_gnssMeasurementUncertaintyVelocityUnit));
            flow::ApplyChanges();
        }

        ImGui::TreePop();
    }

    // ###########################################################################################################
    //                                        𝐏 Error covariance matrix
    // ###########################################################################################################

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("P Error covariance matrix (init)##{}", size_t(id)).c_str()))
    {
        if (gui::widgets::InputDouble3WithUnit(fmt::format("Position covariance ({})##{}",
                                                           _initCovariancePositionUnit == InitCovariancePositionUnit::rad2_rad2_m2
                                                                   || _initCovariancePositionUnit == InitCovariancePositionUnit::meter2
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovariancePosition.data(), reinterpret_cast<int*>(&_initCovariancePositionUnit), "rad^2, rad^2, m^2\0"
                                                                                                                                                             "rad, rad, m\0"
                                                                                                                                                             "m^2, m^2, m^2\0"
                                                                                                                                                             "m, m, m\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovariancePosition changed to {}", nameId(), _initCovariancePosition);
            LOG_DEBUG("{}: initCovariancePositionUnit changed to {}", nameId(), fmt::underlying(_initCovariancePositionUnit));
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Velocity covariance ({})##{}",
                                                           _initCovarianceVelocityUnit == InitCovarianceVelocityUnit::m2_s2
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceVelocity.data(), reinterpret_cast<int*>(&_initCovarianceVelocityUnit), "m^2/s^2\0"
                                                                                                                                                             "m/s\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceVelocity changed to {}", nameId(), _initCovarianceVelocity);
            LOG_DEBUG("{}: initCovarianceVelocityUnit changed to {}", nameId(), fmt::underlying(_initCovarianceVelocityUnit));
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Flight Angles covariance ({})##{}",
                                                           _initCovarianceAttitudeAnglesUnit == InitCovarianceAttitudeAnglesUnit::rad2
                                                                   || _initCovarianceAttitudeAnglesUnit == InitCovarianceAttitudeAnglesUnit::deg2
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceAttitudeAngles.data(), reinterpret_cast<int*>(&_initCovarianceAttitudeAnglesUnit), "rad^2\0"
                                                                                                                                                                         "deg^2\0"
                                                                                                                                                                         "rad\0"
                                                                                                                                                                         "deg\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceAttitudeAngles changed to {}", nameId(), _initCovarianceAttitudeAngles);
            LOG_DEBUG("{}: initCovarianceAttitudeAnglesUnit changed to {}", nameId(), fmt::underlying(_initCovarianceAttitudeAnglesUnit));
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker(_frame == Frame::ECEF
                                     ? "Angles rotating the ECEF frame into the body frame."
                                     : "Angles rotating the local navigation frame into the body frame (Roll, Pitch, Yaw)");

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Accelerometer Bias covariance ({})##{}",
                                                           _initCovarianceBiasAccelUnit == InitCovarianceBiasAccelUnit::m2_s4
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceBiasAccel.data(), reinterpret_cast<int*>(&_initCovarianceBiasAccelUnit), "m^2/s^4\0"
                                                                                                                                                               "m/s^2\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceBiasAccel changed to {}", nameId(), _initCovarianceBiasAccel);
            LOG_DEBUG("{}: initCovarianceBiasAccelUnit changed to {}", nameId(), fmt::underlying(_initCovarianceBiasAccelUnit));
            flow::ApplyChanges();
        }

        if (gui::widgets::InputDouble3WithUnit(fmt::format("Gyroscope Bias covariance ({})##{}",
                                                           _initCovarianceBiasGyroUnit == InitCovarianceBiasGyroUnit::rad2_s2
                                                                   || _initCovarianceBiasGyroUnit == InitCovarianceBiasGyroUnit::deg2_s2
                                                               ? "Variance σ²"
                                                               : "Standard deviation σ",
                                                           size_t(id))
                                                   .c_str(),
                                               configWidth, unitWidth, _initCovarianceBiasGyro.data(), reinterpret_cast<int*>(&_initCovarianceBiasGyroUnit), "rad^2/s^2\0"
                                                                                                                                                             "deg^2/s^2\0"
                                                                                                                                                             "rad/s\0"
                                                                                                                                                             "deg/s\0\0",
                                               "%.2e", ImGuiInputTextFlags_CharsScientific))
        {
            LOG_DEBUG("{}: initCovarianceBiasGyro changed to {}", nameId(), _initCovarianceBiasGyro);
            LOG_DEBUG("{}: initCovarianceBiasGyroUnit changed to {}", nameId(), fmt::underlying(_initCovarianceBiasGyroUnit));
            flow::ApplyChanges();
        }

        ImGui::TreePop();
    }
}

[[nodiscard]] json NAV::LooselyCoupledKF::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["showKalmanFilterOutputPins"] = _showKalmanFilterOutputPins;
    j["checkKalmanMatricesRanks"] = _checkKalmanMatricesRanks;

    j["frame"] = _frame;
    j["phiCalculationAlgorithm"] = _phiCalculationAlgorithm;
    j["phiCalculationTaylorOrder"] = _phiCalculationTaylorOrder;
    j["qCalculationAlgorithm"] = _qCalculationAlgorithm;

    j["randomProcessAccel"] = _randomProcessAccel;
    j["randomProcessGyro"] = _randomProcessGyro;
    j["stdev_ra"] = _stdev_ra;
    j["stdevAccelNoiseUnits"] = _stdevAccelNoiseUnits;
    j["stdev_rg"] = _stdev_rg;
    j["stdevGyroNoiseUnits"] = _stdevGyroNoiseUnits;
    j["stdev_bad"] = _stdev_bad;
    j["tau_bad"] = _tau_bad;
    j["stdevAccelBiasUnits"] = _stdevAccelBiasUnits;
    j["stdev_bgd"] = _stdev_bgd;
    j["tau_bgd"] = _tau_bgd;
    j["stdevGyroBiasUnits"] = _stdevGyroBiasUnits;

    j["gnssMeasurementUncertaintyPositionUnit"] = _gnssMeasurementUncertaintyPositionUnit;
    j["gnssMeasurementUncertaintyPosition"] = _gnssMeasurementUncertaintyPosition;
    j["gnssMeasurementUncertaintyVelocityUnit"] = _gnssMeasurementUncertaintyVelocityUnit;
    j["gnssMeasurementUncertaintyVelocity"] = _gnssMeasurementUncertaintyVelocity;

    j["initCovariancePositionUnit"] = _initCovariancePositionUnit;
    j["initCovariancePosition"] = _initCovariancePosition;
    j["initCovarianceVelocityUnit"] = _initCovarianceVelocityUnit;
    j["initCovarianceVelocity"] = _initCovarianceVelocity;
    j["initCovarianceAttitudeAnglesUnit"] = _initCovarianceAttitudeAnglesUnit;
    j["initCovarianceAttitudeAngles"] = _initCovarianceAttitudeAngles;
    j["initCovarianceBiasAccelUnit"] = _initCovarianceBiasAccelUnit;
    j["initCovarianceBiasAccel"] = _initCovarianceBiasAccel;
    j["initCovarianceBiasGyroUnit"] = _initCovarianceBiasGyroUnit;
    j["initCovarianceBiasGyro"] = _initCovarianceBiasGyro;

    return j;
}

void NAV::LooselyCoupledKF::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("showKalmanFilterOutputPins"))
    {
        j.at("showKalmanFilterOutputPins").get_to(_showKalmanFilterOutputPins);
        if (_showKalmanFilterOutputPins)
        {
            addKalmanMatricesPins();
        }
    }
    if (j.contains("checkKalmanMatricesRanks"))
    {
        j.at("checkKalmanMatricesRanks").get_to(_checkKalmanMatricesRanks);
    }

    if (j.contains("frame"))
    {
        j.at("frame").get_to(_frame);
    }
    if (j.contains("phiCalculationAlgorithm"))
    {
        j.at("phiCalculationAlgorithm").get_to(_phiCalculationAlgorithm);
    }
    if (j.contains("phiCalculationTaylorOrder"))
    {
        j.at("phiCalculationTaylorOrder").get_to(_phiCalculationTaylorOrder);
    }
    if (j.contains("qCalculationAlgorithm"))
    {
        j.at("qCalculationAlgorithm").get_to(_qCalculationAlgorithm);
    }
    // ------------------------------- 𝐐 System/Process noise covariance matrix ---------------------------------
    if (j.contains("randomProcessAccel"))
    {
        j.at("randomProcessAccel").get_to(_randomProcessAccel);
    }
    if (j.contains("randomProcessGyro"))
    {
        j.at("randomProcessGyro").get_to(_randomProcessGyro);
    }
    if (j.contains("stdev_ra"))
    {
        _stdev_ra = j.at("stdev_ra");
    }
    if (j.contains("stdevAccelNoiseUnits"))
    {
        j.at("stdevAccelNoiseUnits").get_to(_stdevAccelNoiseUnits);
    }
    if (j.contains("stdev_rg"))
    {
        _stdev_rg = j.at("stdev_rg");
    }
    if (j.contains("stdevGyroNoiseUnits"))
    {
        j.at("stdevGyroNoiseUnits").get_to(_stdevGyroNoiseUnits);
    }
    if (j.contains("stdev_bad"))
    {
        _stdev_bad = j.at("stdev_bad");
    }
    if (j.contains("tau_bad"))
    {
        _tau_bad = j.at("tau_bad");
    }
    if (j.contains("stdevAccelBiasUnits"))
    {
        j.at("stdevAccelBiasUnits").get_to(_stdevAccelBiasUnits);
    }
    if (j.contains("stdev_bgd"))
    {
        _stdev_bgd = j.at("stdev_bgd");
    }
    if (j.contains("tau_bgd"))
    {
        _tau_bgd = j.at("tau_bgd");
    }
    if (j.contains("stdevGyroBiasUnits"))
    {
        j.at("stdevGyroBiasUnits").get_to(_stdevGyroBiasUnits);
    }
    // -------------------------------- 𝐑 Measurement noise covariance matrix -----------------------------------
    if (j.contains("gnssMeasurementUncertaintyPositionUnit"))
    {
        j.at("gnssMeasurementUncertaintyPositionUnit").get_to(_gnssMeasurementUncertaintyPositionUnit);
    }
    if (j.contains("gnssMeasurementUncertaintyPosition"))
    {
        _gnssMeasurementUncertaintyPosition = j.at("gnssMeasurementUncertaintyPosition");
    }
    if (j.contains("gnssMeasurementUncertaintyVelocityUnit"))
    {
        j.at("gnssMeasurementUncertaintyVelocityUnit").get_to(_gnssMeasurementUncertaintyVelocityUnit);
    }
    if (j.contains("gnssMeasurementUncertaintyVelocity"))
    {
        _gnssMeasurementUncertaintyVelocity = j.at("gnssMeasurementUncertaintyVelocity");
    }
    // -------------------------------------- 𝐏 Error covariance matrix -----------------------------------------
    if (j.contains("initCovariancePositionUnit"))
    {
        j.at("initCovariancePositionUnit").get_to(_initCovariancePositionUnit);
    }
    if (j.contains("initCovariancePosition"))
    {
        _initCovariancePosition = j.at("initCovariancePosition");
    }
    if (j.contains("initCovarianceVelocityUnit"))
    {
        j.at("initCovarianceVelocityUnit").get_to(_initCovarianceVelocityUnit);
    }
    if (j.contains("initCovarianceVelocity"))
    {
        _initCovarianceVelocity = j.at("initCovarianceVelocity");
    }
    if (j.contains("initCovarianceAttitudeAnglesUnit"))
    {
        j.at("initCovarianceAttitudeAnglesUnit").get_to(_initCovarianceAttitudeAnglesUnit);
    }
    if (j.contains("initCovarianceAttitudeAngles"))
    {
        _initCovarianceAttitudeAngles = j.at("initCovarianceAttitudeAngles");
    }
    if (j.contains("initCovarianceBiasAccelUnit"))
    {
        j.at("initCovarianceBiasAccelUnit").get_to(_initCovarianceBiasAccelUnit);
    }
    if (j.contains("initCovarianceBiasAccel"))
    {
        _initCovarianceBiasAccel = j.at("initCovarianceBiasAccel");
    }
    if (j.contains("initCovarianceBiasGyroUnit"))
    {
        j.at("initCovarianceBiasGyroUnit").get_to(_initCovarianceBiasGyroUnit);
    }
    if (j.contains("initCovarianceBiasGyro"))
    {
        _initCovarianceBiasGyro = j.at("initCovarianceBiasGyro");
    }
}

bool NAV::LooselyCoupledKF::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _kalmanFilter.setZero();

    _latestInertialNavSol = nullptr;
    _lastPredictTime.reset();
    _lastPredictRequestedTime.reset();
    _accumulatedAccelBiases.setZero();
    _accumulatedGyroBiases.setZero();

    // Initial Covariance of the attitude angles in [rad²]
    Eigen::Vector3d variance_angles = Eigen::Vector3d::Zero();
    if (_initCovarianceAttitudeAnglesUnit == InitCovarianceAttitudeAnglesUnit::rad2)
    {
        variance_angles = _initCovarianceAttitudeAngles;
    }
    else if (_initCovarianceAttitudeAnglesUnit == InitCovarianceAttitudeAnglesUnit::deg2)
    {
        variance_angles = deg2rad(_initCovarianceAttitudeAngles);
    }
    else if (_initCovarianceAttitudeAnglesUnit == InitCovarianceAttitudeAnglesUnit::rad)
    {
        variance_angles = _initCovarianceAttitudeAngles.array().pow(2);
    }
    else if (_initCovarianceAttitudeAnglesUnit == InitCovarianceAttitudeAnglesUnit::deg)
    {
        variance_angles = deg2rad(_initCovarianceAttitudeAngles).array().pow(2);
    }

    // Initial Covariance of the velocity in [m²/s²]
    Eigen::Vector3d variance_vel = Eigen::Vector3d::Zero();
    if (_initCovarianceVelocityUnit == InitCovarianceVelocityUnit::m2_s2)
    {
        variance_vel = _initCovarianceVelocity;
    }
    else if (_initCovarianceVelocityUnit == InitCovarianceVelocityUnit::m_s)
    {
        variance_vel = _initCovarianceVelocity.array().pow(2);
    }

    // Initial Covariance of the position in [m²]
    Eigen::Vector3d e_variance = Eigen::Vector3d::Zero();
    // Initial Covariance of the position in [rad² rad² m²]
    Eigen::Vector3d lla_variance = Eigen::Vector3d::Zero();
    if (_initCovariancePositionUnit == InitCovariancePositionUnit::rad2_rad2_m2)
    {
        e_variance = trafo::lla2ecef_WGS84(_initCovariancePosition.cwiseSqrt()).array().pow(2);
        lla_variance = _initCovariancePosition;
    }
    else if (_initCovariancePositionUnit == InitCovariancePositionUnit::rad_rad_m)
    {
        e_variance = trafo::lla2ecef_WGS84(_initCovariancePosition).array().pow(2);
        lla_variance = _initCovariancePosition.array().pow(2);
    }
    else if (_initCovariancePositionUnit == InitCovariancePositionUnit::meter)
    {
        e_variance = _initCovariancePosition.array().pow(2);
        lla_variance = (trafo::ecef2lla_WGS84(trafo::ned2ecef(_initCovariancePosition, { 0, 0, 0 }))).array().pow(2);
    }
    else if (_initCovariancePositionUnit == InitCovariancePositionUnit::meter2)
    {
        e_variance = _initCovariancePosition;
        lla_variance = (trafo::ecef2lla_WGS84(trafo::ned2ecef(_initCovariancePosition.cwiseSqrt(), { 0, 0, 0 }))).array().pow(2);
    }

    // Initial Covariance of the accelerometer biases in [m^2/s^4]
    Eigen::Vector3d variance_accelBias = Eigen::Vector3d::Zero();
    if (_initCovarianceBiasAccelUnit == InitCovarianceBiasAccelUnit::m2_s4)
    {
        variance_accelBias = _initCovarianceBiasAccel;
    }
    else if (_initCovarianceBiasAccelUnit == InitCovarianceBiasAccelUnit::m_s2)
    {
        variance_accelBias = _initCovarianceBiasAccel.array().pow(2);
    }

    // Initial Covariance of the gyroscope biases in [rad^2/s^2]
    Eigen::Vector3d variance_gyroBias = Eigen::Vector3d::Zero();
    if (_initCovarianceBiasGyroUnit == InitCovarianceBiasGyroUnit::rad2_s2)
    {
        variance_gyroBias = _initCovarianceBiasGyro;
    }
    else if (_initCovarianceBiasGyroUnit == InitCovarianceBiasGyroUnit::deg2_s2)
    {
        variance_gyroBias = deg2rad(_initCovarianceBiasGyro.array().sqrt()).array().pow(2);
    }
    else if (_initCovarianceBiasGyroUnit == InitCovarianceBiasGyroUnit::rad_s)
    {
        variance_gyroBias = _initCovarianceBiasGyro.array().pow(2);
    }
    else if (_initCovarianceBiasGyroUnit == InitCovarianceBiasGyroUnit::deg_s)
    {
        variance_gyroBias = deg2rad(_initCovarianceBiasGyro).array().pow(2);
    }

    // 𝐏 Error covariance matrix
    _kalmanFilter.P = initialErrorCovarianceMatrix_P0(variance_angles,                                  // Flight Angles covariance
                                                      variance_vel,                                     // Velocity covariance
                                                      _frame == Frame::NED ? lla_variance : e_variance, // Position (Lat, Lon, Alt) / ECEF covariance
                                                      variance_accelBias,                               // Accelerometer Bias covariance
                                                      variance_gyroBias);                               // Gyroscope Bias covariance

    LOG_DEBUG("{}: initialized", nameId());
    LOG_DATA("{}: P_0 =\n{}", nameId(), _kalmanFilter.P);

    return true;
}

void NAV::LooselyCoupledKF::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::LooselyCoupledKF::recvInertialNavigationSolution(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */) // NOLINT(readability-convert-member-functions-to-static)
{
    auto inertialNavSol = std::static_pointer_cast<const InertialNavSol>(queue.extract_front());
    LOG_DATA("{}: recvInertialNavigationSolution at time [{} - {}]", nameId(), inertialNavSol->insTime.toYMDHMS(), inertialNavSol->insTime.toGPSweekTow());

    double tau_i = !_lastPredictTime.empty()
                       ? static_cast<double>((inertialNavSol->insTime - _lastPredictTime).count())
                       : 0.0;

    if (tau_i > 0)
    {
        _lastPredictTime = _latestInertialNavSol->insTime + std::chrono::duration<double>(tau_i);
        looselyCoupledPrediction(_latestInertialNavSol, tau_i);
    }
    else
    {
        _lastPredictTime = inertialNavSol->insTime;
    }
    _latestInertialNavSol = inertialNavSol;

    if (!inputPins[INPUT_PORT_INDEX_GNSS].queue.empty() && inputPins[INPUT_PORT_INDEX_GNSS].queue.front()->insTime == _lastPredictTime)
    {
        looselyCoupledUpdate(std::static_pointer_cast<const PosVel>(inputPins[INPUT_PORT_INDEX_GNSS].queue.extract_front()));
        if (inputPins[INPUT_PORT_INDEX_GNSS].queue.empty() && inputPins[INPUT_PORT_INDEX_GNSS].link.getConnectedPin()->mode == OutputPin::Mode::REAL_TIME)
        {
            outputPins[OUTPUT_PORT_INDEX_SYNC].mode = OutputPin::Mode::REAL_TIME;
            for (auto& link : outputPins[OUTPUT_PORT_INDEX_SYNC].links)
            {
                link.connectedNode->wakeWorker();
            }
        }
    }
}

void NAV::LooselyCoupledKF::recvGNSSNavigationSolution(InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto gnssMeasurement = queue.front();
    LOG_DATA("{}: recvGNSSNavigationSolution at time [{} - {}]", nameId(), gnssMeasurement->insTime.toYMDHMS(), gnssMeasurement->insTime.toGPSweekTow());

    auto nodeData = std::make_shared<NodeData>();
    nodeData->insTime = gnssMeasurement->insTime;
    _lastPredictRequestedTime = gnssMeasurement->insTime;

    invokeCallbacks(OUTPUT_PORT_INDEX_SYNC, nodeData); // Prediction consists out of ImuIntegration and prediction (gets triggered from it)
}

// ###########################################################################################################
//                                               Kalman Filter
// ###########################################################################################################

void NAV::LooselyCoupledKF::looselyCoupledPrediction(const std::shared_ptr<const InertialNavSol>& inertialNavSol, double tau_i)
{
    auto dt = fmt::format("{:0.5f}", tau_i);
    dt.erase(std::find_if(dt.rbegin(), dt.rend(), [](char ch) { return ch != '0'; }).base(), dt.end());

    InsTime predictTime = inertialNavSol->insTime + std::chrono::duration<double>(tau_i);
    LOG_DATA("{}: Predicting to [{}] (dt = {}s)", nameId(), predictTime, dt);

    // ------------------------------------------- GUI Parameters ----------------------------------------------

    // 𝜎_ra Standard deviation of the noise on the accelerometer specific-force state [m / (s^2 · √(s))]
    Eigen::Vector3d sigma_ra = Eigen::Vector3d::Zero();
    switch (_stdevAccelNoiseUnits)
    {
    case StdevAccelNoiseUnits::mg_sqrtHz: // [mg / √(Hz)]
        sigma_ra = _stdev_ra * 1e-3;      // [g / √(Hz)]
        sigma_ra *= InsConst::G_NORM;     // [m / (s^2 · √(Hz))] = [m / (s · √(s))]
        // sigma_ra /= 1.;                // [m / (s^2 · √(s))]
        break;
    case StdevAccelNoiseUnits::m_s2_sqrtHz: // [m / (s^2 · √(Hz))] = [m / (s · √(s))]
        sigma_ra = _stdev_ra;
        // sigma_ra /= 1.;                  // [m / (s^2 · √(s))]
        break;
    }
    LOG_DATA("{}:     sigma_ra = {} [m / (s^2 · √(s))]", nameId(), sigma_ra.transpose());

    // 𝜎_rg Standard deviation of the noise on the gyro angular-rate state [rad / (s · √(s))]
    Eigen::Vector3d sigma_rg = Eigen::Vector3d::Zero();
    switch (_stdevGyroNoiseUnits)
    {
    case StdevGyroNoiseUnits::deg_hr_sqrtHz: // [deg / hr / √(Hz)] (see Woodman (2007) Chp. 3.2.2 - eq. 7 with seconds instead of hours)
        sigma_rg = deg2rad(_stdev_rg);       // [rad / hr / √(Hz)]
        sigma_rg /= 60.;                     // [rad / √(hr)]
        sigma_rg /= 60.;                     // [rad / √(s)]
        // sigma_rg /= 1.;                    // [rad / (s · √(s))]
        break;
    case StdevGyroNoiseUnits::rad_s_sqrtHz: // [rad / (s · √(Hz))] = [rad / √(s)]
        sigma_rg = _stdev_rg;
        // sigma_rg /= 1.;                  // [rad / (s · √(s))]
        break;
    }
    LOG_DATA("{}:     sigma_rg = {} [rad / (s · √(s))]", nameId(), sigma_rg.transpose());

    // 𝜎_bad Standard deviation of the accelerometer dynamic bias [m / s^2]
    Eigen::Vector3d sigma_bad = Eigen::Vector3d::Zero();
    switch (_stdevAccelBiasUnits)
    {
    case StdevAccelBiasUnits::microg:  // [µg]
        sigma_bad = _stdev_bad * 1e-6; // [g]
        sigma_bad *= InsConst::G_NORM; // [m / s^2]
        break;
    case StdevAccelBiasUnits::m_s2: // [m / s^2]
        sigma_bad = _stdev_bad;
        break;
    }
    LOG_DATA("{}:     sigma_bad = {} [m / s^2]", nameId(), sigma_bad.transpose());

    // 𝜎_bgd Standard deviation of the gyro dynamic bias [rad / s]
    Eigen::Vector3d sigma_bgd = Eigen::Vector3d::Zero();
    switch (_stdevGyroBiasUnits)
    {
    case StdevGyroBiasUnits::deg_h:      // [° / h]
        sigma_bgd = _stdev_bgd / 3600.0; // [° / s]
        sigma_bgd = deg2rad(sigma_bgd);  // [rad / s]
        break;
    case StdevGyroBiasUnits::rad_s: // [rad / s]
        sigma_bgd = _stdev_bgd;
        break;
    }
    LOG_DATA("{}:     sigma_bgd = {} [rad / s]", nameId(), sigma_bgd.transpose());

    // ---------------------------------------------- Prediction -------------------------------------------------

    // Latitude 𝜙, longitude λ and altitude (height above ground) in [rad, rad, m] at the time tₖ₋₁
    const Eigen::Vector3d& lla_position = inertialNavSol->lla_position();
    LOG_DATA("{}:     lla_position = {} [rad, rad, m]", nameId(), lla_position.transpose());
    // Prime vertical radius of curvature (East/West) [m]
    double R_E = calcEarthRadius_E(lla_position(0));
    LOG_DATA("{}:     R_E = {} [m]", nameId(), R_E);
    // Geocentric Radius in [m]
    double r_eS_e = calcGeocentricRadius(lla_position(0), R_E);
    LOG_DATA("{}:     r_eS_e = {} [m]", nameId(), r_eS_e);

    // a_p Acceleration in [m/s^2], in body coordinates
    auto b_acceleration = _latestInertialNavSol->imuObs == nullptr
                              ? Eigen::Vector3d::Zero()
                              : Eigen::Vector3d(inertialNavSol->imuObs->imuPos.b_quatAccel_p() * inertialNavSol->imuObs->accelUncompXYZ.value()
                                                - _accumulatedAccelBiases);
    LOG_DATA("{}:     b_acceleration = {} [m/s^2]", nameId(), b_acceleration.transpose());

    // System Matrix
    KeyedMatrix<double, KFStates, KFStates, 15, 15> F(Eigen::Matrix<double, 15, 15>::Zero(), States);

    if (_frame == Frame::NED)
    {
        // n_velocity (tₖ₋₁) Velocity in [m/s], in navigation coordinates, at the time tₖ₋₁
        const Eigen::Vector3d& n_velocity = inertialNavSol->n_velocity();
        LOG_DATA("{}:     n_velocity = {} [m / s]", nameId(), n_velocity.transpose());
        // q (tₖ₋₁) Quaternion, from body to navigation coordinates, at the time tₖ₋₁
        const Eigen::Quaterniond& n_Quat_b = inertialNavSol->n_Quat_b();
        LOG_DATA("{}:     n_Quat_b --> Roll, Pitch, Yaw = {} [deg]", nameId(), deg2rad(trafo::quat2eulerZYX(n_Quat_b).transpose()));

        // Meridian radius of curvature in [m]
        double R_N = calcEarthRadius_N(lla_position(0));
        LOG_DATA("{}:     R_N = {} [m]", nameId(), R_N);

        // Conversion matrix between cartesian and curvilinear perturbations to the position
        Eigen::Matrix3d T_rn_p = conversionMatrixCartesianCurvilinear(lla_position, R_N, R_E);
        LOG_DATA("{}:     T_rn_p =\n{}", nameId(), T_rn_p);

        // Gravitation at surface level in [m/s^2]
        double g_0 = n_calcGravitation_EGM96(lla_position).norm();

        // omega_in^n = omega_ie^n + omega_en^n
        Eigen::Vector3d n_omega_in = inertialNavSol->n_Quat_e() * InsConst::e_omega_ie
                                     + n_calcTransportRate(lla_position, n_velocity, R_N, R_E);
        LOG_DATA("{}:     n_omega_in = {} [rad/s]", nameId(), n_omega_in.transpose());

        // System Matrix
        F = n_systemMatrix_F(n_Quat_b, b_acceleration, n_omega_in, n_velocity, lla_position, R_N, R_E, g_0, r_eS_e, _tau_bad, _tau_bgd);
        LOG_DATA("{}:     F =\n{}", nameId(), F);

        if (_qCalculationAlgorithm == QCalculationAlgorithm::Taylor1)
        {
            // 2. Calculate the system noise covariance matrix Q_{k-1}
            if (_showKalmanFilterOutputPins) { requestOutputValueLock(OUTPUT_PORT_INDEX_Q); }

            _kalmanFilter.Q = n_systemNoiseCovarianceMatrix_Q(sigma_ra.array().square(), sigma_rg.array().square(),
                                                              sigma_bad.array().square(), sigma_bgd.array().square(),
                                                              _tau_bad, _tau_bgd,
                                                              F.block<3>(Vel, Att), T_rn_p,
                                                              n_Quat_b.toRotationMatrix(), tau_i);
        }
    }
    else // if (_frame == Frame::ECEF)
    {
        // e_position (tₖ₋₁) Position in [m/s], in ECEF coordinates, at the time tₖ₋₁
        const Eigen::Vector3d& e_position = inertialNavSol->e_position();
        LOG_DATA("{}:     e_position = {} [m]", nameId(), e_position.transpose());
        // q (tₖ₋₁) Quaternion, from body to Earth coordinates, at the time tₖ₋₁
        const Eigen::Quaterniond& e_Quat_b = inertialNavSol->e_Quat_b();
        LOG_DATA("{}:     e_Quat_b = {}", nameId(), e_Quat_b);

        // Gravitation in [m/s^2] in ECEF coordinates
        Eigen::Vector3d e_gravitation = trafo::e_Quat_n(lla_position(0), lla_position(1)) * n_calcGravitation_EGM96(lla_position);

        // System Matrix
        F = e_systemMatrix_F(e_Quat_b, b_acceleration, e_position, e_gravitation, r_eS_e, InsConst::e_omega_ie, _tau_bad, _tau_bgd);
        LOG_DATA("{}:     F =\n{}", nameId(), F);

        if (_qCalculationAlgorithm == QCalculationAlgorithm::Taylor1)
        {
            // 2. Calculate the system noise covariance matrix Q_{k-1}
            if (_showKalmanFilterOutputPins) { requestOutputValueLock(OUTPUT_PORT_INDEX_Q); }

            _kalmanFilter.Q = e_systemNoiseCovarianceMatrix_Q(sigma_ra.array().square(), sigma_rg.array().square(),
                                                              sigma_bad.array().square(), sigma_bgd.array().square(),
                                                              _tau_bad, _tau_bgd,
                                                              F.block<3>(Vel, Att),
                                                              e_Quat_b.toRotationMatrix(), tau_i);
        }
    }

    if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
    {
        // Noise Input Matrix
        auto G = noiseInputMatrix_G(_frame == Frame::NED ? inertialNavSol->n_Quat_b() : inertialNavSol->e_Quat_b());
        LOG_DATA("{}:     G =\n{}", nameId(), G);

        Eigen::Matrix<double, 12, 12> W = noiseScaleMatrix_W(sigma_ra, sigma_rg,
                                                             sigma_bad, sigma_bgd,
                                                             _tau_bad, _tau_bgd);
        LOG_DATA("{}:     W =\n{}", nameId(), W);

        LOG_DATA("{}:     G*W*G^T =\n{}", nameId(), G(all, all) * W * G(all, all).transpose());

        auto [Phi, Q] = calcPhiAndQWithVanLoanMethod(F(all, all), G(all, all), W, tau_i);

        // 1. Calculate the transition matrix 𝚽_{k-1}
        if (_showKalmanFilterOutputPins) { requestOutputValueLock(OUTPUT_PORT_INDEX_Phi); }

        _kalmanFilter.Phi(all, all) = Phi;

        // 2. Calculate the system noise covariance matrix Q_{k-1}
        if (_showKalmanFilterOutputPins) { requestOutputValueLock(OUTPUT_PORT_INDEX_Q); }

        _kalmanFilter.Q(all, all) = Q;
    }

    // If Q was calculated over Van Loan, then the Phi matrix was automatically calculated with the exponential matrix
    if (_phiCalculationAlgorithm != PhiCalculationAlgorithm::Exponential || _qCalculationAlgorithm != QCalculationAlgorithm::VanLoan)
    {
        if (_showKalmanFilterOutputPins) { requestOutputValueLock(OUTPUT_PORT_INDEX_Phi); }

        if (_phiCalculationAlgorithm == PhiCalculationAlgorithm::Exponential)
        {
            // 1. Calculate the transition matrix 𝚽_{k-1}
            _kalmanFilter.Phi = transitionMatrix_Phi_exp(F, tau_i);
        }
        else if (_phiCalculationAlgorithm == PhiCalculationAlgorithm::Taylor)
        {
            // 1. Calculate the transition matrix 𝚽_{k-1}
            _kalmanFilter.Phi = transitionMatrix_Phi_Taylor(F, tau_i, static_cast<size_t>(_phiCalculationTaylorOrder));
        }
        else
        {
            LOG_CRITICAL("{}: Calculation algorithm '{}' for the system matrix Phi is not supported.", nameId(), fmt::underlying(_phiCalculationAlgorithm));
        }
    }

    LOG_DATA("{}:     KF.Phi =\n{}", nameId(), _kalmanFilter.Phi);
    LOG_DATA("{}:     KF.Q =\n{}", nameId(), _kalmanFilter.Q);
    if (_showKalmanFilterOutputPins)
    {
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_Phi, predictTime);
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_Q, predictTime);
    }

    LOG_DATA("{}:     Q - Q^T =\n{}", nameId(), _kalmanFilter.Q(all, all) - _kalmanFilter.Q(all, all).transpose());
    LOG_DATA("{}:     KF.P (before prediction) =\n{}", nameId(), _kalmanFilter.P);

    // 3. Propagate the state vector estimate from x(+) and x(-)
    // 4. Propagate the error covariance matrix from P(+) and P(-)
    if (_showKalmanFilterOutputPins)
    {
        requestOutputValueLock(OUTPUT_PORT_INDEX_x);
        requestOutputValueLock(OUTPUT_PORT_INDEX_P);
    }

    _kalmanFilter.predict();

    if (_showKalmanFilterOutputPins)
    {
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_x, predictTime);
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_P, predictTime);
    }
    LOG_DATA("{}:     KF.x = {}", nameId(), _kalmanFilter.x.transposed());
    LOG_DATA("{}:     KF.P (after prediction) =\n{}", nameId(), _kalmanFilter.P);

    // Averaging of P to avoid numerical problems with symmetry (did not work)
    // _kalmanFilter.P = ((_kalmanFilter.P + _kalmanFilter.P.transpose()) / 2.0);

    // LOG_DEBUG("{}: F\n{}\n", nameId(), F);
    // LOG_DEBUG("{}: Phi\n{}\n", nameId(), _kalmanFilter.Phi);

    // LOG_DEBUG("{}: Q\n{}\n", nameId(), _kalmanFilter.Q);
    // LOG_DEBUG("{}: Q - Q^T\n{}\n", nameId(), _kalmanFilter.Q - _kalmanFilter.Q.transpose());

    // LOG_DEBUG("{}: x\n{}\n", nameId(), _kalmanFilter.x);

    // LOG_DEBUG("{}: P\n{}\n", nameId(), _kalmanFilter.P);
    // LOG_DEBUG("{}: P - P^T\n{}\n", nameId(), _kalmanFilter.P - _kalmanFilter.P.transpose());

    if (_checkKalmanMatricesRanks)
    {
        Eigen::FullPivLU<Eigen::MatrixXd> lu(_kalmanFilter.P(all, all));
        auto rank = lu.rank();
        if (rank != _kalmanFilter.P(all, all).rows())
        {
            LOG_WARN("{}: P.rank = {}", nameId(), rank);
        }
    }
}

void NAV::LooselyCoupledKF::looselyCoupledUpdate(const std::shared_ptr<const PosVel>& gnssMeasurement)
{
    LOG_DATA("{}: Updating to [{}] (lastInertial at [{}])", nameId(), gnssMeasurement->insTime, _latestInertialNavSol->insTime);

    // -------------------------------------------- GUI Parameters -----------------------------------------------

    // Latitude 𝜙, longitude λ and altitude (height above ground) in [rad, rad, m] at the time tₖ₋₁
    const Eigen::Vector3d& lla_position = _latestInertialNavSol->lla_position();
    LOG_DATA("{}:     lla_position = {} [rad, rad, m]", nameId(), lla_position.transpose());

    // GNSS measurement uncertainty for the position (Variance σ²) in [m^2]
    Eigen::Vector3d gnssSigmaSquaredPosition = Eigen::Vector3d::Zero();
    // GNSS measurement uncertainty for the position (Variance σ²) in [rad^2, rad^2, m^2]
    Eigen::Vector3d gnssSigmaSquaredLatLonAlt = Eigen::Vector3d::Zero();
    switch (_gnssMeasurementUncertaintyPositionUnit)
    {
    case GnssMeasurementUncertaintyPositionUnit::meter:
        gnssSigmaSquaredPosition = _gnssMeasurementUncertaintyPosition.array().pow(2);
        gnssSigmaSquaredLatLonAlt = (trafo::ecef2lla_WGS84(trafo::ned2ecef(_gnssMeasurementUncertaintyPosition, lla_position)) - lla_position).array().pow(2);
        break;
    case GnssMeasurementUncertaintyPositionUnit::meter2:
        gnssSigmaSquaredPosition = _gnssMeasurementUncertaintyPosition;
        gnssSigmaSquaredLatLonAlt = (trafo::ecef2lla_WGS84(trafo::ned2ecef(_gnssMeasurementUncertaintyPosition.cwiseSqrt(), lla_position)) - lla_position).array().pow(2);
        break;
    case GnssMeasurementUncertaintyPositionUnit::rad_rad_m:
        gnssSigmaSquaredPosition = (trafo::lla2ecef_WGS84(lla_position + _gnssMeasurementUncertaintyPosition) - _latestInertialNavSol->e_position()).array().pow(2);
        gnssSigmaSquaredLatLonAlt = _gnssMeasurementUncertaintyPosition.array().pow(2);
        break;
    case GnssMeasurementUncertaintyPositionUnit::rad2_rad2_m2:
        gnssSigmaSquaredPosition = (trafo::lla2ecef_WGS84(lla_position + _gnssMeasurementUncertaintyPosition.cwiseSqrt()) - _latestInertialNavSol->e_position()).array().pow(2);
        gnssSigmaSquaredLatLonAlt = _gnssMeasurementUncertaintyPosition;
        break;
    }
    LOG_DATA("{}:     gnssSigmaSquaredPosition = {} [m^2]", nameId(), gnssSigmaSquaredPosition.transpose());
    LOG_DATA("{}:     gnssSigmaSquaredLatLonAlt = {} [rad^2, rad^2, m^2]", nameId(), gnssSigmaSquaredLatLonAlt.transpose());

    // GNSS measurement uncertainty for the velocity (Variance σ²) in [m^2/s^2]
    Eigen::Vector3d gnssSigmaSquaredVelocity = Eigen::Vector3d::Zero();
    switch (_gnssMeasurementUncertaintyVelocityUnit)
    {
    case GnssMeasurementUncertaintyVelocityUnit::m_s:
        gnssSigmaSquaredVelocity = _gnssMeasurementUncertaintyVelocity.array().pow(2);
        break;
    case GnssMeasurementUncertaintyVelocityUnit::m2_s2:
        gnssSigmaSquaredVelocity = _gnssMeasurementUncertaintyVelocity;
        break;
    }
    LOG_DATA("{}:     gnssSigmaSquaredVelocity = {} [m^2/S^2]", nameId(), gnssSigmaSquaredVelocity.transpose());

    // ---------------------------------------------- Correction -------------------------------------------------

    // Angular rate measured in units of [rad/s], and given in the body frame
    auto b_omega_ip = _latestInertialNavSol->imuObs == nullptr
                          ? Eigen::Vector3d::Zero()
                          : Eigen::Vector3d(_latestInertialNavSol->imuObs->imuPos.b_quatGyro_p() * _latestInertialNavSol->imuObs->gyroUncompXYZ.value()
                                            - _accumulatedGyroBiases);
    LOG_DATA("{}:     b_omega_ip = {} [rad/s]", nameId(), b_omega_ip.transpose());

    if (_frame == Frame::NED)
    {
        // Prime vertical radius of curvature (East/West) [m]
        double R_E = calcEarthRadius_E(lla_position(0));
        LOG_DATA("{}:     R_E = {} [m]", nameId(), R_E);
        // Meridian radius of curvature in [m]
        double R_N = calcEarthRadius_N(lla_position(0));
        LOG_DATA("{}:     R_N = {} [m]", nameId(), R_N);

        // Direction Cosine Matrix from body to navigation coordinates, at the time tₖ₋₁
        Eigen::Matrix3d n_Dcm_b = _latestInertialNavSol->n_Quat_b().toRotationMatrix();
        LOG_DATA("{}:     n_Dcm_b =\n{}", nameId(), n_Dcm_b);

        // Conversion matrix between cartesian and curvilinear perturbations to the position
        Eigen::Matrix3d T_rn_p = conversionMatrixCartesianCurvilinear(lla_position, R_N, R_E);
        LOG_DATA("{}:     T_rn_p =\n{}", nameId(), T_rn_p);

        // Skew-symmetric matrix of the Earth-rotation vector in local navigation frame axes
        Eigen::Matrix3d n_Omega_ie = math::skewSymmetricMatrix(_latestInertialNavSol->n_Quat_e() * InsConst::e_omega_ie);
        LOG_DATA("{}:     n_Omega_ie =\n{}", nameId(), n_Omega_ie);

        // 5. Calculate the measurement matrix H_k
        if (_showKalmanFilterOutputPins) { requestOutputValueLock(OUTPUT_PORT_INDEX_H); }

        _kalmanFilter.H = n_measurementMatrix_H(T_rn_p, n_Dcm_b, b_omega_ip, _b_leverArm_InsGnss, n_Omega_ie);

        // 6. Calculate the measurement noise covariance matrix R_k
        if (_showKalmanFilterOutputPins) { requestOutputValueLock(OUTPUT_PORT_INDEX_R); }

        _kalmanFilter.R = n_measurementNoiseCovariance_R(gnssSigmaSquaredLatLonAlt, gnssSigmaSquaredVelocity);

        // 8. Formulate the measurement z_k
        if (_showKalmanFilterOutputPins) { requestOutputValueLock(OUTPUT_PORT_INDEX_z); }

        _kalmanFilter.z = n_measurementInnovation_dz(gnssMeasurement->lla_position(), _latestInertialNavSol->lla_position(),
                                                     gnssMeasurement->n_velocity(), _latestInertialNavSol->n_velocity(),
                                                     T_rn_p, _latestInertialNavSol->n_Quat_b(), _b_leverArm_InsGnss, b_omega_ip, n_Omega_ie);
    }
    else // if (_frame == Frame::ECEF)
    {
        // Direction Cosine Matrix from body to navigation coordinates, at the time tₖ₋₁
        Eigen::Matrix3d e_Dcm_b = _latestInertialNavSol->e_Quat_b().toRotationMatrix();
        LOG_DATA("{}:     e_Dcm_b =\n{}", nameId(), e_Dcm_b);

        // Skew-symmetric matrix of the Earth-rotation vector in local navigation frame axes
        Eigen::Matrix3d e_Omega_ie = math::skewSymmetricMatrix(InsConst::e_omega_ie);
        LOG_DATA("{}:     e_Omega_ie =\n{}", nameId(), e_Omega_ie);

        // 5. Calculate the measurement matrix H_k
        if (_showKalmanFilterOutputPins) { requestOutputValueLock(OUTPUT_PORT_INDEX_H); }

        _kalmanFilter.H = e_measurementMatrix_H(e_Dcm_b, b_omega_ip, _b_leverArm_InsGnss, e_Omega_ie);

        // 6. Calculate the measurement noise covariance matrix R_k
        if (_showKalmanFilterOutputPins) { requestOutputValueLock(OUTPUT_PORT_INDEX_R); }

        _kalmanFilter.R = e_measurementNoiseCovariance_R(gnssSigmaSquaredPosition, gnssSigmaSquaredVelocity);

        // 8. Formulate the measurement z_k
        if (_showKalmanFilterOutputPins) { requestOutputValueLock(OUTPUT_PORT_INDEX_z); }

        _kalmanFilter.z = e_measurementInnovation_dz(gnssMeasurement->e_position(), _latestInertialNavSol->e_position(),
                                                     gnssMeasurement->e_velocity(), _latestInertialNavSol->e_velocity(),
                                                     _latestInertialNavSol->e_Quat_b(), _b_leverArm_InsGnss, b_omega_ip, e_Omega_ie);
    }

    if (_showKalmanFilterOutputPins)
    {
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_H, gnssMeasurement->insTime);
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_R, gnssMeasurement->insTime);
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_z, gnssMeasurement->insTime);
    }
    LOG_DATA("{}:     KF.H =\n{}", nameId(), _kalmanFilter.H);
    LOG_DATA("{}:     KF.R =\n{}", nameId(), _kalmanFilter.R);
    LOG_DATA("{}:     KF.z =\n{}", nameId(), _kalmanFilter.z);

    if (_checkKalmanMatricesRanks)
    {
        Eigen::FullPivLU<Eigen::MatrixXd> lu(_kalmanFilter.H(all, all) * _kalmanFilter.P(all, all) * _kalmanFilter.H(all, all).transpose() + _kalmanFilter.R(all, all));
        auto rank = lu.rank();
        if (rank != _kalmanFilter.H(all, all).rows())
        {
            LOG_WARN("{}: (HPH^T + R).rank = {}", nameId(), rank);
        }
    }

    // 7. Calculate the Kalman gain matrix K_k
    // 9. Update the state vector estimate from x(-) to x(+)
    // 10. Update the error covariance matrix from P(-) to P(+)
    if (_showKalmanFilterOutputPins)
    {
        requestOutputValueLock(OUTPUT_PORT_INDEX_K);
        requestOutputValueLock(OUTPUT_PORT_INDEX_x);
        requestOutputValueLock(OUTPUT_PORT_INDEX_P);
    }

    _kalmanFilter.correctWithMeasurementInnovation();

    if (_showKalmanFilterOutputPins)
    {
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_K, gnssMeasurement->insTime);
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_x, gnssMeasurement->insTime);
        notifyOutputValueChanged(OUTPUT_PORT_INDEX_P, gnssMeasurement->insTime);
    }
    LOG_DATA("{}:     KF.K =\n{}", nameId(), _kalmanFilter.K);
    LOG_DATA("{}:     KF.x =\n{}", nameId(), _kalmanFilter.x);
    LOG_DATA("{}:     KF.P =\n{}", nameId(), _kalmanFilter.P);

    // Averaging of P to avoid numerical problems with symmetry (did not work)
    // _kalmanFilter.P = ((_kalmanFilter.P + _kalmanFilter.P.transpose()) / 2.0);

    if (_checkKalmanMatricesRanks)
    {
        Eigen::FullPivLU<Eigen::MatrixXd> lu(_kalmanFilter.H(all, all) * _kalmanFilter.P(all, all) * _kalmanFilter.H(all, all).transpose() + _kalmanFilter.R(all, all));
        auto rank = lu.rank();
        if (rank != _kalmanFilter.H(all, all).rows())
        {
            LOG_WARN("{}: (HPH^T + R).rank = {}", nameId(), rank);
        }

        Eigen::FullPivLU<Eigen::MatrixXd> luK(_kalmanFilter.K(all, all));
        rank = luK.rank();
        if (rank != _kalmanFilter.K(all, all).cols())
        {
            LOG_WARN("{}: K.rank = {}", nameId(), rank);
        }

        Eigen::FullPivLU<Eigen::MatrixXd> luP(_kalmanFilter.P(all, all));
        rank = luP.rank();
        if (rank != _kalmanFilter.P(all, all).rows())
        {
            LOG_WARN("{}: P.rank = {}", nameId(), rank);
        }
    }

    // LOG_DEBUG("{}: H\n{}\n", nameId(), _kalmanFilter.H);
    // LOG_DEBUG("{}: R\n{}\n", nameId(), _kalmanFilter.R);
    // LOG_DEBUG("{}: z = {}", nameId(), _kalmanFilter.z.transpose());

    // LOG_DEBUG("{}: K\n{}\n", nameId(), _kalmanFilter.K);
    // LOG_DEBUG("{}: x = {}", nameId(), _kalmanFilter.x.transpose());
    // LOG_DEBUG("{}: P\n{}\n", nameId(), _kalmanFilter.P);

    // LOG_DEBUG("{}: K * z = {}", nameId(), (_kalmanFilter.K * _kalmanFilter.z).transpose());

    // LOG_DEBUG("{}: P - P^T\n{}\n", nameId(), _kalmanFilter.P - _kalmanFilter.P.transpose());

    _accumulatedAccelBiases += _kalmanFilter.x.segment<3>(AccBias) * (1. / SCALE_FACTOR_ACCELERATION);
    _accumulatedGyroBiases += _kalmanFilter.x.segment<3>(GyrBias) * (1. / SCALE_FACTOR_ANGULAR_RATE);

    // Push out the new data
    auto lcKfInsGnssErrors = std::make_shared<LcKfInsGnssErrors>();
    lcKfInsGnssErrors->insTime = gnssMeasurement->insTime;
    lcKfInsGnssErrors->positionError = _kalmanFilter.x.segment<3>(Pos);
    lcKfInsGnssErrors->velocityError = _kalmanFilter.x.segment<3>(Vel);
    lcKfInsGnssErrors->attitudeError = _kalmanFilter.x.segment<3>(Att) * (1. / SCALE_FACTOR_ATTITUDE);
    lcKfInsGnssErrors->b_biasAccel = _accumulatedAccelBiases;
    lcKfInsGnssErrors->b_biasGyro = _accumulatedGyroBiases;

    if (_frame == Frame::NED)
    {
        lcKfInsGnssErrors->positionError = lcKfInsGnssErrors->positionError.array() * Eigen::Array3d(1. / SCALE_FACTOR_LAT_LON, 1. / SCALE_FACTOR_LAT_LON, 1);
        lcKfInsGnssErrors->frame = LcKfInsGnssErrors::Frame::NED;
    }
    else // if (_frame == Frame::ECEF)
    {
        lcKfInsGnssErrors->frame = LcKfInsGnssErrors::Frame::ECEF;
    }

    // Closed loop
    if (_showKalmanFilterOutputPins) { requestOutputValueLock(OUTPUT_PORT_INDEX_x); }

    _kalmanFilter.x(all).setZero();

    invokeCallbacks(OUTPUT_PORT_INDEX_ERROR, lcKfInsGnssErrors);
}

// ###########################################################################################################
//                                             System matrix 𝐅
// ###########################################################################################################

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFStates, NAV::LooselyCoupledKF::KFStates, 15, 15>
    NAV::LooselyCoupledKF::n_systemMatrix_F(const Eigen::Quaterniond& n_Quat_b,
                                            const Eigen::Vector3d& b_specForce_ib,
                                            const Eigen::Vector3d& n_omega_in,
                                            const Eigen::Vector3d& n_velocity,
                                            const Eigen::Vector3d& lla_position,
                                            double R_N,
                                            double R_E,
                                            double g_0,
                                            double r_eS_e,
                                            const Eigen::Vector3d& tau_bad,
                                            const Eigen::Vector3d& tau_bgd) const
{
    double latitude = lla_position(0); // Geodetic latitude of the body in [rad]
    double altitude = lla_position(2); // Geodetic height of the body in [m]

    Eigen::Vector3d beta_bad = 1. / tau_bad.array(); // Gauss-Markov constant for the accelerometer 𝛽 = 1 / 𝜏 (𝜏 correlation length)
    Eigen::Vector3d beta_bgd = 1. / tau_bgd.array(); // Gauss-Markov constant for the gyroscope 𝛽 = 1 / 𝜏 (𝜏 correlation length)

    // System matrix 𝐅
    // Math: \mathbf{F}^n = \begin{pmatrix} \mathbf{F}_{\dot{\psi},\psi}^n & \mathbf{F}_{\dot{\psi},\delta v}^n & \mathbf{F}_{\dot{\psi},\delta r}^n & \mathbf{0}_3 & \mathbf{C}_b^n \\ \mathbf{F}_{\delta \dot{v},\psi}^n & \mathbf{F}_{\delta \dot{v},\delta v}^n & \mathbf{F}_{\delta \dot{v},\delta r}^n & \mathbf{C}_b^n & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{F}_{\delta \dot{r},\delta v}^n & \mathbf{F}_{\delta \dot{r},\delta r}^n & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \vee -\mathbf{\beta} & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \vee -\mathbf{\beta} \end{pmatrix}
    KeyedMatrix<double, KFStates, KFStates, 15, 15> F(Eigen::Matrix<double, 15, 15>::Zero(), States);

    F.block<3>(Att, Att) = n_F_dpsi_dpsi(n_omega_in);
    F.block<3>(Att, Vel) = n_F_dpsi_dv(latitude, altitude, R_N, R_E);
    F.block<3>(Att, Pos) = n_F_dpsi_dr(latitude, altitude, n_velocity, R_N, R_E);
    F.block<3>(Att, GyrBias) = n_F_dpsi_dw(n_Quat_b.toRotationMatrix());
    F.block<3>(Vel, Att) = n_F_dv_dpsi(n_Quat_b * b_specForce_ib);
    F.block<3>(Vel, Vel) = n_F_dv_dv(n_velocity, latitude, altitude, R_N, R_E);
    F.block<3>(Vel, Pos) = n_F_dv_dr(n_velocity, latitude, altitude, R_N, R_E, g_0, r_eS_e);
    F.block<3>(Vel, AccBias) = n_F_dv_df(n_Quat_b.toRotationMatrix());
    F.block<3>(Pos, Vel) = n_F_dr_dv(latitude, altitude, R_N, R_E);
    F.block<3>(Pos, Pos) = n_F_dr_dr(n_velocity, latitude, altitude, R_N, R_E);
    if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
    {
        F.block<3>(AccBias, AccBias) = n_F_df_df(_randomProcessAccel == RandomProcess::RandomWalk ? Eigen::Vector3d::Zero() : beta_bad);
        F.block<3>(GyrBias, GyrBias) = n_F_dw_dw(_randomProcessGyro == RandomProcess::RandomWalk ? Eigen::Vector3d::Zero() : beta_bgd);
    }

    F.middleRows<3>(Att) *= SCALE_FACTOR_ATTITUDE; // 𝜓' [deg / s] = 180/π * ... [rad / s]
    F.middleCols<3>(Att) *= 1. / SCALE_FACTOR_ATTITUDE;

    // F.middleRows<3>(Vel) *= 1.; // 𝛿v' [m / s^2] = 1 * [m / s^2]
    // F.middleCols<3>(Vel) *= 1. / 1.;

    F.middleRows<2>({ PosLat, PosLon }) *= SCALE_FACTOR_LAT_LON; // 𝛿ϕ' [pseudometre / s] = R0 * [rad / s]
    F.middleCols<2>({ PosLat, PosLon }) *= 1. / SCALE_FACTOR_LAT_LON;
    // F.row(PosAlt) *= 1.; // 𝛿h' [m / s] = 1 * [m / s]
    // F.col(PosAlt) *= 1. / 1.;

    F.middleRows<3>(AccBias) *= SCALE_FACTOR_ACCELERATION; // 𝛿f' [mg / s] = 1e3 / g * [m / s^3]
    F.middleCols<3>(AccBias) *= 1. / SCALE_FACTOR_ACCELERATION;

    F.middleRows<3>(GyrBias) *= SCALE_FACTOR_ANGULAR_RATE; // 𝛿ω' [mrad / s^2] = 1e3 * [rad / s^2]
    F.middleCols<3>(GyrBias) *= 1. / SCALE_FACTOR_ANGULAR_RATE;

    return F;
}

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFStates, NAV::LooselyCoupledKF::KFStates, 15, 15>
    NAV::LooselyCoupledKF::e_systemMatrix_F(const Eigen::Quaterniond& e_Quat_b,
                                            const Eigen::Vector3d& b_specForce_ib,
                                            const Eigen::Vector3d& e_position,
                                            const Eigen::Vector3d& e_gravitation,
                                            double r_eS_e,
                                            const Eigen::Vector3d& e_omega_ie,
                                            const Eigen::Vector3d& tau_bad,
                                            const Eigen::Vector3d& tau_bgd) const
{
    Eigen::Vector3d beta_bad = 1. / tau_bad.array(); // Gauss-Markov constant for the accelerometer 𝛽 = 1 / 𝜏 (𝜏 correlation length)
    Eigen::Vector3d beta_bgd = 1. / tau_bgd.array(); // Gauss-Markov constant for the gyroscope 𝛽 = 1 / 𝜏 (𝜏 correlation length)

    // System matrix 𝐅
    // Math: \mathbf{F}^e = \begin{pmatrix} \mathbf{F}_{\dot{\psi},\psi}^n & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{C}_b^e \\ \mathbf{F}_{\delta \dot{v},\psi}^n & \mathbf{F}_{\delta \dot{v},\delta v}^n & \mathbf{F}_{\delta \dot{v},\delta r}^n & \mathbf{C}_b^e & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{F}_{\delta \dot{r},\delta v}^n & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \vee -\mathbf{\beta} & \mathbf{0}_3 \\ \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{0}_3 \vee -\mathbf{\beta} \end{pmatrix}
    KeyedMatrix<double, KFStates, KFStates, 15, 15> F(Eigen::Matrix<double, 15, 15>::Zero(), States);

    F.block<3>(Att, Att) = e_F_dpsi_dpsi(e_omega_ie.z());
    F.block<3>(Att, GyrBias) = e_F_dpsi_dw(e_Quat_b.toRotationMatrix());
    F.block<3>(Vel, Att) = e_F_dv_dpsi(e_Quat_b * b_specForce_ib);
    F.block<3>(Vel, Vel) = e_F_dv_dv(e_omega_ie.z());
    F.block<3>(Vel, Pos) = e_F_dv_dr(e_position, e_gravitation, r_eS_e, e_omega_ie);
    F.block<3>(Vel, AccBias) = e_F_dv_df(e_Quat_b.toRotationMatrix());
    F.block<3>(Pos, Vel) = e_F_dr_dv();
    if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
    {
        F.block<3>(AccBias, AccBias) = e_F_df_df(_randomProcessAccel == RandomProcess::RandomWalk ? Eigen::Vector3d::Zero() : beta_bad);
        F.block<3>(GyrBias, GyrBias) = e_F_dw_dw(_randomProcessGyro == RandomProcess::RandomWalk ? Eigen::Vector3d::Zero() : beta_bgd);
    }

    F.middleRows<3>(Att) *= SCALE_FACTOR_ATTITUDE; // 𝜓' [deg / s] = 180/π * ... [rad / s]
    F.middleCols<3>(Att) *= 1. / SCALE_FACTOR_ATTITUDE;

    // F.middleRows<3>(Vel) *= 1.; // 𝛿v' [m / s^2] = 1 * [m / s^2]
    // F.middleCols<3>(Vel) *= 1. / 1.;

    // F.middleRows<3>(Pos) *= 1.; // 𝛿r' [m / s] = 1 * [m / s]
    // F.middleCols<3>(Pos) *= 1. / 1.;

    F.middleRows<3>(AccBias) *= SCALE_FACTOR_ACCELERATION; // 𝛿f' [mg / s] = 1e3 / g * [m / s^3]
    F.middleCols<3>(AccBias) *= 1. / SCALE_FACTOR_ACCELERATION;

    F.middleRows<3>(GyrBias) *= SCALE_FACTOR_ANGULAR_RATE; // 𝛿ω' [mrad / s^2] = 1e3 * [rad / s^2]
    F.middleCols<3>(GyrBias) *= 1. / SCALE_FACTOR_ANGULAR_RATE;

    return F;
}

// ###########################################################################################################
//                                    Noise input matrix 𝐆 & Noise scale matrix 𝐖
//                                     System noise covariance matrix 𝐐
// ###########################################################################################################

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFStates, NAV::LooselyCoupledKF::KFStates, 15, 12>
    NAV::LooselyCoupledKF::noiseInputMatrix_G(const Eigen::Quaterniond& ien_Quat_b)
{
    // DCM matrix from body to navigation frame
    Eigen::Matrix3d ien_Dcm_b = ien_Quat_b.toRotationMatrix();

    // Math: \mathbf{G}_{a} = \begin{bmatrix} -\mathbf{C}_b^{i,e,n} & 0 & 0 & 0 \\ 0 & \mathbf{C}_b^{i,e,n} & 0 & 0 \\ 0 & 0 & 0 & 0 \\ 0 & 0 & \mathbf{I}_3 & 0 \\ 0 & 0 & 0 & \mathbf{I}_3 \end{bmatrix}
    KeyedMatrix<double, KFStates, KFStates, 15, 12> G(Eigen::Matrix<double, 15, 12>::Zero(), States,
                                                      { Roll, Pitch, Yaw,
                                                        VelN, VelE, VelD,
                                                        AccBiasX, AccBiasY, AccBiasZ,
                                                        GyrBiasX, GyrBiasY, GyrBiasZ });

    G.block<3>(Att, Att) = SCALE_FACTOR_ATTITUDE * -ien_Dcm_b;
    G.block<3>(Vel, Vel) = ien_Dcm_b;
    G.block<3>(AccBias, AccBias) = SCALE_FACTOR_ACCELERATION * Eigen::Matrix3d::Identity();
    G.block<3>(GyrBias, GyrBias) = SCALE_FACTOR_ANGULAR_RATE * Eigen::Matrix3d::Identity();

    return G;
}

Eigen::Matrix<double, 12, 12> NAV::LooselyCoupledKF::noiseScaleMatrix_W(const Eigen::Vector3d& sigma_ra, const Eigen::Vector3d& sigma_rg,
                                                                        const Eigen::Vector3d& sigma_bad, const Eigen::Vector3d& sigma_bgd,
                                                                        const Eigen::Vector3d& tau_bad, const Eigen::Vector3d& tau_bgd)
{
    Eigen::Matrix<double, 12, 12> W = Eigen::Matrix<double, 12, 12>::Zero();

    W.diagonal() << sigma_rg,
        sigma_ra,
        _randomProcessAccel == RandomProcess::RandomWalk ? sigma_bad : psdBiasGaussMarkov(sigma_bad.array().square(), tau_bad), // S_bad
        _randomProcessGyro == RandomProcess::RandomWalk ? sigma_bgd : psdBiasGaussMarkov(sigma_bgd.array().square(), tau_bgd);  // S_bgd

    return W;
}

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFStates, NAV::LooselyCoupledKF::KFStates, 15, 15>
    NAV::LooselyCoupledKF::n_systemNoiseCovarianceMatrix_Q(const Eigen::Vector3d& sigma2_ra, const Eigen::Vector3d& sigma2_rg,
                                                           const Eigen::Vector3d& sigma2_bad, const Eigen::Vector3d& sigma2_bgd,
                                                           const Eigen::Vector3d& tau_bad, const Eigen::Vector3d& tau_bgd,
                                                           const Eigen::Matrix3d& n_F_21, const Eigen::Matrix3d& T_rn_p,
                                                           const Eigen::Matrix3d& n_Dcm_b, const double& tau_s)
{
    // Math: \mathbf{Q}_{INS}^n = \begin{pmatrix} \mathbf{Q}_{11} & {\mathbf{Q}_{21}^n}^T & {\mathbf{Q}_{31}^n}^T & \mathbf{0}_3 & {\mathbf{Q}_{51}^n}^T \\ \mathbf{Q}_{21}^n & \mathbf{Q}_{22}^n & {\mathbf{Q}_{32}^n}^T & {\mathbf{Q}_{42}^n}^T & \mathbf{Q}_{25}^n \\ \mathbf{Q}_{31}^n & \mathbf{Q}_{32}^n & \mathbf{Q}_{33}^n & \mathbf{Q}_{34}^n & \mathbf{Q}_{35}^n \\ \mathbf{0}_3 & \mathbf{Q}_{42}^n & {\mathbf{Q}_{34}^n}^T & S_{bad}\tau_s\mathbf{I}_3 & \mathbf{0}_3 \\ \mathbf{Q}_{51}^n & \mathbf{Q}_{52}^n & {\mathbf{Q}_{35}^n}^T & \mathbf{0}_3 & S_{bgd}\tau_s\mathbf{I}_3 \end{pmatrix} \qquad \text{P. Groves}\,(14.80)
    Eigen::Vector3d S_ra = sigma2_ra * tau_s;
    Eigen::Vector3d S_rg = sigma2_rg * tau_s;
    Eigen::Vector3d S_bad = sigma2_bad.array() / tau_bad.array();
    Eigen::Vector3d S_bgd = sigma2_bgd.array() / tau_bgd.array();

    Eigen::Matrix3d b_Dcm_n = n_Dcm_b.transpose();

    KeyedMatrix<double, KFStates, KFStates, 15, 15> Q(Eigen::Matrix<double, 15, 15>::Zero(), States, States);
    Q.block<3>(Att, Att) = Q_psi_psi(S_rg, S_bgd, tau_s);                              // Q_11
    Q.block<3>(Vel, Att) = ien_Q_dv_psi(S_rg, S_bgd, n_F_21, tau_s);                   // Q_21
    Q.block<3>(Vel, Vel) = ien_Q_dv_dv(S_ra, S_bad, S_rg, S_bgd, n_F_21, tau_s);       // Q_22
    Q.block<3>(Vel, GyrBias) = ien_Q_dv_domega(S_bgd, n_F_21, n_Dcm_b, tau_s);         // Q_25
    Q.block<3>(Pos, Att) = n_Q_dr_psi(S_rg, S_bgd, n_F_21, T_rn_p, tau_s);             // Q_31
    Q.block<3>(Pos, Vel) = n_Q_dr_dv(S_ra, S_bad, S_rg, S_bgd, n_F_21, T_rn_p, tau_s); // Q_32
    Q.block<3>(Pos, Pos) = n_Q_dr_dr(S_ra, S_bad, S_rg, S_bgd, n_F_21, T_rn_p, tau_s); // Q_33
    Q.block<3>(Pos, AccBias) = n_Q_dr_df(S_bgd, T_rn_p, n_Dcm_b, tau_s);               // Q_34
    Q.block<3>(Pos, GyrBias) = n_Q_dr_domega(S_bgd, n_F_21, T_rn_p, n_Dcm_b, tau_s);   // Q_35
    Q.block<3>(AccBias, Vel) = Q_df_dv(S_bad, b_Dcm_n, tau_s);                         // Q_42
    Q.block<3>(AccBias, AccBias) = Q_df_df(S_bad, tau_s);                              // Q_44
    Q.block<3>(GyrBias, Att) = Q_domega_psi(S_bgd, b_Dcm_n, tau_s);                    // Q_51
    Q.block<3>(GyrBias, GyrBias) = Q_domega_domega(S_bgd, tau_s);                      // Q_55

    Q.block<3>(Att, Vel) = Q.block<3>(Vel, Att).transpose();         // Q_21^T
    Q.block<3>(Att, Pos) = Q.block<3>(Pos, Att).transpose();         // Q_31^T
    Q.block<3>(Vel, Pos) = Q.block<3>(Pos, Vel).transpose();         // Q_32^T
    Q.block<3>(AccBias, Pos) = Q.block<3>(Pos, AccBias).transpose(); // Q_34^T
    Q.block<3>(GyrBias, Vel) = Q.block<3>(Vel, GyrBias).transpose(); // Q_25^T
    Q.block<3>(GyrBias, Pos) = Q.block<3>(Pos, GyrBias).transpose(); // Q_35^T
    Q.block<3>(Vel, AccBias) = Q.block<3>(AccBias, Vel).transpose(); // Q_42^T
    Q.block<3>(Att, GyrBias) = Q.block<3>(GyrBias, Att).transpose(); // Q_51^T

    Q.middleRows<3>(Att) *= SCALE_FACTOR_ATTITUDE;
    Q.middleRows<2>({ PosLat, PosLon }) *= SCALE_FACTOR_LAT_LON;
    Q.middleRows<3>(AccBias) *= SCALE_FACTOR_ACCELERATION;
    Q.middleRows<3>(GyrBias) *= SCALE_FACTOR_ANGULAR_RATE;

    Q.middleCols<3>(Att) *= SCALE_FACTOR_ATTITUDE;
    Q.middleCols<2>({ PosLat, PosLon }) *= SCALE_FACTOR_LAT_LON;
    Q.middleCols<3>(AccBias) *= SCALE_FACTOR_ACCELERATION;
    Q.middleCols<3>(GyrBias) *= SCALE_FACTOR_ANGULAR_RATE;

    return Q;
}

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFStates, NAV::LooselyCoupledKF::KFStates, 15, 15>
    NAV::LooselyCoupledKF::e_systemNoiseCovarianceMatrix_Q(const Eigen::Vector3d& sigma2_ra, const Eigen::Vector3d& sigma2_rg,
                                                           const Eigen::Vector3d& sigma2_bad, const Eigen::Vector3d& sigma2_bgd,
                                                           const Eigen::Vector3d& tau_bad, const Eigen::Vector3d& tau_bgd,
                                                           const Eigen::Matrix3d& e_F_21,
                                                           const Eigen::Matrix3d& e_Dcm_b, const double& tau_s)
{
    // Math: \mathbf{Q}_{INS}^e = \begin{pmatrix} \mathbf{Q}_{11} & {\mathbf{Q}_{21}^e}^T & {\mathbf{Q}_{31}^e}^T & \mathbf{0}_3 & {\mathbf{Q}_{51}^e}^T \\ \mathbf{Q}_{21}^e & \mathbf{Q}_{22}^e & {\mathbf{Q}_{32}^e}^T & {\mathbf{Q}_{42}^e}^T & \mathbf{Q}_{25}^e \\ \mathbf{Q}_{31}^e & \mathbf{Q}_{32}^e & \mathbf{Q}_{33}^e & \mathbf{Q}_{34}^e & \mathbf{Q}_{35}^e \\ \mathbf{0}_3 & \mathbf{Q}_{42}^e & {\mathbf{Q}_{34}^e}^T & S_{bad}\tau_s\mathbf{I}_3 & \mathbf{0}_3 \\ \mathbf{Q}_{51}^e & \mathbf{Q}_{52}^e & {\mathbf{Q}_{35}^e}^T & \mathbf{0}_3 & S_{bgd}\tau_s\mathbf{I}_3 \end{pmatrix} \qquad \text{P. Groves}\,(14.80)
    Eigen::Vector3d S_ra = sigma2_ra * tau_s;
    Eigen::Vector3d S_rg = sigma2_rg * tau_s;
    Eigen::Vector3d S_bad = sigma2_bad.array() / tau_bad.array();
    Eigen::Vector3d S_bgd = sigma2_bgd.array() / tau_bgd.array();

    Eigen::Matrix3d b_Dcm_e = e_Dcm_b.transpose();

    KeyedMatrix<double, KFStates, KFStates, 15, 15> Q(Eigen::Matrix<double, 15, 15>::Zero(), States, States);
    Q.block<3>(Att, Att) = Q_psi_psi(S_rg, S_bgd, tau_s);                        // Q_11
    Q.block<3>(Vel, Att) = ien_Q_dv_psi(S_rg, S_bgd, e_F_21, tau_s);             // Q_21
    Q.block<3>(Vel, Vel) = ien_Q_dv_dv(S_ra, S_bad, S_rg, S_bgd, e_F_21, tau_s); // Q_22
    Q.block<3>(Vel, GyrBias) = ien_Q_dv_domega(S_bgd, e_F_21, e_Dcm_b, tau_s);   // Q_25
    Q.block<3>(Pos, Att) = ie_Q_dr_psi(S_rg, S_bgd, e_F_21, tau_s);              // Q_31
    Q.block<3>(Pos, Vel) = ie_Q_dr_dv(S_ra, S_bad, S_rg, S_bgd, e_F_21, tau_s);  // Q_32
    Q.block<3>(Pos, Pos) = ie_Q_dr_dr(S_ra, S_bad, S_rg, S_bgd, e_F_21, tau_s);  // Q_33
    Q.block<3>(Pos, AccBias) = ie_Q_dr_df(S_bgd, e_Dcm_b, tau_s);                // Q_34
    Q.block<3>(Pos, GyrBias) = ie_Q_dr_domega(S_bgd, e_F_21, e_Dcm_b, tau_s);    // Q_35
    Q.block<3>(AccBias, Vel) = Q_df_dv(S_bad, b_Dcm_e, tau_s);                   // Q_42
    Q.block<3>(AccBias, AccBias) = Q_df_df(S_bad, tau_s);                        // Q_44
    Q.block<3>(GyrBias, Att) = Q_domega_psi(S_bgd, b_Dcm_e, tau_s);              // Q_51
    Q.block<3>(GyrBias, GyrBias) = Q_domega_domega(S_bgd, tau_s);                // Q_55

    Q.block<3>(Att, Vel) = Q.block<3>(Vel, Att).transpose();         // Q_21^T
    Q.block<3>(Att, Pos) = Q.block<3>(Pos, Att).transpose();         // Q_31^T
    Q.block<3>(Vel, Pos) = Q.block<3>(Pos, Vel).transpose();         // Q_32^T
    Q.block<3>(AccBias, Pos) = Q.block<3>(Pos, AccBias).transpose(); // Q_34^T
    Q.block<3>(GyrBias, Vel) = Q.block<3>(Vel, GyrBias).transpose(); // Q_25^T
    Q.block<3>(GyrBias, Pos) = Q.block<3>(Pos, GyrBias).transpose(); // Q_35^T
    Q.block<3>(Vel, AccBias) = Q.block<3>(AccBias, Vel).transpose(); // Q_42^T
    Q.block<3>(Att, GyrBias) = Q.block<3>(GyrBias, Att).transpose(); // Q_51^T

    Q.middleRows<3>(Att) *= SCALE_FACTOR_ATTITUDE;
    Q.middleRows<3>(AccBias) *= SCALE_FACTOR_ACCELERATION;
    Q.middleRows<3>(GyrBias) *= SCALE_FACTOR_ANGULAR_RATE;

    Q.middleCols<3>(Att) *= SCALE_FACTOR_ATTITUDE;
    Q.middleCols<3>(AccBias) *= SCALE_FACTOR_ACCELERATION;
    Q.middleCols<3>(GyrBias) *= SCALE_FACTOR_ANGULAR_RATE;

    return Q;
}

// ###########################################################################################################
//                                         Error covariance matrix P
// ###########################################################################################################

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFStates, NAV::LooselyCoupledKF::KFStates, 15, 15>
    NAV::LooselyCoupledKF::initialErrorCovarianceMatrix_P0(const Eigen::Vector3d& variance_angles,
                                                           const Eigen::Vector3d& variance_vel,
                                                           const Eigen::Vector3d& variance_pos,
                                                           const Eigen::Vector3d& variance_accelBias,
                                                           const Eigen::Vector3d& variance_gyroBias) const
{
    double scaleFactorPosition = _frame == Frame::NED ? SCALE_FACTOR_LAT_LON : 1.0;

    // 𝐏 Error covariance matrix
    Eigen::Matrix<double, 15, 15> P = Eigen::Matrix<double, 15, 15>::Zero();

    P.diagonal() << std::pow(SCALE_FACTOR_ATTITUDE, 2) * variance_angles, // Flight Angles covariance
        variance_vel,                                                     // Velocity covariance
        std::pow(scaleFactorPosition, 2) * variance_pos(0),               // Latitude/Pos X covariance
        std::pow(scaleFactorPosition, 2) * variance_pos(1),               // Longitude/Pos Y covariance
        variance_pos(2),                                                  // Altitude/Pos Z covariance
        std::pow(SCALE_FACTOR_ACCELERATION, 2) * variance_accelBias,      // Accelerometer Bias covariance
        std::pow(SCALE_FACTOR_ANGULAR_RATE, 2) * variance_gyroBias;       // Gyroscope Bias covariance

    return { P, States };
}

// ###########################################################################################################
//                                                Correction
// ###########################################################################################################

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFMeas, NAV::LooselyCoupledKF::KFStates, 6, 15>
    NAV::LooselyCoupledKF::n_measurementMatrix_H(const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& n_Dcm_b, const Eigen::Vector3d& b_omega_ib, const Eigen::Vector3d& b_leverArm_InsGnss, const Eigen::Matrix3d& n_Omega_ie)
{
    // Math: \mathbf{H}_{G,k}^n = \begin{pmatrix} \mathbf{H}_{r1}^n & \mathbf{0}_3 & -\mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{H}_{v1}^n & -\mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{H}_{v5}^n \end{pmatrix}_k \qquad \text{P. Groves}\,(14.113)
    // G denotes GNSS indicated

    NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFMeas, NAV::LooselyCoupledKF::KFStates, 6, 15> H(Eigen::Matrix<double, 6, 15>::Zero(), Meas, States);

    // Math: \mathbf{H}_{r1}^n \approx \mathbf{\hat{T}}_{r(n)}^p \begin{bmatrix} \begin{pmatrix} \mathbf{C}_b^n \mathbf{l}_{ba}^p \end{pmatrix} \wedge \end{bmatrix} \qquad \text{P. Groves}\,(14.114)
    H.block<3>(dPos, Att) = T_rn_p * math::skewSymmetricMatrix(n_Dcm_b * b_leverArm_InsGnss);
    H.block<3>(dPos, Pos) = -Eigen::Matrix3d::Identity();
    // Math: \mathbf{H}_{v1}^n \approx \begin{bmatrix} \begin{Bmatrix} \mathbf{C}_b^n (\mathbf{\hat{\omega}}_{ib}^b \wedge \mathbf{l}_{ba}^b) - \mathbf{\hat{\Omega}}_{ie}^n \mathbf{C}_b^n \mathbf{l}_{ba}^b \end{Bmatrix} \wedge \end{bmatrix} \qquad \text{P. Groves}\,(14.114)
    H.block<3>(dVel, Att) = math::skewSymmetricMatrix(n_Dcm_b * (b_omega_ib.cross(b_leverArm_InsGnss)) - n_Omega_ie * n_Dcm_b * b_leverArm_InsGnss);
    H.block<3>(dVel, Vel) = -Eigen::Matrix3d::Identity();
    // Math: \mathbf{H}_{v5}^n = \mathbf{C}_b^n \begin{bmatrix} \mathbf{l}_{ba}^b \wedge \end{bmatrix} \qquad \text{P. Groves}\,(14.114)
    H.block<3>(dVel, GyrBias) = n_Dcm_b * math::skewSymmetricMatrix(b_leverArm_InsGnss);

    H.middleRows<2>({ dPosLat, dPosLon }) *= SCALE_FACTOR_LAT_LON;

    H.middleCols<3>(Att) *= 1. / SCALE_FACTOR_ATTITUDE;
    H.middleCols<2>({ PosLat, PosLon }) *= 1. / SCALE_FACTOR_LAT_LON;
    // H.middleCols<3>(AccBias) *= 1. / SCALE_FACTOR_ACCELERATION; // Only zero elements
    H.middleCols<3>(GyrBias) *= 1. / SCALE_FACTOR_ANGULAR_RATE;

    return H;
}

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFMeas, NAV::LooselyCoupledKF::KFStates, 6, 15>
    NAV::LooselyCoupledKF::e_measurementMatrix_H(const Eigen::Matrix3d& e_Dcm_b, const Eigen::Vector3d& b_omega_ib, const Eigen::Vector3d& b_leverArm_InsGnss, const Eigen::Matrix3d& e_Omega_ie)
{
    // Math: \mathbf{H}_{G,k}^e = \begin{pmatrix} \mathbf{H}_{r1}^e & \mathbf{0}_3 & -\mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{H}_{v1}^e & -\mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{H}_{v5}^e \end{pmatrix}_k \qquad \text{P. Groves}\,(14.113)
    // G denotes GNSS indicated

    NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFMeas, NAV::LooselyCoupledKF::KFStates, 6, 15> H(Eigen::Matrix<double, 6, 15>::Zero(), Meas, States);

    // Math: \mathbf{H}_{r1}^e \approx \begin{bmatrix} \begin{pmatrix} \mathbf{C}_b^e \mathbf{l}_{ba}^p \end{pmatrix} \wedge \end{bmatrix} \qquad \text{P. Groves}\,(14.114)
    H.block<3>(dPos, Att) = math::skewSymmetricMatrix(e_Dcm_b * b_leverArm_InsGnss);
    H.block<3>(dPos, Pos) = -Eigen::Matrix3d::Identity();
    // Math: \mathbf{H}_{v1}^e \approx \begin{bmatrix} \begin{Bmatrix} \mathbf{C}_b^e (\mathbf{\hat{\omega}}_{ib}^b \wedge \mathbf{l}_{ba}^b) - \mathbf{\hat{\Omega}}_{ie}^e \mathbf{C}_b^e \mathbf{l}_{ba}^b \end{Bmatrix} \wedge \end{bmatrix} \qquad \text{P. Groves}\,(14.114)
    H.block<3>(dVel, Att) = math::skewSymmetricMatrix(e_Dcm_b * (b_omega_ib.cross(b_leverArm_InsGnss)) - e_Omega_ie * e_Dcm_b * b_leverArm_InsGnss);
    H.block<3>(dVel, Vel) = -Eigen::Matrix3d::Identity();
    // Math: \mathbf{H}_{v5}^e = \mathbf{C}_b^e \begin{bmatrix} \mathbf{l}_{ba}^b \wedge \end{bmatrix} \qquad \text{P. Groves}\,(14.114)
    H.block<3>(dVel, GyrBias) = e_Dcm_b * math::skewSymmetricMatrix(b_leverArm_InsGnss);

    H.middleCols<3>(Att) *= 1. / SCALE_FACTOR_ATTITUDE;
    // H.middleCols<3>(AccBias) *= 1. / SCALE_FACTOR_ACCELERATION; // Only zero elements
    H.middleCols<3>(GyrBias) *= 1. / SCALE_FACTOR_ANGULAR_RATE;

    return H;
}

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFMeas, NAV::LooselyCoupledKF::KFMeas, 6, 6>
    NAV::LooselyCoupledKF::n_measurementNoiseCovariance_R(const Eigen::Vector3d& gnssVarianceLatLonAlt, const Eigen::Vector3d& gnssVarianceVelocity)
{
    // Math: \mathbf{R} = \begin{pmatrix} \sigma^2_\phi & 0 & 0 & 0 & 0 & 0 \\ 0 & \sigma^2_\lambda & 0 & 0 & 0 & 0 \\ 0 & 0 & \sigma^2_h & 0 & 0 & 0 \\ 0 & 0 & 0 & \sigma^2_{v_N} & 0 & 0 \\ 0 & 0 & 0 & 0 & \sigma^2_{v_E} & 0 \\ 0 & 0 & 0 & 0 & 0 & \sigma^2_{v_D} \end{pmatrix}
    KeyedMatrix<double, KFMeas, KFMeas, 6, 6> R(Eigen::Matrix<double, 6, 6>::Zero(), Meas);
    R.block<3>(dPos, dPos).diagonal() = gnssVarianceLatLonAlt;
    R.block<3>(dVel, dVel).diagonal() = gnssVarianceVelocity;

    R.block<2>({ dPosLat, dPosLon }, { dPosLat, dPosLon }).diagonal() *= std::pow(SCALE_FACTOR_LAT_LON, 2);

    return R;
}

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFMeas, NAV::LooselyCoupledKF::KFMeas, 6, 6>
    NAV::LooselyCoupledKF::e_measurementNoiseCovariance_R(const Eigen::Vector3d& gnssVariancePosition, const Eigen::Vector3d& gnssVarianceVelocity)
{
    // Math: \mathbf{R} = \begin{pmatrix} \sigma^2_x & 0 & 0 & 0 & 0 & 0 \\ 0 & \sigma^2_y & 0 & 0 & 0 & 0 \\ 0 & 0 & \sigma^2_z & 0 & 0 & 0 \\ 0 & 0 & 0 & \sigma^2_{v_x} & 0 & 0 \\ 0 & 0 & 0 & 0 & \sigma^2_{v_y} & 0 \\ 0 & 0 & 0 & 0 & 0 & \sigma^2_{v_z} \end{pmatrix}
    KeyedMatrix<double, KFMeas, KFMeas, 6, 6> R(Eigen::Matrix<double, 6, 6>::Zero(), Meas);
    R.block<3>(dPos, dPos).diagonal() = gnssVariancePosition;
    R.block<3>(dVel, dVel).diagonal() = gnssVarianceVelocity;

    return R;
}

NAV::KeyedVector<double, NAV::LooselyCoupledKF::KFMeas, 6>
    NAV::LooselyCoupledKF::n_measurementInnovation_dz(const Eigen::Vector3d& lla_positionMeasurement, const Eigen::Vector3d& lla_positionEstimate,
                                                      const Eigen::Vector3d& n_velocityMeasurement, const Eigen::Vector3d& n_velocityEstimate,
                                                      const Eigen::Matrix3d& T_rn_p, const Eigen::Quaterniond& n_Quat_b, const Eigen::Vector3d& b_leverArm_InsGnss,
                                                      const Eigen::Vector3d& b_omega_ib, const Eigen::Matrix3d& n_Omega_ie)
{
    // Math: \delta\mathbf{z}_{G,k}^{n-} = \begin{pmatrix} \mathbf{\hat{p}}_{aG} - \mathbf{\hat{p}}_b - \mathbf{\hat{T}}_{r(n)}^p \mathbf{C}_b^n \mathbf{l}_{ba}^b \\ \mathbf{\hat{v}}_{eaG}^n - \mathbf{\hat{v}}_{eb}^n - \mathbf{C}_b^n (\mathbf{\hat{\omega}}_{ib}^b \wedge \mathbf{l}_{ba}^b) + \mathbf{\hat{\Omega}}_{ie}^n \mathbf{C}_b^n \mathbf{l}_{ba}^b \end{pmatrix}_k \qquad \text{P. Groves}\,(14.103)
    Eigen::Vector3d deltaLLA = lla_positionMeasurement - lla_positionEstimate - T_rn_p * (n_Quat_b * b_leverArm_InsGnss);
    Eigen::Vector3d deltaVel = n_velocityMeasurement - n_velocityEstimate - n_Quat_b * (b_omega_ib.cross(b_leverArm_InsGnss)) + n_Omega_ie * (n_Quat_b * b_leverArm_InsGnss);

    deltaLLA.topRows<2>() *= SCALE_FACTOR_LAT_LON;

    Eigen::Matrix<double, 6, 1> innovation;
    innovation << deltaLLA, deltaVel;

    return { innovation, Meas };
}

NAV::KeyedVector<double, NAV::LooselyCoupledKF::KFMeas, 6>
    NAV::LooselyCoupledKF::e_measurementInnovation_dz(const Eigen::Vector3d& e_positionMeasurement, const Eigen::Vector3d& e_positionEstimate,
                                                      const Eigen::Vector3d& e_velocityMeasurement, const Eigen::Vector3d& e_velocityEstimate,
                                                      const Eigen::Quaterniond& e_Quat_b, const Eigen::Vector3d& b_leverArm_InsGnss,
                                                      const Eigen::Vector3d& b_omega_ib, const Eigen::Matrix3d& e_Omega_ie)
{
    // Math: \delta\mathbf{z}_{G,k}^{e-} = \begin{pmatrix} \mathbf{\hat{r}}_{eaG}^e - \mathbf{\hat{r}}_{eb}^e - \mathbf{C}_b^e \mathbf{l}_{ba}^b \\ \mathbf{\hat{v}}_{eaG}^e - \mathbf{\hat{v}}_{eb}^e - \mathbf{C}_b^e (\mathbf{\hat{\omega}}_{ib}^b \wedge \mathbf{l}_{ba}^b) + \mathbf{\hat{\Omega}}_{ie}^e \mathbf{C}_b^e \mathbf{l}_{ba}^b \end{pmatrix}_k \qquad \text{P. Groves}\,(14.102)
    Eigen::Vector3d deltaPos = e_positionMeasurement - e_positionEstimate - e_Quat_b * b_leverArm_InsGnss;
    Eigen::Vector3d deltaVel = e_velocityMeasurement - e_velocityEstimate - e_Quat_b * (b_omega_ib.cross(b_leverArm_InsGnss)) + e_Omega_ie * (e_Quat_b * b_leverArm_InsGnss);

    Eigen::Matrix<double, 6, 1> innovation;
    innovation << deltaPos, deltaVel;

    return { innovation, Meas };
}