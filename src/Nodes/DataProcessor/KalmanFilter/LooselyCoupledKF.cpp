// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "LooselyCoupledKF.hpp"

#include "NodeData/State/PosVel.hpp"
#include "internal/Node/Pin.hpp"
#include "util/Eigen.hpp"
#include <numbers>
#include <cmath>

#include <imgui.h>
#include <imgui_internal.h>
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

#include "internal/FlowManager.hpp"
#include "internal/NodeManager.hpp"
#include <fmt/format.h>
namespace nm = NAV::NodeManager;
#include "NodeRegistry.hpp"
#include "Navigation/Constants.hpp"
#include "Navigation/Ellipsoid/Ellipsoid.hpp"
#include "Navigation/INS/Functions.hpp"
#include "Navigation/INS/ProcessNoise.hpp"
#include "Navigation/INS/EcefFrame/ErrorEquations.hpp"
#include "Navigation/INS/LocalNavFrame/ErrorEquations.hpp"
#include "Navigation/Math/Math.hpp"
#include "Navigation/Gravity/Gravity.hpp"
#include "Navigation/Transformations/Units.hpp"
#include "util/Logger.hpp"
#include "util/Assert.h"

#include "NodeData/IMU/ImuObsWDelta.hpp"
#include "NodeData/State/PosVelAtt.hpp"
#include "NodeData/State/InsGnssLCKFSolution.hpp"
#include "NodeData/Baro/BaroHgt.hpp"

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
    _guiConfigDefaultWindowSize = { 822, 936 };

    nm::CreateInputPin(
        this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type(), NAV::ImuObsWDelta::type() }, &LooselyCoupledKF::recvImuObservation,
        [](const Node* node, const InputPin& inputPin) {
            const auto* lckf = static_cast<const LooselyCoupledKF*>(node); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
            return !inputPin.queue.empty() && lckf->_inertialIntegrator.hasInitialPosition();
        },
        1); // Priority 1 ensures, that the IMU obs (prediction) is evaluated before the PosVel obs (update)
    nm::CreateInputPin(
        this, "PosVel", Pin::Type::Flow, { NAV::PosVel::type() }, &LooselyCoupledKF::recvPosVelObservation,
        [](const Node* node, const InputPin& inputPin) {
            const auto* lckf = static_cast<const LooselyCoupledKF*>(node); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
            return !inputPin.queue.empty() && (!lckf->_initializeStateOverExternalPin || lckf->_inertialIntegrator.hasInitialPosition());
        },
        2); // Initially this has higher priority than the IMU obs, to initialize the position from it

    nm::CreateOutputPin(this, "PosVelAtt", Pin::Type::Flow, { NAV::InsGnssLCKFSolution::type() });
}

NAV::LooselyCoupledKF::~LooselyCoupledKF()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::LooselyCoupledKF::typeStatic()
{
    return "INS/GNSS LCKF"; // Loosely-coupled Kalman Filter
}

std::string NAV::LooselyCoupledKF::type() const
{
    return typeStatic();
}

std::string NAV::LooselyCoupledKF::category()
{
    return "Data Processor";
}

void NAV::LooselyCoupledKF::updateInputPins()
{
    size_t pinIdx = INPUT_PORT_INDEX_POS_VEL_ATT_INIT;

    auto updatePin = [&](bool pinExists, bool enabled,
                         const char* pinName, Pin::Type pinType, const std::vector<std::string>& dataIdentifier = {},
                         auto callback = nullptr, InputPin::FlowFirableCheckFunc firable = nullptr, int priority = 0) {
        if (!pinExists && enabled)
        {
            nm::CreateInputPin(this, pinName, pinType, dataIdentifier, callback,
                               firable, priority, static_cast<int>(pinIdx));
        }
        else if (pinExists && !enabled)
        {
            nm::DeleteInputPin(inputPins.at(pinIdx));
        }
        if (enabled) { pinIdx++; }
    };

    updatePin(std::ranges::any_of(inputPins, [](const InputPin& pin) { return pin.dataIdentifier.front() == PosVelAtt::type(); }), _initializeStateOverExternalPin,
              "Init PVA", Pin::Type::Flow, { NAV::PosVelAtt::type() }, &LooselyCoupledKF::recvPosVelAttInit, nullptr, 3);
    updatePin(std::ranges::any_of(inputPins, [](const InputPin& pin) { return pin.dataIdentifier.front() == BaroHgt::type(); }), _enableBaroHgt,
              "BaroHgt", Pin::Type::Flow, { NAV::BaroHgt::type() }, &LooselyCoupledKF::recvBaroHeight, nullptr, -1);
}

void NAV::LooselyCoupledKF::guiConfig()
{
    float configWidth = 400.0F * gui::NodeEditorApplication::windowFontRatio();
    float unitWidth = 150.0F * gui::NodeEditorApplication::windowFontRatio();

    float taylorOrderWidth = 75.0F * gui::NodeEditorApplication::windowFontRatio();

    if (ImGui::CollapsingHeader(fmt::format("Initialization##{}", size_t(id)).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
    {
        if (ImGui::Checkbox(fmt::format("Initialize over pin##{}", size_t(id)).c_str(), &_initializeStateOverExternalPin))
        {
            updateInputPins();
            flow::ApplyChanges();
        }
        if (!_initializeStateOverExternalPin)
        {
            ImGui::SetNextItemWidth(80 * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::DragDouble(fmt::format("##initalRollPitchYaw(0) {}", size_t(id)).c_str(),
                                  _initalRollPitchYaw.data(), 1.0F, -180.0, 180.0, "%.3f °"))
            {
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(80 * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::DragDouble(fmt::format("##initalRollPitchYaw(1) {}", size_t(id)).c_str(),
                                  &_initalRollPitchYaw[1], 1.0F, -90.0, 90.0, "%.3f °"))
            {
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(80 * gui::NodeEditorApplication::windowFontRatio());
            if (ImGui::DragDouble(fmt::format("##initalRollPitchYaw(2) {}", size_t(id)).c_str(),
                                  &_initalRollPitchYaw[2], 1.0, -180.0, 180.0, "%.3f °"))
            {
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            ImGui::TextUnformatted("Roll, Pitch, Yaw");
        }
    }

    if (ImGui::CollapsingHeader(fmt::format("IMU Integrator settings##{}", size_t(id)).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
    {
        if (InertialIntegratorGui(std::to_string(size_t(id)).c_str(), _inertialIntegrator))
        {
            flow::ApplyChanges();
        }
        if (inputPins.at(INPUT_PORT_INDEX_IMU).isPinLinked()
            && NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(inputPins.at(INPUT_PORT_INDEX_IMU).link.getConnectedPin()->dataIdentifier, { ImuObsWDelta::type() }))
        {
            ImGui::Separator();
            if (ImGui::Checkbox(fmt::format("Prefer raw measurements over delta##{}", size_t(id)).c_str(), &_preferAccelerationOverDeltaMeasurements))
            {
                flow::ApplyChanges();
            }
        }
    }
    if (ImGui::CollapsingHeader(fmt::format("Kalman Filter settings##{}", size_t(id)).c_str(), ImGuiTreeNodeFlags_DefaultOpen))
    {
        if (_phiCalculationAlgorithm == PhiCalculationAlgorithm::Taylor)
        {
            ImGui::SetNextItemWidth(configWidth - taylorOrderWidth);
        }
        else
        {
            ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
        }
        if (auto phiCalculationAlgorithm = static_cast<int>(_phiCalculationAlgorithm);
            ImGui::Combo(fmt::format("##Phi calculation algorithm {}", size_t(id)).c_str(), &phiCalculationAlgorithm, "Van Loan\0Taylor\0\0"))
        {
            _phiCalculationAlgorithm = static_cast<decltype(_phiCalculationAlgorithm)>(phiCalculationAlgorithm);
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
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() - ImGui::GetStyle().ItemSpacing.x + ImGui::GetStyle().ItemInnerSpacing.x);
        ImGui::Text("Phi calculation algorithm%s", _phiCalculationAlgorithm == PhiCalculationAlgorithm::Taylor ? " (up to order)" : "");

        ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
        if (auto qCalculationAlgorithm = static_cast<int>(_qCalculationAlgorithm);
            ImGui::Combo(fmt::format("Q calculation algorithm##{}", size_t(id)).c_str(), &qCalculationAlgorithm, "Van Loan\0Taylor 1st Order (Groves 2013)\0\0"))
        {
            _qCalculationAlgorithm = static_cast<decltype(_qCalculationAlgorithm)>(qCalculationAlgorithm);
            LOG_DEBUG("{}: Q calculation algorithm changed to {}", nameId(), fmt::underlying(_qCalculationAlgorithm));
            flow::ApplyChanges();
        }
        if (ImGui::Checkbox(fmt::format("Enable barometric height pin##{}", size_t(id)).c_str(), &_enableBaroHgt))
        {
            updateInputPins();
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
                if (auto randomProcessAccel = static_cast<int>(_randomProcessAccel);
                    ImGui::Combo(fmt::format("Random Process Accelerometer##{}", size_t(id)).c_str(), &randomProcessAccel, "Random Walk\0"
                                                                                                                           "Gauss-Markov 1st Order\0\0"))
                {
                    _randomProcessAccel = static_cast<decltype(_randomProcessAccel)>(randomProcessAccel);
                    LOG_DEBUG("{}: randomProcessAccel changed to {}", nameId(), fmt::underlying(_randomProcessAccel));
                    flow::ApplyChanges();
                }
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Standard deviation of the noise on the\naccelerometer specific-force measurements##{}", size_t(id)).c_str(),
                                                   configWidth, unitWidth, _stdev_ra.data(), _stdevAccelNoiseUnits, "mg/√(Hz)\0m/s^2/√(Hz)\0\0",
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
                                                   configWidth, unitWidth, _stdev_bad.data(), _stdevAccelBiasUnits, "µg\0m/s^2\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: stdev_bad changed to {}", nameId(), _stdev_bad.transpose());
                LOG_DEBUG("{}: stdevAccelBiasUnits changed to {}", nameId(), fmt::underlying(_stdevAccelBiasUnits));
                flow::ApplyChanges();
            }

            if (_randomProcessAccel == RandomProcess::GaussMarkov1 || _qCalculationAlgorithm == QCalculationAlgorithm::Taylor1)
            {
                int unitCorrelationLength = 0;
                if (gui::widgets::InputDouble3LWithUnit(fmt::format("Correlation length τ of the accel {}##Correlation length τ of the accel dynamic bias {}",
                                                                    _qCalculationAlgorithm == QCalculationAlgorithm::VanLoan ? "bias noise" : "dynamic bias", size_t(id))
                                                            .c_str(),
                                                        configWidth, unitWidth, _tau_bad.data(), 0., std::numeric_limits<double>::max(),
                                                        unitCorrelationLength, "s\0\0", "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: tau_bad changed to {}", nameId(), _tau_bad);
                    flow::ApplyChanges();
                }
            }

            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 20.F);

            // ----------------------------------------------- Gyroscope -------------------------------------------------

            if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
            {
                ImGui::SetNextItemWidth(configWidth + ImGui::GetStyle().ItemSpacing.x);
                if (auto randomProcessGyro = static_cast<int>(_randomProcessGyro);
                    ImGui::Combo(fmt::format("Random Process Gyroscope##{}", size_t(id)).c_str(), &randomProcessGyro, "Random Walk\0"
                                                                                                                      "Gauss-Markov 1st Order\0\0"))
                {
                    _randomProcessGyro = static_cast<decltype(_randomProcessGyro)>(randomProcessGyro);
                    LOG_DEBUG("{}: randomProcessGyro changed to {}", nameId(), fmt::underlying(_randomProcessGyro));
                    flow::ApplyChanges();
                }
            }

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Standard deviation of the noise on\nthe gyro angular-rate measurements##{}", size_t(id)).c_str(),
                                                   configWidth, unitWidth, _stdev_rg.data(), _stdevGyroNoiseUnits, "deg/hr/√(Hz)\0rad/s/√(Hz)\0\0",
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
                                                   configWidth, unitWidth, _stdev_bgd.data(), _stdevGyroBiasUnits, "°/h\0rad/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: stdev_bgd changed to {}", nameId(), _stdev_bgd.transpose());
                LOG_DEBUG("{}: stdevGyroBiasUnits changed to {}", nameId(), fmt::underlying(_stdevGyroBiasUnits));
                flow::ApplyChanges();
            }

            if (_randomProcessGyro == RandomProcess::GaussMarkov1 || _qCalculationAlgorithm == QCalculationAlgorithm::Taylor1)
            {
                int unitCorrelationLength = 0;
                if (gui::widgets::InputDouble3LWithUnit(fmt::format("Correlation length τ of the gyro {}##Correlation length τ of the gyro dynamic bias {}",
                                                                    _qCalculationAlgorithm == QCalculationAlgorithm::VanLoan ? "bias noise" : "dynamic bias", size_t(id))
                                                            .c_str(),
                                                        configWidth, unitWidth, _tau_bgd.data(), 0., std::numeric_limits<double>::max(),
                                                        unitCorrelationLength, "s\0\0", "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: tau_bgd changed to {}", nameId(), _tau_bgd);
                    flow::ApplyChanges();
                }
            }

            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 20.F);

            // ----------------------------------------- Barometer -------------------------------------------
            if (_enableBaroHgt)
            {
                if (gui::widgets::InputDoubleWithUnit(fmt::format("Standard deviation of the baro height bias##{}", size_t(id))
                                                          .c_str(),
                                                      configWidth, unitWidth, &_stdevBaroHeightBias, _stdevBaroHeightBiasUnits,
                                                      "m\0\0", 0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: stdevBaroHeightBias changed to: {}", nameId(), _stdevBaroHeightBias);
                    LOG_DEBUG("{}: stdevBaroHeightBiasUnits changed to: {}", nameId(), fmt::underlying(_stdevBaroHeightBiasUnits));
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                gui::widgets::HelpMarker("Random walk");
                if (gui::widgets::InputDoubleWithUnit(fmt::format("Standard deviation of the baro scale##{}", size_t(id))
                                                          .c_str(),
                                                      configWidth, unitWidth, &_stdevBaroHeightScale, _stdevBaroHeightScaleUnits,
                                                      "m/m\0\0", 0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: stdevBaroHeightScale changed to: {}", nameId(), _stdevBaroHeightScale);
                    LOG_DEBUG("{}: stdevBaroHeightScaleUnits changed to: {}", nameId(), fmt::underlying(_stdevBaroHeightScaleUnits));
                    flow::ApplyChanges();
                }
                ImGui::SameLine();
                gui::widgets::HelpMarker("Random walk");
            }

            ImGui::TreePop();
        }

        // ###########################################################################################################
        //                                        Measurement Uncertainties 𝐑
        // ###########################################################################################################

        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("R - Measurement noise covariance matrix##{}", size_t(id)).c_str()))
        {
            float curPosX = ImGui::GetCursorPosX();
            if (ImGui::Checkbox(fmt::format("##Override R Position {}", size_t(id)).c_str(), &_gnssMeasurementUncertaintyPositionOverride))
            {
                flow::ApplyChanges();
            }
            if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Override the position variance in the measurements with these values."); }
            ImGui::SameLine();
            float checkWidth = ImGui::GetCursorPosX() - curPosX;
            if (gui::widgets::InputDouble3WithUnit(fmt::format("{} {} of the GNSS position measurements##{}",
                                                               NAV::to_string(_inertialIntegrator.getIntegrationFrame()),
                                                               _gnssMeasurementUncertaintyPositionUnit == GnssMeasurementUncertaintyPositionUnit::meter2
                                                                   ? "Variance"
                                                                   : "Standard deviation",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth - checkWidth, unitWidth, _gnssMeasurementUncertaintyPosition.data(), _gnssMeasurementUncertaintyPositionUnit, "m^2, m^2, m^2\0"
                                                                                                                                                                             "m, m, m\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: gnssMeasurementUncertaintyPosition changed to {}", nameId(), _gnssMeasurementUncertaintyPosition.transpose());
                LOG_DEBUG("{}: gnssMeasurementUncertaintyPositionUnit changed to {}", nameId(), fmt::underlying(_gnssMeasurementUncertaintyPositionUnit));
                flow::ApplyChanges();
            }

            if (ImGui::Checkbox(fmt::format("##Override R Velocity {}", size_t(id)).c_str(), &_gnssMeasurementUncertaintyVelocityOverride))
            {
                flow::ApplyChanges();
            }
            if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Override the velocity variance in the measurements with these values."); }
            ImGui::SameLine();
            if (gui::widgets::InputDouble3WithUnit(fmt::format("{} {} of the GNSS velocity measurements##{}",
                                                               NAV::to_string(_inertialIntegrator.getIntegrationFrame()),
                                                               _gnssMeasurementUncertaintyVelocityUnit == GnssMeasurementUncertaintyVelocityUnit::m2_s2 ? "Variance" : "Standard deviation",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth - checkWidth, unitWidth, _gnssMeasurementUncertaintyVelocity.data(), _gnssMeasurementUncertaintyVelocityUnit, "m^2/s^2\0"
                                                                                                                                                                             "m/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: gnssMeasurementUncertaintyVelocity changed to {}", nameId(), _gnssMeasurementUncertaintyVelocity);
                LOG_DEBUG("{}: gnssMeasurementUncertaintyVelocityUnit changed to {}", nameId(), fmt::underlying(_gnssMeasurementUncertaintyVelocityUnit));
                flow::ApplyChanges();
            }

            if (_enableBaroHgt)
            {
                if (ImGui::Checkbox(fmt::format("##Override R Baro Height {}", size_t(id)).c_str(), &_baroHeightMeasurementUncertaintyOverride))
                {
                    flow::ApplyChanges();
                }
                if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Override the barometric height variance in the measurements with this value."); }
                ImGui::SameLine();
                if (gui::widgets::InputDoubleWithUnit(fmt::format("{} of the barometric height measurements##{}", _barometricHeightMeasurementUncertaintyUnit == BaroHeightMeasurementUncertaintyUnit::m2 ? "Variance" : "Standard deviation",
                                                                  size_t(id))
                                                          .c_str(),
                                                      configWidth - checkWidth, unitWidth, &_barometricHeightMeasurementUncertainty, _barometricHeightMeasurementUncertaintyUnit, "m^2\0"
                                                                                                                                                                                  "m\0\0",

                                                      0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: barometricHeightMeasurementUncertainty changed to {}", nameId(), _barometricHeightMeasurementUncertainty);
                    LOG_DEBUG("{}: barometricHeightMeasurementUncertaintyUnit changed to {}", nameId(), fmt::underlying(_barometricHeightMeasurementUncertaintyUnit));
                    flow::ApplyChanges();
                }
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
                                                   configWidth, unitWidth, _initCovariancePosition.data(), _initCovariancePositionUnit, "rad^2, rad^2, m^2\0"
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
                                                   configWidth, unitWidth, _initCovarianceVelocity.data(), _initCovarianceVelocityUnit, "m^2/s^2\0"
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
                                                   configWidth, unitWidth, _initCovarianceAttitudeAngles.data(), _initCovarianceAttitudeAnglesUnit, "rad^2\0"
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
            gui::widgets::HelpMarker(_inertialIntegrator.getIntegrationFrame() == InertialIntegrator::IntegrationFrame::ECEF
                                         ? "Angles rotating the ECEF frame into the body frame."
                                         : "Angles rotating the local navigation frame into the body frame (Roll, Pitch, Yaw)");

            if (gui::widgets::InputDouble3WithUnit(fmt::format("Accelerometer Bias covariance ({})##{}",
                                                               _initCovarianceBiasAccelUnit == InitCovarianceBiasAccelUnit::m2_s4
                                                                   ? "Variance σ²"
                                                                   : "Standard deviation σ",
                                                               size_t(id))
                                                       .c_str(),
                                                   configWidth, unitWidth, _initCovarianceBiasAccel.data(), _initCovarianceBiasAccelUnit, "m^2/s^4\0"
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
                                                   configWidth, unitWidth, _initCovarianceBiasGyro.data(), _initCovarianceBiasGyroUnit, "rad^2/s^2\0"
                                                                                                                                        "deg^2/s^2\0"
                                                                                                                                        "rad/s\0"
                                                                                                                                        "deg/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: initCovarianceBiasGyro changed to {}", nameId(), _initCovarianceBiasGyro);
                LOG_DEBUG("{}: initCovarianceBiasGyroUnit changed to {}", nameId(), fmt::underlying(_initCovarianceBiasGyroUnit));
                flow::ApplyChanges();
            }
            if (_enableBaroHgt)
            {
                if (gui::widgets::InputDoubleWithUnit(fmt::format("Height Bias covariance ({})##{}",
                                                                  _initCovarianceBiasHeightUnit == InitCovarianceBiasHeightUnit::m2
                                                                      ? "Variance σ²"
                                                                      : "Standard deviation σ",
                                                                  size_t(id))
                                                          .c_str(),
                                                      configWidth, unitWidth, &_initCovarianceBiasHeight, _initCovarianceBiasHeightUnit, "m^2\0"
                                                                                                                                         "m\0\0",
                                                      0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: initCovarianceBiasHeight changed to {}", nameId(), _initCovarianceBiasHeight);
                    LOG_DEBUG("{}: initCovarianceBiasHeightUnit changed to {}", nameId(), fmt::underlying(_initCovarianceBiasHeightUnit));
                    flow::ApplyChanges();
                }
                if (gui::widgets::InputDoubleWithUnit(fmt::format("Height Scale covariance ({})##{}",
                                                                  _initCovarianceScaleHeightUnit == InitCovarianceScaleHeight::m2_m2
                                                                      ? "Variance σ²"
                                                                      : "Standard deviation σ",
                                                                  size_t(id))
                                                          .c_str(),
                                                      configWidth, unitWidth, &_initCovarianceScaleHeight, _initCovarianceScaleHeightUnit, "m^2/m^2\0"
                                                                                                                                           "m/m\0\0",
                                                      0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
                {
                    LOG_DEBUG("{}: initCovarianceScaleHeight changed to {}", nameId(), _initCovarianceScaleHeight);
                    LOG_DEBUG("{}: initCovarianceScaleHeightUnit changed to {}", nameId(), fmt::underlying(_initCovarianceScaleHeightUnit));
                    flow::ApplyChanges();
                }
            }

            ImGui::TreePop();
        }

        // ###########################################################################################################
        //                                                IMU biases
        // ###########################################################################################################

        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("IMU biases (init)##{}", size_t(id)).c_str()))
        {
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Accelerometer biases##{}", size_t(id)).c_str(),
                                                   configWidth, unitWidth, _initBiasAccel.data(), _initBiasAccelUnit, "µg\0m/s^2\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: initBiasAccel changed to {}", nameId(), _initBiasAccel.transpose());
                LOG_DEBUG("{}: initBiasAccelUnit changed to {}", nameId(), fmt::underlying(_initBiasAccelUnit));
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            gui::widgets::HelpMarker("In body frame coordinates");
            if (gui::widgets::InputDouble3WithUnit(fmt::format("Gyro biases##{}", size_t(id)).c_str(),
                                                   configWidth, unitWidth, _initBiasGyro.data(), _initBiasGyroUnit, "rad/s\0deg/s\0\0",
                                                   "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: initBiasGyro changed to {}", nameId(), _initBiasGyro.transpose());
                LOG_DEBUG("{}: initBiasGyroUnit changed to {}", nameId(), fmt::underlying(_initBiasGyroUnit));
                flow::ApplyChanges();
            }
            ImGui::SameLine();
            gui::widgets::HelpMarker("In body frame coordinates");

            ImGui::TreePop();
        }

        ImGui::SetNextItemOpen(false, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("Kalman Filter matrices##{}", size_t(id)).c_str()))
        {
            _kalmanFilter.showKalmanFilterMatrixViews(std::to_string(size_t(id)).c_str());
            ImGui::TreePop();
        }

        ImGui::Separator();

        if (ImGui::Checkbox(fmt::format("Rank check for Kalman filter matrices##{}", size_t(id)).c_str(), &_checkKalmanMatricesRanks))
        {
            LOG_DEBUG("{}: checkKalmanMatricesRanks {}", nameId(), _checkKalmanMatricesRanks);
            flow::ApplyChanges();
        }
    }
}

[[nodiscard]] json NAV::LooselyCoupledKF::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["inertialIntegrator"] = _inertialIntegrator;
    j["preferAccelerationOverDeltaMeasurements"] = _preferAccelerationOverDeltaMeasurements;
    j["initalRollPitchYaw"] = _initalRollPitchYaw;
    j["initializeStateOverExternalPin"] = _initializeStateOverExternalPin;

    j["checkKalmanMatricesRanks"] = _checkKalmanMatricesRanks;

    j["phiCalculationAlgorithm"] = _phiCalculationAlgorithm;
    j["phiCalculationTaylorOrder"] = _phiCalculationTaylorOrder;
    j["qCalculationAlgorithm"] = _qCalculationAlgorithm;

    j["enableBaroHgt"] = _enableBaroHgt;

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
    j["stdevBaroHeightBiasUnits"] = _stdevBaroHeightBiasUnits;
    j["stdevBaroHeightBias"] = _stdevBaroHeightBias;
    j["stdevBaroHeightScaleUnits"] = _stdevBaroHeightScaleUnits;
    j["stdevBaroHeightScale"] = _stdevBaroHeightScale;

    j["gnssMeasurementUncertaintyPositionUnit"] = _gnssMeasurementUncertaintyPositionUnit;
    j["gnssMeasurementUncertaintyPosition"] = _gnssMeasurementUncertaintyPosition;
    j["gnssMeasurementUncertaintyPositionOverride"] = _gnssMeasurementUncertaintyPositionOverride;
    j["gnssMeasurementUncertaintyVelocityUnit"] = _gnssMeasurementUncertaintyVelocityUnit;
    j["gnssMeasurementUncertaintyVelocity"] = _gnssMeasurementUncertaintyVelocity;
    j["gnssMeasurementUncertaintyVelocityOverride"] = _gnssMeasurementUncertaintyVelocityOverride;
    j["barometricHeightMeasurementUncertaintyUnit"] = _barometricHeightMeasurementUncertaintyUnit;
    j["barometricHeightMeasurementUncertainty"] = _barometricHeightMeasurementUncertainty;
    j["baroHeightMeasurementUncertaintyOverride"] = _baroHeightMeasurementUncertaintyOverride;

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
    j["initCovarianceBiasHeightUnit"] = _initCovarianceBiasHeightUnit;
    j["initCovarianceBiasHeight"] = _initCovarianceBiasHeight;
    j["initCovarianceScaleHeightUnit"] = _initCovarianceScaleHeightUnit;
    j["initCovarianceScaleHeight"] = _initCovarianceScaleHeight;

    j["initBiasAccel"] = _initBiasAccel;
    j["initBiasAccelUnit"] = _initBiasAccelUnit;
    j["initBiasGyro"] = _initBiasGyro;
    j["initBiasGyroUnit"] = _initBiasGyroUnit;

    return j;
}

void NAV::LooselyCoupledKF::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("inertialIntegrator"))
    {
        j.at("inertialIntegrator").get_to(_inertialIntegrator);
    }
    if (j.contains("preferAccelerationOverDeltaMeasurements"))
    {
        j.at("preferAccelerationOverDeltaMeasurements").get_to(_preferAccelerationOverDeltaMeasurements);
    }
    if (j.contains("initalRollPitchYaw"))
    {
        j.at("initalRollPitchYaw").get_to(_initalRollPitchYaw);
    }
    if (j.contains("initializeStateOverExternalPin"))
    {
        j.at("initializeStateOverExternalPin").get_to(_initializeStateOverExternalPin);
        updateInputPins();
    }
    if (j.contains("checkKalmanMatricesRanks"))
    {
        j.at("checkKalmanMatricesRanks").get_to(_checkKalmanMatricesRanks);
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
    if (j.contains("enableBaroHgt"))
    {
        j.at("enableBaroHgt").get_to(_enableBaroHgt);
        updateInputPins();
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
    if (j.contains("stdevBaroHeightBiasUnits"))
    {
        j.at("stdevBaroHeightBiasUnits").get_to(_stdevBaroHeightBiasUnits);
    }
    if (j.contains("stdevBaroHeightBias"))
    {
        _stdevBaroHeightBias = j.at("stdevBaroHeightBias");
    }
    if (j.contains("stdevBaroHeightScaleUnits"))
    {
        j.at("stdevBaroHeightScaleUnits").get_to(_stdevBaroHeightScaleUnits);
    }
    if (j.contains("stdevBaroHeightScale"))
    {
        _stdevBaroHeightScale = j.at("stdevBaroHeightScale");
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
    if (j.contains("gnssMeasurementUncertaintyPositionOverride"))
    {
        j.at("gnssMeasurementUncertaintyPositionOverride").get_to(_gnssMeasurementUncertaintyPositionOverride);
    }
    if (j.contains("gnssMeasurementUncertaintyVelocityUnit"))
    {
        j.at("gnssMeasurementUncertaintyVelocityUnit").get_to(_gnssMeasurementUncertaintyVelocityUnit);
    }
    if (j.contains("gnssMeasurementUncertaintyVelocity"))
    {
        _gnssMeasurementUncertaintyVelocity = j.at("gnssMeasurementUncertaintyVelocity");
    }
    if (j.contains("gnssMeasurementUncertaintyVelocityOverride"))
    {
        j.at("gnssMeasurementUncertaintyVelocityOverride").get_to(_gnssMeasurementUncertaintyVelocityOverride);
    }
    if (j.contains("barometricHeightMeasurementUncertaintyUnit"))
    {
        j.at("barometricHeightMeasurementUncertaintyUnit").get_to(_barometricHeightMeasurementUncertaintyUnit);
    }
    if (j.contains("barometricHeightMeasurementUncertainty"))
    {
        _barometricHeightMeasurementUncertainty = j.at("barometricHeightMeasurementUncertainty");
    }
    if (j.contains("baroHeightMeasurementUncertaintyOverride"))
    {
        j.at("baroHeightMeasurementUncertaintyOverride").get_to(_baroHeightMeasurementUncertaintyOverride);
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
    if (j.contains("initCovarianceBiasHeightUnit"))
    {
        j.at("initCovarianceBiasHeightUnit").get_to(_initCovarianceBiasHeightUnit);
    }
    if (j.contains("initCovarianceBiasHeight"))
    {
        _initCovarianceBiasHeight = j.at("initCovarianceBiasHeight");
    }
    if (j.contains("initCovarianceScaleHeightUnit"))
    {
        j.at("initCovarianceScaleHeightUnit").get_to(_initCovarianceScaleHeightUnit);
    }
    if (j.contains("initCovarianceScaleHeight"))
    {
        _initCovarianceScaleHeight = j.at("initCovarianceScaleHeight");
    }

    // ---------------------------------------- Initial IMU biases -------------------------------------------
    if (j.contains("initBiasAccel"))
    {
        _initBiasAccel = j.at("initBiasAccel");
    }
    if (j.contains("initBiasAccelUnit"))
    {
        _initBiasAccelUnit = j.at("initBiasAccelUnit");
    }
    if (j.contains("initBiasGyro"))
    {
        _initBiasGyro = j.at("initBiasGyro");
    }
    if (j.contains("initBiasGyroUnit"))
    {
        _initBiasGyroUnit = j.at("initBiasGyroUnit");
    }
}

bool NAV::LooselyCoupledKF::initialize()
{
    LOG_TRACE("{}: called", nameId());

    inputPins[INPUT_PORT_INDEX_GNSS].priority = 2; // PosVel used for initialization

    _inertialIntegrator.reset();
    _lastImuObs = nullptr;
    _externalInitTime.reset();
    _initialSensorBiasesApplied = false;
    _heightBiasTotal = 0.0;
    _heightScaleTotal = 1.0;

    _kalmanFilter.setZero();

    // Initial Covariance of the attitude angles in [rad²]
    Eigen::Vector3d variance_angles = Eigen::Vector3d::Zero();
    switch (_initCovarianceAttitudeAnglesUnit)
    {
    case InitCovarianceAttitudeAnglesUnit::rad2:
        variance_angles = _initCovarianceAttitudeAngles;
        break;
    case InitCovarianceAttitudeAnglesUnit::deg2:
        variance_angles = deg2rad(_initCovarianceAttitudeAngles);
        break;
    case InitCovarianceAttitudeAnglesUnit::rad:
        variance_angles = _initCovarianceAttitudeAngles.array().pow(2);
        break;
    case InitCovarianceAttitudeAnglesUnit::deg:
        variance_angles = deg2rad(_initCovarianceAttitudeAngles).array().pow(2);
        break;
    }

    // Initial Covariance of the velocity in [m²/s²]
    Eigen::Vector3d variance_vel = Eigen::Vector3d::Zero();
    switch (_initCovarianceVelocityUnit)
    {
    case InitCovarianceVelocityUnit::m2_s2:
        variance_vel = _initCovarianceVelocity;
        break;
    case InitCovarianceVelocityUnit::m_s:
        variance_vel = _initCovarianceVelocity.array().pow(2);
        break;
    }

    Eigen::Vector3d e_variance = Eigen::Vector3d::Zero();   // Initial Covariance of the position in [m²]
    Eigen::Vector3d lla_variance = Eigen::Vector3d::Zero(); // Initial Covariance of the position in [rad² rad² m²]
    switch (_initCovariancePositionUnit)
    {
    case InitCovariancePositionUnit::rad2_rad2_m2:
        e_variance = trafo::lla2ecef_WGS84(_initCovariancePosition.cwiseSqrt()).array().pow(2);
        lla_variance = _initCovariancePosition;
        break;
    case InitCovariancePositionUnit::rad_rad_m:
        e_variance = trafo::lla2ecef_WGS84(_initCovariancePosition).array().pow(2);
        lla_variance = _initCovariancePosition.array().pow(2);
        break;
    case InitCovariancePositionUnit::meter:
        e_variance = _initCovariancePosition.array().pow(2);
        lla_variance = (trafo::ecef2lla_WGS84(trafo::ned2ecef(_initCovariancePosition, Eigen::Vector3d{ 0, 0, 0 }))).array().pow(2);
        break;
    case InitCovariancePositionUnit::meter2:
        e_variance = _initCovariancePosition;
        lla_variance = (trafo::ecef2lla_WGS84(trafo::ned2ecef(_initCovariancePosition.cwiseSqrt(), Eigen::Vector3d{ 0, 0, 0 }))).array().pow(2);
        break;
    }

    // Initial Covariance of the accelerometer biases in [m^2/s^4]
    Eigen::Vector3d variance_accelBias = Eigen::Vector3d::Zero();
    switch (_initCovarianceBiasAccelUnit)
    {
    case InitCovarianceBiasAccelUnit::m2_s4:
        variance_accelBias = _initCovarianceBiasAccel;
        break;
    case InitCovarianceBiasAccelUnit::m_s2:
        variance_accelBias = _initCovarianceBiasAccel.array().pow(2);
        break;
    }

    // Initial Covariance of the gyroscope biases in [rad^2/s^2]
    Eigen::Vector3d variance_gyroBias = Eigen::Vector3d::Zero();
    switch (_initCovarianceBiasGyroUnit)
    {
    case InitCovarianceBiasGyroUnit::rad2_s2:
        variance_gyroBias = _initCovarianceBiasGyro;
        break;
    case InitCovarianceBiasGyroUnit::deg2_s2:
        variance_gyroBias = deg2rad(_initCovarianceBiasGyro.array().sqrt()).array().pow(2);
        break;
    case InitCovarianceBiasGyroUnit::rad_s:
        variance_gyroBias = _initCovarianceBiasGyro.array().pow(2);
        break;
    case InitCovarianceBiasGyroUnit::deg_s:
        variance_gyroBias = deg2rad(_initCovarianceBiasGyro).array().pow(2);
        break;
    }

    // Initial Covariance of the height bias in [m^2]
    double variance_heightBias = 0.0;
    switch (_initCovarianceBiasHeightUnit)
    {
    case InitCovarianceBiasHeightUnit::m:
        variance_heightBias = std::pow(_initCovarianceBiasHeight, 2);
        break;
    case InitCovarianceBiasHeightUnit::m2:
        variance_heightBias = _initCovarianceBiasHeight;
        break;
    }

    // Initial Covariance of the height scale in [m^2/m^2]
    double variance_heightScale = 0.0;
    switch (_initCovarianceScaleHeightUnit)
    {
    case InitCovarianceScaleHeight::m_m:
        variance_heightScale = std::pow(_initCovarianceScaleHeight, 2);
        break;
    case InitCovarianceScaleHeight::m2_m2:
        variance_heightScale = _initCovarianceScaleHeight;
        break;
    }

    // 𝐏 Error covariance matrix
    _kalmanFilter.P = initialErrorCovarianceMatrix_P0(variance_angles, // Flight Angles covariance
                                                      variance_vel,    // Velocity covariance
                                                      _inertialIntegrator.getIntegrationFrame() == InertialIntegrator::IntegrationFrame::NED
                                                          ? lla_variance
                                                          : e_variance,      // Position (Lat, Lon, Alt) / ECEF covariance
                                                      variance_accelBias,    // Accelerometer Bias covariance
                                                      variance_gyroBias,     // Gyroscope Bias covariance
                                                      variance_heightBias,   // height bias covariance
                                                      variance_heightScale); // height scale covariance

    // Initial acceleration bias in body frame coordinates in [m/s^2]
    switch (_initBiasAccelUnit)
    {
    case InitBiasAccelUnit::microg:
        _accelBiasTotal = _initBiasAccel * 1e-6 * InsConst::G_NORM;
        break;
    case InitBiasAccelUnit::m_s2:
        _accelBiasTotal = _initBiasAccel;
        break;
    }

    // Initial angular rate bias in body frame coordinates in [rad/s]
    switch (_initBiasGyroUnit)
    {
    case InitBiasGyroUnit::deg_s:
        _gyroBiasTotal = deg2rad(_initBiasGyro);
        break;
    case InitBiasGyroUnit::rad_s:
        _gyroBiasTotal = _initBiasGyro;
        break;
    }

    LOG_DEBUG("{}: initialized", nameId());
    LOG_DATA("{}: P_0 =\n{}", nameId(), _kalmanFilter.P);

    return true;
}

void NAV::LooselyCoupledKF::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::LooselyCoupledKF::invokeCallbackWithPosVelAtt(const PosVelAtt& posVelAtt)
{
    auto lckfSolution = std::make_shared<InsGnssLCKFSolution>();
    lckfSolution->insTime = posVelAtt.insTime;
    if (posVelAtt.e_CovarianceMatrix())
    {
        lckfSolution->setPosVelAttAndCov_e(posVelAtt.e_position(),
                                           posVelAtt.e_velocity(),
                                           posVelAtt.e_Quat_b(),
                                           (*posVelAtt.e_CovarianceMatrix())(all, all));
    }
    else
    {
        lckfSolution->setPosVelAtt_e(posVelAtt.e_position(), posVelAtt.e_velocity(), posVelAtt.e_Quat_b());
    }

    lckfSolution->frame = _inertialIntegrator.getIntegrationFrame() == InertialIntegrator::IntegrationFrame::NED
                              ? InsGnssLCKFSolution::Frame::NED
                              : InsGnssLCKFSolution::Frame::ECEF;

    if (_lastImuObs)
    {
        lckfSolution->b_biasAccel.value = _lastImuObs->imuPos.b_quatAccel_p() * -_inertialIntegrator.p_getLastAccelerationBias();
        lckfSolution->b_biasAccel.stdDev = Eigen::Vector3d{
            std::sqrt(_kalmanFilter.P(KFStates::AccBiasX, KFStates::AccBiasX) * (1. / std::pow(SCALE_FACTOR_ACCELERATION, 2))),
            std::sqrt(_kalmanFilter.P(KFStates::AccBiasY, KFStates::AccBiasY) * (1. / std::pow(SCALE_FACTOR_ACCELERATION, 2))),
            std::sqrt(_kalmanFilter.P(KFStates::AccBiasZ, KFStates::AccBiasZ) * (1. / std::pow(SCALE_FACTOR_ACCELERATION, 2)))
        };
        lckfSolution->b_biasGyro.value = _lastImuObs->imuPos.b_quatGyro_p() * -_inertialIntegrator.p_getLastAngularRateBias();
        lckfSolution->b_biasGyro.stdDev = Eigen::Vector3d{
            std::sqrt(_kalmanFilter.P(KFStates::GyrBiasX, KFStates::GyrBiasX) * (1. / std::pow(SCALE_FACTOR_ANGULAR_RATE, 2))),
            std::sqrt(_kalmanFilter.P(KFStates::GyrBiasY, KFStates::GyrBiasY) * (1. / std::pow(SCALE_FACTOR_ANGULAR_RATE, 2))),
            std::sqrt(_kalmanFilter.P(KFStates::GyrBiasZ, KFStates::GyrBiasZ) * (1. / std::pow(SCALE_FACTOR_ANGULAR_RATE, 2)))
        };
    }
    lckfSolution->heightBias = { .value = _heightBiasTotal, .stdDev = std::sqrt(_kalmanFilter.P(KFStates::HeightBias, KFStates::HeightBias)) };
    lckfSolution->heightScale = { .value = _heightScaleTotal, .stdDev = std::sqrt(_kalmanFilter.P(KFStates::HeightScale, KFStates::HeightScale)) };
    if (!hasInputPinWithSameTime(lckfSolution->insTime))
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_SOLUTION, lckfSolution);
    }
}

void NAV::LooselyCoupledKF::recvImuObservation(InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto nodeData = queue.extract_front();
    if (nodeData->insTime.empty())
    {
        LOG_ERROR("{}: Can't set new imuObs__t0 because the observation has no time tag (insTime)", nameId());
        return;
    }

    std::shared_ptr<NAV::PosVelAtt> inertialNavSol = nullptr;

    _lastImuObs = std::static_pointer_cast<const ImuObs>(nodeData);

    if (!_preferAccelerationOverDeltaMeasurements
        && NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(inputPins.at(INPUT_PORT_INDEX_IMU).link.getConnectedPin()->dataIdentifier, { ImuObsWDelta::type() }))
    {
        auto obs = std::static_pointer_cast<const ImuObsWDelta>(nodeData);
        LOG_DATA("{}: [{}] recvImuObsWDelta", nameId(), obs->insTime.toYMDHMS(GPST));

        // Initialize biases
        if (!_initialSensorBiasesApplied)
        {
            _inertialIntegrator.setTotalSensorBiases(obs->imuPos.p_quatAccel_b() * -_accelBiasTotal, obs->imuPos.p_quatGyro_b() * -_gyroBiasTotal);
            _initialSensorBiasesApplied = true;
        }

        inertialNavSol = _inertialIntegrator.calcInertialSolutionDelta(obs->insTime, obs->dtime, obs->dvel, obs->dtheta, obs->imuPos);
    }
    else
    {
        auto obs = std::static_pointer_cast<const ImuObs>(nodeData);
        LOG_DATA("{}: [{}] recvImuObs", nameId(), obs->insTime.toYMDHMS(GPST));

        // Initialize biases
        if (!_initialSensorBiasesApplied)
        {
            _inertialIntegrator.setTotalSensorBiases(obs->imuPos.p_quatAccel_b() * -_accelBiasTotal, obs->imuPos.p_quatGyro_b() * -_gyroBiasTotal);
            _initialSensorBiasesApplied = true;
        }

        inertialNavSol = _inertialIntegrator.calcInertialSolution(obs->insTime, obs->p_acceleration, obs->p_angularRate, obs->imuPos);
    }
    if (inertialNavSol && _inertialIntegrator.getMeasurements().back().dt > 1e-8)
    {
        looselyCoupledPrediction(inertialNavSol, _inertialIntegrator.getMeasurements().back().dt, std::static_pointer_cast<const ImuObs>(nodeData)->imuPos);

        Eigen::Matrix<double, 9, 9> deg2rad_2 = Eigen::Matrix<double, 9, 9>::Identity();
        deg2rad_2.bottomRightCorner<3, 3>().diagonal().setConstant(std::pow(std::numbers::pi_v<double> / 180.0, 2.0));
        Eigen::Matrix<double, 10, 9> J = Eigen::Matrix<double, 10, 9>::Zero();
        J.topLeftCorner<6, 6>().setIdentity();
        if (_inertialIntegrator.getIntegrationFrame() == InertialIntegrator::IntegrationFrame::NED)
        {
            J.bottomRightCorner<4, 3>() = trafo::covRPY2quatJacobian(_inertialIntegrator.getLatestState().value().get().n_Quat_b());

            inertialNavSol->setPosVelAttAndCov_n(inertialNavSol->lla_position(),
                                                 inertialNavSol->n_velocity(),
                                                 inertialNavSol->n_Quat_b(),
                                                 J * (_kalmanFilter.P(KFPosVelAtt, KFPosVelAtt) * deg2rad_2) * J.transpose());
        }
        else // if (_inertialIntegrator.getIntegrationFrame() == InertialIntegrator::IntegrationFrame::ECEF)
        {
            J.bottomRightCorner<4, 3>() = trafo::covRPY2quatJacobian(_inertialIntegrator.getLatestState().value().get().e_Quat_b());
            inertialNavSol->setPosVelAttAndCov_e(inertialNavSol->e_position(),
                                                 inertialNavSol->e_velocity(),
                                                 inertialNavSol->e_Quat_b(),
                                                 J * (_kalmanFilter.P(KFPosVelAtt, KFPosVelAtt) * deg2rad_2) * J.transpose());
        }

        LOG_DATA("{}:   e_position   = {}", nameId(), inertialNavSol->e_position().transpose());
        LOG_DATA("{}:   e_velocity   = {}", nameId(), inertialNavSol->e_velocity().transpose());
        LOG_DATA("{}:   rollPitchYaw = {}", nameId(), rad2deg(inertialNavSol->rollPitchYaw()).transpose());
        if (const auto& q = inputPins.at(INPUT_PORT_INDEX_GNSS).queue;
            q.empty() || q.front()->insTime != nodeData->insTime)
        {
            LOG_DATA("{}: [{}] Sending out predicted solution", nameId(), inertialNavSol->insTime.toYMDHMS(GPST));
            invokeCallbackWithPosVelAtt(*inertialNavSol);
        }
    }
}

void NAV::LooselyCoupledKF::recvPosVelObservation(InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto obs = std::static_pointer_cast<const PosVel>(queue.extract_front());
    LOG_DATA("{}: [{}] recvPosVelObservation", nameId(), obs->insTime.toYMDHMS(GPST));

    if (!_initializeStateOverExternalPin && !_inertialIntegrator.hasInitialPosition())
    {
        inputPins[INPUT_PORT_INDEX_GNSS].priority = 0; // IMU obs (prediction) should be evaluated before the PosVel obs (update)

        PosVelAtt posVelAtt;
        posVelAtt.insTime = obs->insTime;
        posVelAtt.setPosVelAtt_n(obs->lla_position(), obs->n_velocity(),
                                 trafo::n_Quat_b(deg2rad(_initalRollPitchYaw[0]), deg2rad(_initalRollPitchYaw[1]), deg2rad(_initalRollPitchYaw[2])));

        _inertialIntegrator.setInitialState(posVelAtt);
        LOG_DATA("{}:   e_position   = {}", nameId(), posVelAtt.e_position().transpose());
        LOG_DATA("{}:   e_velocity   = {}", nameId(), posVelAtt.e_velocity().transpose());
        LOG_DATA("{}:   rollPitchYaw = {}", nameId(), rad2deg(posVelAtt.rollPitchYaw()).transpose());

        LOG_DATA("{}: [{}] Sending out initial solution", nameId(), obs->insTime.toYMDHMS(GPST));
        invokeCallbackWithPosVelAtt(posVelAtt);
        return;
    }

    if (_externalInitTime == obs->insTime) { return; }
    if (!_lastImuObs)
    {
        PosVelAtt posVelAtt;
        posVelAtt.insTime = obs->insTime;
        posVelAtt.setPosVelAtt_n(obs->lla_position(), obs->n_velocity(), _inertialIntegrator.getLatestState()->get().n_Quat_b());
        _inertialIntegrator.setState(posVelAtt);

        LOG_DATA("{}: [{}] Sending out received solution, as no IMU data yet", nameId(), obs->insTime.toYMDHMS(GPST));
        invokeCallbackWithPosVelAtt(posVelAtt);
        return;
    }

    looselyCoupledUpdate(obs);
}

void NAV::LooselyCoupledKF::recvBaroHeight([[maybe_unused]] InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto obs = std::static_pointer_cast<const BaroHgt>(queue.extract_front());
    LOG_DATA("{}: [{}] recvPosVelObservation", nameId(), obs->insTime.toYMDHMS(GPST));

    if (!_inertialIntegrator.hasInitialPosition())
    {
        return;
    }

    if (_externalInitTime == obs->insTime) { return; }
    if (!_lastImuObs)
    {
        return;
    }

    looselyCoupledUpdate(obs);
}

void NAV::LooselyCoupledKF::recvPosVelAttInit(InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto posVelAtt = std::static_pointer_cast<const PosVelAtt>(queue.extract_front());
    inputPins[INPUT_PORT_INDEX_POS_VEL_ATT_INIT].queueBlocked = true;
    inputPins[INPUT_PORT_INDEX_POS_VEL_ATT_INIT].queue.clear();

    LOG_DATA("{}: recvPosVelAttInit at time [{}]", nameId(), posVelAtt->insTime.toYMDHMS());

    inputPins[INPUT_PORT_INDEX_GNSS].priority = 0; // IMU obs (prediction) should be evaluated before the PosVel obs (update)
    _externalInitTime = posVelAtt->insTime;

    _inertialIntegrator.setInitialState(*posVelAtt);
    LOG_DATA("{}:   e_position   = {}", nameId(), posVelAtt->e_position().transpose());
    LOG_DATA("{}:   e_velocity   = {}", nameId(), posVelAtt->e_velocity().transpose());
    LOG_DATA("{}:   rollPitchYaw = {}", nameId(), rad2deg(posVelAtt->rollPitchYaw()).transpose());

    invokeCallbackWithPosVelAtt(*posVelAtt);
}

// ###########################################################################################################
//                                               Kalman Filter
// ###########################################################################################################

void NAV::LooselyCoupledKF::looselyCoupledPrediction(const std::shared_ptr<const PosVelAtt>& inertialNavSol, double tau_i, const ImuPos& imuPos)
{
    LOG_DATA("{}: [{}] Predicting", nameId(), inertialNavSol->insTime.toYMDHMS(GPST));

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

    // Standard deviation of the barometric height bias [m]
    double sigma_heightBias{};
    switch (_stdevBaroHeightBiasUnits)
    {
    case StdevBaroHeightBiasUnits::m: // [m]
        sigma_heightBias = _stdevBaroHeightBias;
        break;
    }
    LOG_DATA("{}: sigma_heightBias = {} [m]", nameId(), sigma_heightBias);

    // Standard deviation of the barometric height scale [m/m]
    double sigma_heightScale{};
    switch (_stdevBaroHeightScaleUnits)
    {
    case StdevBaroHeightScaleUnits::m_m: // [m/m]
        sigma_heightScale = _stdevBaroHeightScale;
        break;
    }

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

    auto p_acceleration = _inertialIntegrator.p_calcCurrentAcceleration();
    // Acceleration in [m/s^2], in body coordinates
    Eigen::Vector3d b_acceleration = p_acceleration
                                         ? imuPos.b_quatAccel_p() * p_acceleration.value()
                                         : Eigen::Vector3d::Zero();
    LOG_DATA("{}:     b_acceleration = {} [m/s^2]", nameId(), b_acceleration.transpose());

    if (_inertialIntegrator.getIntegrationFrame() == InertialIntegrator::IntegrationFrame::NED)
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
        _kalmanFilter.F = n_systemMatrix_F(n_Quat_b, b_acceleration, n_omega_in, n_velocity, lla_position, R_N, R_E, g_0, r_eS_e, _tau_bad, _tau_bgd);
        LOG_DATA("{}:     F =\n{}", nameId(), _kalmanFilter.F);
        if (_qCalculationAlgorithm == QCalculationAlgorithm::Taylor1)
        {
            // 2. Calculate the system noise covariance matrix Q_{k-1}
            _kalmanFilter.Q = n_systemNoiseCovarianceMatrix_Q(sigma_ra.array().square(), sigma_rg.array().square(),
                                                              sigma_bad.array().square(), sigma_bgd.array().square(),
                                                              sigma_heightBias, sigma_heightScale,
                                                              _tau_bad, _tau_bgd,
                                                              _kalmanFilter.F.block<3>(KFVel, KFAtt), T_rn_p,
                                                              n_Quat_b.toRotationMatrix(), tau_i);
        }
    }
    else // if (_inertialIntegrator.getIntegrationFrame() == InertialIntegrator::IntegrationFrame::ECEF)
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
        _kalmanFilter.F = e_systemMatrix_F(e_Quat_b, b_acceleration, e_position, e_gravitation, r_eS_e, InsConst::e_omega_ie, _tau_bad, _tau_bgd);
        LOG_DATA("{}:     F =\n{}", nameId(), _kalmanFilter.F);
        if (_qCalculationAlgorithm == QCalculationAlgorithm::Taylor1)
        {
            // 2. Calculate the system noise covariance matrix Q_{k-1}
            _kalmanFilter.Q = e_systemNoiseCovarianceMatrix_Q(sigma_ra.array().square(), sigma_rg.array().square(),
                                                              sigma_bad.array().square(), sigma_bgd.array().square(),
                                                              sigma_heightBias, sigma_heightScale,
                                                              _tau_bad, _tau_bgd,
                                                              _kalmanFilter.F.block<3>(KFVel, KFAtt),
                                                              e_Quat_b.toRotationMatrix(), tau_i);
        }
    }
    if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
    {
        // Noise Input Matrix
        _kalmanFilter.G = noiseInputMatrix_G(_inertialIntegrator.getIntegrationFrame() == InertialIntegrator::IntegrationFrame::NED
                                                 ? inertialNavSol->n_Quat_b()
                                                 : inertialNavSol->e_Quat_b());
        LOG_DATA("{}:     G =\n{}", nameId(), _kalmanFilter.G);

        _kalmanFilter.W(all, all) = noiseScaleMatrix_W(sigma_ra, sigma_rg,
                                                       sigma_bad, sigma_bgd,
                                                       _tau_bad, _tau_bgd,
                                                       sigma_heightBias, sigma_heightScale);
        LOG_DATA("{}:     W =\n{}", nameId(), _kalmanFilter.W(all, all));
        LOG_DATA("{}:     G*W*G^T =\n{}", nameId(), _kalmanFilter.G(all, all) * _kalmanFilter.W(all, all) * _kalmanFilter.G(all, all).transpose());

        // 1. Calculate the transition matrix 𝚽_{k-1}
        // 2. Calculate the system noise covariance matrix Q_{k-1}
        _kalmanFilter.calcPhiAndQWithVanLoanMethod(tau_i);
    }
    // If Q was calculated over Van Loan, then the Phi matrix was automatically calculated with the exponential matrix
    if (_phiCalculationAlgorithm != PhiCalculationAlgorithm::Exponential || _qCalculationAlgorithm != QCalculationAlgorithm::VanLoan)
    {
        auto calcPhi = [&]() {
            if (_phiCalculationAlgorithm == PhiCalculationAlgorithm::Exponential)
            {
                // 1. Calculate the transition matrix 𝚽_{k-1}
                _kalmanFilter.calcTransitionMatrix_Phi_exp(tau_i);
            }
            else if (_phiCalculationAlgorithm == PhiCalculationAlgorithm::Taylor)
            {
                // 1. Calculate the transition matrix 𝚽_{k-1}
                _kalmanFilter.calcTransitionMatrix_Phi_Taylor(tau_i, static_cast<size_t>(_phiCalculationTaylorOrder));
            }
            else
            {
                LOG_CRITICAL("{}: Calculation algorithm '{}' for the system matrix Phi is not supported.", nameId(), fmt::underlying(_phiCalculationAlgorithm));
            }
        };
        calcPhi();
    }
    LOG_DATA("{}:     KF.Phi =\n{}", nameId(), _kalmanFilter.Phi);
    LOG_DATA("{}:     KF.Q =\n{}", nameId(), _kalmanFilter.Q);

    LOG_DATA("{}:     Q - Q^T =\n{}", nameId(), _kalmanFilter.Q(all, all) - _kalmanFilter.Q(all, all).transpose());
    LOG_DATA("{}:     KF.P (before prediction) =\n{}", nameId(), _kalmanFilter.P);

    // 3. Propagate the state vector estimate from x(+) and x(-)
    // 4. Propagate the error covariance matrix from P(+) and P(-)
    _kalmanFilter.predict();

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
            LOG_WARN("{}: [{}] P.rank = {}", nameId(), inertialNavSol->insTime.toYMDHMS(GPST), rank);
        }
    }
}

void NAV::LooselyCoupledKF::looselyCoupledUpdate(const std::shared_ptr<const PosVel>& posVelObs)
{
    INS_ASSERT_USER_ERROR(_inertialIntegrator.getLatestState().has_value(), "The update should not even trigger without an initial state.");

    LOG_DATA("{}: [{}] Updating (lastInertial at [{}])", nameId(), posVelObs->insTime.toYMDHMS(GPST), _inertialIntegrator.getLatestState().value().get().insTime.toYMDHMS(GPST));

    // -------------------------------------------- GUI Parameters -----------------------------------------------

    // Latitude 𝜙, longitude λ and altitude (height above ground) in [rad, rad, m] at the time tₖ₋₁
    const Eigen::Vector3d& lla_position = _inertialIntegrator.getLatestState().value().get().lla_position();
    LOG_DATA("{}:     lla_position = {} [rad, rad, m]", nameId(), lla_position.transpose());

    // ---------------------------------------------- Correction -------------------------------------------------

    auto p_omega_ip = _inertialIntegrator.p_calcCurrentAngularRate();
    // Angular rate measured in units of [rad/s], and given in the body frame
    Eigen::Vector3d b_omega_ip = p_omega_ip
                                     ? _lastImuObs->imuPos.b_quatGyro_p() * p_omega_ip.value()
                                     : Eigen::Vector3d::Zero();
    LOG_DATA("{}:     b_omega_ip = {} [rad/s]", nameId(), b_omega_ip.transpose());

    _kalmanFilter.setMeasurements(Meas);

    if (_inertialIntegrator.getIntegrationFrame() == InertialIntegrator::IntegrationFrame::NED)
    {
        // Prime vertical radius of curvature (East/West) [m]
        double R_E = calcEarthRadius_E(lla_position(0));
        LOG_DATA("{}:     R_E = {} [m]", nameId(), R_E);
        // Meridian radius of curvature in [m]
        double R_N = calcEarthRadius_N(lla_position(0));
        LOG_DATA("{}:     R_N = {} [m]", nameId(), R_N);

        // Direction Cosine Matrix from body to navigation coordinates, at the time tₖ₋₁
        Eigen::Matrix3d n_Dcm_b = _inertialIntegrator.getLatestState().value().get().n_Quat_b().toRotationMatrix();
        LOG_DATA("{}:     n_Dcm_b =\n{}", nameId(), n_Dcm_b);

        // Conversion matrix between cartesian and curvilinear perturbations to the position
        Eigen::Matrix3d T_rn_p = conversionMatrixCartesianCurvilinear(lla_position, R_N, R_E);
        LOG_DATA("{}:     T_rn_p =\n{}", nameId(), T_rn_p);

        // Skew-symmetric matrix of the Earth-rotation vector in local navigation frame axes
        Eigen::Matrix3d n_Omega_ie = math::skewSymmetricMatrix(_inertialIntegrator.getLatestState().value().get().n_Quat_e() * InsConst::e_omega_ie);
        LOG_DATA("{}:     n_Omega_ie =\n{}", nameId(), n_Omega_ie);

        // 5. Calculate the measurement matrix H_k
        _kalmanFilter.H = n_measurementMatrix_H(T_rn_p, n_Dcm_b, b_omega_ip, _b_leverArm_InsGnss, n_Omega_ie);

        // 6. Calculate the measurement noise covariance matrix R_k
        _kalmanFilter.R = n_measurementNoiseCovariance_R(posVelObs, lla_position, R_N, R_E);

        // 8. Formulate the measurement z_k
        _kalmanFilter.z = n_measurementInnovation_dz(posVelObs->lla_position(), _inertialIntegrator.getLatestState().value().get().lla_position(),
                                                     posVelObs->n_velocity(), _inertialIntegrator.getLatestState().value().get().n_velocity(),
                                                     T_rn_p, _inertialIntegrator.getLatestState().value().get().n_Quat_b(), _b_leverArm_InsGnss, b_omega_ip, n_Omega_ie);
    }
    else // if (_inertialIntegrator.getIntegrationFrame() == InertialIntegrator::IntegrationFrame::ECEF)
    {
        // Direction Cosine Matrix from body to navigation coordinates, at the time tₖ₋₁
        Eigen::Matrix3d e_Dcm_b = _inertialIntegrator.getLatestState().value().get().e_Quat_b().toRotationMatrix();
        LOG_DATA("{}:     e_Dcm_b =\n{}", nameId(), e_Dcm_b);

        // Skew-symmetric matrix of the Earth-rotation vector in local navigation frame axes
        Eigen::Matrix3d e_Omega_ie = math::skewSymmetricMatrix(InsConst::e_omega_ie);
        LOG_DATA("{}:     e_Omega_ie =\n{}", nameId(), e_Omega_ie);

        // 5. Calculate the measurement matrix H_k
        _kalmanFilter.H = e_measurementMatrix_H(e_Dcm_b, b_omega_ip, _b_leverArm_InsGnss, e_Omega_ie);

        // 6. Calculate the measurement noise covariance matrix R_k
        _kalmanFilter.R = e_measurementNoiseCovariance_R(posVelObs);

        // 8. Formulate the measurement z_k
        _kalmanFilter.z = e_measurementInnovation_dz(posVelObs->e_position(), _inertialIntegrator.getLatestState().value().get().e_position(),
                                                     posVelObs->e_velocity(), _inertialIntegrator.getLatestState().value().get().e_velocity(),
                                                     _inertialIntegrator.getLatestState().value().get().e_Quat_b(), _b_leverArm_InsGnss, b_omega_ip, e_Omega_ie);
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
            LOG_WARN("{}: [{}] (HPH^T + R).rank = {}", nameId(), posVelObs->insTime.toYMDHMS(GPST), rank);
        }
    }

    // 7. Calculate the Kalman gain matrix K_k
    // 9. Update the state vector estimate from x(-) to x(+)
    // 10. Update the error covariance matrix from P(-) to P(+)
    _kalmanFilter.correctWithMeasurementInnovation();

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
            LOG_WARN("{}: [{}] (HPH^T + R).rank = {}", nameId(), posVelObs->insTime.toYMDHMS(GPST), rank);
        }

        Eigen::FullPivLU<Eigen::MatrixXd> luP(_kalmanFilter.P(all, all));
        rank = luP.rank();
        if (rank != _kalmanFilter.P(all, all).rows())
        {
            LOG_WARN("{}: [{}] P.rank = {}", nameId(), posVelObs->insTime.toYMDHMS(GPST), rank);
        }
    }

    // LOG_DEBUG("{}: H\n{}\n", nameId(), _kalmanFilter.H);
    // LOG_DEBUG("{}: R\n{}\n", nameId(), _kalmanFilter.R);
    // LOG_DEBUG("{}: z =\n{}", nameId(), _kalmanFilter.z.transposed());

    // LOG_DEBUG("{}: K\n{}\n", nameId(), _kalmanFilter.K);
    // LOG_DEBUG("{}: x =\n{}", nameId(), _kalmanFilter.x.transposed());
    // LOG_DEBUG("{}: P\n{}\n", nameId(), _kalmanFilter.P);

    // LOG_DEBUG("{}: K * z =\n{}", nameId(), (_kalmanFilter.K(all, all) * _kalmanFilter.z(all)).transpose());

    // LOG_DEBUG("{}: P - P^T\n{}\n", nameId(), _kalmanFilter.P(all, all) - _kalmanFilter.P(all, all).transpose());

    // Push out the new data
    auto lckfSolution = std::make_shared<InsGnssLCKFSolution>();
    lckfSolution->insTime = posVelObs->insTime;
    lckfSolution->positionError = _kalmanFilter.x.segment<3>(KFPos);
    lckfSolution->velocityError = _kalmanFilter.x.segment<3>(KFVel);
    lckfSolution->attitudeError = _kalmanFilter.x.segment<3>(KFAtt) * (1. / SCALE_FACTOR_ATTITUDE);

    LOG_DATA("{}: Accumulated biases before error has been applied: b_biasAccel.value = {}, b_biasGyro.value = {}", nameId(), _inertialIntegrator.p_getLastAccelerationBias().transpose(), _inertialIntegrator.p_getLastAngularRateBias().transpose());

    _inertialIntegrator.applySensorBiasesIncrements(_lastImuObs->imuPos.p_quatAccel_b() * -_kalmanFilter.x.segment<3>(KFAccBias) * (1. / SCALE_FACTOR_ACCELERATION),
                                                    _lastImuObs->imuPos.p_quatGyro_b() * -_kalmanFilter.x.segment<3>(KFGyrBias) * (1. / SCALE_FACTOR_ANGULAR_RATE));
    lckfSolution->b_biasAccel.value = -_inertialIntegrator.p_getLastAccelerationBias();
    lckfSolution->b_biasAccel.stdDev = Eigen::Vector3d{
        std::sqrt(_kalmanFilter.P(KFStates::AccBiasX, KFStates::AccBiasX) * (1. / std::pow(SCALE_FACTOR_ACCELERATION, 2))),
        std::sqrt(_kalmanFilter.P(KFStates::AccBiasY, KFStates::AccBiasY) * (1. / std::pow(SCALE_FACTOR_ACCELERATION, 2))),
        std::sqrt(_kalmanFilter.P(KFStates::AccBiasZ, KFStates::AccBiasZ) * (1. / std::pow(SCALE_FACTOR_ACCELERATION, 2)))
    };
    lckfSolution->b_biasGyro.value = -_inertialIntegrator.p_getLastAngularRateBias();
    lckfSolution->b_biasGyro.stdDev = Eigen::Vector3d{
        std::sqrt(_kalmanFilter.P(KFStates::GyrBiasX, KFStates::GyrBiasX) * (1. / std::pow(SCALE_FACTOR_ANGULAR_RATE, 2))),
        std::sqrt(_kalmanFilter.P(KFStates::GyrBiasY, KFStates::GyrBiasY) * (1. / std::pow(SCALE_FACTOR_ANGULAR_RATE, 2))),
        std::sqrt(_kalmanFilter.P(KFStates::GyrBiasZ, KFStates::GyrBiasZ) * (1. / std::pow(SCALE_FACTOR_ANGULAR_RATE, 2)))
    };

    LOG_DATA("{}: Biases after error has been applied: b_biasAccel.value = {}, b_biasGyro.value = {}", nameId(), lckfSolution->b_biasAccel.value.transpose(), lckfSolution->b_biasGyro.value.transpose());

    Eigen::Matrix<double, 9, 9> deg2rad_2 = Eigen::Matrix<double, 9, 9>::Identity();
    deg2rad_2.bottomRightCorner<3, 3>().diagonal().setConstant(std::pow(std::numbers::pi_v<double> / 180.0, 2.0));
    if (_inertialIntegrator.getIntegrationFrame() == InertialIntegrator::IntegrationFrame::NED)
    {
        lckfSolution->positionError.topRows<2>() *= 1.0 / SCALE_FACTOR_LAT_LON;
        lckfSolution->frame = InsGnssLCKFSolution::Frame::NED;
        _inertialIntegrator.applyStateErrors_n(lckfSolution->positionError, lckfSolution->velocityError, lckfSolution->attitudeError);

        Eigen::Matrix<double, 10, 9> J = Eigen::Matrix<double, 10, 9>::Zero();
        J.topLeftCorner<6, 6>().setIdentity();
        J.bottomRightCorner<4, 3>() = trafo::covRPY2quatJacobian(_inertialIntegrator.getLatestState().value().get().n_Quat_b());

        lckfSolution->setPosVelAttAndCov_n(_inertialIntegrator.getLatestState().value().get().lla_position(),
                                           _inertialIntegrator.getLatestState().value().get().n_velocity(),
                                           _inertialIntegrator.getLatestState().value().get().n_Quat_b(),
                                           J * (_kalmanFilter.P(KFPosVelAtt, KFPosVelAtt) * deg2rad_2) * J.transpose());
    }
    else // if (_inertialIntegrator.getIntegrationFrame() == InertialIntegrator::IntegrationFrame::ECEF)
    {
        lckfSolution->frame = InsGnssLCKFSolution::Frame::ECEF;
        _inertialIntegrator.applyStateErrors_e(lckfSolution->positionError, lckfSolution->velocityError, lckfSolution->attitudeError);

        Eigen::Matrix<double, 10, 9> J = Eigen::Matrix<double, 10, 9>::Zero();
        J.topLeftCorner<6, 6>().setIdentity();
        J.bottomRightCorner<4, 3>() = trafo::covRPY2quatJacobian(_inertialIntegrator.getLatestState().value().get().e_Quat_b());

        lckfSolution->setPosVelAttAndCov_e(_inertialIntegrator.getLatestState().value().get().e_position(),
                                           _inertialIntegrator.getLatestState().value().get().e_velocity(),
                                           _inertialIntegrator.getLatestState().value().get().e_Quat_b(),
                                           J * (_kalmanFilter.P(KFPosVelAtt, KFPosVelAtt) * deg2rad_2) * J.transpose());
    }
    lckfSolution->heightBias = { .value = _heightBiasTotal, .stdDev = std::sqrt(_kalmanFilter.P(KFStates::HeightBias, KFStates::HeightBias)) };
    lckfSolution->heightScale = { .value = _heightScaleTotal, .stdDev = std::sqrt(_kalmanFilter.P(KFStates::HeightScale, KFStates::HeightScale)) };

    // Closed loop
    _kalmanFilter.x(all).setZero();

    LOG_DATA("{}: [{}] Sending out updated solution", nameId(), lckfSolution->insTime.toYMDHMS(GPST));
    if (!hasInputPinWithSameTime(lckfSolution->insTime))
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_SOLUTION, lckfSolution);
    }
}

void NAV::LooselyCoupledKF::looselyCoupledUpdate(const std::shared_ptr<const BaroHgt>& baroHgtObs)
{
    INS_ASSERT_USER_ERROR(_inertialIntegrator.getLatestState().has_value(), "The update should not even trigger without an initial state.");

    LOG_DATA("{}: [{}] Updating (lastInertial at [{}])", nameId(), baroHgtObs->insTime.toYMDHMS(GPST), _inertialIntegrator.getLatestState().value().get().insTime.toYMDHMS(GPST));

    // -------------------------------------------- GUI Parameters -----------------------------------------------

    // Latitude 𝜙, longitude λ and altitude (height above ground) in [rad, rad, m] at the time tₖ₋₁
    const Eigen::Vector3d& lla_position = _inertialIntegrator.getLatestState().value().get().lla_position();
    LOG_DATA("{}:     lla_position = {} [rad, rad, m]", nameId(), lla_position.transpose());

    // Baro height measurement uncertainty (Variance σ²) in [m^2]
    double baroHeightSigmaSquared{};
    if (_baroHeightMeasurementUncertaintyOverride || !baroHgtObs->baro_heightStdev.has_value())
    {
        switch (_barometricHeightMeasurementUncertaintyUnit)
        {
        case BaroHeightMeasurementUncertaintyUnit::m:
            baroHeightSigmaSquared = std::pow(_barometricHeightMeasurementUncertainty, 2);
            break;
        case BaroHeightMeasurementUncertaintyUnit::m2:
            baroHeightSigmaSquared = _barometricHeightMeasurementUncertainty;
            break;
        }
    }
    else
    {
        baroHeightSigmaSquared = std::pow(baroHgtObs->baro_heightStdev.value(), 2);
    }
    LOG_DATA("{}:     baroHeightSigmaSquared = {} [m^2]", nameId(), baroHeightSigmaSquared);

    // ---------------------------------------------- Correction -------------------------------------------------
    _kalmanFilter.setMeasurements(std::array{ KFMeas::dHgt });

    // 5. Calculate the measurement matrix H_k
    if (_inertialIntegrator.getIntegrationFrame() == InertialIntegrator::IntegrationFrame::NED)
    {
        _kalmanFilter.H = n_measurementMatrix_H(lla_position(2), _heightScaleTotal);
    }
    else // if (_inertialIntegrator.getIntegrationFrame() == InertialIntegrator::IntegrationFrame::ECEF)
    {
        _kalmanFilter.H = e_measurementMatrix_H(_inertialIntegrator.getLatestState().value().get().e_position(), lla_position(2), _heightScaleTotal);
    }

    // 6. Calculate the measurement noise covariance matrix R_k (here we can use the n-Frame covariance matrix! )
    _kalmanFilter.R = n_measurementNoiseCovariance_R(baroHeightSigmaSquared);

    // 8. Formulate the measurement z_k
    _kalmanFilter.z = n_measurementInnovation_dz(baroHgtObs->baro_height, lla_position(2), _heightBiasTotal, _heightScaleTotal);

    LOG_DATA("{}:     KF.H =\n{}", nameId(), _kalmanFilter.H);
    LOG_DATA("{}:     KF.R =\n{}", nameId(), _kalmanFilter.R);
    LOG_DATA("{}:     KF.z =\n{}", nameId(), _kalmanFilter.z);

    if (_checkKalmanMatricesRanks)
    {
        Eigen::FullPivLU<Eigen::MatrixXd> lu(_kalmanFilter.H(all, all) * _kalmanFilter.P(all, all) * _kalmanFilter.H(all, all).transpose() + _kalmanFilter.R(all, all));
        auto rank = lu.rank();
        if (rank != _kalmanFilter.H(all, all).rows())
        {
            LOG_WARN("{}: [{}] (HPH^T + R).rank = {}", nameId(), baroHgtObs->insTime.toYMDHMS(GPST), rank);
        }
    }

    // 7. Calculate the Kalman gain matrix K_k
    // 9. Update the state vector estimate from x(-) to x(+)
    // 10. Update the error covariance matrix from P(-) to P(+
    _kalmanFilter.correctWithMeasurementInnovation();

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
            LOG_WARN("{}: [{}] (HPH^T + R).rank = {}", nameId(), baroHgtObs->insTime.toYMDHMS(GPST), rank);
        }

        Eigen::FullPivLU<Eigen::MatrixXd> luP(_kalmanFilter.P(all, all));
        rank = luP.rank();
        if (rank != _kalmanFilter.P(all, all).rows())
        {
            LOG_WARN("{}: [{}] P.rank = {}", nameId(), baroHgtObs->insTime.toYMDHMS(GPST), rank);
        }
    }

    // LOG_DEBUG("{}: H\n{}\n", nameId(), _kalmanFilter.H);
    // LOG_DEBUG("{}: R\n{}\n", nameId(), _kalmanFilter.R);
    // LOG_DEBUG("{}: z =\n{}", nameId(), _kalmanFilter.z.transposed());

    // LOG_DEBUG("{}: K\n{}\n", nameId(), _kalmanFilter.K);
    // LOG_DEBUG("{}: x =\n{}", nameId(), _kalmanFilter.x.transposed());
    // LOG_DEBUG("{}: P\n{}\n", nameId(), _kalmanFilter.P);

    // LOG_DEBUG("{}: K * z =\n{}", nameId(), (_kalmanFilter.K(all, all) * _kalmanFilter.z(all)).transpose());

    // LOG_DEBUG("{}: P - P^T\n{}\n", nameId(), _kalmanFilter.P(all, all) - _kalmanFilter.P(all, all).transpose());

    // Push out the new data
    auto lckfSolution = std::make_shared<InsGnssLCKFSolution>();
    lckfSolution->insTime = baroHgtObs->insTime;
    lckfSolution->positionError = _kalmanFilter.x.segment<3>(KFPos);
    lckfSolution->velocityError = _kalmanFilter.x.segment<3>(KFVel);
    lckfSolution->attitudeError = _kalmanFilter.x.segment<3>(KFAtt) * (1. / SCALE_FACTOR_ATTITUDE);

    LOG_DATA("{}: Accumulated biases before error has been applied: b_biasAccel.value = {}, b_biasGyro.value = {}", nameId(), _inertialIntegrator.p_getLastAccelerationBias().transpose(), _inertialIntegrator.p_getLastAngularRateBias().transpose());

    _inertialIntegrator.applySensorBiasesIncrements(_lastImuObs->imuPos.p_quatAccel_b() * -_kalmanFilter.x.segment<3>(KFAccBias) * (1. / SCALE_FACTOR_ACCELERATION),
                                                    _lastImuObs->imuPos.p_quatGyro_b() * -_kalmanFilter.x.segment<3>(KFGyrBias) * (1. / SCALE_FACTOR_ANGULAR_RATE));
    lckfSolution->b_biasAccel.value = -_inertialIntegrator.p_getLastAccelerationBias();
    lckfSolution->b_biasAccel.stdDev = Eigen::Vector3d{
        std::sqrt(_kalmanFilter.P(KFStates::AccBiasX, KFStates::AccBiasX) * (1. / std::pow(SCALE_FACTOR_ACCELERATION, 2))),
        std::sqrt(_kalmanFilter.P(KFStates::AccBiasY, KFStates::AccBiasY) * (1. / std::pow(SCALE_FACTOR_ACCELERATION, 2))),
        std::sqrt(_kalmanFilter.P(KFStates::AccBiasZ, KFStates::AccBiasZ) * (1. / std::pow(SCALE_FACTOR_ACCELERATION, 2)))
    };
    lckfSolution->b_biasGyro.value = -_inertialIntegrator.p_getLastAngularRateBias();
    lckfSolution->b_biasGyro.stdDev = Eigen::Vector3d{
        std::sqrt(_kalmanFilter.P(KFStates::GyrBiasX, KFStates::GyrBiasX) * (1. / std::pow(SCALE_FACTOR_ANGULAR_RATE, 2))),
        std::sqrt(_kalmanFilter.P(KFStates::GyrBiasY, KFStates::GyrBiasY) * (1. / std::pow(SCALE_FACTOR_ANGULAR_RATE, 2))),
        std::sqrt(_kalmanFilter.P(KFStates::GyrBiasZ, KFStates::GyrBiasZ) * (1. / std::pow(SCALE_FACTOR_ANGULAR_RATE, 2)))
    };

    LOG_DATA("{}: Biases after error has been applied: b_biasAccel.value = {}, b_biasGyro.value = {}", nameId(), lckfSolution->b_biasAccel.value.transpose(), lckfSolution->b_biasGyro.value.transpose());

    Eigen::Matrix<double, 9, 9> deg2rad_2 = Eigen::Matrix<double, 9, 9>::Identity();
    deg2rad_2.bottomRightCorner<3, 3>().diagonal().setConstant(std::pow(std::numbers::pi_v<double> / 180.0, 2.0));
    if (_inertialIntegrator.getIntegrationFrame() == InertialIntegrator::IntegrationFrame::NED)
    {
        lckfSolution->positionError.topRows<2>() *= 1.0 / SCALE_FACTOR_LAT_LON;
        lckfSolution->frame = InsGnssLCKFSolution::Frame::NED;
        _inertialIntegrator.applyStateErrors_n(lckfSolution->positionError, lckfSolution->velocityError, lckfSolution->attitudeError);

        Eigen::Matrix<double, 10, 9> J = Eigen::Matrix<double, 10, 9>::Zero();
        J.topLeftCorner<6, 6>().setIdentity();
        J.bottomRightCorner<4, 3>() = trafo::covRPY2quatJacobian(_inertialIntegrator.getLatestState().value().get().n_Quat_b());

        lckfSolution->setPosVelAttAndCov_n(_inertialIntegrator.getLatestState().value().get().lla_position(),
                                           _inertialIntegrator.getLatestState().value().get().n_velocity(),
                                           _inertialIntegrator.getLatestState().value().get().n_Quat_b(),
                                           J * (_kalmanFilter.P(KFPosVelAtt, KFPosVelAtt) * deg2rad_2) * J.transpose());
        lckfSolution->setPosVelCovarianceMatrix_n(_kalmanFilter.P(KFPosVel, KFPosVel));
    }
    else // if (_inertialIntegrator.getIntegrationFrame() == InertialIntegrator::IntegrationFrame::ECEF)
    {
        lckfSolution->frame = InsGnssLCKFSolution::Frame::ECEF;
        _inertialIntegrator.applyStateErrors_e(lckfSolution->positionError, lckfSolution->velocityError, lckfSolution->attitudeError);

        Eigen::Matrix<double, 10, 9> J = Eigen::Matrix<double, 10, 9>::Zero();
        J.topLeftCorner<6, 6>().setIdentity();
        J.bottomRightCorner<4, 3>() = trafo::covRPY2quatJacobian(_inertialIntegrator.getLatestState().value().get().e_Quat_b());

        lckfSolution->setPosVelAttAndCov_e(_inertialIntegrator.getLatestState().value().get().e_position(),
                                           _inertialIntegrator.getLatestState().value().get().e_velocity(),
                                           _inertialIntegrator.getLatestState().value().get().e_Quat_b(),
                                           J * (_kalmanFilter.P(KFPosVelAtt, KFPosVelAtt) * deg2rad_2) * J.transpose());
        lckfSolution->setPosVelCovarianceMatrix_e(_kalmanFilter.P(KFPosVel, KFPosVel));
    }

    // Closed loop
    _heightBiasTotal += _kalmanFilter.x(KFStates::HeightBias);
    _heightScaleTotal += _kalmanFilter.x(KFStates::HeightScale);
    lckfSolution->heightBias = { .value = _heightBiasTotal, .stdDev = std::sqrt(_kalmanFilter.P(KFStates::HeightBias, KFStates::HeightBias)) };
    lckfSolution->heightScale = { .value = _heightScaleTotal, .stdDev = std::sqrt(_kalmanFilter.P(KFStates::HeightScale, KFStates::HeightScale)) };
    _kalmanFilter.x(all).setZero();

    LOG_DATA("{}: [{}] Sending out updated solution", nameId(), lckfSolution->insTime.toYMDHMS(GPST));
    if (!hasInputPinWithSameTime(lckfSolution->insTime))
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_SOLUTION, lckfSolution);
    }
}

// ###########################################################################################################
//                                             System matrix 𝐅
// ###########################################################################################################

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFStates, NAV::LooselyCoupledKF::KFStates, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>
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
    KeyedMatrix<double, KFStates, KFStates, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>
        F(Eigen::Matrix<double, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>::Zero(), States);

    F.block<3>(KFAtt, KFAtt) = n_F_dpsi_dpsi(n_omega_in);
    F.block<3>(KFAtt, KFVel) = n_F_dpsi_dv(latitude, altitude, R_N, R_E);
    F.block<3>(KFAtt, KFPos) = n_F_dpsi_dr(latitude, altitude, n_velocity, R_N, R_E);
    F.block<3>(KFAtt, KFGyrBias) = n_F_dpsi_dw(n_Quat_b.toRotationMatrix());
    F.block<3>(KFVel, KFAtt) = n_F_dv_dpsi(n_Quat_b * b_specForce_ib);
    F.block<3>(KFVel, KFVel) = n_F_dv_dv(n_velocity, latitude, altitude, R_N, R_E);
    F.block<3>(KFVel, KFPos) = n_F_dv_dr(n_velocity, latitude, altitude, R_N, R_E, g_0, r_eS_e);
    F.block<3>(KFVel, KFAccBias) = n_F_dv_df(n_Quat_b.toRotationMatrix());
    F.block<3>(KFPos, KFVel) = n_F_dr_dv(latitude, altitude, R_N, R_E);
    F.block<3>(KFPos, KFPos) = n_F_dr_dr(n_velocity, latitude, altitude, R_N, R_E);
    if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
    {
        F.block<3>(KFAccBias, KFAccBias) = n_F_df_df(_randomProcessAccel == RandomProcess::RandomWalk ? Eigen::Vector3d::Zero() : beta_bad);
        F.block<3>(KFGyrBias, KFGyrBias) = n_F_dw_dw(_randomProcessGyro == RandomProcess::RandomWalk ? Eigen::Vector3d::Zero() : beta_bgd);
    }

    F.middleRows<3>(KFAtt) *= SCALE_FACTOR_ATTITUDE; // 𝜓' [deg / s] = 180/π * ... [rad / s]
    F.middleCols<3>(KFAtt) *= 1. / SCALE_FACTOR_ATTITUDE;

    // F.middleRows<3>(Vel) *= 1.; // 𝛿v' [m / s^2] = 1 * [m / s^2]
    // F.middleCols<3>(Vel) *= 1. / 1.;

    F.middleRows<2>(std::array{ PosLat, PosLon }) *= SCALE_FACTOR_LAT_LON; // 𝛿ϕ' [pseudometre / s] = R0 * [rad / s]
    F.middleCols<2>(std::array{ PosLat, PosLon }) *= 1. / SCALE_FACTOR_LAT_LON;
    // F.row(PosAlt) *= 1.; // 𝛿h' [m / s] = 1 * [m / s]
    // F.col(PosAlt) *= 1. / 1.;

    F.middleRows<3>(KFAccBias) *= SCALE_FACTOR_ACCELERATION; // 𝛿f' [mg / s] = 1e3 / g * [m / s^3]
    F.middleCols<3>(KFAccBias) *= 1. / SCALE_FACTOR_ACCELERATION;

    F.middleRows<3>(KFGyrBias) *= SCALE_FACTOR_ANGULAR_RATE; // 𝛿ω' [mrad / s^2] = 1e3 * [rad / s^2]
    F.middleCols<3>(KFGyrBias) *= 1. / SCALE_FACTOR_ANGULAR_RATE;

    return F;
}

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFStates, NAV::LooselyCoupledKF::KFStates, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>
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
    KeyedMatrix<double, KFStates, KFStates, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>
        F(Eigen::Matrix<double, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>::Zero(), States);

    F.block<3>(KFAtt, KFAtt) = e_F_dpsi_dpsi(e_omega_ie.z());
    F.block<3>(KFAtt, KFGyrBias) = e_F_dpsi_dw(e_Quat_b.toRotationMatrix());
    F.block<3>(KFVel, KFAtt) = e_F_dv_dpsi(e_Quat_b * b_specForce_ib);
    F.block<3>(KFVel, KFVel) = e_F_dv_dv(e_omega_ie.z());
    F.block<3>(KFVel, KFPos) = e_F_dv_dr(e_position, e_gravitation, r_eS_e, e_omega_ie);
    F.block<3>(KFVel, KFAccBias) = e_F_dv_df(e_Quat_b.toRotationMatrix());
    F.block<3>(KFPos, KFVel) = e_F_dr_dv();
    if (_qCalculationAlgorithm == QCalculationAlgorithm::VanLoan)
    {
        F.block<3>(KFAccBias, KFAccBias) = e_F_df_df(_randomProcessAccel == RandomProcess::RandomWalk ? Eigen::Vector3d::Zero() : beta_bad);
        F.block<3>(KFGyrBias, KFGyrBias) = e_F_dw_dw(_randomProcessGyro == RandomProcess::RandomWalk ? Eigen::Vector3d::Zero() : beta_bgd);
    }

    F.middleRows<3>(KFAtt) *= SCALE_FACTOR_ATTITUDE; // 𝜓' [deg / s] = 180/π * ... [rad / s]
    F.middleCols<3>(KFAtt) *= 1. / SCALE_FACTOR_ATTITUDE;

    // F.middleRows<3>(Vel) *= 1.; // 𝛿v' [m / s^2] = 1 * [m / s^2]
    // F.middleCols<3>(Vel) *= 1. / 1.;

    // F.middleRows<3>(Pos) *= 1.; // 𝛿r' [m / s] = 1 * [m / s]
    // F.middleCols<3>(Pos) *= 1. / 1.;

    F.middleRows<3>(KFAccBias) *= SCALE_FACTOR_ACCELERATION; // 𝛿f' [mg / s] = 1e3 / g * [m / s^3]
    F.middleCols<3>(KFAccBias) *= 1. / SCALE_FACTOR_ACCELERATION;

    F.middleRows<3>(KFGyrBias) *= SCALE_FACTOR_ANGULAR_RATE; // 𝛿ω' [mrad / s^2] = 1e3 * [rad / s^2]
    F.middleCols<3>(KFGyrBias) *= 1. / SCALE_FACTOR_ANGULAR_RATE;

    return F;
}

// ###########################################################################################################
//                                    Noise input matrix 𝐆 & Noise scale matrix 𝐖
//                                     System noise covariance matrix 𝐐
// ###########################################################################################################

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFStates, NAV::LooselyCoupledKF::KFStates, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>
    NAV::LooselyCoupledKF::noiseInputMatrix_G(const Eigen::Quaterniond& ien_Quat_b)
{
    // DCM matrix from body to navigation frame
    Eigen::Matrix3d ien_Dcm_b = ien_Quat_b.toRotationMatrix();

    // Math: \mathbf{G}_{a} = \begin{bmatrix} -\mathbf{C}_b^{i,e,n} & 0 & 0 & 0 \\ 0 & \mathbf{C}_b^{i,e,n} & 0 & 0 \\ 0 & 0 & 0 & 0 \\ 0 & 0 & \mathbf{I}_3 & 0 \\ 0 & 0 & 0 & \mathbf{I}_3 \end{bmatrix}
    KeyedMatrix<double, KFStates, KFStates, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>
        G(Eigen::Matrix<double, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>::Zero(), States, States);

    G.block<3>(KFAtt, KFAtt) = SCALE_FACTOR_ATTITUDE * ien_Dcm_b;
    G.block<3>(KFVel, KFVel) = ien_Dcm_b;
    G.block<3>(KFAccBias, KFAccBias) = SCALE_FACTOR_ACCELERATION * Eigen::Matrix3d::Identity();
    G.block<3>(KFGyrBias, KFGyrBias) = SCALE_FACTOR_ANGULAR_RATE * Eigen::Matrix3d::Identity();
    G(KFStates::HeightBias, KFStates::HeightBias) = 1.0;
    G(KFStates::HeightScale, KFStates::HeightScale) = 1.0;
    return G;
}

Eigen::Matrix<double, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>
    NAV::LooselyCoupledKF::noiseScaleMatrix_W(const Eigen::Vector3d& sigma_ra, const Eigen::Vector3d& sigma_rg,
                                              const Eigen::Vector3d& sigma_bad, const Eigen::Vector3d& sigma_bgd,
                                              const Eigen::Vector3d& tau_bad, const Eigen::Vector3d& tau_bgd,
                                              const double& sigma_heightBias, const double& sigma_heightScale)
{
    Eigen::Matrix<double, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT> W =
        Eigen::Matrix<double, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>::Zero();

    W.diagonal() << sigma_rg.array().square(),
        sigma_ra.array().square(),
        Eigen::Vector3d::Zero(),
        (_randomProcessAccel == RandomProcess::RandomWalk ? sigma_bad : psdBiasGaussMarkov(sigma_bad.array().square(), tau_bad)).array().square(), // S_bad
        (_randomProcessGyro == RandomProcess::RandomWalk ? sigma_bgd : psdBiasGaussMarkov(sigma_bgd.array().square(), tau_bgd)).array().square(),  // S_bgd
        sigma_heightBias * sigma_heightBias,
        sigma_heightScale * sigma_heightScale;

    return W;
}

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFStates, NAV::LooselyCoupledKF::KFStates, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>
    NAV::LooselyCoupledKF::n_systemNoiseCovarianceMatrix_Q(const Eigen::Vector3d& sigma2_ra, const Eigen::Vector3d& sigma2_rg,
                                                           const Eigen::Vector3d& sigma2_bad, const Eigen::Vector3d& sigma2_bgd,
                                                           const double& sigma_heightBias, const double& sigma_heightScale,
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

    KeyedMatrix<double, KFStates, KFStates, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>
        Q(Eigen::Matrix<double, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>::Zero(), States, States);
    Q.block<3>(KFAtt, KFAtt) = Q_psi_psi(S_rg, S_bgd, tau_s);                              // Q_11
    Q.block<3>(KFVel, KFAtt) = ien_Q_dv_psi(S_rg, S_bgd, n_F_21, tau_s);                   // Q_21
    Q.block<3>(KFVel, KFVel) = ien_Q_dv_dv(S_ra, S_bad, S_rg, S_bgd, n_F_21, tau_s);       // Q_22
    Q.block<3>(KFVel, KFGyrBias) = ien_Q_dv_domega(S_bgd, n_F_21, n_Dcm_b, tau_s);         // Q_25
    Q.block<3>(KFPos, KFAtt) = n_Q_dr_psi(S_rg, S_bgd, n_F_21, T_rn_p, tau_s);             // Q_31
    Q.block<3>(KFPos, KFVel) = n_Q_dr_dv(S_ra, S_bad, S_rg, S_bgd, n_F_21, T_rn_p, tau_s); // Q_32
    Q.block<3>(KFPos, KFPos) = n_Q_dr_dr(S_ra, S_bad, S_rg, S_bgd, n_F_21, T_rn_p, tau_s); // Q_33
    Q.block<3>(KFPos, KFAccBias) = n_Q_dr_df(S_bgd, T_rn_p, n_Dcm_b, tau_s);               // Q_34
    Q.block<3>(KFPos, KFGyrBias) = n_Q_dr_domega(S_bgd, n_F_21, T_rn_p, n_Dcm_b, tau_s);   // Q_35
    Q.block<3>(KFAccBias, KFVel) = Q_df_dv(S_bad, b_Dcm_n, tau_s);                         // Q_42
    Q.block<3>(KFAccBias, KFAccBias) = Q_df_df(S_bad, tau_s);                              // Q_44
    Q.block<3>(KFGyrBias, KFAtt) = Q_domega_psi(S_bgd, b_Dcm_n, tau_s);                    // Q_51
    Q.block<3>(KFGyrBias, KFGyrBias) = Q_domega_domega(S_bgd, tau_s);                      // Q_55

    Q.block<3>(KFAtt, KFVel) = Q.block<3>(KFVel, KFAtt).transpose();         // Q_21^T
    Q.block<3>(KFAtt, KFPos) = Q.block<3>(KFPos, KFAtt).transpose();         // Q_31^T
    Q.block<3>(KFVel, KFPos) = Q.block<3>(KFPos, KFVel).transpose();         // Q_32^T
    Q.block<3>(KFAccBias, KFPos) = Q.block<3>(KFPos, KFAccBias).transpose(); // Q_34^T
    Q.block<3>(KFGyrBias, KFVel) = Q.block<3>(KFVel, KFGyrBias).transpose(); // Q_25^T
    Q.block<3>(KFGyrBias, KFPos) = Q.block<3>(KFPos, KFGyrBias).transpose(); // Q_35^T
    Q.block<3>(KFVel, KFAccBias) = Q.block<3>(KFAccBias, KFVel).transpose(); // Q_42^T
    Q.block<3>(KFAtt, KFGyrBias) = Q.block<3>(KFGyrBias, KFAtt).transpose(); // Q_51^T
    Q(KFStates::HeightBias, KFStates::HeightBias) = sigma_heightBias * sigma_heightBias * tau_s;
    Q(KFStates::HeightScale, KFStates::HeightScale) = sigma_heightScale * sigma_heightScale * tau_s;

    Q.middleRows<3>(KFAtt) *= SCALE_FACTOR_ATTITUDE;
    Q.middleRows<2>(std::array{ PosLat, PosLon }) *= SCALE_FACTOR_LAT_LON;
    Q.middleRows<3>(KFAccBias) *= SCALE_FACTOR_ACCELERATION;
    Q.middleRows<3>(KFGyrBias) *= SCALE_FACTOR_ANGULAR_RATE;

    Q.middleCols<3>(KFAtt) *= SCALE_FACTOR_ATTITUDE;
    Q.middleCols<2>(std::array{ PosLat, PosLon }) *= SCALE_FACTOR_LAT_LON;
    Q.middleCols<3>(KFAccBias) *= SCALE_FACTOR_ACCELERATION;
    Q.middleCols<3>(KFGyrBias) *= SCALE_FACTOR_ANGULAR_RATE;

    return Q;
}

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFStates, NAV::LooselyCoupledKF::KFStates, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>
    NAV::LooselyCoupledKF::e_systemNoiseCovarianceMatrix_Q(const Eigen::Vector3d& sigma2_ra, const Eigen::Vector3d& sigma2_rg,
                                                           const Eigen::Vector3d& sigma2_bad, const Eigen::Vector3d& sigma2_bgd,
                                                           const double& sigma_heightBias, const double& sigma_heightScale,
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

    KeyedMatrix<double, KFStates, KFStates, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>
        Q(Eigen::Matrix<double, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>::Zero(), States, States);
    Q.block<3>(KFAtt, KFAtt) = Q_psi_psi(S_rg, S_bgd, tau_s);                        // Q_11
    Q.block<3>(KFVel, KFAtt) = ien_Q_dv_psi(S_rg, S_bgd, e_F_21, tau_s);             // Q_21
    Q.block<3>(KFVel, KFVel) = ien_Q_dv_dv(S_ra, S_bad, S_rg, S_bgd, e_F_21, tau_s); // Q_22
    Q.block<3>(KFVel, KFGyrBias) = ien_Q_dv_domega(S_bgd, e_F_21, e_Dcm_b, tau_s);   // Q_25
    Q.block<3>(KFPos, KFAtt) = ie_Q_dr_psi(S_rg, S_bgd, e_F_21, tau_s);              // Q_31
    Q.block<3>(KFPos, KFVel) = ie_Q_dr_dv(S_ra, S_bad, S_rg, S_bgd, e_F_21, tau_s);  // Q_32
    Q.block<3>(KFPos, KFPos) = ie_Q_dr_dr(S_ra, S_bad, S_rg, S_bgd, e_F_21, tau_s);  // Q_33
    Q.block<3>(KFPos, KFAccBias) = ie_Q_dr_df(S_bgd, e_Dcm_b, tau_s);                // Q_34
    Q.block<3>(KFPos, KFGyrBias) = ie_Q_dr_domega(S_bgd, e_F_21, e_Dcm_b, tau_s);    // Q_35
    Q.block<3>(KFAccBias, KFVel) = Q_df_dv(S_bad, b_Dcm_e, tau_s);                   // Q_42
    Q.block<3>(KFAccBias, KFAccBias) = Q_df_df(S_bad, tau_s);                        // Q_44
    Q.block<3>(KFGyrBias, KFAtt) = Q_domega_psi(S_bgd, b_Dcm_e, tau_s);              // Q_51
    Q.block<3>(KFGyrBias, KFGyrBias) = Q_domega_domega(S_bgd, tau_s);                // Q_55

    Q.block<3>(KFAtt, KFVel) = Q.block<3>(KFVel, KFAtt).transpose();         // Q_21^T
    Q.block<3>(KFAtt, KFPos) = Q.block<3>(KFPos, KFAtt).transpose();         // Q_31^T
    Q.block<3>(KFVel, KFPos) = Q.block<3>(KFPos, KFVel).transpose();         // Q_32^T
    Q.block<3>(KFAccBias, KFPos) = Q.block<3>(KFPos, KFAccBias).transpose(); // Q_34^T
    Q.block<3>(KFGyrBias, KFVel) = Q.block<3>(KFVel, KFGyrBias).transpose(); // Q_25^T
    Q.block<3>(KFGyrBias, KFPos) = Q.block<3>(KFPos, KFGyrBias).transpose(); // Q_35^T
    Q.block<3>(KFVel, KFAccBias) = Q.block<3>(KFAccBias, KFVel).transpose(); // Q_42^T
    Q.block<3>(KFAtt, KFGyrBias) = Q.block<3>(KFGyrBias, KFAtt).transpose(); // Q_51^T
    Q(KFStates::HeightBias, KFStates::HeightBias) = sigma_heightBias * sigma_heightBias * tau_s;
    Q(KFStates::HeightScale, KFStates::HeightScale) = sigma_heightScale * sigma_heightScale * tau_s;

    Q.middleRows<3>(KFAtt) *= SCALE_FACTOR_ATTITUDE;
    Q.middleRows<3>(KFAccBias) *= SCALE_FACTOR_ACCELERATION;
    Q.middleRows<3>(KFGyrBias) *= SCALE_FACTOR_ANGULAR_RATE;

    Q.middleCols<3>(KFAtt) *= SCALE_FACTOR_ATTITUDE;
    Q.middleCols<3>(KFAccBias) *= SCALE_FACTOR_ACCELERATION;
    Q.middleCols<3>(KFGyrBias) *= SCALE_FACTOR_ANGULAR_RATE;

    return Q;
}

// ###########################################################################################################
//                                         Error covariance matrix P
// ###########################################################################################################

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFStates, NAV::LooselyCoupledKF::KFStates, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>
    NAV::LooselyCoupledKF::initialErrorCovarianceMatrix_P0(const Eigen::Vector3d& variance_angles,
                                                           const Eigen::Vector3d& variance_vel,
                                                           const Eigen::Vector3d& variance_pos,
                                                           const Eigen::Vector3d& variance_accelBias,
                                                           const Eigen::Vector3d& variance_gyroBias,
                                                           const double& variance_heightBias,
                                                           const double& variance_heightScale) const
{
    double scaleFactorPosition = _inertialIntegrator.getIntegrationFrame() == InertialIntegrator::IntegrationFrame::NED ? SCALE_FACTOR_LAT_LON : 1.0;

    // 𝐏 Error covariance matrix
    Eigen::Matrix<double, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT> P =
        Eigen::Matrix<double, NAV::LooselyCoupledKF::KFStates_COUNT, NAV::LooselyCoupledKF::KFStates_COUNT>::Zero();

    P.diagonal() << std::pow(SCALE_FACTOR_ATTITUDE, 2) * variance_angles, // Flight Angles covariance
        variance_vel,                                                     // Velocity covariance
        std::pow(scaleFactorPosition, 2) * variance_pos(0),               // Latitude/Pos X covariance
        std::pow(scaleFactorPosition, 2) * variance_pos(1),               // Longitude/Pos Y covariance
        variance_pos(2),                                                  // Altitude/Pos Z covariance
        std::pow(SCALE_FACTOR_ACCELERATION, 2) * variance_accelBias,      // Accelerometer Bias covariance
        std::pow(SCALE_FACTOR_ANGULAR_RATE, 2) * variance_gyroBias,       // Gyroscope Bias covariance
        variance_heightBias,                                              // Height Bias covariance
        variance_heightScale;                                             // Height Scale covariance
    return { P, States };
}

// ###########################################################################################################
//                                                Correction
// ###########################################################################################################

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFMeas, NAV::LooselyCoupledKF::KFStates, 6, NAV::LooselyCoupledKF::KFStates_COUNT>
    NAV::LooselyCoupledKF::n_measurementMatrix_H(const Eigen::Matrix3d& T_rn_p, const Eigen::Matrix3d& n_Dcm_b, const Eigen::Vector3d& b_omega_ib, const Eigen::Vector3d& b_leverArm_InsGnss, const Eigen::Matrix3d& n_Omega_ie)
{
    // Math: \mathbf{H}_{G,k}^n = \begin{pmatrix} \mathbf{H}_{r1}^n & \mathbf{0}_3 & -\mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{H}_{v1}^n & -\mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{H}_{v5}^n \end{pmatrix}_k \qquad \text{P. Groves}\,(14.113)
    // G denotes GNSS indicated

    NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFMeas, NAV::LooselyCoupledKF::KFStates, 6, NAV::LooselyCoupledKF::KFStates_COUNT>
        H(Eigen::Matrix<double, 6, NAV::LooselyCoupledKF::KFStates_COUNT>::Zero(), Meas, States);

    // Math: \mathbf{H}_{r1}^n \approx \mathbf{\hat{T}}_{r(n)}^p \begin{bmatrix} \begin{pmatrix} \mathbf{C}_b^n \mathbf{l}_{ba}^p \end{pmatrix} \wedge \end{bmatrix} \qquad \text{P. Groves}\,(14.114)
    H.block<3>(dPos, KFAtt) = T_rn_p * math::skewSymmetricMatrix(n_Dcm_b * b_leverArm_InsGnss);
    H.block<3>(dPos, KFPos) = -Eigen::Matrix3d::Identity();
    // Math: \mathbf{H}_{v1}^n \approx \begin{bmatrix} \begin{Bmatrix} \mathbf{C}_b^n (\mathbf{\hat{\omega}}_{ib}^b \wedge \mathbf{l}_{ba}^b) - \mathbf{\hat{\Omega}}_{ie}^n \mathbf{C}_b^n \mathbf{l}_{ba}^b \end{Bmatrix} \wedge \end{bmatrix} \qquad \text{P. Groves}\,(14.114)
    H.block<3>(dVel, KFAtt) = math::skewSymmetricMatrix(n_Dcm_b * (b_omega_ib.cross(b_leverArm_InsGnss)) - n_Omega_ie * n_Dcm_b * b_leverArm_InsGnss);
    H.block<3>(dVel, KFVel) = -Eigen::Matrix3d::Identity();
    // Math: \mathbf{H}_{v5}^n = \mathbf{C}_b^n \begin{bmatrix} \mathbf{l}_{ba}^b \wedge \end{bmatrix} \qquad \text{P. Groves}\,(14.114)
    H.block<3>(dVel, KFGyrBias) = n_Dcm_b * math::skewSymmetricMatrix(b_leverArm_InsGnss);

    H.middleRows<2>(std::array{ dPosLat, dPosLon }) *= SCALE_FACTOR_LAT_LON;

    H.middleCols<3>(KFAtt) *= 1. / SCALE_FACTOR_ATTITUDE;
    H.middleCols<2>(std::array{ PosLat, PosLon }) *= 1. / SCALE_FACTOR_LAT_LON;
    // H.middleCols<3>(AccBias) *= 1. / SCALE_FACTOR_ACCELERATION; // Only zero elements
    H.middleCols<3>(KFGyrBias) *= 1. / SCALE_FACTOR_ANGULAR_RATE;

    return H;
}

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFMeas, NAV::LooselyCoupledKF::KFStates, 1, NAV::LooselyCoupledKF::KFStates_COUNT>
    NAV::LooselyCoupledKF::n_measurementMatrix_H(const double& height, const double& scale)
{
    NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFMeas, NAV::LooselyCoupledKF::KFStates, 1, NAV::LooselyCoupledKF::KFStates_COUNT>
        H(Eigen::Matrix<double, 1, NAV::LooselyCoupledKF::KFStates_COUNT>::Zero(), std::array{ KFMeas::dHgt }, States);

    H(KFMeas::dHgt, PosAlt) = -scale;
    H(KFMeas::dHgt, HeightBias) = 1.0;
    H(KFMeas::dHgt, HeightScale) = height;

    return H;
}

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFMeas, NAV::LooselyCoupledKF::KFStates, 6, NAV::LooselyCoupledKF::KFStates_COUNT>
    NAV::LooselyCoupledKF::e_measurementMatrix_H(const Eigen::Matrix3d& e_Dcm_b, const Eigen::Vector3d& b_omega_ib, const Eigen::Vector3d& b_leverArm_InsGnss, const Eigen::Matrix3d& e_Omega_ie)
{
    // Math: \mathbf{H}_{G,k}^e = \begin{pmatrix} \mathbf{H}_{r1}^e & \mathbf{0}_3 & -\mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 \\ \mathbf{H}_{v1}^e & -\mathbf{I}_3 & \mathbf{0}_3 & \mathbf{0}_3 & \mathbf{H}_{v5}^e \end{pmatrix}_k \qquad \text{P. Groves}\,(14.113)
    // G denotes GNSS indicated

    NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFMeas, NAV::LooselyCoupledKF::KFStates, 6, NAV::LooselyCoupledKF::KFStates_COUNT>
        H(Eigen::Matrix<double, 6, NAV::LooselyCoupledKF::KFStates_COUNT>::Zero(), Meas, States);

    // Math: \mathbf{H}_{r1}^e \approx \begin{bmatrix} \begin{pmatrix} \mathbf{C}_b^e \mathbf{l}_{ba}^p \end{pmatrix} \wedge \end{bmatrix} \qquad \text{P. Groves}\,(14.114)
    H.block<3>(dPos, KFAtt) = math::skewSymmetricMatrix(e_Dcm_b * b_leverArm_InsGnss);
    H.block<3>(dPos, KFPos) = -Eigen::Matrix3d::Identity();
    // Math: \mathbf{H}_{v1}^e \approx \begin{bmatrix} \begin{Bmatrix} \mathbf{C}_b^e (\mathbf{\hat{\omega}}_{ib}^b \wedge \mathbf{l}_{ba}^b) - \mathbf{\hat{\Omega}}_{ie}^e \mathbf{C}_b^e \mathbf{l}_{ba}^b \end{Bmatrix} \wedge \end{bmatrix} \qquad \text{P. Groves}\,(14.114)
    H.block<3>(dVel, KFAtt) = math::skewSymmetricMatrix(e_Dcm_b * (b_omega_ib.cross(b_leverArm_InsGnss)) - e_Omega_ie * e_Dcm_b * b_leverArm_InsGnss);
    H.block<3>(dVel, KFVel) = -Eigen::Matrix3d::Identity();
    // Math: \mathbf{H}_{v5}^e = \mathbf{C}_b^e \begin{bmatrix} \mathbf{l}_{ba}^b \wedge \end{bmatrix} \qquad \text{P. Groves}\,(14.114)
    H.block<3>(dVel, KFGyrBias) = e_Dcm_b * math::skewSymmetricMatrix(b_leverArm_InsGnss);

    H.middleCols<3>(KFAtt) *= 1. / SCALE_FACTOR_ATTITUDE;
    // H.middleCols<3>(AccBias) *= 1. / SCALE_FACTOR_ACCELERATION; // Only zero elements
    H.middleCols<3>(KFGyrBias) *= 1. / SCALE_FACTOR_ANGULAR_RATE;

    return H;
}

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFMeas, NAV::LooselyCoupledKF::KFStates, 1, NAV::LooselyCoupledKF::KFStates_COUNT>
    NAV::LooselyCoupledKF::e_measurementMatrix_H(const Eigen::Vector3d& e_positionEstimate, const double& height, const double& scale)
{
    NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFMeas, NAV::LooselyCoupledKF::KFStates, 1, NAV::LooselyCoupledKF::KFStates_COUNT>
        H(Eigen::Matrix<double, 1, NAV::LooselyCoupledKF::KFStates_COUNT>::Zero(), std::array{ KFMeas::dHgt }, States);

    H(KFMeas::dHgt, KFPos) = -scale * e_positionEstimate.normalized().transpose();
    H(KFMeas::dHgt, HeightBias) = 1.0;
    H(KFMeas::dHgt, HeightScale) = height;

    return H;
}

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFMeas, NAV::LooselyCoupledKF::KFMeas, 6, 6>
    NAV::LooselyCoupledKF::n_measurementNoiseCovariance_R(const std::shared_ptr<const PosVel>& posVelObs,
                                                          const Eigen::Vector3d& lla_position,
                                                          double R_N,
                                                          double R_E) const
{
    // GNSS measurement uncertainty for the position (Variance σ²) in [m^2]
    Eigen::Vector3d gnssSigmaSquaredPosition = Eigen::Vector3d::Zero();
    switch (_gnssMeasurementUncertaintyPositionUnit)
    {
    case GnssMeasurementUncertaintyPositionUnit::meter:
        gnssSigmaSquaredPosition = _gnssMeasurementUncertaintyPosition.array().pow(2);
        break;
    case GnssMeasurementUncertaintyPositionUnit::meter2:
        gnssSigmaSquaredPosition = _gnssMeasurementUncertaintyPosition;
        break;
    }
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

    KeyedMatrix<double, KFMeas, KFMeas, 6, 6> R(Eigen::Matrix<double, 6, 6>::Zero(), Meas);

    if (!_gnssMeasurementUncertaintyPositionOverride && !_gnssMeasurementUncertaintyVelocityOverride
        && posVelObs->n_CovarianceMatrix() && posVelObs->n_CovarianceMatrix()->get().hasRows(Keys::PosVel<Keys::MotionModelKey>))
    {
        R(all, all) = posVelObs->n_CovarianceMatrix()->get()(Keys::PosVel<Keys::MotionModelKey>, Keys::PosVel<Keys::MotionModelKey>);
    }
    else if (!_gnssMeasurementUncertaintyPositionOverride
             && posVelObs->n_CovarianceMatrix() && posVelObs->n_CovarianceMatrix()->get().hasRows(Keys::Pos<Keys::MotionModelKey>))
    {
        R.block<3>(dPos, dPos) = posVelObs->n_CovarianceMatrix()->get()(Keys::Pos<Keys::MotionModelKey>, Keys::Pos<Keys::MotionModelKey>);
        R.block<3>(dVel, dVel).diagonal() = gnssSigmaSquaredVelocity;
    }
    else if (!_gnssMeasurementUncertaintyVelocityOverride
             && posVelObs->n_CovarianceMatrix() && posVelObs->n_CovarianceMatrix()->get().hasRows(Keys::Vel<Keys::MotionModelKey>))
    {
        R.block<3>(dPos, dPos).diagonal() = gnssSigmaSquaredPosition;
        R.block<3>(dVel, dVel) = posVelObs->n_CovarianceMatrix()->get()(Keys::Vel<Keys::MotionModelKey>, Keys::Vel<Keys::MotionModelKey>);
    }
    else // if (_gnssMeasurementUncertaintyPositionOverride && _gnssMeasurementUncertaintyVelocityOverride)
    {
        R.block<3>(dPos, dPos).diagonal() = gnssSigmaSquaredPosition;
        R.block<3>(dVel, dVel).diagonal() = gnssSigmaSquaredVelocity;
    }

    // transform NED covariance into LLA covariance
    Eigen::Matrix6d J = Eigen::Matrix6d::Identity();
    J.topLeftCorner<3, 3>() = n_F_dr_dv(lla_position.x(), lla_position.z(), R_N, R_E);
    R(all, all) = J * R(all, all) * J.transpose();

    // scale units
    R.middleRows<2>(std::array{ dPosLat, dPosLon }) *= SCALE_FACTOR_LAT_LON;
    R.middleCols<2>(std::array{ dPosLat, dPosLon }) *= SCALE_FACTOR_LAT_LON;

    return R;
}

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFMeas, NAV::LooselyCoupledKF::KFMeas, 1, 1>
    NAV::LooselyCoupledKF::n_measurementNoiseCovariance_R(const double& baroVarianceHeight)
{
    KeyedMatrix<double, KFMeas, KFMeas, 1, 1> R(Eigen::Matrix<double, 1, 1>::Zero(), std::array{ KFMeas::dHgt });
    R(KFMeas::dHgt, KFMeas::dHgt) = baroVarianceHeight;

    return R;
}

NAV::KeyedMatrix<double, NAV::LooselyCoupledKF::KFMeas, NAV::LooselyCoupledKF::KFMeas, 6, 6>
    NAV::LooselyCoupledKF::e_measurementNoiseCovariance_R(const std::shared_ptr<const PosVel>& posVelObs) const
{
    // GNSS measurement uncertainty for the position (Variance σ²) in [m^2]
    Eigen::Vector3d gnssSigmaSquaredPosition = Eigen::Vector3d::Zero();
    switch (_gnssMeasurementUncertaintyPositionUnit)
    {
    case GnssMeasurementUncertaintyPositionUnit::meter:
        gnssSigmaSquaredPosition = _gnssMeasurementUncertaintyPosition.array().pow(2);
        break;
    case GnssMeasurementUncertaintyPositionUnit::meter2:
        gnssSigmaSquaredPosition = _gnssMeasurementUncertaintyPosition;
        break;
    }
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

    KeyedMatrix<double, KFMeas, KFMeas, 6, 6> R(Eigen::Matrix<double, 6, 6>::Zero(), Meas);

    if (!_gnssMeasurementUncertaintyPositionOverride && !_gnssMeasurementUncertaintyVelocityOverride
        && posVelObs->e_CovarianceMatrix() && posVelObs->e_CovarianceMatrix()->get().hasRows(Keys::PosVel<Keys::MotionModelKey>))
    {
        R(all, all) = posVelObs->e_CovarianceMatrix()->get()(Keys::PosVel<Keys::MotionModelKey>, Keys::PosVel<Keys::MotionModelKey>);
    }
    else if (!_gnssMeasurementUncertaintyPositionOverride
             && posVelObs->e_CovarianceMatrix() && posVelObs->e_CovarianceMatrix()->get().hasRows(Keys::Pos<Keys::MotionModelKey>))
    {
        R.block<3>(dPos, dPos) = posVelObs->e_CovarianceMatrix()->get()(Keys::Pos<Keys::MotionModelKey>, Keys::Pos<Keys::MotionModelKey>);
        R.block<3>(dVel, dVel).diagonal() = gnssSigmaSquaredVelocity;
    }
    else if (!_gnssMeasurementUncertaintyVelocityOverride
             && posVelObs->e_CovarianceMatrix() && posVelObs->e_CovarianceMatrix()->get().hasRows(Keys::Vel<Keys::MotionModelKey>))
    {
        R.block<3>(dPos, dPos).diagonal() = gnssSigmaSquaredPosition;
        R.block<3>(dVel, dVel) = posVelObs->e_CovarianceMatrix()->get()(Keys::Vel<Keys::MotionModelKey>, Keys::Vel<Keys::MotionModelKey>);
    }
    else // if (_gnssMeasurementUncertaintyPositionOverride && _gnssMeasurementUncertaintyVelocityOverride)
    {
        R.block<3>(dPos, dPos).diagonal() = gnssSigmaSquaredPosition;
        R.block<3>(dVel, dVel).diagonal() = gnssSigmaSquaredVelocity;
    }

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

NAV::KeyedVector<double, NAV::LooselyCoupledKF::KFMeas, 1>
    NAV::LooselyCoupledKF::n_measurementInnovation_dz(const double& baroheight, const double& height, const double& heightbias, const double& heightscale)
{
    Eigen::Matrix<double, 1, 1> innovation;
    innovation << baroheight - (height * heightscale + heightbias);

    return { innovation, std::array{ KFMeas::dHgt } };
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

std::ostream& operator<<(std::ostream& os, const NAV::LooselyCoupledKF::KFStates& obj)
{
    return os << fmt::format("{}", obj);
}
std::ostream& operator<<(std::ostream& os, const NAV::LooselyCoupledKF::KFMeas& obj)
{
    return os << fmt::format("{}", obj);
}