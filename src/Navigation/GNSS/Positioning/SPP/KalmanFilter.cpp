// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file KalmanFilter.cpp
/// @brief SPP Kalman Filter
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-12-22

#include "KalmanFilter.hpp"

#include <algorithm>
#include <functional>
#include <variant>

#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "Navigation/GNSS/Positioning/SPP/Keys.hpp"
#include "Navigation/GNSS/SystemModel/InterFrequencyBiasModel.hpp"
#include "Navigation/GNSS/SystemModel/ReceiverClockModel.hpp"
#include "Navigation/GNSS/SystemModel/SystemModel.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "util/Logger.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <fmt/format.h>

namespace NAV::SPP
{
void KalmanFilter::reset(const std::vector<SatelliteSystem>& satelliteSystems)
{
    // Covariance of the P matrix initialization velocity uncertainty [m²/s²]
    switch (_gui_initCovarianceVelocityUnit)
    {
    case InitCovarianceVelocityUnits::m_s:
        _initCovarianceVelocity = std::pow(_gui_initCovarianceVelocity, 2);
        break;
    case InitCovarianceVelocityUnits::m2_s2:
        _initCovarianceVelocity = _gui_initCovarianceVelocity;
        break;
    }

    // Covariance of the P matrix initialization clock drift uncertainty [m²/s²]
    switch (_gui_initCovarianceClockDriftUnit)
    {
    case InitCovarianceClockDriftUnits::m_s:
        _initCovarianceClockDrift = std::pow(_gui_initCovarianceClockDrift, 2);
        break;
    case InitCovarianceClockDriftUnits::s_s:
        _initCovarianceClockDrift = std::pow(_gui_initCovarianceClockDrift * InsConst::C, 2);
        break;
    case InitCovarianceClockDriftUnits::m2_s2:
        _initCovarianceClockDrift = _gui_initCovarianceClockDrift;
        break;
    case InitCovarianceClockDriftUnits::s2_s2:
        _initCovarianceClockDrift = _gui_initCovarianceClockDrift * std::pow(InsConst::C, 2);
        break;
    }

    // Covariance of the P matrix initialization inter system clock drift uncertainty [m²/s²]
    switch (_gui_initCovarianceInterSysClockDriftUnit)
    {
    case InitCovarianceClockDriftUnits::m_s:
        _initCovarianceInterSysClockDrift = std::pow(_gui_initCovarianceInterSysClockDrift, 2);
        break;
    case InitCovarianceClockDriftUnits::s_s:
        _initCovarianceInterSysClockDrift = std::pow(_gui_initCovarianceInterSysClockDrift * InsConst::C, 2);
        break;
    case InitCovarianceClockDriftUnits::m2_s2:
        _initCovarianceInterSysClockDrift = _gui_initCovarianceInterSysClockDrift;
        break;
    case InitCovarianceClockDriftUnits::s2_s2:
        _initCovarianceInterSysClockDrift = _gui_initCovarianceInterSysClockDrift * std::pow(InsConst::C, 2);
        break;
    }

    // ###########################################################################################################

    // Reset values to 0 and remove inter system states
    _kalmanFilter = KeyedKalmanFilterD<States::StateKeyType,
                                       Meas::MeasKeyTypes>{ PosVelKey, {} };

    _motionModel.initialize(_kalmanFilter.F, _kalmanFilter.W);

    for (const auto& satSys : satelliteSystems)
    {
        _kalmanFilter.addState(Keys::RecvClkBias{ satSys });
        _kalmanFilter.addState(Keys::RecvClkDrift{ satSys });
    }
    _receiverClockModel.initialize(_kalmanFilter.F, _kalmanFilter.G, _kalmanFilter.W);

    LOG_DATA("F = \n{}", _kalmanFilter.F);
    LOG_DATA("G = \n{}", _kalmanFilter.G);
    LOG_DATA("W = \n{}", _kalmanFilter.W);
    _initialized = false;
}

void KalmanFilter::initialize(const KeyedVectorXd<States::StateKeyType>& states, const KeyedMatrixXd<States::StateKeyType, States::StateKeyType>& variance)
{
    LOG_DATA("x_KF(pre-init) = \n{}", _kalmanFilter.x.transposed());
    _kalmanFilter.x(states.rowKeys()) = states(all);
    _kalmanFilter.P(variance.rowKeys(), variance.colKeys()) = variance(all, all) * 100.0; // LSQ variance is very small. So make bigger

    if (!states.hasAnyRows(VelKey)) // We always estimate velocity in the KF, but LSQ could not, so set a default value
    {
        _kalmanFilter.P(VelKey, VelKey).diagonal() << Eigen::Vector3d::Ones() * _initCovarianceVelocity;
    }
    // We always estimate receiver clock drift in the KF, but LSQ could not, so set a default value
    for (const auto& key : _kalmanFilter.P.rowKeys())
    {
        if (std::holds_alternative<Keys::RecvClkDrift>(key) && !states.hasRow(key))
        {
            _kalmanFilter.P(key, key) = _initCovarianceClockDrift;
        }
    }

    _initialized = true;
}

void KalmanFilter::deinitialize()
{
    _initialized = false;
}

void KalmanFilter::predict(const double& dt, const Eigen::Vector3d& lla_pos, [[maybe_unused]] const std::string& nameId)
{
    // Update the State transition matrix (𝚽) and the Process noise covariance matrix (𝐐)

    LOG_DATA("{}: F =\n{}", nameId, _kalmanFilter.F);
    LOG_DATA("{}: G =\n{}", nameId, _kalmanFilter.G);
    LOG_DATA("{}: W =\n{}", nameId, _kalmanFilter.W);

    _motionModel.updatePhiAndQ(_kalmanFilter.Phi,
                               _kalmanFilter.Q,
                               _kalmanFilter.G,
                               _kalmanFilter.F,
                               _kalmanFilter.W,
                               dt,
                               lla_pos(0),
                               lla_pos(1),
                               _systemModelCalcAlgorithm);

    _receiverClockModel.updatePhiAndQ(_kalmanFilter.Phi,
                                      _kalmanFilter.Q,
                                      _kalmanFilter.F,
                                      _kalmanFilter.G,
                                      _kalmanFilter.W,
                                      dt,
                                      _systemModelCalcAlgorithm);

    _interFrequencyBiasModel.updatePhiAndQ(_kalmanFilter.Phi,
                                           _kalmanFilter.Q,
                                           dt);

    LOG_DATA("{}: Phi =\n{}", nameId, _kalmanFilter.Phi);
    LOG_DATA("{}: Q =\n{}", nameId, _kalmanFilter.Q);

    LOG_DATA("{}: P (a posteriori) =\n{}", nameId, _kalmanFilter.P);
    LOG_DATA("{}: x (a posteriori) =\n{}", nameId, _kalmanFilter.x.transposed());
    _kalmanFilter.predict();
    LOG_DATA("{}: x (a priori    ) =\n{}", nameId, _kalmanFilter.x.transposed());
    LOG_DATA("{}: P (a priori    ) =\n{}", nameId, _kalmanFilter.P);
}

void KalmanFilter::update(const std::vector<Meas::MeasKeyTypes>& measKeys,
                          const KeyedMatrixXd<Meas::MeasKeyTypes, States::StateKeyType>& H,
                          const KeyedMatrixXd<Meas::MeasKeyTypes, Meas::MeasKeyTypes>& R,
                          const KeyedVectorXd<Meas::MeasKeyTypes>& dz,
                          [[maybe_unused]] const std::string& nameId)
{
    LOG_DATA("{}: called", nameId);

    _kalmanFilter.setMeasurements(measKeys);

    _kalmanFilter.H = H;
    _kalmanFilter.R = R;
    _kalmanFilter.z = dz;

    _kalmanFilter.correctWithMeasurementInnovation();
}

void KalmanFilter::addInterFrequencyBias(const Frequency& freq)
{
    auto bias = Keys::InterFreqBias{ freq };
    _kalmanFilter.addState(bias);
    _kalmanFilter.P(bias, bias) = _kalmanFilter.P(Keys::RecvClkBias{ freq.getSatSys() },
                                                  Keys::RecvClkBias{ freq.getSatSys() });
    _interFrequencyBiasModel.initialize(bias,
                                        _kalmanFilter.F,
                                        _kalmanFilter.G,
                                        _kalmanFilter.W);
}

bool KalmanFilter::ShowGuiWidgets(const char* id, bool useDoppler, bool multiConstellation, bool estimateInterFrequencyBiases, float itemWidth, float unitWidth)
{
    bool changed = false;

    itemWidth -= ImGui::GetStyle().IndentSpacing;
    float configWidth = itemWidth + unitWidth;

    ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("System/Process noise##{}", id).c_str()))
    {
        changed |= SystemModelGui(_systemModelCalcAlgorithm, itemWidth, id);

        changed |= _motionModel.ShowGui(configWidth, unitWidth, id);
        changed |= _receiverClockModel.ShowGui(configWidth, unitWidth, id);
        if (estimateInterFrequencyBiases)
        {
            changed |= _interFrequencyBiasModel.ShowGui(configWidth, unitWidth, id);
        }

        ImGui::TreePop();
    }

    if (!useDoppler)
    {
        ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
        if (ImGui::TreeNode(fmt::format("Initial Error Covariance Matrix P##{}", id).c_str()))
        {
            if (gui::widgets::InputDoubleWithUnit(fmt::format("{} of the velocity uncertainty##{}",
                                                              _gui_initCovarianceVelocityUnit == InitCovarianceVelocityUnits::m_s ? "Standard deviation" : "Variance", id)
                                                      .c_str(),
                                                  configWidth, unitWidth, &_gui_initCovarianceVelocity, reinterpret_cast<int*>(&_gui_initCovarianceVelocityUnit), "m/s\0m^2/s^2\0\0",
                                                  0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: _gui_initCovarianceVelocity changed to {}", id, _gui_initCovarianceVelocity);
                LOG_DEBUG("{}: _gui_initCovarianceVelocityUnit changed to {}", id, fmt::underlying(_gui_initCovarianceVelocityUnit));
                changed = true;
            }
            if (gui::widgets::InputDoubleWithUnit(fmt::format("{} of the clock drift uncertainty##{}",
                                                              (_gui_initCovarianceClockDriftUnit == InitCovarianceClockDriftUnits::m_s
                                                               || _gui_initCovarianceClockDriftUnit == InitCovarianceClockDriftUnits::s_s)
                                                                  ? "Standard deviation"
                                                                  : "Variance",
                                                              id)
                                                      .c_str(),
                                                  configWidth, unitWidth, &_gui_initCovarianceClockDrift, reinterpret_cast<int*>(&_gui_initCovarianceClockDriftUnit), "m/s\0s/s\0m^2/s^2\0s^2/s^2\0\0",
                                                  0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: _gui_initCovarianceClockDrift changed to {}", id, _gui_initCovarianceClockDrift);
                LOG_DEBUG("{}: _gui_initCovarianceClockDriftUnit changed to {}", id, fmt::underlying(_gui_initCovarianceClockDriftUnit));
                changed = true;
            }
            if (multiConstellation
                && gui::widgets::InputDoubleWithUnit(fmt::format("{} of the inter system clock drift uncertainty##{}",
                                                                 (_gui_initCovarianceInterSysClockDriftUnit == InitCovarianceClockDriftUnits::m_s
                                                                  || _gui_initCovarianceInterSysClockDriftUnit == InitCovarianceClockDriftUnits::s_s)
                                                                     ? "Standard deviation"
                                                                     : "Variance",
                                                                 id)
                                                         .c_str(),
                                                     configWidth, unitWidth, &_gui_initCovarianceInterSysClockDrift, reinterpret_cast<int*>(&_gui_initCovarianceInterSysClockDriftUnit), "m/s\0s/s\0m^2/s^2\0s^2/s^2\0\0",
                                                     0.0, 0.0, "%.2e", ImGuiInputTextFlags_CharsScientific))
            {
                LOG_DEBUG("{}: _gui_initCovarianceInterSysClockDrift changed to {}", id, _gui_initCovarianceInterSysClockDrift);
                LOG_DEBUG("{}: _gui_initCovarianceInterSysClockDriftUnit changed to {}", id, fmt::underlying(_gui_initCovarianceInterSysClockDriftUnit));
                changed = true;
            }

            ImGui::TreePop();
        }
    }

    ImGui::SetNextItemOpen(false, ImGuiCond_FirstUseEver);
    if (ImGui::TreeNode(fmt::format("Kalman Filter matrices##{}", id).c_str()))
    {
        _kalmanFilter.showKalmanFilterMatrixViews(id);
        ImGui::TreePop();
    }

    return changed;
}

void KalmanFilter::setClockBiasErrorCovariance(double clkPhaseDrift)
{
    for (const auto& state : _kalmanFilter.P.rowKeys())
    {
        if (const auto* bias = std::get_if<Keys::RecvClkBias>(&state))
        {
            _kalmanFilter.P(*bias, *bias) = clkPhaseDrift;
        }
    }
}

const std::vector<SPP::States::StateKeyType>& KalmanFilter::getStateKeys() const
{
    return _kalmanFilter.x.rowKeys();
}

const KeyedVectorX<double, States::StateKeyType>& KalmanFilter::getState() const
{
    return _kalmanFilter.x;
}

const KeyedMatrixXd<States::StateKeyType, States::StateKeyType>& KalmanFilter::getErrorCovarianceMatrix() const
{
    return _kalmanFilter.P;
}

void to_json(json& j, const KalmanFilter& data)
{
    j = {
        { "systemModelCalcAlgorithm", data._systemModelCalcAlgorithm },
        { "motionModel", data._motionModel },
        { "receiverClockModel", data._receiverClockModel },
        { "interFrequencyBiasModel", data._interFrequencyBiasModel },
    };
} // namespace NAV::SPP

void from_json(const json& j, KalmanFilter& data)
{
    if (j.contains("systemModelCalcAlgorithm")) { j.at("systemModelCalcAlgorithm").get_to(data._systemModelCalcAlgorithm); }
    if (j.contains("motionModel")) { j.at("motionModel").get_to(data._motionModel); }
    if (j.contains("receiverClockModel")) { j.at("receiverClockModel").get_to(data._receiverClockModel); }
    if (j.contains("interFrequencyBiasModel")) { j.at("interFrequencyBiasModel").get_to(data._interFrequencyBiasModel); }
}

} // namespace NAV::SPP
