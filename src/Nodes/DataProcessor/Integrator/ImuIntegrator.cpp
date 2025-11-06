// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImuIntegrator.hpp"

#include <memory>
#include <algorithm>
#include <cstddef>

#include <imgui.h>
#include <imgui_internal.h>

#include "Navigation/INS/InertialPreIntegrator.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"

#include "NodeRegistry.hpp"
#include <fmt/format.h>
#include "internal/FlowManager.hpp"

#include "Navigation/Constants.hpp"
#include "Navigation/Math/Math.hpp"

#include "util/Logger.hpp"
#include "util/Eigen.hpp"

NAV::ImuIntegrator::ImuIntegrator()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 422, 146 };

    CreateInputPin("ImuObs", Pin::Type::Flow, { NAV::ImuObs::type(), NAV::ImuObsWDelta::type() }, &ImuIntegrator::recvObservation,
                   [](const Node* node, const InputPin& inputPin) {
                       const auto* imuIntegrator = static_cast<const ImuIntegrator*>(node); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
                       return !inputPin.queue.empty() && imuIntegrator->_inertialIntegrator.hasInitialPosition();
                   });
    CreateInputPin("PosVelAttInit", Pin::Type::Flow, { NAV::PosVelAtt::type() }, &ImuIntegrator::recvPosVelAttInit, nullptr, 1);

    CreateOutputPin("PosVelAtt", Pin::Type::Flow, { NAV::PosVelAtt::type() });
}

NAV::ImuIntegrator::~ImuIntegrator()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::ImuIntegrator::typeStatic()
{
    return "ImuIntegrator";
}

std::string NAV::ImuIntegrator::type() const
{
    return typeStatic();
}

std::string NAV::ImuIntegrator::category()
{
    return "Data Processor";
}

void NAV::ImuIntegrator::guiConfig()
{
    if (ImGui::Checkbox(fmt::format("IMU Preintegration##{}", size_t(id)).c_str(), &_imuPreintegration))
    {
        flow::ApplyChanges();
    }

    if (_imuPreintegration)
    {
        if (ImGui::Checkbox(fmt::format("Reset every epoch##{}", size_t(id)).c_str(), &_resetPreintegratorEveryEpoch))
        {
            flow::ApplyChanges();
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("Resetting the preintegrator every epoch makes it behave like a normal integrator");

        if (InertialPreIntegratorGui(std::to_string(size_t(id)).c_str(), _inertialPreintegrator))
        {
            flow::ApplyChanges();
        }
    }
    else
    {
        if (InertialIntegratorGui(std::to_string(size_t(id)).c_str(), _inertialIntegrator, _preferAccelerationOverDeltaMeasurements))
        {
            flow::ApplyChanges();
        }
    }
}

[[nodiscard]] json NAV::ImuIntegrator::save() const
{
    LOG_TRACE("{}: called", nameId());

    return {
        { "imuPreintegration", _imuPreintegration },
        { "resetPreintegratorEveryEpoch", _resetPreintegratorEveryEpoch },
        { "inertialIntegrator", _inertialIntegrator },
        { "inertialPreintegrator", _inertialPreintegrator },
        { "preferAccelerationOverDeltaMeasurements", _preferAccelerationOverDeltaMeasurements },
    };
}

void NAV::ImuIntegrator::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("imuPreintegration")) { j.at("imuPreintegration").get_to(_imuPreintegration); }
    if (j.contains("resetPreintegratorEveryEpoch")) { j.at("resetPreintegratorEveryEpoch").get_to(_resetPreintegratorEveryEpoch); }
    if (j.contains("inertialIntegrator")) { j.at("inertialIntegrator").get_to(_inertialIntegrator); }
    if (j.contains("inertialPreintegrator")) { j.at("inertialPreintegrator").get_to(_inertialPreintegrator); }
    if (j.contains("preferAccelerationOverDeltaMeasurements")) { j.at("preferAccelerationOverDeltaMeasurements").get_to(_preferAccelerationOverDeltaMeasurements); }
}

bool NAV::ImuIntegrator::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _inertialIntegrator.reset();
    _inertialPreintegrator.reset();
    _lastImuObs.reset();
    _lastPosVelAtt.reset();

    LOG_DEBUG("ImuIntegrator initialized");

    return true;
}

void NAV::ImuIntegrator::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::ImuIntegrator::recvObservation(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto nodeData = queue.extract_front();
    if (nodeData->insTime.empty())
    {
        LOG_ERROR("{}: Can't set new imuObs__t0 because the observation has no time tag (insTime)", nameId());
        return;
    }

    std::shared_ptr<NAV::PosVelAtt> integratedPosVelAtt = nullptr;

    if (_imuPreintegration)
    {
        auto obs = std::static_pointer_cast<const ImuObs>(nodeData);
        LOG_DATA("{}: recvImuObs at time [{}] (preintegration)", nameId(), obs->insTime.toYMDHMS(GPST));
        // LOG_DATA("{}:   p_accel = {}", nameId(), obs->p_acceleration.transpose());
        // LOG_DATA("{}:   p_gyro  = {}", nameId(), obs->p_angularRate.transpose());

        if (!_lastImuObs)
        {
            _lastImuObs = obs;
            LOG_DATA("{}:   Skipping as first epoch", nameId());
            return;
        }

        auto dt = static_cast<double>((obs->insTime - _lastImuObs->insTime).count());
        // LOG_DATA("{}:   dt(imu) = {}", nameId(), dt);
        if (dt <= 1e-9)
        {
            LOG_DATA("{}:   Skipping as dt is negative or zero", nameId());
            return;
        }
        InertialPreIntegrator::Measurement meas{
            .meas = InertialPreIntegrator::ImuMeasurement{ .dt = dt,
                                                           .p_acceleration = _lastImuObs->p_acceleration,
                                                           .p_angularRate = _lastImuObs->p_angularRate },
            .imu = InertialPreIntegrator::ImuState<double>{}
        };
        InertialPreIntegrator::PVAState<double> pvaOld{
            .position = _lastPosVelAtt->e_position(),
            .velocity = _lastPosVelAtt->e_velocity(),
            .attitude = _lastPosVelAtt->e_Quat_b()
        };

        _inertialPreintegrator.addInertialMeasurement(meas, obs->imuPos, nameId());

        auto pvaNew = _inertialPreintegrator.calcIntegratedState(pvaOld, InertialPreIntegrator::ImuState<double>{}, obs->imuPos, nameId());

        integratedPosVelAtt = std::make_shared<PosVelAtt>();
        integratedPosVelAtt->insTime = obs->insTime;
        integratedPosVelAtt->setPosVelAtt_e(pvaNew.position, pvaNew.velocity, pvaNew.attitude);

        _lastImuObs = obs;

        if (_resetPreintegratorEveryEpoch)
        {
            _lastPosVelAtt = integratedPosVelAtt;
            _inertialPreintegrator.reset();
        }
    }
    else
    {
        if (!_preferAccelerationOverDeltaMeasurements
            && NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(inputPins.at(INPUT_PORT_INDEX_IMU_OBS).link.getConnectedPin()->dataIdentifier, { ImuObsWDelta::type() }))
        {
            auto obs = std::static_pointer_cast<const ImuObsWDelta>(nodeData);
            LOG_DATA("{}: recvImuObsWDelta at time [{}]", nameId(), obs->insTime.toYMDHMS(GPST));

            integratedPosVelAtt = _inertialIntegrator.calcInertialSolutionDelta(obs->insTime, obs->dtime, obs->dvel, obs->dtheta, obs->imuPos, nameId().c_str());
        }
        else
        {
            auto obs = std::static_pointer_cast<const ImuObs>(nodeData);
            LOG_DATA("{}: recvImuObs at time [{}]", nameId(), obs->insTime.toYMDHMS(GPST));

            integratedPosVelAtt = _inertialIntegrator.calcInertialSolution(obs->insTime, obs->p_acceleration, obs->p_angularRate, obs->imuPos, nameId().c_str());
        }
    }

    if (integratedPosVelAtt)
    {
        LOG_DATA("{}:   e_position   = {}", nameId(), integratedPosVelAtt->e_position().transpose());
        LOG_DATA("{}:   e_velocity   = {}", nameId(), integratedPosVelAtt->e_velocity().transpose());
        LOG_DATA("{}:   rollPitchYaw = {}", nameId(), rad2deg(integratedPosVelAtt->rollPitchYaw()).transpose());
        invokeCallbacks(OUTPUT_PORT_INDEX_INERTIAL_NAV_SOL, integratedPosVelAtt);
    }
}

void NAV::ImuIntegrator::recvPosVelAttInit(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto posVelAtt = std::static_pointer_cast<const PosVelAtt>(queue.extract_front());
    inputPins[INPUT_PORT_INDEX_POS_VEL_ATT_INIT].queueBlocked = true;
    inputPins[INPUT_PORT_INDEX_POS_VEL_ATT_INIT].queue.clear();

    LOG_DATA("{}: recvPosVelAttInit at time [{}]", nameId(), posVelAtt->insTime.toYMDHMS(GPST));

    _lastPosVelAtt = posVelAtt;
    _inertialIntegrator.setInitialState(*posVelAtt, nameId().c_str());
    LOG_DATA("{}:   e_position   = {}", nameId(), posVelAtt->e_position().transpose());
    LOG_DATA("{}:   e_velocity   = {}", nameId(), posVelAtt->e_velocity().transpose());
    LOG_DATA("{}:   rollPitchYaw = {}", nameId(), rad2deg(posVelAtt->rollPitchYaw()).transpose());

    invokeCallbacks(OUTPUT_PORT_INDEX_INERTIAL_NAV_SOL, posVelAtt);
}