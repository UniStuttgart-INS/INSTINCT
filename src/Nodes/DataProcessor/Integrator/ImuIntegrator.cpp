// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImuIntegrator.hpp"

#include "util/Logger.hpp"

#include "NodeData/IMU/ImuObs.hpp"
#include "NodeData/IMU/ImuObsWDelta.hpp"
#include "NodeData/State/PosVelAtt.hpp"

#include "Navigation/Constants.hpp"
#include "Navigation/Math/Math.hpp"

#include "internal/gui/widgets/HelpMarker.hpp"
#include <imgui_internal.h>

#include "util/Eigen.hpp"

#include "NodeRegistry.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include <algorithm>

NAV::ImuIntegrator::ImuIntegrator()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 422, 146 };

    nm::CreateInputPin(this, "ImuObs", Pin::Type::Flow, { NAV::ImuObs::type(), NAV::ImuObsWDelta::type() }, &ImuIntegrator::recvObservation,
                       [](const Node* node, const InputPin& inputPin) {
                           const auto* imuIntegrator = static_cast<const ImuIntegrator*>(node); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
                           return !inputPin.queue.empty() && imuIntegrator->_inertialIntegrator.hasInitialPosition();
                       });
    nm::CreateInputPin(this, "PosVelAttInit", Pin::Type::Flow, { NAV::PosVelAtt::type() }, &ImuIntegrator::recvPosVelAttInit, nullptr, 1);

    nm::CreateOutputPin(this, "PosVelAtt", Pin::Type::Flow, { NAV::PosVelAtt::type() });
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
    if (InertialIntegratorGui(std::to_string(size_t(id)).c_str(), _inertialIntegrator))
    {
        flow::ApplyChanges();
    }
    if (inputPins.at(INPUT_PORT_INDEX_IMU_OBS).isPinLinked()
        && NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(inputPins.at(INPUT_PORT_INDEX_IMU_OBS).link.getConnectedPin()->dataIdentifier, { ImuObsWDelta::type() }))
    {
        ImGui::Separator();
        if (ImGui::Checkbox(fmt::format("Prefer raw measurements over delta##{}", size_t(id)).c_str(), &_preferAccelerationOverDeltaMeasurements))
        {
            flow::ApplyChanges();
        }
    }
}

[[nodiscard]] json NAV::ImuIntegrator::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["inertialIntegrator"] = _inertialIntegrator;
    j["preferAccelerationOverDeltaMeasurements"] = _preferAccelerationOverDeltaMeasurements;

    return j;
}

void NAV::ImuIntegrator::restore(json const& j)
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
}

bool NAV::ImuIntegrator::initialize()
{
    LOG_TRACE("{}: called", nameId());

    _inertialIntegrator.reset();

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

    if (!_preferAccelerationOverDeltaMeasurements
        && NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(inputPins.at(INPUT_PORT_INDEX_IMU_OBS).link.getConnectedPin()->dataIdentifier, { ImuObsWDelta::type() }))
    {
        auto obs = std::static_pointer_cast<const ImuObsWDelta>(nodeData);
        LOG_DATA("{}: recvImuObsWDelta at time [{}]", nameId(), obs->insTime.toYMDHMS());

        integratedPosVelAtt = _inertialIntegrator.calcInertialSolutionDelta(obs->insTime, obs->dtime, obs->dvel, obs->dtheta, obs->p_acceleration, obs->p_angularRate, obs->imuPos);
    }
    else
    {
        auto obs = std::static_pointer_cast<const ImuObs>(nodeData);
        LOG_DATA("{}: recvImuObs at time [{}]", nameId(), obs->insTime.toYMDHMS());

        integratedPosVelAtt = _inertialIntegrator.calcInertialSolution(obs->insTime, obs->p_acceleration, obs->p_angularRate, obs->imuPos);
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

    LOG_DATA("{}: recvPosVelAttInit at time [{}]", nameId(), posVelAtt->insTime.toYMDHMS());

    if (!_inertialIntegrator.hasInitialPosition())
    {
        _inertialIntegrator.setInitialState(*posVelAtt);
        LOG_DATA("{}:   e_position   = {}", nameId(), posVelAtt->e_position().transpose());
        LOG_DATA("{}:   e_velocity   = {}", nameId(), posVelAtt->e_velocity().transpose());
        LOG_DATA("{}:   rollPitchYaw = {}", nameId(), rad2deg(posVelAtt->rollPitchYaw()).transpose());

        invokeCallbacks(OUTPUT_PORT_INDEX_INERTIAL_NAV_SOL, posVelAtt);
    }
}