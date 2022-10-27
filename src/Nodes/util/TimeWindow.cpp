// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "TimeWindow.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

#include <imgui_internal.h>

// ---------------------------------------------------------- Private variabels ------------------------------------------------------------

namespace NAV
{
/// List of supported data identifiers
const std::vector<std::string> supportedDataIdentifier{ ImuObs::type(), PosVelAtt::type() };
} // namespace NAV

// ---------------------------------------------------------- Member functions -------------------------------------------------------------

NAV::TimeWindow::TimeWindow() : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 812, 332 }; // FIXME: default window size

    nm::CreateInputPin(this, "Input", Pin::Type::Flow, supportedDataIdentifier, &TimeWindow::receiveObs);

    nm::CreateOutputPin(this, "Output", Pin::Type::Flow, supportedDataIdentifier);
}

NAV::TimeWindow::~TimeWindow()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::TimeWindow::typeStatic()
{
    return "TimeWindow";
}

std::string NAV::TimeWindow::type() const
{
    return typeStatic();
}

std::string NAV::TimeWindow::category()
{
    return "DataProcessor";
}

void NAV::TimeWindow::guiConfig()
{}

json NAV::TimeWindow::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    return j;
}

void NAV::TimeWindow::restore(json const& j)
{
    if (j) // FIXME: dummy
    {}
    LOG_TRACE("{}: called", nameId());
}

bool NAV::TimeWindow::resetNode()
{
    LOG_TRACE("{}: called", nameId());

    return true;
}

void NAV::TimeWindow::afterCreateLink(OutputPin& startPin, InputPin& endPin)
{
    if (startPin.isPinLinked() || endPin.isPinLinked()) // FIXME: dummy
    {
    }
}

void NAV::TimeWindow::afterDeleteLink(OutputPin& startPin, InputPin& endPin)
{
    LOG_TRACE("{}: called for {} ==> {}", nameId(), size_t(startPin.id), size_t(endPin.id));
}

void NAV::TimeWindow::receiveObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    // Select the correct data type and make a copy of the node data to modify
    if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.front() == ImuObs::type())
    {
        receiveImuObs(std::make_shared<ImuObs>(*std::static_pointer_cast<const ImuObs>(queue.extract_front())));
    }
    else if (outputPins.at(OUTPUT_PORT_INDEX_FLOW).dataIdentifier.front() == PosVelAtt::type())
    {
        receivePosVelAtt(std::make_shared<PosVelAtt>(*std::static_pointer_cast<const PosVelAtt>(queue.extract_front())));
    }
}

void NAV::TimeWindow::receiveImuObs(const std::shared_ptr<ImuObs>& imuObs)
{
    if (imuObs) // FIXME: dummy
    {}
}

void NAV::TimeWindow::receivePosVelAtt(const std::shared_ptr<PosVelAtt>& posVelAtt)
{
    if (posVelAtt) // FIXME: dummy
    {}
}