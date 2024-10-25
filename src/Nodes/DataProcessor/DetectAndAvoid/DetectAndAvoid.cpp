// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DetectAndAvoid.hpp"

#include <imgui.h>
#include <imgui_internal.h>

#include "NodeData/State/PosVelAtt.hpp"
#include "internal/FlowManager.hpp"
#include "internal/Node/Pin.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/InputWithUnit.hpp"
#include "internal/gui/NodeEditorApplication.hpp"
#include "NodeRegistry.hpp"
#include "util/Logger.hpp"

NAV::DetectAndAvoid::DetectAndAvoid()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 822, 936 }; // TODO: Adapt this, once the node is finished

    nm::CreateInputPin(this, "PosVelAttIn", Pin::Type::Flow, { NAV::PosVelAtt::type() }, &DetectAndAvoid::recvPosVelAtt);
    nm::CreateOutputPin(this, "PosVelAttOut", Pin::Type::Flow, { PosVelAtt::type() });
}

NAV::DetectAndAvoid::~DetectAndAvoid()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::DetectAndAvoid::typeStatic()
{
    return "DetectAndAvoid";
}

std::string NAV::DetectAndAvoid::type() const
{
    return typeStatic();
}

std::string NAV::DetectAndAvoid::category()
{
    return "Data Processor";
}

void NAV::DetectAndAvoid::guiConfig()
{
    [[maybe_unused]] float itemWidth = 470 * gui::NodeEditorApplication::windowFontRatio(); // TODO: Adapt this, once the node is finished
    [[maybe_unused]] float unitWidth = 180 * gui::NodeEditorApplication::windowFontRatio(); // TODO: Adapt this, once the node is finished
}

json NAV::DetectAndAvoid::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    return j;
}

void NAV::DetectAndAvoid::restore(json const& /* j */) // TODO: Adapt this, once the node is finished
{
    LOG_TRACE("{}: called", nameId());
}

bool NAV::DetectAndAvoid::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return true;
}

void NAV::DetectAndAvoid::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::DetectAndAvoid::recvPosVelAtt(InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto posVelAtt = std::static_pointer_cast<const PosVelAtt>(queue.extract_front());

    invokeCallbackWithPosVelAtt(*posVelAtt);
}

void NAV::DetectAndAvoid::invokeCallbackWithPosVelAtt(const PosVelAtt& posVelAtt)
{
    auto posVelAtt_solution = std::make_shared<PosVelAtt>();
    posVelAtt_solution->insTime = posVelAtt.insTime;
    invokeCallbacks(OUTPUT_PORT_INDEX_SOLUTION, posVelAtt_solution);
}