// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Terminator.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

// ---------------------------------------------------------- Member functions -------------------------------------------------------------

NAV::Terminator::Terminator() : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = false;
    kind = Kind::Simple;

    nm::CreateInputPin(this, "", Pin::Type::Flow, { NodeData::type() }, &Terminator::receiveObs);
}

NAV::Terminator::~Terminator()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::Terminator::typeStatic()
{
    return "Terminator";
}

std::string NAV::Terminator::type() const
{
    return typeStatic();
}

std::string NAV::Terminator::category()
{
    return "Utility";
}

void NAV::Terminator::receiveObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */) // NOLINT(readability-convert-member-functions-to-static)
{
    queue.pop_front();
}