// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GroupBox.hpp"

#include "util/Logger.hpp"

NAV::GroupBox::GroupBox()
    : Node(typeStatic())
{
    LOG_TRACE("called");

    kind = Node::Kind::GroupBox;
    _hasConfig = false;
    _size = ImVec2(400, 300);
}

NAV::GroupBox::~GroupBox()
{
    LOG_TRACE("called");
}

std::string NAV::GroupBox::typeStatic()
{
    return "GroupBox";
}

std::string NAV::GroupBox::type() const
{
    return typeStatic();
}

std::string NAV::GroupBox::category()
{
    return "Utility";
}

[[nodiscard]] json NAV::GroupBox::save() const { return {}; }

void NAV::GroupBox::restore(json const& /* j */) {}