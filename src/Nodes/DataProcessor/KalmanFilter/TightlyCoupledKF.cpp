// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "TightlyCoupledKF.hpp"

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

NAV::TightlyCoupledKF::TightlyCoupledKF()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 800, 700 }; // TODO: Adapt, once config options are implemented

    nm::CreateInputPin(this, "InertialNavSol", Pin::Type::Flow, { NAV::InertialNavSol::type() }, &TightlyCoupledKF::recvInertialNavigationSolution, nullptr, 1);
    inputPins.back().neededForTemporalQueueCheck = false;
    // TODO: Create Input and Output pins
}

NAV::TightlyCoupledKF::~TightlyCoupledKF()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::TightlyCoupledKF::typeStatic()
{
    return "TightlyCoupledKF";
}

std::string NAV::TightlyCoupledKF::type() const
{
    return typeStatic();
}

std::string NAV::TightlyCoupledKF::category()
{
    return "Data Processor";
}

void NAV::TightlyCoupledKF::guiConfig()
{
    // float configWidth = 380.0F * gui::NodeEditorApplication::windowFontRatio();
    // float unitWidth = 150.0F * gui::NodeEditorApplication::windowFontRatio();

    // TODO: Implement config options
}

[[nodiscard]] json NAV::TightlyCoupledKF::save() const
{
    LOG_TRACE("{}: called", nameId());

    [[maybe_unused]] json j; // TODO: Remove '[[maybe_unused]]', once there are variables to save

    return j;
}

void NAV::TightlyCoupledKF::restore([[maybe_unused]] json const& j) // TODO: Remove '[[maybe_unused]]', once there are variables to save
{
    LOG_TRACE("{}: called", nameId());
}

bool NAV::TightlyCoupledKF::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return true;
}

void NAV::TightlyCoupledKF::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::TightlyCoupledKF::recvInertialNavigationSolution(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */) // NOLINT(readability-convert-member-functions-to-static)
{
    auto inertialNavSol = std::static_pointer_cast<const InertialNavSol>(queue.extract_front());
    LOG_DATA("{}: recvInertialNavigationSolution at time [{} - {}]", nameId(), inertialNavSol->insTime.toYMDHMS(), inertialNavSol->insTime.toGPSweekTow());
}