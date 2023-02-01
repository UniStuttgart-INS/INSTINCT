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
    nm::CreateInputPin(this, "GNSSobs", Pin::Type::Flow, { NAV::GnssObs::type() }, &TightlyCoupledKF::recvGnssObs,
                       [](const Node* node, const InputPin& inputPin) {
                           const auto* tckf = static_cast<const TightlyCoupledKF*>(node); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
                           return !inputPin.queue.empty() && tckf->_lastPredictRequestedTime < inputPin.queue.front()->insTime;
                       });
    inputPins.back().dropQueueIfNotFirable = false;
    // nm::CreateOutputPin(this, "Errors", Pin::Type::Flow, { NAV::LcKfInsGnssErrors::type() }); // TODO: Enable, once output is provided
    nm::CreateOutputPin(this, "Sync", Pin::Type::Flow, { NAV::NodeData::type() });
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

    double tau_i = !_lastPredictTime.empty()
                       ? static_cast<double>((inertialNavSol->insTime - _lastPredictTime).count())
                       : 0.0;

    if (tau_i > 0)
    {
        _lastPredictTime = _latestInertialNavSol->insTime + std::chrono::duration<double>(tau_i);
        tightlyCoupledPrediction(_latestInertialNavSol, tau_i);
    }
    else
    {
        _lastPredictTime = inertialNavSol->insTime;
    }
    _latestInertialNavSol = inertialNavSol;

    if (!inputPins[INPUT_PORT_INDEX_GNSS].queue.empty() && inputPins[INPUT_PORT_INDEX_GNSS].queue.front()->insTime == _lastPredictTime)
    {
        tightlyCoupledUpdate(std::static_pointer_cast<const GnssObs>(inputPins[INPUT_PORT_INDEX_GNSS].queue.extract_front()));
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

void NAV::TightlyCoupledKF::recvGnssObs(InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto gnssObservation = queue.front();
    LOG_DATA("{}: recvGNSSNavigationSolution at time [{} - {}]", nameId(), gnssObservation->insTime.toYMDHMS(), gnssObservation->insTime.toGPSweekTow());

    auto nodeData = std::make_shared<NodeData>();
    nodeData->insTime = gnssObservation->insTime;
    _lastPredictRequestedTime = gnssObservation->insTime;

    invokeCallbacks(OUTPUT_PORT_INDEX_SYNC, nodeData); // Prediction consists out of ImuIntegration and prediction (gets triggered from it)
}

// ###########################################################################################################
//                                               Kalman Filter
// ###########################################################################################################

void NAV::TightlyCoupledKF::tightlyCoupledPrediction(const std::shared_ptr<const InertialNavSol>& inertialNavSol, double tau_i)
{
    auto dt = fmt::format("{:0.5f}", tau_i);
    dt.erase(std::find_if(dt.rbegin(), dt.rend(), [](char ch) { return ch != '0'; }).base(), dt.end());

    InsTime predictTime = inertialNavSol->insTime + std::chrono::duration<double>(tau_i);
    LOG_DATA("{}: Predicting (dt = {}s) from [{} - {}] to [{} - {}]", nameId(), dt,
             inertialNavSol->insTime.toYMDHMS(), inertialNavSol->insTime.toGPSweekTow(), predictTime.toYMDHMS(), predictTime.toGPSweekTow());

    // ------------------------------------------- GUI Parameters ----------------------------------------------

    // ---------------------------------------------- Prediction -----------------------------------------------
}

void NAV::TightlyCoupledKF::tightlyCoupledUpdate(const std::shared_ptr<const GnssObs>& gnssObservation)
{
    LOG_DATA("{}: Updating to time {} - {} (lastInertial at {} - {})", nameId(), gnssObservation->insTime.toYMDHMS(), gnssObservation->insTime.toGPSweekTow(),
             _latestInertialNavSol->insTime.toYMDHMS(), _latestInertialNavSol->insTime.toGPSweekTow());

    [[maybe_unused]] auto bla = gnssObservation;

    // -------------------------------------------- GUI Parameters -----------------------------------------------

    // ---------------------------------------------- Update -----------------------------------------------------
}

// ###########################################################################################################
//                                                Prediction
// ###########################################################################################################

// ###########################################################################################################
//                                                  Update
// ###########################################################################################################
