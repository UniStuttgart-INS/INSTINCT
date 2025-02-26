// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PressureToHeight.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/Baro/BarometricPressureObs.hpp"
#include "NodeData/Baro/BarometricHeight.hpp"

#include "Navigation/Gravity/Gravity.hpp"

#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/EnumCombo.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

NAV::PressureToHeight::PressureToHeight()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 710, 220 };

    nm::CreateInputPin(this, "BarometricPressureObs", Pin::Type::Flow, { NAV::BarometricPressureObs::type() }, &PressureToHeight::receiveObs);

    nm::CreateOutputPin(this, "BarometricHeight", Pin::Type::Flow, { NAV::BarometricHeight::type() });
}

NAV::PressureToHeight::~PressureToHeight()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::PressureToHeight::typeStatic()
{
    return "PressureToHeight";
}

std::string NAV::PressureToHeight::type() const
{
    return typeStatic();
}

std::string NAV::PressureToHeight::category()
{
    return "Data Processor";
}

void NAV::PressureToHeight::guiConfig()
{
    float columnWidth = 140.0F * gui::NodeEditorApplication::windowFontRatio();

    if (ImGui::InputDouble(fmt::format("Sea-level pressure##{}", size_t(id)).c_str(), &_pressure0, 0.0, 0.0, "%.3f hPa"))
    {
        LOG_DATA("{}: pressure0 changed to {}", nameId(), _pressure0);
        flow::ApplyChanges();
    }
    if (ImGui::InputDouble(fmt::format("Sea-level temperature##{}", size_t(id)).c_str(), &_temp0, 0.0, 0.0, "%.3f K"))
    {
        LOG_DATA("{}: temp0 changed to {}", nameId(), _temp0);
        flow::ApplyChanges();
    }
    if (ImGui::InputDouble(fmt::format("Temperature lapse rate##{}", size_t(id)).c_str(), &_lapserate, 0.0, 0.0, "%.6f  K / m"))
    {
        LOG_DATA("{}: lapserate changed to {}", nameId(), _lapserate);
        flow::ApplyChanges();
    }
    if (ImGui::InputDouble(fmt::format("Geoid undulation##{}", size_t(id)).c_str(), &_geoidhgt, 0.0, 0.0, "%.3f  m"))
    {
        LOG_DATA("{}: geoidhgt changed to {}", nameId(), _geoidhgt);
        flow::ApplyChanges();
    }
    ImGui::SetNextItemWidth(columnWidth);
    if (gui::widgets::EnumCombo(fmt::format("Gravity input##{}", size_t(id)).c_str(), _gravityInput))
    {
        LOG_DATA("{}: gravity input changed to {}", nameId(), fmt::underlying(_gravityInput));
        flow::ApplyChanges();
    }
    ImGui::SameLine();
    gui::widgets::HelpMarker("The 'position' input calculates the gravity magnitude from EGM96.\n\n");
    if (_gravityInput == GravityInput::Manual)
    {
        if (ImGui::InputDouble(fmt::format("Gravity##{}", size_t(id)).c_str(), &_gravity, 0.0, 0.0, "%.5f  m / s²"))
        {
            LOG_DATA("{}: gravity changed to {}", nameId(), _gravity);
            flow::ApplyChanges();
        }
    }
    else // (_gravityInput == GravityInput::Position)
    {
        if (ImGui::InputDouble3(fmt::format("Position LLA [deg, deg, m]##{}", size_t(id)).c_str(), _initPos.data()))
        {
            flow::ApplyChanges();
        }
    }
}

bool NAV::PressureToHeight::initialize()
{
    LOG_TRACE("{}: called", nameId());
    if (_gravityInput == GravityInput::Position)
    {
        _gravity = n_calcGravitation_EGM96(_initPos).norm();
        LOG_DATA("{}: PressureToHeight - Local gravity magnitude: {} m/s² at initial position: {} [deg, deg, m]", nameId(), _gravity, _initPos.transpose());
    }
    _exponent = _gravity * InsConst::dMtr / InsConst::Rg / _lapserate;
    return true;
}

void NAV::PressureToHeight::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

[[nodiscard]] json NAV::PressureToHeight::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["temp0"] = _temp0;
    j["pressure0"] = _pressure0;
    j["lapserate"] = _lapserate;
    j["geoidhgt"] = _geoidhgt;
    j["gravity"] = _gravity;
    j["initPos"] = _initPos;
    j["gravityInput"] = _gravityInput;
    return j;
}

void NAV::PressureToHeight::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("temp0"))
    {
        j.at("temp0").get_to(_temp0);
    }
    if (j.contains("pressure0"))
    {
        j.at("pressure0").get_to(_pressure0);
    }
    if (j.contains("lapserate"))
    {
        j.at("lapserate").get_to(_lapserate);
    }
    if (j.contains("geoidhgt"))
    {
        j.at("geoidhgt").get_to(_geoidhgt);
    }
    if (j.contains("gravity"))
    {
        j.at("gravity").get_to(_gravity);
    }
    if (j.contains("initPos"))
    {
        j.at("initPos").get_to(_initPos);
    }
    if (j.contains("gravityInput"))
    {
        j.at("gravityInput").get_to(_gravityInput);
    }
}

void NAV::PressureToHeight::receiveObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto pressureObs = std::static_pointer_cast<const BarometricPressureObs>(queue.extract_front());

    auto baroheight = std::make_shared<BarometricHeight>();

    baroheight->insTime = pressureObs->insTime;

    baroheight->baro_height = _temp0 / _lapserate * (1.0 - std::pow(pressureObs->baro_pressure / _pressure0, 1.0 / _exponent)) + _geoidhgt;

    invokeCallbacks(OUTPUT_PORT_INDEX_BAROHEIGHT, baroheight);
}