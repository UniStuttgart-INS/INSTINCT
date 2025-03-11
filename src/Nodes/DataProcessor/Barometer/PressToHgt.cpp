// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "PressToHgt.hpp"

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/Baro/BaroPressObs.hpp"
#include "NodeData/Baro/BaroHgt.hpp"

#include "Navigation/Gravity/Gravity.hpp"

#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/EnumCombo.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

NAV::PressToHgt::PressToHgt()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 710, 220 };

    nm::CreateInputPin(this, "BaroPressObs", Pin::Type::Flow, { NAV::BaroPressObs::type() }, &PressToHgt::receiveObs);

    nm::CreateOutputPin(this, "BaroHgt", Pin::Type::Flow, { NAV::BaroHgt::type() });
}

NAV::PressToHgt::~PressToHgt()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::PressToHgt::typeStatic()
{
    return "PressToHgt";
}

std::string NAV::PressToHgt::type() const
{
    return typeStatic();
}

std::string NAV::PressToHgt::category()
{
    return "Data Processor";
}

void NAV::PressToHgt::guiConfig()
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

bool NAV::PressToHgt::initialize()
{
    LOG_TRACE("{}: called", nameId());
    if (_gravityInput == GravityInput::Position)
    {
        _gravity = n_calcGravitation_EGM96(_initPos).norm();
        LOG_DATA("{}: PressToHgt - Local gravity magnitude: {} m/s² at initial position: {} [deg, deg, m]", nameId(), _gravity, _initPos.transpose());
    }
    _exponent = _gravity * InsConst::dMtr / InsConst::Rg / _lapserate;
    return true;
}

void NAV::PressToHgt::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

[[nodiscard]] json NAV::PressToHgt::save() const
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

void NAV::PressToHgt::restore(json const& j)
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

void NAV::PressToHgt::receiveObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto pressureObs = std::static_pointer_cast<const BaroPressObs>(queue.extract_front());

    auto baroheight = std::make_shared<BaroHgt>();

    baroheight->insTime = pressureObs->insTime;

    baroheight->baro_height = _temp0 / _lapserate * (1.0 - std::pow(pressureObs->baro_pressure / _pressure0, 1.0 / _exponent)) + _geoidhgt;

    // if uncertainty of pressure value is provided carry out error propagation for barometric height (assuming all other parameters are error free)
    if (pressureObs->baro_pressureStdev.has_value())
    {
        baroheight->baro_heightStdev = std::fabs(_temp0 / (_lapserate * _exponent * pressureObs->baro_pressure) * std::pow(pressureObs->baro_pressure / _pressure0, 1.0 / _exponent - 1.0)) * pressureObs->baro_pressureStdev.value();
    }

    invokeCallbacks(OUTPUT_PORT_INDEX_BAROHEIGHT, baroheight);
}