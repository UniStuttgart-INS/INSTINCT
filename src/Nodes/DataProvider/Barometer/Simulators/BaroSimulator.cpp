// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "BaroSimulator.hpp"

#include <limits>

#include "util/Logger.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/State/Pos.hpp"
#include "NodeData/Baro/BarometricPressureObs.hpp"

#include "Navigation/Gravity/Gravity.hpp"

#include "internal/gui/widgets/imgui_ex.hpp"
#include "internal/gui/widgets/EnumCombo.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/NodeEditorApplication.hpp"

NAV::BaroSimulator::BaroSimulator()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);
    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 710, 220 };

    std::mt19937_64 gen(static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count()));
    std::uniform_int_distribution<uint64_t> dist(0, std::numeric_limits<uint64_t>::max() / 2);
    _pressureRng.seed = dist(gen);

    nm::CreateOutputPin(this, "BarometricPressureObs", Pin::Type::Flow, { NAV::BarometricPressureObs::type() });

    nm::CreateInputPin(this, "Pos", Pin::Type::Flow, { NAV::Pos::type() }, &BaroSimulator::receiveObs);
}

NAV::BaroSimulator::~BaroSimulator()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::BaroSimulator::typeStatic()
{
    return "BaroSimulator";
}

std::string NAV::BaroSimulator::type() const
{
    return typeStatic();
}

std::string NAV::BaroSimulator::category()
{
    return "Data Simulator";
}

void NAV::BaroSimulator::guiConfig()
{
    float columnWidth = 140.0F * gui::NodeEditorApplication::windowFontRatio();

    if (ImGui::InputDouble(fmt::format("Sea-level pressure##{}", size_t(id)).c_str(), &_pressure0, 0.0, 0.0, "%.3f hPa"))
    {
        LOG_DATA("{}: pressure0 changed to {}", nameId(), _pressure0);
        flow::ApplyChanges();
    }
    if (ImGui::InputDouble(fmt::format("Sea-level temperature##{}", size_t(id)).c_str(), &_temp0, 0.0, 0.0, "%.3f K"))
    {
        LOG_DATA("{}: temo0 changed to {}", nameId(), _temp0);
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
    else // (_gravityInput == GravityInput::position)
    {
        if (ImGui::InputDouble3(fmt::format("Position LLA [deg, deg, m]##{}", size_t(id)).c_str(), _initPos.data()))
        {
            flow::ApplyChanges();
        }
    }
    if (ImGui::InputDouble(fmt::format("Std. Dev. of Pressure noise##{}", size_t(id)).c_str(), &_pressurenoise, 0.0, 0.0, "%.3f  hPa"))
    {
        LOG_DEBUG("{}: pressurenoise changed to {}", nameId(), _pressurenoise);
        flow::ApplyChanges();
    }
    float itemWidth = 470 * gui::NodeEditorApplication::windowFontRatio();
    auto rngInput = [&](const char* title, RandomNumberGenerator& rng) {
        float currentCursorX = ImGui::GetCursorPosX();
        if (ImGui::Checkbox(fmt::format("##rng.useSeed {} {}", title, size_t(id)).c_str(), &rng.useSeed))
        {
            LOG_DEBUG("{}: {} rng.useSeed changed to {}", nameId(), title, rng.useSeed);
            flow::ApplyChanges();
        }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Use seed?"); }
        ImGui::SameLine();
        if (!rng.useSeed)
        {
            ImGui::BeginDisabled();
        }
        ImGui::SetNextItemWidth(itemWidth - (ImGui::GetCursorPosX() - currentCursorX));
        if (ImGui::SliderULong(fmt::format("{} Seed##{}", title, size_t(id)).c_str(), &rng.seed, 0, std::numeric_limits<uint64_t>::max() / 2, "%lu"))
        {
            LOG_DEBUG("{}: {} rng.seed changed to {}", nameId(), title, rng.seed);
            flow::ApplyChanges();
        }
        if (!rng.useSeed)
        {
            ImGui::EndDisabled();
        }
    };
    rngInput("Pressure Noise seed", _pressureRng);
}

bool NAV::BaroSimulator::initialize()
{
    LOG_TRACE("{}: called", nameId());
    if (_gravityInput == GravityInput::Position)
    {
        _gravity = n_calcGravitation_EGM96(_initPos).norm();
        LOG_DATA("{}: PressureToHeight - Local gravity magnitude: {} m/s² at initial position: {} [deg, deg, m]", nameId(), _gravity, _initPos.transpose());
    }
    _exponent = _gravity * InsConst::dMtr / InsConst::Rg / _lapserate;
    _pressureRng.resetSeed();
    return true;
}

void NAV::BaroSimulator::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

[[nodiscard]] json NAV::BaroSimulator::save() const
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
    j["pressurenoise"] = _pressurenoise;
    j["pressureRng"] = _pressureRng;
    return j;
}

void NAV::BaroSimulator::restore(json const& j)
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
    if (j.contains("pressurenoise"))
    {
        j.at("pressurenoise").get_to(_pressurenoise);
    }
    if (j.contains("pressureRng"))
    {
        j.at("pressureRng").get_to(_pressureRng);
    }
}

void NAV::BaroSimulator::receiveObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto PosObs = std::static_pointer_cast<const Pos>(queue.extract_front());

    auto pressureObs = std::make_shared<BarometricPressureObs>();

    pressureObs->insTime = PosObs->insTime;

    pressureObs->baro_pressure = _pressure0 * std::pow(1 - _lapserate * (PosObs->altitude() - _geoidhgt) / _temp0, _exponent) + _pressureRng.getRand_normalDist(0.0, _pressurenoise);

    invokeCallbacks(OUTPUT_PORT_INDEX_BAROPRESSURE, pressureObs);
}