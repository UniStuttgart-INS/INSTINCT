// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SinglePointPositioning.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"
#include "internal/gui/NodeEditorApplication.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"

#include "Navigation/Constants.hpp"

#include "NodeData/GNSS/GnssObs.hpp"
#include "NodeData/GNSS/GnssNavInfo.hpp"
#include "NodeData/GNSS/SppSolution.hpp"

#include "util/Logger.hpp"
#include "util/Container/Vector.hpp"

NAV::SinglePointPositioning::SinglePointPositioning()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 538, 536 };

    nm::CreateInputPin(this, NAV::GnssObs::type().c_str(), Pin::Type::Flow, { NAV::GnssObs::type() }, &SinglePointPositioning::recvGnssObs);
    _dynamicInputPins.addPin(this); // GnssNavInfo

    nm::CreateOutputPin(this, NAV::SppSolution::type().c_str(), Pin::Type::Flow, { NAV::SppSolution::type() });
}

NAV::SinglePointPositioning::~SinglePointPositioning()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::SinglePointPositioning::typeStatic()
{
    return "SinglePointPositioning - SPP";
}

std::string NAV::SinglePointPositioning::type() const
{
    return typeStatic();
}

std::string NAV::SinglePointPositioning::category()
{
    return "Data Processor";
}

void NAV::SinglePointPositioning::guiConfig()
{
    auto nSatColumnContent = [&](size_t pinIndex) -> bool {
        if (const auto* gnssNavInfo = getInputValue<const GnssNavInfo>(pinIndex))
        {
            size_t usedSatNum = 0;
            std::string usedSats;
            std::string allSats;

            std::string filler = ", ";
            for (const auto& satellite : gnssNavInfo->satellites())
            {
                if (_algorithm.getObsFilter().isSatelliteAllowed(satellite.first))
                {
                    usedSatNum++;
                    usedSats += (allSats.empty() ? "" : filler) + fmt::format("{}", satellite.first);
                }
                allSats += (allSats.empty() ? "" : filler) + fmt::format("{}", satellite.first);
            }
            ImGui::TextUnformatted(fmt::format("{} / {}", usedSatNum, gnssNavInfo->nSatellites()).c_str());
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("Used  satellites: %s\n"
                                  "Avail satellites: %s",
                                  usedSats.c_str(), allSats.c_str());
            }
        }
        return false;
    };

    if (_dynamicInputPins.ShowGuiWidgets(size_t(id), inputPins, this, { { "# Sat", nSatColumnContent } }))
    {
        flow::ApplyChanges();
    }

    ImGui::Separator();

    // ###########################################################################################################

    const float itemWidth = 280.0F * gui::NodeEditorApplication::windowFontRatio();
    const float unitWidth = 100.0F * gui::NodeEditorApplication::windowFontRatio();

    if (_algorithm.ShowGuiWidgets(nameId().c_str(), itemWidth, unitWidth))
    {
        flow::ApplyChanges();
    }
}

[[nodiscard]] json NAV::SinglePointPositioning::save() const
{
    LOG_TRACE("{}: called", nameId());

    return {
        { "dynamicInputPins", _dynamicInputPins },
        { "algorithm", _algorithm }
    };
}

void NAV::SinglePointPositioning::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("dynamicInputPins")) { NAV::gui::widgets::from_json(j.at("dynamicInputPins"), _dynamicInputPins, this); }
    if (j.contains("algorithm")) { j.at("algorithm").get_to(_algorithm); }
}

bool NAV::SinglePointPositioning::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (std::all_of(inputPins.begin() + INPUT_PORT_INDEX_GNSS_NAV_INFO, inputPins.end(), [](const InputPin& inputPin) { return !inputPin.isPinLinked(); }))
    {
        LOG_ERROR("{}: You need to connect a GNSS NavigationInfo provider", nameId());
        return false;
    }

    _algorithm.reset();

    LOG_DEBUG("{}: initialized", nameId());

    return true;
}

void NAV::SinglePointPositioning::deinitialize()
{
    LOG_TRACE("{}: called", nameId());
}

void NAV::SinglePointPositioning::pinAddCallback(Node* node)
{
    nm::CreateInputPin(node, NAV::GnssNavInfo::type().c_str(), Pin::Type::Object, { NAV::GnssNavInfo::type() });
}

void NAV::SinglePointPositioning::pinDeleteCallback(Node* node, size_t pinIdx)
{
    nm::DeleteInputPin(node->inputPins.at(pinIdx));
}

void NAV::SinglePointPositioning::recvGnssObs(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    // Collection of all connected navigation data providers
    std::vector<const GnssNavInfo*> gnssNavInfos;
    std::vector<std::unique_lock<std::mutex>> guards;
    for (size_t i = 0; i < _dynamicInputPins.getNumberOfDynamicPins(); i++)
    {
        if (auto* mutex = getInputValueMutex(INPUT_PORT_INDEX_GNSS_NAV_INFO + i))
        {
            guards.emplace_back(*mutex);
            if (const auto* gnssNavInfo = getInputValue<const GnssNavInfo>(INPUT_PORT_INDEX_GNSS_NAV_INFO + i))
            {
                gnssNavInfos.push_back(gnssNavInfo);
            }
            else
            {
                guards.pop_back();
            }
        }
    }
    if (gnssNavInfos.empty()) { return; }

    auto gnssObs = std::static_pointer_cast<const GnssObs>(queue.extract_front());
    LOG_DATA("{}: Calculating SPP for [{}]", nameId(), gnssObs->insTime);

    if (auto sppSol = _algorithm.calcSppSolution(gnssObs, gnssNavInfos, nameId()))
    {
        invokeCallbacks(OUTPUT_PORT_INDEX_SPPSOL, sppSol);
    }
}