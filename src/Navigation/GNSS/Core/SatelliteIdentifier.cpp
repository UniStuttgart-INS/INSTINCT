// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SatelliteIdentifier.hpp"

#include <imgui.h>
#include <unordered_set>
#include <algorithm>
#include "Navigation/GNSS/Core/SatelliteSystem.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/BDSEphemeris.hpp"
#include "Navigation/GNSS/Satellite/Ephemeris/QZSSEphemeris.hpp"
#include <fmt/core.h>
#include <fmt/ranges.h>

namespace NAV
{

bool SatId::isGeo() const
{
    if (satSys == QZSS)
    {
        if (satNum == 3) { return true; }
    }
    else if (satSys == BDS)
    {
        if (satNum <= 5 || (satNum >= 59 && satNum <= 63)) { return true; }
    }
    else if (satSys == IRNSS)
    {
        if (std::unordered_set<uint16_t> sats = { 3, 6, 7 };
            sats.contains(satNum)) { return true; }
    }
    return false;
}

bool lessCompareSatSigId(const std::string& lhs, const std::string& rhs)
{
    auto lhsSatSys = SatelliteSystem::fromChar(lhs.substr(0, 1).at(0));
    auto rhsSatSys = SatelliteSystem::fromChar(rhs.substr(0, 1).at(0));
    if (lhsSatSys == rhsSatSys)
    {
        auto lhsSatNum = std::stoi(lhs.substr(lhs.size() - 2, 2));
        auto rhsSatNum = std::stoi(rhs.substr(rhs.size() - 2, 2));
        if (lhsSatNum == rhsSatNum)
        {
            // Code, e.g. G1C-02
            return lhs.substr(1, 2) < rhs.substr(1, 2);
        }
        return lhsSatNum < rhsSatNum;
    }
    return lhsSatSys < rhsSatSys;
}

void to_json(json& j, const SatId& data)
{
    j = json{
        { "sys", data.satSys },
        { "num", data.satNum },
    };
}
void from_json(const json& j, SatId& data)
{
    j.at("sys").get_to(data.satSys);
    j.at("num").get_to(data.satNum);
}

void to_json(json& j, const SatSigId& data)
{
    j = json{
        { "code", data.code },
        { "num", data.satNum },
    };
}
void from_json(const json& j, SatSigId& data)
{
    j.at("code").get_to(data.code);
    j.at("num").get_to(data.satNum);
}

bool ShowSatelliteSelector(const char* label, std::vector<SatId>& satellites, SatelliteSystem filterSys, bool displayOnlyNumber)
{
    bool valueChanged = false;
    std::string preview;
    if (displayOnlyNumber)
    {
        for (size_t i = 0; i < satellites.size(); i++)
        {
            if (i != 0) { preview += " | "; }
            preview += std::to_string(satellites.at(i).satNum);
        }
    }
    else { preview = fmt::format("{}", fmt::join(satellites, ", ")); }

    if (ImGui::BeginCombo(label, preview.c_str()))
    {
        if (ImGui::BeginTable(fmt::format("{} Table", label).c_str(), 7, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_ScrollY))
        {
            ImGui::TableSetupScrollFreeze(0, 1);
            for (uint64_t satSys = 0xFF; satSys < 0xFFUL << (7 * 8); satSys = satSys << 8UL)
            {
                ImGui::TableSetupColumn(std::string(SatelliteSystem(SatelliteSystem_(satSys))).c_str());
            }
            ImGui::TableHeadersRow();

            ImGui::TableNextRow();
            for (uint64_t satSys = 0xFF; satSys < 0xFFUL << (7 * 8); satSys = satSys << 8UL)
            {
                auto satSystem = SatelliteSystem(SatelliteSystem_(satSys));

                ImGui::TableNextColumn();
                for (const auto& num : satSystem.getSatellites())
                {
                    SatId satId{ satSystem, num };
                    auto iter = std::ranges::find(satellites, satId);
                    bool isExcluded = iter != satellites.end();
                    if (!SatelliteSystem_(satSystem & filterSys) || satId.isGeo()) { ImGui::BeginDisabled(); }

                    auto satInfo = satSystem.getSatelliteInfo(num);

                    if (ImGui::Checkbox(fmt::format("{}{}##{} {}",
                                                    num,
                                                    satInfo ? fmt::format(" ({})", *satInfo) : "",
                                                    satSys,
                                                    label)
                                            .c_str(),
                                        &isExcluded))
                    {
                        if (isExcluded)
                        {
                            satellites.push_back(satId);
                            std::sort(satellites.begin(), satellites.end()); // NOLINT(boost-use-ranges,modernize-use-ranges) // ranges::sort is not supported yet
                        }
                        else
                        {
                            satellites.erase(iter);
                        }
                        valueChanged = true;
                    }
                    if (!SatelliteSystem_(satSystem & filterSys) || satId.isGeo()) { ImGui::EndDisabled(); }
                }
            }

            ImGui::EndTable();
        }
        ImGui::EndCombo();
    }
    return valueChanged;
}

bool ShowSatelliteSelector(const char* label, SatId& satellite, SatelliteSystem filterSys, bool displayOnlyNumber)
{
    std::vector<SatId> vec = { satellite };

    if (ShowSatelliteSelector(label, vec, filterSys, displayOnlyNumber))
    {
        for (const auto& s : vec)
        {
            if (s != satellite)
            {
                satellite = s;
                return true;
            }
        }
        return false;
    }
    return false;
}

} // namespace NAV

std::ostream& operator<<(std::ostream& os, const NAV::SatId& obj)
{
    return os << fmt::format("{}", obj);
}

std::ostream& operator<<(std::ostream& os, const NAV::SatSigId& obj)
{
    return os << fmt::format("{}", obj);
}