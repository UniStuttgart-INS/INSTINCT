/// This file is part of INSTINCT, the INS Toolkit for Integrated
/// Navigation Concepts and Training by the Institute of Navigation of
/// the University of Stuttgart, Germany.
///
/// This Source Code Form is subject to the terms of the Mozilla Public
/// License, v. 2.0. If a copy of the MPL was not distributed with this
/// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file RandomNumberGenerator.cpp
/// @brief Random Number generator
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2023-08-24

#include "RandomNumberGenerator.hpp"

#include <imgui.h>
#include "util/Logger.hpp"
#include "internal/gui/widgets/imgui_ex.hpp"

namespace NAV
{

bool RandomNumberGenerator::showGui(const char* title, float itemWidth, const std::string& nameId)
{
    bool changed = false;
    float currentCursorX = ImGui::GetCursorPosX();
    if (ImGui::Checkbox(fmt::format("##rng.useSeed {} {}", title, nameId).c_str(), &useSeed))
    {
        LOG_DEBUG("{}: {} rng.useSeed changed to {}", nameId, title, useSeed);
        changed = true;
    }
    if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Use seed?"); }
    ImGui::SameLine();
    if (!useSeed)
    {
        ImGui::BeginDisabled();
    }
    ImGui::SetNextItemWidth(itemWidth - (ImGui::GetCursorPosX() - currentCursorX));
    if (ImGui::SliderULong(fmt::format("{} Seed##{}", title, nameId).c_str(), &seed, 0, std::numeric_limits<uint64_t>::max() / 2, "%lu"))
    {
        LOG_DEBUG("{}: {} rng.seed changed to {}", nameId, title, seed);
        changed = true;
    }
    if (!useSeed)
    {
        ImGui::EndDisabled();
    }
    return changed;
}

void to_json(json& j, const RandomNumberGenerator& rng)
{
    j = json{
        { "useSeed", rng.useSeed },
        { "seed", rng.seed },
    };
}
void from_json(const json& j, RandomNumberGenerator& rng)
{
    if (j.contains("useSeed"))
    {
        j.at("useSeed").get_to(rng.useSeed);
    }
    if (j.contains("seed"))
    {
        j.at("seed").get_to(rng.seed);
    }
}

} // namespace NAV