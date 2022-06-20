#include "SatelliteIdentifier.hpp"

#include <imgui.h>
#include <fmt/core.h>
#include <fmt/ranges.h>

namespace NAV
{

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
        { "freq", data.freq },
        { "num", data.satNum },
    };
}
void from_json(const json& j, SatSigId& data)
{
    j.at("freq").get_to(data.freq);
    j.at("num").get_to(data.satNum);
}

bool ShowSatelliteSelector(const char* label, std::vector<SatId>& satellites)
{
    bool valueChanged = false;
    if (ImGui::BeginCombo(label, fmt::format("{}", fmt::join(satellites, ", ")).c_str()))
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
                    auto iter = std::find(satellites.begin(), satellites.end(), satId);
                    bool isExcluded = iter != satellites.end();
                    if (ImGui::Checkbox(fmt::format("{}##{} {}", num, satSys, label).c_str(), &isExcluded))
                    {
                        if (isExcluded)
                        {
                            satellites.push_back(satId);
                            std::sort(satellites.begin(), satellites.end());
                        }
                        else
                        {
                            satellites.erase(iter);
                        }
                        valueChanged = true;
                    }
                }
            }

            ImGui::EndTable();
        }
        ImGui::EndCombo();
    }
    return valueChanged;
}

} // namespace NAV
