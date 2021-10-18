#include "HelpMarker.hpp"

#include "imgui.h"
#include <imgui_internal.h>

void NAV::gui::widgets::HelpMarker(const char* desc, const char* symbol) // NOLINT(clang-diagnostic-unused-function)
{
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
    ImGui::TextUnformatted(symbol);
    ImGui::PopItemFlag();
    ImGui::PopStyleVar();

    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0F);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

bool NAV::gui::widgets::BeginHelpMarker(const char* symbol)
{
    ImGui::PushItemFlag(ImGuiItemFlags_Disabled, true);
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5F);
    ImGui::TextUnformatted(symbol);
    ImGui::PopItemFlag();
    ImGui::PopStyleVar();

    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0F);
        return true;
    }
    return false;
}

void NAV::gui::widgets::EndHelpMarker()
{
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
}