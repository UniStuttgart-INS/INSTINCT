#include "HelpMarker.hpp"

#include "imgui.h"

void NAV::gui::widgets::HelpMarker(const char* desc) // NOLINT(clang-diagnostic-unused-function)
{
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0F);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}
