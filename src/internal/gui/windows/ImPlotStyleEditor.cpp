#include "ImPlotStyleEditor.hpp"

#include <imgui.h>
#include <implot.h>
#include <implot_internal.h>

#include "internal/gui/widgets/HelpMarker.hpp"

void NAV::gui::windows::ShowImPlotStyleEditor(bool* show /* = nullptr*/)
{
    if (!ImGui::Begin("Node Editor Style", show))
    {
        ImGui::End();
        return;
    }


    ImGui::End();
}