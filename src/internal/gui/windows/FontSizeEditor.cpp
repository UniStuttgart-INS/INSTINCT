#include "FontSizeEditor.hpp"

#include "internal/gui/NodeEditorApplication.hpp"
#include <imgui.h>

void NAV::gui::windows::ShowFontSizeEditor(bool* show /* = nullptr*/)
{
    if (!ImGui::Begin("Font Size Editor", show))
    {
        ImGui::End();
        return;
    }

    bool useBigDefaultFont = gui::NodeEditorApplication::isUsingBigDefaultFont();
    if (ImGui::Checkbox("Use big default font", &useBigDefaultFont))
    {
        gui::NodeEditorApplication::swapDefaultFont(useBigDefaultFont);
    }

    bool useBigWindowFont = gui::NodeEditorApplication::isUsingBigWindowFont();
    if (ImGui::Checkbox("Use big window font", &useBigWindowFont))
    {
        gui::NodeEditorApplication::swapWindowFont(useBigWindowFont);
    }

    bool useBigMonoFont = gui::NodeEditorApplication::isUsingBigMonoFont();
    if (ImGui::Checkbox("Use big mono font", &useBigMonoFont))
    {
        gui::NodeEditorApplication::swapMonoFont(useBigMonoFont);
    }

    // bool useBigHeaderFont = gui::NodeEditorApplication::isUsingBigHeaderFont();
    // if (ImGui::Checkbox("Use big header font", &useBigHeaderFont))
    // {
    //     gui::NodeEditorApplication::swapHeaderFont(useBigHeaderFont);
    // }

    ImGui::End();
}