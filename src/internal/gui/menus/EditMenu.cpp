#include "EditMenu.hpp"

#include <imgui.h>
#include "internal/gui/GlobalActions.hpp"

void NAV::gui::menus::ShowEditMenu()
{
    if (ImGui::MenuItem("Undo", "CTRL+Z", false, canUndoLastAction()))
    {
        undoLastAction();
    }
    if (ImGui::MenuItem("Redo", "CTRL+Y", false, canRedoLastAction()))
    {
        redoLastAction();
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Cut", "CTRL+X", false, canCutOrCopyFlowElements()))
    {
        cutFlowElements();
    }
    if (ImGui::MenuItem("Copy", "CTRL+C", false, canCutOrCopyFlowElements()))
    {
        copyFlowElements();
    }
    if (ImGui::MenuItem("Paste", "CTRL+V", false, canPasteFlowElements()))
    {
        pasteFlowElements();
    }
}