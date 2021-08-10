#include "GlobalActions.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "internal/FlowManager.hpp"

#include <imgui_node_editor.h>
namespace ed = ax::NodeEditor;

bool NAV::gui::canCutOrCopyFlowElements()
{
    return static_cast<bool>(ed::GetSelectedObjectCount());
}

bool NAV::gui::canPasteFlowElements()
{
}

void NAV::gui::cutFlowElements()
{
}

void NAV::gui::copyFlowElements()
{
}

void NAV::gui::pasteFlowElements()
{
}

bool NAV::gui::canUndoLastAction()
{
}

bool NAV::gui::canRedoLastAction()
{
}

void NAV::gui::undoLastAction()
{
}

void NAV::gui::redoLastAction()
{
}