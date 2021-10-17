/// @file GlobalActions.hpp
/// @brief Global Gui Actions
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-19

#pragma once

/// @brief Possible Global Actions to perform in the GUI
enum GlobalActions
{
    None,
    Quit,
    QuitUnsaved,
    SaveAs,
    Clear,
    Load,
    RunFlow,
};

namespace NAV::gui
{
/// @brief Checks if elements can be cutted/copied
bool canCutOrCopyFlowElements();

/// @brief Checks if elements can be pasted
bool canPasteFlowElements();

/// @brief Cuts the currently selected elements
void cutFlowElements();

/// @brief Copies the currently selected elements
void copyFlowElements();

/// @brief Pastes the copied/cutted elements
void pasteFlowElements();

/// @brief Checks if an action can be undone
bool canUndoLastAction();

/// @brief Checks if an action can be redone
bool canRedoLastAction();

/// @brief Clears the list of last actions
void clearLastActionList();

/// @brief Undo the last action
void undoLastAction();

/// @brief Redo the last action
void redoLastAction();

/// @brief Saves the last action to the action list
void saveLastAction();

} // namespace NAV::gui
