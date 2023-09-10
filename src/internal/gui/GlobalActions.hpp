// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file GlobalActions.hpp
/// @brief Global Gui Actions
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-12-19

#pragma once

/// @brief Possible Global Actions to perform in the GUI
enum GlobalActions
{
    None,        ///< None
    Quit,        ///< Quit the program
    QuitUnsaved, ///< Quit the program without saving
    SaveAs,      ///< Save the flow as filename
    Clear,       ///< Clear the flow
    Load,        ///< Load a flow
    RunFlow,     ///< Run the flow
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
