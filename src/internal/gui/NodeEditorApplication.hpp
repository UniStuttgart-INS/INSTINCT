// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// @file NodeEditorApplication.hpp
/// @brief GUI callbacks
/// @author T. Topp (topp@ins.uni-stuttgart.de)
/// @date 2020-12-14

#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <application.h>
#include <imgui.h>
#include <implot.h>

#include "internal/gui/GlobalActions.hpp"
#include "internal/gui/windows/FontSizeEditor.hpp"

namespace NAV
{
class Node;
class Pin;

namespace gui
{
/// @brief Application class providing all relevant GUI callbacks
class NodeEditorApplication : public Application
{
  private:
    constexpr static float BOTTOM_VIEW_COLLAPSED_MIN_HEIGHT = 23.0F;    ///< Minimal height of the bottom view if it is collapsed
    constexpr static float BOTTOM_VIEW_UNCOLLAPSED_MIN_HEIGHT = 200.0F; ///< Minimal height of the bottom view if it is not collapsed

  public:
    /// @brief Default constructor
    NodeEditorApplication() = delete;
    /// @brief Destructor
    ~NodeEditorApplication() override = default;
    /// @brief Copy constructor
    NodeEditorApplication(const NodeEditorApplication&) = delete;
    /// @brief Move constructor
    NodeEditorApplication(NodeEditorApplication&&) = delete;
    /// @brief Copy assignment operator
    NodeEditorApplication& operator=(const NodeEditorApplication&) = delete;
    /// @brief Move assignment operator
    NodeEditorApplication& operator=(NodeEditorApplication&&) = delete;

    /// Bring the Application Constructors into this class (NOLINTNEXTLINE)
    using Application::Application;

    // static Node* SpawnInputActionNode();

    // static Node* SpawnLessNode();

    // static Node* SpawnGroupBox();

    /// @brief Called when the application is started
    void OnStart() override;

    /// @brief Called when the application is stopped
    void OnStop() override;

    /// @brief Called when the user request the application to close
    ///
    /// Checks whether there are unsaved changes and then prompts the user to save them before quit
    /// @return True if no unsaved changes
    bool OnQuitRequest() override;

    /// @brief Called on every frame
    /// @param[in] deltaTime Time since last frame
    void OnFrame(float deltaTime) override;

    /// @brief Shows a PopupModal asking the user if he wants to quit with unsaved changes
    void ShowQuitRequested();
    /// @brief Shows a PopupModel where the user can select a path to save his flow to
    void ShowSaveAsRequested();
    /// @brief Shows a PopupModel to clear the current flow
    void ShowClearNodesRequested();
    /// @brief Shows a PopupModel loading a new flow
    void ShowLoadRequested();
    /// @brief Shows a PopupModal where the user can rename the node
    /// @param[in, out] renameNode Pointer to the node to rename. Pointer gets nulled when finished.
    static void ShowRenameNodeRequest(Node*& renameNode);
    /// @brief Shows a PopupModal where the user can rename the pin
    /// @param[in, out] renamePin Pointer to the pin to rename. Pointer gets nulled when finished.
    static void ShowRenamePinRequest(Pin*& renamePin);

    /// @brief Frame counter to block the navigate to content function till nodes are correctly loaded
    int frameCountNavigate = 0;

    /// Shows the queue size on the pins (every frame the queue mutex will be locked)
    static inline bool _showQueueSizeOnPins = false;

    /// @brief Pointer to the texture for the instinct logo
    static inline std::array<ImTextureID, 2> m_InstinctLogo{ nullptr, nullptr };

    /// @brief Pointer to the texture for the INS logo
    static inline std::array<ImTextureID, 2> m_InsLogo{ nullptr, nullptr };

    /// @brief Pointer to the texture for the save button
    static inline ImTextureID m_SaveButtonImage = nullptr;

    /// @brief Pointer to the texture for the rose figure (ImuSimulator node)
    static inline ImTextureID m_RoseFigure = nullptr;

    /// @brief Default style of the ImPlot library to compare changes against
    static inline ImPlotStyle imPlotReferenceStyle;

    static inline bool hideLeftPane = false;                                 ///< Hide left pane
    static inline bool hideFPS = false;                                      ///< Hide FPS counter
    inline static float leftPaneWidth = 350.0F;                              ///< Width of the left pane
    inline static float rightPaneWidth = 850.0F;                             ///< Width of the right pane
    inline static float menuBarHeight = 20;                                  ///< Height of the menu bar on top
    constexpr static float SPLITTER_THICKNESS = 4.0F;                        ///< Thickness of the splitter between left and right pane
    inline static float bottomViewHeight = BOTTOM_VIEW_COLLAPSED_MIN_HEIGHT; ///< Height of the log viewer

    /// Ratio to multiply for default GUI elements
    static float defaultFontRatio();
    /// Ratio to multiply for GUI window elements
    static float windowFontRatio();
    /// Ratio to multiply for GUI editor elements
    static float panelFontRatio();
    /// Ratio to multiply for log output GUI elements
    static float monoFontRatio();
    /// Ratio to multiply for node header elements
    static float headerFontRatio();

    /// Available color settings
    enum Colors : uint8_t
    {
        COLOR_GROUP_HEADER_TEXT,  ///< Color of the group header text
        COLOR_GROUP_HEADER_BG,    ///< Color of the group header background
        COLOR_GROUP_OUTER_BORDER, ///< Color of the group outer border
    };

    /// Color settings
    static inline std::vector<ImVec4> m_colors = {
        { 1.0, 1.0, 1.0, 1.0 },  // GROUP_HEADER_TEXT
        { 1.0, 1.0, 1.0, 0.25 }, // GROUP_HEADER_BG
        { 1.0, 1.0, 1.0, 0.25 }, // GROUP_OUTER_BORDER
    };
    /// Color names
    static inline std::vector<const char*> m_colorsNames = {
        "GroupHeaderText",  // GROUP_HEADER_TEXT
        "GroupHeaderBg",    // GROUP_HEADER_BG
        "GroupOuterBorder", // GROUP_OUTER_BORDER
    };

  private:
    /// @brief Tabs displayed in the bottom view
    enum class BottomViewTabItem : uint8_t
    {
        None,      ///< The cross item is selected
        LogOutput, ///< The log output item is selected
    };

    BottomViewTabItem bottomViewSelectedTab = BottomViewTabItem::None; ///< Selected Tab item in the bottom view

    /// @brief Pointer to the texture for the node headers
    ImTextureID m_HeaderBackground = nullptr;

    /// @brief Global action to execute
    GlobalActions globalAction = GlobalActions::None; // TODO: Move to the GlobalAction.cpp as a global variable

    /// @brief Shows a window for choosing the font size
    /// @param[in, out] show Flag which indicates whether the window is shown
    friend void windows::ShowFontSizeEditor(bool* show);
};

} // namespace gui
} // namespace NAV