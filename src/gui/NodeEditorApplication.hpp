/// @file NodeEditorApplication.hpp
/// @brief Gui Callbacks
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-14

#pragma once

#include <application.h>

#include <imgui.h>

#include "gui/GlobalActions.hpp"

class Node;

namespace NAV::gui
{
class NodeEditorApplication : public Application
{
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

    /// Bring the Application Constructors into this class
    using Application::Application;

    // static Node* SpawnInputActionNode();

    // static Node* SpawnLessNode();

    // static Node* SpawnGroupBox();

    void OnStart() override;

    void OnStop() override;

    bool OnQuitRequest() override;

    void OnFrame(float deltaTime) override;

    void ShowQuitRequested();
    void ShowSaveAsRequested();
    void ShowClearNodesRequested();
    void ShowLoadRequested();

  private:
    ImTextureID m_HeaderBackground = nullptr;

    GlobalActions globalAction = GlobalActions::None;

    bool shortcutsEnabled = true;

    int frameCountNavigate = 0;
};

} // namespace NAV::gui