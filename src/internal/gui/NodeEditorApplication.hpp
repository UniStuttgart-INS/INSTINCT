/// @file NodeEditorApplication.hpp
/// @brief Gui Callbacks
/// @author T. Topp (thomas@topp.cc)
/// @date 2020-12-14

#pragma once

#include <application.h>

#include <imgui.h>

#include "internal/gui/GlobalActions.hpp"

#include <thread>
#include <deque>

namespace NAV
{
class Node;

namespace gui
{
class NodeEditorApplication : public Application
{
  public:
    /// @brief Default constructor
    NodeEditorApplication() = delete;
    /// @brief Destructor
    ~NodeEditorApplication() override;
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

    void OnStart() override;

    void OnStop() override;

    bool OnQuitRequest() override;

    void OnFrame(float deltaTime) override;

    void ShowQuitRequested();
    void ShowSaveAsRequested();
    void ShowClearNodesRequested();
    void ShowLoadRequested();
    static void ShowRenameNodeRequest(Node*& renameNode);

    int frameCountNavigate = 0;

    static inline ImTextureID m_InstinctLogo = nullptr;

  private:
    ImTextureID m_HeaderBackground = nullptr;

    GlobalActions globalAction = GlobalActions::None;

    // bool initThread_stopRequested = false;
    std::thread initThread;
    size_t currentInitNodeId = 0;
    /// List of Node* & flag (init=true, deinit=false)
    std::deque<std::pair<Node*, bool>> initList;
};

} // namespace gui
} // namespace NAV