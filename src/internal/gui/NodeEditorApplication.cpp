#include "NodeEditorApplication.hpp"

#include <imgui_node_editor.h>
#include <imgui_node_editor_internal.h>
namespace ed = ax::NodeEditor;

#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui_internal.h>
#include <imgui_stdlib.h>

#include "ImGuiFileDialog.h"

#include "implot.h"

#include "internal/gui/Shortcuts.hpp"
#include "internal/gui/TouchTracker.hpp"
#include "internal/gui/FlowAnimation.hpp"

#include "internal/gui/panels/BlueprintNodeBuilder.hpp"
namespace util = ax::NodeEditor::Utilities;

#include "internal/gui/panels/LeftPane.hpp"

#include "internal/gui/menus/MainMenuBar.hpp"

#include "internal/gui/widgets/Splitter.hpp"
#include "internal/gui/widgets/HelpMarker.hpp"
#include "internal/gui/widgets/Spinner.hpp"

#include "internal/gui/windows/Global.hpp"
#include "internal/gui/windows/ImPlotStyleEditor.hpp"

#include "internal/Node/Pin.hpp"
#include "internal/Node/Node.hpp"
#include "internal/Node/Link.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "NodeRegistry.hpp"

#include "internal/ConfigManager.hpp"
#include "internal/FlowManager.hpp"
#include "internal/FlowExecutor.hpp"

#include "util/Json.hpp"

#include <string>
#include <array>
#include <vector>
#include <map>
#include <algorithm>
#include <utility>
#include <filesystem>

#include "internal/CMakeRC.hpp"

ax::NodeEditor::EditorContext* m_Editor = nullptr;

NAV::gui::NodeEditorApplication::~NodeEditorApplication()
{
    if (initThread.joinable())
    {
        // initThread_stopRequested = true;
        initThread.join();
    }
}

void NAV::gui::NodeEditorApplication::OnStart()
{
    LOG_TRACE("called");

    ed::Config config;

    // config.SettingsFile = "INSTINCT.json";
    // config.UserPointer = this;

    // Stops the Editor from creating a log file, as we do it ourselves
    // clang-format off
    config.SaveSettings = [](const char* /*data*/, size_t /*size*/,
                             ed::SaveReasonFlags /*reason*/, void* /*userPointer*/) -> bool {
        // clang-format on

        return false;
    };

    // Trigger the changed bar on the node overview list when a node is moved
    // clang-format off
    config.SaveNodeSettings = [](ed::NodeId nodeId, const char* /*data*/, size_t /*size*/,
                                 ed::SaveReasonFlags /*reason*/, void* /*userPointer*/) -> bool {
        // clang-format on

        // auto* self = static_cast<NodeEditorApplication*>(userPointer);

        flow::ApplyChanges();
        gui::TouchNode(nodeId);

        return true;
    };

    m_Editor = ed::CreateEditor(&config);
    ed::SetCurrentEditor(m_Editor);

    ImGui::GetStyle().FrameRounding = 4.0F;
    ed::GetStyle().FlowDuration = 1.0F;

    ImPlot::CreateContext();
    imPlotReferenceStyle = ImPlot::GetStyle();

    std::filesystem::path imPlotConfigFilepath = flow::GetProgramRootPath();
    if (std::filesystem::path inputPath{ ConfigManager::Get<std::string>("implot-config") };
        inputPath.is_relative())
    {
        imPlotConfigFilepath /= inputPath;
    }
    else
    {
        imPlotConfigFilepath = inputPath;
    }
    std::ifstream imPlotConfigFilestream(imPlotConfigFilepath);

    if (!imPlotConfigFilestream.good())
    {
        LOG_ERROR("The ImPlot style config file could not be loaded: {}", imPlotConfigFilepath);
    }
    else
    {
        json j;
        imPlotConfigFilestream >> j;

        if (j.contains("implot") && j.at("implot").contains("style"))
        {
            j.at("implot").at("style").get_to(ImPlot::GetStyle());
        }
    }

    auto fs = cmrc::instinct::get_filesystem();

    if (fs.is_file("resources/images/BlueprintBackground.png"))
    {
        auto fd = fs.open("resources/images/BlueprintBackground.png");

        LOG_DEBUG("Generating Texture for Blueprint Background ({} byte)", fd.size());

        auto is = cmrc::memstream(const_cast<char*>(fd.begin()), // NOLINT(cppcoreguidelines-pro-type-const-cast)
                                  const_cast<char*>(fd.end()));  // NOLINT(cppcoreguidelines-pro-type-const-cast)

        std::vector<char> buffer;
        buffer.resize(fd.size(), '\0');

        is.read(buffer.data(),
                static_cast<std::streamsize>(buffer.size()));

        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        m_HeaderBackground = LoadTexture(reinterpret_cast<const void*>(const_cast<const char*>(buffer.data())),
                                         static_cast<int>(fd.size()));
    }
    else
    {
        m_HeaderBackground = LoadTexture("resources/images/BlueprintBackground.png");
    }

    if (fs.is_file("resources/images/INSTINCT_Logo_Text_white_small.png"))
    {
        auto fd = fs.open("resources/images/INSTINCT_Logo_Text_white_small.png");

        LOG_DEBUG("Generating Texture for INSTINCT Logo ({} byte)", fd.size());

        auto is = cmrc::memstream(const_cast<char*>(fd.begin()), // NOLINT(cppcoreguidelines-pro-type-const-cast)
                                  const_cast<char*>(fd.end()));  // NOLINT(cppcoreguidelines-pro-type-const-cast)

        std::vector<char> buffer;
        buffer.resize(fd.size(), '\0');

        is.read(buffer.data(),
                static_cast<std::streamsize>(buffer.size()));

        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        m_InstinctLogo = LoadTexture(reinterpret_cast<const void*>(const_cast<const char*>(buffer.data())),
                                     static_cast<int>(fd.size()));
    }
    else
    {
        m_InstinctLogo = LoadTexture("resources/images/INSTINCT_Logo_Text_white_small.png");
    }
}

void NAV::gui::NodeEditorApplication::OnStop()
{
    LOG_TRACE("called");

    FlowExecutor::stop();

    initList.clear();
    nm::Stop();

    nm::DeleteAllNodes();

    auto releaseTexture = [this](ImTextureID& id) {
        if (id)
        {
            DestroyTexture(id);
            id = nullptr;
        }
    };

    releaseTexture(m_InstinctLogo);
    releaseTexture(m_HeaderBackground);

    if (m_Editor)
    {
        ed::DestroyEditor(m_Editor);
        m_Editor = nullptr;
    }
    if (ImPlotContext* ctx = ImPlot::GetCurrentContext())
    {
        ImPlot::DestroyContext(ctx);
    }
}

bool NAV::gui::NodeEditorApplication::OnQuitRequest()
{
    LOG_TRACE("called");

    if (flow::HasUnsavedChanges())
    {
        globalAction = GlobalActions::QuitUnsaved;
        return false;
    }

    return true;
}

void NAV::gui::NodeEditorApplication::ShowQuitRequested()
{
    auto& io = ImGui::GetIO();
    if (!io.KeyCtrl && !io.KeyAlt && !io.KeyShift && !io.KeySuper)
    {
        if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Escape)))
        {
            globalAction = GlobalActions::None;
            return;
        }
    }

    ImGui::OpenPopup("Quit with unsaved changes?");
    if (ImGui::BeginPopupModal("Quit with unsaved changes?", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::Text("Do you want to save your changes or discard them?");
        if (ImGui::Button("Save"))
        {
            if (flow::GetCurrentFilename().empty())
            {
                ImGuiFileDialog::Instance()->OpenModal("Save Flow", "Save Flow", ".flow", (flow::GetFlowPath() / ".").string(), 1, nullptr, ImGuiFileDialogFlags_ConfirmOverwrite);
                ImGuiFileDialog::Instance()->SetFileStyle(IGFD_FileStyleByExtention, ".flow", ImVec4(0.0F, 1.0F, 0.0F, 0.9F));
            }
            else
            {
                flow::SaveFlowAs(flow::GetCurrentFilename());
                globalAction = GlobalActions::None;
                flow::DiscardChanges();
                Quit();
                ImGui::CloseCurrentPopup();
            }
        }

        if (ImGuiFileDialog::Instance()->Display("Save Flow", ImGuiWindowFlags_NoCollapse, ImVec2(600, 400)))
        {
            if (ImGuiFileDialog::Instance()->IsOk())
            {
                flow::SetCurrentFilename(ImGuiFileDialog::Instance()->GetFilePathName());
                flow::SaveFlowAs(flow::GetCurrentFilename());
            }

            globalAction = GlobalActions::None;
            ImGuiFileDialog::Instance()->Close();
            Quit();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Discard"))
        {
            globalAction = GlobalActions::None;
            flow::DiscardChanges();
            Quit();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            globalAction = GlobalActions::None;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void NAV::gui::NodeEditorApplication::ShowSaveAsRequested()
{
    ImGuiFileDialog::Instance()->OpenDialog("Save Flow", "Save Flow", ".flow", (flow::GetFlowPath() / ".").string(), 1, nullptr, ImGuiFileDialogFlags_ConfirmOverwrite);
    ImGuiFileDialog::Instance()->SetFileStyle(IGFD_FileStyleByExtention, ".flow", ImVec4(0.0F, 1.0F, 0.0F, 0.9F));

    auto& io = ImGui::GetIO();
    if (!io.KeyCtrl && !io.KeyAlt && !io.KeyShift && !io.KeySuper)
    {
        if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Escape)))
        {
            globalAction = GlobalActions::None;
            ImGuiFileDialog::Instance()->Close();
            return;
        }
    }

    if (ImGuiFileDialog::Instance()->Display("Save Flow", ImGuiWindowFlags_NoCollapse, ImVec2(600, 400)))
    {
        if (ImGuiFileDialog::Instance()->IsOk())
        {
            flow::SetCurrentFilename(ImGuiFileDialog::Instance()->GetFilePathName());
            flow::SaveFlowAs(flow::GetCurrentFilename());
        }

        globalAction = GlobalActions::None;
        ImGuiFileDialog::Instance()->Close();
    }
}

void NAV::gui::NodeEditorApplication::ShowClearNodesRequested()
{
    auto& io = ImGui::GetIO();
    if (!io.KeyCtrl && !io.KeyAlt && !io.KeyShift && !io.KeySuper)
    {
        if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Escape)))
        {
            globalAction = GlobalActions::None;
            return;
        }
    }

    ImGui::OpenPopup("Clear Nodes and discard unsaved changes?");
    if (ImGui::BeginPopupModal("Clear Nodes and discard unsaved changes?", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::Text("Do you want to save your changes before clearing the nodes?");
        if (ImGui::Button("Save"))
        {
            if (flow::GetCurrentFilename().empty())
            {
                ImGuiFileDialog::Instance()->OpenModal("Save Flow", "Save Flow", ".flow", (flow::GetFlowPath() / ".").string(), 1, nullptr, ImGuiFileDialogFlags_ConfirmOverwrite);
                ImGuiFileDialog::Instance()->SetFileStyle(IGFD_FileStyleByExtention, ".flow", ImVec4(0.0F, 1.0F, 0.0F, 0.9F));
            }
            else
            {
                flow::SaveFlowAs(flow::GetCurrentFilename());
                globalAction = GlobalActions::None;
                nm::DeleteAllLinksAndNodes();
                flow::DiscardChanges();
                flow::SetCurrentFilename("");
                ImGui::CloseCurrentPopup();
            }
        }

        if (ImGuiFileDialog::Instance()->Display("Save Flow", ImGuiWindowFlags_NoCollapse, ImVec2(600, 400)))
        {
            if (ImGuiFileDialog::Instance()->IsOk())
            {
                flow::SetCurrentFilename(ImGuiFileDialog::Instance()->GetFilePathName());
                flow::SaveFlowAs(flow::GetCurrentFilename());

                ImGuiFileDialog::Instance()->Close();
                nm::DeleteAllLinksAndNodes();
                flow::DiscardChanges();
                flow::SetCurrentFilename("");
            }

            globalAction = GlobalActions::None;

            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Discard"))
        {
            globalAction = GlobalActions::None;
            nm::DeleteAllLinksAndNodes();
            flow::DiscardChanges();
            flow::SetCurrentFilename("");
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            globalAction = GlobalActions::None;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void NAV::gui::NodeEditorApplication::ShowLoadRequested()
{
    if (flow::HasUnsavedChanges())
    {
        ImGui::OpenPopup("Discard unsaved changes?");
        if (ImGui::BeginPopupModal("Discard unsaved changes?", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Do you want to save your changes or discard them before loading?");
            if (ImGui::Button("Save"))
            {
                if (flow::GetCurrentFilename().empty())
                {
                    ImGuiFileDialog::Instance()->OpenModal("Save Flow##Load", "Save Flow", ".flow", (flow::GetFlowPath() / ".").string(), 1, nullptr, ImGuiFileDialogFlags_ConfirmOverwrite);
                    ImGuiFileDialog::Instance()->SetFileStyle(IGFD_FileStyleByExtention, ".flow", ImVec4(0.0F, 1.0F, 0.0F, 0.9F));
                }
                else
                {
                    flow::SaveFlowAs(flow::GetCurrentFilename());
                    flow::DiscardChanges();
                    ImGui::CloseCurrentPopup();
                }
            }

            if (ImGuiFileDialog::Instance()->Display("Save Flow##Load", ImGuiWindowFlags_NoCollapse, ImVec2(600, 400)))
            {
                if (ImGuiFileDialog::Instance()->IsOk())
                {
                    flow::SetCurrentFilename(ImGuiFileDialog::Instance()->GetFilePathName());
                    flow::SaveFlowAs(flow::GetCurrentFilename());
                }

                flow::DiscardChanges();
                ImGuiFileDialog::Instance()->Close();
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Discard"))
            {
                flow::DiscardChanges();
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
            {
                globalAction = GlobalActions::None;
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
    }
    else
    {
        ImGuiFileDialog::Instance()->OpenDialog("Load Flow", "Load Flow", ".flow", (flow::GetFlowPath() / ".").string(), 1, nullptr);
        ImGuiFileDialog::Instance()->SetFileStyle(IGFD_FileStyleByExtention, ".flow", ImVec4(0.0F, 1.0F, 0.0F, 0.9F));

        static bool loadSuccessful = true;

        auto& io = ImGui::GetIO();
        if (!io.KeyCtrl && !io.KeyAlt && !io.KeyShift && !io.KeySuper)
        {
            if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Escape)))
            {
                globalAction = GlobalActions::None;
                loadSuccessful = true;
                ImGuiFileDialog::Instance()->Close();
                return;
            }
        }

        if (ImGuiFileDialog::Instance()->Display("Load Flow", ImGuiWindowFlags_NoCollapse, ImVec2(600, 400)))
        {
            if (ImGuiFileDialog::Instance()->IsOk())
            {
                loadSuccessful = flow::LoadFlow(ImGuiFileDialog::Instance()->GetFilePathName());
                if (loadSuccessful)
                {
                    globalAction = GlobalActions::None;
                    frameCountNavigate = ImGui::GetFrameCount();
                    ImGuiFileDialog::Instance()->Close();
                }
            }
            else
            {
                globalAction = GlobalActions::None;
                ImGuiFileDialog::Instance()->Close();
            }
        }
        if (!loadSuccessful)
        {
            ImGui::OpenPopup("Could not open file");
        }

        if (ImGui::BeginPopupModal("Could not open file", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoResize))
        {
            ImGui::Text("The filename specified is invalid\nor the program has insufficient\npermissions to access the file");
            if (ImGui::Button("Close"))
            {
                loadSuccessful = true;
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
    }
}

void NAV::gui::NodeEditorApplication::ShowRenameNodeRequest(Node*& renameNode)
{
    const char* title = renameNode->kind == Node::Kind::GroupBox ? "Rename Group Box" : "Rename Node";
    ImGui::OpenPopup(title);
    if (ImGui::BeginPopupModal(title, nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        static std::string nameBackup = renameNode->name;
        if (nameBackup.empty())
        {
            nameBackup = renameNode->name;
        }

        auto& io = ImGui::GetIO();
        if (!io.KeyCtrl && !io.KeyAlt && !io.KeyShift && !io.KeySuper)
        {
            if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Escape)))
            {
                if (renameNode)
                {
                    renameNode->name = nameBackup;
                }
                nameBackup.clear();
                renameNode = nullptr;
                ImGui::CloseCurrentPopup();
                ImGui::EndPopup();
                return;
            }
        }

        if (ImGui::InputTextMultiline(fmt::format("##{}", size_t(renameNode->id)).c_str(), &renameNode->name, ImVec2(0, 65), ImGuiInputTextFlags_CtrlEnterForNewLine | ImGuiInputTextFlags_EnterReturnsTrue))
        {
            nameBackup.clear();
            renameNode = nullptr;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("Hold SHIFT or use mouse to select text.\n"
                                 "CTRL+Left/Right to word jump.\n"
                                 "CTRL+A or double-click to select all.\n"
                                 "CTRL+X,CTRL+C,CTRL+V clipboard.\n"
                                 "CTRL+Z,CTRL+Y undo/redo.\n"
                                 "ESCAPE to revert.");
        if (ImGui::Button("Accept"))
        {
            nameBackup.clear();
            renameNode = nullptr;
            flow::ApplyChanges();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            if (renameNode)
            {
                renameNode->name = nameBackup;
            }
            nameBackup.clear();
            renameNode = nullptr;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void NAV::gui::NodeEditorApplication::ShowRenamePinRequest(Pin*& renamePin)
{
    const char* title = "Rename Pin";
    ImGui::OpenPopup(title);
    if (ImGui::BeginPopupModal(title, nullptr, ImGuiWindowFlags_AlwaysAutoResize))
    {
        static std::string nameBackup = renamePin->name;
        if (nameBackup.empty())
        {
            nameBackup = renamePin->name;
        }

        auto& io = ImGui::GetIO();
        if (!io.KeyCtrl && !io.KeyAlt && !io.KeyShift && !io.KeySuper)
        {
            if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Escape)))
            {
                if (renamePin)
                {
                    renamePin->name = nameBackup;
                }
                nameBackup.clear();
                renamePin = nullptr;
                ImGui::CloseCurrentPopup();
                ImGui::EndPopup();
                return;
            }
        }

        if (ImGui::InputTextMultiline(fmt::format("##{}", size_t(renamePin->id)).c_str(), &renamePin->name, ImVec2(0, 65), ImGuiInputTextFlags_CtrlEnterForNewLine | ImGuiInputTextFlags_EnterReturnsTrue))
        {
            nameBackup.clear();
            renamePin = nullptr;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        gui::widgets::HelpMarker("Hold SHIFT or use mouse to select text.\n"
                                 "CTRL+Left/Right to word jump.\n"
                                 "CTRL+A or double-click to select all.\n"
                                 "CTRL+X,CTRL+C,CTRL+V clipboard.\n"
                                 "CTRL+Z,CTRL+Y undo/redo.\n"
                                 "ESCAPE to revert.");
        if (ImGui::Button("Accept"))
        {
            nameBackup.clear();
            renamePin = nullptr;
            flow::ApplyChanges();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel"))
        {
            if (renamePin)
            {
                renamePin->name = nameBackup;
            }
            nameBackup.clear();
            renamePin = nullptr;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void NAV::gui::NodeEditorApplication::OnFrame(float deltaTime)
{
    bool firstFrame = ImGui::GetFrameCount() == 1;

    if (frameCountNavigate && ImGui::GetFrameCount() - frameCountNavigate > 3)
    {
        frameCountNavigate = 0;
        ed::NavigateToContent();
    }

    switch (globalAction)
    {
    case GlobalActions::Quit:
        Quit();
        break;
    case GlobalActions::QuitUnsaved:
        ShowQuitRequested();
        break;
    case GlobalActions::SaveAs:
        ShowSaveAsRequested();
        break;
    case GlobalActions::Clear:
        ShowClearNodesRequested();
        break;
    case GlobalActions::Load:
        ShowLoadRequested();
        break;
    default:
        break;
    }

    if (!initList.empty()) // Start thread to (De-)/Initialize Nodes
    {
        Node* node = initList.front().first;
        bool init = initList.front().second;
        if ((init && !node->isInitializing())        // Finished with init (success or fail)
            || (!init && !node->isDeinitializing())) // Finished with deinit
        {
            initList.pop_front();
            currentInitNodeId = 0;
        }
        else if (currentInitNodeId == 0) // Currently no thread running
        {
            currentInitNodeId = size_t(node->id);
            if (initThread.joinable())
            {
                // initThread.request_stop();
                // initThread_stopRequested = true;
                initThread.join();
            }
            // initThread = std::jthread([node, init]() {
            initThread = std::thread([node, init]() {
                if (init)
                {
                    node->_isInitializing = false;
                    node->initializeNode();
                }
                else
                {
                    node->_isDeinitializing = false;
                    node->deinitializeNode();
                }
            });
        }
    }

    gui::UpdateTouch(deltaTime);

    if (ed::AreShortcutsEnabled())
    {
        gui::checkShortcuts(globalAction);
    }

    gui::menus::ShowMainMenuBar(globalAction, initList);
    menuBarHeight = ImGui::GetCursorPosY();

    ed::SetCurrentEditor(m_Editor);

    static ed::NodeId contextNodeId = 0;
    static ed::LinkId contextLinkId = 0;
    static ed::PinId contextPinId = 0;
    static bool createNewNode = false;
    static Pin* newNodeLinkPin = nullptr;

    gui::widgets::Splitter("Main Splitter", true, SPLITTER_THICKNESS, &leftPaneWidth, &rightPaneWidth, 50.0F, 50.0F);

    bool leftPaneActive = gui::panels::ShowLeftPane(leftPaneWidth - SPLITTER_THICKNESS);

    ImGui::SameLine(0.0F, 12.0F);

    // ToolTips have to be shown outside of the NodeEditor Context, so save the Tooltip and push it afterwards
    std::string tooltipText;

    ImGui::BeginGroup();

    if (bottomViewSelectedTab != BottomViewTabItem::None)
    {
        float blueprintHeight = ImGui::GetContentRegionAvail().y - bottomViewHeight + 28.5F;
        ImGui::PushStyleColor(ImGuiCol_Separator, IM_COL32_BLACK_TRANS);
        gui::widgets::Splitter("Log Splitter", false, 6.0F, &blueprintHeight, &bottomViewHeight, 400.0F, BOTTOM_VIEW_UNCOLLAPSED_MIN_HEIGHT);
        ImGui::PopStyleColor();
    }

    ed::Begin("Node editor", ImVec2(0, ImGui::GetContentRegionAvail().y - bottomViewHeight + SPLITTER_THICKNESS));
    {
        static Pin* newLinkPin = nullptr;

        auto cursorTopLeft = ImGui::GetCursorScreenPos();

        static util::BlueprintNodeBuilder builder(m_HeaderBackground, GetTextureWidth(m_HeaderBackground), GetTextureHeight(m_HeaderBackground));

        for (const auto& node : nm::m_Nodes()) // Blueprint || Simple
        {
            if (node->kind != Node::Kind::Blueprint && node->kind != Node::Kind::Simple) // NOLINT(misc-redundant-expression) // FIXME: error: equivalent expression on both sides of logical operator
            {
                continue;
            }

            const auto isSimple = node->kind == Node::Kind::Simple;

            bool hasOutputDelegates = false;
            bool hasOutputFlows = false;
            for (const auto& output : node->outputPins)
            {
                if (output.type == Pin::Type::Delegate)
                {
                    hasOutputDelegates = true;
                }
                else if (output.type == Pin::Type::Flow)
                {
                    hasOutputFlows = true;
                }
            }

            builder.Begin(node->id);

            if (!isSimple) // Header Text for Blueprint Nodes
            {
                if (!node->isEnabled()) // Node disabled
                {
                    builder.Header(ImColor(192, 192, 192)); // Silver
                }
                else if (node->isInitialized())
                {
                    builder.Header(ImColor(128, 255, 128)); // Light green
                }
                else
                {
                    builder.Header(ImColor(255, 128, 128)); // Light red
                }
                ImGui::Spring(0);
                ImGui::TextUnformatted(node->name.c_str());
                ImGui::Spring(1);
                if (node->isInitializing())
                {
                    gui::widgets::Spinner(("##Spinner " + node->nameId()).c_str(), ImColor(144, 238, 144), 10.0F, 1.0F);
                }
                else if (node->isDeinitializing())
                {
                    gui::widgets::Spinner(("##Spinner " + node->nameId()).c_str(), ImColor(255, 160, 122), 10.0F, 1.0F);
                }
                else if (hasOutputFlows)
                {
                    bool itemDisabled = false;
                    if (!node->isInitialized() && !node->callbacksEnabled)
                    {
                        ImGui::PushDisabled();
                        itemDisabled = true;
                    }

                    ImGui::Checkbox("", &node->callbacksEnabled);
                    if (ImGui::IsItemHovered())
                    {
                        ed::Suspend();
                        ImGui::SetTooltip("Enable Callbacks");
                        ed::Resume();
                    }

                    if (itemDisabled)
                    {
                        ImGui::PopDisabled();
                    }
                    ImGui::Dummy(ImVec2(0, 26));
                }
                else
                {
                    ImGui::Dummy(ImVec2(0, 26));
                }
                if (hasOutputDelegates)
                {
                    ImGui::BeginVertical("delegates", ImVec2(0, 26));
                    ImGui::Spring(1, 0);
                    for (const auto& output : node->outputPins)
                    {
                        if (output.type != Pin::Type::Delegate)
                        {
                            continue;
                        }

                        auto alpha = ImGui::GetStyle().Alpha;
                        if (newLinkPin && !newLinkPin->canCreateLink(output) && &output != newLinkPin)
                        {
                            alpha = alpha * (48.0F / 255.0F);
                        }

                        ed::BeginPin(output.id, ed::PinKind::Output);
                        ed::PinPivotAlignment(ImVec2(1.0F, 0.5F));
                        ed::PinPivotSize(ImVec2(0, 0));
                        ImGui::BeginHorizontal(output.id.AsPointer());
                        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, alpha);
                        if (!output.name.empty())
                        {
                            ImGui::TextUnformatted(output.name.c_str());
                            ImGui::Spring(0);
                        }
                        output.drawPinIcon(nm::IsPinLinked(output.id), static_cast<int>(alpha * 255));
                        ImGui::Spring(0, ImGui::GetStyle().ItemSpacing.x / 2);
                        ImGui::EndHorizontal();
                        ImGui::PopStyleVar();
                        ed::EndPin();

                        // DrawItemRect(ImColor(255, 0, 0));
                    }
                    ImGui::Spring(1, 0);
                    ImGui::EndVertical();
                    ImGui::Spring(0, ImGui::GetStyle().ItemSpacing.x / 2);
                }
                else
                {
                    ImGui::Spring(0);
                }
                builder.EndHeader();
            }

            for (const auto& input : node->inputPins) // Input Pins
            {
                auto alpha = ImGui::GetStyle().Alpha;
                if (newLinkPin && !newLinkPin->canCreateLink(input) && &input != newLinkPin)
                {
                    alpha = alpha * (48.0F / 255.0F);
                }

                builder.Input(input.id);
                ImGui::PushStyleVar(ImGuiStyleVar_Alpha, alpha);
                input.drawPinIcon(nm::IsPinLinked(input.id), static_cast<int>(alpha * 255));
                if (ImGui::IsItemHovered())
                {
                    tooltipText = fmt::format("{}", fmt::join(input.dataIdentifier, "\n"));
                }

                ImGui::Spring(0);
                if (!input.name.empty())
                {
                    ImGui::TextUnformatted(input.name.c_str());
                    ImGui::Spring(0);
                }
                ImGui::PopStyleVar();
                util::BlueprintNodeBuilder::EndInput();
            }

            if (isSimple) // Middle Text for Simple Nodes
            {
                builder.Middle();

                ImGui::Spring(1, 0);
                ImGui::TextUnformatted(node->name.c_str());
                ImGui::Spring(1, 0);
            }

            for (const auto& output : node->outputPins) // Output Pins
            {
                if (!isSimple && output.type == Pin::Type::Delegate)
                {
                    continue;
                }

                auto alpha = ImGui::GetStyle().Alpha;
                if (newLinkPin && !newLinkPin->canCreateLink(output) && &output != newLinkPin)
                {
                    alpha = alpha * (48.0F / 255.0F);
                }

                ImGui::PushStyleVar(ImGuiStyleVar_Alpha, alpha);
                builder.Output(output.id);
                if (!output.name.empty())
                {
                    ImGui::Spring(0);
                    ImGui::TextUnformatted(output.name.c_str());
                }
                ImGui::Spring(0);
                output.drawPinIcon(nm::IsPinLinked(output.id), static_cast<int>(alpha * 255));
                if (ImGui::IsItemHovered())
                {
                    tooltipText = fmt::format("{}", fmt::join(output.dataIdentifier, "\n"));
                }
                ImGui::PopStyleVar();
                util::BlueprintNodeBuilder::EndOutput();
            }

            builder.End();
        }

        for (const auto& node : nm::m_Nodes()) // GroupBox
        {
            if (node->kind != Node::Kind::GroupBox)
            {
                continue;
            }

            const float commentAlpha = 0.75F;

            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, commentAlpha);
            ed::PushStyleColor(ed::StyleColor_NodeBg, ImColor(255, 255, 255, 64));
            ed::PushStyleColor(ed::StyleColor_NodeBorder, ImColor(255, 255, 255, 64));
            ed::BeginNode(node->id);
            ImGui::PushID(node->id.AsPointer());
            ImGui::BeginVertical("content");
            ImGui::BeginHorizontal("horizontal");
            ImGui::Spring(1);
            ImGui::TextUnformatted(node->name.c_str());
            ImGui::Spring(1);
            ImGui::EndHorizontal();
            ed::Group(node->_size);
            ImGui::EndVertical();
            ImGui::PopID();
            ed::EndNode();
            ed::PopStyleColor(2);
            ImGui::PopStyleVar();

            if (ed::BeginGroupHint(node->id))
            {
                // auto alpha   = static_cast<int>(commentAlpha * ImGui::GetStyle().Alpha * 255);
                auto bgAlpha = static_cast<int>(ImGui::GetStyle().Alpha * 255);

                // ImGui::PushStyleVar(ImGuiStyleVar_Alpha, commentAlpha * ImGui::GetStyle().Alpha);

                auto min = ed::GetGroupMin();
                // auto max = ed::GetGroupMax();

                ImGui::SetCursorScreenPos(min - ImVec2(-8, ImGui::GetTextLineHeightWithSpacing() + 4));
                ImGui::BeginGroup();
                ImGui::TextUnformatted(node->name.c_str());
                ImGui::EndGroup();

                auto* drawList = ed::GetHintBackgroundDrawList();

                auto ImRect_Expanded = [](const ImRect& rect, float x, float y) -> ImRect {
                    auto result = rect;
                    result.Min.x -= x;
                    result.Min.y -= y;
                    result.Max.x += x;
                    result.Max.y += y;
                    return result;
                };

                auto hintBounds = ImRect(ImGui::GetItemRectMin(), ImGui::GetItemRectMax());
                auto hintFrameBounds = ImRect_Expanded(hintBounds, 8, 4);

                drawList->AddRectFilled(
                    hintFrameBounds.GetTL(),
                    hintFrameBounds.GetBR(),
                    IM_COL32(255, 255, 255, 64 * bgAlpha / 255), 4.0F);

                drawList->AddRect(
                    hintFrameBounds.GetTL(),
                    hintFrameBounds.GetBR(),
                    IM_COL32(255, 255, 255, 128 * bgAlpha / 255), 4.0F);

                // ImGui::PopStyleVar();
            }
            ed::EndGroupHint();
        }

        for (const auto& link : nm::m_Links()) // Links
        {
            ed::Link(link.id, link.startPinId, link.endPinId, link.color, 2.0F);
        }

        if (!createNewNode)
        {
            if (ed::BeginCreate(ImColor(255, 255, 255), 2.0F))
            {
                auto showLabel = [](const char* label, ImColor color) {
                    ImGui::SetCursorPosY(ImGui::GetCursorPosY() - ImGui::GetTextLineHeight());
                    auto size = ImGui::CalcTextSize(label);

                    auto padding = ImGui::GetStyle().FramePadding;
                    auto spacing = ImGui::GetStyle().ItemSpacing;

                    ImGui::SetCursorPos(ImGui::GetCursorPos() + ImVec2(spacing.x, -spacing.y));

                    auto rectMin = ImGui::GetCursorScreenPos() - padding;
                    auto rectMax = ImGui::GetCursorScreenPos() + size + padding;

                    auto* drawList = ImGui::GetWindowDrawList();
                    drawList->AddRectFilled(rectMin, rectMax, color, size.y * 0.15F);
                    ImGui::TextUnformatted(label);
                };

                ed::PinId startPinId = 0;
                ed::PinId endPinId = 0;
                if (ed::QueryNewLink(&startPinId, &endPinId))
                {
                    auto* startPin = nm::FindPin(startPinId);
                    auto* endPin = nm::FindPin(endPinId);

                    newLinkPin = startPin ? startPin : endPin;

                    if (startPin && startPin->kind == Pin::Kind::Input)
                    {
                        std::swap(startPin, endPin);
                        std::swap(startPinId, endPinId);
                    }

                    if (startPin && endPin)
                    {
                        if (endPin == startPin)
                        {
                            ed::RejectNewItem(ImColor(255, 0, 0), 2.0F);
                        }
                        else if (endPin->kind == startPin->kind)
                        {
                            showLabel("x Incompatible Pin Kind", ImColor(45, 32, 32, 180));
                            ed::RejectNewItem(ImColor(255, 0, 0), 2.0F);
                        }
                        else if (endPin->parentNode == startPin->parentNode)
                        {
                            showLabel("x Cannot connect to self", ImColor(45, 32, 32, 180));
                            ed::RejectNewItem(ImColor(255, 0, 0), 1.0F);
                        }
                        else if (endPin->type != startPin->type)
                        {
                            showLabel("x Incompatible Pin Type", ImColor(45, 32, 32, 180));
                            ed::RejectNewItem(ImColor(255, 128, 128), 1.0F);
                        }
                        else if (nm::IsPinLinked(endPin->id))
                        {
                            showLabel("End Pin already linked", ImColor(45, 32, 32, 180));
                            ed::RejectNewItem(ImColor(255, 128, 128), 1.0F);
                        }
                        else if (startPin->type == Pin::Type::Flow
                                 && !NAV::NodeRegistry::NodeDataTypeAnyIsChildOf(startPin->dataIdentifier, endPin->dataIdentifier))
                        {
                            showLabel(fmt::format("The data type [{}]\ncan't be linked to [{}]",
                                                  fmt::join(startPin->dataIdentifier, ","),
                                                  fmt::join(endPin->dataIdentifier, ","))
                                          .c_str(),
                                      ImColor(45, 32, 32, 180));
                            ed::RejectNewItem(ImColor(255, 128, 128), 1.0F);
                        }
                        else if (startPin->type == Pin::Type::Delegate
                                 && (startPin->parentNode == nullptr
                                     || std::find(endPin->dataIdentifier.begin(), endPin->dataIdentifier.end(), startPin->parentNode->type()) == endPin->dataIdentifier.end()))
                        {
                            if (startPin->parentNode != nullptr)
                            {
                                showLabel(fmt::format("The delegate type [{}]\ncan't be linked to [{}]",
                                                      startPin->parentNode->type(),
                                                      fmt::join(endPin->dataIdentifier, ","))
                                              .c_str(),
                                          ImColor(45, 32, 32, 180));
                            }

                            ed::RejectNewItem(ImColor(255, 128, 128), 1.0F);
                        }
                        else if ((startPin->type == Pin::Type::Object || startPin->type == Pin::Type::Matrix) // NOLINT(misc-redundant-expression) // FIXME: error: equivalent expression on both sides of logical operator
                                 && !Pin::dataIdentifierHaveCommon(startPin->dataIdentifier, endPin->dataIdentifier))
                        {
                            showLabel(fmt::format("The data type [{}]\ncan't be linked to [{}]",
                                                  fmt::join(startPin->dataIdentifier, ","),
                                                  fmt::join(endPin->dataIdentifier, ","))
                                          .c_str(),
                                      ImColor(45, 32, 32, 180));
                            ed::RejectNewItem(ImColor(255, 128, 128), 1.0F);
                        }
                        else
                        {
                            showLabel("+ Create Link", ImColor(32, 45, 32, 180));
                            if (ed::AcceptNewItem(ImColor(128, 255, 128), 4.0F))
                            {
                                nm::CreateLink(startPin, endPin);
                            }
                        }
                    }
                }

                ed::PinId pinId = 0;
                if (ed::QueryNewNode(&pinId))
                {
                    newLinkPin = nm::FindPin(pinId);
                    if (newLinkPin->kind == Pin::Kind::Input && nm::IsPinLinked(newLinkPin->id))
                    {
                        showLabel("End Pin already linked", ImColor(45, 32, 32, 180));
                        ed::RejectNewItem(ImColor(255, 128, 128), 1.0F);
                    }
                    else
                    {
                        if (newLinkPin)
                        {
                            showLabel("+ Create Node", ImColor(32, 45, 32, 180));
                        }

                        if (ed::AcceptNewItem())
                        {
                            createNewNode = true;
                            newNodeLinkPin = nm::FindPin(pinId);
                            newLinkPin = nullptr;
                            ed::Suspend();
                            ImGui::OpenPopup("Create New Node");
                            ed::Resume();
                        }
                    }
                }
            }
            else
            {
                newLinkPin = nullptr;
            }

            ed::EndCreate();

            if (ed::BeginDelete())
            {
                ed::LinkId linkId = 0;
                while (ed::QueryDeletedLink(&linkId))
                {
                    if (ed::AcceptDeletedItem())
                    {
                        nm::DeleteLink(linkId);
                    }
                }

                ed::NodeId nodeId = 0;
                while (ed::QueryDeletedNode(&nodeId))
                {
                    if (Node* node = nm::FindNode(nodeId))
                    {
                        if (node->isInitializing() || node->isDeinitializing())
                        {
                            break;
                        }
                    }
                    if (ed::AcceptDeletedItem())
                    {
                        nm::DeleteNode(nodeId);
                    }
                }
            }
            ed::EndDelete();
        }

        ImGui::SetCursorScreenPos(cursorTopLeft);
    }

    // Shortcut enable/disable
    ax::NodeEditor::EnableShortcuts(ed::IsActive() || leftPaneActive);

    // auto openPopupPosition = ImGui::GetMousePos();
    ed::Suspend();
    if (ed::ShowNodeContextMenu(&contextNodeId))
    {
        ImGui::OpenPopup("Node Context Menu");
    }
    else if (ed::ShowPinContextMenu(&contextPinId))
    {
        ImGui::OpenPopup("Pin Context Menu");
    }
    else if (ed::ShowLinkContextMenu(&contextLinkId) && ed::IsActive())
    {
        ImGui::OpenPopup("Link Context Menu");
    }
    else if (ed::ShowBackgroundContextMenu() && ed::IsActive())
    {
        ImGui::OpenPopup("Create New Node");
        newNodeLinkPin = nullptr;
    }
    else if (ed::NodeId doubleClickedNodeId = ed::GetDoubleClickedNode())
    {
        Node* node = nm::FindNode(doubleClickedNodeId);
        node->_showConfig = true;
    }
    ed::Resume();

    ed::Suspend();
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 8));
    static Node* renameNode = nullptr;
    static Pin* renamePin = nullptr;
    if (ImGui::BeginPopup("Node Context Menu"))
    {
        auto* node = nm::FindNode(contextNodeId);

        ImGui::TextUnformatted("Node Context Menu");
        ImGui::Separator();
        if (node)
        {
            ImGui::Text("ID: %lu", size_t(node->id));
            ImGui::Text("Type: %s", node->type().c_str());
            ImGui::Text("Kind: %s", std::string(node->kind).c_str());
            ImGui::Text("Inputs: %lu", node->inputPins.size());
            ImGui::Text("Outputs: %lu", node->outputPins.size());
            ImGui::Separator();
            if (ImGui::MenuItem(node->isInitialized() ? "Reinitialize" : "Initialize", "", false, node->isEnabled() && !node->isInitializing() && !node->isDeinitializing()))
            {
                if (node->isInitialized())
                {
                    node->_isDeinitializing = true;
                    initList.emplace_back(node, false);
                }
                node->_isInitializing = true;
                initList.emplace_back(node, true);
            }
            if (ImGui::MenuItem("Deinitialize", "", false, node->isEnabled() && node->isInitialized() && !node->isInitializing() && !node->isDeinitializing()))
            {
                node->_isDeinitializing = true;
                initList.emplace_back(node, false);
            }
            ImGui::Separator();
            if (node->_hasConfig && ImGui::MenuItem("Configure", "", false))
            {
                node->_showConfig = true;
            }
            if (ImGui::MenuItem(node->isEnabled() ? "Disable" : "Enable", "", false))
            {
                if (node->isEnabled())
                {
                    if (node->isInitialized())
                    {
                        node->_isDeinitializing = true;
                        initList.emplace_back(node, false);
                    }
                    node->_isEnabled = false;
                }
                else
                {
                    node->_isEnabled = true;
                }
                flow::ApplyChanges();
            }
            if (ImGui::MenuItem("Rename"))
            {
                renameNode = node;
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Delete", "", false, !node->isInitializing() && !node->isDeinitializing()))
            {
                ed::DeleteNode(contextNodeId);
            }
        }
        else
        {
            ImGui::Text("Unknown node: %lu", size_t(contextNodeId));
        }

        ImGui::EndPopup();
    }

    if (renameNode) // Popup for renaming a node
    {
        ShowRenameNodeRequest(renameNode);
    }

    if (ImGui::BeginPopup("Pin Context Menu"))
    {
        auto* pin = nm::FindPin(contextPinId);

        ImGui::TextUnformatted("Pin Context Menu");
        ImGui::Separator();
        if (pin)
        {
            ImGui::Text("ID: %lu", size_t(pin->id));
            ImGui::Text("Node: %s", pin->parentNode ? std::to_string(size_t(pin->parentNode->id)).c_str() : "<none>");
            ImGui::Text("Type: %s", std::string(pin->type).c_str());
            ImGui::Separator();
            if (ImGui::MenuItem("Rename"))
            {
                renamePin = pin;
            }
            if (!pin->callbacks.empty())
            {
                ImGui::Separator();
                ImGui::Text("Callbacks:");
                ImGui::Indent();
                for (auto& callback : pin->callbacks)
                {
                    ImGui::BulletText("%s", std::get<0>(callback)->nameId().c_str());
                }
                ImGui::Unindent();
            }
        }
        else
        {
            ImGui::Text("Unknown pin: %lu", size_t(contextPinId));
        }

        ImGui::EndPopup();
    }

    if (renamePin) // Popup for renaming a pin
    {
        ShowRenamePinRequest(renamePin);
    }

    if (ImGui::BeginPopup("Link Context Menu"))
    {
        auto* link = nm::FindLink(contextLinkId);

        ImGui::TextUnformatted("Link Context Menu");
        ImGui::Separator();
        if (link)
        {
            ImGui::Text("ID: %lu", size_t(link->id));
            ImGui::Text("From: %lu", size_t(link->startPinId));
            ImGui::Text("To: %lu", size_t(link->endPinId));
        }
        else
        {
            ImGui::Text("Unknown link: %lu", size_t(contextLinkId));
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Delete"))
        {
            ed::DeleteLink(contextLinkId);
        }
        ImGui::EndPopup();
    }

    static bool setKeyboardFocus = true;
    static ImVec2 newNodeSpawnPos{ -1, -1 };
    if (ImGui::BeginPopup("Create New Node"))
    {
        if (newNodeSpawnPos.x == -1 || newNodeSpawnPos.y == -1)
        {
            auto viewRect = reinterpret_cast<ax::NodeEditor::Detail::EditorContext*>(ed::GetCurrentEditor())->GetViewRect();
            newNodeSpawnPos = ImGui::GetMousePos();
            newNodeSpawnPos.x -= leftPaneWidth + SPLITTER_THICKNESS + 10.0F;
            newNodeSpawnPos.y -= menuBarHeight;

            newNodeSpawnPos *= ed::GetCurrentZoom();

            newNodeSpawnPos += viewRect.GetTL();

            LOG_DEBUG("New Node will spawn at {}x{} - Zoom {}", newNodeSpawnPos.x, newNodeSpawnPos.y, ed::GetCurrentZoom());
        }

        static ImGuiTextFilter filter;

        filter.Draw("");

        if (setKeyboardFocus)
        {
            filter.Clear();
            setKeyboardFocus = false;
            ImGui::SetKeyboardFocusHere(0);
        }

        Node* node = nullptr;
        for (const auto& [category, nodeInfoList] : NAV::NodeRegistry::RegisteredNodes())
        {
            // Prevent category from showing, if it is empty
            bool categoryHasItems = false;
            for (const auto& nodeInfo : nodeInfoList)
            {
                if (nodeInfo.hasCompatiblePin(newNodeLinkPin)
                    && (filter.PassFilter(nodeInfo.type.c_str()) || filter.PassFilter(category.c_str())))
                {
                    categoryHasItems = true;
                    break;
                }
            }
            if (categoryHasItems)
            {
                ImGui::SetNextItemOpen(true, ImGuiCond_Once);
                if (ImGui::TreeNode((category + "##NewNodeTree").c_str()))
                {
                    for (const auto& nodeInfo : nodeInfoList)
                    {
                        const auto& displayName = nodeInfo.type;
                        const auto& constructor = nodeInfo.constructor;
                        ImGui::Indent();
                        if (nodeInfo.hasCompatiblePin(newNodeLinkPin)
                            && (filter.PassFilter(nodeInfo.type.c_str()) || filter.PassFilter(category.c_str()))
                            && ImGui::MenuItem(displayName.c_str()))
                        {
                            filter.Clear();
                            node = constructor();
                            nm::AddNode(node);
                        }
                        ImGui::Unindent();
                    }
                    ImGui::TreePop();
                }
            }
        }

        if (node)
        {
            createNewNode = false;

            ed::SetNodePosition(node->id, newNodeSpawnPos);
            newNodeSpawnPos = { -1, -1 };

            if (auto* startPin = newNodeLinkPin)
            {
                auto& pins = startPin->kind == Pin::Kind::Input ? node->outputPins : node->inputPins;

                for (auto& pin : pins)
                {
                    if (startPin->canCreateLink(pin))
                    {
                        auto* endPin = &pin;
                        if (startPin->kind == Pin::Kind::Input)
                        {
                            std::swap(startPin, endPin);
                        }

                        nm::CreateLink(startPin, endPin);

                        break;
                    }
                }
            }
        }

        ImGui::EndPopup();
    }
    else
    {
        setKeyboardFocus = true;
        createNewNode = false;
        newNodeSpawnPos = { -1, -1 };
    }
    ImGui::PopStyleVar();

    for (const auto& node : nm::m_Nodes()) // Config Windows for nodes
    {
        if (node->_hasConfig && node->_showConfig)
        {
            ImVec2 center(ImGui::GetIO().DisplaySize.x * 0.5F, ImGui::GetIO().DisplaySize.y * 0.5F);
            ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5F, 0.5F));
            ImGui::SetNextWindowSize(node->_guiConfigDefaultWindowSize, ImGuiCond_FirstUseEver);
            if (ImGui::Begin(fmt::format("{} ({})", node->nameId(), node->type()).c_str(), &(node->_showConfig),
                             ImGuiWindowFlags_None))
            {
                node->guiConfig();
            }
            else // Window is collapsed
            {
                if (ImGui::IsWindowFocused())
                {
                    ed::EnableShortcuts(true);
                }
            }

            ImGui::End();
        }
    }
    ed::Resume();

    ed::End();

    ImGui::Indent(SPLITTER_THICKNESS);
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    if (ImGui::BeginTabBar("BottomViewTabBar"))
    {
        bool noItemSelected = bottomViewSelectedTab == BottomViewTabItem::None;
        if (noItemSelected)
        {
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.0F);
        }
        if (ImGui::BeginTabItem("▼"))
        {
            bottomViewSelectedTab = BottomViewTabItem::None;
            bottomViewHeight = BOTTOM_VIEW_COLLAPSED_MIN_HEIGHT;
            ImGui::EndTabItem();
        }
        else
        {
            bottomViewHeight = std::max(bottomViewHeight, BOTTOM_VIEW_UNCOLLAPSED_MIN_HEIGHT);
        }
        if (noItemSelected)
        {
            ImGui::PopStyleVar();
        }

        if (ImGui::BeginTabItem("Log Output", nullptr, firstFrame ? ImGuiTabItemFlags_SetSelected : ImGuiTabItemFlags_None))
        {
            bottomViewSelectedTab = BottomViewTabItem::LogOutput;

            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::Unindent();

    ImGui::EndGroup();

    FlowAnimation::ProcessQueue();

    // Push the Tooltip on the stack if needed
    if (!tooltipText.empty())
    {
        ImGui::SetTooltip("%s", tooltipText.c_str());
    }

    gui::windows::renderGlobalWindows();

    std::string title = (flow::HasUnsavedChanges() ? "● " : "")
                        + (flow::GetCurrentFilename().empty() ? "" : flow::GetCurrentFilename() + " - ")
                        + "INSTINCT";
    SetTitle(title.c_str());
}
