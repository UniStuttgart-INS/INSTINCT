#include "RunMenu.hpp"

#include <imgui.h>

#include "internal/FlowExecutor.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "util/Logger.hpp"

void NAV::gui::menus::ShowRunMenu()
{
    bool hasInitializedNodes = false;
    bool allNodesInitialized = true;
    for (const auto* node : nm::m_Nodes())
    {
        if (node->isInitialized())
        {
            hasInitializedNodes = true;
        }
        else if (node->isEnabled())
        {
            allNodesInitialized = false;
        }
    }
    if (ImGui::MenuItem("Initialize all Nodes", nullptr, false, !allNodesInitialized))
    {
        // TODO
        // for (auto* node : nm::m_Nodes())
        // {
        //     if (node->isEnabled() && !node->isInitialized())
        //     {
        //         node->_state = Node::State::DoInitialize;
        //         initList.emplace_back(node, true);
        //     }
        // }
    }
    if (ImGui::MenuItem("Reinitialize all Nodes", nullptr, false, hasInitializedNodes))
    {
        // TODO
        // for (auto* node : nm::m_Nodes())
        // {
        //     if (node->isEnabled() && node->isInitialized())
        //     {
        //         node->_state = Node::State::DoDeinitialize;
        //         initList.emplace_back(node, false);
        //     }
        // }
        // for (auto* node : nm::m_Nodes())
        // {
        //     if (node->isEnabled())
        //     {
        //         initList.emplace_back(node, true);
        //     }
        // }
    }
    if (ImGui::MenuItem("Deinitialize all Nodes", nullptr, false, hasInitializedNodes))
    {
        // TODO
        // for (auto* node : nm::m_Nodes())
        // {
        //     if (node->isEnabled() && node->isInitialized())
        //     {
        //         node->_state = Node::State::DoDeinitialize;
        //         initList.emplace_back(node, false);
        //     }
        // }
    }

    ImGui::Separator();

    if (ImGui::MenuItem("Run Flow", "F7", false, !FlowExecutor::isRunning()))
    {
        if (!FlowExecutor::isRunning())
        {
            LOG_INFO("Starting Flow Execution");
            FlowExecutor::start();
        }
    }
    if (ImGui::MenuItem("Stop Execution", "Esc", false, FlowExecutor::isRunning()))
    {
        if (FlowExecutor::isRunning())
        {
            LOG_INFO("Canceling Execution...");
            FlowExecutor::stop();
            LOG_INFO("Execution canceled");
        }
    }
}