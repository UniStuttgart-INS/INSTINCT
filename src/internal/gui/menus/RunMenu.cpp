#include "RunMenu.hpp"

#include <imgui.h>

#include "internal/FlowExecutor.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "util/Logger.hpp"

void NAV::gui::menus::ShowRunMenu(std::deque<std::pair<Node*, bool>>& initList)
{
    bool hasInitializedNodes = false;
    bool allNodesInitialized = true;
    for (const auto* node : nm::m_Nodes())
    {
        if (node->getState() == Node::State::Initialized
            || node->getState() == Node::State::Initializing
            || node->getState() == Node::State::InitializationPlanned)
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
        for (auto* node : nm::m_Nodes())
        {
            if (node->isEnabled() && node->getState() != Node::State::Initialized)
            {
                node->_state = Node::State::InitializationPlanned;
                initList.emplace_back(node, true);
            }
        }
    }
    if (ImGui::MenuItem("Reinitialize all Nodes", nullptr, false, hasInitializedNodes))
    {
        for (auto* node : nm::m_Nodes())
        {
            if (node->isEnabled() && node->getState() == Node::State::Initialized)
            {
                node->_state = Node::State::DeinitializationPlanned;
                initList.emplace_back(node, false);
            }
        }
        for (auto* node : nm::m_Nodes())
        {
            if (node->isEnabled())
            {
                initList.emplace_back(node, true);
            }
        }
    }
    if (ImGui::MenuItem("Deinitialize all Nodes", nullptr, false, hasInitializedNodes))
    {
        for (auto* node : nm::m_Nodes())
        {
            if (node->isEnabled() && node->getState() == Node::State::Initialized)
            {
                node->_state = Node::State::DeinitializationPlanned;
                initList.emplace_back(node, false);
            }
        }
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