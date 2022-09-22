// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "RunMenu.hpp"

#include <imgui.h>

#include "internal/FlowExecutor.hpp"
#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;

#include "util/Logger.hpp"

void NAV::gui::menus::ShowRunMenu()
{
    bool hasInitializedNodes = false;
    bool hasDeInitializedNodes = false;
    for (const auto* node : nm::m_Nodes())
    {
        if (node->getState() == Node::State::Initialized)
        {
            hasInitializedNodes = true;
        }
        else if (node->getState() == Node::State::Deinitialized)
        {
            hasDeInitializedNodes = true;
        }
    }
    if (ImGui::MenuItem("Initialize all Nodes", nullptr, false, hasDeInitializedNodes))
    {
        for (auto* node : nm::m_Nodes())
        {
            if (node->getState() == Node::State::Deinitialized)
            {
                node->doInitialize();
            }
        }
    }
    if (ImGui::MenuItem("Reinitialize all Nodes", nullptr, false, hasInitializedNodes))
    {
        for (auto* node : nm::m_Nodes())
        {
            if (node->isInitialized())
            {
                node->doReinitialize();
            }
            else if (node->getState() == Node::State::Deinitialized)
            {
                node->doInitialize();
            }
        }
    }
    if (ImGui::MenuItem("Deinitialize all Nodes", nullptr, false, hasInitializedNodes))
    {
        for (auto* node : nm::m_Nodes())
        {
            if (node->isInitialized())
            {
                node->doDeinitialize();
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