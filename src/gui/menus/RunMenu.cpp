#include "RunMenu.hpp"

#include <imgui.h>

#include "internal/FlowExecutor.hpp"

#include "util/Logger.hpp"

void NAV::gui::menus::ShowRunMenu()
{
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