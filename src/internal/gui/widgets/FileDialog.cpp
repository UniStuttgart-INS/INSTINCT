// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "FileDialog.hpp"

#include "internal/FlowManager.hpp"

#include "util/Logger.hpp"

#include "imgui.h"
#include "imgui_stdlib.h"
#include "ImGuiFileDialog.h"

#include <filesystem>

bool NAV::gui::widgets::FileDialogSave(std::string& path, const char* vName,
                                       const char* vFilters, const std::vector<std::string>& extensions,
                                       const std::filesystem::path& startPath,
                                       size_t id, [[maybe_unused]] const std::string& nameId,
                                       const char* buttonText)
{
    bool changed = false;
    // Filepath
    if (ImGui::InputText("Filepath", &path))
    {
        LOG_DEBUG("{}: Filepath changed to {}", nameId, path);
        changed = true;
    }
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("%s", path.c_str());
    }
    ImGui::SameLine();
    std::string saveFileDialogKey = fmt::format("Save File ({})", id);
    if (ImGui::Button(buttonText))
    {
        ImGuiFileDialog::Instance()->OpenDialog(saveFileDialogKey, vName, vFilters, (startPath / ".").string(), 1, nullptr, ImGuiFileDialogFlags_ConfirmOverwrite);
        for (const auto& ext : extensions)
        {
            ImGuiFileDialog::Instance()->SetFileStyle(IGFD_FileStyleByExtention, ext.c_str(), ImVec4(0.0F, 1.0F, 0.0F, 0.9F));
        }
    }

    if (ImGuiFileDialog::Instance()->Display(saveFileDialogKey, ImGuiWindowFlags_NoCollapse, ImVec2(600, 500)))
    {
        if (ImGuiFileDialog::Instance()->IsOk())
        {
            path = ImGuiFileDialog::Instance()->GetFilePathName();
            if (path.find(startPath.string()) != std::string::npos)
            {
                path = path.substr(startPath.string().size() + 1);
            }
            LOG_DEBUG("{}: Selected file: {}", nameId, path);
            changed = true;
        }

        ImGuiFileDialog::Instance()->Close();
    }

    return changed;
}

bool NAV::gui::widgets::FileDialogLoad(std::string& path, const char* vName,
                                       const char* vFilters, const std::vector<std::string>& extensions,
                                       const std::filesystem::path& startPath,
                                       size_t id, [[maybe_unused]] const std::string& nameId,
                                       const char* buttonText)
{
    bool changed = false;
    // Filepath
    if (ImGui::InputText("Filepath", &path))
    {
        LOG_DEBUG("{}: Filepath changed to {}", nameId, path);
        changed = true;
    }
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("%s", path.c_str());
    }
    ImGui::SameLine();
    std::string openFileDialogKey = fmt::format("Select File ({})", id);
    if (ImGui::Button(buttonText))
    {
        ImGuiFileDialog::Instance()->OpenDialog(openFileDialogKey, vName, vFilters, (startPath / ".").string());
        for (const auto& ext : extensions)
        {
            ImGuiFileDialog::Instance()->SetFileStyle(ext.starts_with('(') ? IGFD_FileStyleByFullName : IGFD_FileStyleByExtention, ext.c_str(), ImVec4(0.0F, 1.0F, 0.0F, 0.9F));
        }
    }

    if (ImGuiFileDialog::Instance()->Display(openFileDialogKey, ImGuiWindowFlags_NoCollapse, ImVec2(600, 500)))
    {
        if (ImGuiFileDialog::Instance()->IsOk())
        {
            if (!ImGuiFileDialog::Instance()->GetSelection().empty())
            {
                if (std::filesystem::exists(ImGuiFileDialog::Instance()->GetSelection().begin()->second))
                {
                    path = ImGuiFileDialog::Instance()->GetSelection().begin()->second;
                    if (path.find(startPath.string()) != std::string::npos)
                    {
                        path = path.substr(startPath.string().size() + 1);
                    }
                    LOG_DEBUG("{}: Selected file: {}", nameId, path);
                    changed = true;
                }
                else
                {
                    LOG_WARN("{}: Selected path does not exist: {}", nameId, ImGuiFileDialog::Instance()->GetSelection().begin()->second);
                }
            }
            else
            {
                LOG_WARN("{}: FileDialog is okay, but nothing selected", nameId);
            }
        }

        ImGuiFileDialog::Instance()->Close();
    }

    return changed;
}