#include "FileDialog.hpp"

#include "util/Logger.hpp"

#include "imgui.h"
#include "imgui_stdlib.h"
#include "ImGuiFileDialog.h"

#include <filesystem>

bool NAV::gui::widgets::FileDialogSave(std::string& path, const char* vName,
                                       const char* vFilters, const std::vector<std::string>& extensions,
                                       size_t id, const std::string& nameId)
{
    bool changed = false;
    // Filepath
    if (ImGui::InputText("Filepath", &path))
    {
        LOG_DEBUG("{}: Filepath changed to {}", nameId, path);
        changed = true;
    }
    ImGui::SameLine();
    std::string saveFileDialogKey = fmt::format("Save File ({})", id);
    if (ImGui::Button("Save"))
    {
        igfd::ImGuiFileDialog::Instance()->OpenDialog(saveFileDialogKey, vName, vFilters, "", 1, nullptr, ImGuiFileDialogFlags_ConfirmOverwrite);
        for (const auto& ext : extensions)
        {
            igfd::ImGuiFileDialog::Instance()->SetExtentionInfos(ext, ImVec4(0.0F, 1.0F, 0.0F, 0.9F));
        }
    }

    if (igfd::ImGuiFileDialog::Instance()->FileDialog(saveFileDialogKey, ImGuiWindowFlags_NoCollapse, ImVec2(600, 500)))
    {
        if (igfd::ImGuiFileDialog::Instance()->IsOk)
        {
            path = igfd::ImGuiFileDialog::Instance()->GetFilePathName();
            LOG_DEBUG("{}: Selected file: {}", nameId, path);
            changed = true;
        }

        igfd::ImGuiFileDialog::Instance()->CloseDialog();
    }

    return changed;
}

bool NAV::gui::widgets::FileDialogLoad(std::string& path, const char* vName,
                                       const char* vFilters, const std::vector<std::string>& extensions,
                                       size_t id, const std::string& nameId)
{
    bool changed = false;
    // Filepath
    if (ImGui::InputText("Filepath", &path))
    {
        LOG_DEBUG("{}: Filepath changed to {}", nameId, path);
        changed = true;
    }
    ImGui::SameLine();
    std::string openFileDialogKey = fmt::format("Select File ({})", id);
    if (ImGui::Button("Open"))
    {
        igfd::ImGuiFileDialog::Instance()->OpenDialog(openFileDialogKey, vName, vFilters, "");
        for (const auto& ext : extensions)
        {
            igfd::ImGuiFileDialog::Instance()->SetExtentionInfos(ext, ImVec4(0.0F, 1.0F, 0.0F, 0.9F));
        }
    }

    if (igfd::ImGuiFileDialog::Instance()->FileDialog(openFileDialogKey, ImGuiWindowFlags_NoCollapse, ImVec2(600, 500)))
    {
        if (igfd::ImGuiFileDialog::Instance()->IsOk)
        {
            if (std::filesystem::exists(igfd::ImGuiFileDialog::Instance()->GetFilePathName()))
            {
                path = igfd::ImGuiFileDialog::Instance()->GetFilePathName();
                LOG_DEBUG("{}: Selected file: {}", nameId, path);
                changed = true;
            }
            else
            {
                LOG_WARN("{}: Selected path does not exist: {}", nameId, igfd::ImGuiFileDialog::Instance()->GetFilePathName());
            }
        }

        igfd::ImGuiFileDialog::Instance()->CloseDialog();
    }

    return changed;
}