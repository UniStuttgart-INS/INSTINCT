#include "UbloxDataLogger.hpp"

#include "NodeData/GNSS/UbloxObs.hpp"

#include "util/Logger.hpp"

#include "imgui_stdlib.h"
#include "ImGuiFileDialog.h"

#include <iomanip> // std::setprecision

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::UbloxDataLogger::UbloxDataLogger()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    fileType = FileType::BINARY;

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateOutputPin(this, "", Pin::Type::Delegate, "UbloxDataLogger", this);

    nm::CreateInputPin(this, "writeObservation", Pin::Type::Flow, NAV::UbloxObs::type(), &UbloxDataLogger::writeObservation);
}

NAV::UbloxDataLogger::~UbloxDataLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::UbloxDataLogger::typeStatic()
{
    return "UbloxDataLogger";
}

std::string NAV::UbloxDataLogger::type() const
{
    return typeStatic();
}

std::string NAV::UbloxDataLogger::category()
{
    return "Data Logger";
}

void NAV::UbloxDataLogger::guiConfig()
{
    // Filepath
    if (ImGui::InputText("Filepath", &path))
    {
        LOG_DEBUG("{}: Filepath changed to {}", nameId(), path);
        flow::ApplyChanges();
        deinitialize();
    }
    ImGui::SameLine();
    std::string saveFileDialogKey = fmt::format("Save File ({})", id.AsPointer());
    if (ImGui::Button("Save"))
    {
        igfd::ImGuiFileDialog::Instance()->OpenDialog(saveFileDialogKey, "Save File", ".ubx", "");
        igfd::ImGuiFileDialog::Instance()->SetExtentionInfos(".ubx", ImVec4(0.0F, 1.0F, 0.0F, 0.9F));
    }

    if (igfd::ImGuiFileDialog::Instance()->FileDialog(saveFileDialogKey, ImGuiWindowFlags_NoCollapse, ImVec2(600, 500)))
    {
        if (igfd::ImGuiFileDialog::Instance()->IsOk)
        {
            path = igfd::ImGuiFileDialog::Instance()->GetFilePathName();
            LOG_DEBUG("{}: Selected file: {}", nameId(), path);
            flow::ApplyChanges();
            initialize();
        }

        igfd::ImGuiFileDialog::Instance()->CloseDialog(saveFileDialogKey);
    }
}

[[nodiscard]] json NAV::UbloxDataLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileWriter"] = FileWriter::save();

    return j;
}

void NAV::UbloxDataLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileWriter"))
    {
        FileWriter::restore(j.at("FileWriter"));
    }
}

bool NAV::UbloxDataLogger::initialize()
{
    deinitialize();

    LOG_TRACE("{}: called", nameId());

    if (!Node::initialize()
        || !FileWriter::initialize())
    {
        return false;
    }

    return isInitialized = true;
}

void NAV::UbloxDataLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
    Node::deinitialize();
}

void NAV::UbloxDataLogger::writeObservation(std::shared_ptr<NodeData> nodeData)
{
    auto obs = std::static_pointer_cast<UbloxObs>(nodeData);

    if (obs->raw.getRawDataLength() > 0)
    {
        filestream.write(reinterpret_cast<const char*>(obs->raw.getRawData().data()), static_cast<std::streamsize>(obs->raw.getRawDataLength()));
    }
    else
    {
        LOG_ERROR("{}: Tried to write binary, but observation had no binary data.", nameId());
    }
}