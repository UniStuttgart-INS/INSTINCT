#include "ImuDataLogger.hpp"

#include "NodeData/IMU/ImuObs.hpp"

#include "util/Logger.hpp"

#include "ImGuiFileDialog.h"

#include <iomanip> // std::setprecision

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::ImuDataLogger::ImuDataLogger()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    fileType = FileType::ASCII;

    color = ImColor(255, 128, 128);
    hasConfig = true;

    nm::CreateOutputPin(this, "", Pin::Type::Delegate, "ImuDataLogger", this);

    nm::CreateInputPin(this, "writeObservation", Pin::Type::Flow, NAV::ImuObs::type(), &ImuDataLogger::writeObservation);
}

NAV::ImuDataLogger::~ImuDataLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::ImuDataLogger::typeStatic()
{
    return "ImuDataLogger";
}

std::string NAV::ImuDataLogger::type() const
{
    return typeStatic();
}

std::string NAV::ImuDataLogger::category()
{
    return "Data Logger";
}

void NAV::ImuDataLogger::guiConfig()
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
        igfd::ImGuiFileDialog::Instance()->OpenDialog(saveFileDialogKey, "Save File", ".csv", "", 1, nullptr, ImGuiFileDialogFlags_ConfirmOverwrite);
        igfd::ImGuiFileDialog::Instance()->SetExtentionInfos(".csv", ImVec4(0.0F, 1.0F, 0.0F, 0.9F));
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

        igfd::ImGuiFileDialog::Instance()->CloseDialog();
    }
}

[[nodiscard]] json NAV::ImuDataLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileWriter"] = FileWriter::save();

    return j;
}

void NAV::ImuDataLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileWriter"))
    {
        FileWriter::restore(j.at("FileWriter"));
    }
}

bool NAV::ImuDataLogger::initialize()
{
    deinitialize();

    LOG_TRACE("{}: called", nameId());

    if (!Node::initialize()
        || !FileWriter::initialize())
    {
        return false;
    }

    filestream << "GpsCycle,GpsWeek,GpsToW,TimeStartup,"
               << "UnCompMagX,UnCompMagY,UnCompMagZ,UnCompAccX,UnCompAccY,UnCompAccZ,UnCompGyroX,UnCompGyroY,UnCompGyroZ,"
               << "Temperature" << std::endl;

    return isInitialized = true;
}

void NAV::ImuDataLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
    Node::deinitialize();
}

void NAV::ImuDataLogger::writeObservation(const std::shared_ptr<NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<ImuObs>(nodeData);

    constexpr int gpsCyclePrecision = 3;
    constexpr int gpsTimePrecision = 12;
    constexpr int valuePrecision = 9;

    if (obs->insTime.has_value())
    {
        filestream << std::fixed << std::setprecision(gpsCyclePrecision) << obs->insTime.value().toGPSweekTow().gpsCycle;
    }
    filestream << ",";
    if (obs->insTime.has_value())
    {
        filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.value().toGPSweekTow().gpsWeek;
    }
    filestream << ",";
    if (obs->insTime.has_value())
    {
        filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.value().toGPSweekTow().tow;
    }
    filestream << ",";
    if (obs->timeSinceStartup.has_value())
    {
        filestream << std::setprecision(valuePrecision) << obs->timeSinceStartup.value();
    }
    filestream << ",";
    if (obs->magUncompXYZ.has_value())
    {
        filestream << obs->magUncompXYZ.value().x();
    }
    filestream << ",";
    if (obs->magUncompXYZ.has_value())
    {
        filestream << obs->magUncompXYZ.value().y();
    }
    filestream << ",";
    if (obs->magUncompXYZ.has_value())
    {
        filestream << obs->magUncompXYZ.value().z();
    }
    filestream << ",";
    if (obs->accelUncompXYZ.has_value())
    {
        filestream << obs->accelUncompXYZ.value().x();
    }
    filestream << ",";
    if (obs->accelUncompXYZ.has_value())
    {
        filestream << obs->accelUncompXYZ.value().y();
    }
    filestream << ",";
    if (obs->accelUncompXYZ.has_value())
    {
        filestream << obs->accelUncompXYZ.value().z();
    }
    filestream << ",";
    if (obs->gyroUncompXYZ.has_value())
    {
        filestream << obs->gyroUncompXYZ.value().x();
    }
    filestream << ",";
    if (obs->gyroUncompXYZ.has_value())
    {
        filestream << obs->gyroUncompXYZ.value().y();
    }
    filestream << ",";
    if (obs->gyroUncompXYZ.has_value())
    {
        filestream << obs->gyroUncompXYZ.value().z();
    }
    filestream << ",";
    if (obs->temperature.has_value())
    {
        filestream << obs->temperature.value();
    }
    filestream << '\n';
}