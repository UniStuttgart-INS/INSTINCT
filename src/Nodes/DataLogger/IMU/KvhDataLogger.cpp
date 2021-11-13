#include "KvhDataLogger.hpp"

#include "NodeData/IMU/KvhObs.hpp"

#include "util/Logger.hpp"

#include "internal/gui/widgets/FileDialog.hpp"

#include <iomanip> // std::setprecision

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::KvhDataLogger::KvhDataLogger()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    fileType = FileType::CSV;

    hasConfig = true;
    guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateInputPin(this, "writeObservation", Pin::Type::Flow, { NAV::KvhObs::type() }, &KvhDataLogger::writeObservation);
}

NAV::KvhDataLogger::~KvhDataLogger()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::KvhDataLogger::typeStatic()
{
    return "KvhDataLogger";
}

std::string NAV::KvhDataLogger::type() const
{
    return typeStatic();
}

std::string NAV::KvhDataLogger::category()
{
    return "Data Logger";
}

void NAV::KvhDataLogger::guiConfig()
{
    if (gui::widgets::FileDialogSave(path, "Save File", ".csv", { ".csv" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        deinitializeNode();
    }
}

[[nodiscard]] json NAV::KvhDataLogger::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileWriter"] = FileWriter::save();

    return j;
}

void NAV::KvhDataLogger::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileWriter"))
    {
        FileWriter::restore(j.at("FileWriter"));
    }
}

void NAV::KvhDataLogger::flush()
{
    filestream.flush();
}

bool NAV::KvhDataLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!FileWriter::initialize())
    {
        return false;
    }

    filestream << "GpsCycle,GpsWeek,GpsToW,TimeStartup,"
               << "UnCompMagX,UnCompMagY,UnCompMagZ,UnCompAccX,UnCompAccY,UnCompAccZ,UnCompGyroX,UnCompGyroY,UnCompGyroZ,"
               << "Temperature,Status,SequenceNumber" << std::endl;

    return true;
}

void NAV::KvhDataLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}

void NAV::KvhDataLogger::writeObservation(const std::shared_ptr<const NodeData>& nodeData, ax::NodeEditor::LinkId /*linkId*/)
{
    auto obs = std::static_pointer_cast<const KvhObs>(nodeData);

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
    filestream << ",";
    filestream << obs->status;
    filestream << ",";
    filestream << static_cast<uint16_t>(obs->sequenceNumber);
    filestream << '\n';
}