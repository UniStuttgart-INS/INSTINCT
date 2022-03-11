#include "KvhDataLogger.hpp"

#include "NodeData/IMU/KvhObs.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::KvhDataLogger::KvhDataLogger()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _fileType = FileType::CSV;

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 70 };

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
    if (FileWriter::guiConfig(".csv", { ".csv" }, size_t(id), nameId()))
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
    _filestream.flush();
}

bool NAV::KvhDataLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!FileWriter::initialize())
    {
        return false;
    }

    _filestream << "GpsCycle,GpsWeek,GpsToW,TimeStartup,"
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
        _filestream << std::fixed << std::setprecision(gpsCyclePrecision) << obs->insTime.value().toGPSweekTow().gpsCycle;
    }
    _filestream << ",";
    if (obs->insTime.has_value())
    {
        _filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.value().toGPSweekTow().gpsWeek;
    }
    _filestream << ",";
    if (obs->insTime.has_value())
    {
        _filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.value().toGPSweekTow().tow;
    }
    _filestream << ",";
    if (obs->timeSinceStartup.has_value())
    {
        _filestream << std::setprecision(valuePrecision) << obs->timeSinceStartup.value();
    }
    _filestream << ",";
    if (obs->magUncompXYZ.has_value())
    {
        _filestream << obs->magUncompXYZ.value().x();
    }
    _filestream << ",";
    if (obs->magUncompXYZ.has_value())
    {
        _filestream << obs->magUncompXYZ.value().y();
    }
    _filestream << ",";
    if (obs->magUncompXYZ.has_value())
    {
        _filestream << obs->magUncompXYZ.value().z();
    }
    _filestream << ",";
    if (obs->accelUncompXYZ.has_value())
    {
        _filestream << obs->accelUncompXYZ.value().x();
    }
    _filestream << ",";
    if (obs->accelUncompXYZ.has_value())
    {
        _filestream << obs->accelUncompXYZ.value().y();
    }
    _filestream << ",";
    if (obs->accelUncompXYZ.has_value())
    {
        _filestream << obs->accelUncompXYZ.value().z();
    }
    _filestream << ",";
    if (obs->gyroUncompXYZ.has_value())
    {
        _filestream << obs->gyroUncompXYZ.value().x();
    }
    _filestream << ",";
    if (obs->gyroUncompXYZ.has_value())
    {
        _filestream << obs->gyroUncompXYZ.value().y();
    }
    _filestream << ",";
    if (obs->gyroUncompXYZ.has_value())
    {
        _filestream << obs->gyroUncompXYZ.value().z();
    }
    _filestream << ",";
    if (obs->temperature.has_value())
    {
        _filestream << obs->temperature.value();
    }
    _filestream << ",";
    _filestream << obs->status;
    _filestream << ",";
    _filestream << static_cast<uint16_t>(obs->sequenceNumber);
    _filestream << '\n';
}