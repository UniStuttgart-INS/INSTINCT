#include "ImuDataLogger.hpp"

#include "NodeData/IMU/ImuObs.hpp"

#include "util/Logger.hpp"

#include <iomanip> // std::setprecision

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

NAV::ImuDataLogger::ImuDataLogger()
    : Node(typeStatic())
{
    LOG_TRACE("{}: called", name);

    _fileType = FileType::CSV;

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateInputPin(this, "writeObservation", Pin::Type::Flow, { NAV::ImuObs::type() }, &ImuDataLogger::writeObservation);
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
    if (FileWriter::guiConfig(".csv", { ".csv" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        doDeinitialize();
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

void NAV::ImuDataLogger::flush()
{
    _filestream.flush();
}

bool NAV::ImuDataLogger::initialize()
{
    LOG_TRACE("{}: called", nameId());

    if (!FileWriter::initialize())
    {
        return false;
    }

    CommonLog::initialize();

    _filestream << "Time [s],GpsCycle,GpsWeek,GpsToW [s],TimeStartup [ns],"
                << "UnCompMagX [Gauss],UnCompMagY [Gauss],UnCompMagZ [Gauss],"
                << "UnCompAccX [m/s^2],UnCompAccY [m/s^2],UnCompAccZ [m/s^2],"
                << "UnCompGyroX [rad/s],UnCompGyroY [rad/s],UnCompGyroZ [rad/s],"
                << "Temperature [Celsius]" << std::endl;

    return true;
}

void NAV::ImuDataLogger::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileWriter::deinitialize();
}

void NAV::ImuDataLogger::writeObservation(NAV::InputPin::NodeDataQueue& queue, size_t /* pinIdx */)
{
    auto obs = std::static_pointer_cast<const ImuObs>(queue.extract_front());

    constexpr int gpsCyclePrecision = 3;
    constexpr int gpsTimePrecision = 12;
    constexpr int valuePrecision = 9;

    if (!obs->insTime.empty())
    {
        _filestream << std::setprecision(valuePrecision) << std::round(calcTimeIntoRun(obs->insTime) * 1e9) / 1e9;
    }
    _filestream << ",";
    if (!obs->insTime.empty())
    {
        _filestream << std::fixed << std::setprecision(gpsCyclePrecision) << obs->insTime.toGPSweekTow().gpsCycle;
    }
    _filestream << ",";
    if (!obs->insTime.empty())
    {
        _filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.toGPSweekTow().gpsWeek;
    }
    _filestream << ",";
    if (!obs->insTime.empty())
    {
        _filestream << std::defaultfloat << std::setprecision(gpsTimePrecision) << obs->insTime.toGPSweekTow().tow;
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
    _filestream << '\n';
}