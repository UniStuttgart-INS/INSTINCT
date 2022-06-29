#include "MultiImuFile.hpp"

#include "util/Logger.hpp"

#include "Navigation/Transformations/CoordinateFrames.hpp"
#include "Navigation/Transformations/Units.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/ImuObs.hpp"

NAV::MultiImuFile::MultiImuFile()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 618, 261 };

    updateNumberOfOutputPins();
}

NAV::MultiImuFile::~MultiImuFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::MultiImuFile::typeStatic()
{
    return "MultiImuFile";
}

std::string NAV::MultiImuFile::type() const
{
    return typeStatic();
}

std::string NAV::MultiImuFile::category()
{
    return "Data Provider";
}

void NAV::MultiImuFile::guiConfig()
{
    if (FileReader::guiConfig(".txt", { ".txt" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        deinitializeNode();
    }

    Imu::guiConfig();
}

[[nodiscard]] json NAV::MultiImuFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();
    j["Imu"] = Imu::save();

    return j;
}

void NAV::MultiImuFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
    if (j.contains("Imu"))
    {
        Imu::restore(j.at("Imu"));
    }
}

bool NAV::MultiImuFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return FileReader::initialize();
}

void NAV::MultiImuFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool NAV::MultiImuFile::resetNode()
{
    FileReader::resetReader();

    return true;
}

void NAV::MultiImuFile::updateNumberOfOutputPins()
{
    // nm::CreateOutputPin(this, "ImuObs 1", Pin::Type::Flow, { NAV::ImuObs::type() }, &MultiImuFile::pollData);
    while (outputPins.size() < _nSensors)
    {
        nm::CreateOutputPin(this, fmt::format("ImuObs {}", outputPins.size() + 1).c_str(), Pin::Type::Flow, { NAV::ImuObs::type() }, &MultiImuFile::pollData);
    }
}

NAV::FileReader::FileType NAV::MultiImuFile::determineFileType()
{
    LOG_TRACE("called");

    auto filepath = getFilepath();

    if (_filestream.good())
    {
        return FileType::CSV;
    }

    LOG_ERROR("Could not open file {}", filepath.string());
    return FileType::NONE;
}

void NAV::MultiImuFile::readHeader()
{
    LOG_TRACE("called");

    bool gpggaFound = false;
    std::string line;
    const char* gpgga = "GPGGA";

    // Find first line of data
    while (std::getline(_filestream, line))
    {
        // Remove any trailing non text characters
        line.erase(std::find_if(line.begin(), line.end(), [](int ch) { return std::iscntrl(ch); }), line.end());

        if (line.find(gpgga) != std::string::npos)
        {
            gpggaFound = true;
            continue;
        }
        if ((std::find_if(line.begin(), line.begin() + 1, [](int ch) { return std::isdigit(ch); }) != (std::begin(line) + 1)) && gpggaFound)
        {
            LOG_DEBUG("{}: Found first line of data: {}", nameId(), line);
            break;
        }
    }
}

std::shared_ptr<const NAV::NodeData> NAV::MultiImuFile::pollData(bool peek)
{
    auto obs = std::make_shared<ImuObs>(_imuPos);

    // Read line
    std::string line;
    // Get current position
    auto len = _filestream.tellg();
    std::getline(_filestream, line);
    if (peek)
    {
        // Return to position before "Read line".
        _filestream.seekg(len, std::ios_base::beg);
    }
    // Remove any starting non text characters
    line.erase(line.begin(), std::find_if(line.begin(), line.end(), [](int ch) { return std::isgraph(ch); }));

    if (line.empty())
    {
        return nullptr;
    }

    // Convert line into stream
    std::stringstream lineStream(line);
    std::string cell;

    size_t sensorId{};
    double gpsSecond{};
    double timeNumerator{};
    double timeDenominator{};
    std::optional<double> accelX;
    std::optional<double> accelY;
    std::optional<double> accelZ;
    std::optional<double> gyroX;
    std::optional<double> gyroY;
    std::optional<double> gyroZ;

    // Split line at comma
    for (const auto& col : _columns)
    {
        if (std::getline(lineStream, cell, ' '))
        {
            // Remove any trailing non text characters
            cell.erase(std::find_if(cell.begin(), cell.end(), [](int ch) { return std::iscntrl(ch); }), cell.end());
            while (cell.empty())
            {
                std::getline(lineStream, cell, ' ');
            }

            if (col == "sensorId")
            {
                sensorId = std::stoul(cell); //NOLINT(clang-diagnostic-implicit-int-conversion)
            }
            if (col == "gpsSecond")
            {
                gpsSecond = std::stod(cell); // [s]
                // Start time axis from first timestamp onwards
                if (_startTime == 0)
                {
                    _startTime = gpsSecond;
                }
            }
            if (col == "timeNumerator")
            {
                timeNumerator = std::stod(cell);
            }
            if (col == "timeDenominator")
            {
                timeDenominator = std::stod(cell);
            }
            if (col == "accelX")
            {
                accelX = 0.001 * std::stod(cell); // [m/s²]
            }
            if (col == "accelY")
            {
                accelY = 0.001 * std::stod(cell); // [m/s²]
            }
            if (col == "accelZ")
            {
                accelZ = 0.001 * std::stod(cell); // [m/s²]
            }
            if (col == "gyroX")
            {
                gyroX = deg2rad(std::stod(cell) / 131); // [deg/s]
            }
            if (col == "gyroY")
            {
                gyroY = deg2rad(std::stod(cell)) / 131; // [deg/s]
            }
            if (col == "gyroZ")
            {
                gyroZ = deg2rad(std::stod(cell)) / 131; // [deg/s]
            }
        }
    }

    auto timeStamp = gpsSecond + timeNumerator / timeDenominator - _startTime;

    obs->insTime.emplace(0, 0, 0, 0, 0, timeStamp);

    if (accelX.has_value() && accelY.has_value() && accelZ.has_value())
    {
        obs->accelUncompXYZ.emplace(accelX.value(), accelY.value(), accelZ.value());
    }
    if (gyroX.has_value() && gyroY.has_value() && gyroZ.has_value())
    {
        obs->gyroUncompXYZ.emplace(gyroX.value(), gyroY.value(), gyroZ.value());
    }

    // Calls all the callbacks
    if (!peek)
    {
        invokeCallbacks(sensorId - 1, obs);
    }

    return obs;
}