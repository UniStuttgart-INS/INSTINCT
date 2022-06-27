#include "MultiImuFile.hpp"

#include "util/Logger.hpp"

#include "Navigation/Transformations/CoordinateFrames.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "NodeData/IMU/ImuObs.hpp"

NAV::MultiImuFile::MultiImuFile()
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    _hasConfig = true;
    _guiConfigDefaultWindowSize = { 377, 201 }; // TODO: adapt size

    nm::CreateOutputPin(this, "ImuObs", Pin::Type::Flow, { NAV::MultiImuFile::type() }, &MultiImuFile::pollData);
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
{}

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

    return obs;
}