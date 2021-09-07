#include "UbloxFile.hpp"

#include "util/Logger.hpp"

#include "internal/gui/widgets/FileDialog.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "util/UartSensors/Ublox/UbloxUtilities.hpp"
#include "util/Time/TimeBase.hpp"

#include "NodeData/GNSS/UbloxObs.hpp"

NAV::UbloxFile::UbloxFile()
    : sensor(typeStatic())
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateOutputPin(this, "UbloxObs", Pin::Type::Flow, { NAV::UbloxObs::type() }, &UbloxFile::pollData);
}

NAV::UbloxFile::~UbloxFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::UbloxFile::typeStatic()
{
    return "UbloxFile";
}

std::string NAV::UbloxFile::type() const
{
    return typeStatic();
}

std::string NAV::UbloxFile::category()
{
    return "Data Provider";
}

void NAV::UbloxFile::guiConfig()
{
    if (gui::widgets::FileDialogLoad(path, "Select File", ".ubx", { ".ubx" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        initializeNode();
    }
}

[[nodiscard]] json NAV::UbloxFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();

    return j;
}

void NAV::UbloxFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
}

bool NAV::UbloxFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return FileReader::initialize();
}

void NAV::UbloxFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool NAV::UbloxFile::resetNode()
{
    FileReader::resetReader();

    return true;
}

std::shared_ptr<NAV::NodeData> NAV::UbloxFile::pollData(bool peek)
{
    // Get current position
    auto pos = filestream.tellg();
    uint8_t i = 0;
    std::unique_ptr<uart::protocol::Packet> packet = nullptr;
    while (filestream.readsome(reinterpret_cast<char*>(&i), 1))
    {
        packet = sensor.findPacket(i);

        if (packet != nullptr)
        {
            break;
        }
    }

    if (!packet)
    {
        return nullptr;
    }

    auto obs = std::make_shared<UbloxObs>(*packet);

    // Check if package is empty
    if (obs->raw.getRawDataLength() == 0)
    {
        return nullptr;
    }

    sensors::ublox::decryptUbloxObs(obs, peek);

    if (obs->insTime.has_value())
    {
        if (util::time::GetMode() == util::time::Mode::REAL_TIME)
        {
            util::time::SetCurrentTime(obs->insTime.value());
        }
    }
    else if (auto currentTime = util::time::GetCurrentInsTime();
             !currentTime.empty())
    {
        obs->insTime = currentTime;
    }

    if (peek)
    {
        // Return to position before "Read line".
        filestream.seekg(pos, std::ios_base::beg);
    }

    // Calls all the callbacks
    if (!peek)
    {
        invokeCallbacks(OutputPortIndex_UbloxObs, obs);
    }

    return obs;
}

NAV::FileReader::FileType NAV::UbloxFile::determineFileType()
{
    LOG_TRACE("called for {}", nameId());

    auto filestream = std::ifstream(path);

    constexpr uint16_t BUFFER_SIZE = 10;

    std::array<char, BUFFER_SIZE> buffer{};
    if (filestream.good())
    {
        filestream.read(buffer.data(), BUFFER_SIZE);

        if ((static_cast<uint8_t>(buffer.at(0)) == sensors::ublox::UbloxUartSensor::BinarySyncChar1
             && static_cast<uint8_t>(buffer.at(1)) == sensors::ublox::UbloxUartSensor::BinarySyncChar2)
            || buffer.at(0) == sensors::ublox::UbloxUartSensor::AsciiStartChar)
        {
            filestream.close();
            LOG_DEBUG("{} has the file type: Binary", nameId());
            return FileType::BINARY;
        }
        filestream.close();

        LOG_ERROR("{} could not determine file type", nameId());
        return FileType::NONE;
    }

    LOG_ERROR("{} could not open file {}", nameId(), path);
    return FileType::NONE;
}