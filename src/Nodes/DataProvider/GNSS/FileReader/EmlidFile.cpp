#include "EmlidFile.hpp"

#include "util/Logger.hpp"

#include "internal/gui/widgets/FileDialog.hpp"

#include "internal/NodeManager.hpp"
namespace nm = NAV::NodeManager;
#include "internal/FlowManager.hpp"

#include "util/UartSensors/Emlid/EmlidUtilities.hpp"
#include "util/Time/TimeBase.hpp"

#include "NodeData/GNSS/EmlidObs.hpp"

NAV::EmlidFile::EmlidFile()
    : sensor(typeStatic())
{
    name = typeStatic();

    LOG_TRACE("{}: called", name);

    hasConfig = true;
    guiConfigDefaultWindowSize = { 380, 70 };

    nm::CreateOutputPin(this, "EmlidObs", Pin::Type::Flow, { NAV::EmlidObs::type() }, &EmlidFile::pollData);
}

NAV::EmlidFile::~EmlidFile()
{
    LOG_TRACE("{}: called", nameId());
}

std::string NAV::EmlidFile::typeStatic()
{
    return "EmlidFile";
}

std::string NAV::EmlidFile::type() const
{
    return typeStatic();
}

std::string NAV::EmlidFile::category()
{
    return "Data Provider";
}

void NAV::EmlidFile::guiConfig()
{
    if (gui::widgets::FileDialogLoad(path, "Select File", ".ubx", { ".ubx" }, size_t(id), nameId()))
    {
        flow::ApplyChanges();
        initializeNode();
    }
}

[[nodiscard]] json NAV::EmlidFile::save() const
{
    LOG_TRACE("{}: called", nameId());

    json j;

    j["FileReader"] = FileReader::save();

    return j;
}

void NAV::EmlidFile::restore(json const& j)
{
    LOG_TRACE("{}: called", nameId());

    if (j.contains("FileReader"))
    {
        FileReader::restore(j.at("FileReader"));
    }
}

bool NAV::EmlidFile::initialize()
{
    LOG_TRACE("{}: called", nameId());

    return FileReader::initialize();
}

void NAV::EmlidFile::deinitialize()
{
    LOG_TRACE("{}: called", nameId());

    FileReader::deinitialize();
}

bool NAV::EmlidFile::resetNode()
{
    FileReader::resetReader();

    return true;
}

std::shared_ptr<NAV::NodeData> NAV::EmlidFile::pollData(bool peek)
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

    auto obs = std::make_shared<EmlidObs>(*packet);

    // Check if package is empty
    if (obs->raw.getRawDataLength() == 0)
    {
        return nullptr;
    }

    sensors::emlid::decryptEmlidObs(obs, peek);

    if (!obs->insTime.has_value())
    {
        if (auto currentTime = util::time::GetCurrentInsTime();
            !currentTime.empty())
        {
            obs->insTime = currentTime;
        }
    }

    if (peek)
    {
        // Return to position before "Read line".
        filestream.seekg(pos, std::ios_base::beg);
    }

    // Calls all the callbacks
    if (!peek)
    {
        invokeCallbacks(OutputPortIndex_EmlidObs, obs);
    }

    return obs;
}

NAV::FileReader::FileType NAV::EmlidFile::determineFileType()
{
    LOG_TRACE("called for {}", nameId());

    auto filestream = std::ifstream(path);

    constexpr uint16_t BUFFER_SIZE = 10;

    std::array<char, BUFFER_SIZE> buffer{};
    if (filestream.good())
    {
        filestream.read(buffer.data(), BUFFER_SIZE);

        if ((static_cast<uint8_t>(buffer.at(0)) == sensors::emlid::EmlidUartSensor::BinarySyncChar1
             && static_cast<uint8_t>(buffer.at(1)) == sensors::emlid::EmlidUartSensor::BinarySyncChar2)
            || buffer.at(0) == sensors::emlid::EmlidUartSensor::AsciiStartChar)
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