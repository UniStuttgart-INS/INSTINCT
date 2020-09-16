#include "EmlidFile.hpp"

#include "util/Logger.hpp"
#include <ios>
#include <cmath>
#include <array>

#include "util/UartSensors/Emlid/EmlidUtilities.hpp"

NAV::EmlidFile::EmlidFile(const std::string& name, const std::map<std::string, std::string>& options)
    : GnssFileReader(name, options), sensor(name) {}

std::shared_ptr<NAV::EmlidObs> NAV::EmlidFile::pollData(bool peek)
{
    if (fileType == FileType::ASCII)
    {
        // TODO: Implement EmlidFile Ascii reading
        LOG_CRITICAL("Ascii EmlidFile pollData is not implemented yet.");
        return nullptr;
    }

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

    sensors::emlid::decryptEmlidObs(obs, currentInsTime, peek);

    if (peek)
    {
        // Return to position before "Read line".
        filestream.seekg(pos, std::ios_base::beg);
    }

    if (obs->insTime.has_value())
    {
        // Has time value, but value should not be displayed
        if (obs->insTime.value() < lowerLimit)
        {
            // Resetting the value will make the read loop skip the message
            obs->insTime.reset();
            return obs;
        }
        if (obs->insTime.value() > upperLimit)
        {
            return nullptr;
        }
    }

    // Calls all the callbacks
    if (!peek)
    {
        invokeCallbacks(obs);
    }

    return obs;
}

NAV::FileReader::FileType NAV::EmlidFile::determineFileType()
{
    LOG_TRACE("called for {}", name);

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
            LOG_DEBUG("{} has the file type: Binary", name);
            return FileType::BINARY;
        }
        filestream.close();

        LOG_CRITICAL("{} could not determine file type", name);
    }

    LOG_CRITICAL("{} could not open file {}", name, path);
    return FileType::NONE;
}